
import numpy as np, matplotlib.pyplot as plt, json, math, os
from pathlib import Path
from PIL import Image

OUT = Path("outputs"); FR = OUT/"frames"
OUT.mkdir(exist_ok=True, parents=True); FR.mkdir(exist_ok=True, parents=True)

def wrap_angle(a):
    return (a + np.pi)%(2*np.pi) - np.pi

class EKF2D:
    """ State: [x, y, vx, vy]
        IMU: ax (body), ay (body); Vision: bearing angle (camera forward=x axis)
    """
    def __init__(self):
        self.x = np.array([ -3.0, 0.5, 0.8, 0.0 ])
        self.P = np.eye(4)*0.5
        self.R = np.diag([ (2*np.pi/180)**2 ])  # bearing noise
        self.Q = np.diag([0,0, 1e-3, 1e-3])    # process on velocity
    def predict(self, a_world, dt):
        F = np.array([[1,0,dt,0],
                      [0,1,0,dt],
                      [0,0,1,0],
                      [0,0,0,1]])
        B = np.array([[0.5*dt*dt, 0],
                      [0, 0.5*dt*dt],
                      [dt,0],
                      [0,dt]])
        self.x = F@self.x + B@a_world
        self.P = F@self.P@F.T + self.Q
    def update_bearing(self, theta_meas):
        x,y, vx,vy = self.x
        theta_pred = np.arctan2(y, x+1e-9)
        H = np.array([[( -y)/((x+1e-9)**2 + y*y), ( x)/((x+1e-9)**2 + y*y), 0, 0]])
        yk = wrap_angle(theta_meas - theta_pred)
        S = H@self.P@H.T + self.R
        K = self.P@H.T@np.linalg.inv(S)
        self.x = self.x + (K*yk).reshape(-1)
        self.P = (np.eye(4) - K@H)@self.P

def simulate():
    dt = 0.02; steps = 600
    # Truth
    x = np.array([-4.0, 0.5, 1.0, 0.0])  # [x,y,vx,vy] target at origin
    # Target rotation for "visual" (for display only)
    target_angle = 0.0; target_w = 4*np.pi/180

    ekf = EKF2D()
    traj_t = []; traj_e = []; image_err = []
    contact = False; contact_k = None
    Kp_bear = 0.8; V_forward = 0.6  # guidance gains
    imp_K = 50.0; imp_B = 8.0       # impedance params
    peak_force = 0.0

    for k in range(steps):
        t = k*dt
        # --- Vision measurement (bearing) ---
        theta = np.arctan2(x[1], x[0]+1e-9) + np.random.randn()*np.deg2rad(0.8)
        # --- IMU (world accel known here for demo) ---
        a_cmd_world = np.zeros(2)

        # Guidance (no contact): drive bearing -> 0 (center image); keep forward speed
        if not contact:
            # Lateral accel to reduce bearing
            a_lat = -Kp_bear * theta
            # Forward accel to maintain approach speed
            v_forward = (x[0]*x[2] + x[1]*x[3]) / (np.linalg.norm(x[:2])+1e-6)
            a_fwd = 0.5*(V_forward - v_forward)
            # Compose in world frame along LOS unit vectors
            r = x[:2]; r_n = r/(np.linalg.norm(r)+1e-9)
            t_hat = r_n
            n_hat = np.array([-r_n[1], r_n[0]])
            a_cmd_world = a_fwd*t_hat + a_lat*n_hat
        # Contact detection
        if np.linalg.norm(x[:2]) < 0.15 and not contact:
            contact = True; contact_k = k

        # Impedance control upon contact (desired dock at [0.05,0])
        if contact:
            x_d = np.array([0.05, 0.0])
            v_d = np.zeros(2)
            pos_err = x[:2] - x_d
            vel_err = x[2:] - v_d
            # contact force (virtual)
            F = -imp_K*pos_err - imp_B*vel_err
            peak_force = max(peak_force, float(np.linalg.norm(F)))
            a_cmd_world = F  # unit mass

        # --- Truth propagation ---
        x = np.array([
            x[0] + x[2]*dt + 0.5*a_cmd_world[0]*dt*dt,
            x[1] + x[3]*dt + 0.5*a_cmd_world[1]*dt*dt,
            x[2] + a_cmd_world[0]*dt,
            x[3] + a_cmd_world[1]*dt
        ])

        target_angle += target_w*dt

        # --- EKF ---
        ekf.predict(a_cmd_world, dt)
        ekf.update_bearing(theta)

        traj_t.append(x.copy())
        traj_e.append(ekf.x.copy())
        image_err.append(float(theta))

        # --- Frame for animation ---
        if k % 4 == 0:
            fig, ax = plt.subplots(figsize=(4,4))
            ax.plot(0,0,'ks',ms=8,label="Target")
            ax.add_patch(plt.Circle((0,0),0.15, fill=False, linestyle="--"))
            ax.plot(x[0], x[1], 'ro', label="Chaser (truth)")
            ax.plot(ekf.x[0], ekf.x[1], 'bx', label="Estimate")
            # heading line
            ax.plot([x[0], x[0]+0.4*np.cos(theta)], [x[1], x[1]+0.4*np.sin(theta)], 'r-')
            # target orientation
            ax.plot([0, 0.3*np.cos(target_angle)], [0, 0.3*np.sin(target_angle)], 'k-')
            ax.set_aspect('equal', 'box')
            ax.set_xlim(-1.2, 0.8); ax.set_ylim(-0.8, 0.8)
            ax.grid(True); ax.legend(loc="upper right", fontsize=7)
            ax.set_title("OOS Approach (k={})".format(k))
            fig.tight_layout()
            fig.savefig(FR/f"frame_{k:04d}.png", dpi=120)
            plt.close(fig)

    traj_t = np.array(traj_t); traj_e = np.array(traj_e)
    t = np.arange(steps)*dt

    # Plots
    fig, ax = plt.subplots(figsize=(6,3.8))
    ax.plot(traj_t[:,0], traj_t[:,1], label="Truth")
    ax.plot(traj_e[:,0], traj_e[:,1], label="EKF est.")
    ax.plot(0,0,'ks',ms=6); ax.add_patch(plt.Circle((0,0),0.15, fill=False, linestyle="--"))
    ax.set_aspect('equal', 'box'); ax.grid(True)
    ax.set_title("Relative Trajectory"); ax.legend()
    fig.tight_layout(); fig.savefig(OUT/"traj.png", dpi=170); plt.close(fig)

    fig, ax = plt.subplots(figsize=(6,3.0))
    ax.plot(t, np.rad2deg(np.abs(image_err)))
    ax.set_xlabel("Time (s)"); ax.set_ylabel("|Image bearing| (deg)")
    ax.set_title("Image-plane Error"); ax.grid(True)
    fig.tight_layout(); fig.savefig(OUT/"image_error.png", dpi=170); plt.close(fig)

    # Force profile: recompute from saved states with impedance phase
    force = []
    contact_started = False
    x_d = np.array([0.05,0.0])
    for k in range(steps):
        xx = traj_t[k,:]
        if not contact_started and np.linalg.norm(xx[:2]) < 0.15:
            contact_started = True
        if contact_started:
            pos_err = xx[:2] - x_d
            vel_err = xx[2:]
            F = -imp_K*pos_err - imp_B*vel_err
            force.append(np.linalg.norm(F))
        else:
            force.append(0.0)
    fig, ax = plt.subplots(figsize=(6,3.0))
    ax.plot(t, force)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Contact force (arb. units)")
    ax.set_title("Impedance Contact Force"); ax.grid(True)
    fig.tight_layout(); fig.savefig(OUT/"force_profile.png", dpi=170); plt.close(fig)

    # Build GIF
    frames = sorted(FR.glob("frame_*.png"))
    imgs = [Image.open(p) for p in frames[:150]]  # limit length
    if imgs:
        imgs[0].save(OUT/"demo.gif", save_all=True, append_images=imgs[1:], duration=80, loop=0)

    # Benchmarks
    capture_time = None
    for k in range(steps):
        if np.linalg.norm(traj_t[k,:2]) < 0.15:
            capture_time = t[k]; break
    metrics = {
        "time_to_capture_s": float(capture_time) if capture_time is not None else None,
        "peak_force": float(np.max(force)),
        "final_position_error": float(np.linalg.norm(traj_t[-1,:2] - x_d)),
        "final_velocity_norm": float(np.linalg.norm(traj_t[-1,2:])),
    }
    (OUT/"benchmarks.json").write_text(json.dumps(metrics, indent=2))

if __name__ == "__main__":
    simulate()
