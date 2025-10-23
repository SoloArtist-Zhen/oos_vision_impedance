# OOS Demo: Relative Navigation (Vision+IMU) + Impedance Contact Capture
Purpose: A compact 2D simulation to illustrate a chaser approaching a rotating target using:
- Bearing-only **vision** + noisy **IMU** to estimate relative state (EKF).
- Visual servo style guidance to center in the image.
- **Impedance control** upon contact to absorb impact and settle to a desired dock pose.
- Outputs rich plots and an animated GIF (frames/ directory has individual frames).

Run:
```bash
python main.py
```

Outputs:
- `outputs/traj.png`         : relative trajectory (est vs. truth)
- `outputs/image_error.png`  : image-plane error vs. time
- `outputs/force_profile.png`: contact force vs. time
- `outputs/demo.gif`         : simple animation of approach and docking
- `outputs/benchmarks.json`  : time-to-capture, peak force, final errors
