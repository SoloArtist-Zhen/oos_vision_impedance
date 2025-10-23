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

  
# OOS 演示：相对导航（视觉+IMU）+阻抗接触捕捉
目的：一个简洁的二维仿真，演示追逐者如何接近旋转目标，使用：
- 纯方位**视觉** + 噪声**IMU** 估计相对状态（扩展卡尔曼滤波）。
- 视觉伺服式引导，使目标位于图像中心。
- 接触时**阻抗控制**，吸收冲击力并稳定到所需的停靠位置。
- 输出丰富的图表和 GIF 动画（frames/ 目录包含单独的帧）。
![Uploading demo.gif…]()

<img width="1020" height="510" alt="image_error" src="https://github.com/user-attachments/assets/f405fa39-d763-4992-aa0a-bc1fcbd7fe2a" />
<img width="1020" height="646" alt="traj" src="https://github.com/user-attachments/assets/be245acd-1890-41a2-8533-d04effb13918" />
