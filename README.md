<div align="center" style="text-align: center;">

  <h1>Zeroth-01 Bot</h1> 

<p> Super hackable, affordable, and end-to-end (sim2real, RL) 3D-printed open-source humanoid robot platform. Fully open-source, including hardware, SDK, and sim environments. BoM starts at $350. </p>

<p> This project is built by the open-source community and is currently work in progress. We welcome your feedback, issues, and pull requests in GitHub or joining our <a href="https://discord.gg/G6KP76uha5">Discord</a>. </p>

<h3>
  <a href="https://docs.kscale.dev/docs/zeroth-01">Docs</a>
  <span> · </span>
  <a href="https://discord.gg/G6KP76uha5">Contribute</a>
  <span> · </span>
  <a href="https://discord.gg/G6KP76uha5">Community</a>
</h3>

<img width="1491" alt="image" src="/public/wave.webp">

</div>

---

### Project Status
Public Beta. Zeroth-01 Bot is available for basic locomotion, vision, and speech, but expect breaking changes. We plan to release stable V1.0 in June 2025.

## Core Components

### KOS-ZBot - Operating System
[kos-zbot](https://github.com/kscalelabs/kos-zbot) provides the robot operating system and hardware abstraction layer:

- **Hardware drivers**: Servo control (Feetech), IMU interfaces, actuator management
- **Python API**: High-level control via `kos_zbot` package for easy developer experience in Python.
- **Real-time control**: Low-latency servo communication and sensor data processing  
- **CLI tools**: Robot calibration, diagnostics, and system configuration

### K-Sim Gym ZBot - RL Training
[ksim-gym-zbot](https://github.com/kscalelabs/ksim-gym-zbot) provides reinforcement learning training:

- **Training pipeline**: GPU-accelerated RL for whole-body control, from walking to human imitation
- **Sim-to-real**: SysID-calibrated actuators with pre-loaded MJCF and URDF models
- **Model deployment**: Trained policies export directly to real robots via KOS

### Build Guide
- Please see the Getting Started documentation at [https://docs.kscale.dev/docs/zeroth-01](https://docs.kscale.dev/docs/zeroth-01).

### To start developing Zeroth Bot
Zeroth Bot is developed by the open-source community. We welcome both pull requests and issues on GitHub.

- Join the community [Discord](https://discord.gg/G6KP76uha5)
- Documentation at [https://docs.kscale.dev/docs/zeroth-01](https://docs.kscale.dev/docs/zeroth-01)

### License
This project is licensed under the MIT License.
