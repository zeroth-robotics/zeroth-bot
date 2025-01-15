# Runtime Documentation

## Setup

1. Flash the Image to the SD Card
Download the OpenLCH image from the [releases page](https://github.com/Zeroth-Robotics/OpenLCH-buildroot/releases).

Use Balena Etcher (or any other flashing tool) to flash the image onto your SD card.

Once flashing is complete, remove and reinsert the SD card into your computer.

3. Configure Wi-Fi on Startup
Open the boot folder on the SD card (it should be visible on macOS since it is formatted as FAT32).
Create a new file named wpa_supplicant.conf in the boot folder and add the following content:

```bash
ctrl_interface=/var/run/wpa_supplicant
ap_scan=1
update_config=1

network={
  ssid="YOURWIFINAME"
  psk="YOURPASSWORD"
  key_mgmt=WPA-PSK
}
```
Replace YOURWIFINAME and YOURPASSWORD with your Wi-Fi credentials.

3. Connect via USB
If you're connecting via USB, use the following details:

- IP Address: 192.168.42.1
- Username: root
- Password: milkv

SSH:
`ssh root@192.168.42.1`

4. Build Runtime
```bash
cargo check && gitlab-ci-local # check and build
```

5. Transfer Files to the Target Device
Find binaries in target/riscv64gc-unknown-linux-musl/release/

`scp -O target/riscv64gc-unknown-linux-musl/release/runtime $MILKV_IP:/usr/local/bin/`

Debug:
Note that you cannot ping the device in Cursor editor terminal for some reason. Try:
```bash
ping 192.168.42.1
```

6. Run the Application
To run the servo setup, execute the following on the target device:

```bash

# List available binaries
ls /usr/local/bin/

# Run your desired binary (example: runtime)
 /usr/local/bin/runtime
```





If the board is connected over usb, ip is `192.168.42.1`






---

## Architecture

<img src="public/runtime.png" alt="Runtime Architecture">

The goal of the runtime is to provide a unified interface for:

- robot configuration and declaration
- control loop execution
- state management
- model inference
- error handling and safety

## `config/[robot-name].toml`

Define the robot: joints, servos, parameters, etc.

Based on robot from `kscalelabs/firmware` config.

## `robot.rs`

Create the robot struct based on the config.toml file with all the servos and joints.

Provides the states for the robot.

Based on robot from `kscalelabs/firmware` Robot struct.

## `hal.rs`

Servo control code

## `controller.rs`

`StandingControllerPID` -> controller for standing using pre-set positions and PID controller

`StandingControllerPPO` -> controller for standing using PPO model

## `model.rs`

onnx inference session and initalization.

## `main.rs`

initialize config, robot, controller and start standing using controller.


# Hardware
Servo ID:
![image](https://github.com/user-attachments/assets/93db0404-bf76-4b4f-9201-665cc868364b)

Wiring:
![image](https://github.com/user-attachments/assets/d1c02231-ae6e-4333-bf7e-74db47416b88)

