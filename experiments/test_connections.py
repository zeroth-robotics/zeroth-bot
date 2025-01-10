"""Test the connections to the robot."""

import pykos


ID_TO_JOINT_NAME = {
    1: "right_ankle_pitch",
    2: "right_knee_pitch",
    3: "right_hip_pitch",
    4: "right_hip_yaw",
    5: "right_hip_roll",
    6: "left_ankle_pitch",
    7: "left_knee_pitch",
    8: "left_hip_pitch",
    9: "left_hip_yaw",
    10: "left_hip_roll",
    11: "right_elbow_pitch",
    12: "right_shoulder_pitch",
    13: "right_shoulder_yaw",
    14: "left_shoulder_yaw",
    15: "left_shoulder_pitch",
    16: "left_elbow_pitch",
}

JOINT_NAME_TO_ID = {v: k for k, v in ID_TO_JOINT_NAME.items()}

kos = pykos.KOS("192.168.42.1")

for i in range(1, 17):  
    kos.actuator.configure_actuator(actuator_id=i, kp=32, kd=32, torque_enabled=True)

print(kos.actuator.get_actuators_state([2]))

for id in range(1, 17):
    print(kos.actuator.get_actuators_state([id])[0].position)

joint_id = JOINT_NAME_TO_ID["right_hip_pitch"]
position = kos.actuator.get_actuators_state([joint_id])[0].position
breakpoint()
# kos.actuator.command_actuators([{"actuator_id": joint_id, "position": position -15}])
for id in range(1, 17):
    kos.actuator.configure_actuator(actuator_id=id, torque_enabled=False)
print("Torque disabled")

