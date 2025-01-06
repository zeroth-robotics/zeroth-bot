import pykos 

kos = pykos.KOS("192.168.42.1")


ID_TO_SIGNS = {
    1: 1,
    2: 1,
    3: 1,
    4: 1,
    5: 1,
    6: 1,
    7: -1,
    8: -1,
    9: -1,
    10: -1,
    11: 1,
    12: 1,
    13: 1,
    14: 1,
    15: 1,
    16: 1,
}

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
breakpoint()

for i in range(1, 17):  
    kos.actuator.configure_actuator(i, torque_enabled=True)

kos.actuator.configure_actuator(1, torque_enabled=True)


kos.actuator.get_actuators_state([1])
kos.actuator.command_actuators([{"actuator_id": 1, "position": -80}])


kos.actuator.command_actuators([
    {"actuator_id": 1, "position": 0},
    {"actuator_id": 2, "position": 45},
    {"actuator_id": 3, "position": 10},
    {"actuator_id": 4, "position": 45},
    {"actuator_id": 5, "position": 0},
    {"actuator_id": 6, "position": 0},
    {"actuator_id": 7, "position": -45},
    {"actuator_id": 8, "position": -10},
    {"actuator_id": 9, "position": -45},
    {"actuator_id": 10, "position": 0}
])
