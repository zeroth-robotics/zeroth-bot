import pykos 

kos = pykos.KOS("192.168.42.1")

kos.actuator.get_actuators_state([1])

kos.actuator.configure_actuator(1, torque_enabled=True)

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