"""Test the connections to the robot."""

import pykos

kos = pykos.KOS("192.168.42.1")

for i in range(1, 17):  
    kos.actuator.configure_actuator(actuator_id=i, kp=32, kd=32, torque_enabled=True)

print(kos.actuator.get_actuators_state([1]))

for id in range(1, 17):
    kos.actuator.configure_actuator(actuator_id=id, torque_enabled=False)
print("Torque disabled")