"""Script to record the positions of the actuators."""

import argparse
import logging
import time

import pykos

logger = logging.getLogger(__name__)

ACTUATOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.42.1")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO)

    kos = pykos.KOS(args.ip)
    logger.info("Connected to KOS at %s", args.ip)

    imu_states = kos.imu.get_imu_advanced_values()
    while True:
        kos_joint_states = kos.actuator.get_actuators_state(ACTUATOR_IDS)
        logger.info("Positions: %s", [state.position for state in kos_joint_states])
        time.sleep(0.1)

        print(kos.imu.get_imu_advanced_values())

    # kos.actuator.configure_actuator(actuator_id=1, torque_enabled=True)
    # for id in ACTUATOR_IDS:
    #     kos.actuator.configure_actuator(actuator_id=id, torque_enabled=True)

    # time.sleep(1.0)

if __name__ == "__main__":
    main()
