""" Deploying policy for standing tests 
# Motor joint order:
# Left leg:
#   - left_hip_roll
#   - left_hip_yaw  
#   - left_hip_pitch
#   - left_knee_pitch
#   - left_ankle_pitch
# Right leg:
#   - right_hip_roll
#   - right_hip_yaw
#   - right_hip_pitch
#   - right_knee_pitch
#   - right_ankle_pitch
"""
import argparse
import time

import numpy as np
import pykos
# from imu import HexmoveImuReader
from kinfer.inference import ONNXModel
import logging


DT = FREQUENCY = 1/100. # 100Hz

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


class RealPPOController:
    def __init__(
            self, 
            model_path: str,
            joint_mapping_signs: np.ndarray,
            kos: pykos.KOS = None,
            logger: logging.Logger = None,
        ) -> None:

        if kos is None:
            self.kos = pykos.KOS()
        else:
            self.kos = kos

        self.logger = logger

        # Walking command defaults
        self.command = {
            "x_vel": 0.1,
            "y_vel": 0.0,
            "rot": 0.0,
        }

        self.joint_mapping_signs = joint_mapping_signs

        # Get model metadata
        self.kinfer = ONNXModel(model_path)
        metadata = self.kinfer.get_metadata()
        self.model_info = {
            "num_actions": metadata["num_actions"],
            "num_observations": metadata["num_observations"],
            "robot_effort": metadata["robot_effort"],
            "robot_stiffness": metadata["robot_stiffness"],
            "robot_damping": metadata["robot_damping"],
            "default_standing": metadata["default_standing"],
        }

        self.left_arm_ids = [14, 15, 16]
        self.right_arm_ids = [11, 12, 13]
        
        self.left_leg_ids = [10, 9, 8, 7, 6]
        self.right_leg_ids = [5, 4, 3, 2, 1]

        self.all_ids = self.left_leg_ids + self.right_leg_ids

        self.model_info["default_standing"] = np.array([0.0, 0.0, -0.377, 0.796, 0.377, 0.0, 0.0, 0.377, -0.796, -0.377])

        for id in self.all_ids:
            self.kos.actuator.configure_actuator(id, kp=32, kd=32, torque_enabled=True)

        self.initial_offsets = []
        for id in self.all_ids:
            self.initial_offsets.append(np.deg2rad(self.kos.actuator.get_actuators_state([id])[0].position))

        self.initial_offsets = np.array(self.initial_offsets)

        # Adjust for the sign of each joint
        self.left_offsets = self.joint_mapping_signs[:5] * np.array(self.model_info["default_standing"][:5])
        self.right_offsets = self.joint_mapping_signs[5:] * np.array(self.model_info["default_standing"][5:])
        self.offsets = np.concatenate([self.left_offsets, self.right_offsets])
        self.offsets = self.initial_offsets + self.offsets
        print(f"Offsets: {self.offsets}")

        # Add IMU initialization
        # self.imu_reader = HexmoveImuReader("can0", 1, 1)
        # self.projected_gravity = self.imu_reader.get_projected_gravity()
        # Initialize input state with dynamic sizes from metadata
        self.input_data = {
            "x_vel.1": np.zeros(1, dtype=np.float32),
            "y_vel.1": np.zeros(1, dtype=np.float32),
            "rot.1": np.zeros(1, dtype=np.float32),
            "t.1": np.zeros(1, dtype=np.float32),
            "dof_pos.1": np.zeros(self.model_info["num_actions"], dtype=np.float32),
            "dof_vel.1": np.zeros(self.model_info["num_actions"], dtype=np.float32),
            "prev_actions.1": np.zeros(self.model_info["num_actions"], dtype=np.float32),
            "projected_gravity.1": np.zeros(3, dtype=np.float32),
            "buffer.1": np.zeros(self.model_info["num_observations"], dtype=np.float32),
        }

        # Track previous actions and buffer for recurrent state
        self.actions = np.zeros(self.model_info["num_actions"], dtype=np.float32)
        self.buffer = np.zeros(self.model_info["num_observations"], dtype=np.float32)

        breakpoint()
        self.set_default_position()
        breakpoint()
        time.sleep(2)
        print('Default position set')

    def update_robot_state(self) -> None:
        """Update input data from robot sensors"""

        motor_feedback = self.kos.actuator.get_actuators_state(self.all_ids)

        # Create dictionary of motor feedback to motor id
        self.motor_feedback_dict = {
            motor.actuator_id: motor for motor in motor_feedback
        }

        # Check that each motor is enabled
        for motor in self.motor_feedback_dict.values():
            if not motor.online and motor.actuator_id in self.left_leg_ids + self.right_leg_ids:
                raise RuntimeError(f"Motor {motor.actuator_id} is not online")

        # Should be arranged left to right, top to bottom
        joint_positions = np.concatenate([
            np.array([self.motor_feedback_dict[id].position for id in self.left_leg_ids]),
            np.array([self.motor_feedback_dict[id].position for id in self.right_leg_ids])
        ])

        joint_velocities = np.concatenate([
            np.array([self.motor_feedback_dict[id].velocity for id in self.left_leg_ids]),
            np.array([self.motor_feedback_dict[id].velocity for id in self.right_leg_ids])
        ])

        joint_positions = np.deg2rad(joint_positions)
        joint_velocities = np.deg2rad(joint_velocities)

        joint_positions -= self.offsets

        joint_positions = self.joint_mapping_signs * joint_positions
        joint_velocities = self.joint_mapping_signs * joint_velocities

        # projected_gravity = self.imu_reader.get_projected_gravity()
        projected_gravity = np.array([0, 0, -1])
        # Update input dictionary
        self.input_data["dof_pos.1"] = joint_positions.astype(np.float32)
        self.input_data["dof_vel.1"] = joint_velocities.astype(np.float32)
        self.input_data["projected_gravity.1"] = projected_gravity.astype(np.float32)
        self.input_data["prev_actions.1"] = self.actions
        self.input_data["buffer.1"] = self.buffer

    def zero_position(self) -> None:
        """Zero out the position of the robot"""
        for id in self.all_ids:
            self.kos.actuator.configure_actuator(id, torque_enabled=False)

    def set_default_position(self) -> None:
        """Set the robot to the default position!"""
        self.move_actuators(np.rad2deg(self.offsets))

    def set_zero_position(self) -> None:
        """Set the robot to the zero position!"""
        self.move_actuators(np.zeros(self.model_info["num_actions"]))

    def move_actuators(self, positions: np.ndarray) -> None:
        """Move actuators to desired positions!"""
        left_positions = positions[:5]
        right_positions = positions[5:]

        actuator_commands = []

        for id, position in zip(self.left_leg_ids, left_positions):
            actuator_commands.append({"actuator_id": id, "position": position})

        for id, position in zip(self.right_leg_ids, right_positions):
            actuator_commands.append({"actuator_id": id, "position": position})

        self.kos.actuator.command_actuators(actuator_commands)

    def step(self, time: float) -> np.ndarray:
        """Run one control step!"""
        # Update command velocities
        self.input_data["x_vel.1"][0] = np.float32(self.command["x_vel"])
        self.input_data["y_vel.1"][0] = np.float32(self.command["y_vel"])
        self.input_data["rot.1"][0] = np.float32(self.command["rot"])
        self.input_data["t.1"][0] = np.float32(time)

        # Update robot state
        self.update_robot_state()

        # Run inference
        outputs = self.kinfer(self.input_data)

        assert isinstance(outputs, dict), "Outputs are not a dictionary!"
        assert "actions_scaled" in outputs, "actions_scaled not in outputs!"
        assert "actions" in outputs, "actions not in outputs!"
        assert "x.3" in outputs, "x.3 not in outputs!"

        # Extract outputs
        positions = outputs["actions_scaled"]

        self.actions = outputs["actions"]
        self.buffer = outputs["x.3"]

        # Clip positions for safety
        positions = np.clip(positions, -0.75, 0.75)

        positions = self.joint_mapping_signs * positions

        expected_positions = positions + self.offsets
        expected_positions = np.rad2deg(expected_positions)

        # Send positions to robot
        # EMI issues:
        # for _ in range(3):
        self.move_actuators(expected_positions)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, default="experiments/zbot_walking.kinfer")
    parser.add_argument("--ip", type=str, default="192.168.42.1")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    kos = pykos.KOS(args.ip)
    motor_signs = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1])

    standing = args.model

    controller = RealPPOController(
        model_path=standing,
        joint_mapping_signs=motor_signs,
        kos=kos,
        logger=logger,
    )

    time.sleep(1)
    counter = 0
    try:
        logger.info("Start loop")

        while True:
            loop_start_time = time.time()
            controller.step(DT * counter)
            counter += 1
            print(f"Time taken: {time.time() - loop_start_time}")
            time.sleep(max(0, FREQUENCY - (time.time() - loop_start_time)))
    except KeyboardInterrupt:
        logger.info("Exiting...")
    except RuntimeError as e:
        logger.error(e)
    finally:
        for id in controller.all_ids:
            controller.kos.actuator.configure_actuator(actuator_id=id, torque_enabled=False)

        logger.info("Torque disabled")


if __name__ == "__main__":
    main()