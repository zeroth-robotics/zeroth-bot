""" Deploying policy for tests.

Run:
    python deploy_policy.py --model zbot_walking_armature_friction.kinfer
"""
import argparse
import time

import numpy as np
import pykos
from kinfer.inference import ONNXModel
import logging


def get_gravity_orientation(quaternion):
    """
    Args:
        quaternion: np.ndarray[float, float, float, float]
    
    Returns:
        gravity_orientation: np.ndarray[float, float, float]
    """
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)

    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


ACTIONS_OUTPUT_ORDER = {
    10: "L_Hip_Roll",
    9: "L_Hip_Yaw",
    8: "L_Hip_Pitch",
    7: "L_Knee_Pitch",
    6: "L_Ankle_Pitch",
    5: "R_Hip_Roll",
    4: "R_Hip_Yaw",
    3: "R_Hip_Pitch",
    2: "R_Knee_Pitch",
    1: "R_Ankle_Pitch",
}


ID_TO_JOINT_NAME = {
    # "left_hip_yaw": 31,
    # "left_hip_roll": 32,
    # "left_hip_pitch": 33,
    # "left_knee_pitch": 34,
    # "left_ankle_pitch": 35,
    # "right_hip_yaw": 41,
    # "right_hip_roll": 42,
    # "right_hip_pitch": 43,
    # "right_knee_pitch": 44,
    # "right_ankle_pitch": 45,
    31: "L_Hip_Yaw",
    32: "L_Hip_Roll",
    33: "L_Hip_Pitch",
    34: "L_Knee_Pitch",
    35: "L_Ankle_Pitch",
    41: "R_Hip_Yaw",
    42: "R_Hip_Roll",
    43: "R_Hip_Pitch",
    44: "R_Knee_Pitch",
    45: "R_Ankle_Pitch",
}

# ID_TO_JOINT_NAME = {
#     1: "R_Ankle_Pitch",
#     2: "R_Knee_Pitch",
#     3: "R_Hip_Pitch",
#     4: "R_Hip_Yaw",
#     5: "R_Hip_Roll",
#     6: "L_Ankle_Pitch",
#     7: "L_Knee_Pitch",
#     8: "L_Hip_Pitch",
#     9: "L_Hip_Yaw",
#     10: "L_Hip_Roll",
#     11: "R_Elbow_Pitch",
#     12: "R_Shoulder_Pitch",
#     13: "R_Shoulder_Yaw",
#     14: "L_Shoulder_Yaw",
#     15: "L_Shoulder_Pitch",
#     16: "L_Elbow_Pitch",
# }

MOTOR_SIGNS = {
    "R_Ankle_Pitch": 1,
    "R_Knee_Pitch": 1,
    "R_Hip_Pitch": 1,
    "R_Hip_Yaw": 1,
    "R_Hip_Roll": 1,
    "L_Ankle_Pitch": 1,
    "L_Knee_Pitch": 1,
    "L_Hip_Pitch": 1,
    "L_Hip_Yaw": -1,
    "L_Hip_Roll": 1,
}


class RealPPOController:
    def __init__(
            self, 
            model_path: str,
            kos: pykos.KOS = None,
            logger: logging.Logger = None,
            disable_torque: bool = False,
        ) -> None:

        if kos is None:
            self.kos = pykos.KOS()
        else:
            self.kos = kos

        self.logger = logger

        # Walking command defaults
        self.command = {
            "x_vel": 0.4,
            "y_vel": 0.0,
            "rot": 0.0,
        }

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
        self.frequency = metadata["sim_dt"] * metadata["sim_decimation"]

        self.left_leg_ids = [32, 31, 33, 34, 35]
        self.right_leg_ids = [42, 41, 43, 44, 45]
        self.all_ids = self.left_leg_ids + self.right_leg_ids

        self.joint_mapping_signs = np.array([MOTOR_SIGNS[ID_TO_JOINT_NAME[id]] for id in self.all_ids])

        self.model_info["default_standing"] = np.array([
            0.00,    # L_Hip_Roll
            0.00,    # L_Hip_Yaw  
            -0.377, # L_Hip_Pitch
            0.796,  # L_Knee_Pitch
            0.377,  # L_Ankle_Pitch
            -0.00,    # R_Hip_Roll
            -0.00,    # R_Hip_Yaw
            0.377,  # R_Hip_Pitch
            -0.796, # R_Knee_Pitch  
            -0.377  # R_Ankle_Pitch
        ])
        
        for id in self.all_ids:
            # self.kos.actuator.configure_actuator(actuator_id=id, kp=70, kd=32, torque_enabled=True)
            self.kos.actuator.configure_actuator(actuator_id=id, kp=70, kd=32, torque_enabled=True, zero_position=False)

        self.initial_offsets = []

        for id in self.all_ids:
            self.initial_offsets.append(np.deg2rad(self.kos.actuator.get_actuators_state([id]).states[0].position))
        print(f"Initial offsets: {self.initial_offsets}")
        self.initial_offsets = np.array([0 for ii in range(10)])

        self.set_initial_offsets()

        # Adjust for the sign of each joint
        self.standing_offsets = self.joint_mapping_signs * self.model_info["default_standing"]
        self.offsets = self.initial_offsets + self.standing_offsets  # in radians
        print(f"Offsets: {self.offsets}")

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
        time.sleep(1)

        print('Default position set')

        # Add a scaling factor as an attribute
        self.pitch_scale = 1

    def set_initial_offsets(self) -> None:
        for idx, id in enumerate(self.all_ids):
            self.kos.actuator.command_actuators([{"actuator_id": id, "position": np.rad2deg(self.initial_offsets[idx])}])

    def update_robot_state(self) -> None:
        """Update input data from robot sensors"""
        motor_feedback = self.kos.actuator.get_actuators_state(actuator_ids=self.all_ids).states

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

        joint_positions -= self.offsets # in radians

        joint_positions = self.joint_mapping_signs * joint_positions
        joint_velocities = self.joint_mapping_signs * joint_velocities

        # TODO - check this against mujoco
        # pfb30
        # imu = self.kos.imu.get_imu_advanced_values()
        # zbot_grav_x, zbot_grav_y, zbot_grav_z = imu.grav_x, imu.grav_y, imu.grav_z
        # projected_gravity = -(1 / 9.8) * np.array([zbot_grav_z, zbot_grav_x, zbot_grav_y])
        # print(projected_gravity)
        projected_gravity = np.array([0.0, 0.0, -1.0])

        # Update input dictionary
        self.input_data["dof_pos.1"] = joint_positions.astype(np.float32)
        self.input_data["dof_vel.1"] = joint_velocities.astype(np.float32)
        self.input_data["projected_gravity.1"] = projected_gravity.astype(np.float32)
        self.input_data["prev_actions.1"] = self.actions
        self.input_data["buffer.1"] = self.buffer

    def zero_position(self) -> None:
        """Zero out the position of the robot"""
        for id in self.all_ids:
            self.kos.actuator.configure_actuator(actuator_id=id, torque_enabled=False)

    def set_default_position(self) -> None:
        """Set the robot to the default position!"""
        self.move_actuators(np.rad2deg(self.offsets))

    def set_zero_position(self) -> None:
        """Set the robot to the zero position!"""
        self.move_actuators(np.zeros(self.model_info["num_actions"]))

    def move_actuators(self, positions: np.ndarray) -> None:
        """Move actuators to desired positions in degrees"""
        left_positions = positions[:5]
        right_positions = positions[5:]

        actuator_commands = []

        for id, position in zip(self.left_leg_ids, left_positions):
            actuator_commands.append({"actuator_id": id, "position": position})

        for id, position in zip(self.right_leg_ids, right_positions):
            actuator_commands.append({"actuator_id": id, "position": position})

        self.kos.actuator.command_actuators(actuator_commands)

    def step(self, time_freq: float) -> np.ndarray:
        """Run one control step!"""
        # Update command velocities
        self.input_data["x_vel.1"][0] = np.float32(self.command["x_vel"])
        self.input_data["y_vel.1"][0] = np.float32(self.command["y_vel"])
        self.input_data["rot.1"][0] = np.float32(self.command["rot"])
        self.input_data["t.1"][0] = np.float32(self.frequency * time_freq)

        # Update robot state
        time_start = time.time()
        self.update_robot_state()
        time_end = time.time()
        # print(f"Time taken update robot state: {time_end - time_start}")

        # Run inference
        outputs = self.kinfer(self.input_data)

        # Extract outputs
        positions = outputs["actions_scaled"]

        # **Multiply knee and hip pitch by 1.5**
        positions[[2, 3, 7, 8]] *= self.pitch_scale

        self.actions = outputs["actions"]
        self.buffer = outputs["x.3"]

        positions = self.joint_mapping_signs * positions

        expected_positions = positions + self.offsets  # in radians

        time_start = time.time()
        self.move_actuators(np.rad2deg(expected_positions))
        time_end = time.time()
        # print(f"Time taken move actuators: {time_end - time_start}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, default="zbot_walking_armature_friction.kinfer")
    # parser.add_argument("--ip", type=str, default="10.33.10.63")
    parser.add_argument("--ip", type=str, default="192.168.42.1")
    parser.add_argument("--d_torque", type=bool, default=False)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    kos = pykos.KOS(args.ip)

    controller = RealPPOController(
        model_path=args.model,
        kos=kos,
        logger=logger,
        disable_torque=args.d_torque,
    )

    time.sleep(1)
    counter = 0
    try:
        logger.info("Start loop")

        while True:
            loop_start_time = time.time()
            controller.step(counter)
            counter += 1
            sleep_time = max(0, controller.frequency - (time.time() - loop_start_time))
            
            print(f"Time taken: {time.time() - loop_start_time}, sleep time: {sleep_time}")
            time.sleep(sleep_time)

            if counter > 1000: # 1 second out
                raise RuntimeError("1 second out")

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
