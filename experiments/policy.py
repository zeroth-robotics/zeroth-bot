""" Deploying policy for standing tests 

<motor name="left_hip_roll" joint="left_hip_roll" ctrllimited="true" ctrlrange="-20 20" />
<motor name="left_hip_yaw" joint="left_hip_yaw" ctrllimited="true" ctrlrange="-20 20" />
<motor name="left_hip_pitch" joint="left_hip_pitch" ctrllimited="true" ctrlrange="-20 20" />
<motor name="left_knee_pitch" joint="left_knee_pitch" ctrllimited="true" ctrlrange="-20 20" />
<motor name="left_ankle_pitch" joint="left_ankle_pitch" ctrllimited="true" ctrlrange="-20 20" />
<motor name="right_hip_roll" joint="right_hip_roll" ctrllimited="true" ctrlrange="-20 20" />
<motor name="right_hip_yaw" joint="right_hip_yaw" ctrllimited="true" ctrlrange="-20 20" />
<motor name="right_hip_pitch" joint="right_hip_pitch" ctrllimited="true" ctrlrange="-20 20" />
<motor name="right_knee_pitch" joint="right_knee_pitch" ctrllimited="true" ctrlrange="-20 20" />
<motor name="right_ankle_pitch" joint="right_ankle_pitch" ctrllimited="true" ctrlrange="-20 20" />
"""
import time

import numpy as np
import pykos
# from imu import HexmoveImuReader
from kinfer.inference import ONNXModel

DT = FREQUENCY = 1/100. # 100Hz

class RealPPOController:
    def __init__(
            self, 
            model_path: str,
            joint_mapping_signs: np.ndarray,
            kos: pykos.KOS = None
        ) -> None:

        if kos is None:
            self.kos = pykos.KOS()
        else:
            self.kos = kos

        # Walking command defaults
        self.command = {
            "x_vel": 0.1,
            "y_vel": 0.0,
            "rot": 0.0,
        }

        self.joint_mapping_signs = joint_mapping_signs

        # Get model metadata
        # self.kinfer = ONNXModel(model_path)
        # metadata = self.kinfer.get_metadata()
        # self.model_info = {
        #     "num_actions": metadata["num_actions"],
        #     "num_observations": metadata["num_observations"],
        #     "robot_effort": metadata["robot_effort"],
        #     "robot_stiffness": metadata["robot_stiffness"],
        #     "robot_damping": metadata["robot_damping"],
        #     "default_standing": metadata["default_standing"],
        # }

        # Add IMU initialization
        # self.imu_reader = HexmoveImuReader("can0", 1, 1)
        # self.euler_signs = np.array([1, 1, 1])

        self.left_arm_ids = [14, 15, 16]
        self.right_arm_ids = [11, 12, 13]
        
        self.left_leg_ids = [10, 9, 8, 7, 6]
        self.right_leg_ids = [5, 4, 3, 2, 1]

        self.all_ids = self.left_leg_ids + self.right_leg_ids

        self.xml_offsets = np.array([0, 0, 0, 2.76, 1.19,  0, 0, 0, 0, 0])
        self.model_info = {
            "default_standing": np.array([0.0, 0.0, 0.296, 2.2, 0.927, 0.0, 0.0, -0.296, 0.579, 0.283])
        }

        self.model_info["default_standing"] = self.model_info["default_standing"] - self.xml_offsets

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
        breakpoint()
        # eu_ang[eu_ang > math.pi] -= 2 * math.pi
        # self.zero_position()

        self.kos.actuator.command_actuators([{"actuator_id": self.all_ids[0], "position": np.rad2deg(self.offsets[0])}])
        self.kos.actuator.command_actuators([{"actuator_id": self.all_ids[1], "position": np.rad2deg(self.offsets[1])}])
        self.kos.actuator.command_actuators([{"actuator_id": self.all_ids[2], "position": np.rad2deg(self.offsets[2])}])
        self.kos.actuator.command_actuators([{"actuator_id": self.all_ids[3], "position": np.rad2deg(self.offsets[3])}])
        self.kos.actuator.command_actuators([{"actuator_id": self.all_ids[4], "position": np.rad2deg(self.offsets[4])}])
        self.kos.actuator.command_actuators([{"actuator_id": self.all_ids[5], "position": np.rad2deg(self.offsets[5])}])
        self.kos.actuator.command_actuators([{"actuator_id": self.all_ids[6], "position": np.rad2deg(self.offsets[6])}])
        self.kos.actuator.command_actuators([{"actuator_id": self.all_ids[7], "position": np.rad2deg(self.offsets[7])}])
        self.kos.actuator.command_actuators([{"actuator_id": self.all_ids[8], "position": np.rad2deg(self.offsets[8])}])
        self.kos.actuator.command_actuators([{"actuator_id": self.all_ids[9], "position": np.rad2deg(self.offsets[9])}])

        self.set_default_position()
        # Initialize input state with dynamic sizes from metadata
        self.input_data = {
            "x_vel.1": np.zeros(1, dtype=np.float32),
            "y_vel.1": np.zeros(1, dtype=np.float32),
            "rot.1": np.zeros(1, dtype=np.float32),
            "t.1": np.zeros(1, dtype=np.float32),
            "dof_pos.1": np.zeros(self.model_info["num_actions"], dtype=np.float32),
            "dof_vel.1": np.zeros(self.model_info["num_actions"], dtype=np.float32),
            "prev_actions.1": np.zeros(self.model_info["num_actions"], dtype=np.float32),
            "imu_ang_vel.1": np.zeros(3, dtype=np.float32),
            "imu_euler_xyz.1": np.zeros(3, dtype=np.float32),
            "buffer.1": np.zeros(self.model_info["num_observations"], dtype=np.float32),
        }

        # Track previous actions and buffer for recurrent state
        self.actions = np.zeros(self.model_info["num_actions"], dtype=np.float32)
        self.buffer = np.zeros(self.model_info["num_observations"], dtype=np.float32)

        self.set_default_position()
        time.sleep(3)
        print('Default position set')

    def update_robot_state(self) -> None:
        """Update input data from robot sensors"""
        angles = np.array([0, 0, 0])
        imu_ang_vel = np.array([0, 0, 0])
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

        # Update input dictionary
        self.input_data["dof_pos.1"] = joint_positions.astype(np.float32)
        self.input_data["dof_vel.1"] = joint_velocities.astype(np.float32)
        self.input_data["imu_ang_vel.1"] = imu_ang_vel.astype(np.float32)
        self.input_data["imu_euler_xyz.1"] = angles.astype(np.float32)
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
        self.move_actuators(expected_positions)

        return positions


def main() -> None:
    kos = pykos.KOS("192.168.42.1")
    motor_signs = np.array([1, 1, -1, -1, -1, 1, 1, -1, -1, -1])

    standing = "policies/gpr_standing.kinfer"

    controller = RealPPOController(
        model_path=standing,
        joint_mapping_signs=motor_signs,
        kos=kos,
    )

    time.sleep(1)
    counter = 0
    try:
        while True:
            loop_start_time = time.time()
            controller.step(DT * counter)
            counter += 1
            time.sleep(max(0, FREQUENCY - (time.time() - loop_start_time)))
    except KeyboardInterrupt:
        print("Exiting...")
    except RuntimeError as e:
        print(e)
    finally:
        for id in controller.all_ids:
            controller.kos.actuator.configure_actuator(actuator_id=id, torque_enabled=False)

        print("Torque disabled")


if __name__ == "__main__":
    main()