import hid
import time
from typing import List, Tuple, Dict

class ServoController:
    CMD_SERVO_MOVE = 0x03
    CMD_GET_BATTERY_VOLTAGE = 0x0F
    CMD_DISABLE_MOTORS = 0x14
    CMD_READ_SERVO_ANGLE = 0x15
    FRAME_HEADER = 0x55

    def __init__(self, vid: int, pid: int):
        self.device = None
        self.timeout = 1.0  # Default timeout of 1 second

        try:
            self.device = hid.device()
            self.device.open(vid, pid)
            self.device.set_nonblocking(False)
        except IOError as e:
            print(f"Error opening device: {e}")

    def write(self, data: List[int]):
        if not self.device:
            raise IOError("Device not opened")
        
        # Pad data to 64 bytes
        padded_data = data + [0] * (64 - len(data))
        self.device.write(padded_data[:64])

    def read(self) -> List[int] | None:
        if not self.device:
            raise IOError("Device not opened")
        
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            data = self.device.read(64, self.timeout)
            if data:
                return list(data)
            time.sleep(0.0001)  # Short sleep to prevent busy-waiting
        
        return None  # Return None if timeout occurs

    def move_servos(self, servos: List[Tuple[int, int]], time_ms: int):
        num_servos = len(servos)
        if num_servos < 1 or num_servos > 32 or time_ms <= 0:
            raise ValueError("Invalid number of servos or time")

        data_length = num_servos * 3 + 5
        command_data = [
            self.FRAME_HEADER, self.FRAME_HEADER,
            data_length,
            self.CMD_SERVO_MOVE,
            num_servos,
            time_ms & 0xFF,  # Low byte of time
            (time_ms >> 8) & 0xFF  # High byte of time
        ]

        for servo_id, angle in servos:
            command_data.extend([
                servo_id,
                angle & 0xFF,  # Low byte of angle
                (angle >> 8) & 0xFF  # High byte of angle
            ])

        self.write(command_data)

    def get_battery_voltage(self) -> float:
        command_data = [
            self.FRAME_HEADER, self.FRAME_HEADER,
            2,  # Data length
            self.CMD_GET_BATTERY_VOLTAGE
        ]

        self.write(command_data)

        # Read the response
        response = self.read()
        if response and len(response) == 64:
            # Check for valid frame header and command
            if (response[0] == self.FRAME_HEADER and 
                response[1] == self.FRAME_HEADER and 
                response[3] == self.CMD_GET_BATTERY_VOLTAGE):
                
                data_length = response[2]
                if data_length == 4:  # Expected length for battery voltage response
                    voltage_mv = (response[5] << 8) | response[4]
                    return voltage_mv / 1000.0  # Convert mV to V
                else:
                    raise ValueError(f"Unexpected data length: {data_length}")
            else:
                raise ValueError("Invalid response header or command")
        else:
            raise IOError("Failed to read battery voltage")

    def read_servo_angles(self, servo_ids: List[int]) -> Dict[int, int]:
        if not servo_ids or any(id < 0 or id > 255 for id in servo_ids):
            raise ValueError("Invalid servo IDs")

        num_servos = len(servo_ids)
        command_data = [
            self.FRAME_HEADER, self.FRAME_HEADER,
            num_servos + 3,  # Data length: num_servos + 3 (command byte + num_servos byte)
            self.CMD_READ_SERVO_ANGLE,
            num_servos
        ] + servo_ids

        self.write(command_data)

        response = self.read()
        # print(response)

        if response and len(response) == 64:
            # Check for valid frame header and command
            if (response[0] == self.FRAME_HEADER and 
                response[1] == self.FRAME_HEADER and 
                response[3] == self.CMD_READ_SERVO_ANGLE):
                
                data_length = response[2]
                num_servos = (data_length - 2) // 3  # Each servo data is 3 bytes

                servo_angles = {}
                for i in range(num_servos):
                    servo_id = response[4 + i*3]
                    angle = int.from_bytes(response[6 + i*3 : 8 + i*3], byteorder='little', signed=True)
                    servo_angles[response[5 + i * 3]] = angle

                return servo_angles
            else:
                raise ValueError("Invalid response header or command")
        else:
            raise IOError("Failed to read servo angles")

    def disable_motors(self, id_from: int, id_to: int):
        if id_from < 0 or id_to > 255 or id_from > id_to:
            raise ValueError("Invalid servo ID range")

        command_data = [
            self.FRAME_HEADER, self.FRAME_HEADER,
            4,  # Data length
            self.CMD_DISABLE_MOTORS,
            id_from,
            id_to
        ]

        self.write(command_data)
