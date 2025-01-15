from hiwonder_lsbc_v16 import ServoController
from time import sleep, time

if __name__ == '__main__':
    controller = ServoController(vid=0x0483, pid=0x5750)
    # controller.move_servos([(1, 0)], 5000)
    print(f'voltage: {controller.get_battery_voltage()}')
    # print(f'pos {1}: {controller.read_servo_angles(1, 1)}')
    # sleep(0.6)
    # controller.disable_motors(1, 1)
    f = open('policy_walk.txt', 'w')
    try:
        while True:
            input()
            start_time = time()
            angles = controller.read_servo_angles(list(range(1, 14)))
            print(f'{angles}')
            f.write(f'{{"pos": {list(angles.values())}, "trans_time": 600}},\n')
            sleep(0.05)
    finally:
        f.close()
