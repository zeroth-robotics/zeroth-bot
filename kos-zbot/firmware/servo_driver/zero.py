from hiwonder_lsbc_v16 import ServoController
from time import sleep, time
from sys import argv

if __name__ == '__main__':
    controller = ServoController(vid=0x0483, pid=0x5750)
    x = int(argv[1])
    try:
        controller.move_servos([(x, int(argv[2]))], 5000)
        sleep(5)
    finally:
        controller.disable_motors(1, x)
    print(f'voltage: {controller.get_battery_voltage()}')
    # # print(f'pos {1}: {controller.read_servo_angles(1, 1)}')
    # # sleep(0.6)
    # # controller.disable_motors(1, 1)
    # f = open('policy1.txt', 'w')
    # try:
    #     while True:
    #         # input()
    #         start_time = time()
    #         angles = controller.read_servo_angles(list(range(1, 14)))
    #         print(f'{angles}')
    #         # f.write(f'{{"pos": {list(angles.values())}, "trans_time": 600}},\n')
    #         sleep(0.05)
    # finally:
    #     f.close()
