from controller.hardware.kernel import Kernel
import time


if __name__ == '__main__':

    ker = Kernel('/dev/ttyAMA0', 115200)

    print(f"Voltage: {ker.get_voltage()}")
    print(f"Current: {ker.get_current()}")

    time.sleep(0.5)

    step = 5
    delay = 0.04

    try:
        while True:
            # Servo 0: 0 -> 45, Servo 1: 0 -> -45
            for angle in range(0, 46, step):
                ker.set_servo_angles([(1, angle), (2, -angle)])
                time.sleep(delay)

            # Servo 0: 45 -> 0, Servo 1: -45 -> 0
            for angle in range(45, -1, -step):
                ker.set_servo_angles([(1, angle), (2, -angle)])
                time.sleep(delay)

    except KeyboardInterrupt:
        print("Sweep stopped.")
        ker.set_servo_angles([(1, 0), (2, 0)])
        ker.detach_servos()
        ker.close()