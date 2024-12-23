import argparse
import time
from interface import Interface


# Interpolation function for a single servo
def interpolate_servo(current_angle, target_angle, speed, delta_time):
    if current_angle == target_angle:
        return current_angle  # No movement needed

    # Calculate the step size
    step = speed * delta_time
    # Move closer to the target
    if abs(target_angle - current_angle) <= step:
        return target_angle  # Reached the target

    return current_angle + step if target_angle > current_angle else current_angle - step


# Main loop
def main_loop(sid, min_angle, max_angle, speed, refresh_rate, duration):
    interface = Interface()

    with interface:
        current_angle = min_angle  # Start position
        target_angle = max_angle  # Start by moving towards max_angle
        start_time = time.time()  # Record the start time
        last_update_time = start_time  # Initialize the last update timestamp

        print("Attaching servos...")
        interface.attach_servos()
        time.sleep(1)

        while time.time() - start_time < duration:
            # Calculate the time delta
            now = time.time()
            delta_time = now - last_update_time
            last_update_time = now

            # Update the servo angle
            current_angle = interpolate_servo(current_angle, target_angle, speed, delta_time)

            # Send the updated angle to the servo
            interface.set_angle(sid, current_angle)

            # Reverse direction when the target is reached
            if current_angle == target_angle:
                target_angle = min_angle if target_angle == max_angle else max_angle
            
            # print(f'Setting servo [{sid}] to angle {current_angle:.2f}')

            # Wait for the next update
            time.sleep(refresh_rate)

        print("Detaching servos...")
        interface.detach_servos()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Control a servo producing a smooth motion.")
    parser.add_argument('--sid', type=int, default=0, help="Servo ID")
    parser.add_argument('--min_angle', type=float, default=0, help="Minimum angle for the servo (degrees)")
    parser.add_argument('--max_angle', type=float, default=90, help="Maximum angle for the servo (degrees)")
    parser.add_argument('--speed', type=float, default=90.0, help="Speed of servo movement (degrees per second)")
    parser.add_argument('--refresh_rate', type=float, default=0.02, help="Refresh rate of updates (seconds)")
    parser.add_argument('--duration', type=float, default=10.0, help="Duration of the program run (seconds)")

    args = parser.parse_args()

    # Run the main loop with arguments
    main_loop(
        sid=args.sid,
        min_angle=args.min_angle,
        max_angle=args.max_angle,
        speed=args.speed,
        refresh_rate=args.refresh_rate,
        duration=args.duration
    )
