from interface import Interface
import time

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
def main_loop(duration=5):
    
    interface = Interface()

    with interface:

        current_angle = 0.0   # Start position
        target_angle = 30.0   # Target position
        speed = 30.0          # Degrees per second
        refresh_rate = 0.01   # 10ms update interval
        sid = 1               # Servo ID
        start_time = time.time()  # Record the start time
        
        while time.time() - start_time < duration:
            # Calculate the time delta
            delta_time = refresh_rate
            
            # Update the servo angle
            current_angle = interpolate_servo(current_angle, target_angle, speed, delta_time)
            
            # Send the updated angle to the servo
            interface.set_angle(sid, current_angle)
            
            # Reverse direction when the target is reached
            if current_angle == target_angle:
                target_angle = 0.0 if target_angle == 30.0 else 30.0
            
            # Wait for the next update
            time.sleep(refresh_rate)

# Run the program
main_loop(duration=5)  # Run for 5 seconds
