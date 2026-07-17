"""
Scripted hardware demo.

The board runs the whole control loop, so the Pi only opens the link, provisions
the board, sets the gait and streams the scripted setpoints while heartbeating.
"""
import argparse
import time

from hexapod import GaitId, connect


GAITS = {"tripod": GaitId.TRIPOD, "wave": GaitId.WAVE, "ripple": GaitId.RIPPLE}

# Seconds to keep feeding the watchdog after a shutdown so the board can finish 
# its sit-down before we drop the link
SIT_GRACE_S = 2.5


def graceful_stop(bot, control_rate):
    # Stop motion, sit the robot down and keep the watchdog fed until it settles
    control_dt = 1.0 / control_rate
    try:
        bot.set_velocity(0.0, 0.0, 0.0)
        bot.shutdown()
        elapsed = 0.0
        while elapsed < SIT_GRACE_S:
            bot.heartbeat()
            time.sleep(control_dt)
            elapsed += control_dt
    except Exception:
        pass


def main():

    ap = argparse.ArgumentParser(description="Hexapod hardware demo")
    ap.add_argument("--gait", "-g", default="tripod", choices=list(GAITS))
    ap.add_argument("--vx", "-x", type=float, default=120.0, help="forward mm/s")
    ap.add_argument("--vy", "-y", type=float, default=0.0, help="strafe mm/s")
    ap.add_argument("--yaw", "-v", type=float, default=20.0, help="yaw deg/s")
    ap.add_argument("--control-rate", "-c", type=float, default=50.0,
                    help="Hz (matches cfg::CONTROL_RATE_HZ)")
    ap.add_argument("--config", default=None,
                    help="path to config.yml (defaults to the packaged one)")
    ap.add_argument("--port", "-p", default=None,
                    help="serial port (overrides serial.port from the config)")
    ap.add_argument("--baud", "-b", type=int, default=921600, help="serial baud")
    ap.add_argument("--no-provision", action="store_true",
                    help="skip provisioning if the board is already set up")
    args = ap.parse_args()

    # Open the link and provision the board, then pick the gait
    bot = connect(config=args.config, port=args.port, baud=args.baud,
                  provision=not args.no_provision)
    bot.set_gait(GAITS[args.gait])
    print(f"connected, gait={args.gait}, running scripted demo")

    # Scripted timeline (seconds -> label, action)
    timeline = [
        (0.5,  "enable (stand up)",     lambda: bot.enable()),
        (5.0,  "body pose tilt A",      lambda: bot.set_body_pose(roll=5.0, pitch=-5.0, yaw=5.0)),
        (6.0,  "body pose tilt B",      lambda: bot.set_body_pose(roll=-5.0, pitch=-5.0, yaw=-5.0)),
        (7.0,  "body pose level",       lambda: bot.set_body_pose(roll=0.0, pitch=0.0, yaw=0.0)),
        (8.0,  "walk forward",          lambda: bot.set_velocity(args.vx, args.vy, 0.0)),
        (12.0, "walk forward + yaw",    lambda: bot.set_velocity(args.vx, args.vy, args.yaw)),
        (18.0, "stop",                  lambda: bot.set_velocity(0.0, 0.0, 0.0)),
        (22.0, "shutdown (sit down)",   lambda: bot.shutdown()),
        (25.0, "done",                  None),
    ]
    ti = 0

    control_dt = 1.0 / args.control_rate
    t = 0.0
    last = time.perf_counter()
    try:
        while True:
            # Fire the next scheduled action(s)
            while ti < len(timeline) and t >= timeline[ti][0]:
                when, label, action = timeline[ti]
                print(f"[{when:5.1f}s] {label}")
                if action is None:
                    return
                action()
                ti += 1

            bot.heartbeat()                  # Keep the command watchdog fed
            t += control_dt
            last += control_dt               # Pace to wall clock
            time.sleep(max(0.0, last - time.perf_counter()))
    except KeyboardInterrupt:
        print("\ninterrupted, sitting the robot down")
        graceful_stop(bot, args.control_rate)
    finally:
        try:
            print("final telemetry:", bot.get_telemetry())
        except Exception:
            pass
        bot.close()


if __name__ == "__main__":
    main()
