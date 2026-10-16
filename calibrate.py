"""
Offline per-servo pulse-width calibration.

This measures each servo's own pulse<->angle transfer function, ideally on the
bare servo before it is bolted into a leg, so it carries no knowledge of the
joint's mechanical range. For every servo you find just two reference pulses:
the one that puts the shaft at -90 degrees and the one that puts it at +90.
The midpoint pulse (0 degrees) is taken as their average. Nominally identical
servos respond slightly differently to the same pulse, so every one of the
18 servos is measured individually.

Because the endpoints are the servo's own +-90, the joint's structural offsets
(hardware.trim / hardware.direction) and limits (safety.*_range) stay in their
own config sections; this file only maps pulses to servo degrees.

This is a standalone tool, not part of the running robot: it talks to the board
only through the low-level JogServo debug command (the FSM stays OFF, servos are
driven raw), and writes the result back into the config under `hardware.pulses`
as [pulse@-90, pulse@0, pulse@+90] triples (per leg, ordered coxa / femur /
tibia). Next time you run the main software, connect() provisions those pulses
automatically.

Controls
--------
    [up   / w]   fine   +   ([down / s] fine   -)
    [right/ d]   coarse +   ([left / a] coarse -)
    [enter]      register current pulse for this reference angle (-90 -> +90)
    [u / bksp]   undo last registration on this servo
    [q / esc]    save what's done so far and quit

For each servo you jog to -90 degrees and register, then to +90 and register;
the tool stores those two pulses (plus their average as the midpoint) and moves
on to the next servo.

Reference pose check (--pose)
-----------------------------
Once the pulses are calibrated, --pose checks the structural offsets
(hardware.trim). It puts every leg at right angles (coxa straight out from the
body, femur horizontal, tibia vertical) using the config's direction, trim and
pulses, so a joint that is visibly off (e.g. a tibia that is not vertical) has
a wrong trim. Then type commands at the prompt:

    x y          set servo x (1..18) to angle y (0..180 deg, integer)
    r            put every servo back in the reference pose
    q            release all servos, save the trims and quit
    ctrl+c       release all servos and quit without saving

Servos are numbered leg-major in the canonical leg order, joints coxa / femur /
tibia (1-3 front_right, 4-6 middle_right, ..., 16-18 front_left). Angles are
servo angles: 90 is the servo's centre (the pulse@0 of its calibration). When
you find the angle that looks right, the trim for that joint changes by the
same amount you moved it from the reference. On quit, every servo left away
from its reference has its trim corrected in the config under `hardware.trim`
(a .bak copy of the old file is kept, --no-save skips the write).

Run:  python3 calibrate.py            [--port /dev/ttyAMA0] [--channels 0 1 2]
      python3 calibrate.py --pose     [--port /dev/ttyAMA0]
"""

from __future__ import annotations

import argparse
import re
import shutil
import time
from pathlib import Path

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedSeq

from sshkeyboard import listen_keyboard, stop_listening

from hexapod import HexapodClient, SerialTransport
from hexapod.client import DEFAULT_CONFIG, JOINTS, LEGS

# Round-trip YAML: reads and writes config.yml while preserving comments, key
# order and the inline pulse rows, so calibration never mangles the hand-edited file
yaml_rt = YAML()
yaml_rt.preserve_quotes = True


def channel_label(channel: int) -> str:
    leg, joint = divmod(channel, len(JOINTS))
    return f"{LEGS[leg]:12s} {JOINTS[joint]:5s} (ch {channel:2d})"


class ServoCalibrator:
    """Drive a keyboard-jog calibration session over a list of logical channels."""

    PHASES = ("min", "max")
    PHASE_LABEL = {"min": "-90 deg", "max": "+90 deg"}

    def __init__(
        self,
        client: HexapodClient,
        channels,
        small_step=5,
        large_step=50,
        min_pulse=500,
        max_pulse=2500,
        start_pulse=None,
    ):
        self.client = client
        self.channels = list(channels)
        self.small_step = small_step
        self.large_step = large_step
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.start_pulse = (
            start_pulse if start_pulse is not None else (min_pulse + max_pulse) // 2
        )

        self.results = {}  # channel -> [min, mid, max]
        self.job_idx = 0
        self.phase_idx = 0
        self.pulse = self.start_pulse
        self.captured = {}  # phase -> pulse, for the servo in progress
        self.finished = False

    @property
    def channel(self) -> int:
        return self.channels[self.job_idx]

    def _send(self) -> None:
        self.client.jog_servo(self.channel, int(round(self.pulse)))

    def _release(self) -> None:
        self.client.jog_servo(self.channel, 0)

    def _status(self) -> None:
        phase = self.PHASES[min(self.phase_idx, len(self.PHASES) - 1)]
        captured = "  ".join(f"{k}={int(round(v))}" for k, v in self.captured.items())
        line = (
            f"[{self.job_idx + 1:2d}/{len(self.channels)}] "
            f"{channel_label(self.channel)} | set {self.PHASE_LABEL[phase]:8s} | "
            f"pulse = {int(round(self.pulse)):4d} us"
        )
        if captured:
            line += f"   [{captured}]"
        print(f"\r{line}\033[K", end="", flush=True)

    def jog(self, step: int) -> None:
        self.pulse = min(self.max_pulse, max(self.min_pulse, self.pulse + step))
        self._send()
        self._status()
        time.sleep(0.02)

    def register(self) -> None:
        phase = self.PHASES[self.phase_idx]
        self.captured[phase] = self.pulse
        self.phase_idx += 1
        if self.phase_idx >= len(self.PHASES):
            self._finalize_servo()
        else:
            self._send()
            self._status()

    def undo(self) -> None:
        if self.phase_idx == 0:
            self._status()
            return
        self.phase_idx -= 1
        phase = self.PHASES[self.phase_idx]
        if phase in self.captured:
            self.pulse = self.captured.pop(phase)
        self._send()
        self._status()

    def _finalize_servo(self) -> None:
        # Keep the pulses in reference-angle order: [-90, mid, +90]. Do NOT sort;
        # a servo whose pulse decreases with angle legitimately has pulse@-90 >
        # pulse@+90, and the board's calibration maps it correctly either way.
        lo = self.captured["min"]  # pulse at -90 deg
        hi = self.captured["max"]  # pulse at +90 deg
        mid = (lo + hi) / 2.0  # pulse at 0 deg
        self.results[self.channel] = [int(round(lo)), int(round(mid)), int(round(hi))]
        self._release()
        print()  # keep the finished servo's line in the scrollback
        self.job_idx += 1
        if self.job_idx >= len(self.channels):
            self.finished = True
            print("\nAll servos calibrated.")
            _stop_keyboard()
            return
        self.phase_idx = 0
        self.captured = {}
        self.pulse = self.start_pulse
        self._send()
        self._status()

    def quit(self) -> None:
        print("\n\nStopping. Calibrated servos so far will be saved.")
        self._release()
        _stop_keyboard()

    def on_key(self, key: str) -> None:
        if key in ("up", "w"):
            self.jog(self.small_step)
        elif key in ("down", "s"):
            self.jog(-self.small_step)
        elif key in ("right", "d"):
            self.jog(self.large_step)
        elif key in ("left", "a"):
            self.jog(-self.large_step)
        elif key == "enter":
            self.register()
        elif key in ("u", "backspace"):
            self.undo()
        elif key in ("q", "esc"):
            self.quit()

    def run(self) -> None:

        print(__doc__)
        print(
            f"Step sizes: fine +/-{self.small_step} us, coarse +/-{self.large_step} us."
        )
        input("\nPress Enter to energize and begin (servos will move to centre)... ")
        self._send()
        self._status()
        try:
            listen_keyboard(self.on_key, delay_second_char=0.05)
        finally:
            for ch in self.channels:  # make sure nothing is left holding
                self.client.jog_servo(ch, 0)
            self.client.close()


def _stop_keyboard() -> None:
    stop_listening()


def flow(values) -> CommentedSeq:
    """A YAML list written inline, like the hand-edited rows in config.yml"""
    seq = CommentedSeq(values)
    seq.fa.set_flow_style()
    return seq


def save_results(config_path, results) -> Path:
    """Merge calibration into config.yml under `hardware.pulses`, preserving
    comments/formatting. Returns the backup path."""
    path = Path(config_path)
    backup = path.with_name(path.name + ".bak")
    shutil.copy2(path, backup)

    with path.open() as f:
        data = yaml_rt.load(f)
    hardware = data.setdefault("hardware", {})
    pulses = hardware.get("pulses") or {}

    n_joints = len(JOINTS)
    for channel, triple in results.items():
        leg, joint = divmod(channel, n_joints)
        leg_name = LEGS[leg]
        if leg_name not in pulses or not isinstance(pulses.get(leg_name), list):
            pulses[leg_name] = flow([flow([500, 1500, 2500]) for _ in JOINTS])
        pulses[leg_name][joint] = flow([int(v) for v in triple])
    hardware["pulses"] = pulses

    with path.open("w") as f:
        yaml_rt.dump(data, f)
    return backup


def save_trims(config_path, trims) -> Path:
    """Write corrected trims ({channel: deg}) into config.yml under
    `hardware.trim`. Only the numbers change: the rows are edited in place as
    text, so comments and the hand-aligned columns survive. Returns the backup
    path"""
    path = Path(config_path)
    backup = path.with_name(path.name + ".bak")
    shutil.copy2(path, backup)

    lines = path.read_text().splitlines(keepends=True)
    start = next(i for i, line in enumerate(lines)
                 if re.match(r"\s+trim:\s*(#.*)?$", line))
    for channel, value in trims.items():
        leg, joint = divmod(channel, len(JOINTS))
        row = re.compile(rf"(\s+{LEGS[leg]}:\s*\[)([^\]]*)(\].*)", re.DOTALL)
        for i in range(start + 1, len(lines)):
            match = row.match(lines[i])
            if match:
                values = [v.strip() for v in match.group(2).split(",")]
                values[joint] = f"{round(value, 1):g}"
                lines[i] = match.group(1) + ", ".join(values) + match.group(3)
                break
        else:
            raise ValueError(f"hardware.trim.{LEGS[leg]} not found in {path}")

    path.write_text("".join(lines))
    return backup


# Kinematic angles (deg) of the reference pose: coxa straight out, femur
# horizontal, tibia pointing straight down
REFERENCE_POSE = (0.0, 0.0, -90.0)


def angle_to_pulse(triple, angle: float) -> int:
    """Servo angle (0..180, 90 = centre) to pulse (us), piecewise linear through
    the calibrated [pulse@-90, pulse@0, pulse@+90] triple"""
    lo, mid, hi = (float(v) for v in triple)
    deg = angle - 90.0
    if deg < 0:
        return int(round(mid + (mid - lo) * deg / 90.0))
    return int(round(mid + (hi - mid) * deg / 90.0))


class PoseChecker:
    """Hold the robot in the right-angle reference pose and let the user move
    single servos to spot (and quantify) wrong trims."""

    def __init__(self, client: HexapodClient, cfg):
        self.client = client
        hardware = cfg.get("hardware", {})
        safety = cfg.get("safety", {})
        self.pulses = []     # per channel [pulse@-90, pulse@0, pulse@+90]
        self.trim = []       # per channel, deg
        self.reference = []  # per channel, servo angle (0..180) of the pose
        self.ranges = [[float(v) + 90.0 for v in safety[f"{j}_range"]] for j in JOINTS]
        for leg in LEGS:
            for joint in range(len(JOINTS)):
                pulses = hardware.get("pulses", {}).get(leg)
                self.pulses.append(pulses[joint] if pulses else (500, 1500, 2500))
                trim = float(hardware["trim"][leg][joint])
                direction = float(hardware["direction"][leg][joint])
                self.trim.append(trim)
                # Same map as the firmware (kin_deg * direction + trim), shifted to 0..180
                self.reference.append(REFERENCE_POSE[joint] * direction + trim + 90.0)
        self.angles = list(self.reference)

    def corrections(self) -> dict:
        """Corrected trim for every servo left away from its reference angle"""
        return {
            channel: self.trim[channel] + (now - ref)
            for channel, (ref, now) in enumerate(zip(self.reference, self.angles))
            if now != ref
        }

    def set_angle(self, channel: int, angle: float) -> None:
        self.angles[channel] = angle
        self.client.jog_servo(channel, angle_to_pulse(self.pulses[channel], angle))

    def reset(self) -> None:
        for channel, angle in enumerate(self.reference):
            self.set_angle(channel, angle)
            time.sleep(0.02)  # stagger the moves so the rail is not hit all at once

    def release(self) -> None:
        for channel in range(len(self.angles)):
            self.client.jog_servo(channel, 0)

    def print_table(self) -> None:
        print(f"\n{'servo':>5}  {'leg':12s} {'joint':5s}  {'reference':>9}  {'now':>5}")
        for channel, (ref, now) in enumerate(zip(self.reference, self.angles)):
            leg, joint = divmod(channel, len(JOINTS))
            print(f"{channel + 1:5d}  {LEGS[leg]:12s} {JOINTS[joint]:5s}  "
                  f"{ref:9.1f}  {now:5.1f}")

    def handle(self, line: str) -> bool:
        """Run one command; False once the user asks to quit"""
        parts = line.split()
        if not parts:
            return True
        if parts == ["q"]:
            return False
        if parts == ["r"]:
            self.reset()
            self.print_table()
            return True
        if len(parts) != 2 or not all(p.isdigit() for p in parts):
            print("Expected 'x y' (servo 1..18, angle 0..180), 'r' or 'q'")
            return True
        servo, angle = int(parts[0]), int(parts[1])
        if not 1 <= servo <= len(self.angles):
            print(f"Servo must be between 1 and {len(self.angles)}")
            return True
        if not 0 <= angle <= 180:
            print("Angle must be between 0 and 180")
            return True

        channel = servo - 1
        leg, joint = divmod(channel, len(JOINTS))
        lo, hi = self.ranges[joint]
        if not lo <= angle <= hi:
            print(f"Warning: outside the {JOINTS[joint]} safety range "
                  f"[{lo:.0f}, {hi:.0f}], the firmware would clamp it while walking")
        self.set_angle(channel, angle)

        offset = angle - self.reference[channel]
        print(f"Servo {servo} ({LEGS[leg]} {JOINTS[joint]}) at {angle}, "
              f"{offset:+.1f} from the reference: saving on quit sets "
              f"hardware.trim.{LEGS[leg]}[{joint}] = {self.trim[channel] + offset:g}")
        return True

    def run(self) -> bool:
        """Interactive session; True if it ended with a quit that should save"""
        print(__doc__)
        input("\nPress Enter to energize and move to the reference pose... ")
        try:
            self.reset()
            self.print_table()
            while True:
                try:
                    line = input("\n> ")
                except EOFError:
                    break
                if not self.handle(line.strip().lower()):
                    break
        except KeyboardInterrupt:
            print("\nAborted, nothing will be saved.")
            return False
        finally:
            self.release()
            self.client.close()
            print("Servos released.")
        return True


def _parse_channels(channels, legs):
    if channels:
        return sorted(channels)
    if legs:
        idx = {name: i for i, name in enumerate(LEGS)}
        out = []
        for leg in legs:
            base = idx[leg] * len(JOINTS)
            out += [base + j for j in range(len(JOINTS))]
        return sorted(out)
    return list(range(len(LEGS) * len(JOINTS)))  # all 18


def main() -> None:

    ap = argparse.ArgumentParser(description="Per-servo pulse-width calibration.")
    ap.add_argument(
        "--config",
        "-c",
        default=str(DEFAULT_CONFIG),
        help="config.yml (read for serial/pins, written with results).",
    )
    ap.add_argument("--port", "-p", default=None, help="serial port override.")
    ap.add_argument(
        "--channels",
        "-C",
        type=int,
        nargs="+",
        default=None,
        help="only calibrate these logical channels (0..17).",
    )
    ap.add_argument(
        "--legs",
        "-L",
        nargs="+",
        default=None,
        help=f"only calibrate these legs ({', '.join(LEGS)}).",
    )
    ap.add_argument("--small-step", "-s", type=int, default=5)
    ap.add_argument("--large-step", "-l", type=int, default=50)
    ap.add_argument("--min-pulse", "-u", type=int, default=500)
    ap.add_argument("--max-pulse", "-U", type=int, default=2500)
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument(
        "--pose",
        action="store_true",
        help="hold the right-angle reference pose to check the trims.",
    )
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml_rt.load(f)
    serial = cfg.get("serial", {})
    port = args.port or serial.get("port")
    baud = int(serial.get("baud", 921600))
    client = HexapodClient(SerialTransport(port, baud))

    # Make logical channels map to the right physical servos before jogging
    hardware = cfg.get("hardware", {})
    if "pins" in hardware:
        client.set_pins([int(v) for leg in LEGS for v in hardware["pins"][leg]])

    if args.pose:
        checker = PoseChecker(client, cfg)
        trims = checker.corrections() if checker.run() else {}
        if trims and not args.no_save:
            backup = save_trims(args.config, trims)
            for channel, value in sorted(trims.items()):
                leg, joint = divmod(channel, len(JOINTS))
                print(f"  {LEGS[leg]} {JOINTS[joint]}: trim {checker.trim[channel]:g} -> {value:g}")
            print(f"Saved {len(trims)} trim(s) to {args.config} (backup: {backup}).")
        elif trims:
            print("--no-save set; trims were not written.")
        else:
            print("No servo moved from its reference; nothing to save.")
        return

    channels = _parse_channels(args.channels, args.legs)
    cal = ServoCalibrator(
        client,
        channels,
        small_step=args.small_step,
        large_step=args.large_step,
        min_pulse=args.min_pulse,
        max_pulse=args.max_pulse,
    )
    cal.run()

    if cal.results and not args.no_save:
        backup = save_results(args.config, cal.results)
        print(f"Saved {len(cal.results)} servo(s) to {args.config} (backup: {backup}).")
    elif cal.results:
        print("--no-save set; results were not written.")
    else:
        print("No servos were fully calibrated; nothing to save.")


if __name__ == "__main__":
    main()
