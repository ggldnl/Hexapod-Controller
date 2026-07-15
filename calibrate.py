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

Run:  python3 calibrate.py            [--port /dev/ttyAMA0] [--channels 0 1 2]
"""

from __future__ import annotations

import argparse
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

    def flow(values):
        seq = CommentedSeq(values)
        seq.fa.set_flow_style()
        return seq

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
