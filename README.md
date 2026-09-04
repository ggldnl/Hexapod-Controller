# Hexapod Controller

This repository contains the Pi-side Python controller for my Hexapod robot.

The Servo2040 board runs the whole control loop (finite-state machine, gait generator and inverse kinematics). This controller opens the serial link, provisions the board with the robot's configuration at connect, then streams high-level setpoints (a velocity, a gait, a body pose) and reads telemetry back. This way we won't ever have to deal with framing and the transport layer. The wire protocol is a hand-mirror of `src/communication/protocol.hpp` in the firmware, checked byte-for-byte against the C++ encoder in the tests so the two stay in lockstep.

For a complete overview of the project refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod.git). Take also a look at the [Firmware repository](https://github.com/ggldnl/Hexapod-Firmware.git) and the [Simulation repository](https://github.com/ggldnl/Hexapod-Simulation.git), which drives this same controller against the host-compiled firmware.

## 🗂️ Layout

```
hexapod/
  protocol.py       # opcodes, enums, struct payload formats (mirror of the firmware)
  framing.py        # CRC-16/CCITT, encode_frame, FrameParser
  transport.py      # Transport interface + SerialTransport (pyserial)
  client.py         # HexapodClient, high-level API
  config/
    config.yml      # the robot description pushed to the board at connect
calibrate.py        # offline per-servo pulse-width calibration tool
tests/              # pure-Python tests (no hardware, no pyserial)
```

## 🛠️ Setup

Before you start, make sure you completed the [setup of the Servo2040](https://github.com/ggldnl/Hexapod-Firmware.git). The controller can be used with any OS, but you might want to switch to a [ROS2 controller](https://github.com/ggldnl/Hexapod-ROS-Python.git) in the future, so I suggest Ubuntu 24.04 for better ROS2 compatibility.

### Enable serial interface

We need to enable the serial interface to let the Pi communicate with the Servo2040. The following might vary based on the OS and its version.

- On Raspbian:

    ```bash
    sudo raspi-config
    ```

    Navigate to `Interface Options` > `Serial`, disable the login shell, enable the hardware port, then reboot.

- On Ubuntu 24.04 the procedure is more involved:

  - Edit `/boot/firmware/config.txt`:

    ```bash
    # Enable UART
    enable_uart=1
    ```
    On the Pi 5, Bluetooth no longer shares the main UART (unlike the Pi 4), so disabling BT is usually not necessary.

  - By default, Ubuntu uses the serial port as a console. Disable it:

    ```bash
    sudo systemctl disable serial-getty@ttyAMA0.service
    sudo systemctl stop serial-getty@ttyAMA0.service
    ```

  - Remove the console references from the kernel cmdline:

    ```bash
    sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.bak
    sudo sed -i -E 's/\bconsole=(serial[0-9]+|ttyAMA[0-9]+|ttyS[0-9]+)[^[:space:]]*[[:space:]]*//g' /boot/firmware/cmdline.txt
    # or do it manually: sudo nano /boot/firmware/cmdline.txt
    ```

  - Add your user to the `dialout` group:

    ```bash
    sudo usermod -aG dialout $USER
    ```

  - Reboot:

    ```bash
    sudo reboot
    ```

### Install

The controller is a standard Python package. Clone the repo, create a conda environment and install it. Two optional extras are available: `serial` pulls in pyserial for talking to real hardware, and `calibrate` pulls in ruamel.yaml for the calibration tool. Install both to get everything:

```bash
git clone https://github.com/ggldnl/Hexapod-Controller.git
cd Hexapod-Controller
pip install ".[serial,calibrate]"
```

The runtime client itself needs neither extra: `pip install .` is enough to import `hexapod` and drive the board, as long as the transport you build is not the real serial one.

For development, install it in editable mode instead so your changes are picked up without reinstalling:

```bash
pip install -e ".[serial,calibrate]"
```

The robot configuration in `hexapod/config/config.yml` ships with the package, so `connect()` works out of the box against the packaged default. Point it at your own file whenever you need to.

## 🚀 Usage

Open the link and provision the board from the config, then drive it:

```python
from hexapod import connect, GaitId

bot = connect()                 # opens the serial port and provisions from config.yml
bot.enable()                    # stand up
bot.set_gait(GaitId.TRIPOD)
bot.set_velocity(120, 0, 0)     # vx, vy in mm/s, yaw in deg/s
print(bot.get_telemetry())      # state, odometry, voltage, current
bot.shutdown()                  # sit down
```

`connect()` reads the serial port and baud from the config, provisions every section in wire order, and returns a ready client. Pass a different config file or port when you need to, and skip provisioning if the board is already set up:

```python
bot = connect(config="my_robot.yml", port="/dev/ttyAMA0", baud=921600)
bot = connect(provision=False)
```

If you only want the transport without the config machinery, build the client directly:

```python
from hexapod import HexapodClient, SerialTransport

bot = HexapodClient(SerialTransport("/dev/ttyAMA0", 921600))
```

In simulation the same client is driven through an in-process transport to the host-compiled firmware instead of a serial port (see the Simulation repository).

## 🎛️ Calibration

`calibrate.py` is an offline tool that jogs each servo to find its min, center and max pulse widths and writes the result back into `config.yml` under `hardware.pulses`. It writes with ruamel.yaml to preserve the file's comments and layout, so it needs the `serial` and `calibrate` extras installed (see the setup above). Run it with the board connected and the robot de-energized:

```bash
python calibrate.py                     # uses the packaged config.yml
python calibrate.py --config my_robot.yml --port /dev/ttyAMA0
```

## 🧪 Tests

```bash
./tests/run.sh        # pure Python, no hardware and no pyserial needed
```

The suite exercises the framing (CRC and golden frames against the firmware encoder) and the client (fire-and-forget setpoints, queries, error handling) against a fake board that speaks the real wire protocol.

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.
