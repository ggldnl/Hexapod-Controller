# Hexapod Controller

This repository  contains the code for a stand-alone python controller for my Hexapod robot. The controller is intended to run on a Raspberry Pi. I'm using an RPI 5.

For a complete overview of the project refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod.git). Take also a look to the [Firmware repository](https://github.com/ggldnl/Hexapod-Firmware.git). 

Below, you will find instructions on how to build and deploy the code.

## 🛠️ Setup

Before you start, make sure you completed the [setup of the Servo2040](https://github.com/ggldnl/Hexapod-Firmware.git).
The stand-alone controller can be used with any OS, but you might want to switch to a ROS2 controller in the future, 
so I suggest using Ubuntu 24.04 for better ROS2 compatibility.

### Enable serial interface

We need to enable the serial interface to let the RPI communicate with the Servo2040. 
The following might vary based on what OS you use and its version.

- On Raspbian:

    ```bash
    sudo raspi-config
    ```
  
    Navigate to `Interface Options` > `Serial` > disable the login shell, enable the hardware port.
    Reboot.

- On Ubuntu 24.04 the procedure is more involved:

  - Edit `/boot/firmware/config.txt`:

    ```bash
    # Enable UART
    enable_uart=1
    
    # Disable bluetooth
    # dtoverlay=disable-bt
    ```
    On the Pi 5, Bluetooth no longer shares the main UART (unlike Pi 4), so disabling BT is usually not necessary.

  - By default, Ubuntu uses the serial port as a console. Disable it:
    
    ```bash
    sudo systemctl disable serial-getty@ttyAMA0.service
    sudo systemctl stop serial-getty@ttyAMA0.service 
    ```

  - Remove the console references from the kernel cmdline:
  
    ```bash
    sudo nano /boot/firmware/cmdline.txt
    ```
  
  - Add your user to the `dialout` group:

    ```bash
    sudo usermod -aG dialout $USER
    ```
    
  - Reboot
    
    ```bash
    sudo reboot
    ```

### Install miniforge

Anaconda lets you easily manage python environments in the same system and has also a package manager called Conda that can be used to download and maintain dependencies.
Unfortunately, this is not supported by Raspberry Pis. We can use Miniforge instead, which can provide the same core features. It can be installed by downloading an installer script from the [GitHub repository](https://github.com/conda-forge/miniforge) that will take care of everything. If you are using the Raspberry Pi 5 as I am, select the arm64 version, download the installer and execute it.

```bash
cd /tmp
wget https://github.com/conda-forge/miniforge/releases/download/24.11.0-0/Miniforge3-24.11.0-0-Linux-aarch64.sh
bash Miniforge3-24.11.0-0-Linux-aarch64.sh
# Follow the instructions
```

### Create an environment

Once you have Miniforge installed, you can clone this repo, create and activate the environment:

```bash
git clone https://github.com/ggldnl/Hexapod-Controller.git
cd Hexapod-Controller
conda env create -f environment.yml
conda activate hexapod
```

After the installation, from within the `hexapod` environment, you will be able to do something like this:

```python
from controller import HexapodController
```

## 🚀 Delpoy

Run the main script on the Raspberry Pi. The Controller will continuously generate and send messages to the Servo2040. You can open the script to check the available command line arguments. Keep in mind that this is only a sample script that executes predefined commands; in order to control the robot properly, use the [ROS2 controller node](https://github.com/ggldnl/Hexapod-ROS-Python.git) (it wraps the same controller code).

```bash
python main.py
```

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.
