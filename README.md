# Hexapod Controller

This repository  contains the code for a python controller for my Hexapod robot.

For a complete overview of the project refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod.git). Take also a look to the [Firmware repository](https://github.com/ggldnl/Hexapod-Operator.git). 

Below, you will find instructions on how to build and deploy the code.

## 🛠️ Setup

Before you start, make sure you completed the [setup of the Servo2040](https://github.com/ggldnl/Hexapod-Operator.git).

### Install miniforge

Anaconda lets you easily manage python environments in the same system and has also a package manager called Conda that can be used to download and maintain dependencies.
Unfortunately, this is not supported by Raspberry Pis. We can use Miniforge instead, which can provide the same core features. It can be installed by downloading an installer script from the [GitHub repository](https://github.com/conda-forge/miniforge) that will take care of everything. If you are using the Raspberry Pi 5 as I am, select the arm64 version, download the installer and execute it.

```bash
cd /tmp
wget https://github.com/conda-forge/miniforge/releases/download/24.11.0-0/Miniforge3-24.11.0-0-Linux-aarch64.sh
bash Miniforge3-24.11.0-0-Linux-aarch64.sh
# Follow the instructions
```

Add miniforge to the system path:
```bash
echo 'export PATH=~/miniforge3/bin:$PATH' >> .bashrc
source .bashrc
```

To check conda is properly installed:

```bash
which conda
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

Run the main script on the Raspberry Pi. The Controller will continuously generate and send messages to the Servo2040, while streaming the robot status on a websocket.

```bash
python main.py
```

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.
