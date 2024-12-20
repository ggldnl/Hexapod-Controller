# Hexapod Controller

My hexapod robot consists of two parts: a Controller and an Operator. The Controller is responsible for generating commands, which are then sent to the Operator for execution. I used a Raspberry Pi 5 as Controller  and a Servo2040 board as Operator, for handling low-level control of the hexapod's servos.
This repository contains the firmware for the Raspberry Pi.

For a complete overview of the project refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod.git). Take also a look to the [repository containing the Operator's code](https://github.com/ggldnl/Hexapod-Operator.git). 

Below, you will find instructions on how to build and deploy the code.

## 🛠️ Build and deployment

Before you start, make sure you completed the setup of the [Operator](https://github.com/ggldnl/Hexapod-Operator.git).

### 📦 Install miniforge

Anaconda let's you easily manage python environments in the same system and has also a package manager called Conda that can be used to download and maintain dependencies.
Unfortunately, this is not supported by Raspberry Pis. We can use Miniforge instead, which can provide the same core features. It can be installed by downloading an installer script from the [GitHub repository](https://github.com/conda-forge/miniforge) that will take care of everything. If you are using the Raspberry Pi 5 as I am, select the arm64 version, download the installer and execute it. For this to work you must also use a 64-bit OS. 

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

### 🚀 Delpoy

TODO
