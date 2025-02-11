# Hexapod Controller

My hexapod robot consists of two parts: a Controller and an Operator. The Controller is responsible for generating commands, which are then sent to the Operator for execution. I used a Raspberry Pi 5 as Controller  and a Servo2040 board as Operator, for handling low-level control of the hexapod's servos.
This repository contains the firmware for the Raspberry Pi.

For a complete overview of the project refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod.git). Take also a look to the [repository containing the Operator's code](https://github.com/ggldnl/Hexapod-Operator.git). 

Below, you will find instructions on how to build and deploy the code.

## 🛠️ Setup

Before you start, make sure you completed the setup of the [Operator](https://github.com/ggldnl/Hexapod-Operator.git).

### Install miniforge

Anaconda lets you easily manage python environments in the same system and has also a package manager called Conda that can be used to download and maintain dependencies.
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

## 🚀 Delpoy

Run the main script on the Controller. The code will continuously generate and send messages to the Operator.

```bash
python controller/main.py
```

The main script contains a list of predefined actions to showcase the capabilities of the robot. Old actions can be modified and new actions can be added easily:

```python
# Stand (2 seconds)
controller.stand(2)

# Move the body
controller.set_body_pose(2, body_orientation=np.array([0, np.deg2rad(10), np.deg2rad(10)]))
controller.set_body_pose(2, body_orientation=np.array([0, np.deg2rad(-10), np.deg2rad(-10)]))
controller.set_body_pose(2, body_orientation=np.array([0, 0, 0]))

# Raise a leg by a certain amount
leg_index = 0
current_leg_position = controller.get_last_state_in_queue().legs_positions[leg_index]
x, y, z = current_leg_position
amount = 50
controller.set_legs_positions(1, np.array([x, y, z + amount]), indices=[leg_index])

# Sit
controller.sit(2)
```

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.
