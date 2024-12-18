from interface import Interface
from hexapod import Hexapod


class Controller:

    def __init__(self, hexapod, interface):
        self.hexapod = hexapod
        self.interface = interface

    def __enter__(self):
        self.interface.open()

    def __exit__(self):
        self.interface.close()

    def get_all(self):
        return [0] * 18

    def set_all(self, angles):
        assert len(angles) == 18
        self.interface.set_angles(range(18), angles)


if __name__ == '__main__':

    from interface import Interface
    from pathlib import Path
    import json

    # Read the config
    path = Path('../config/hexapod.json')
    with open(path) as f:
        config = json.load(f)

    # Create a Hexapod object
    name = 'hexapod'
    hexapod = Hexapod(config[name])

    # Create the serial interfacing class
    dev='/dev/ttyACM0'
    baud=115200
    interface = Interface(dev=dev, baud=baud)

    # Create the Controller
    controller = Controller(hexapod, interface)

    with controller as c:

        print(f'Connected to hexapod [{name}]')
