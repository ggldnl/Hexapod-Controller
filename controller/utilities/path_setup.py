import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UTILITIES_DIR = os.path.dirname(SCRIPT_DIR)
ROOT_DIR = os.path.dirname(UTILITIES_DIR)

# Add the package folder to the python path so that we can find imports
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)
