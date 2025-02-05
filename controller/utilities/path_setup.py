import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(SCRIPT_DIR)

# Add the package folder to the python path so that we can find imports
if PARENT_DIR not in sys.path:
    sys.path.append(PARENT_DIR)
