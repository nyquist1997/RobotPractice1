from Robot import Robot
import numpy as np
import until
import time
import pandas as pd
import math
from scipy.interpolate import CubicSpline
import sys
# Write the path to the IK folder here
# For example, the IK folder is in your Documents folder
import getpass

sys.path.append(f"C:\\Users\\Fanyi\\Desktop\\coppy")
import ikt

if __name__ == "__main__":
    r = Robot(com='COM4', baud=250000)
    r.connect()
    while True:
        print(r.syncFeedback())
        time.sleep(1.00)
        