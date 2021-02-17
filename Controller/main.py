from DronePortController import DronePortController

import signal
import sys
import time

def main():
    DP = DronePortController()
    print(DP._getUAVPosition())
