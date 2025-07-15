#=========IMPORTS=========
from adafruit_servokit import ServoKit
from simple_pid import PID
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time
import math
import pygame
import sys
import matplotlib.pyplot as plt
import numpy as np
import io
from PIL import Image
from gpiozero import LED
import os
import datetime
import csv
#=========================

## calibration

servoPin = 4
s = ServoKit(channels=16)














#========FUNCTIONS========
def servoCalibrationRaspberry()
#=========================