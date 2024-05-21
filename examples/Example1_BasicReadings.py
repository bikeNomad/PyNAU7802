""" Example1_BasicReadings.py
  Adapted from the SparkFun Qwiic Scale NAU7802 Arduino Library
  https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library

  Translated from the Example1_BasicReadings.ino example file
  https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library/tree/master/examples/Example1_BasicReadings
  
  Simple example to read the weight from the NAU7802 with MicroPython.
"""

from machine import I2C
from PyNAU7802 import NAU7802


def setup() -> None:
    print("Qwiic Scale Example")

    if not myScale.begin():
        print("Scale not detected. Please check wiring. Freezing...")
        exit(1)

    print("Scale detected!")


def loop() -> None:
    if myScale.available():
        currentReading = myScale.getReading()
        print(f"Reading: {currentReading}")


if __name__ == "__main__":
    i2c = I2C(0, I2C.MASTER, baudrate=100000)  # Create I2C instance

    myScale = NAU7802()  # Create instance of the NAU7802 class

    setup()

    while True:
        loop()
