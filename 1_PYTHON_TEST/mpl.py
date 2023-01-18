# SPDX-FileCopyrightText: 2019 Tony DiCola for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple demo of the MPL3115A2 sensor.
# Will read the pressure and temperature and print them out every second.
import time
import numpy as np
import board
import adafruit_mpl3115a2


# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialize the MPL3115A2.
sensor = adafruit_mpl3115a2.MPL3115A2(i2c)
# Alternatively you can specify a different I2C address for the device:
# sensor = adafruit_mpl3115a2.MPL3115A2(i2c, address=0x10)

# You can configure the pressure at sealevel to get better altitude estimates.
# This value has to be looked up from your local weather forecast or meteorological
# reports.  It will change day by day and even hour by hour with weather
# changes.  Remember altitude estimation from barometric pressure is not exact!
# Set this to a value in pascals:
sensor.sealevel_pressure = 101960
a = []
b = []
for i in range(10):
    t_0 = time.time()
    b.append(sensor.altitude)
    print(i, time.time() - t_0)
print(max(b) - min(b))
for i in range(10):
    a.append(sensor.altitude)
offset = np.mean(a)
print(max(a) - min(a))
print(offset, a)

# Main loop to read the sensor values and print them every second.
while True:
    # pressure = sensor.pressure
    # print("Pressure: {0:0.3f} pascals".format(pressure))
    altitude = sensor.altitude
    print("Altitude: {:.3f} meters, {:.3f}".format(altitude, altitude - offset))
    # temperature = sensor.temperature
    # print("Temperature: {0:0.3f} degrees Celsius".format(temperature))
