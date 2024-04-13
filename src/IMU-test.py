#Need to install following package
# pip3 install adafruit-circuitpython-lsm6ds
#NL has found that this is done within a virtual environment
#Without this package/library, this current code will not run



import time  # Import the time module to enable time delays
import board  # Import board module for board pin definitions
import busio  # Import busio for using I2C communication
from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC  # Import the LSM6DS3TRC class from the adafruit_lsm6ds library

# Initialize I2C communication
# board.SCL and board.SDA are default I2C clock and data lines for the Raspberry Pi
i2c = busio.I2C(board.SCL, board.SDA)

# Create an instance of the LSM6DS3TRC sensor
# Pass the I2C object created above to the sensor, allowing it to communicate over I2C
sensor = LSM6DS3TRC(i2c)

# Enter an infinite loop
while True:
    # Read the acceleration from the sensor and print it
    # sensor.acceleration returns a 3-tuple (x, y, z) representing acceleration values in m/s^2
    # The '%.2f' format specifier is used to format the numbers to two decimal places
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % sensor.acceleration)
    
    # Read the gyroscopic values from the sensor and print them
    # sensor.gyro returns a 3-tuple (x, y, z) representing gyroscopic values in radians/s
    print("Gyro: X:%.2f, Y: %.2f, Z: %.2f radians/s" % sensor.gyro)
    
    # Print an empty line for better readability in the output
    print("")
    
    # Pause for 0.5 seconds before the next read
    # This slows down the loop and makes the output readable
    time.sleep(0.5)
