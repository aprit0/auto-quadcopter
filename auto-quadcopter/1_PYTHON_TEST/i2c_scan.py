import board
import busio

freq = 400000
i2c = busio.I2C(board.SCL, board.SDA, frequency=freq)
print([hex(i) for i in i2c.scan()])

