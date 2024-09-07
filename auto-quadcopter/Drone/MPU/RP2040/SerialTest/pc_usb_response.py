import serial
print(serial.__file__) 
import time
import asyncio

serialPort = serial.Serial(
    port="COM3", baudrate=230400#, bytesize=8, timeout=0, stopbits=serial.STOPBITS_ONE
)
t_0 = time.time()
msg_queue = []
async def read_usb_serial():
    while True:
        serialString = serialPort.read(serialPort.in_waiting)
        if serialString:
            s = serialString.decode("Ascii")
            msg_queue.append(s)
            print("READ: ", time.time() - t_0, serialString, s, len(serialString))
        await asyncio.sleep(0)

async def write_usb_serial():
    count = 0
    while True:
        out = "abc"
        message = f"<Message {count}: {out}>\n"
        serialPort.write(message.encode())
        print("WRITE: ", message)
        t_0 = time.time()
        count += 1
        await asyncio.sleep(1)  # Adjust the delay as needed

async def main():
    await asyncio.gather(read_usb_serial(), write_usb_serial())

# Run the main function
asyncio.run(main())