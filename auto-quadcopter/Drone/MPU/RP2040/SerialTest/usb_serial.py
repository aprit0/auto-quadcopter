import asyncio
import time
import busio
import os, sys

msg_queue = []
async def read_usb_serial():
    while True:
        line = sys.stdin.readline()  # Read a line from USB serial
        if line:
            msg_queue.append(line)
        await asyncio.sleep(0)

async def write_usb_serial():
    count = 0
    while True:
        if msg_queue:
            out = msg_queue.pop(0)
            message = f"<Message {count}: {out}>"
            print(message, end='')  # Use end='' to avoid double newlines
            count += 1
        await asyncio.sleep(0)  # Adjust the delay as needed

async def main():
    await asyncio.gather(read_usb_serial(), write_usb_serial())

# Run the main function
asyncio.run(main())