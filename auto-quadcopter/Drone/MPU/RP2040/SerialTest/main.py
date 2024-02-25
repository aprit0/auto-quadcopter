# import supervisor
# import sys
# import time
# s = ''
# new_data = False
# end_char = "\n"

# while True:
#     n = supervisor.runtime.serial_bytes_available
#     # print(n)
#     if n > 0:
#         if len(s) == 0:
#             t_0 = time.monotonic()
#         s += sys.stdin.read(n)
#         if s.endswith(end_char):
#             new_data = True
#         print(s)
    

#     if new_data:
#         print("NEW DATA: ",s, time.monotonic() - t_0)
#         t_0 = 0
#         s = ''
#         new_data = False
import board
import busio
import time
uart_out = busio.UART(board.GP4, board.GP5, baudrate=115200)
uart_in = busio.UART(board.GP16, board.GP17, baudrate=115200)
message_started = False
message = []
last_message = ""

while True:
    gps = uart_in.readline()
    gps_str = str(gps)
    # byte_read = uart_out.read(1)  # read up to 32 bytes
    # if byte_read:
    #     print(byte_read)  # this is a bytearray type

    # if byte_read == b"<":
    #     t_0 = time.monotonic()
    #     message_started = True
    #     message = []
    #     continue
    # elif byte_read == b">":
    #     message_started = False
    #     print(len(message), message, time.monotonic() - t_0)
    #     last_message = "".join(message)
    # if message_started:
    #     message.append(chr(byte_read[0]))
    # if not message_started:
    uart_out.write(bytes(f"<{gps_str}>", "ascii"))
