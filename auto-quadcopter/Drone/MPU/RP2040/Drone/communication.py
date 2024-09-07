import board
import busio
import asyncio
import gc
import time
import digitalio
import sys
# <TYPE,KEY:VALUE,KEY:VALUE> ... <TYPE,DATA,DATA>
# <0x0,0:[('I', '2.0'), ('P', '1.0'), ('D', '3.0')]> 
# <B,P:1.0,I:2.0,D:3.0>
message_mapping = {
    # Setters
    "A": "set_setpoints",
    "B": "set_pid_setpoints",
    "C": "set_arm",
    "D": "write_config", # NEED WORKAROUND FOR READONLY
    # Getters
    "Z": "get_pid_setpoints",
    "Y": "get_pose",
    # Debug
    "0x0": "set_log"
}
inv_message_mapping = {value: key for [key, value] in message_mapping.items()}

class PICO2PI:
    def __init__(self, baudrate=230400) -> None:
        self.LED = digitalio.DigitalInOut(board.LED)
        self.LED.direction = digitalio.Direction.OUTPUT
        self.LED.value = False
        # time.sleep(3)
        # self.uart = busio.UART(board.GP4, board.GP5, baudrate=baudrate, timeout=0)
        self.str_out = ""
        self.recv_message_queue = []
        self.send_message_queue = []
        self.buffer = ""
        self.uart_msg_lost = 0
        self.set_log("MAIN DRONE BEGIN")

    def set_log(self, msg, level=0):
        self.write_msg("set_log", {level: msg})

    def check_string(self, str_in):
        self.buffer += str_in
        possible_msg = [i.split(">")[0] for i in self.buffer.split("<")]
        # print("POS", possible_msg, self.buffer)
        while possible_msg:
            msg = possible_msg.pop(0)
            # print("MSG", msg)
            if msg and "," in msg and ":" in msg:
                str_out = f"<{msg}>"
                if str_out not in self.recv_message_queue:
                    self.recv_message_queue.append(str_out)
                self.buffer = self.buffer.replace(str_out, "")
            else:
                # self.set_log(f"INVALID MSG {msg}, BUFFER: {self.buffer}")
                pass
        if len(self.buffer) > 50:
            self.buffer = ""
        return 0

    def get_message(self, msg):
        error = ""
        if "<" == msg[0] and ">" == msg[-1]:
            # Valid message
            msg = msg[1:-1]
            msg_type = msg.split(",")[0]
            data = msg.split(",")[1:]
            data = [data] if type(data) != type([]) else data
            try:
                msg_data = {i.split(":")[0]: i.split(":")[1] for i in data}
                return msg_type, msg_data
            except Exception as e:
                error += str(e)
                self.set_log(f"Error get_message, msg {msg} data, {data}, error {error}")
        # print("Invalid_msg", msg, error)
        self.set_log(f"Invalid get_message {msg, error}")
        return None, None

    
    def process_msg(self, _msg_type, _msg):
        if _msg_type in message_mapping:
            # print("Processed", _msg_type, _msg)
            call_func = getattr(self, message_mapping[_msg_type])
            call_func(_msg)
        else:
            self.set_log(f"Failed process_msg {_msg_type, _msg}")
            # print("Failed process_msg", _msg_type, _msg)
            pass

    def write_msg(self, _msg_func, _msg_dict):
        data_str = ",".join([f"{key}:{value}" for [key, value] in _msg_dict.items()])
        type_str = inv_message_mapping[_msg_func]
        out_str = f"<{type_str},{data_str}>"
        self.send_message_queue.append(out_str)
    
    async def update_state(self):
        while True:
            # print("0update_state")
            while self.recv_message_queue:
                msg = self.recv_message_queue.pop(0)
                msg_type, msg = self.get_message(msg)
                if type(msg_type) != None:
                    self.process_msg(msg_type, msg)
                else:
                    # print("Failed update msg", msg_type)
                    pass
            # self.set_log(f"update_state: {self.recv_message_queue}")
            await asyncio.sleep(0)
    
    async def sender(self):
        while True:
            if self.send_message_queue:
                while self.send_message_queue:
                    msg = self.send_message_queue.pop(0)
                    print(msg, end='')  # Use end='' to avoid double newlines
                    self.LED.value = False if self.LED.value == True else True
            await asyncio.sleep(0)

    async def receiver(self):
        reader = asyncio.StreamReader(sys.stdin)
        while True:
            bytes_out = await reader.readline()  # Read a line from USB serial
            if bytes_out:
                try:
                    if type(bytes_out) != type(''):
                        str_out = bytes_out.decode("ascii") 
                    else:
                        str_out = bytes_out
                    status = self.check_string(str_out)
                except Exception as e:
                    self.set_log(f"Error receiver {bytes_out, e}")
            await asyncio.sleep(0)



if __name__ == "__main__":
    pi = PICO2PI()
    loop = asyncio.get_event_loop()
    loop.create_task(pi.read_serial())
    loop.create_task(pi.update_state())
    loop.run_forever()