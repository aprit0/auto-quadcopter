import board
import busio
import asyncio
import gc
import time
import digitalio
# <TYPE,KEY:VALUE,KEY:VALUE> ... <TYPE,DATA,DATA>
message_mapping = {
    # Setters
    "A": "set_setpoints",
    "B": "set_pid_setpoints",
    "C": "set_arm",
    "D": "write_config", # NEED WORKAROUND FOR READONLY
    # Getters
    "Z": "get_pid_setpoints",
    "Y": "get_pose",
}
inv_message_mapping = {value: key for [key, value] in message_mapping.items()}

class PICO2PI:
    def __init__(self, baudrate=460800) -> None:
        # TX = digitalio.DigitalInOut(board.GP4)
        # TX.switch_to_input(pull=digitalio.Pull.DOWN)
        # RX = digitalio.DigitalInOut(board.GP5)
        # RX.switch_to_input(pull=digitalio.Pull.DOWN)
        self.LED = digitalio.DigitalInOut(board.LED)
        self.LED.direction = digitalio.Direction.OUTPUT
        self.LED.value = False
        # time.sleep(120)
        # TX.deinit()
        # RX.deinit()
        # time.sleep(1)
        self.uart = busio.UART(board.GP4, board.GP5, baudrate=baudrate)
        self.str_out = ""
        self.message_queue = []
        self.buffer = ["", 0]
        self.uart_msg_lost = 0

    def check_string(self, str_in, is_buffer=False):
        start_idx = [idx for idx, i in enumerate(str_in) if i == "<"]
        end_idx = [idx for idx, i in enumerate(str_in) if i == ">"]
        if start_idx and end_idx and end_idx[-1] > start_idx[-1]: 
            str_out = str_in[start_idx[-1]:end_idx[-1]+1]
            self.buffer = ["", 0]
            # print("VALID READ: ", str_out)
            self.message_queue.append(str_out)
            return 1
        elif self.buffer[1] <= 2 and not is_buffer:
            self.buffer[0] += str_in
            self.buffer[1] += 1
            print("0INVALID READ: ", str_in, self.buffer)
        elif self.buffer[1] > 2 and not is_buffer:
            self.buffer = ["", 0]
            print("1INVALID READ: ", str_in)
        return 0

    async def read_serial(self):
        reader = asyncio.StreamReader(self.uart)
        while True:
            self.LED.value = False if self.LED.value == True else True
            # print("0read_serial", self.LED.value)
            bytes_out = await reader.readline()
            gc.collect()
            # if bytes_out:
            # print(1, bytes_out)
            str_out = str(bytes_out).lstrip("b''").rstrip("\n")
            if str_out:
                status = self.check_string(str_out)
                if not status and self.buffer[0]:
                    print("CHECKING BUFFER: ", str_out)
                    status = self.check_string(self.buffer[0])

    def get_message(self, msg):
        error = ""
        if "<" == msg[0] and ">" == msg[-1]:
            # Valid message
            # print("MSG: ",msg)
            msg = msg[1:-1]
            msg_type = msg.split(",")[0]
            data = msg.split(",")[1:]
            data = [data] if type(data) != type([]) else data
            try:
                msg_data = {i.split(":")[0]: i.split(":")[1] for i in data}
                return msg_type, msg_data
            except Exception as e:
                error += str(e)
                # print("AAAAAAAAAAAAAAAAAAAAAAAA: ", error)
        print("Invalid_msg", msg, error)
        return None, None

    
    def process_msg(self, _msg_type, _msg):
        if _msg_type in message_mapping:
            call_func = getattr(self, message_mapping[_msg_type])
            call_func(_msg)
        else:
            print("Failed process_msg", _msg_type, _msg)

    def write_msg(self, _msg_func, _msg_dict):
        data_str = ",".join([f"{key}:{value}" for [key, value] in _msg_dict.items()])
        type_str = inv_message_mapping[_msg_func]
        out_str = f"<{type_str},{data_str}>\n"
        self.uart.write(bytes(out_str, "ascii"))
    
    async def update_state(self):
        while True:
            # print("0update_state")
            while self.message_queue:
                msg = self.message_queue.pop(0)
                msg_type, msg = self.get_message(msg)
                if type(msg_type) != None:
                    self.process_msg(msg_type, msg)
                else:
                    print("Failed update msg", msg_type)
            # print("1update_state")
            await asyncio.sleep(0.1)

    # async def main(self):
    #     while True:
    #         while self.message_queue:
    #             print(len(self.message_queue))
    #             out = self.message_queue.pop(0)
    #             print(out)
    #             msg, data = self.get_message(out)
    #             print(msg, data)

    #         await asyncio.sleep(0)



if __name__ == "__main__":
    pi = PICO2PI()
    loop = asyncio.get_event_loop()
    loop.create_task(pi.read_serial())
    loop.create_task(pi.update_state())
    loop.run_forever()