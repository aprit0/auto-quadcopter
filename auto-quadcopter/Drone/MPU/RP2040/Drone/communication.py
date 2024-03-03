import board
import busio
import asyncio

# <TYPE,KEY:VALUE,KEY:VALUE> ... <TYPE,DATA,DATA>
message_mapping = {
    # Setters
    "A": "set_setpoints",
    "B": "set_pid_setpoints",
    "C": "set_arm",
    "D": "write_config",
    # Getters
    "Z": "get_pid_setpoints",
    "Y": "get_pose",
}
inv_message_mapping = {value: key for [key, value] in message_mapping.items()}

class PICO2PI:
    def __init__(self, baudrate=115200) -> None:
        self.uart = busio.UART(board.GP4, board.GP5, baudrate=baudrate)
        self.sreader = asyncio.StreamReader(self.uart)
        self.str_out = ""
        self.message_queue = []

    async def read_serial(self):
        while True:
            bytes_out = await self.sreader.readline()
            str_out = bytes_out.decode().rstrip("\n")
            self.message_queue.append(str_out)

    def get_message(self, msg):
        if "<" == msg[0] and ">" == msg[-1]:
            # Valid message
            msg = msg[1:-1]
            msg_type, data = msg.split(",")
            data = [data] if type(data) != type([]) else data
            print(msg_type, data) 
            msg_data = {i.split(":")[0]: i.split(":")[1] for i in data}
            return msg_type, msg_data
        print("Invalid_msg", msg)
        return None, None

    
    def process_msg(self, _msg_type, _msg):
        if _msg_type in message_mapping:
            call_func = getattr(self, message_mapping[_msg_type])
            call_func(_msg)

    def write_msg(self, _msg_func, _msg_dict):
        data_str = ",".join([f"{key}:{value}" for [key, value] in _msg_dict.items()])
        type_str = inv_message_mapping[_msg_func]
        out_str = f"<{type_str},{data_str}"
        self.uart.write(bytes(out_str, "ascii"))
    
    async def update_state(self):
        while True:
            for msg in self.message_queue:
                msg_type, msg = self.get_message(msg)
                if type(msg_type) != None:
                    self.process_msg(msg_type, msg)
            await asyncio.sleep(0)

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