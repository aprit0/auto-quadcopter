import serial
import time


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
    # Debug
    "0x0": "get_log"
}
inv_message_mapping = {value: key for [key, value] in message_mapping.items()}


class PI2PICO(object):
    def __init__(self) -> None:
        self.serialPort = serial.Serial(
            port="/dev/ttyACM0", baudrate=230400#, bytesize=8, timeout=0, #stopbits=serial.STOPBITS_ONE
        )
        self.message_queue = []
        self.buffer = ""
        self.t_0 = 0
        self.euler_pose = []

    def check_string(self, str_in, is_buffer=False):
        self.buffer += str_in
        start_idx = [idx for idx, i in enumerate(self.buffer) if i == "<"]
        end_idx = [idx for idx, i in enumerate(self.buffer) if i == ">"]
        if start_idx and end_idx and end_idx[-1] > start_idx[-1]: 
            str_out = self.buffer[start_idx[-1]:end_idx[-1]+1]
            self.buffer = ""
            # print("VALID READ: ", str_out)
            self.message_queue.append(str_out)
            return 1
        else:
            # print("1oop READ: ", str_in, self.buffer)
            pass
        return 0

    
    def read_serial(self):
        bytes_out = self.serialPort.read(self.serialPort.in_waiting)
        try:
            str_out = bytes_out.decode("ascii")
            if str_out:
                print("reading", str_out, self.buffer)
                status = self.check_string(str_out)
        except Exception as e:
            print(e)


    def get_message(self, msg):
        error=""
        if "<" == msg[0] and ">" == msg[-1]:
            # Valid message
            msg = msg[1:-1]
            msg_type = msg.split(",")[0]
            data = msg.split(",")[1:]
            data = [data] if type(data) != type([]) else data
            data = [i for i in data if i]
            try:
                msg_data = {i.split(":")[0]: i.split(":")[1] for i in data}
                return msg_type, msg_data
            except Exception as e:
                error += str(e)
                print("AAAAAAAAAAAAAAAAAAAAAAAA: ", msg, data, error)
        else:
            print("get_message[FAIL]", msg)
        return None, None
    
    def process_msg(self, _msg_type, _msg):
        if _msg_type in message_mapping:
            call_func = getattr(self, message_mapping[_msg_type])
            try:
                call_func(_msg)
            except KeyError:
                pass

        else:
            print("Failed process_msg", _msg_type, _msg)

    def write_msg(self, _msg_func, _msg_dict={"A":0, "B":0}):
        data_str = ",".join([f"{key}:{value}" for [key, value] in _msg_dict.items()])
        type_str = inv_message_mapping[_msg_func]
        out_str = f"<{type_str},{data_str}>\n"
        # self.serialPort.flush()
        self.serialPort.write(bytes(out_str, "ascii"))
        print("writing", out_str)

    def update_state(self):
        while self.message_queue:
            msg = self.message_queue.pop(0)
            msg_type, msg = self.get_message(msg)
            if type(msg_type) != None:
                self.process_msg(msg_type, msg)
            else:
                print("Failed update msg", msg_type)

    def set_setpoints(self, _msg):
        out_dict = {key: round(value, 1) for key, value in zip(["T", "R", "P", "Y"], _msg)}
        self.write_msg("set_setpoints", out_dict)
    
    def set_pid_setpoints(self, _msg):
        out_dict = {key: round(value, 1) for key, value in zip(["P", "I", "D"], _msg)}
        self.write_msg("set_pid_setpoints", out_dict)

    def set_arm(self, inp):
        out_dict = {key: round(value, 1) for key, value in zip(["A"], [inp])}
        self.write_msg("set_arm", out_dict)

    def write_config(self):
        out_dict = {key: round(value, 1) for key, value in zip(["A"], [1])}
        self.write_msg("write_config", out_dict)

    def get_pid_setpoints(self, _msg=""):
        if _msg:
            print("get_pid_setpoints", _msg)
        else:
            self.write_msg("get_pid_setpoints")
    
    def get_log(self, _msg=""):
        if _msg:
            print("AAAAAAAAAAAAA ", _msg)

    def get_pose(self, _msg=""):
        if _msg:
            # print(time.time() - self.t_0)
            # print("get_pose", _msg)
            self.euler_pose = [round(float(_msg[i]),2) for i in ["R", "P", "Y"]]
            # print("get_pose", self.euler_pose)
        else:
            self.t_0 = time.time()
            self.write_msg("get_pose")


    def main(self):
        i = 1
        t_0 = time.time()
        while True:
            self.read_serial()
            self.update_state()
            if time.time() - t_0 > 0.5:
                self.get_pose()
                # print(self.euler_pose)
                t_0 = time.time()



if __name__ == "__main__":
    comms = PI2PICO()
    comms.main()