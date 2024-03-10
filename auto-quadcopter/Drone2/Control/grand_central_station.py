'''
Control Mappings
Mode Hold | Control Mode
0   0   | Human
0   1   | Semi_z
1   0   | Semi_xy
1   1   | Auto
'''
class GRANDCENTRALSTATION:
    BASE_THROTTLE = 1000
    def __init__(self):
        self.bools = {
            "ARM": False,
            "MODE": False,
            "HOLD": False,
        }
        self.base_cmd_angle = [self.BASE_THROTTLE, 0, 0, 0]
        self.base_cmd_twist = None


    def set_command(self, msg, msg_type="Int16MultiArray"):
        # Receive and interpret command from base
        if msg_type == "Int16MultiArray":
            self.base_cmd_angle = list(msg.data)
        elif msg_type == "Twist":
            pass

    def get_command(self):
        # Given state of bools, get next command in T, Euler
        cmd = self.get_mode()
        return cmd[0], cmd[1:]

    def get_mode(self):
        if self.bools["ARM"]:
            if self.bools["MODE"] and self.bools["HOLD"]:
                return self._auto_mode()
            elif self.bools["MODE"]:
                return self._semi_xy()
            elif self.bools["HOLD"]:
                return self._semi_z()
            else:
                return self._human_mode()
        else:
            return [self.BASE_THROTTLE, 0, 0, 0]

    def update_bools(self, name: str, value: bool = False) -> None:
        self.bools[name] = value
    
    def _human_mode(self):
        return self.base_cmd_angle

    def _semi_xy(self):
        pass

    def _semi_z(self):
        pass

    def _auto_mode(self):
        pass

