import time
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import subprocess

from tools.wifi import *
# from wifi import *


class OLED:
    def __init__(self):
        self.disp = Adafruit_SSD1306.SSD1306_128_32(rst=None)
        self.disp.begin()
        self.width = self.disp.width
        self.height = self.disp.height
        self.status_dict = None
        self.clear()

    def clear(self):
        self.disp.clear()
        self.disp.display()

    def set_status(self):
        image = Image.new('1', (self.width, self.height))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0,0,self.width,self.height), outline=0, fill=0)
        padding = -2
        top = padding
        bottom = self.height-padding
        font = ImageFont.load_default()
        self.status_dict = self.get_status()
        x = 0
        keys = list(self.status_dict.keys())
        for i in range(len(keys)):
            draw.text((x, top + i * 8), self.status_dict[keys[i]], font=font, fill=255)
        self.disp.image(image)
        self.disp.display()

    def get_status(self):
        cmd = "hostname -I | cut -d\' \' -f1"
        IP = str(subprocess.check_output(cmd, shell = True ))
        cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
        CPU = str(subprocess.check_output(cmd, shell = True ))
        wifi = main() # From wifi
        return {'Name': wifi['Name'],
                'IP': IP[2:-3],
                'Quality': wifi['Quality'],
                'RSSI': wifi['Signal'],
                'CPU': CPU[2:-1]}

if __name__ == '__main__':
    disp = OLED()
    t_0 = time.time()
    while True:
        if time.time() - t_0 > 0.5:
            disp.set_status()
            t_0 = time.time()

    