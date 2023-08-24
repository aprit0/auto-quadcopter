# BN-880Q
Setup: https://www.engineersgarage.com/articles-raspberry-pi-neo-6m-gps-module-interfacing/  
U-Centre: https://oscarliang.com/gps-settings-u-center/

RPI U-Boot Autoboot to avoid RX Pin overload:
1. Connect to your Pi with a serial console (for example using screen):
2. Abort the boot process through pressing a key and set the bootdelay variable to -2:
3. `U-Boot> setenv bootdelay -2`
4. `U-Boot> saveenv`

# To Test
CPO May offer single module millimetre precision
- https://github.com/utiasASRL/cpo

