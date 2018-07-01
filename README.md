# Raspberry_XL320_cpp
Raspberry_XL320_cpp is a class designed to be used with a Raspberry Pi (stretch), using the serial interface, to drive Dynamixel XL320 servomotors.

Personally I use the PIXL board which is plugged on the raspberry

It is necessary to install wiringPi first.

The UART must be enabled by modifying the initial settings such as:
- in /boot.config.txt add : <br>
    dtoverlay=pi3-miniuart-bt <br>
    core_freq=250
- in /boot/cmdline.txt : <br>
    console=tty1

