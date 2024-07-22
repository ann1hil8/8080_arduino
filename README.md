This project's goal is to run CP/M 2.2 on an original IMSAI 8080 by emulating the Intel 8080 CPU on an Arduino Mega 2560.

An IMSAI 8080 runs at 2MHz. Testing with the 16MHz Mega 2560 produces a current 8080 emulation speed of 250kHz.

i8080.c and i8080.h are from  https://github.com/superzazu/8080/tree/master Modified to compile in the Arduino IDE and not count total cycles. TODO: Add Status Word information and flags to emulator.

An extension to this project could be some type of S-100 bus diagnostic tool.
