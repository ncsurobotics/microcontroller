#!/bin/bash

avr-gcc -O3 -mmcu=atxmega16d4 -c -o power_board.o power_board.c
avr-gcc -O3 -mmcu=atxmega16d4 power_board.o -o power_board.obj
avr-objcopy  -R .eeprom -O ihex power_board.obj power_board.hex
avrdude -v -c avrispmkII -p x16d4 -U power_board.hex
