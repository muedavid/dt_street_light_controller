#!/bin/bash

#install everything
sudo apt-get update
yes Y | command-that-asks-for-input
sudo apt-get install bison autoconf flex
yes Y | command-that-asks-for-input
sudo apt-get install gcc-avr binutils-avr gdb-avr avr-libc avrdude
yes Y | command-that-asks-for-input
sudo apt-get install build-essential
yes Y | command-that-asks-for-input
sudo apt-get install python-smbus
yes Y | command-that-asks-for-input

#copy avrdude.conf
cd ./software\ -\ WT/_avrdudeconfig
sudo cp avrdude.conf /etc/avrdude.conf

#make fuses and make
cd ~/software\ -\ WT
make fuses
make clean
make



