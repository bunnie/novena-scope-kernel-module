#!/bin/sh
./configure.sh novena_fpga.bit
sudo ./novena-scope -adc_pll
sudo ./novena-scope -adc_cal
sudo ./novena-scope -ddr3dump 0x0 0x1000 temp.bin
./chee-z-print temp.bin
