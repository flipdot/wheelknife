#!/bin/bash
screen -L -Logfile `date +%s`.csv /dev/ttyUSB0 9600
