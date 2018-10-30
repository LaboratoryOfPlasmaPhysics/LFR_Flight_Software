#!/usr/bin/env python3

__author__ = "Alexis Jeandet"
__copyright__ = "Copyright 2018, Laboratory of Plasma Physics"
__credits__ = []
__license__ = "GPLv2"
__version__ = "1.0.0"
__maintainer__ = "Alexis Jeandet"
__email__ = "alexis.jeandet@member.fsf.org"
__status__ = "Development"

import time
import sys
import os
import serial
import argparse
from datetime import datetime

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port", help="Serial port")
parser.add_argument("-s", "--speed", help="Baud rate")
args = parser.parse_args()



def main():
    with open('gcov_out_'+str(datetime.now())+'.txt','w') as gcov:
        with open('console_'+str(datetime.now())+'.txt','w') as console:
            with serial.Serial(args.port, args.speed, timeout=None) as ser:
                line = ser.readline().decode()
                while '_GCOVEXIT_BEGIN_' not in line:
                    console.write(line)
                    line = ser.readline().decode()
                line = ser.readline().decode()
                while '_GCOVEXIT_END_' not in line:
                    gcov.write(line)
                    line = ser.readline().decode()

if __name__ == "__main__":
    main()
