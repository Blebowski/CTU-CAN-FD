################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Backend library for generation of HW, SW design files from IP-XACT 
##   specification based on pyxact XML parser. Contains basic functions and
##   includes needed by the whole generation framework
##
##	Revision history:
##		24.01.2018	First implementation based on the previous stand-alone
##                  script for generation of VHDL package
##
################################################################################

import argparse
import sys
import time
import importlib.util
import os
import inspect
import math

################################################################################
# File path to the local repo of the PyXact framework
################################################################################
PYXACT_PATH = "E:\Skola\CVUT-FEL\ipyxact"

sys.path.insert(0, PYXACT_PATH)
from ipyxact.ipyxact import Component
from license_updater import *

supFileTypes = ['.h','vhd']
commentSigns = {'.h':'*','.vhd':'-'}

def open_output(output):
    return open(output, 'w') if output else sys.stdout

def split_string(input, size):
	return [input[start:start+size] for start in range(0, len(input), size)]
	
def is_sup_file(fileType):
	retVal = false
	for type in supFileTypes:
		if (type == fileType):
			retVal = true
			break
	return retVal
	