################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##   
##	 Library with auxiliarly functions for pyXact generator.   
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
ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
PYXACT_PATH = "./pyXact_generator/ipyxact_parser"

sys.path.insert(0, PYXACT_PATH)
from ipyxact.ipyxact import Component
from license_updater import *


def open_output(output):
	return open(output, 'w')

def split_string(input, size):
	return [input[start:start+size] for start in range(0, len(input), size)]
	
def str_arg_to_bool(input):
	if (input == "yes" or
		input == "true" or
		input == "True" or
		input == "y"):
		return True
	else:
		return False

def checkIsList(obj):
	"""
	"""
	if (not (type(obj)) == list):
		print(str(obj) + " should be a list!")
		return false
	return True

def checkIsDict(obj):
	"""
	"""
	if (not (type(obj)) == dict):
		print(str(obj) + " should be dictionary!")
		return False
	return True

