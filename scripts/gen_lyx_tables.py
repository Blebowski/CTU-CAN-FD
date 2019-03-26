################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Script for generation of Lyx table headers from VHDL entities interface.
##   Recognizes generics and ports. Recognizes comments before signals.
##   Splits signals into sections. Each section is started by "large" comment
##   which is delimited by whole line of comment characters like so:
##
##   --------------------------------------------------------------------------
##   -- My fancy section of signals which I would like to have in docs!
##   --------------------------------------------------------------------------
##   -- This signal is so cool!
##   device_is_completely_stuck    :  in std_logic;
##
##   -- Restart the whole device
##   restart_q                     :  out std_logic;
##
###############################################################################
##
##   Arguments:
##		configPath  Path to YAML config file which specifies files that will
##                  be generated.
##
##	Revision history:
##		22.03.2019	Implemented the script
##
################################################################################

import argparse
import sys
import time
import importlib.util
import os
import inspect
import math
import yaml

import pyXact_generator

from pyXact_generator.gen_lib import *
from pyXact_generator.VhdlLyxEntityGeneratorWrapper import VhdlLyxEntityGeneratorWrapper

def parse_args():
	parser = argparse.ArgumentParser(
				description="""Script for generation of Lyx Tables from VHDL
                               entity interfaces""")

	parser.add_argument('--configPath', dest='configPath', help="""Path to YAML
                         config file""")

	return parser.parse_args();



if __name__ == '__main__':
	args = parse_args()

	print( 80 * "*")
	print("**  Generating Lyx docs for VHDL entity interfaces!")
	print(80 * "*")

	pythonVersion = sys.version.split('.')
	pythonCmd = "python" + pythonVersion[0] + "." + pythonVersion[1]
	print("\n Python version is: %s \n" % pythonCmd)

	with open(args.configPath, 'rt') as f:
		config = yaml.safe_load(f)

		gen = VhdlLyxEntityGeneratorWrapper()
		gen.set_config(config)
		gen.do_update()

	print( 80 * "*")
	print("**  Finished")
	print(80 * "*")

