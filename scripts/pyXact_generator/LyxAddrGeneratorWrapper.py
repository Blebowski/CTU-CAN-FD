################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Class for generation of Lyx document from IP-XACT specification.
##   
##   In case of CAN FD Core the register map is specified with two register 
##   maps. 8-bit map with register fields described. 32 bit register maps with
##   name aliases used on 32 bit Avalon and AXI.
##
##	Revision history:
##		31.01.2018	Implemented the script
##      27.11.2018  Changed script to be a class
##
################################################################################

import argparse
import sys
import time
import importlib.util
import os
import inspect
import math

from .gen_lib import *
from .ip_xact.lyx_addr_generator import LyxAddrGenerator


class LyxAddrGeneratorWrapper():

	# Path to a IP-XACT specification file with register maps
	xactSpec = ""

	# Name of the IP-XACT Memory map which should be used for VHDL package generatio.
	memMap = None

	# Size of the access bus word. Register bit field offsets are concatenated into 
	# word width size instead of simple offset from beginning of register. (E.g. 32 bit  ->
	# bitfields from first four 8-bit register are concatenated into 32 bit values)
	wordWidth = 32

	# Output where to write the VHDL package.
	outFile = ""

	# If memory map region overview should be generated
	genRegions = False

	# If field descriptions should be generated
	genFiDesc = False

	# Lyx template path
	lyxTemplate = ""


	def do_update(self):

		args = parse_args()
		with open(self.xactSpec) as f:
			name = None
			offset = 0
			addrMap = None
			fieldMap = None
		    
			component = Component()
			component.load(f)
			
			with open_output(self.outFile) as of:
				
				lyxGen = LyxAddrGenerator(component, self.memMap, self.wordWidth, 
											genRegions=self.genRegions,
											genFiDesc=self.genFiDesc)
				lyxGen.set_of(of)
				lyxGen.load_lyx_template(self.lyxTemplate)
				
				# Write the documentation
				lyxGen.write_mem_map_both()

				lyxGen.lyxGen.commit_append_lines_all()
				
				lyxGen.commit_to_file()

	if __name__ == '__main__':
		self.do_update()
