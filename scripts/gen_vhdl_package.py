################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Script for generation of VHDL package from IP-XACT specification. Register
##   map addresses, bit field offsets and enums are generated. Two separate
##   register maps can be specified: one for bit fields, one for addresses.
##   
##   In case of CAN FD Core the register map is specified with two register 
##   maps. 8-bit map with register fields described. 32 bit register maps with
##   name aliases used on 32 bit Avalon and AXI.
##
##   Arguments:
##		licPath 	- File with license which should be placed to header of the
##                    all source code files.
##		xactSpec    - Path to a IP-XACT specification file with register maps
##		adrMap      - Name of the IP-XACT Memory map which should be used for
##					  address constants generation.
##      fieldMap    - Name of the IP-XACT Memory map which should be used for
##					  bit field constants and enums generation.
##      wordWidth   - Size of the access bus word. Register bit field offsets 
##					  are concatenated into word width size instead of simple
##					  offset from beginning of register. (E.g. 32 bit  ->  
##					  bitfields from first four 8-bit register are concatenated
##					  into 32 bit values)
##		packName	- Name of the VHDL package to create
##		outFile		- Output where to write the VHDL package.
##
##
##	Revision history:
##		16.01.2018	Implemented the script
##
################################################################################

import argparse
import sys
import time
import importlib.util
import os
import inspect
import math

from pyXact_generator.gen_lib import *
from pyXact_generator.ip_xact.vhdl_addr_generator import VhdlAddrGenerator

def parse_args():
	parser = argparse.ArgumentParser(
				description='Generate a VHDL package from an IP-XACT file')
	parser.add_argument('--licPath', dest='licPath' , help=""" File with
							license which should be placed to header of the
							all source code files""")
	parser.add_argument('--xactSpec', dest='xactSpec', help="""Path to a IP-XACT
							specification file with register maps""")
	parser.add_argument('--fieldMap', dest='fieldMap', help=""" Name of the
							IP-XACT Memory map which should be used for
							bit field constants and enums generation""")
	parser.add_argument('--addrMap', dest='addrMap', help=""" Name of the 
								IP-XACT Memory map which should be used for
								address constants generation.""")
	parser.add_argument('--wordWidth', dest='wordWidth', type=int, 
							help=""" Size of the
							access bus word. Register bit field offsets are
							concatenated into word width size instead of simple
							offset from beginning of register. (E.g. 32 bit  ->
							bitfields from first four 8-bit register are
							concatenated into 32 bit values)""")
	parser.add_argument('--packName', dest='packName', help="""Name of the 
						VHDL package to create""")											
	parser.add_argument('--outFile', dest='outFile', help=""" Output where to write 
							the VHDL package.""")											
	return parser.parse_args();


if __name__ == '__main__':
	args = parse_args()
	with open(args.xactSpec) as f:
		name = None
		offset = 0
		
        # Load IP-Xact component
		component = Component()
		component.load(f)
			
		with open_output(args.outFile) as of:
			
			vhdlGen = VhdlAddrGenerator(component, args.addrMap, args.fieldMap,
										args.wordWidth)
			vhdlGen.set_of(of)
			
			if (args.licPath != ""):
				lic_text = load_license(args.licPath)
				write_license(lic_text, '-', of)
			
			vhdlGen.create_addrMap_package(args.packName)
			
			vhdlGen.commit_to_file()