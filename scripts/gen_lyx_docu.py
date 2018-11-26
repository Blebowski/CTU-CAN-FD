################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Script for generation of Lyx document from IP-XACT specification.
##   
##   In case of CAN FD Core the register map is specified with two register 
##   maps. 8-bit map with register fields described. 32 bit register maps with
##   name aliases used on 32 bit Avalon and AXI.
##
##   Arguments:
##		xactSpec    - Path to a IP-XACT specification file with register maps
##		adrMap      - Name of the IP-XACT Memory map which should be used for
##					  address constants generation.
##      memMap      - Name of the IP-XACT Memory map
##      wordWidth   - Size of the access bus word. Register bit field offsets 
##					  are concatenated into word width size instead of simple
##					  offset from beginning of register. (E.g. 32 bit  ->  
##					  bitfields from first four 8-bit register are concatenated
##					  into 32 bit values)
##      genRegions  - If memory map region overview should be generated
##      genFiDesc   - If field descriptions should be generated
##		chaptName   - Name of the Chapter to create
##      lyxTemplate - Path to an empty Lyx document with document header.
##		outFile		- Output where to write the Lyx chapter.
##
##	Revision history:
##		31.01.2018	Implemented the script
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
from pyXact_generator.ip_xact.lyx_addr_generator import LyxAddrGenerator

def parse_args():
	parser = argparse.ArgumentParser(
				description='Create Lyx document from an IP-XACT file')
	parser.add_argument('--xactSpec', dest='xactSpec', help="""Path to a IP-XACT
							specification file with register maps""")
	parser.add_argument('--memMap', dest='memMap', help=""" Name of the
							IP-XACT Memory map which should be used for
							bit field constants and enums generation""")
	parser.add_argument('--wordWidth', dest='wordWidth', type=int, 
							help=""" Size of the
							access bus word. Register bit field offsets are
							concatenated into word width size instead of simple
							offset from beginning of register. (E.g. 32 bit  ->
							bitfields from first four 8-bit register are
							concatenated into 32 bit values)""")
	parser.add_argument('--genRegions', dest='genRegions', help=""" If memory map 
								region overview should be generated""")
	parser.add_argument('--genFiDesc', dest='genFiDesc', help="""If field 
								descriptions should be generated""")
	parser.add_argument('--chaptName', dest='packName', help="""Name of the 
						C Header IFDEF to create""")											
	parser.add_argument('--outFile', dest='outFile', help=""" Output where to write 
							the VHDL package.""")
	parser.add_argument('--lyxTemplate', dest='lyxTemplate', help="""Path to an 
							empty Lyx document with document header.""")	
										
	return parser.parse_args();
		

if __name__ == '__main__':
	args = parse_args()
	with open(args.xactSpec) as f:
		name = None
		offset = 0
		addrMap = None
		fieldMap = None
        
		component = Component()
		component.load(f)
		
		with open_output(args.outFile) as of:
			
			lyxGen = LyxAddrGenerator(component, args.memMap, args.wordWidth, 
										genRegions=args.genRegions,
										genFiDesc=args.genFiDesc)
			lyxGen.set_of(of)
			lyxGen.load_lyx_template(args.lyxTemplate)
			
			# Write the documentation
			lyxGen.write_mem_map_both()

			lyxGen.lyxGen.commit_append_lines_all()
			
			lyxGen.commit_to_file()
