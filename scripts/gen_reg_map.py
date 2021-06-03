################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Script for generation of VHDL register map entity from IP-XACT 
##   specification.
##
##   Arguments:
##		licPath 	- File with license which should be placed to header of the
##                    all source code files.
##		xactSpec    - Path to a IP-XACT specification file with register maps
##		memMap      - Name of the IP-XACT Memory map which should be used for
##					  register map implementation.
##      wordWidth   - Size of the access bus word. Register bit field offsets 
##					  are concatenated into word width size instead of simple
##					  offset from beginning of register. (E.g. 32 bit  ->  
##					  bitfields from first four 8-bit register are concatenated
##					  into 32 bit values)
##		outDir		- Output directory where register map is created.
##
##
##	Revision history:
##		25.11.2018	Implemented the script
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
from pyXact_generator.ip_xact.vhdl_reg_map_generator import VhdlRegMapGenerator

from shutil import copyfile

def parse_args():
	parser = argparse.ArgumentParser(
				description='Generate a VHDL package from an IP-XACT file')

	parser.add_argument('--licPath', dest='licPath' , help=""" File with
							license which should be placed to header of the
							all source code files""")

	parser.add_argument('--xactSpec', dest='xactSpec', help="""Path to a IP-XACT
							specification file with register maps""")

	parser.add_argument('--memMap', dest='memMap', help=""" Name of the
							IP-XACT Memory map which should be used for
							register map implementation""")

	parser.add_argument('--wordWidth', dest='wordWidth', type=int, 
							help=""" Size of the
							access bus word. Register bit field offsets are
							concatenated into word width size instead of simple
							offset from beginning of register. (E.g. 32 bit  ->
							bitfields from first four 8-bit register are
							concatenated into 32 bit values)""")

	parser.add_argument('--registeredRead', dest='registeredRead',
							help=""" When set to "True" read data are read with
							one clock cycle delay. When set to false read data
							are available within the same clock cycle""")

	parser.add_argument('--outDir', dest='outDir', help=""" Output directory 
                         where to write VHDL register map implementation.""")

	return parser.parse_args()


def write_reg_map_package(vhdlGen, dir_path):
	"""
	Create package with records for register blocks within an address block.
	"""
	reg_map_pkg_name = os.path.join(dir_path, vhdlGen.memMap.name.lower() + "_pkg.vhd")
	
	of = open(reg_map_pkg_name, 'w')
	vhdlGen.set_of(of)

	write_license(lic_text, '-', of)
	vhdlGen.write_reg_map_pkg()
	vhdlGen.commit_to_file()

	of.close()


def write_reg_map_implementation(vhdlGen, dir_path):
	"""
	Write register map implementation. Create separate entity file for
	each register memory block.
	"""
	for block in vhdlGen.memMap.addressBlock:
		print("Processing memory block: " + block.name)
		
		if (block.usage == "register"):
			file_path = os.path.join(dir_path, block.name.lower() + "_reg_map.vhd")

			of = open(file_path, 'w')
			vhdlGen.set_of(of)

			write_license(lic_text, '-', of)
			vhdlGen.write_reg_block(block)
			vhdlGen.commit_to_file()

			of.close()

		else:
			print("Skipping unsupported block type: " + block.usage)

		print("\n")


def copy_reg_map_sources(vhdlGen, dir_path, destDir):
	"""
	Copy VHDL templates to destination directory!
	"""
	for templ_name, templ_path in vhdlGen.template_sources.items():
		src_path = os.path.join(ROOT_PATH, templ_path)
		dest_path = os.path.join(ROOT_PATH, destDir)
		dest_path = os.path.join(dest_path, os.path.basename(templ_path))

		copyfile(src_path, dest_path)


if __name__ == '__main__':
	args = parse_args()
	with open(args.xactSpec) as f:
		name = None
		offset = 0
		
        # Load IP-Xact component
		component = Component()
		component.load(f)

		# Create new VHDL register map generator
		vhdlGen = VhdlRegMapGenerator(component, args.memMap, args.wordWidth)

		# Load license text
		lic_text = ""
		if (args.licPath != ""):
			lic_text = load_license(args.licPath)

		# Check output directory
		dir_path = os.path.join(ROOT_PATH, args.outDir)
		if (not os.path.isdir(dir_path)):
			print("ERROR: " + dir_path + " is not a directory")
			sys.exit(1)

		# Configure registered / non-registered read
		if (args.registeredRead == "y" or args.registeredRead == "yes"):
			vhdlGen.registered_read = True
		else:
			vhdlGen.registered_read = False

		# Create common package for whole address map
		write_reg_map_package(vhdlGen, dir_path)

		# Create implementation of each register block within address map
		write_reg_map_implementation(vhdlGen, dir_path)

		# Copy source templates to destination directory
		copy_reg_map_sources(vhdlGen, dir_path, args.outDir)


