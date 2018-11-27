################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Class for generation of VHDL register map entity from IP-XACT 
##   specification.
##
##	Revision history:
##		25.11.2018	Implemented the script
##      27.11.2018  Changed implementation to be a class
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
from .ip_xact.vhdl_reg_map_generator import VhdlRegMapGenerator

from shutil import copyfile

class VhdlRegMapGeneratorWrapper():

	# File with license which should be placed to header of the all source code files
	licPath = ""

	# Path to a IP-XACT specification file with register maps
	xactSpec = ""

	# Name of the IP-XACT Memory map which should be used for VHDL package generatio.
	memMap = None

	# Size of the access bus word. Register bit field offsets are concatenated into 
	# word width size instead of simple offset from beginning of register. (E.g. 32 bit  ->
	# bitfields from first four 8-bit register are concatenated into 32 bit values)
	wordWidth = 32

	# When set to "True" read data are read with one clock cycle delay. When set to 
	# false read data are available within the same clock cycle
	registeredRead = True

	# Output directory where to write VHDL register map implementation.
	outDir = ""


	# Variable for loaded license Text
	lic_text = ""


	def write_reg_map_package(self, vhdlGen, dir_path):
		"""
		Create package with records for register blocks within an address block.
		"""
		reg_map_pkg_name = os.path.join(dir_path, vhdlGen.memMap.name.lower() + "_pkg.vhd")
		
		of = open(reg_map_pkg_name, 'w')
		vhdlGen.set_of(of)

		write_license(self.lic_text, '-', of)
		vhdlGen.write_reg_map_pkg()
		vhdlGen.commit_to_file()

		of.close()


	def write_reg_map_implementation(self, vhdlGen, dir_path):
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

				write_license(self.lic_text, '-', of)
				vhdlGen.write_reg_block(block)
				vhdlGen.commit_to_file()

				of.close()

			else:
				print("Skipping unsupported block type: " + block.usage)

			print("\n")


	def copy_reg_map_sources(self, vhdlGen, dir_path, destDir):
		"""
		Copy VHDL templates to destination directory!
		"""
		for templ_name, templ_path in vhdlGen.template_sources.items():
			src_path = os.path.join(ROOT_PATH, templ_path)
			dest_path = os.path.join(ROOT_PATH, destDir)
			dest_path = os.path.join(dest_path, os.path.basename(templ_path))

			copyfile(src_path, dest_path)


	def do_update(self):

		with open(self.xactSpec) as f:
			name = None
			offset = 0

			# Load IP-Xact component
			component = Component()
			component.load(f)

			# Create new VHDL register map generator
			vhdlGen = VhdlRegMapGenerator(component, self.memMap, self.wordWidth)

			# Load license text
			self.lic_text = ""
			if (self.licPath != ""):
				self.lic_text = load_license(self.licPath)

			# Check output directory
			dir_path = os.path.join(ROOT_PATH, self.outDir)
			if (not os.path.isdir(dir_path)):
				print("ERROR: " + dir_path + " is not a directory")
				sys.exit(1)

			# Configure registered / non-registered read
			if (str_arg_to_bool(self.registeredRead)):
				vhdlGen.registered_read = True
			else:
				vhdlGen.registered_read = False

			# Create common package for whole address map
			self.write_reg_map_package(vhdlGen, dir_path)

			# Create implementation of each register block within address map
			self.write_reg_map_implementation(vhdlGen, dir_path)

			# Copy source templates to destination directory
			self.copy_reg_map_sources(vhdlGen, dir_path, self.outDir)


	if __name__ == '__main__':
		self.do_update()

