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
##	Example of usage from IDLE shell
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

################################################################################
# File path to the local repo of the PyXact framework
################################################################################
PYXACT_PATH = "E:\Skola\CVUT-FEL\ipyxact"

sys.path.insert(0, PYXACT_PATH)
from ipyxact.ipyxact import Component
from license_updater import *


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

def open_output(output):
    return open(output, 'w') if output else sys.stdout

def split_string(input, size):
	return [input[start:start+size] for start in range(0, len(input), size)]

def write_comment(of, input, prefix):
	splt_str = split_string(input, 75)
	for line in splt_str:
		of.write('{}-- {}\n'.format(prefix, line))


################################################################################
# Write the header part of the VHDL package. With Basic iee.std_logic_1164 
# include.
# 
# Arguments:
#  packageName 	- name of the package
#  comment		- comment sign to use ('-')
################################################################################
def write_prologue(of, packageName, comment):
	comLine = '{:{fill}<80}\n'.format("", fill="-",)
	purp = '-- Purpose:\n'
	of.write(comLine + purp)
	write_comment(of, comment, "")
	of.write(comLine+'\n')
	of.write('Library ieee;\nuse ieee.std_logic_1164.all;\n\n')
	of.write('package {} is\n\n'.format(packageName))


################################################################################
# Write the ending part of the package.
#
# Arguments:
#
################################################################################	
def write_epilogue(of):
	of.write('end package;')

################################################################################
# Write single bitfield as VHDL constant of certain register.
#
# Arguments:
#  of		 	- Output file to write
#  elem         - Element to write the enum constants for
#  type			- VHDL type that should define this enum
################################################################################	
def write_enums(of, elem, type, single_type):
	if (elem.enumeratedValues != []):
		
		of.write('\n')
		of.write('  -- "{}" field enumerated values\n'.format(elem.name))
		for es in elem.enumeratedValues:
			for e in sorted(es.enumeratedValue, key=lambda x: x.value):
				if (elem.bitWidth > 1):
					hexFmt= ':0{}X'.format(math.ceil(float(elem.bitWidth)/4))
					hexFmt='{'+hexFmt+'}'
					eVal = hexFmt.format(e.value)
					of.write('  constant {} : {}({} downto 0) := x"{}";\n'.format(
								e.name, type, elem.bitWidth-1, eVal))
				else:
					of.write("  constant {} : {} := '{}';\n".format(
								e.name, single_type, e.value))

################################################################################
# Write single bitfield as VHDL constant of certain register.
#
# Arguments:
#  of		 	- Output file to write
#  elem			- Bitfield element to write
#  reg			- Register that owns the bitfield
#  type			- type of the VHDL constant (TODO: support other than natural)
#  busWidth		- Width of the data word for bitfield offset concatenation
################################################################################	
def write_reg_elem(of, elem, reg, type, busWidth):
	#Calculate the bit index in the data word based on bus width that
	#is used to acccess the register
	bitIndexL = elem.bitOffset
	bitIndexH = elem.bitOffset + elem.bitWidth-1
	bitIndexL = bitIndexL + ((reg.addressOffset*8) % busWidth)
	bitIndexH = bitIndexH + ((reg.addressOffset*8) % busWidth)
			
	if (bitIndexH == bitIndexL):
		iter = [["IND", bitIndexL]]
	else:
		iter = [["L", bitIndexL], ["H", bitIndexH]]
			
	for item in iter:
		pref = '  constant {}_{} '.format(elem.name.upper(), item[0])
		post = ': {} := {};\n'.format(type, item[1])
		post = '{:>{}}'.format(post, 50-len(pref))
		of.write(pref + post)


################################################################################
# Process each address block and write the register field constants within it.
#
# Arguments:
#  of		 	- Output file to write
#  memoryMap	- Memory map object to process
#  busWidth		- Width of the bus for bit field offset concatenation!
################################################################################	
def write_reg_bits(of, registers, type, busWidth):
	for reg in registers:
		
		#Write the register title
		comLine = '{:{fill}<80}\n'.format("  ", fill="-",)
		of.write(comLine)
		of.write('  -- {} register\n'.format(reg.name.upper()))
		of.write('  --\n')
		write_comment(of, reg.description, "  ")
		of.write(comLine)
		
		#Write the individual elements
		for elem in reg.field:
			write_reg_elem(of, elem, reg, type, busWidth)
		
		#Write the enums (iterate separately not to mix up fields and enums)
		for elem in reg.field:
			write_enums(of, elem, 'std_logic_vector', 'std_logic')
			
		of.write('\n')


################################################################################
# Process each address block and write the register field constants within it.
#
# Arguments:
#  of		 	- Output file to write
#  memoryMap	- Memory map object to process
#  busWidth		- Width of the bus for bit field offset concatenation!
################################################################################	
def write_memory_map_fields(of, memoryMap, busWidth):
	for block in memoryMap.addressBlock:
		write_reg_bits(of, block.register, "natural", busWidth)


################################################################################
# Write Address block offset as VHDL constant.
#
# TODO: Add support for naturals!
#
# Arguments:
#  of		 	- Output file to write
#  addressBlock	- Address block where to write the registers from
#  vhdlType		- VHDL type to use for constant (std_logic_vector etc...)
#  vhdlLen		- Length of the VHDL type - TODO: create support for 'natural'!
################################################################################	
def write_address_block_head(of, addressBlock, vhdlType, vhdlLen):
	
	# Write capital comment with name of the address Block
	comLine = '{:{fill}<78}\n'.format("  ", fill="-",)
	com = '  -- Address block: {}\n'.format(addressBlock.name)
	of.write(comLine+com+comLine)
	
	# Write the VHDL constant for Address block offset defined as:
	#	block.baseAddress/block.range
	pref = '  constant {}_BLOCK'.format(addressBlock.name.upper())
	post = ': {}({} downto 0) := x"{}";\n'.format(vhdlType, 3, 
				int(addressBlock.baseAddress/addressBlock.range))
	pref_len = 80-len(pref)
	# Right align to 80 chars
	post = '{:>{}}'.format(post, pref_len)
	of.write(pref+post+"\n")


################################################################################
# Write register offsets as VHDL constants of defined type in from given
# address block.
#
# Arguments:
#  of		 	- Output file to write
#  addressBlock	- Address block where to write the registers from
#  vhdlType		- VHDL type to use for constant (std_logic_vector etc...)
#  vhdlLen		- Length of the VHDL type - TODO: create support for 'natural'!
################################################################################	
def write_address_block_regs(of, addressBlock, vhdlType, vhdlLen):
	
	for reg in sorted(addressBlock.register, key=lambda a: a.addressOffset):
		pref = '  constant {}_ADR '.format(reg.name.upper())
		addr = '{:03X}'.format(reg.addressOffset+addressBlock.baseAddress, fill='0')
		post = ': {}({} downto 0) := x"{}";\n'.format(vhdlType, vhdlLen, addr)
		pref_len = 80-len(pref)
		post = '{:>{}}'.format(post, pref_len)
		of.write(pref+post)
	of.write('\n\n');
	
		
################################################################################
# Iterate through address blocks in the memory map and write:
#  1. Address constant representing the highest bits defined by region size.
#		(Assuming blocks of the same size!)
#  2. Adresss offsets of each register within the Address block!
#
# Arguments:
#  of		 	- Output file to write
#  memoryMap	- Memory map object from ipyxact
#  vhdlType		- VHDL type to use for constant (std_logic_vector etc...)
################################################################################	
def write_memory_map_addr(of, memoryMap, vhdlType):
	#Each Address block reflects to VHDL memory region
	for block in memoryMap.addressBlock:
		write_address_block_head(of, block, "std_logic_vector", 3)
		write_address_block_regs(of, block, "std_logic_vector", 11)
		

if __name__ == '__main__':
	args = parse_args()
	with open(args.xactSpec) as f:
		name = None
		offset = 0
        
		component = Component()
		component.load(f)
		
		with open_output(args.outFile) as of:
			
			if (args.licPath != ""):
				lic_text = load_license(args.licPath)
				write_license(lic_text, '-', of)
			
			# Create a prologue
			if (args.addrMap != ""):
				comment = """Address constants for register map: %s""" % args.addrMap
			if (args.fieldMap != ""):
				comment = comment + """Bit field constants for register map: %s.""" % args.fieldMap
			comment = comment + " This file is autogenerated, do NOT edit!"
			write_prologue(of, args.packName, comment)
					
			# Process the Memory maps and write the address offsets and fields
			for map in component.memoryMaps.memoryMap:
				if map.name == args.addrMap:
					print ("Writing addresses of '%s' register map" % args.addrMap) 
					write_memory_map_addr(of, map, "std_logic_vector")
				if map.name == args.fieldMap:
					print ("Writing bit fields of '%s' register map" % args.fieldMap)
					write_memory_map_fields(of, map, args.wordWidth)
			
			write_epilogue(of)
			print ("Created '%s' output file" % args.outFile) 