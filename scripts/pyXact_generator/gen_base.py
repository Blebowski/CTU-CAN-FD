################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##
##   IP XACT generator base class. Each class implementing support of different
##   output file (VHDL, header file, .docx, etc...) should inherit from new
##   class.
##
##	Revision history:
##		25.01.2018	First implementation
##
################################################################################

from abc import ABCMeta, abstractmethod

class baseGenerator(metaclass=ABCMeta):

	@abstractmethod
	def is_supported_type(self, type):
		pass

	@abstractmethod
	def write_comment(self, of, input, gap):
		pass


################################################################################
# Write the introductory part of the generated file
# 
# Arguments:
#  name 	    - name of the package, ifdef, etc...
#  comment		- comment sign to use ('-')
################################################################################
	@abstractmethod
	def write_prologue(self, of, name, comment):
		pass


################################################################################
# Write the ending part of the package.
#
# Arguments:
#  name			- Name of the package, ifdef, etc...
################################################################################	
	@abstractmethod
	def write_epilogue(self, of, name):
		pass


################################################################################
# Write single line declaration of an object (constant, variable, signal) etc...
#
# Arguments:
#  name			- Name of the package, ifdef, etc...
################################################################################
	@abstractmethod
	def write_decl(self, of, specifier, name, bitWidth, type, value, alignLen):
		pass


################################################################################
# Write possible enumeration values of register field.
#
# Arguments:
#  of		 	- Output file to write
#  elem         - Element to write the enum constants for
#  type			- VHDL type that should define this enum
################################################################################	
	@abstractmethod
	def write_reg_enums(self, of, field):
		pass


################################################################################
# Write Reset (default) values of register field
#
# Arguments:
#  of		 	- Output file to write
#  elem         - Element to write the enum constants for
################################################################################	
	@abstractmethod
	def write_res_vals(self, of, field):
		pass


################################################################################
# Write single field of a register
#
# Arguments:
#  of		 	- Output file to write
#  field		- Bitfield element to write
#  reg			- Register that owns the bitfield
#  busWidth		- Width of the data word for bitfield offset concatenation
################################################################################	
	@abstractmethod
	def write_reg_field(self, of, field, reg, busWidth):
		pass


################################################################################
# Write register fields constants of single register
#
# Arguments:
#  of		 	- Output file to write
#  reg			- Memory map object to process
#  busWidth		- Width of the bus for bit field offset concatenation!
################################################################################		
	@abstractmethod
	def write_reg(self, of, reg, busWidth): 
		pass


################################################################################
# Process each address block and write the register field constants within it.
#
# Arguments:
#  of		 	- Output file to write
#  memoryMap	- Memory map object to process
#  busWidth		- Width of the bus for bit field offset concatenation!
################################################################################		
	@abstractmethod
	def write_regs(self, of, regs, busWidth):
		pass


################################################################################
# Process each address block and write the register field constants within it.
#
# Arguments:
#  of		 	- Output file to write
#  memoryMap	- Memory map object to process
#  busWidth		- Width of the bus for bit field offset concatenation!
################################################################################	
	@abstractmethod
	def write_mem_map_fields(self, of, memoryMap, busWidth):
		pass


################################################################################
# Write Address block head
#
# Arguments:
#  of		 	- Output file to write
#  addressBlock	- Address block where to write the registers from
#  vhdlType		- VHDL type to use for constant (std_logic_vector etc...)
#  vhdlLen		- Length of the VHDL type - TODO: create support for 'natural'!
################################################################################
	@abstractmethod
	def write_addrbl_head(self, of, addressBlock):
		pass


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
	@abstractmethod
	def write_addrbl_regs(self, of, addressBlock):
		pass


################################################################################
# Iterate through address blocks in the memory map and write:
#  1. Address constant representing the highest bits defined by region size.
#		(Assuming blocks of the same size!)
#  2. Adresss offsets of each register within the Address block!
#
# Arguments:
#  of		 	- Output file to write
#  memoryMap	- Memory map object from ipyxact
################################################################################
	@abstractmethod
	def write_mem_map_addr(of, memoryMap):
		pass
	
	
	
	