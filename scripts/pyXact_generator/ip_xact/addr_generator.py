################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##
##	 Base class for specific address Map generators.
##   Two separate address maps are considered. Map for address creation
##   and map for bitfields, enums and reset values creation.
##
##	Revision history:
##		25.01.2018	First implementation
##
################################################################################

from abc import ABCMeta, abstractmethod

import math

class IpXactAddrGenerator(metaclass=ABCMeta):

	# IP-XACT memory map object
	memMap = None

	# Word width (in bits)
	wrdWidthBit = None

	# Word width in Bytes
	wrdWidthByte = None 

	pyXactComp = None	
	
	of = None
	
	def __init__(self, pyXactComp, memMap, wordWidth):
		self.wrdWidthBit = wordWidth
		self.wrdWidthByte = int(wordWidth / 8)

		if (not pyXactComp.memoryMaps):
			return None

		for map_inst in pyXactComp.memoryMaps.memoryMap:
			if map_inst.name == memMap:
				self.memMap = map_inst

		self.pyXactComp = pyXactComp		


	def commit_to_file(self, of, text):
		""" 
		Write a text into the output file
		Arguments:
			of			Open output file
			text		List of strings to write
		"""
		for line in text :
			of.write(line)
		
	
	
	def set_of(self, of):
		""" 
		Sets the output file to the internal output file of instance
		Arguments:
			of		Output file to set
		"""
		self.of = of
	
	
	def move_till_text(self, of, text):
		""" 
		Move till text in a file. The file must be opened for reading.
		Arguments:
			of			Output file
			text		Text until which to move in a file
		"""
		line = "BEGIN"
		while (line != None):
			line = of.read()
			if (line == text):
				break


	def get_regs_from_word(self, word_addr, block):
		"""
		Create list of registers within given memory word address
		"""
		regs_in_wrd = []
		for reg in sorted(block.register, key=lambda a: a.addressOffset):
			reg_wrd_addr = reg.addressOffset - reg.addressOffset % self.wrdWidthByte;

			if (reg_wrd_addr == word_addr):
				regs_in_wrd.append(reg)

			# Exit upon last possible register (further are for sure in higher 
			# words), no need to search further
			if (reg.addressOffset > (word_addr + self.wrdWidthByte)):
				break

		return regs_in_wrd


	def addr_reg_lookup(self, fieldReg):
		""" 
		Search the "memMap" for register with the same address offset aligned
		to memory word and return it.
		Arguments:
			fieldReg	Register from the field map to search for in the address
						map.
		"""
		for block in self.memMap.addressBlock:
			for reg in block.register:
				if (reg.addressOffset * 4 == fieldReg.addressOffset):
					return reg
		return None


	def align_addr_to_wrd(self, addr):
		"""
		Align address of a register to word address.
		"""
		return math.floor(addr - (addr % self.wrdWidthByte))


	def reg_is_access_type(self, reg, accesses):
		"""
		Check if register is explicitly of given access type. If input
		access type is sub-set of register access type, False is returned.
		E.g.:
			register access type: read-writeOnce
		    searched access type: write-Once
			False is returned		
		"""
		for access in accesses:
			if (access == reg.access):
				return True
		return False		


	def reg_has_access_type(self, reg, accesses):
		"""
		Check if register contains given access type. If input access type
		is sub-set of register acces type, True is returned.
		E.g.:
			register access type: read-writeOnce
		    searched access type: write-Once
			True is returned
		"""
		for access in accesses:
			if (access in reg.access):
				return True
		return False


	def is_reg_write_indicate(self, reg):
		"""
		Check if register contains at least one field which has write
		"modifyWriteValue" property set to modify.
		"""
		for field in sorted(reg.field, key=lambda a: a.bitOffset):
			if (field.modifiedWriteValue == "modify"):
				return True

		return False


	def is_reg_read_indicate(self, reg):
		"""
		Check if any field of a register has readAction set to "modify"
		This indicates that special signal which indicates read from a
		register should be placed. 
		"""
		for field in sorted(reg.field, key=lambda a: a.bitOffset):
			if (field.readAction == "modify"):
				return True

		return False


	def calc_blk_wrd_span(self, block, accesses=[""]):
		"""
		Calculate minimal address span for address block with registers of
		given access types.
		Returns:
			[low_addr, high_addr] - Lowest higher addresses within a block
				with registers of given access types.
		"""
		low_addr = block.range
		high_addr = 0

		for reg in block.register:
			if (not (self.reg_has_access_type(reg, accesses))):
				continue

			if (reg.addressOffset < low_addr):
				low_addr = self.align_addr_to_wrd(reg.addressOffset) 

			if (reg.addressOffset > high_addr):
				high_addr = self.align_addr_to_wrd(reg.addressOffset)

		return [low_addr, high_addr]


	def calc_blk_wrd_count(self, block, accesses=[""]):
		"""
		Calculate number of memory words in a block occupied by a registers
		of this block.
		Arguments:
			block       Block object
			accesses    List of register access types that should be considered.
						If not specified, every register is considered.
		"""
		cnt = 0
		highestWrd = -1
		for reg in sorted(block.register, key=lambda a: a.addressOffset):

			# Check if register is of given Access type
			if (not (self.reg_has_access_type(reg, accesses))):
				continue

			act_wrd = self.align_addr_to_wrd(reg.addressOffset)

			if (highestWrd < act_wrd):
				highestWrd = act_wrd
				cnt = cnt + 1

		return cnt


	def get_wrd_index(self, block, reg, accesses=[""]):
		"""
		Calculate index of memory word which contains given register. Take
		into account only registers with given access types.
		"""
		index = 0

		# Check each word in the memory block
		[low_addr, high_addr] = self.calc_blk_wrd_span(block, accesses)
		high_addr += self.wrdWidthByte

		for wrd_addr in range(low_addr, high_addr, self.wrdWidthByte):
			
			# Check that on this memory word, there is a register exisiting
			# with a given access type            
			for s_reg in sorted(block.register, key=lambda a: a.addressOffset): 

				# Skip registers whose access type we are not interested in
				if (not self.reg_has_access_type(s_reg, accesses)):
					continue;

				# Check that register resides within this word
				aligned_reg_addr = self.align_addr_to_wrd(s_reg.addressOffset)
				if (aligned_reg_addr == wrd_addr):
					index += 1
					break

			# If register word address is matching the actually searched
			# address, we the index holds the
			searched_addr = self.align_addr_to_wrd(reg.addressOffset)
			if (searched_addr == wrd_addr):
				return index

		return None


	def parameter_lookup(self, uid):
		"""
		Search for paramater in loaded IP-XACT component. Returns name of
		the parameter if found, false otherwise.
		"""
		for parameter in self.pyXactComp.parameters.parameter:
			if (parameter.parameterId == uid):
				return parameter.name

		return None


	def calc_addr_width_from_size(self, size):
		"""
		"""
		return math.ceil(math.log(size, 2))


	def calc_wrd_address_width(self, block):
		"""
		Calculate number of bits from address, necessary to address a word
		within an address block.
		"""
		addr_width = self.calc_addr_width_from_size(block.range * 8) - \
						self.calc_addr_width_from_size(self.wrdWidthBit)
		return addr_width

