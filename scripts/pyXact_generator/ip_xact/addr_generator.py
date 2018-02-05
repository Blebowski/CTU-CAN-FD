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

class IpXactAddrGenerator(metaclass=ABCMeta):

	addrMap = None
	fieldMap = None
	busWidth = None
	
	of = None
	
	def __init__(self, pyXactComp, addrMap, fieldMap, busWidth):
		self.busWidth = busWidth
		if (not pyXactComp.memoryMaps):
			return None
		for map in pyXactComp.memoryMaps.memoryMap:
			if map.name == addrMap:
				self.addrMap = map
			if map.name == fieldMap:
				self.fieldMap = map
		
		
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
	
	
	def addr_reg_lookup(self, fieldReg):
		""" 
		Search the "addrMap" for register with the same address offset aligned
		to "busWidth" and return it.
		Arguments:
			fieldReg	Register from the field map to search for in the address
						map.
		"""
		for block in self.addrMap.addressBlock:
			for reg in block.register:
				if (reg.addressOffset * 4 == fieldReg.addressOffset):
					return reg
		return None


	@abstractmethod
	def write_mem_map_addr(self):
		""" 
		Write the address map into the generator output.
		"""
		pass
	
	
	@abstractmethod
	def write_mem_map_fields(self):
		""" 
		Write the register field map with reset values, bit indices and enums
		into the output generator.
		"""
		pass


	@abstractmethod
	def write_mem_map_both(self):
		""" 
		Write address map and field map into the generator output.
		"""
		pass
		

	@abstractmethod
	def write_reg(self, reg, writeFields, writeRstVal, writeEnums):
		""" 
		Write single register into the generator output.
		Arguments:
			writeFields		If fields indices should be written.
			writeRstVal		If Reset values should be written
			writeEnums		If Enum values should be written.
		"""
		pass