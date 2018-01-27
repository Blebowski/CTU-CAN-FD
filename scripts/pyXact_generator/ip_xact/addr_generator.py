################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##
##	 Base class for language specific address Map generators.
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
		for line in text :
			of.write(line)
	
	def set_of(self, of): 
		self.of = of
		

################################################################################
#  Write the address map into output file
# 
# Arguments:
#  of		 	- Output file to write
################################################################################
	@abstractmethod
	def write_mem_map_addr(self):
		pass
	
	
################################################################################
# Write the bitfield map into the output file
#
# Arguments:
#  of		 	- Output file to write
################################################################################	
	@abstractmethod
	def write_mem_map_fields(self):
		pass


################################################################################
# Write both memory maps into the output file
#
# Arguments:
#  of		 	- Output file to write
################################################################################	
	@abstractmethod
	def write_mem_map_both(self):
		pass
		

################################################################################
# Write register fields constants of single register
#
# Arguments:
################################################################################		
	@abstractmethod
	def write_reg(self, reg, writeFields, writeRstVal, writeEnums): 
		pass