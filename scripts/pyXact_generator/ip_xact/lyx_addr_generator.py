################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##
##   Address map generator to Lyx document with constants.  
## 
##	Revision history:
##		25.01.2018	First implementation
##
################################################################################

from abc import ABCMeta, abstractmethod
from pyXact_generator.ip_xact.addr_generator import IpXactAddrGenerator

from pyXact_generator.languages.gen_lyx import LyxGenerator
from pyXact_generator.languages.declaration import LanDeclaration

class LyxAddrGenerator(IpXactAddrGenerator):

	lyxGen = None

	def __init__(self, pyXactComp, addrMap, fieldMap, busWidth):
		super().__init__(pyXactComp, addrMap, fieldMap, busWidth)
		self.lyxGen = LyxGenerator()
	
	
	def commit_to_file(self):
		for line in self.vhdlGen.out :
			self.of.write(line)
	


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
	
	
	