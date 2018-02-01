################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##
##   Base language generator class. Supports methods for basic source code
##   generation (declarations, enums, structures).
##
##	Revision history:
##		25.01.2018	First implementation
##
################################################################################

from abc import ABCMeta, abstractmethod
from pyXact_generator.languages.gen_base import BaseGenerator

class LanBaseGenerator(BaseGenerator):
	
	# Field of declaration supported types for given language generator
	supportedTypes = None
	
	# Bit sizes of the types defined in "supportedTypes"
	typeSizes = None
	
	def __init__(self):
		super().__init__()
		self.supportedTypes = []
		self.typesSizes = []


################################################################################
#	BaseGenerator inherited functions		
################################################################################		

	def wr_line(self, line):
		return super().wr_line(line)
	
	def append_line(self, line):
		return super().append_line(line)
	
	def wr_nl(self):
		return super().wr_nl()
	
	def commit_append_line(self, count):
		return super().commit_append_line(count)
		
	def commit_append_lines_all(self):
		return super().commit_append_lines_all()
		
		
################################################################################
#	Language generator specific functions which should be implemented
#	by each child and provide language specific way of implementing the
#	given construct.
################################################################################			

	def is_supported_type(self, type):
		for i in self.supportedTypes:
			if (type == i):
				return True
		print("{} is not supported type for {} class".format(
					type, self.__class__.__name__))
		return False
	
	@abstractmethod
	def write_comm_line(self, gap=2):
		pass

	@abstractmethod
	def write_comment(self, input, gap, caption=None, small=False):
		pass

	@abstractmethod
	def create_includes(self, includeList):
		pass
	
	@abstractmethod
	def write_decl(self, decl):
		pass
	
	@abstractmethod
	def create_package(self, name):
		pass

	@abstractmethod
	def create_structure(self, name, decls):
		pass
		
	@abstractmethod
	def create_enum(self, name, decls):
		pass	