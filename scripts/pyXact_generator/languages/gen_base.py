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

class BaseGenerator(metaclass=ABCMeta):
	
	supportedTypes = None
	typeSizes = None
	out = None
	commentSign = None
	packageNames = None
	packageIdentifier = "%$%$@_"
	
	
	def __init__(self):
		self.out = []
		self.packageNames = []
	
	def wr_line(self, line):
		self.out.append(line)
	
	def wr_nl(self):
		self.wr_line("\n")
		
	def is_supported_type(self, type):
		for i in self.supportedTypes:
			if (type == i):
				return True
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
	def create_package(self, name, start=True):
		pass

	@abstractmethod
	def create_structure(self, name, decls):
		pass
		
	@abstractmethod
	def create_enum(self, name, decls):
		pass	