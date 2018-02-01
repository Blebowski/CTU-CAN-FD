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
	
	# Text to be printed at the end of declaration which is multiline
	# and contains more objects (like declarations) within it...
	appendText = None
	
	def __init__(self):
		self.out = []
		self.packageNames = []
		self.appendText = []
	
	def wr_line(self, line):
		self.out.append(line)
	
	def append_line(self, line):
		self.appendText.insert(0, line)
	
	def wr_nl(self):
		self.wr_line("\n")
	
	def commit_append_line(self, count):
		for i in range(0, min(count, len(self.appendText))):
			self.wr_line(self.appendText[0])
			self.appendText.pop(0)
	
	def commit_append_lines_all(self):
		self.commit_append_line(len(self.appendText))
		
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
	def create_package(self, name):
		pass

	@abstractmethod
	def create_structure(self, name, decls):
		pass
		
	@abstractmethod
	def create_enum(self, name, decls):
		pass	