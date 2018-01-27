################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   C header file generator for PyXact parsed obejects from 
##   IP-Xact specification 
##	
##	Revision history:
##		25.01.2018	First Implementation
##
################################################################################

import argparse
import sys
import time
import importlib.util
import os
import inspect
import math
import copy

from pyXact_generator.gen_lib import *
from pyXact_generator.languages.gen_base import BaseGenerator

class HeaderGenerator(BaseGenerator):
	
	def __init__(self):
		super().__init__()
		self.supportedTypes = ["int","char","unsigned char","signed char",
								"short","unsigned short","long","unsigned long",
								"uint8_t","uint16_t","uint32_t","uint64_t"]
		self.typeSizes = [32, 8, 8, 8, 16, 16, 64, 64, 8, 16, 32, 64]
		self.commentSign = "*"
	
	def is_supported_type(self, type):
		return super().is_supported_type(type)
	
	def __wr_line(self, line):
		super(HeaderGenerator, self).wr_line(line)
	
	def wr_nl(self):
		self.__wr_line("\n")
	
	def write_comm_line(self, gap=2):
		self.__wr_line('/*{:{fill}<78}\n'.format(" " * gap, fill=self.commentSign))	
	
	def write_comment(self, input, gap, caption=None, small=False):
		spltStr = split_string(input, 75 - gap)
		if (caption != None):
			spltStr.append("")
			spltStr.append(caption)
		elif (len(spltStr) == 1):
			self.__wr_line("{}/* {} */\n".format(" " * gap, input))
			return None
		
		self.__wr_line("{}/*\n".format(" " * gap))
		for (i,line) in enumerate(spltStr):
			self.__wr_line("{} * {}".format(" " * gap, line))
		self.__wr_line("{}*/\n".format(" " * gap))


	def create_enum(self, name, decls):
		self.__wr_line("enum {} {}\n".format(name, "{"))
		
		# Check the maximum string length in the enum
		maxLen = 0
		for item in decls:
			if (len(item.name) > maxLen):
				maxLen = len(item.name)
		
		# Write the enum
		for item in decls:
			pref = "	{}".format(item.name)
			if (item.value != None):
				post = " = {},\n".format(hex(item.value))
				post = '{:>{}}'.format(post, maxLen + 15 - len(pref))
			else:
				post = ",\n"
			self.__wr_line(pref + post)
			
		self.__wr_line("};\n")
	
	
	def write_decl(self, decl):
		if(self.is_supported_type(decl.type) == False):
			return False
		
		if (decl.specifier == None):
			intSpec = ""
		else:
			intSpec = decl.specifier + " "
			
		pref = "{}{}{}{}".format("	" * decl.gap, intSpec,
									decl.type + " ", decl.name)
		post = 	";\n"
		# Initialization of declaration
		if (decl.intType == "initialized"):
			post = " = {};\n".format(decl.value)
		elif (decl.intType == "enum"):
			post = " = {},\n".format(hex(decl.value))
		elif (decl.intType == "bitfield"):
			post = ": {};\n".format(decl.bitWidth)
			
		if (post != ";\n"):
			post = '{:>{}}'.format(post, decl.alignLen - len(pref))
		
		self.__wr_line(pref + post)
		return True


	def __write_structure_norm(self, decls):
		for decl in decls:
			self.write_decl(decl)


	def __insert_rsvd_bitfield(self, decls, neighbour, index):
		decl = copy.copy(neighbour)
		high = ""
		if (neighbour.bitIndex - index > 1):
			high = "_{}".format(neighbour.bitIndex - 1)
		decl.name = "reserved{}_{}".format(high, index)
		decl.bitIndex = index
		decl.bitWidth = neighbour.bitIndex - index
		decls.insert(decls.index(neighbour), decl)
		return index + decl.bitWidth 


	def __write_structure_bitfields(self, decls, bitFieldWidth):
		
		# Copy the list of declarations and insert the reserved values
		# check if we reached the full width in the end
		# Add one dummy element at the end of the list so that we dont have
		# to solve the special case of the interval between last element and
		# end of the bitfield
		tmp = copy.copy(decls)
		index = 0
		tmp.append(copy.copy(tmp[-1]))
		tmp[-1].bitIndex = bitFieldWidth
		for item in tmp:
			if (item.bitIndex > index):
				index = self.__insert_rsvd_bitfield(tmp, item, index)
			else:
				index = index + item.bitWidth
		tmp.remove(tmp[-1])
		
		# Write the bitfield values
		if (len(tmp) > 1):
			self.__wr_line("#ifdef __BIG_ENDIAN_BITFIELD\n")
		for decl in sorted(tmp , key=lambda a: a.bitIndex):
			self.write_decl(decl)
		if (len(tmp) > 1):
			self.__wr_line("#else\n")
			for decl in sorted(tmp , key=lambda a: a.bitIndex, reverse=True):
				self.write_decl(decl)
			self.__wr_line("#endif\n")

	
	def __get_type_size(self, type):
		if (not self.is_supported_type(type)):
			return 0
		
		for i,iter in enumerate(self.supportedTypes):
			if (iter == type):
				return self.typeSizes[i]
	
	
	def create_union(self, name, decls):
		
		self.__wr_line("{}union {} {}\n".format("	" * (decls[0].gap - 1),
							name, "{"))
		for decl in decls:	
			
			# Recursive
			if (type(decl) == list):
			
				# So far hardcoded that union contains structure for register
				# map... For future this will be changed with some kind of
				# joined type classification
				self.create_structure(name + "_s", decl)
			else:
				self.write_decl(decl)
		
		self.__wr_line("{}{};\n".format("	" * (decls[0].gap - 1), "}"))

	def create_structure(self, name, decls):
		self.__wr_line("{}struct {} {}\n".format("	" * (decls[0].gap - 1),
							name, "{"))
		if (decls[0].intType == "bitfield"):
			tmp = self.__get_type_size(decls[0].type)
			self.__write_structure_bitfields(decls, tmp)
			s = "s"
		else:
			s = ""
			self.__write_structure_norm(decls)
			
		self.__wr_line("{}{}{};\n".format("	" * (decls[0].gap - 1), 
							"}", s))
		
				
	def create_package(self, name, start=True):
		if (start):
			self.__wr_line("#ifndef __{}__\n".format(name))
			self.__wr_line("#define __{}__\n".format(name))
		else:
			self.__wr_line("#endif\n")
	
	
	def create_includes(self, includeList):
		for include in includeList:
			self.__wr_line("#include {}\n")
			
			

	
		
