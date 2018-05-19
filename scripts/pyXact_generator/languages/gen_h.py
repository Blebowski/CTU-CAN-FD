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
from pyXact_generator.languages.gen_lan_base import LanBaseGenerator

class HeaderGenerator(LanBaseGenerator):
	
	def __init__(self):
		super().__init__()
		self.supportedTypes = ["int","char","unsigned char","signed char",
								"short","unsigned short","long","unsigned long",
								"uint8_t","uint16_t","uint32_t","uint64_t"]
		self.typeSizes = [32, 8, 8, 8, 16, 16, 64, 64, 8, 16, 32, 64]
		self.commentSign = "*"
	

################################################################################
#	LanBaseGenerator inherited function
################################################################################		
	
	def is_supported_type(self, type):
		return super().is_supported_type(type)
	
	def __wr_line(self, line):
		super(HeaderGenerator, self).wr_line(line)
	
	def wr_nl(self):
		self.__wr_line("\n")
	
	
################################################################################
#	C syntax specific generation functions
################################################################################			
	
	def write_comm_line(self, gap=2):
		""" 
		Write C comment line in format: (gap)/******/ aligned to 80 characters
		Arguments:
			gap		 Number of tabs before the comment line
		"""
		self.__wr_line('/*{:{fill}<78}\n'.format(" " * gap, 
							fill=self.commentSign))
	
	
	def write_comment(self, input, gap, caption=None, small=False):
		""" 
		Write C comment line aligned to 80 characters in format:
			/* COMMENT */
		or:
			/* 
			 * COMMENT LONGER THAN 80 CHARACTERS SPLIT BETWEEN LINES
			 */
		Arguments:
			gap		 Number of tabs before the comment
			caption  Caption to append to the comment in the first line
			small	 So far no meaning for C implementation
		"""
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
		""" 
		Create a C enum. Align to the maximum declaration name within the
		elements declaration list.
		Arguments:
			name		Enum name
			decls		List of enum elements as declarations
		"""
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
		""" 
		Create simple C declaration. This function supports:
			- simple variable declaration (initialized, non-initialized)
			- enum element declaration (value must be specified on declarations)
			- bitfield element declaration (bitWidth must be specified)
		Arguments:
			decl		Declaration to create
		"""
		if(self.is_supported_type(decl.type) == False):
			return False
		
		if (decl.specifier == None):
			intSpec = ""
		else:
			intSpec = decl.specifier + " "
		
		if (decl.comment != None):
			self.write_comment(decl.comment, decl.gap, small=True)
		
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
		""" 
		Write structure declaration without bitfields
		Arguments:
			decls		Structure elements
		"""
		for decl in decls:
			self.write_decl(decl)
			

	def __insert_rsvd_bitfield(self, decls, neighbour, index):
		""" 
		Fill the declarations list with reserved bitfields on missing indices.
		Arguments:
			decls		List of declarations to extend
			neighbour   Neighbour to copy the element declaration into the
						reserved field declaration
			index		Last filled bit by previous declaration
		Returns:
			Index of last filled bit with the new reserved bit field generated.
		"""
		decl = copy.copy(neighbour)
		high = ""
		if (neighbour.bitIndex - index > 1):
			high = "_{}".format(neighbour.bitIndex - 1)
		decl.name = "reserved{}_{}".format(high, index)
		decl.bitIndex = index
		decl.comment = None
		decl.bitWidth = neighbour.bitIndex - index
		decls.insert(decls.index(neighbour), decl)
		return index + decl.bitWidth 


	def __write_structure_bitfields(self, decls, bitFieldWidth):
		""" 
		Create structure declaration with bitfields.
		Arguments:
			decls			List of declarations describing the structure 
							elements
			bitFieldWidth 	Width of the bitfield to create
		"""
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
		
		# Write bitfield values
		if (len(tmp) > 1):
			self.__wr_line("#ifdef __LITTLE_ENDIAN_BITFIELD\n")
		for decl in sorted(tmp , key=lambda a: a.bitIndex):
			self.write_decl(decl)
		if (len(tmp) > 1):
			self.__wr_line("#else\n")
			for decl in sorted(tmp , key=lambda a: a.bitIndex, reverse=True):
				backUp = decl.comment
				decl.comment = None
				self.write_decl(decl)
				decl.comment = backUp
			self.__wr_line("#endif\n")

	
	def __get_type_size(self, type):
		""" 
		Get bit size of given type
		Arguments:
			type	Declaration type to get the size
		Returns:
			Bit size of the given type
		"""
		if (not self.is_supported_type(type)):
			return 0
		
		for i,iter in enumerate(self.supportedTypes):
			if (iter == type):
				return self.typeSizes[i]
	
	
	def create_union(self, name, decls):
		""" 
		Create union with declarations.
		Arguments:
			name	Union name
			decls	List with declarations contained within the union.
		"""
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
		""" 
		Create structure, either with or without bitfields.
		Arguments:
			name	structure name
			decls	List with declarations contained within the structure.
		"""
		self.__wr_line("{}struct {} {}\n".format("	" * (decls[0].gap - 1),
							name, "{"))
		if (decls[0].intType == "bitfield"):
			tmp = self.__get_type_size(decls[0].type)
			self.__write_structure_bitfields(decls, tmp)
			s = " s"
		else:
			s = ""
			self.__write_structure_norm(decls)
			
		self.__wr_line("{}{}{};\n".format("	" * (decls[0].gap - 1), 
							"}", s))
		
				
	def create_package(self, name):
		""" 
		Create C header file construct like:
			 #ifndef "name"
			 #define "name"
		Arguments:
			name	ifdef name to create
		"""
		self.__wr_line("#ifndef __{}__\n".format(name))
		self.__wr_line("#define __{}__\n".format(name))
		self.append_line("#endif\n")
	
	
	def create_includes(self, includeList):
		""" 
		Create C header includes.
		Arguments:
			includeList		List of C includes
		"""
		for include in includeList:
			self.__wr_line("#include {}\n")
			
			

	
		
