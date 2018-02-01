################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   VHDL Generator Class for PyXact parsed obejects from IP-Xact specification 
##	
##	Revision history:
##		16.01.2018	Implemented the script
##
################################################################################

import argparse
import sys
import time
import importlib.util
import os
import inspect
import math

from pyXact_generator.gen_lib import *
from pyXact_generator.languages.gen_lan_base import LanBaseGenerator

class VhdlGenerator(LanBaseGenerator):
	
	
	def __init__(self):
		super().__init__()
		self.supportedTypes = ["std_logic", "natural"]
		self.commentSign = "-"
	

	def __wr_line(self, line):
		super(VhdlGenerator, self).wr_line(line)

	def is_supported_type(self, type):
		return super().is_supported_type(type)

	def write_comm_line(self, gap=2):
		self.__wr_line('{:{fill}<80}\n'.format(" " * gap, fill=self.commentSign))
	
	def write_comment(self, input, gap, caption=None, small=False):
		if (small == False):
			self.write_comm_line()
		
		if (caption != None):
			self.__wr_line(" " * gap + self.commentSign + self.commentSign + 
								" {}\n{}\n".format(caption," " *gap+"--"))
		
		lines = split_string(input, 75)
		for line in lines:
			self.__wr_line('{}{} {}\n'.format(" " * gap, self.commentSign *2 ,
							line))
		
		if (small == False):
			self.write_comm_line()
			

	def create_includes(self, includeList):
		if (includeList == None):
			return False
			
		self.__wr_line("Library ieee;\n")	
		for include in includeList:
			self.__wr_line("use ieee.{};\n".format(include))
	

	def write_decl(self, decl):
		if (not self.is_supported_type(decl.type)):
			return False
		
		strType = decl.type
		strVal = ""
		if (decl.type == "std_logic"):	
			if (decl.bitWidth > 1):
				strType = strType + "_vector"
				strType = strType + "({} downto 0)".format(decl.bitWidth - 1)
				
				if (decl.bitWidth % 4 == 0):
					fmt = ':0{}x'.format(math.ceil(float(decl.bitWidth/4)))
					hPref = 'x' 
				else:
					fmt = ':0{}b'.format(math.ceil(float(decl.bitWidth)))
					hPref = ''
				fmt = '{'+fmt+'}'
				strVal = hPref+'"'+fmt.format(int(decl.value)).upper()+'"'
				
				## For longer logic vectors use (OTHERS => )
				if ((decl.bitWidth > 8 and hPref == '') or
				   decl.bitWidth > 32):
					binStr = str(':0{}b'.format(decl.value))
					isBinaryEq = True
					for binChr in binStr[1:-1]:
						if (binChr != binStr[1]):
							isBinaryEq = False
					
					if (isBinaryEq):
						strVal = "(OTHERS => '{}')".format(binStr[1])
			else:
				strVal = "'{}'".format(decl.value)
		elif (decl.type == "natural"):
			strVal = decl.value
		
		pref = '  {} {}'.format(decl.specifier, decl.name.upper())			
		post = ' : {} := {};\n'.format(strType, strVal)
		post = '{:>{}}'.format(post, decl.alignLen-len(pref))
		newLineChar = ""
		if (len(pref) + len(post) > 80):
			newLineChar = "\n                "
		
		self.__wr_line(pref + newLineChar + post)
		return True


	def create_package(self, name):
		self.__wr_line("package {} is\n".format(name))
		self.append_line("end package;\n")

	
	def create_structure(self, name, decls):
		if (not decls):
			return False
		self.__wr_line("type {} is record\n".format(name))
		for decl in decls:
			self.write_decl(decl)
		sefl.__wr_line("end record;\n")
		return True


	def create_enum(self, name, decls):
		if (not decls):
			return False
		self.__wr_line("type {} is\n".format(name))
		for (i,item) in enumerate(decls):
			if (i == len(decls) - 1):
				self.__wr_line(item.name + '\n')
			else:
				self.__wr_line(item.name + ',\n')
		self.__wr_line(");")
		return True























	
