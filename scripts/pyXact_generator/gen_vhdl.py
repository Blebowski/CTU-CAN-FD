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
from pyXact_generator.gen_base import baseGenerator

class vhdlGenerator(baseGenerator):
	
	def is_supported_type(self, type):
		if ((type == "std_logic") or (type == "natural")):
		    return True
		else:
			return False


	def write_comment(self, of, input, gap):
		strGap = " " * gap
		spltStr = split_string(input, 75)
		for line in spltStr:
			of.write('{}-- {}\n'.format(strGap, line))

		
	def write_prologue(self, of, name, comment):
		comLine = '{:{fill}<80}\n'.format("", fill="-",)
		purp = '-- Purpose:\n'
		of.write(comLine + purp)
		self.write_comment(of, comment, 0)
		of.write(comLine+'\n')
		of.write('Library ieee;\n')
		of.write('use ieee.std_logic_1164.all;\n\n')
		of.write('package {} is\n\n'.format(name))


	def write_epilogue(self, of, name):
		of.write('end package;')


	def write_decl(self, of, specifier, name, bitWidth, type, value, alignLen):
		if(self.is_supported_type(type) == False):
			return False
		
		spec = ""
		strType = type
		strVal = ""
		if (type == "std_logic"):	
			if (bitWidth > 1):
				strType = strType + "_vector"
				strType = strType + "({} downto 0)".format(bitWidth - 1)
				
				if (bitWidth % 4 == 0):
					fmt = ':0{}x'.format(math.ceil(float(bitWidth/4)))
					hPref = 'x' 
				else:
					fmt = ':0{}b'.format(math.ceil(float(bitWidth)))
					hPref = ''
				fmt = '{'+fmt+'}'
				strVal = hPref+'"'+fmt.format(int(value)).upper()+'"'
				
				## For longer logic vectors use (OTHERS => )
				if ((bitWidth > 8 and hPref == '') or
				   bitWidth > 32):
					binStr = str(':0{}b'.format(value))
					isBinaryEq = True
					for binChr in binStr[1:-1]:
						if (binChr != binStr[1]):
							isBinaryEq = False
					
					if (isBinaryEq):
						strVal = "(OTHERS => '{}')".format(binStr[1])
			else:
				strVal = "'{}'".format(value)
		elif (type == "natural"):
			strVal = value
		
		pref = '  {} {}'.format(specifier, name.upper())			
		post = ' : {} := {};\n'.format(strType, strVal)
		post = '{:>{}}'.format(post, alignLen-len(pref))
		newLineChar = ""
		if (len(pref) + len(post) > 80):
			newLineChar = "\n                "
		
		of.write(pref + newLineChar + post)
		return True


	def write_reg_enums(self, of, field):
		if (field.enumeratedValues == []):
			return False
		
		of.write('\n')
		of.write('  -- "{}" field enumerated values\n'.format(field.name))
		for es in field.enumeratedValues:
			for e in sorted(es.enumeratedValue, key=lambda x: x.value):
				self.write_decl(of, "constant", e.name, field.bitWidth, 
							"std_logic", e.value, 50)

				
	def write_res_vals(self, of, field):
		if (field.resets == None):
			return False
		
		if (field.resets.reset == None):
			return False
		
		self.write_decl(of, "constant", field.name+"_RSTVAL", field.bitWidth,
						"std_logic", field.resets.reset.value, 50)


	def write_reg_field(self, of, field, reg, busWidth):
		bitIndexL = field.bitOffset
		bitIndexH = field.bitOffset + field.bitWidth-1
	
		bitIndexL = bitIndexL + ((reg.addressOffset*8) % busWidth)
		bitIndexH = bitIndexH + ((reg.addressOffset*8) % busWidth)
		
		if (bitIndexH == bitIndexL):
			iter = [["_IND", bitIndexL]]
		else:
			iter = [["_L", bitIndexL], ["_H", bitIndexH]]
			
		for item in iter:
			self.write_decl(of, "constant", field.name+item[0], field.bitWidth,
						"natural", item[1], 50)
	
	
	def write_reg(self, of, reg, busWidth):
		
		# Write the register title
		comLine = '{:{fill}<80}\n'.format("  ", fill="-",)
		of.write(comLine)
		of.write('  -- {} register\n'.format(reg.name.upper()))
		of.write('  --\n')
		self.write_comment(of, reg.description, 2)
		of.write(comLine)
		
		#Write the individual elements
		for field in reg.field:
			self.write_reg_field(of, field, reg, busWidth)
		
		#Write the enums (iterate separately not to mix up fields and enums)
		for field in reg.field:
			self.write_reg_enums(of, field)
		
		#Write reset values for each field
		of.write('\n  --{} reset values\n'.format(reg.name))
		for field in reg.field:
			self.write_res_vals(of, field)
			
		of.write('\n')
		

	
	def write_regs(self, of, regs, busWidth):
		for reg in sorted(regs, key=lambda a: a.addressOffset):
			self.write_reg(of, reg, busWidth)
	
	
	def write_mem_map_fields(self, of, memoryMap, busWidth):
		for block in memoryMap.addressBlock:
			self.write_regs(of, block.register, busWidth)



	def write_addrbl_head(self, of, addressBlock):
		# Write capital comment with name of the address Block
		comLine = '{:{fill}<78}\n'.format("  ", fill="-",)
		com = '  -- Address block: {}\n'.format(addressBlock.name)
		of.write(comLine+com+comLine)
	
		# Write the VHDL constant for Address block offset defined as:
		#	block.baseAddress/block.range
		bitWidth = 4 # TODO: So far not bound to IP-XACT
		self.write_decl(of, "constant", addressBlock.name+"_BLOCK", bitWidth, 
						"std_logic",
						addressBlock.baseAddress/addressBlock.range, 80)
		of.write("\n")


	def write_addrbl_regs(self, of, addressBlock):
		for reg in sorted(addressBlock.register, key=lambda a: a.addressOffset):
			self.write_decl(of, "constant", reg.name+"_ADR", 12, "std_logic",
						reg.addressOffset+addressBlock.baseAddress, 80)
	
	
	def write_mem_map_addr(self, of, memoryMap):
		#Each Address block reflects to VHDL memory region
		for block in memoryMap.addressBlock:
			self.write_addrbl_head(of, block)
			self.write_addrbl_regs(of, block)
			of.write("\n\n")
		
