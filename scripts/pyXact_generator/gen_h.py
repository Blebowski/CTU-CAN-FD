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

from pyXact_generator.gen_lib import *
from pyXact_generator.gen_base import baseGenerator

class headerGenerator(baseGenerator):
	
	def __init__(self, addrMap, fieldMap, busWidth):
		super().__init__(addrMap, fieldMap, busWidth)
	
	def is_supported_type(self, type):
		if ((type == "int") or 
		    (type == "char") or
		    (type == "unsigned char") or
		    (type == "signed char") or
		    (type == "short") or
		    (type == "unsigned short") or
		    (type == "long") or
		    (type == "unsigned long") or
		    (type == "uint8_t") or
		    (type == "uint16_t") or
		    (type == "uint32_t") or
		    (type == "uint64_t") or
		    (type ==  "unsigned char")):
		    return True
		else:
			return False

	def write_comment(self, of, input, gap):
		strGap = " " * gap
		spltStr = split_string(input, 75)
		comSign = ["/*"," *"," */"]
		actGap = strGap
		
		if (len(spltStr) > 1):
			multiLine = True
		else:
			multiLine = False
		
		of.write("{}/*".format(strGap))
		if (multiLine):
			of.write("\n")
			comSign = " *"
		else:
			actGap = ""
			comSign = ""
			
		for line in spltStr:
			of.write('{}{} {} '.format(actGap, comSign, line))
			if (multiLine):
				of.write("\n")

		if (multiLine):
			actGap = " " + actGap
			
		of.write("{}*/\n".format(actGap))


	def write_prologue(self, of, name, comment):
		of.write("\n")
		self.write_comment(of, comment, 0)
		of.write("\n")
		tmpList = ["#indef", "#define"]
		for pref in tmpList:
			of.write("{} __{}_DEFS_H__\n".format(pref, name.upper()))
		of.write("\n")

	def write_epilogue(self, of, name):
		of.write('\n#endif')


	def write_decl(self, of, specifier, name, bitWidth, type, value, alignLen):
		if(self.is_supported_type(type) == False):
			return False
		
		strVal = hex(value)
		strType = type
		tmp = 0
		
		pref = '  {} {}'.format(specifier, name)			
		post = ' = {},\n'.format(strVal)
		tmp = alignLen - len(pref)
		if (tmp < 0):
			tmp = 0
		post = '{:>{}}'.format(post, tmp)
		
		of.write(pref + post)
		return True


	def write_bitfield(self, of, type, name, width, alignLen):
		tmp = 0
		
		pref = "		{} {}".format(type, name)
		post = ": {};\n".format(width)
		tmp = alignLen - len(pref)
		if (tmp < 0):
			tmp = 0
		post = '{:>{}}'.format(post, tmp)
		
		of.write(pref + post)
		return pref+post


	def write_bitfield_gap(self, of, type, lower, upper, alignLen):
		gap = upper - lower
		if (gap == 1):
			rsvdName = "reserved_" + str(lower)
		elif (gap > 1):
			rsvdName = "reserved_" + str(upper-1) + "_" + str(lower)
		return self.write_bitfield(of, type, rsvdName, gap, alignLen)
		

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
	
	
	
	def write_reg(self, of, reg, outBuffer):
		type = "uint{}_t".format(self.busWidth)
		of.write("		//{}\n".format(reg.name))
		prevIndex = 0
		
		for field in sorted(reg.field, key=lambda a: a.bitOffset):
			gap = field.bitOffset - prevIndex
			
			if (gap > 0):
				lower = prevIndex + ((reg.addressOffset*8) % self.busWidth)
				upper = lower + gap
				outBuffer.append(self.write_bitfield_gap(of, type, lower, upper, 35))
				
			outBuffer.append(self.write_bitfield(of, type, field.name.lower(), 
								field.bitWidth, 35))
			prevIndex = field.bitOffset + field.bitWidth
		
		if (prevIndex != reg.size):
			lower = prevIndex + ((reg.addressOffset*8) % self.busWidth)
			upper = lower + reg.size - prevIndex
			outBuffer.append(self.write_bitfield_gap(of, type, lower, upper, 35))
		
		# Append the name for the reverse order
		outBuffer.append("		//{}\n".format(reg.name))
		
		return outBuffer
			
		# Write the register title
		#comLine = '{:{fill}<80}\n'.format("  ", fill="-",)
		#of.write(comLine)
		#of.write('  -- {} register\n'.format(reg.name.upper()))
		#of.write('  --\n')
		#self.write_comment(of, reg.description, 2)
		#of.write(comLine)
		
		#Write the individual elements
		#for field in reg.field:
		#	self.write_reg_field(of, field, reg, busWidth)
		
		#Write the enums (iterate separately not to mix up fields and enums)
		#for field in reg.field:
		#	self.write_reg_enums(of, field)
		
		#Write reset values for each field
		#of.write('\n  --{} reset values\n'.format(reg.name))
		#for field in reg.field:
		#	self.write_res_vals(of, field)
			
		#of.write('\n')
	
	
	def write_reg_group(self, of, regGroup, addrReg):
		
		of.write("union {} {}\n".format(addrReg.name.lower(), "{"))
		of.write("	uint{}_t u{};\n".format(self.busWidth, self.busWidth))
		of.write("	struct {} {}\n".format(addrReg.name.lower()+"_s", "{"))
		of.write("#ifdef __BIG_ENDIAN_BITFIELD\n")

		# Write the Big endian fields
		rev = []
		for (i, reg) in enumerate(sorted(regGroup, 
								key=lambda a: a.addressOffset)):
			rev = self.write_reg(of, reg, rev)
			if (i != len(regGroup) - 1):
				of.write("\n")
				rev.append("\n")
		
		of.write("#else\n")

		for line in reversed(rev):
			of.write(line)
		#for reg in sorted(regGroup, key=lambda a: a.addressOffset):
		#	self.write_reg(of, reg)
		
		of.write("#endif\n")
		of.write("	} s;\n")
		of.write("};\n\n")
			
	
	
	def addr_reg_lookup(self, fieldReg):
		for block in self.addrMap.addressBlock:
			#print("Looking for addres: {} -register {}".format(fieldReg.addressOffset, fieldReg.name))
			for reg in block.register:
				if (reg.addressOffset*4 == fieldReg.addressOffset):
					print("Found reg: %s" % reg.name)
					return reg
		return None
	
	
	def write_regs(self, of, regs, busWidth):
		regGroups = [[]]
		lowInd = 0
		
		# Sort the registers from field map into sub-lists	
		for reg in sorted(regs, key=lambda a: a.addressOffset):
			
			# We hit the register aligned create new group
			if (reg.addressOffset >= lowInd + self.busWidth/8):
				lowInd = reg.addressOffset - reg.addressOffset % 4
				regGroups.append([])
	
			regGroups[-1].append(reg)
		
		# Debug info
		#for i,regGroup in enumerate(regGroups):
		#	print("Reg group nr: %s" % i)
		#	for reg in regGroup:
		#		print(reg.name)
		#	print("\n")
		
		for regGroup in regGroups:
			self.write_reg_group(of, regGroup, self.addr_reg_lookup(regGroup[0]))

				
	
	def write_mem_map_fields(self, of, memoryMap, busWidth):
		for block in self.fieldMap.addressBlock:
			self.write_regs(of, block.register, busWidth)


	def write_addrbl_head(self, of, addressBlock):
		self.write_comment(of, addressBlock.name, 0)


	def write_addrbl_regs(self, of, addressBlock):
		for reg in sorted(addressBlock.register, key=lambda a: a.addressOffset):
			self.write_decl(of, "	", reg.name.upper(), 0, "int",
						reg.addressOffset+addressBlock.baseAddress, 40)
	
	
	def write_mem_map_addr(self, of, memoryMap):
		
		cmnt = "{} memory map".format(self.addrMap.name)
		self.write_comment(of, cmnt, 0)
		of.write("enum {} {}\n".format(self.addrMap.name,'{'))
		
		for block in self.addrMap.addressBlock:
			of.write("\n")
			self.write_addrbl_head(of, block)
			self.write_addrbl_regs(of, block)
		
		of.write("};\n")
	
		
