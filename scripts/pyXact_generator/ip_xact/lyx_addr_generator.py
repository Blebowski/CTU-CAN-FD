################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##
##   Address map generator to Lyx document from IP-XACT parsed memory map
##	 with pyXact framework.
## 
##	Revision history:
##		25.01.2018	First implementation
##
################################################################################

import math

from abc import ABCMeta, abstractmethod
from pyXact_generator.ip_xact.addr_generator import IpXactAddrGenerator

from pyXact_generator.languages.gen_lyx import LyxGenerator
from pyXact_generator.languages.declaration import LanDeclaration

from pyXact_generator.gen_lib import *

class LyxAddrGenerator(IpXactAddrGenerator):

	lyxGen = None
	template = None

	genFieldDesc = None
	genRegions = None

	def __init__(self, pyXactComp, memMap, wrdWidthBit, genRegions=True,
					genFiDesc=True):
		super().__init__(pyXactComp, memMap, wrdWidthBit)

		self.lyxGen = LyxGenerator()

		self.genFieldDesc = str_arg_to_bool(str(genFiDesc))
		self.genRegions = str_arg_to_bool(str(genRegions))
	
	
	def commit_to_file(self):
		for line in self.lyxGen.out :
			self.of.write(line)	


	def reg_append_short_enums(self, field):
		"""
		"""
		appendText = ""
		if (field.enumeratedValues == []):
			return appendText

		if (len(field.enumeratedValues[0].enumeratedValue) > 0):
			for es in field.enumeratedValues:
				for (i,e) in enumerate(sorted(es.enumeratedValue, key=lambda x: x.value)):
					appendText += "\\begin_inset Newline newline\\end_inset\n"
					binSize = "{:0" + "{}".format(field.bitWidth) + "b}"
					binFmt = binSize.format(e.value)
					appendText += "		0b{}  - {} - {}".format(binFmt, e.name, 
						e.description)
		return appendText		


	def write_reg_field_desc(self, reg):
		"""
		"""
		for field in sorted(reg.field, key=lambda a: a.bitOffset):
			self.lyxGen.insert_layout("Description")
			descText = field.description
			descText += self.reg_append_short_enums(field)
			self.lyxGen.wr_line("{} {}\n".format(field.name, descText))
			self.lyxGen.commit_append_line(1)


	def getBit(self, val, bitIndex):
		"""
		"""
		tmp = "{0:032b}".format(val)
		return tmp[31 - bitIndex]
	
	
	def reg_unwrap_fields(self, reg):
		"""
		"""
		retVal = [[], [], [], []]
		subRegIndex = 0
		highVal = 0
		
		for i in range(0, int(reg.size / 8)):
			for j in range(0, 8):
				retVal[i].append([])
				
				# Check if such a field exists
				fieldExist = False
				for field in sorted(reg.field, key=lambda a: a.bitOffset):
					tmp = (7 - j) + i * 8
					if (field.bitOffset <= tmp and
						field.bitOffset + field.bitWidth > tmp):
						fieldExist = True
						break;
				
				# Insert the field or reserved field
				if (fieldExist):
					fieldName = field.name
					if (field.resets != None and field.resets.reset != None):
						fieldRst = self.getBit(field.resets.reset.value, 
											tmp - field.bitOffset)
					else:
						fieldRst = "X"
					
					# If the field is overllaped over several 8 bit registers
					# add index to define it more clearly
					if (int(field.bitOffset / 8) != 
						int((field.bitOffset + field.bitWidth - 1) / 8)):
						hInd = min(field.bitOffset + field.bitWidth - 1,
									((i + 1) * 8) - 1)
						lInd = max(field.bitOffset, (i * 8))
						hInd = hInd - field.bitOffset
						lInd = lInd - field.bitOffset
						append = "[{}".format(hInd)
						if (hInd != lInd):
							append += ":{}]".format(lInd)
						else:
							append += "]"
						fieldName = fieldName + append
				else:
					fieldName = "Reserved"
					fieldRst = "-"
				
				
				retVal[i][j].append(fieldName)
				retVal[i][j].append(fieldRst)	
		return retVal		
	
	
	def merge_common_fields(self, table, rowIndices, startCol=0, endCol=None):
		"""
		"""
		prevName = ""
		multiOpts = []
		if (endCol == None):
			endCol = len(table[1][0])
		for i,row in enumerate(table[1]):
			if (i in rowIndices):
				highInd = 0
				lowInd = 32
				for j,cell in enumerate(row):
					if (j >= startCol and j <= endCol):
						multicolumn = (prevName == cell[2])
						if (multicolumn):
							mcVal = "2"
							lowInd = j
						else:
							mcVal = "1"
							highInd = j
						prevName = cell[2]
						self.lyxGen.set_cell_option(table, i, j, "multicolumn", 
							mcVal)
					
					# Set the right panel if end is present
					if (lowInd == len(row) - 1):
						self.lyxGen.set_cell_option(table, i, highInd,
								"rightline", "true")
			

	def write_reg_field_table(self, reg):
		"""
		"""
		
		regFields = self.reg_unwrap_fields(reg)
		
		for i in reversed(range(1, int(reg.size / 8 + 1))):
			table = self.lyxGen.build_table(9, 3)
			
			# Set the width
			self.lyxGen.set_columns_option(table, range(1,9),  
							[["width", "1.4cm"]  for j in range(1, 9)])
			
			rows = [[row, j + 1] for j in range(8) for row in range(3)]
			
			# Title row
			self.lyxGen.set_cell_object(table, 0, 0, "Bit index")
			bitIndexes = [str((8 * i) - j) for j in range(1,9)]
			self.lyxGen.set_cells_object(table, [[0, j + 1] for j in range(8)],
												bitIndexes)
			
			# Field name row
			self.lyxGen.set_cell_object(table, 1, 0, "Field name")
			cells = [[1, j + 1] for j in range(8)]
			fieldNames = [regFields[i - 1][j][0] for j in range(8)]
			self.lyxGen.set_cells_object(table, cells, fieldNames)
			
			# Restart value row
			self.lyxGen.set_cell_object(table, 2, 0, "Reset value")
			cells = [[2, j + 1] for j in range(8)]
			rstVals = [regFields[i - 1][j][1] for j in range(8)]
			self.lyxGen.set_cells_object(table, cells, rstVals)
			
			# Merge adjacent fields with the same names
			self.merge_common_fields(table, [1], startCol=1)
				
			self.lyxGen.insert_table(table)
	

	def write_regs(self, block):
		"""
		"""
		# Memory type blocks dont need to be described by field! We use it
		# to express mapping to other registers and thus It means we dont 
		# want unnecessary words described!
		if (block.usage == "memory"):
			return
	
		for reg in sorted(block.register, key=lambda a: a.addressOffset):
			
			# Add the Section title
			self.lyxGen.write_layout_text("Subsection", "{}\n".format(reg.name),
												label="label")
			
			
			# Register type, address, size and description
			self.lyxGen.write_layout_text("Description", "Type: {}\n".format(
												reg.access))
			self.lyxGen.write_layout_text("Description", "Address: {}\n".format(
										"0x{:X}".format(reg.addressOffset +
											block.baseAddress)))
			pluralAp = "s" if (reg.size > 8) else ""
			self.lyxGen.write_layout_text("Description", "Size: {} byte{}\n".format(
										int(reg.size / 8), pluralAp))
			self.lyxGen.write_layout_text("Standard", "{}\n".format(
											reg.description))
			
			# Bit table and bit field descriptions
			if (self.genFieldDesc == True):
				self.write_reg_field_table(reg)
				self.write_reg_field_desc(reg)
				
			
			# Separation from next register
			self.lyxGen.insert_layout("Standard")
			self.lyxGen.insert_inset("VSpace bigskip")
			self.lyxGen.commit_append_line(2)
			
	
	def write_mem_map_title(self):
		"""
		"""
		self.lyxGen.write_layout_text("Chapter", "{}\n".format(
											self.memMap.displayName), label="label")
		
		self.lyxGen.write_layout_text("Standard", "{}\n".format(self.memMap
										.description))


	def write_mem_map_regions(self, memMap):
		"""
		"""
		table = self.lyxGen.build_table(2, len(memMap.addressBlock) + 1)
		self.lyxGen.set_columns_option(table, range(0,2),  
							[["width", "4cm"]  for j in range(0, 2)])
		
		titleCells = [[0, 0] , [0, 1]]
		nameCells = [[i, 0] for i in range(1, len(memMap.addressBlock) + 1)]
		addrCells = [[i, 1] for i in range(1, len(memMap.addressBlock) + 1)]
		
		nameVals = [block.displayName for block in memMap.addressBlock]
		addrVals = ["0x{:03X}".format(block.baseAddress) for block in memMap.addressBlock]
		titleVals = ["Memory region", "Address offset"]
		
		self.lyxGen.set_cells_object(table, nameCells, nameVals)
		self.lyxGen.set_cells_object(table, addrCells, addrVals)
		self.lyxGen.set_cells_object(table, titleCells, titleVals)
		self.lyxGen.insert_table(table)
	
	
	def calc_block_table_len(self, block):
		"""
		"""
		marks = [0] * (int (block.range / (block.width / 8) ))
		for reg in sorted(block.register, key=lambda x: x.addressOffset):
			marks[int((reg.addressOffset * 8) / self.wrdWidthBit)] = 1
		len = 0
		change = True
		for mark in marks:
			if (mark == 1 or change == True):
				len += 1
			change = True if (mark == 1) else False
		return len
		
	
	def write_mem_map_reg_single(self, table, reg, row):
		"""
		"""
		cells = []
		begOff = int(reg.addressOffset % 4)
		for i in range(begOff, begOff + int(reg.size / 8)):
			cells += [[row, 3 - i]]
		text = [reg.name for i in range(self.wrdWidthByte)]
		self.lyxGen.set_cells_object(table, cells, text)
		self.lyxGen.set_cells_text_label(table, cells, ["hyperref" for i in
											range(0, len(cells))])	
		
	
	def write_mem_map_reg_table(self, block):
		"""
		"""
		self.lyxGen.write_layout_text("Section", "{}\n".format(
										block.displayName))
		tableLen = self.calc_block_table_len(block)
		table = self.lyxGen.build_table(5, tableLen + 1)
		
		self.lyxGen.write_layout_text("Standard", block.description)
		
		# Create the header
		cells = [[0, i] for i in range(5)]
		text = ["Bits [{}:{}]".format((i + 1) * 8 - 1, i * 8) 
					for i in reversed(range(0, self.wrdWidthByte))]
		text += ["Address offset"]
		self.lyxGen.set_cells_object(table, cells, text)
		
		self.lyxGen.set_columns_option(table, range(0,4),  
							[["width", "3cm"]  for j in range(0, 4)])
		self.lyxGen.set_column_option(table, 4, "width", "1.5cm")
		
		# Pre write the addresses with "..." for reserved fields
		cells = [[i + 1, 4] for i in range(tableLen)]
		text = ["..." for i in range(tableLen)]
		self.lyxGen.set_cells_object(table, cells, text)
		
		# Write the registers and addresses
		row = 1
		addr = 0
		for reg in sorted(block.register, key=lambda x: x.addressOffset):
				regDiff = math.floor(reg.addressOffset / 4) - addr
				if (regDiff == 1):
					row += 1
				elif (regDiff > 1):
					row += 2
				addr += regDiff
				self.write_mem_map_reg_single(table, reg, row)
				self.lyxGen.set_cell_object(table, row, 4, 
							"0x{:X}".format(4 * math.floor(reg.addressOffset / 4) +
											 block.baseAddress))				
		
		self.merge_common_fields(table, [i for i in range(1, tableLen + 1)],
									endCol=4)
				
		self.lyxGen.insert_table(table)
		


################################################################################
#  Write the memory map region overview for given memory map
################################################################################
	def write_mem_map_addr(self):
		if (self.genRegions):
			self.write_mem_map_regions(self.memMap)
		
			
################################################################################
# Write the bitfield map into the output file
################################################################################	
	def write_mem_map_fields(self):
		for block in self.memMap.addressBlock:
			self.lyxGen.insert_new_page()
			self.write_mem_map_reg_table(block)
			self.write_regs(block)


################################################################################
# Write both memory maps into the output file
#
# Arguments:
#  of		 	- Output file to write
################################################################################	
	def write_mem_map_both(self):
		self.write_mem_map_title()
		self.write_mem_map_addr()
		self.write_mem_map_fields()
		

################################################################################
# Write register fields constants of single register
#
# Arguments:
################################################################################		
	def write_reg(self, reg, writeFields, writeRstVal, writeEnums): 
		pass
	
	
	
	
	def load_lyx_template(self, path):
		self.template = open(path, 'r')
		lines = self.template.readlines()
		for i,line in enumerate(lines):
			self.lyxGen.wr_line(line)
			if (line == "\\begin_body\n"):
				break
		self.lyxGen.append_line("\end_document\n")
		self.lyxGen.append_line("\end_body\n")
