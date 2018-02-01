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
	template = None

	def __init__(self, pyXactComp, addrMap, fieldMap, busWidth):
		super().__init__(pyXactComp, addrMap, fieldMap, busWidth)
		self.lyxGen = LyxGenerator()
	
	
	def commit_to_file(self):
		for line in self.lyxGen.out :
			self.of.write(line)	


	def reg_append_short_enums(self, field):
		appendText = ""
		if (field.enumeratedValues == []):
			return appendText

		print(field.enumeratedValues[0].enumeratedValue)
		if (len(field.enumeratedValues[0].enumeratedValue) < 3 and
			len(field.enumeratedValues[0].enumeratedValue) > 0):
			appendText += "("
			sep = ", "
			for es in field.enumeratedValues:
				for (i,e) in enumerate(sorted(es.enumeratedValue, key=lambda x: x.value)):
					if (i == len(field.enumeratedValues[0].enumeratedValue)):
						sep = ""
					appendText += "{} - {}{}".format(e.value, e.name, sep)
			appendText += ")"
		return appendText		


	def write_reg_field_desc(self, reg):
		for field in sorted(reg.field, key=lambda a: a.bitOffset):
			self.lyxGen.insert_layout("Description")
			descText = field.description
			descText += self.reg_append_short_enums(field)
			self.lyxGen.wr_line("{} {}\n".format(field.name, descText))
			self.lyxGen.commit_append_line(1)


	def getBit(self, val, bitIndex):
		tmp = "{0:032b}".format(val)
		#print(tmp)
		#print("BitIndex {}".format(bitIndex))
		return tmp[31 - bitIndex]
	
	
	def reg_unwrap_fields(self, reg):
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
						print(field.name)
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
	
	
		
	def write_reg_field_table(self, reg):
		
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
			
			# Merge the cells for common bitfields ...
			prevName = ""
			multiOpts = []
			highInd = 9
			for j in range(1, 9):
				multicolumn = (prevName == regFields[i - 1][j - 1][0])
				if (multicolumn):
					multiOpts.append(["multicolumn", "2"])
				else:
					multiOpts.append(["multicolumn", "1"])
					highInd = j
				prevName = regFields[i - 1][j - 1][0]
			self.lyxGen.set_cells_option(table, cells, multiOpts)
			self.lyxGen.set_cell_option(table, 1, highInd, "rightline", "true")
			
			# Restart value row
			self.lyxGen.set_cell_object(table, 2, 0, "Reset value")
			cells = [[2, j + 1] for j in range(8)]
			rstVals = [regFields[i - 1][j][1] for j in range(8)]
			self.lyxGen.set_cells_object(table, cells, rstVals)
					
			self.lyxGen.insert_table(table)
	

	def write_regs(self, regs):
		for reg in sorted(regs, key=lambda a: a.addressOffset):
			
			# Add the Section title
			self.lyxGen.insert_layout("Subsection")
			self.lyxGen.wr_line("{}\n".format(reg.name))
			# TODO: Add Label here!
			self.lyxGen.commit_append_line(1)
			
			# Register type
			self.lyxGen.insert_layout("Description")
			self.lyxGen.wr_line("Type: {}\n".format(reg.access))
			self.lyxGen.commit_append_line(1)
			
			# Register address
			self.lyxGen.insert_layout("Description")
			self.lyxGen.wr_line("Address: {}\n".format(hex(reg.addressOffset).upper()))
			self.lyxGen.commit_append_line(1)
			
			# Size
			self.lyxGen.insert_layout("Description")
			pluralAp = ""
			if (reg.size > 8):
				pluralAp = "s"
			self.lyxGen.wr_line("Size: {} byte{}\n".format(int(reg.size / 8),
									pluralAp))
			
			self.lyxGen.commit_append_line(1)
			
			# Description
			self.lyxGen.insert_layout("Standard")
			self.lyxGen.wr_line("{}\n".format(reg.description))
			self.lyxGen.commit_append_line(1)
			
			# Bit table and bit field descriptions
			self.write_reg_field_table(reg)
			self.write_reg_field_desc(reg)
			
			# Separation from next register
			self.lyxGen.insert_layout("Standard")
			self.lyxGen.insert_inset("VSpace bigskip")
			self.lyxGen.commit_append_line(2)
			


	def write_mem_map_regions(self, memMap):
		table = self.lyxGen.build_table(2, len(memMap.addressBlock) + 1)
		
		self.lyxGen.set_columns_option(table, range(0,2),  
							[["width", "4cm"]  for j in range(0, 2)])
		
		titleCells = [[0, 0] , [0, 1]]
		nameCells = [[i, 0] for i in range(1, len(memMap.addressBlock) + 1)]
		addrCells = [[i, 1] for i in range(1, len(memMap.addressBlock) + 1)]
		
		nameVals = [block.name for block in memMap.addressBlock]
		addrVals = [hex(block.baseAddress) for block in memMap.addressBlock]
		titleVals = ["Memory region", "Address offset"]
		
		self.lyxGen.set_cells_object(table, nameCells, nameVals)
		self.lyxGen.set_cells_object(table, addrCells, addrVals)
		self.lyxGen.set_cells_object(table, titleCells, titleVals)
		self.lyxGen.insert_table(table)
		

################################################################################
#  Write the address map into output file
# 
# Arguments:
#  of		 	- Output file to write
################################################################################
	def write_mem_map_addr(self):
		self.write_mem_map_regions(self.addrMap)
	
	
################################################################################
# Write the bitfield map into the output file
#
# Arguments:
#  of		 	- Output file to write
################################################################################	
	def write_mem_map_fields(self):
		for block in self.fieldMap.addressBlock:
			self.write_regs(block.register)


################################################################################
# Write both memory maps into the output file
#
# Arguments:
#  of		 	- Output file to write
################################################################################	
	def write_mem_map_both(self):
		pass
		

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
		