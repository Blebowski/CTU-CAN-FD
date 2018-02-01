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


	def write_reg_field_desc(self, reg):
		for field in sorted(reg.field, key=lambda a: a.bitOffset):
			self.lyxGen.insert_layout("Description")
			self.lyxGen.wr_line("{} {}\n".format(field.name, field.description))
			self.lyxGen.commit_append_line(1)


	def getBit(self, val, bitIndex):
		tmp = "{0:032b}".format(val)
		print(tmp)
		print("BitIndex {}".format(bitIndex))
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
					print(field.name)
					print(tmp)
					print(field.bitOffset)
					print(field.bitWidth)
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
							append = append + ":{}]".format(lInd)
						else:
							append = append + "]"
						fieldName = fieldName + append
				else:
					fieldName = "Reserved"
					fieldRst = "-"
				
				
				retVal[i][j].append(fieldName)
				retVal[i][j].append(fieldRst)	
		return retVal		
	
	
	
	def write_reg_field_table(self, reg):
		
		regFields = self.reg_unwrap_fields(reg)
		
		#print(regFields[0])
		#print(regFields[1])
		#print(regFields[2])
		#print(regFields[3])
		
		for i in reversed(range(1, int(reg.size / 8 + 1))):
			prevName = ""
			tableOptions = []
			tableOptions.append(["features", {"tabularvalignment" : "middle"}])
			tableOptions.append(["column", {"alignment" : "center" ,
											"valignment" : "top"}])
			for j in range(1,9):
				tableOptions.append(["column", {"alignment" : "center" ,
												"valignment" : "top",
												"width" : "1.4cm"}])
			
			tableCells = [[], [], []]
			
			stdCellAttributes = {"alignment" : "center", "valignment" : "top",
						"topline" : "true", "leftline" : "true",
						"usebox" : "none"}
			
			# Title row
			tableCells[0].append([])
			tableCells[0][0].append(stdCellAttributes)
			tableCells[0][0].append({})
			tableCells[0][0].append("Bit offset\n")
			for j in range(1,9):
				tableCells[0].append([])
				withLine = stdCellAttributes
				if (j == 8):
					withLine = stdCellAttributes.copy()
					withLine["rightline"] = "true" 
				tableCells[0][j].append(withLine)
				tableCells[0][j].append({})
				tableCells[0][j].append("{}\n".format((8 - j) + (i - 1) * 8))
			
			# Field name row
			tableCells[1].append([])
			tableCells[1][0].append(stdCellAttributes)
			tableCells[1][0].append({})
			tableCells[1][0].append("Field name\n")
			for j in range(1,9):
				tableCells[1].append([])
				withLine = stdCellAttributes.copy()
				
				multicolumn = (prevName == regFields[i - 1][j - 1][0])
				if (multicolumn):
					withLine["multicolumn"] = "2"
				else:
					withLine["multicolumn"] = "1"
					
				tableCells[1][j].append(withLine)
				tableCells[1][j].append({})
				tableCells[1][j].append(regFields[i - 1][j - 1][0])
				prevName = regFields[i - 1][j - 1][0]
			
			# Correct the right most field to have proper right panel
			for j in reversed(range(1,9)):
				if (tableCells[1][j][0]["multicolumn"] == "1"):
					tableCells[1][j][0]["rightline"] = "true"
					break	
			
			# Restart value row
			tableCells[2].append([])
			withLine = stdCellAttributes.copy()
			withLine["bottomline"] = "true"	
			tableCells[2][0].append(withLine)
			tableCells[2][0].append({})
			tableCells[2][0].append("Reset value\n")
			for j in range(1,9):
				tableCells[2].append([])
				if (j == 8):
					withLine = withLine.copy()
					withLine["rightline"] = "true"
				tableCells[2][j].append(withLine)
				tableCells[2][j].append({})
				tableCells[2][j].append(regFields[i - 1][j - 1][1])
			
			self.lyxGen.insert_table(tableOptions, tableCells)
			

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
			


################################################################################
#  Write the address map into output file
# 
# Arguments:
#  of		 	- Output file to write
################################################################################
	def write_mem_map_addr(self):
		pass
	
	
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
		