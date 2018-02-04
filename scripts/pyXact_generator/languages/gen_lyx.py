################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Lyx document generator for PyXact parser
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

class LyxGenerator(BaseGenerator):
	
	####################################################################
	# Basic Lyx styles which are assumed to be universal...
	####################################################################
	
	# layouts (used with "\begin_layout")
	stdLayouts = ["Standard", "LyX-Code", "Quotation", "Quote", "Verse",
					"Verbatim", "Verbatim*", "Plain Layout"]
	stdLists = ["Itemize", "List", "Description", "Labeling"]
	stdRefPrefixes = ["Part", "Chapter", "Section", "Subsection",
						"Subsubsection", "Paragraph", "Subparagraph"]
	supStyles = [stdLayouts, stdLists, stdRefPrefixes]
	
	# insets (used with "\begin_inset")
	supInsets = { "NewLine" : {"newline" : ""}, 
					"VSpace" : {"bigskip" : "",
								"defskip" : "",
								"smallSkip" : "",
								"medskip" : "",
								"vfill" : "",
								"cm" : ""}, 
					"space" : {"~" : ""},
					"Graphics" : {"filename" : "__PATH",
									"lyxscale" : "__NUM",
									"scale" : "__NUM",
									"rotateAngle" : "__NUM"},
					"Tabular" : {},
					"Text" : {},
					"CommandInset" : {"toc" : "", 
										"citation" : "",
										"ref" : "",
										"label" : "",
										"LatexCommand" : "__STRING",},
					"ERT" : {"status" : "open"},
					"Quotes" : {"eld" : "", "erd" : ""},
					"Float" : {"table" : "",
								"figure" : "",
								"placement" : "__STRING",
								"wide" : "__BOOL",
								"sideways" : "__BOOL",
								"status" : "open"},
					"Caption" : {"Standard" : ""},
					"Box" : {"Frameless" : ""} # TODO: Other arguments
					}
	
	
	def __init__(self):
		super().__init__()
		self.supportedTypes = [""]
		self.typeSizes = [32, 8, 8, 8, 16, 16, 64, 64, 8, 16, 32, 64]
		self.commentSign = "#"
	
	
	def is_supported_layout(self, layout):
		for styleGroup in LyxGenerator.supStyles:
			for styleIter in styleGroup:
				if (styleIter == layout):
					return True
		return False
	
	
	def is_supported_inset(self, insetName, insetArgs):
		if (not insetName in self.supInsets):
			return False
		
		validArgs = self.supInsets[insetName]
		for insetArg, insetVal in insetArgs.items():
			valid = False
			for validArg, validVal in validArgs.items():
				if ((validArg == insetArg and validVal == insetVal == "") or
					(validArg == insetArg and validVal != "" and insetVal != "")):
					valid = True
					break
	
	def insert_layout(self, layout):
		if (not self.is_supported_layout(layout)):
			return False
		self.wr_line("\\begin_layout {}\n".format(layout))
		self.append_line("\\end_layout\n")
	
	
	def insert_inset(self, inset, options=[]):
		#if (not self.is_supported_inset(inset, options)):
		#	return False
		
		#for option in options:
		#	if (not self.is_supported_option(inset, option)):
		#		return False
			
		self.wr_line("\\begin_inset {}\n".format(inset))
		for option in options:
			self.wr_line("{} {}\n".format(option[0], option[1]))
		self.append_line("\\end_inset\n")
	
	
	def insert_html_table_tag(self, tag, tagOptions=None, endTag=False):
		#if (not self.is_supported_table_tag(tag)):
		#	return False
		
		optStr = ""
		if (tagOptions != None):
			for optName, optVal in sorted(tagOptions.items()):
				optStr = optStr + ' {}="{}"'.format(optName, optVal)
		
		self.wr_line("<{}{}>\n".format(tag, optStr))
		
		if (endTag):
			self.append_line("</{}>\n".format(tag))
	
	
	def is_table_valid(self, options, cells):
		# Check consistency of the table data
		return Trues
	
	
	def insert_text_options(self, textOptions):
		isSup = False
		for textOptKey,textOptVal in sorted(textOptions.items()):
			for supOpt in supTextOptions:
				if (textOptKey == supOpt):
					isSup = True
					self.wr_line("\\{} {}\n".format(textOptKey, textOptVal))
					break
	
	
	def insert_table_cell(self, cell):
		self.insert_html_table_tag("cell", cell[0], endTag=True) 
		self.insert_inset("Text")
		self.wr_line("\n")
		self.write_layout_text("Plain Layout", cell[2], textOptions=cell[1],
						label=cell[3])
		self.wr_line("\n")
		self.commit_append_line(2)
		


	def insert_table(self, table):
		#if (not is_table_valid(tableOptions, cells)):
		#	return False
		
		tableOptions = table[0]
		cells = table[1]
		
		self.insert_layout("Standard")
		self.wr_line("\\noindent\n")
		self.wr_line("\\align center\n")
		
		self.insert_inset("Tabular")
		tableDimension = {"version" : "3", "rows" : '{}'.format(len(cells)), 
						"columns" : '{}'.format(len(cells[0]))}
		self.insert_html_table_tag("lyxtabular", tableDimension, True)
		
		for option in tableOptions:
			self.insert_html_table_tag(option[0], tagOptions=option[1],
											endTag=False)
		
		for row in cells:
			self.insert_html_table_tag("row", endTag=True)
			for cell in row:
				self.insert_table_cell(cell)
			self.commit_append_line(1)
	
		self.commit_append_line(1)
		self.wr_line("\n")
		self.commit_append_line(1)
		self.wr_line("\n")
		self.wr_line("\n")
		self.commit_append_line(1)
	
	
	def replace_reserved_sign(self, text):
		retVal = ""
		for char in text:
			if (char == "_"):
				retVal += "\\backslash textunderscore "
			elif (char == """\ """):
				retVal += "\\"
			else:
				retVal += char
		return retVal
	
	def insert_ref(self, labelText, ref):
		self.insert_inset("ERT")
		self.wr_line("status open\n")
		self.insert_layout("Plain Layout")
		self.wr_line("\\backslash\n")
		
		if (ref == "label"):
			labelStr = "{"+"{}".format(labelText)+"}"
			self.wr_line("{}{}".format(ref, labelStr))
		
		if (ref == "hyperref"):
			refString = "{" + self.replace_reserved_sign(labelText) + "}"
			self.wr_line("{}[{}]{}".format(ref, labelText, refString))
	
		self.commit_append_line(2)


	def write_layout_text(self, layoutType, text, textOptions=None, label=None):
		self.insert_layout(layoutType)
		if (textOptions != None):
			self.insert_text_options(textOptions)
		if (label == "hyperref"):
			self.insert_ref(text, label)
		elif (label != None):
			self.wr_line(text)
			self.insert_ref(text, label)
		else:
			self.wr_line(text)
		
			
		self.commit_append_line(1)
	









	def build_table_options(self, columnCount, rowCount):
		tableOptions = []
		tableOptions.append(["features", {"tabularvalignment" : "middle"}])
		for i in range(0, columnCount):
			tableOptions.append(["column", {"alignment" : "center" ,
											"valignment" : "top"}])
		return tableOptions
	
	
	
	def build_table_cells(self, columnCount, rowCount, defCellText):
		tableCells = []
		stdCellAttributes = {"alignment" : "center", "valignment" : "top",
						"topline" : "true", "leftline" : "true",
						"usebox" : "none"}
		
		for row in range(0, rowCount):
			tableCells.append([])
			for column in range(0, columnCount):
				
				tableCells[row].append([])
				actCell = tableCells[row][column]
				
				actCell.append(stdCellAttributes.copy())
				if (column == columnCount - 1):
					actCell[0]["rightline"] = "true"
				if (row == rowCount - 1):
					actCell[0]["bottomline"] = "true"
				actCell.append({})
				actCell.append(defCellText)
				actCell.append(None)
				
		return tableCells
				
	
	def build_table(self, columnCount, rowCount, defCellText="Reserved"):
		table = []	
		tableOptions = self.build_table_options(columnCount, rowCount)
		tableCells = self.build_table_cells(columnCount, rowCount, defCellText)
		table.append(tableOptions)
		table.append(tableCells)
		return table


	def set_column_option(self, table, column, optKey, optVal):
		table[0][column + 1][1][optKey] = optVal

	def set_cell_option(self, table, row, column, optKey, optVal):
		table[1][row][column][0][optKey] = optVal
	
	def set_cell_object(self, table, row, column, object):
		table[1][row][column][2] = object

	def set_cell_text_prop(self, table, row, column, propName, propVal):
		table[1][row][column][1][propName] = propVal
	
	def set_cell_text_label(self, table, row, column, label):
		table[1][row][column][3] = label

		

	def set_columns_option(self, table, columns, opPairs):
		for column,opt in zip(columns, opPairs):
			self.set_column_option(table, column, opt[0], opt[1])

	def set_cells_object(self, table, cells, objects):
		for cell,object in zip(cells, objects):
			self.set_cell_object(table, cell[0], cell[1], object)
	
	def set_cells_option(self, table, cells, opPairs):
		for cell,opPair in zip(cells, opPairs):
			table[1][cell[0]][cell[1]][0][opPair[0]] = opPair[1]
	
	def set_cells_text_label(self, table, cells, labels):
		for cell,label in zip(cells, labels):
			self.set_cell_text_label(table, cell[0], cell[1], label)

	

	def insert_new_page(self):
		self.insert_layout("Standard")
		self.insert_inset("Newpage newpage")
		self.commit_append_line(2)
	
	
	
	
	
		
		
		
		
		
		
		
		
		
		
		
		
		
	def write_comm_line(self, gap=2):
		self.__wr_line('{:{fill}<78}\n'.format(" " * gap, fill=self.commentSign))
		
	def write_comment(self, input, gap, caption=None, small=False):
		self.__wr_line('{}# {}'.format("	" * gap, input))
		
	def create_enum(self, name, decls):
		pass
	
	def write_decl(self, decl):
		pass
	
	def create_union(self, name, decls):
		pass

	def create_structure(self, name, decls):
		pass
				
	def create_package(self, name, start=True):
		pass
	
	def create_includes(self, includeList):
		pass
		
		
			

	
		
