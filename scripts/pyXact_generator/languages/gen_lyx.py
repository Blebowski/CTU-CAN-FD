################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   A simple Lyx document generator.
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
	
	# TODO: Should we do also supported insets?? I dont want to complicate
	#       it so lets just leave little responisibility on the one who
	#       is using the generator
	
	
	def __init__(self):
		super().__init__()
		self.supportedTypes = [""]
		self.typeSizes = [32, 8, 8, 8, 16, 16, 64, 64, 8, 16, 32, 64]
		self.commentSign = "#"
	
	
	def is_supported_layout(self, layout):
		"""
		Finds out whether the layout is supported by the basic lyx packages.
		Arguments:
			layout		Name of the lyx layout
		Returns:
			True id supported, False otherwise
		"""
		for styleGroup in LyxGenerator.supStyles:
			for styleIter in styleGroup:
				if (styleIter == layout):
					return True
		return False
	
	
	def insert_layout(self, layout):
		"""
		Insert lyx layout into the generator output (Chapter, Section etc...)
		Arguments:
			layout		Name of the lyx layout
		"""
		if (not self.is_supported_layout(layout)):
			return False
		self.wr_line("\\begin_layout {}\n".format(layout))
		self.append_line("\\end_layout\n")
	
	
	def insert_inset(self, inset, options=[]):
		"""
		Insert lyx inset into the generator output (Text, Tabular, ERT)
		Arguments:
			layout		Name of the lyx inset.
		"""
		self.wr_line("\\begin_inset {}\n".format(inset))
		for option in options:
			self.wr_line("{} {}\n".format(option[0], option[1]))
		self.append_line("\\end_inset\n")
	
	
	def insert_html_table_tag(self, tag, tagOptions=None, endTag=False):
		"""
		Insert lyx HTML Table Tag.
		Arguments:
			tag			Tag name
			tagOptions  Dictionary of tag options in format:
						{optName=optVal,...}
			endTag		Whether </tag> should be pushed on the top of append
						stack.
		"""
		optStr = ""
		if (tagOptions != None):
			for optName, optVal in sorted(tagOptions.items()):
				optStr = optStr + ' {}="{}"'.format(optName, optVal)
		
		self.wr_line("<{}{}>\n".format(tag, optStr))
		
		if (endTag):
			self.append_line("</{}>\n".format(tag))
	
	
	def insert_text_options(self, textOptions):
		"""
		Insert Lyx Text options.
		Arguments:
			textOptions  Options dictionary to insert in format:
							{textOptKey:textOptVal}.  E.g {"\series":"bold"}
		"""
		isSup = False
		for textOptKey,textOptVal in sorted(textOptions.items()):
			self.wr_line("\\{} {}\n".format(textOptKey, textOptVal))

	
	def insert_table_cell(self, cell):
		"""
		Insert Table cell into the into the generator output.
		Arguments:
			cell	Cell to insert in format:
					[cellOptions, cellTextOptions, cellText, cellLabel] where:
						cellOptions - Dictionary with cell options such as:
									{"alignment":"center", "bottomline":"true"}
					cellTextOptions	- Text options to for the text in the cell
					cellText		- Text to be written into the cell
					cellLabel		- Special label which specifies the function
									  of the cell. At the moment supported:
										"label" - creates lyx Label
										"hyperref" - creates reference
		"""
		self.insert_html_table_tag("cell", cell[0], endTag=True) 
		self.insert_inset("Text")
		self.wr_line("\n")
		self.write_layout_text("Plain Layout", cell[2], textOptions=cell[1],
						label=cell[3])
		self.wr_line("\n")
		self.commit_append_line(2)
		


	def insert_table(self, table):
		"""
		Insert lyx table into the generator output.
		Arguments:
			table 		Lyx table in the following format:
							[tableOptions, tableCells] where:
								tableOptions has following format:
									[tagName, tagOptions] where:
										tagName - name of HTML options tag
										tagOptions - dictionary with HTML tag
													 options
								tableCells has following format:
									[[cellx0y0, cellx1y0 ... , cellxny0],
									 [cellx0y1, cellx1y1 ... , cellxny1],
													...
									 [cellx0ym, cellx1ym ... , cellxnym]]
									 for the table of width "n" and height "m".
									 Each "cell" object has structure of list
									 as defined in "insert_table_cell" function.
		"""
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
		"""
		Replace reserved character of "_" in the lyx hypertext function.
		Arguments:
			text		Text to replace the option in
		Returns:
			Text with reserved characters replaced with according backslash
			option.
		"""
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
		"""
		Insert label or reference into the generator output.
		Arguments:
			labelText	Text to write into the label
			ref			Label type. At the moment can be:
							"label" - LabelText is written and label object is
									  created directly behind.
							"hyperref" - Hyperref object is created with 
										 "labelText" placed in the hyperref.
										This makes the reference to the label
										with the same name if clicked on in PDF.
		"""
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
		"""
		Insert layout type into the generator output and append a simple text.
		If text options are specified then the text in the layout is printed
		with these options.
		Arguments:
			layoutType		Name of the layout as in "insert_layout"
			text			Text to be written in the layout.
			textOptions		Text options as specified in the "insert_text_options"
			label			If the layout should be written as label. Current
							options are supported:
								None 		Layout is written normally
								"label"		Label with the same text as "text"
											is inserted behind the text
								"hyperref"	Reference is inserted with the
											same text as in the text.
		"""
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
		"""
		Creates default table options object with default column alignment
		set to center of the cell.
		Arguments:
			columnCount		Number of table columns
			rowCount		Number of table rows
		Returns:
			New table options object which can be used in "insert_html_table_tag"
			function.
		"""
		tableOptions = []
		tableOptions.append(["features", {"tabularvalignment" : "middle"}])
		for i in range(0, columnCount):
			tableOptions.append(["column", {"alignment" : "center" ,
											"valignment" : "top"}])
		return tableOptions
	
	
	def build_table_cells(self, columnCount, rowCount, defCellText):
		"""
		Creates default table cell objects with standard cell attributes.
		Arguments:
			columnCount		Number of table columns
			rowCount		Number of table rows
			defCellText		Text which should be put to the cell by default.
		Returns:
			New table cells object which has the structure as tableCells in
			the table argument of function "insert_table".
		"""
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
		"""
		Builds the table 
		Arguments:
			columnCount		Number of table columns
			rowCount		Number of table rows
			defCellText		Text which should be put to the cell by default.
		Returns:
			New table object with structure as "table" argument in "insert_table"
			function.
		"""
		table = []	
		tableOptions = self.build_table_options(columnCount, rowCount)
		tableCells = self.build_table_cells(columnCount, rowCount, defCellText)
		table.append(tableOptions)
		table.append(tableCells)
		return table


	def set_column_option(self, table, column, optKey, optVal):
		"""
		Set option on column of a table.
		Arguments:
			table			Table object as in "insert_table" function.
			column			Index of the column at which to set the option.
			optKey			Option key to set (e.g. "alignment")
			optVal			Option value to be set to the option given by 
							"optKey"
		"""
		table[0][column + 1][1][optKey] = optVal


	def set_cell_option(self, table, row, column, optKey, optVal):
		"""
		Set option on the cell of a table.
		Arguments:
			table			Table object as in "insert_table" function.
			column			Index of the cell column at which to set the option
			row				Index of the cell row at which to set the option
			optKey			Option key to set (e.g. "bottomline", "multicolumn")
			optVal			Option value to be set to the option given by 
							"optKey"
		"""
		table[1][row][column][0][optKey] = optVal
	
	
	def set_cell_object(self, table, row, column, object):
		"""
		Set text in the cell object of a table
		Arguments:
			table			Table object as in "insert_table" function.
			column			Index of the cell column at which to set the option
			row				Index of the cell row at which to set the option
			object			Text to be set
		"""
		table[1][row][column][2] = object
		

	def set_cell_text_prop(self, table, row, column, propName, propVal):
		"""
		Set text properties in the cell object.
		Arguments:
			table			Table object as in "insert_table" function.
			column			Index of the cell column at which to set the option
			row				Index of the cell row at which to set the option
			propName		Text property name
			propVal			Text property value
		"""
		table[1][row][column][1][propName] = propVal
	
	
	def set_cell_text_label(self, table, row, column, label):
		"""
		Set label in the cell object.
		Arguments:
			table			Table object as in "insert_table" function.
			column			Index of the cell column at which to set the option
			row				Index of the cell row at which to set the option
			label			Label type to be set. Currently supported:
								None		No special label
								"label"		Label will be inserted into the
											cell with the same text as cell
											text
								"hyperref"  Text will be written into the
											table as Lyx reference.
		"""
		table[1][row][column][3] = label

		

	def set_columns_option(self, table, columns, opPairs):
		"""
		Extended function "set_column_option" for multiple columns.
		Arguments:
			table			Table object as in "insert_table" function.
			columns			List of column options to be set
			opPairs			List of column option values to be set
		"""
		for column,opt in zip(columns, opPairs):
			self.set_column_option(table, column, opt[0], opt[1])


	def set_cells_object(self, table, cells, objects):
		"""
		Extended function "set_cell_object" for multiple cells.
		Arguments:
			table			Table object as in "insert_table" function.
			cells			List of cell text to be set
			objects			List of cell texts to be set
		"""
		for cell,object in zip(cells, objects):
			self.set_cell_object(table, cell[0], cell[1], object)


	def set_cells_option(self, table, cells, opPairs):
		"""
		Extended function "set_cell_option" for multiple cells.
		Arguments:
			table			Table object as in "insert_table" function.
			cells			List of cell options to be set
			opPairs			List of cell option values to be set
		"""
		for cell,opPair in zip(cells, opPairs):
			table[1][cell[0]][cell[1]][0][opPair[0]] = opPair[1]
	
	
	def set_cells_text_label(self, table, cells, labels):
		"""
		Extended function "set_cell_text_label" for multiple cells.
		Arguments:
			table			Table object as in "insert_table" function.
			cells			List of cell to be set the label on
			labels			List of label values to set. 
		"""
		for cell,label in zip(cells, labels):
			self.set_cell_text_label(table, cell[0], cell[1], label)


	def insert_new_page(self):
		"""
		Write new page into the generator output.
		"""
		self.insert_layout("Standard")
		self.insert_inset("Newpage newpage")
		self.commit_append_line(2)
	
	
	def write_comm_line(self, gap=2):
		"""
		Write comment line to the lyx document
		"""
		self.__wr_line('{:{fill}<78}\n'.format(" " * gap, fill=self.commentSign))
		
		
	def write_comment(self, input, gap, caption=None, small=False):
		"""
		Write comment to the lyx document
		"""
		self.__wr_line('{}# {}'.format("	" * gap, input))		
		
			

	
		
