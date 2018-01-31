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
		if (not insetName in self.suppInsets):
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
		self.wr_line("\begin_layout {}\n".format(layout))
		self.append_line("\end_layout\n")
	
	
	def insert_inset(self, inset, options=[]):
		if (not self.is_supported_inset(inset)):
			return False
		
		for option in options:
			if (not self.is_supported_option(inset, option)):
				return False
			
		self.wr_line("\begin_inset {}\n".format(inset))
		for option in options:
			self.wr_line("{} {}\n".format(option[0], option[1]))
		self.append_line("\end_inset")
	
	
	def insert_html_table_tag(self, tag, tagOptions=None, endTag=False):
		if (not self.is_supported_table_tag(tag)):
			return False
		
		optStr = ""
		if (tagOptions != None):
			for optName, optVal in tagOptions.items():
				optStr = optStr + ' {}="{}"'.format(optName, optVal)
		
		self.wr_line("<{}{}>\n".format(tag, optionStr))
		
		if (endTag):
			self.append_line("</{}>\n".format(tag))
	
	
	def is_table_valid(self, options, cells):
		# Check consistency of the table data
		return Trues
	
	
	def insert_text_options(self, textOptions):
		isSup = False
		for textOptKey,textOptVal in textOptions.items():
			for supOpt in supTextOptions:
				if (textOptKey == supOpt):
					isSup = True
					self.wr_line("\{} {}\n".format(textOptKey, textOptVal))
					break
	
	
	def insert_table_cell(self, cell):
		self.insert_html_table_tag("cell", cell[0]) 
		self.insert_inset("Text")
		self.insert_layout("Plain Layout")
		self.insert_text_options(cell[1])
		self.wr_line(cell[2])
		self.commit_append_line(3)


	


	def insert_table(self, tableOptions, cells):
		if (not is_table_valid(tableOptions, cells)):
			return False
		
		self.insert_inset("Tabular")
		tableDimension = {"version" : "3", "rows" : '{}'.format(len(cells)), 
						"columns" : '{}'.format(len(cells[0]))}
		self.insert_html_table_tag(self, "lyxTabular", tableDimension, True)
		
		for option in tableOptions:
			self.insert_html_table_tag(option[0], tagOptions=option[1],
											endTag=False)
		
		for row in cells:
			self.insert_html_table_tag("row", True)
			for cell in row:
				self.insert_table_cell(cell)
			self.commit_append_line(1)
	
		self.commit_append_line(1)

	
	


	def insert_new_page(self):
		self.insert_inset("Newpage newpage")
		self.commit_append(1)
	
	
	
	
	
		
		
		
		
		
		
		
		
		
		
		
		
		
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
		
		
			

	
		
