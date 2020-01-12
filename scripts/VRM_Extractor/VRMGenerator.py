################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Simple HTML generator generating HTML document from test information
##   obtained by VRM Processor.
##
##	Revision history:
##		09.01.2020	Implemented the script
##
################################################################################

import os
import re

from pathlib import Path
from yattag import Doc

class VRMGenerator:
    
	vrmInfo = None
	outPath = " "

	def __init__(self, vrmInfo, outPath):
		"""
		"""
		if (type(vrmInfo) != list):
			print("Input to VRMProcessor shall be list of files")
			exit(-1);

		self.vrmInfo = vrmInfo
		self.outPath = outPath

	def add_html_table_header(self, doc, tag, text, headers, back_color="White"):
		"""
		Add header to HTML table.
		"""
		with tag('tr'):
			for header in headers:
				with tag('th', bgcolor=back_color):
				    text(header)

	def gen_single_test_info(self, doc, tag, text, testItem):
		"""
        Generate HTML table row with test info of 1 test.
		"""
		with tag('tr'):
			with tag('td'):
				text(str(testItem["testName"]).split("/")[-1])

				#TODO: Maybe add link to file??			
				#file_name = get_collapsed_file_name(psl_point)
				#with tag('a', href=file_name+".html"):
				#    text("Open collapsed tests")

			with tag('td'):
				text(testItem["purpose"])
				
			with tag('td'):
				with tag('table', width='100%'):
					for vrmEntry in testItem["vrmEntries"]:
						with tag('tr'):
							with tag('td'):
								text(vrmEntry.strip("@"))

			with tag('td'):
				with tag('table', width='100%'):
					for testEntry in testItem["sequence"]:
						with tag('tr'):
							with tag('td'):
								text(testEntry.strip("@"))

			#
			#if (psl_point["status"] == "covered" or \
			#    psl_point["status"] == "passed"):
			#    color = "Lime"
			#else:
			#    color = "red"

			#with tag('td', ('bgcolor',color)):
			#    text(psl_point["status"])


	def gen_VRM_info(self, doc, tag, text):
		"""
        Generate test information for all tests. Each test represents single row
        in VRM HTML table
		"""
		with tag('table', width='100%', border="1px solid black"):
			headers = ["Test Name"]
			headers.append("Purpose")
			headers.append("VRM Entries (What does the test verify)")
			headers.append("Test sequence")
			self.add_html_table_header(doc, tag, text, headers,
										back_color="Aquamarine")

			for testItem in self.vrmInfo:
				print("Generating info for test: {}".format(testItem["testName"]))
				self.gen_single_test_info(doc, tag, text, testItem)


	def gen_VRM_title(self, doc, tag, text, title):
		"""
        Generate HTML VRM title.
		"""
		with tag('table', width='100%', border=0, cellspacing=0, cellpadding=0):
			with tag('tr'):
				with tag('th', ('class','title')):
					with tag('font', size=10):
						text(title)

	def gen_VRM(self, title):
		"""
        Generate HTML VRM for test information.
		"""

		doc, tag, text = Doc().tagtext()

		self.gen_VRM_title(doc, tag, text, title)
		self.gen_VRM_info(doc, tag, text)

		with Path(self.outPath).open('wt', encoding='utf-8') as html_file:
			html_file.write(doc.getvalue())

