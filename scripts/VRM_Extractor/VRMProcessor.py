################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Simple text processor extracting test information from test header files.
##   Following information are extracted:
##     1. VRM entries (what is verified by the test)
##     2. Test sequence
##     3. Test Purpose (simple test description)
##
##	 Tests extract information between tags:
##		@TestInfoStart
##		@TestInfoEnd
##
##	Revision history:
##		09.01.2020	Implemented the script
##
################################################################################

import os
import re

from enum import Enum

class VRMFSM(Enum):
		IDLE = 1,
		STARTED = 2,
		PURPOSE = 3,
		VRM_ENTRIES = 4,
		TEST_SEQUENCE = 5,
		NOTES = 6,
		FINISHED = 7

# Tags which are search for by VRM extractor!
purposeTag = "@Purpose"
vrmEntryTag = "@Verifies"
sequenceTag = "@Test sequence"
notesTag = "@Notes"

startTag = "@TestInfoStart"
endTag = "@TestInfoEnd"

class VRMProcessor:
    
	fileList = []

	# Results are list of entries, where each entry is dictionary with keys:
    #   testName - Name of test file
	#	vrmEntries - List of strings, each with VRM entries.
	#	purpose - Single string
	#	sequence - List of strings with test sequence
	results = []	

	def __init__(self, fileList):
		"""
		"""
		if (type(fileList) != list):
			printf("Input to VRMProcessor shall be list of files")
			system.exit(-1);

		self.fileList = fileList


	def VRMProcess(self, tf):
		"""
		VRMProcessor FSM. Process input file and extract test info!
		"""
		state = VRMFSM.IDLE
		lines = tf.readlines()

		res = {"purpose":"", "vrmEntries":[], "sequence":[], "testName":"", "notes":""}
		
		for lineCnt, line in enumerate(lines):

			if (state == VRMFSM.IDLE and re.search(startTag, line)):
				state = VRMFSM.STARTED

			# Process further states only when we are started!
			if (state == VRMFSM.IDLE or state == VRMFSM.FINISHED):
				continue

			if (re.search(purposeTag, line)):
				state = VRMFSM.PURPOSE
				continue

			if (re.search(vrmEntryTag, line)):
				state = VRMFSM.VRM_ENTRIES
				continue

			if (re.search(sequenceTag, line)):
				state = VRMFSM.TEST_SEQUENCE
				continue

			if (re.search(notesTag, line)):
				state = VRMFSM.NOTES
				continue

			if (re.search(endTag, line)):
				state = VRMFSM.FINISHED
				break			

			line = line.strip('-\n ')

			if (state == VRMFSM.STARTED):
				continue

			if (state == VRMFSM.PURPOSE):
				res["purpose"]+= line + " "

			# VRM entries are linear list always!
			if (state == VRMFSM.VRM_ENTRIES):
				# Start of next step
				if (re.match("@[1-9]+[.]", line)):
					res["vrmEntries"].append("")
				if (len(res["vrmEntries"]) > 0):
					res["vrmEntries"][-1]+=line+" "

			# Test sequence can be nested, but we linearize them for simplicity!
			# Add also item tag so that nested items can be distinguished!
			if (state == VRMFSM.TEST_SEQUENCE):
				if (re.match("@[1-9]+[.].+", line)):
					res["sequence"].append("")
				if (len(res["sequence"]) > 0):
					res["sequence"][-1]+=line+" "

			if (state == VRMFSM.NOTES):
				res["notes"]+=line + " "

			if (lineCnt > 1000):
				print("Too many lines in test info, aborting!");
				break

		return res;

	def printResult(self, res):
		"""
		Print test info result extracted from single test file header
		"""
		print("*" * 80)
		print("PURPOSE:")
		print(res["purpose"])
		
		print("VRM ENTRIES:")
		for vrmEntry in res["vrmEntries"]:
			print(vrmEntry)

		print("TEST SEQUENCE:")
		for testEntry in res["sequence"]:
			print(testEntry)
		print("*" * 80)

	def extractTestInfo(self):
		"""
		Extract test information for each test within self.fileList info
		"""
		for testFile in self.fileList:

			print("Processing file: {}".format(testFile))
			
			tf = open(str(testFile), 'r')
			if (not tf):
				print("Unable to open : {}, skipping".format(testFile))
				continue

			testInfo = self.VRMProcess(tf)
			testInfo["testName"] = testFile

			#print("Extracted test info:")
			#self.printResult(testInfo)
			#print("\n\n")
			
			self.results.append(testInfo)
			tf.close()

		return self.results

