################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Script for complete update of register map. This script generates C header, 
##   VHDL packages and Lyx documentation.
##   
##   Arguments:
##		xactSpec    - Path to a IP-XACT specification file with register maps
##		updVHDL		- Whether VHDL constant definitions should be generated
##						(../src/Libraries)
##		updHeader	- Whether C header file should be generated
##						(../driver)
##		updDocs		- Whether Lyx doocumentation should be generated.
##						(../doc/core)
##      updRegMap   - Whether register map VHDL RTL should be generated. 
##                      (../src/Registers_Memory_Interface)
##
##	Revision history:
##		06.02.2018	Implemented the script
##
################################################################################

import argparse
import sys
import time
import importlib.util
import os
import inspect
import math

import pyXact_generator

from pyXact_generator.gen_lib import *

from pyXact_generator.HeaderAddrGeneratorWrapper import HeaderAddrGeneratorWrapper
from pyXact_generator.LyxAddrGeneratorWrapper import LyxAddrGeneratorWrapper
from pyXact_generator.VhdlAddrGeneratorWrapper import VhdlAddrGeneratorWrapper
from pyXact_generator.VhdlRegMapGeneratorWrapper import VhdlRegMapGeneratorWrapper

MIT_LICENSE_PATH = "../LICENSE"
GPL2_LICENSE_PATH = "../lic/gpl_v2.txt"

def parse_args():
	parser = argparse.ArgumentParser(
				description="""Script for complete update of register map. 
								This script generates C header, 
								VHDL packages and Lyx documentation.""")

	parser.add_argument('--xactSpec', dest='xactSpec', help="""Path to a IP-XACT
							specification file with register maps""")

	parser.add_argument('--updVHDLPackage', dest='updVHDLPackage', help=""" Whether VHDL 
										constant definitions should be generated
										(../src/Libraries)""")

	parser.add_argument('--updHeaderFile', dest='updHeaderFile', help=""" Whether C
										header file should be generated
										(../driver)""")

	parser.add_argument('--updLyxDocs', dest='updLyxDocs', help="""Whether Lyx 
										doocumentation should be generated.
										(../doc/core)""")

	parser.add_argument('--updRTLRegMap', dest='updRTLRegMap', help="""Whether VHDL 
										RTL register map should be generated.
										(../src/Registers_Memory_Interface/generated)""")
	return parser.parse_args();



def ctu_can_update_vhdl_package(specPath, licensePath, memMap, 
								wordWidthBit, outPath, packName):
	"""
	Update VHDL packages of CTU CAN FD register maps.
	"""
	addrGeneratorWrapper = VhdlAddrGeneratorWrapper()

	addrGeneratorWrapper.xactSpec = specPath
	addrGeneratorWrapper.licPath = licensePath
	addrGeneratorWrapper.memMap = memMap
	addrGeneratorWrapper.wordWidth = wordWidthBit
	addrGeneratorWrapper.outFile = outPath
	addrGeneratorWrapper.packName = packName

	addrGeneratorWrapper.do_update()


def ctu_can_update_header(specPath, licensePath, memMap, 
								wordWidthBit, outPath, headName):
	"""
	Update header files of CTU CAN FD register maps.
	"""
	headerGeneratorWrapper = HeaderAddrGeneratorWrapper()

	headerGeneratorWrapper.xactSpec = specPath
	headerGeneratorWrapper.licPath = licensePath
	headerGeneratorWrapper.memMap = memMap
	headerGeneratorWrapper.wordWidth = wordWidthBit
	headerGeneratorWrapper.outFile = outPath
	headerGeneratorWrapper.headName = headName

	headerGeneratorWrapper.do_update()


def ctu_can_update_lyx_docu(specPath, memMap, wordWidthBit, outPath, genRegions, genFiDesc, lyxTemplate):
	"""
	Update Lyx documenation of CTU CAN FD register maps.
	"""
	lyxDocuGeneratorWrapper = LyxAddrGeneratorWrapper()

	lyxDocuGeneratorWrapper.xactSpec = specPath
	lyxDocuGeneratorWrapper.memMap = memMap
	lyxDocuGeneratorWrapper.wordWidth = wordWidthBit
	lyxDocuGeneratorWrapper.outFile = outPath
	lyxDocuGeneratorWrapper.genRegions = genRegions
	lyxDocuGeneratorWrapper.genFiDesc = genFiDesc
	lyxDocuGeneratorWrapper.lyxTemplate = lyxTemplate

	lyxDocuGeneratorWrapper.do_update()


def ctu_can_update_vhdl_rtl(specPath, licensePath, memMap, wordWidthBit, outDir):
	"""
	Update RTL codes of CTU CAN FD register map.
	"""
	vhdlRTLGeneratorWrapper = VhdlRegMapGeneratorWrapper()

	vhdlRTLGeneratorWrapper.licPath = licensePath
	vhdlRTLGeneratorWrapper.xactSpec = specPath
	vhdlRTLGeneratorWrapper.memMap = memMap
	vhdlRTLGeneratorWrapper.wordWidth = wordWidthBit
	vhdlRTLGeneratorWrapper.outDir = outDir

	vhdlRTLGeneratorWrapper.do_update()


if __name__ == '__main__':
	args = parse_args()
	print( 80 * "*")
	print("**  Generating CAN FD register map")
	print(80 * "*")

	pythonVersion = sys.version.split('.')
	pythonCmd = "python" + pythonVersion[0] + "." + pythonVersion[1]
	print("\n Python version is: %s \n" % pythonCmd)


	###########################################################################
	# Generate VHDL Packages
	###########################################################################
	if (str_arg_to_bool(args.updVHDLPackage)):

		print("Generating CAN FD memory registers VHDL packages...\n")

		ctu_can_update_vhdl_package(specPath=args.xactSpec,
									licensePath=MIT_LICENSE_PATH,
									memMap="CAN_Registers",
									wordWidthBit=32,
									outPath="../src/Libraries/CAN_FD_register_map.vhd",
									packName="can_fd_register_map")

		ctu_can_update_vhdl_package(specPath=args.xactSpec,
									licensePath=MIT_LICENSE_PATH,
									memMap="CAN_Frame_format",
									wordWidthBit=32,
									outPath="../src/Libraries/CAN_FD_frame_format.vhd",
									packName="can_fd_frame_format")

		print("\nDone\n")


	###########################################################################
	# Generate C Header File
	###########################################################################	
	if (str_arg_to_bool(args.updHeaderFile)):

		print("Generating CAN FD memory registers Header file...\n")

		ctu_can_update_header(specPath=args.xactSpec,
									licensePath=GPL2_LICENSE_PATH,
									memMap="CAN_Registers",
									wordWidthBit=32,
									outPath="../driver/ctu_can_fd_regs.h",
									headName="CAN_FD_register_map")

		ctu_can_update_header(specPath=args.xactSpec,
									licensePath=GPL2_LICENSE_PATH,
									memMap="CAN_Frame_format",
									wordWidthBit=32,
									outPath="../driver/ctu_can_fd_frame.h",
									headName="CAN_FD_frame_format")

		print("\nDone\n")


	###########################################################################
	# Generate Lyx documentation files
	###########################################################################	
	if (str_arg_to_bool(args.updLyxDocs)):

		print("Generating CAN FD memory registers Documentation...\n")

		ctu_can_update_lyx_docu(specPath=args.xactSpec,
								memMap="CAN_Registers",
								wordWidthBit=32,
								outPath="../doc/core/registerMap.lyx",
								genRegions=True,
								genFiDesc=True,
								lyxTemplate="../doc/core/template.lyx")

		ctu_can_update_lyx_docu(specPath=args.xactSpec,
								memMap="CAN_Frame_format",
								wordWidthBit=32,
								outPath="../doc/core/CANFrameFormat.lyx",
								genRegions=False,
								genFiDesc=True,
								lyxTemplate="../doc/core/template.lyx")

		print("\nDone\n")


	###########################################################################
	# Generate VHDL RTL codes
	###########################################################################	
	if (str_arg_to_bool(args.updRTLRegMap)):

		print("Generating CAN FD memory registers Documentation...\n")

		ctu_can_update_vhdl_rtl(specPath=args.xactSpec,
								licensePath=MIT_LICENSE_PATH,
								memMap="CAN_Registers",
								wordWidthBit=32,
								outDir="../../src/Registers_Memory_Interface/generated")

		# Frame format map not implemented as RTL, virtual map for frame format
		# visualisaion only

		print("\nDone\n")
	
	print( 80 * "*")
	print("**  Finished")
	print(80 * "*")
	
