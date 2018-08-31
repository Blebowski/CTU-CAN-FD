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

from pyXact_generator.gen_lib import *

from gen_c_header import *
from gen_lyx_docu import *
import gen_vhdl_package

def parse_args():
	parser = argparse.ArgumentParser(
				description="""Script for complete update of register map. 
								This script generates C header, 
								VHDL packages and Lyx documentation.""")
	parser.add_argument('--xactSpec', dest='xactSpec', help="""Path to a IP-XACT
							specification file with register maps""")
	parser.add_argument('--updVHDL', dest='updVHDL', help=""" Whether VHDL 
										constant definitions should be generated
										(../src/Libraries)""")
	parser.add_argument('--updHeader', dest='updHeader', help=""" Whether C
										header file should be generated
										(../driver)""")
	parser.add_argument('--updDocs', dest='updDocs', help="""Whether Lyx 
										doocumentation should be generated.
										(../doc/core)""")											
	return parser.parse_args();


if __name__ == '__main__':
	args = parse_args()
	print( 80 * "*")
	print("**  Generating CAN FD register map")
	print(80 * "*")

	pythonVersion = sys.version.split('.')
	pythonAlias = "python" + pythonVersion[0] + "." + pythonVersion[1]
	print("\n Python version is: %s \n" % pythonAlias)

	if (str_arg_to_bool(args.updVHDL)):
		print("Generating CAN FD memory registers VHDL package...\n")
		os.system("""{} gen_vhdl_package.py --licPath ../LICENSE --xactSpec {} --fieldMap Regs --addrMap Regs --wordWidth 32 --outFile ../src/Libraries/CAN_FD_register_map.vhd --packName CAN_FD_register_map""".format(pythonAlias, args.xactSpec))
		os.system("""{} gen_vhdl_package.py --licPath ../LICENSE --xactSpec {} --fieldMap Frame_format --addrMap Frame_format --wordWidth 32 --outFile ../src/Libraries/CAN_FD_frame_format.vhd --packName CAN_FD_frame_format""".format(pythonAlias, args.xactSpec))
		print("\nDone\n")
	
	if (str_arg_to_bool(args.updHeader)):
		print("Generating CAN FD memory registers Header file...\n")
		os.system("""{} gen_c_header.py --licPath ../lic/gpl_v2.txt --xactSpec {} --addrMap Regs --fieldMap Regs --wordWidth 32 --outFile ../driver/ctu_can_fd_regs.h --headName regs""".format(pythonAlias, args.xactSpec))
		os.system("""{} gen_c_header.py --licPath ../lic/gpl_v2.txt --xactSpec {} --addrMap Frame_format --fieldMap Frame_format --wordWidth 32 --outFile ../driver/ctu_can_fd_frame.h --headName frame""".format(pythonAlias, args.xactSpec))
		print("\nDone\n")
	
	if (str_arg_to_bool(args.updDocs)):
		print("Generating CAN FD memory registers Documentation...\n")
		os.system("""{} gen_lyx_docu.py --xactSpec {} --memMap Regs --wordWidth 32 --lyxTemplate ../doc/core/template.lyx --outFile ../doc/core/registerMap.lyx --chaptName "Register map" --genRegions True --genFiDesc True""".format(pythonAlias, args.xactSpec))
		os.system("""{} gen_lyx_docu.py --xactSpec {} --memMap Frame_format --wordWidth 32 --lyxTemplate ../doc/core/template.lyx --outFile ../doc/core/CANFrameFormat.lyx --chaptName "CAN Frame format" --genRegions False --genFiDesc True""".format(pythonAlias, args.xactSpec))
		print("\nDone\n")
	
	print( 80 * "*")
	print("**  Finished")
	print(80 * "*")
	
