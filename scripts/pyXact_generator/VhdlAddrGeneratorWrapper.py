################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Class for generation of VHDL package from IP-XACT specification. Register
##   map addresses, bit field offsets and enums are generated. Two separate
##   register maps can be specified: one for bit fields, one for addresses.
##   
##   In case of CAN FD Core the register map is specified with two register 
##   maps. 8-bit map with register fields described. 32 bit register maps with
##   name aliases used on 32 bit Avalon and AXI.
##
##	Revision history:
##		16.01.2018	Implemented the script
##      25.11.2018  Joined field and address map to a single memory map/
##                  Re-implemented script to be Python class
##
################################################################################

import argparse
import sys
import time
import importlib.util
import os
import inspect
import math

from .gen_lib import *
from .ip_xact.vhdl_addr_generator import VhdlAddrGenerator


class VhdlAddrGeneratorWrapper():

    # File with license which should be placed to header of the all source code files
    licPath = ""

    # Path to a IP-XACT specification file with register maps
    xactSpec = ""

    # Name of the IP-XACT Memory map which should be used for VHDL package generatio.
    memMap = None
    
    # Size of the access bus word. Register bit field offsets are concatenated into 
    # word width size instead of simple offset from beginning of register. (E.g. 32 bit  ->
    # bitfields from first four 8-bit register are concatenated into 32 bit values)
    wordWidth = 32

    # Name of the VHDL package to create
    packName = ""

    # Output where to write the VHDL package.
    outFile = ""


    def do_update(self):

	    with open(self.xactSpec) as spec_file:
		    name = None
		    offset = 0
		    
            # Load IP-Xact component
		    component = Component()
		    component.load(spec_file)
			    
		    with open_output(self.outFile) as of:
			    
			    vhdlGen = VhdlAddrGenerator(component, self.memMap, self.wordWidth)
			    vhdlGen.set_of(of)
			    
			    if (self.licPath != ""):
				    lic_text = load_license(self.licPath)
				    write_license(lic_text, '-', of)
			    
			    vhdlGen.create_addrMap_package(self.packName)
			    
			    vhdlGen.commit_to_file()

    if __name__ == '__main__':
        self.do_update()

