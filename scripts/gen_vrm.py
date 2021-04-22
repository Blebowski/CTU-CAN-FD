################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Script for generation of VRM. What has been verified is extracted from
##   common headers
##   
##   Arguments:
##		configPath  - Path to test config (the same as for test run)
##
##	Revision history:
##		09.01.2020	Implemented the script
##
################################################################################

import argparse
import sys
import time
import importlib.util
import os
import inspect
import math
import yaml

from VRM_Extractor.VRMProcessor import VRMProcessor
from VRM_Extractor.VRMGenerator import VRMGenerator

from pathlib import Path


def parse_args():
	parser = argparse.ArgumentParser(
				description="""Script for generation of VRM. What has been 
                               verified is extracted from common headers""")

	parser.add_argument('--configPath', dest='configPath', help="""Path to 
                                test config (the same as for test run)""")

	return parser.parse_args();


def read_config(configPath):
	"""
	Read CTU CAN FD test configuration and obtain list of file paths with test
	source codes (these can be then searched for VRM entries).
	File paths are returned relative to project root location!
	"""
	with Path(configPath).open("r") as f:
		cfg = yaml.safe_load(f);

	test_dir = Path("../test")
	ftr_dir = test_dir / "main_tb" / "feature_tests"
	unit_dir = test_dir / "unit"
	ref_dir = test_dir / "reference"
	fileList = []

	if ("feature" in cfg):
		ftr = cfg['feature']
		for ftr_tst in ftr['tests'].items():
			print("Processing feature test: {}".format(ftr_tst[0]));
			fileList.append(ftr_dir / "{}_ftest.vhd".format(ftr_tst[0]))

	if ("unit" in cfg):
		uni = cfg['unit']
		for unit_tst in uni['tests'].items():
			t_dict = list(unit_tst)[1]
			test_name = list(unit_tst)[0]
			print("Processing unit test {}".format(test_name))
			tst_name = unit_dir / test_name / "{}_tb.vhd".format(str(test_name).split("/")[-1])
			fileList.append(tst_name)

#	if ("reference" in cfg):
#		fileList.append(ref_dir / "tb_reference.vhd")

	print(fileList)
	return fileList


if __name__ == '__main__':
	args = parse_args()

	print( 80 * "*")
	print("**  Generating CTU CAN FD VRM!")
	print(80 * "*")

	pythonVersion = sys.version.split('.')
	pythonCmd = "python" + pythonVersion[0] + "." + pythonVersion[1]
	print("\n Python version is: %s \n" % pythonCmd)

	test_files = read_config(args.configPath)
	vrmProc = VRMProcessor(test_files)
	results = vrmProc.extractTestInfo()
	vrmGen = VRMGenerator(results, "VRM.html")
	vrmGen.gen_VRM("CTU CAN FD - Test information")

	print( 80 * "*")
	print("**  Finished")
	print(80 * "*")
	
