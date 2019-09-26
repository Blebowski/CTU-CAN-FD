################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Script for building released version of CTU CAN FD code. Released version
##   contains:
##      1. Processed RTL (verification constructs removed)
##      2. List file with all the source code references.
##      3. Built documentation (System Architecture, Datasheet)
##      4. TODO: Verification -> Reports? Functional Coverage? TB?
##      5. TODO: Should we also include Vivado component??
##
##   This script does not intend to replace release tags in CTU CAN FD GIT
##   repository. It intends to create "deliverable" version of CTU CAN FD
##   with minimum dependencies (e.g. for building the docs). 
##
##   IMPORTANT: RTL release will remove ALL code in the RTL files which are
##              between following markers:
##              
##              <RELEASE_OFF>
##                  all code here will be removed.
##              <RELEASE_ON>
##
##              Markers are intended to be in code comments!
##
##              This is done to save simulation time for an integrator. Since
##              GHDL does not support external names, most of assert, cover
##              points are within RTL code itself! These create significant
##              simulation performance problems and are redundant for
##              integrator, since integrator does not intend to verify the IP,
##              he/she wishes to use the IP!
##
###############################################################################
##
##   Arguments:
##		output_dir  Output directory where released version will be built.
##
##  Note: Build adds actual GIT commit Hash as a suffix of directory filename!
##
##	Revision history:
##		20.08.2019	Implemented the script
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
import subprocess

from pathlib import Path

import pyXact_generator

from pyXact_generator.gen_lib import *

RELEASE_ON_TAG = "<RELEASE_ON>"
RELEASE_OFF_TAG = "<RELEASE_OFF>"

DATASHEET_NAME = "Progdokum"
ARCHITECTURE_DOC_NAME = "ctu_can_fd_architecture"

def parse_args():
    parser = argparse.ArgumentParser(
			    description="""Script for building release of CTU CAN FD""")

    parser.add_argument('--output_dir', dest='output_dir', \
                        help="""Directory where release will be built""")

    return parser.parse_args();



if __name__ == '__main__':
    args = parse_args()

    print( 80 * "*")
    print("**  Building release of CTU CAN FD!")
    print(80 * "*")

    pythonVersion = sys.version.split('.')
    pythonCmd = "python" + pythonVersion[0] + "." + pythonVersion[1]
    print("\n Python version is: %s \n" % pythonCmd)

    # Create Release directory
    proc_res = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], stdout=subprocess.PIPE)
    git_hash = str(proc_res.stdout.decode("utf-8")).strip("\n")
    print("Current GIT commit hash: " + git_hash)

    build_dir = Path(args.output_dir + "_{}".format(git_hash))
    os.system("rm -rf {}".format(build_dir))
    os.system("mkdir {}".format(build_dir))
    os.system("mkdir {}".format(build_dir / "src"))
    os.system("mkdir {}".format(build_dir / "doc"))
    print("Build directory: {}\n".format(build_dir))

    # Copy sources, create List file
    repo_root = Path(__file__).absolute().parent.parent
    src_dir = repo_root / "src"
    with open(build_dir / "src.lst", 'w') as list_file:
        for vhdl_file in src_dir.glob("**/*.vhd"):
            print("Processing source file: {}".format(vhdl_file))
            os.system("cp {} {}".format(vhdl_file, build_dir / "src"))
            list_file.write("src/" + vhdl_file.name + "\n")

    # Walk through the released sources, remove lines which should not be Released
    for vhdl_file in build_dir.glob("src/*.vhd"):
        with open(vhdl_file, "r") as orig_fd:
            lines = orig_fd.readlines()
        with open(vhdl_file, "w") as new_fd:
            erase_lines = False
            for line in lines:

                # Turn on replacements
                if (RELEASE_OFF_TAG in line):
                    erase_lines = True

                # Copy lines if not erased
                if (not erase_lines):
                    new_fd.write(line)

                # Turn off replacements
                if (RELEASE_ON_TAG in line):
                    erase_lines = False

    # Build Documentation
    print("\n\n Exporting Datasheet ... \n\n")
    docs_path = repo_root / "doc" / "core"
    os.system("lyx -e pdf2 {}".format(docs_path / "{}.lyx".format(DATASHEET_NAME)))
    os.system("mv {}.pdf {}".format(docs_path / DATASHEET_NAME, build_dir / "doc" / "Datasheet.pdf"))
    print("\n\n Done \n\n")

    print("\n\n Exporting System Architecture ... \n\n")
    os.system("lyx -e pdf2 {}".format(docs_path / "{}.lyx".format(ARCHITECTURE_DOC_NAME)))
    os.system("mv {}.pdf {}".format(docs_path / ARCHITECTURE_DOC_NAME, build_dir / "doc" / "System_architecture.pdf"))
    print("\n\n Done \n\n")


    print( 80 * "*")
    print("**  Finished")
    print(80 * "*")

