################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Script for updating license in the header source codes of CAN FD IP Core.
##   Supports following file extensions: ".vhd" , ".tcl" , ".h" , ".c", ".cpp"
##    
##   Arguments:
##		lic_path 	- File with license which should be placed to header of the
##                    all source code files.
##		sup_files	- Supported file endings which should have the license 
##					  updated.
##     subfolders	- Which sub-folders should be updated
##
##	Example of usage from IDLE shell
##     python ./license_updater.py 'myPrettyLicense.txt' ['.c','.vhd'] 
##				['src','test']
##
##	Revision history:
##		13.12.2017	Implemented the "license_updater"
##
################################################################################

import os
import sys
import shutil
import re
import io

from io import StringIO

################################################################################
## Global variables
################################################################################

## Counter for files where the license was updated
file_counter = 0

global sup_files
global sub_folders
global lic_text


################################################################################
## Help about the script will be printed to command line
################################################################################
def print_help():
	print (" License updater script should be used from Shell like so:")
	print ("")
	print (" python ./license_updater.py <lic_path> <sup_files> <subfolders>")
	print ("")
	print (" Arguments:")
	print ("	lic_path	Location of license file which should be used as "\
			"				source of the license location (Relative location)")
	print ("	sup_files	List with file extensions where license should be "\
			" 				replaced")
	print ("	sub_folders	List of folders which should be recursively searched"\
			"				and licenses headers should be replaced")
	print ("")
	print ("Example:")
	print (" python ./license_updater.py \n"\
			" 		'myPrettyLicense.txt' ['.c','.vhd'] ['src','test']")


def write_license(lic_text, comment_char, file):
	
	line_length = 80
	
	# Write the initial line (TCL, VHDL)
	if (comment_char == "-" or comment_char == "#"):
		for i in range(0, line_length):
			file.write(comment_char)
			
	# Write the initial line (C)
	elif (comment_char == "*"):
		file.write("/")
		for i in range(0, line_length-1):
			file.write(comment_char)
	else:
		print("Unsuported Comment character")
	
	file.write("\n")
	
	# Write rest of the lines	
	buf = io.StringIO(lic_text)
	i = 0
	while True:
		i = i + 1

		# Read the line from text of the license
		lic_line = buf.readline()
		if (not(str.find("<licend1234>", lic_line) == -1)):
			break
		
		# Write Begining of the line
		if (comment_char == "-" or comment_char == "#"):
			file.write(comment_char+comment_char+" ")
		elif (comment_char=="*"):
			file.write(" "+comment_char+" ")
	
		# Write Rest of the line
		file.write(lic_line)
		
	# Write the final line
	if (comment_char == "-" or comment_char == "#"):
		for i in range(0, line_length):
			file.write(comment_char)
	elif (comment_char=="*"):
		for i in range(0, line_length-1):
			file.write(comment_char)
		file.write("/")
		
	file.write("\n")


################################################################################
## Reads the source code from one file until two files
################################################################################
def write_source(source_file, dest_file, comment_sign):
	## Read the first file line because this is always commented
	source_file.readline()
	
	## Read the license lines
	## First or second line of the source code is
	## the "comment character"
	line = source_file.readline()
	while (line.startswith(comment_sign) or line[1]==comment_sign):	
		line = source_file.readline()
		if (len(line)==1):
			break
				 
	dest_file.write("\n")
				 
	## Now read rest of the source code and copy to dest
	while True:
		char = source_file.read()
		if (char == ""):
			break
		dest_file.write(char)
		
	
	
		

################################################################################
## Process the file and change the license header, if the file type is in the
## list of supported file extensions.
################################################################################
def process_file(filename):
	
	global sup_files
	file_ext_match = False
	ext_type = ""
	comment_sign = ""
	
	for ext in sup_files:
		if (filename.endswith(ext)):
			file_ext_match = True
			ext_type = ext
	
	if (file_ext_match == False):
		return
		
	print("Processing file: " + filename)
	
	## Check the comment sign based on file type
	if ((ext_type == ".c") or (ext_type == ".cpp")):
		comment_sign = "*"
	elif (ext_type == ".h"): 
		comment_sign = "*"
	elif (ext_type == ".tcl"): 
		comment_sign = "#"
	elif (ext_type == ".vhd"): 
		comment_sign = "-"
	else:
		comment_sign = "-"

	## Read the file content
	file = open (filename,"r")
	temp_file = open ("temp.txt","wt")
	
	## Write the new license to the temp file
	write_license(lic_text, comment_sign, temp_file)
	
	## Write rest of the file after license update
	write_source(file, temp_file, comment_sign)
	temp_file.close()
	file.close()
	
	## Replace the original file and erase the temp file
	os.remove(filename)
	os.rename("temp.txt", filename)
	

################################################################################
## Parse the command line arguments and fill according global variables
## (I know global variables are not nice, but sufficient in this case...)
################################################################################
def parse_args():
	global sup_files
	global sub_folders
	global lic_path
	global src_path
	
	src_path=os.path.dirname(os.path.abspath(__file__))	
	
	lic_path = sys.argv[1]
	sup_files = sys.argv[2]
	sup_files = sup_files.replace("[","")
	sup_files = sup_files.replace("]","")
	sup_files = sup_files.split(",")
	sub_folders = sys.argv[3]
	sub_folders = sub_folders.replace("[","")
	sub_folders = sub_folders.replace("]","")
	sub_folders = sub_folders.split(",")

################################################################################
## Load license and place it in global variable
################################################################################
def load_license(lic_path):
	global lic_text
	global src_path
	
	src_path=os.path.dirname(os.path.abspath(__file__))	
	
	lic_full_path=os.path.join(src_path, lic_path)
	print(lic_full_path)
	if (os.path.isfile(lic_full_path)):
		lic_file = open(lic_full_path,"r")
		lic_text = lic_file.read()
		print ("Loading license...")
		print ("")
		return lic_text
		##print ("License text")
		##print (lic_text)
	else:
		print ("Invalid license file")
		sys.exit(1)


################################################################################
## Iterate through directory and call recursively on sub-directories.
## Call "process_file" on files.
################################################################################
def iterate_dir(dir_path):

	print ("Processing directory: "+dir_path)
			
	## Process files and directories in this directory
	sub_dirs = os.listdir(dir_path)
	for fold in sub_dirs:
		full_dir = os.path.join(dir_path, fold)
		
		## Execute recursively on directories
		if (os.path.isdir(full_dir)):
			iterate_dir(full_dir)
		
		## Process the files
		if (os.path.isfile(full_dir)):
			process_file(full_dir)
			

################################################################################
## Main function which executes the license replacement
################################################################################
def main():
	
	global sup_files
	global sub_folders
	global lic_text
	global lic_path
	global src_path
	print ("")
	
	## Check valid input arguments
	arg_count = 4  ## 3 arguments + script itself  
	if (len(sys.argv) != arg_count):
		print(" Incorrect amount of arguments!")
		print_help()
		print(" Exiting!")
		sys.exit(1)
		
    ## Parse Command line arguments
	parse_args()
	
	## Check license existance and load it
	load_license(lic_path)
	
	#Check file extensions
	for ext in sup_files:
		if (not ext.startswith(".")):
			print ("Invalid file extension '"+ext+"' File extenstions should "\
					"start with '.'")
			sys.exit(1)
	
	#Replace the license in the subfolders
	for dir in sub_folders:
		dir_path=os.path.join(src_path,'..')
		dir_path=os.path.join(dir_path, dir)
		if (os.path.isdir(dir_path)):
			iterate_dir(dir_path)
		else:
			print(dir_path+" directory does not exist")
		
if __name__ == "__main__":
	main()
