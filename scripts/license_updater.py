################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
##   
##   Script for updating license in the header source codes of CAN FD IP Core.
##   
##    
##   Arguments:
##		lic_path 	- File with license which should be placed to header of the
##                    all source code files.
##		sup_files	- Supported file endings which should have the license 
##					  updated.
##     subfolders	- Which sub-folders should be updated
##
##	Example of usage from IDLE shell
##     
##
##	Revision history:
##		13.12.2017	Implemented the "license_updater"
##
################################################################################

import os
import sys
import shutil
import re

################################################################################
## Global variables
################################################################################



## Counter for files where the license was updated
file_counter = 0

global sup_files
global sub_folders


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


################################################################################
## Process the file and change the license header, if the file type is in the
## list of supported file extensions.
################################################################################
def process_file(filename):
	
	global sup_files
	file_ext_match = False
	for ext in sup_files:
		if (filename.endswith(ext)):
			file_ext_match = True
	
	if (file_ext_match == False):
		return
		
	print("Processing file: "+filename)
	
	file = open (filename)
	file_text = file.read()
	print(file_text)
		


################################################################################
## Iterate through directory and call recursively on sub-directories.
## Call "process_file" on files.
################################################################################
def iterate_dir(dir_path):

	print ("Processing directory: "+dir_path)
			
	## Process files and directories in this directory
	sub_dirs = os.listdir(dir_path)
	for fold in sub_dirs:
		full_dir = os.path.join(dir_path,fold)
		
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
	print ("")
	
	## Check valid input arguments
	arg_count = 4  ## 3 arguments + script itself  
	if (len(sys.argv) != arg_count):
		print(" Incorrect amount of arguments!")
		print_help()
		print(" Exiting!")
		sys.exit(1)
		
    ## Command line arguments
	lic_path = sys.argv[1]
	sup_files = sys.argv[2]
	sup_files = sup_files.replace("[","")
	sup_files = sup_files.replace("]","")
	sup_files = sup_files.split(",")
	sub_folders = sys.argv[3]
	sub_folders = sub_folders.replace("[","")
	sub_folders = sub_folders.replace("]","")
	sub_folders = sub_folders.split(",")
	
	## Check license existance and load it
	scr_path=os.path.dirname(os.path.abspath(__file__))	
	lic_full_path=os.path.join(scr_path,lic_path)	
	if (os.path.isfile(lic_full_path)):
		lic_file = open(lic_full_path,"r")
		lic_text = lic_file.read()
		print ("Loading license...")
		print ("")
		##print ("License text")
		##print (lic_text)
	else:
		print ("Invalid license file")
		sys.exit(1)
	
	#Check file extensions
	for ext in sup_files:
		if (not ext.startswith(".")):
			print ("Invalid file extension '"+ext+"' File extenstions should "\
					"start with '.'")
			sys.exit(1)
	
	#Replace the license in the subfolders
	for dir in sub_folders:
		dir_path=os.path.join(scr_path,'..')
		dir_path=os.path.join(dir_path,dir)
		if (os.path.isdir(dir_path)):
			iterate_dir(dir_path)
		else:
			print(dir_path+" directory does not exist")
		
if __name__ == "__main__":
	main()
