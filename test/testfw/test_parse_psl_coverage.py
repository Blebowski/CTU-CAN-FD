import os
import sys
from json2html import *
import random
import logging
from os.path import join, abspath
from pathlib import Path
import json
from yattag import Doc

test_dir = Path(Path(abspath(__file__)).parent).parent
build_dir = os.path.join(str(test_dir.absolute()), "build")
func_cov_dir = os.path.join(str(build_dir), "functional_coverage")

log = logging.getLogger(__name__)

def load_json_psl_coverage():
	"""
	Load PSL Coverage JSON file to JSON object.
	"""
	psl_cov_path = os.path.join(func_cov_dir, "psl_coverage.json")
	
	# Read JSON string from file
	log.info("Loading JSON PSL output: {}".format(psl_cov_path))	
	json_file = open(psl_cov_path, 'r')
	return json.load(json_file)


def split_json_coverage_by_file(json):
	"""
	Parse input PSL Coverage JSON file. Group PSL endpoints by file.
	Return dictionary in format:
		{filename : psl_points} where psl_points is a list of PSL points in
		filename.
	"""
	file_dict = {}
	for psl_point in json["details"]:

		# Create new list if first PSL of a file is parsed
		if (not(psl_point["file"] in file_dict)):
			file_dict[psl_point["file"]] = []
		
		file_dict[psl_point["file"]].append(psl_point)

	return file_dict


def add_html_table_header(doc, tag, text, headers, back_color="White"):
	"""
	Add header to HTML table.
	"""
	with tag('tr'):
		for header in headers:
			with tag('th', bgcolor=back_color):
				text(header)


def calc_coverage_results(psl_points, psl_type):
	"""
	Calculate coverage results from list of PSL points in JSON format.
	"""
	ok = 0
	nok = 0
	for psl_point in psl_points:
		if (psl_point["directive"] != psl_type):
			continue;
		if (psl_point["status"] == "passed" or
			psl_point["status"] == "covered"):
			ok += 1
		else:
			nok +=1
	return [ok, nok]


def calc_coverage_color(coverage):
	"""
	Return color based on coverage result.
	"""
	if (coverage < 0 or coverage > 100):
		log("Invalid coverage input should be between 0 - 100 %")
		return

	if (coverage > 90):
		return "Lime"
	elif (coverage > 80):
		return "Orange"
	elif (coverage > 70):
		return "OrangeRed"
	else:
		return "Red"


def add_psl_html_header(doc, tag, text, filename, psl_points):
	"""
	Create HTML page header with info about coverage data within list of
	PSL points in JSON format.
	"""
	with tag('table', width='100%', border=0, cellspacing=0, cellpadding=0):
		with tag('tr'):			
			with tag('th', ('class','title')):
				with tag('font', size=10):
					text("GHDL PSL Functional coverage report")
			with tag('table', width='100%', border="1px solid black"):
				headers = ["Filename"]
				headers.append("Covered")
				headers.append("Not-Covered")
				headers.append("Functional coverage")
				headers.append("Passed")
				headers.append("Failed")
				headers.append("Assertions passed")
				add_html_table_header(doc, tag, text, headers, back_color="Aquamarine")

				with tag('td'):
					text(filename)

				# Calculate results for each type
				coverage_types = ["cover", "assertion"]
				for coverage_type in coverage_types:
					[ok, nok] = calc_coverage_results(psl_points, coverage_type)
					summ = max(1, ok + nok)
					percents = ok/summ * 100
					color = calc_coverage_color(percents)
					with tag('td'):
						text(ok)
					with tag('td'):
						text(nok)
					with tag('td', bgcolor=color):
						text("{}%".format(percents))


def add_psl_table_entry(doc, tag, text, psl_point):
	"""
	Add PSL point in JSON format to HTML table.
	"""
	with tag('tr'):
		with tag('td'):
			text(psl_point["name"].split(".")[-1])
		with tag('td'):
			text(psl_point["line"])
		with tag('td'):
			text(psl_point["count"])

		if (psl_point["status"] == "covered" or \
			psl_point["status"] == "passed"):
			color = "Lime"
		else:
			color = "red"

		with tag('td', ('bgcolor',color)):
			text(psl_point["status"])


def create_psl_file_page(filename, psl_points):
	"""
	Create HTML file with list of PSL coverage statements.
	"""
	html_cov_path = os.path.join(func_cov_dir, "{}.html".format(filename))
	html_file = open(html_cov_path, 'w')

	doc, tag, text = Doc().tagtext()

	# Add Common header
	add_psl_html_header(doc, tag, text, filename, psl_points)

	# Add "Cover" and "Assertion" points
	psl_types = [{"name" : "Cover Points" , "type" : "cover"}, \
				 {"name" : "Assertions" , "type" : "assertion"}]
	for psl_type in psl_types:
		with tag('p'):
			with tag('table', width='100%', border="1px solid black"):
				with tag('caption'):
					with tag('font', size=5):
						text(psl_type["name"])
				titles = ["PSL Point Name", "Line", "Count", "Status"]
				add_html_table_header(doc, tag, text, titles, back_color="Peru")
				for psl_point in psl_points:
					if (psl_point["directive"] == psl_type["type"]):
						add_psl_table_entry(doc, tag, text, psl_point)

	html_file.write(doc.getvalue())
	html_file.close()


def create_psl_file_refs_table(doc, tag, text, psl_by_files):
	"""
	Create entries to HTML table for each file. Calculates
	coverage summary for each file. Adds Reference to files.
	"""
	for file_name, psl_list in psl_by_files.items():
		with tag('tr'):
			with tag('td'):
				with tag('a', href=file_name+".html"):
					text(file_name)
			coverage_types = ["cover", "assertion"]
			for coverage_type in coverage_types:
				[ok, nok] = calc_coverage_results(psl_list, coverage_type)
				summ = max(1, ok + nok)
				percents = ok/summ * 100
				color = calc_coverage_color(percents)
				with tag('td', bgcolor=color):
					text("({}/{}) {}%".format(ok, summ, percents))


def create_psl_report(psl_by_files, psl_orig):
	"""
	Generates PSL report. Each list within psl_by_files has separate
	HTML page. Summary page is created from psl_orig
	"""
	# Create HTML page for each source file
	for file_name, psl_list in psl_by_files.items():
		create_psl_file_page(file_name, psl_list)

	html_rep_path = os.path.join(func_cov_dir, "functional_coverage_report.html")
	html_file = open(html_rep_path, 'w')

	doc, tag, text = Doc().tagtext()

	# Add Common Header
	add_psl_html_header(doc, tag, text, "TOP LEVEL", psl_orig)

	with tag('p'):
		with tag('table', width="100%", border="1px solid black"):
			header = ["File name", "Coverage", "Asserts"]
			add_html_table_header(doc, tag, text, header, back_color="Peru")
			create_psl_file_refs_table(doc, tag, text, psl_by_files)
	
	html_file.write(doc.getvalue())
	html_file.close()


if __name__ == "__main__":
	json_orig = load_json_psl_coverage()
	json_parsed = split_json_coverage_by_file(json_orig)
	create_psl_report(json_parsed, json_orig["details"])

