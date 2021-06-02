import os
import os.path
import logging
import json
from pathlib import Path
from yattag import Doc
from typing import Tuple, List, Dict, Any, NewType
from json2html import *

TPslPoint = NewType('TPslPoint', Dict[str, Any])

test_dir = Path(__file__).parent.parent.absolute()
build_dir = test_dir.absolute() / "build"
func_cov_dir = build_dir / "functional_coverage"
psl_dir = func_cov_dir / "coverage_data"
html_dir = func_cov_dir / "html"

dut_top = " "

log = logging.getLogger(__name__)


def merge_psl_coverage_files(out_file: str, in_file_prefix: str) -> None:
    """
    Merge PSL coverage details from multiple files to single file
    """
    if out_file.startswith(in_file_prefix):
        raise ValueError("File name for merging should not have the same prefix as merged files")

    json_out_path = func_cov_dir / out_file
    json_out_list = []  # type: List[TPslPoint]
    for in_filename in psl_dir.glob('{}*.json'.format(in_file_prefix)):
        print("Merging JSON PSL coverage from: {}\n".format(in_filename))
        with in_filename.open('rt') as f:
            json_obj = json.load(f)

        # Add test name to each PSL point
        for psl_point in json_obj["details"]:
            psl_point["test"] = in_filename.with_suffix('').name \
                                           .strip(in_file_prefix)

        json_out_list += json_obj["details"]

    with json_out_path.open('wt') as f:
        json.dump(json_out_list, f, indent=1)


def collapse_psl_coverage_files(non_collapsed):
    """
    Collapses PSL coverage which is output from multiple testcase/testbench
    runs into single psl_coverage output.
    If DUT is instantiated in multiple testbenches, above levels of
    hierarchy from "dut_top" will be ignored and these files will be collapsed.
    E.g. if "dut_top" = "can_top_level",
    then multiple instances of CTU CAN FD will not generate multiple PSL outputs.
    Collapsing policy is following:
        - cover - If at least one of collapsed points is covered -> COVERED
        - assert - If at least one of collapsed points is failed -> FAILED
    Each cover point which is covered has also appended a testcase name where
    it was covered.
    """
    log.info("Collapsing PSL points with common hierarchy below: {}".format(dut_top))
    collapsed = []

    # We do stupid quadratic sort because we don't really care if it is gonna
    # last 10 or 40 seconds... If we ever get to the point that this takes too
    # long, we know that we have reeealy lot of PSL points and we turned into
    # Semiconductor monster!
    for psl_in in non_collapsed:
        found = False
        for psl_out in collapsed:
            # Check if name in output list is equal to searched name from
            # "dut_top" entity down. Skip if not
            in_name = psl_in["name"].split(dut_top)[-1]
            out_name = psl_out["name"].split(dut_top)[-1]
            if (out_name != in_name):
                continue

            if (not ("colapsed_points" in psl_out)):
                psl_out["colapsed_name"] = str(dut_top + in_name)
                psl_out["colapsed_points"] = []

            psl_out["colapsed_points"].append(psl_in)

            # If any of colapsed points is covered -> whole point is covered
            if (psl_in["status"] == "covered"):
                psl_out["status"] = "covered"
                psl_out["finished-count"] += psl_in["finished-count"]

            # If any of colapsed points is failed -> whole point is failed
            if (psl_in["status"] == "failed"):
                psl_out["status"] = "failed"

            # Assertion hits add up for both failed and passed
            if (psl_out["directive"] == "assertion"):
                psl_out["finished-count"] += psl_in["finished-count"]

            found = True
            break

        # Input point was not collapsed into any of output points -> Add directly
        if not found:
            collapsed.append(psl_in)

    return collapsed


def get_collapsed_file_name(psl_point: TPslPoint) -> str:
    """
    Create unique file name for collapsed PSL points
    """
    file_name = dut_top + psl_point["name"].split(dut_top)[-1]
    file_name = file_name.replace(".", "_")
    file_name = file_name.replace(" ", "_")
    file_name = file_name.replace(")", "_")
    file_name = file_name.replace("(", "_")
    file_name = file_name.replace("@", "_")
    file_name = file_name + "_" + str(psl_point["line"])
    return file_name


def load_json_psl_coverage(filename: str):
    """
    Load PSL Coverage JSON file to JSON object.
    """
    psl_cov_path = func_cov_dir / filename

    # Read JSON string from file
    log.info("Loading JSON PSL output: {}".format(psl_cov_path))
    with psl_cov_path.open('rt') as json_file:
        return json.load(json_file)


def split_json_coverage_by_file(json) -> Dict[Path, List[TPslPoint]]:
    """
    Parse input PSL Coverage JSON file. Group PSL endpoints by file.
    Return dictionary in format:
        {filename : psl_points} where psl_points is a list of PSL points in
        filename.
    """
    file_dict = {}  # type: Dict[Path, List[TPslPoint]]
    for psl_point in json:
        file = Path(psl_point["file"])
        # Create new list if first PSL of a file is parsed
        if file not in file_dict:
            file_dict[file] = []
        file_dict[file].append(psl_point)

    return file_dict


def add_html_table_header(doc, tag, text, headers, back_color="White"):
    """
    Add header to HTML table.
    """
    with tag('tr'):
        for header in headers:
            with tag('th', bgcolor=back_color):
                text(header)


def calc_coverage_results(psl_points: List[TPslPoint], psl_type) -> Tuple[int, int]:
    """
    Calculate coverage results from list of PSL points in JSON format.
    """
    ok = 0
    nok = 0
    for psl_point in psl_points:
        if (psl_point["directive"] != psl_type):
            continue
        if (psl_point["status"] == "passed" or
            psl_point["status"] == "covered"):
            ok += 1
        else:
            nok += 1
    return ok, nok


def calc_coverage_color(coverage: float) -> str:
    """
    Return color based on coverage result.
    """
    if (coverage < 0 or coverage > 100):
        raise ValueError("Invalid coverage input should be between 0 - 100 %")

    if (coverage > 90):
        return "Lime"
    elif (coverage > 80):
        return "Orange"
    elif (coverage > 70):
        return "OrangeRed"
    else:
        return "Red"


def print_cov_cell_percentage(doc, tag, text, psl_points: List[TPslPoint],
                              coverage_type, merge_abs_vals) -> None:
    """
    """
    ok, nok = calc_coverage_results(psl_points, coverage_type)
    summ = max(1, ok + nok)
    percents = ok/summ * 100
    color = calc_coverage_color(percents)

    if (merge_abs_vals):
        if (ok + nok > 0):
            with tag('td', bgcolor=color):
                text("({}/{}) {:.1f}%".format(ok, summ, percents))
        else:
            with tag('td', bgcolor="Silver"):
                text("NA")
    else:
        with tag('td'):
            text(ok)
        with tag('td'):
            text(nok)

        if (ok + nok > 0):
            with tag('td', bgcolor=color):
                text("{:.1f}%".format(percents))
        else:
            with tag('td', bgcolor="Silver"):
                text("NA")


def add_psl_html_header(doc, tag, text, filename, psl_points: List[TPslPoint]):
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
                    print_cov_cell_percentage(doc, tag, text, psl_points, \
                        coverage_type, merge_abs_vals=False)


def add_non_colapsed_psl_table_entry(doc, tag, text, psl_point: TPslPoint,
                                     def_bg_color="White"):
    """
    Add HTML table entry for non-collapsed PSL functional coverage point.

    """
    with tag('td'):
        text(psl_point["name"].split(".")[-1])
    with tag('td'):
        text(psl_point["test"])
    with tag('td', width="50%", style="word-break:break-all;"):
        text(dut_top + psl_point["name"])
    with tag('td'):
        text(psl_point["line"])
    with tag('td'):
        text(psl_point["finished-count"])

    if (psl_point["status"] == "covered" or \
        psl_point["status"] == "passed"):
        color = "Lime"
    else:
        color = "red"

    with tag('td', ('bgcolor',color)):
        text(psl_point["status"])


def add_colapsed_psl_table_entry(doc, tag, text, psl_point: TPslPoint,
                                 def_bg_color="White"):
    """
    Add HTML table entry for collapsed PSL functional coverage point. Adds
    llink reference to collapsed entries on separate site.
    """
    with tag('td'):
        text(psl_point["name"].split(".")[-1])

    with tag('td'):
        file_name = get_collapsed_file_name(psl_point)
        with tag('a', href=file_name+".html"):
            text("Open collapsed tests")
    with tag('td'):
        text(dut_top + psl_point["name"].split(dut_top)[-1])
    with tag('td'):
        text(psl_point["line"])
    with tag('td'):
        text(psl_point["finished-count"])

    if (psl_point["status"] == "covered" or \
        psl_point["status"] == "passed"):
        color = "Lime"
    else:
        color = "red"

    with tag('td', ('bgcolor',color)):
        text(psl_point["status"])


def add_psl_table_entry(doc, tag, text, psl_point: TPslPoint,
                        def_bg_color="White"):
    """
    Add PSL point in JSON format to HTML table. For collapsed entries,
    overall result is shown and link to collapsed points is inserted.
    """
    # Add default entry (single or collapsed)
    with tag('tr', ('bgcolor',def_bg_color)):
        if ("colapsed_points" in psl_point):
            add_colapsed_psl_table_entry(doc, tag, text, psl_point, def_bg_color="White")
        else:
            add_non_colapsed_psl_table_entry(doc, tag, text, psl_point, def_bg_color="White")

    # Create separate page with collapsed PSL points for this PSL statement
    # Add unique filename
    if ("colapsed_points" in psl_point):
        file_name = html_dir / get_collapsed_file_name(psl_point)
        create_psl_file_page(file_name, psl_point["colapsed_points"])


def create_psl_file_page(filename: Path, psl_points: List[TPslPoint]) -> None:
    """
    Create HTML file with list of PSL coverage statements.
    """
    parsed_file_name = filename.name
    html_cov_path = html_dir / '{}.html'.format(filename.name)

    doc, tag, text = Doc().tagtext()

    # Add Common header
    add_psl_html_header(doc, tag, text, parsed_file_name, psl_points)

    # Add "Cover" and "Assertion" points
    psl_types = [{"name" : "Cover Points" , "type" : "cover"}, \
                 {"name" : "Assertions", "type" : "assertion"}]
    for psl_type in psl_types:
        with tag('p'):
            with tag('table', width='100%', border="1px solid black"):
                with tag('caption'):
                    with tag('font', size=5):
                        text(psl_type["name"])
                titles = ["PSL Point Name", "Test name", "Full Path Name", "Line", "Count", "Status"]
                add_html_table_header(doc, tag, text, titles, back_color="Peru")
                for psl_point in psl_points:
                    if (psl_point["directive"] == psl_type["type"]):
                        add_psl_table_entry(doc, tag, text, psl_point)

    with html_cov_path.open('wt', encoding='utf-8') as html_file:
        html_file.write(doc.getvalue())


def create_psl_file_refs_table(doc, tag, text, psl_by_files: Dict[Path, List[TPslPoint]]) -> None:
    """
    Create entries to HTML table for each file. Calculates
    coverage summary for each file. Adds Reference to files.
    """
    for file_name, psl_list in psl_by_files.items():
        with tag('tr'):
            with tag('td'):
                name = file_name.name
                with tag('a', href=os.path.join("html", name + ".html")):
                    text(name)
            coverage_types = ["cover", "assertion"]
            for coverage_type in coverage_types:
                print_cov_cell_percentage(doc, tag, text, psl_list, \
                        coverage_type, merge_abs_vals=True)


def create_psl_report(psl_by_files: Dict[Path, List[TPslPoint]], psl_orig) -> None:
    """
    Generates PSL report. Each list within psl_by_files has separate
    HTML page. Summary page is created from psl_orig
    """
    # Create HTML page for each source file
    for file_name, psl_list in psl_by_files.items():
        create_psl_file_page(file_name, psl_list)

    html_rep_path = func_cov_dir / "functional_coverage_report.html"

    doc, tag, text = Doc().tagtext()

    # Add Common Header
    add_psl_html_header(doc, tag, text, "TOP LEVEL", psl_orig)

    with tag('p'):
        with tag('table', width="100%", border="1px solid black"):
            header = ["File name", "Coverage", "Asserts"]
            add_html_table_header(doc, tag, text, header, back_color="Peru")
            create_psl_file_refs_table(doc, tag, text, psl_by_files)

    with html_rep_path.open('wt', encoding='utf-8') as html_file:
        html_file.write(doc.getvalue())


if __name__ == "__main__":
    #dut_top = "can_top_level"
    dut_top = "."

    # Merge PSL coverage files from all Testcases
    merge_psl_coverage_files("merged_psl.json", "psl_cov")
    json_orig = load_json_psl_coverage("merged_psl.json")
    json_by_file = split_json_coverage_by_file(json_orig)

    # Colapse for each source file and also Merge all colapsed into
    # single list so that overal coverage is calculated only out of colapsed
    # psl points!
    json_by_file_colapsed = {}
    json_together_colapsed = []
    for filename, psls_for_file in json_by_file.items():
        colapsed = collapse_psl_coverage_files(psls_for_file)
        json_by_file_colapsed[filename] = colapsed
        json_together_colapsed += colapsed

    # Create PSL report
    create_psl_report(json_by_file_colapsed, json_together_colapsed)
