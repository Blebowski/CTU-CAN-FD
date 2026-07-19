#!/usr/bin/env python3
################################################################################
##
##   CTU CAN FD IP Core
##
##   Checks that every VHDL file listed in a source-list-file (SLF), e.g.
##   src/slf_rtl.yml, analyses cleanly under NVC with "--check-synthesis"
##   and "-Werror". This catches common simulation/synthesis mismatches
##   such as incomplete process sensitivity lists.
##
##   Each file is analysed with a separate "nvc -a" invocation (into the
##   same work library, in source_list order, so later files can resolve
##   packages/entities declared by earlier ones).
##
##   Usage:
##      ./check_sensitivity_lists.py <path-to-slf.yml>
##
################################################################################

import argparse
import os
import subprocess
import sys
import tempfile

import yaml


def parse_args():
    parser = argparse.ArgumentParser(
        description="""Analyse all VHDL files from a source-list-file (SLF)
                        YAML with NVC's --check-synthesis and -Werror.""")
    parser.add_argument("slf_path", help="Path to the SLF YAML file (e.g. src/slf_rtl.yml)")
    return parser.parse_args()


def load_slf(slf_path):
    with open(slf_path) as f:
        return yaml.safe_load(f)


def main():
    args = parse_args()

    slf = load_slf(args.slf_path)
    library = slf["library"]
    slf_dir = os.path.dirname(os.path.abspath(args.slf_path))

    with tempfile.TemporaryDirectory() as work_dir:
        for entry in slf["source_list"]:
            file_path = os.path.join(slf_dir, entry["file"])

            cmd = [
                "nvc",
                "--std=2008",
                f"--work={library}",
                "-a",
                "--check-synthesis",
                "-Werror",
                "--psl",
                file_path,
            ]

            print(f"Analysing: {entry['file']}")
            result = subprocess.run(cmd, cwd=work_dir)

            if result.returncode != 0:
                print(f"\nSensitivity list / synthesis check FAILED on: {entry['file']}",
                      file=sys.stderr)
                sys.exit(1)

    print("\nAll files passed sensitivity list / synthesis check.")


if __name__ == "__main__":
    main()
