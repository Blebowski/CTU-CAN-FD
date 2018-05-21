# run sanity test using vunit
# Requires: vunit
# $ python run.py # select automatically
# $ # select manually
# $ VUNIT_SIMULATOR=modelsim PATH=$PATH:$MODELSIM_BIN python run.py
# $ VUNIT_SIMULATOR=ghdl python run.py

from os.path import join, dirname
from vunit import VUnit
from glob import glob
from pprint import pprint
import os
import signal
import subprocess

def get_children_pids(parent_pid):
    cmd = subprocess.run("ps -o pid --ppid {} --noheaders".format(parent_pid), shell=True, stdout=subprocess.PIPE, check=False)
    out = cmd.stdout.decode('ascii')
    return [int(pid_str) for pid_str in out.split() if int(pid_str) != parent_pid]

def recursive_kill(pid, sig=signal.SIGTERM):
    children = get_children_pids(pid)
    for child in children:
        recursive_kill(child)
        try:
            os.kill(child, sig)
        except ProcessLookupError as e:
            pass

# ghdl creates a new process group for itself and fails to kill its child when it receives a signal :(
def sighandler(signo, frame):
    signal.signal(signo, signal.SIG_DFL)
    recursive_kill(os.getpid(), signo)
    # restore the handler, because vunit swallows the resulting exception
    signal.signal(signo, sighandler)

signal.signal(signal.SIGTERM, sighandler)
signal.signal(signal.SIGINT, sighandler)
# ------------------------------------------------------------------------------

root = dirname(__file__)

ui = VUnit.from_argv()
lib = ui.add_library("lib")
for pattern in ['../src/**/*.vhd', '*.vhd', 'unit/**/*.vhd', 'sanity/*.vhd', 'lib/*.vhd']:
	p = join(root, pattern)
	for f in glob(p, recursive=True):
		lib.add_source_file(str(f))

lib.add_compile_option("ghdl.flags", ["-Wc,-g"])
#ui.add_compile_option('ghdl.flags', ['--ieee=synopsys'])

lib.add_compile_option("ghdl.flags", ["-fprofile-arcs", "-ftest-coverage"])
ui.set_sim_option("ghdl.elab_flags", ["-Wl,-lgcov", "-Wl,--coverage", "-Wl,-no-pie"])
ui.set_sim_option("ghdl.sim_flags", ["--ieee-asserts=disable-at-0"])
try:
    ui.main()
except SystemExit as exc:
    all_ok = exc.code == 0

#if all_ok:
#    subprocess.call(["lcov", "--capture", "--directory", ".", "--output-file",  "code_coverage.info"])
#    subprocess.call(["genhtml", "code_coverage.info", "--output-directory", "code_html"])
