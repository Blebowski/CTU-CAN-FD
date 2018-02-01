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

signal.signal(signal.SIGTERM, sighandler)
signal.signal(signal.SIGINT, sighandler)
# ------------------------------------------------------------------------------

root = dirname(__file__)

ui = VUnit.from_argv()
lib = ui.add_library("lib")
for pattern in ['../src/**/*.vhd', '**/*/*.vhd', '*.vhd']:
	p = join(root, pattern)
	for f in glob(p, recursive=True):
		lib.add_source_file(str(f))

fil = lib.get_source_file('tb_example.vhd')
for f in glob('**/*/*.vhd', recursive=True):
	fil.add_dependency_on(lib.get_source_file(f))

ui.add_compile_option('ghdl.flags', ['--ieee=synopsys'])
#pprint([x.name for x in ui.get_source_files()])
ui.main()
