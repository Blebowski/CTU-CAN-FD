# run sanity test using vunit
# Requires: vunit
# $ python run.py # select automatically
# $ # select manually
# $ VUNIT_SIMULATOR=modelsim PATH=$PATH:$MODELSIM_BIN python run.py
# $ VUNIT_SIMULATOR=ghdl python run.py

from os.path import join, dirname
from vunit.ui import VUnit
from glob import glob
from pprint import pprint
import os
import signal
import subprocess
import re
from textwrap import dedent

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

def create_wrapper(lib):
    files = lib.get_source_files()
    tests = []
    r = re.compile(r'^architecture\s+(\S+)\s+of\s+CAN_test\s+is$')
    for file in files:
        with open(file.name, 'rt', encoding='utf-8') as f:
            for l in f:
                m = r.match(l)
                if m:
                    tests.append(m.group(1))
    configs = []
    tbs = []
    for test in tests:
        configs.append(dedent("""\
            configuration tbconf_{test} of vunittb_wrapper is
            for tb
                for i_test : CAN_test use entity work.CAN_test({test}); end for;
            end for;
            end configuration;
            -- -----------------------------------------------------------------------------
            """.format(test=test)
        ))
        tbs.append(dedent("""\
            library work;
            USE work.CANtestLib.All;

            entity tb_{test} is generic (runner_cfg : string); end entity;
            architecture tb of tb_{test} is
                component vunittb_wrapper is generic (xrunner_cfg : string); end component;
                for all:vunittb_wrapper use configuration work.tbconf_{test};
            begin
                tb:vunittb_wrapper generic map(xrunner_cfg => runner_cfg);
            end architecture;
            -- -----------------------------------------------------------------------------
            """.format(test=test)
        ))
    with open("tb_wrappers.vhd", "wt", encoding='utf-8') as f:
        for c in configs:
            f.write(c)
        for t in tbs:
            f.write(t)

    lib.add_source_file("tb_wrappers.vhd")

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
        if f != "tb_wrappers.vhd":
            lib.add_source_file(str(f))

create_wrapper(lib)

#lib.add_compile_option("ghdl.flags", ["-Wc,-g"])

lib.add_compile_option("ghdl.flags", ["-fprofile-arcs", "-ftest-coverage"])
ui.set_sim_option("ghdl.elab_flags", ["-Wl,-lgcov", "-Wl,--coverage", "-Wl,-no-pie"])
ui.set_sim_option("ghdl.sim_flags", ["--ieee-asserts=disable-at-0"])
ui.main()
