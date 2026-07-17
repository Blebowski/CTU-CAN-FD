#!/usr/bin/env python3
# run sanity test using vunit
# Requires: vunit
# $ python run.py # select automatically
# $ # select manually
# $ VUNIT_SIMULATOR=modelsim PATH=$PATH:$MODELSIM_BIN python run.py
# $ VUNIT_SIMULATOR=nvc python run.py

import yaml
import sys
import os
import copy
import random
import re

from vunit import VUnit


# Global paths
SIM_CFG_PATH = "../sim/ts_sim_config.yml"

# Global configs
SIM_CFG = None


def p_repo_root(path):
    return os.path.join("..", path)


def p_file(file, path):
    #abs_path = os.path.abspath(file)
    return os.path.join(os.path.dirname(file), path)

def get_seed():
    if "seed" in SIM_CFG:
        return SIM_CFG["seed"]
    return random.randint(0, 2**31-1)

def load_sim_cfg():
    global SIM_CFG
    print(f"Loading simulation config file: {SIM_CFG_PATH}")
    with open(SIM_CFG_PATH) as f:
        SIM_CFG = yaml.safe_load(f)

def load_tgt_slfs(vu, tgt):
    for slf in tgt["source_list_files"]:
        load_slf(vu, "..", slf)

def set_comp_options(sf, file):
    # GHDL
    ghdl_comp_opts = ["-fpsl", "-frelaxed-rules", "--ieee=synopsys"]
    sf.add_compile_option("ghdl.a_flags", ghdl_comp_opts)

    # NVC
    nvc_glob_flags = []
    nvc_glob_flags.append('-M')
    nvc_glob_flags.append('256M')
    sf.add_compile_option("nvc.global_flags", nvc_glob_flags)

    sf.add_compile_option("nvc.a_flags", ['--psl'])

def set_glob_options(vu):
    # NVC
    nvc_glob_flags = []
    nvc_glob_flags.append('-M')
    nvc_glob_flags.append('512M')
    nvc_glob_flags.append('--load=main_tb/iso-16845-compliance-tests/build/Debug/src/cosimulation/libNVC_VHPI_COSIM_LIB.so')
    nvc_glob_flags.append("--ieee-warnings=off")
    nvc_glob_flags.append("--messages=compact")

    vu.set_sim_option("nvc.global_flags", nvc_glob_flags, allow_empty=True)

def load_slf(vu, curr_path, slf_path):
    full_path = os.path.join(curr_path, slf_path)
    full_path = os.path.expandvars(full_path)

    if (os.path.exists(full_path)):
        print(f"Loading source list file: {full_path}")
        with open(full_path) as f:
            slf = yaml.safe_load(f)
            lib = vu.add_library(slf["library"], allow_duplicate=True)

            for file in slf["source_list"]:
                #print(f"""Adding file: {p_file(full_path, file["file"])}""")
                sf = lib.add_source_file(p_file(full_path, file["file"]))
                set_comp_options(sf, file)

    else:
        full_slf = os.path.expandvars(slf_path)
        print(f"Loading SLF from dependant target: {full_slf}")
        load_tgt_slfs(vu, SIM_CFG["targets"][full_slf])

def load_tgt_tlf(vu, tgt, tgt_name):
    lib_name = tgt["top_entity"].split('.')[0]
    top_lib = vu.add_library(lib_name, allow_duplicate=True)
    tb = top_lib.get_test_benches()[0]
    tlf_path = os.path.join("..", tgt["test_list_file"])

    with open(tlf_path) as f:
        tlf = yaml.safe_load(f)

    for test in tlf["tests"]:
        generics = {}

        # Propagate seed
        generics["seed"] = get_seed()

        # Propagate test name
        generics[SIM_CFG["test_name_generic"]] = test["name"]

        # Append Target name to keep test names unique
        test_name = tgt_name + "." + test["name"]

        # Target generics
        generics.update(tgt["generics"])

        # Test specific generics
        if "generics" in test:
            generics.update(test["generics"])

        # Remove hierarchy prefixes
        filtered_generics = {}
        for key, value in generics.items():
            new_key = key.split("/")[-1]
            filtered_generics[new_key] = value

        sim_opts = {}

        #######################################################################
        # Set GHDL ELAB / SIM options
        #######################################################################
        sim_opts["ghdl.elab_flags"] = ["-Wl,-no-pie", "-fpsl", "-frelaxed-rules", "--ieee=synopsys"]
        sim_opts["ghdl.sim_flags"] = ["--ieee-asserts=disable"]

        #######################################################################
        # Set NVC ELAB / SIM options
        #######################################################################
        # Per-test cde coverage
        os.system("mkdir -p vunit_out/code_coverage")
        #test_name_normalized = re.sub(r'[^a-zA-Z0-9_-]', '_', test_name)
        covdb_path = "vunit_out/code_coverage/{}_{}.ncdb".format(tgt_name, test_name)

        nvc_elab_opts = []
        nvc_elab_opts.append("-V")

        if ("gate" not in tgt_name):
            nvc_elab_opts.append("--cover=all,include-mems,exclude-unreachable,count-from-undefined")
            nvc_elab_opts.append("--cover-file={}".format(covdb_path))
            nvc_elab_opts.append("--cover-spec=nvc_cover_spec")

        nvc_elab_opts.append("--no-collapse")
        nvc_elab_opts.append("--jit")

        sim_opts["nvc.elab_flags"] = nvc_elab_opts
        sim_opts["nvc.heap_size"] = '256m'
        sim_opts["nvc.sim_flags"] = ['--ieee-warnings=off', "--dump-arrays"]

        # Create the test
        tb.add_config(test_name, generics=filtered_generics, sim_options=sim_opts)

if __name__ == '__main__':
    load_sim_cfg()

    if (len(sys.argv) < 2):
        print("./run.py should have at least one arguments (target from sim/ts_sim_config.yml)!")
        sys.exit(1)

    # First argument is always target pattern -> Drop it, rest is for VUnit
    tgt_pattern = sys.argv[1]
    sys.argv.remove(sys.argv[1])

    vu = VUnit.from_argv()
    vu.add_vhdl_builtins()

    first = True

    for tgt_name,tgt in SIM_CFG["targets"].items():

        if (not re.match(tgt_pattern, tgt_name)):
            continue

        print(f"Target {tgt_name} matches pattern {tgt_pattern}")

        # Load source list files only from first target that matches!
        # Assumes targets have equal SLFs which is true for various
        if first:
            load_tgt_slfs(vu, tgt)
            first = not first

        load_tgt_tlf(vu, tgt, tgt_name)

    set_glob_options(vu)

    # Run
    vu.main()
