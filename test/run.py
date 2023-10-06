#!/usr/bin/env python3
# run sanity test using vunit
# Requires: vunit
# $ python run.py # select automatically
# $ # select manually
# $ VUNIT_SIMULATOR=modelsim PATH=$PATH:$MODELSIM_BIN python run.py
# $ VUNIT_SIMULATOR=ghdl python run.py

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
    opts = SIM_CFG["comp_options"]["ghdl"].split()
    if "comp_options" in file:
        opts.extend(file["comp_options"].split())
    # TODO: Add target specific elabo options
    #print(f"Compile options: {opts}")
    sf.add_compile_option("ghdl.flags", opts)


def set_elab_options(vu, tgt):
    opts = SIM_CFG["elab_options"]["ghdl"].split()
    if "elab_options" in tgt:
        if "ghdl" in tgt["elab_options"]:
            tmp = tgt["elab_options"]["ghdl"].split(" ")
            opts.extend(tmp)
    # TODO: Add test specific elab options
    #print(f"Adding elab flags: {opts}")
    vu.set_sim_option("ghdl.elab_flags", opts)



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

        # Target generics
        generics.update(tgt["generics"])

        # Test specific generics
        if "generics" in test:
            generics.update(test["generics"])

        # Sim options
        opts = SIM_CFG["sim_options"]["ghdl"].split()
        if "sim_options" in tgt:
            if "ghdl" in tgt["sim_options"]:
                tmp = tgt["sim_options"]["ghdl"].split(" ")
                opts.extend(tmp)

        # TODO: Add test specific sim options

        # Add sim options for PSL functional coverage
        name = re.sub(r'[^a-zA-Z0-9_-]', '_', test["name"])
        os.system("mkdir -p vunit_out/functional_coverage/coverage_data")
        psl_path = "vunit_out/functional_coverage/coverage_data/psl_cov_{}_{}.json".format(tgt_name, name)
        opts.extend(["--psl-report={}".format(psl_path)])

        # Create the test
        tb.add_config(test["name"], generics=generics, sim_options={"ghdl.sim_flags": opts})


if __name__ == '__main__':
    load_sim_cfg()

    if (len(sys.argv) < 2):
        print("./run.py should have at least one arguments (target from sim/ts_sim_config.yml)!")
        sys.exit(1)

    # First argument is always target -> Drop it, rest is for VUnit
    tgt_name = sys.argv[1]
    sys.argv.remove(sys.argv[1])

    if (tgt_name not in SIM_CFG["targets"]):
        print(f"Target {tgt_name} does not exist in ts_sim_config.yml!")
        print(f"""Available targets are:{SIM_CFG["targets"].keys()}""")
        sys.exit(1)

    tgt = SIM_CFG["targets"][tgt_name]


    ###########################################################################
    # Invoke VUnit
    ###########################################################################
    vu = VUnit.from_argv()

    load_tgt_slfs(vu, tgt)
    load_tgt_tlf(vu, tgt, tgt_name)

    set_elab_options(vu, tgt)

    # Run
    vu.main()
