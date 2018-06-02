import click
import yaml
from glob import glob
from os.path import join, dirname, abspath
from vunit.ui import VUnit
import os
import re
from textwrap import dedent
from pathlib import Path
from pprint import pprint
from . import vunit_ifc

d = Path(abspath(__file__)).parent


def add_sources(ui):
    lib = ui.add_library("lib")
    for pattern in ['../src/**/*.vhd', '*.vhd', 'unit/**/*.vhd', 'sanity/*.vhd', 'lib/*.vhd']:
        p = join(str(d.parent), pattern)
        print(p)
        for f in glob(p, recursive=True):
            if f != "tb_wrappers.vhd":
                lib.add_source_file(str(f))
    return lib

def add_flags(ui, lib, build):
    unit_tests = lib.get_test_benches('*_unit_test')
    for ut in unit_tests:
        ut.scan_tests_from_file(str(build / "../unit/vunittb_wrapper.vhd"))

    #lib.add_compile_option("ghdl.flags", ["-Wc,-g"])
    lib.add_compile_option("ghdl.flags", ["-fprofile-arcs", "-ftest-coverage"])
    ui.set_sim_option("ghdl.elab_flags", ["-Wl,-lgcov", "-Wl,--coverage", "-Wl,-no-pie"])
    ui.set_sim_option("ghdl.sim_flags", ["--ieee-asserts=disable-at-0"])
    ui.set_sim_option("modelsim.init_files.after_load", [str(d / 'modelsim_init.tcl')])

def create_wrapper(lib, fname):
    fname = str(fname)
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
            use work.CANtestLib.All;
            library vunit_lib;
            context vunit_lib.vunit_context;

            entity tb_{test} is generic (
                runner_cfg : string := runner_cfg_default;
                iterations : natural := 1;
                log_level  : log_lvl_type := info_l;
                error_beh  : err_beh_type := quit;
                error_tol  : natural := 0
            ); end entity;
            architecture tb of tb_{test} is
                component vunittb_wrapper is generic (
                    nested_runner_cfg : string;
                    iterations : natural;
                    log_level  : log_lvl_type;
                    error_beh  : err_beh_type;
                    error_tol  : natural
                ); end component;
                for all:vunittb_wrapper use configuration work.tbconf_{test};
            begin
                tb:vunittb_wrapper generic map(nested_runner_cfg => runner_cfg, iterations => iterations, log_level => log_level, error_beh => error_beh, error_tol => error_tol);
            end architecture;
            -- -----------------------------------------------------------------------------
            """.format(test=test)
        ))
    with open(fname, "wt", encoding='utf-8') as f:
        for c in configs:
            f.write(c)
        for t in tbs:
            f.write(t)

    lib.add_source_file(fname)


@click.group()
def cli():
    pass

@cli.command()
def create():
    pass

@cli.group()
def test():
    pass

@test.command()
@click.argument('config', type=click.Path(exists=True))
@click.argument('vunit_args', nargs=-1)
def unit(config, vunit_args):
    config_file = config
    with open(config_file, 'rt', encoding='utf-8') as f:
        config = yaml.load(f)
    base = Path(config_file).resolve().parent
    build = d.parent / 'build'
    build.mkdir(exist_ok=True)

    ui = VUnit.from_argv(['lib.tb_*_unit_test.*', '--xunit-xml', '../test_unit.xml1'] + list(vunit_args))
    lib = add_sources(ui)
    create_wrapper(lib, build / "tb_wrappers.vhd")
    add_flags(ui, lib, build)

    default = config['default']
    unit_tests = lib.get_test_benches('*_unit_test')
    for name, _cfg in config['tests'].items():
        cfg = default.copy()
        if _cfg:
            cfg.update(_cfg)
        tb = lib.get_test_benches('*tb_{}_unit_test'.format(name), allow_empty=True)
        if not len(tb):
            pprint([x.name for x in unit_tests])
            raise RuntimeError('Testbench {}_unit_test does not exist (buf specified in config).'.format(name))
        assert len(tb) == 1
        tb = tb[0]
        tb.set_generic('iterations', cfg['iterations'])
        tb.set_generic('log_level', cfg['log_level'] + '_l')
        tb.set_generic('error_tol', cfg['error_tolerance'])
        if 'wave' in cfg:
            tb.set_sim_option("modelsim.init_file.gui", str(base / cfg['wave']))

    os.chdir(str(build))

    try:
        vunit_ifc.run(ui)
    except SystemExit:
        pass
    out = build / '../test_unit.xml1'
    if out.exists():
        with out.open('rt', encoding='utf-8') as f:
            c = f.read()
        with open('../test_unit.xml', 'wt', encoding='utf-8') as f:
            print('<?xml version="1.0" encoding="utf-8"?>', file=f)
            print('<?xml-stylesheet href="xunit.xsl" type="text/xsl"?>', file=f)
            f.write(c)
        out.unlink()


"""
- vunit configurations
- pass modelsim gui file via ui.set_sim_option("modelsim.init_file.gui", ...)
- include the standard library files in ui.set_sim_option("modelsim.init_files.after_load", [...])
- set TCOMP global variable

- allow preprocessed calls to log()
- use some log from vunit?
- use random from unit?

- use per-test default configurations (with set tcl files etc.), different sanity configurations
- pass encoded composite generics (sanity test)

- use watchdog - pass the time in config: test_runner_watchdog(runner, 10 ms);
"""
