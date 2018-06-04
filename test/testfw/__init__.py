import yaml
from os.path import join, dirname, abspath
import logging
import logging.config
from pathlib import Path
import click
from glob import glob
import os
import re
import sys
from textwrap import dedent
from pprint import pprint
from .log import MyLogRecord
from . import vunit_ifc
from vunit.ui import VUnit

d = Path(abspath(__file__)).parent

def setup_logging() -> None:
    with Path(d / 'logging.yaml').open('rt', encoding='utf-8') as f:
        cfg = yaml.load(f)
    logging.setLogRecordFactory(MyLogRecord)
    logging.config.dictConfig(cfg)
    global log
    log = logging.getLogger('fw')
#-------------------------------------------------------------------------------

class AttrDict:
    def __init__(self, data):
        self.data = data
    def __getitem__(self, key):
        return getattr(self.data, key)


class AliasedGroup(click.Group):
    def get_command(self, ctx, cmd_name):
        rv = super().get_command(ctx, cmd_name)
        if rv is not None:
            return rv
        matches = [x for x in self.list_commands(ctx)
                   if x.startswith(cmd_name)]
        if not matches:
            return None
        elif len(matches) == 1:
            return click.Group.get_command(self, ctx, matches[0])
        ctx.fail('Too many matches: %s' % ', '.join(sorted(matches)))
#-------------------------------------------------------------------------------


def add_sources(lib, patterns):
    for pattern in patterns:
        p = join(str(d.parent), pattern)
        log.debug('Adding sources matching {}'.format(p))
        for f in glob(p, recursive=True):
            if f != "tb_wrappers.vhd":
                lib.add_source_file(str(f))

def add_common_sources(lib):
    return add_sources(lib, ['../src/**/*.vhd', '*.vhd', 'lib/*.vhd'])

def get_common_modelsim_init_files():
    modelsim_init_files = '../lib/test_lib.tcl,modelsim_init.tcl'
    modelsim_init_files = [str(d/x) for x in modelsim_init_files.split(',')]
    return modelsim_init_files

def add_flags(ui, lib, build):
    unit_tests = lib.get_test_benches('*_unit_test')
    for ut in unit_tests:
        ut.scan_tests_from_file(str(build / "../unit/vunittb_wrapper.vhd"))

    #lib.add_compile_option("ghdl.flags", ["-Wc,-g"])
    lib.add_compile_option("ghdl.flags", ["-fprofile-arcs", "-ftest-coverage"])
    ui.set_sim_option("ghdl.elab_flags", ["-Wl,-lgcov", "-Wl,--coverage", "-Wl,-no-pie"])
    ui.set_sim_option("ghdl.sim_flags", ["--ieee-asserts=disable-at-0"])
    modelsim_init_files = get_common_modelsim_init_files()
    ui.set_sim_option("modelsim.init_files.after_load", modelsim_init_files)

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


@click.group(cls=AliasedGroup)
@click.option('--compile', is_flag=True) #, hidden=True
@click.pass_context
def cli(ctx, compile):
    setup_logging()
    ctx.obj = {'compile': compile}
    pass

@cli.command()
def create():
    pass

@cli.group(cls=AliasedGroup)
def test():
    sys.argv[0] = abspath(sys.argv[0])
    pass

@test.command()
@click.argument('config', type=click.Path())
@click.argument('vunit_args', nargs=-1)
@click.pass_obj
def unit(obj, config, vunit_args):
    config_file = d.parent / config
    with config_file.open('rt', encoding='utf-8') as f:
        config = yaml.load(f)
    base = Path(config_file).resolve().parent
    build = d.parent / 'build'
    build.mkdir(exist_ok=True)
    os.chdir(str(build))

    args = []
    if obj['compile']:
        args += ['--compile']
    args += ['--xunit-xml', '../test_unit.xml1'] + list(vunit_args)
    ui = VUnit.from_argv(args)

    lib = ui.add_library("lib")
    add_common_sources(lib)
    add_sources(lib, ['unit/**/*.vhd'])
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
            raise RuntimeError('Testbench {}_unit_test does not exist (but specified in config).'.format(name))
        assert len(tb) == 1
        tb = tb[0]
        tb.set_generic('iterations', cfg['iterations'])
        tb.set_generic('log_level', cfg['log_level'] + '_l')
        tb.set_generic('error_tol', cfg['error_tolerance'])

        # generate & set per-test modelsim tcl file
        tcl = build / 'modelsim_init_{}.tcl'.format(name)
        with tcl.open('wt', encoding='utf-8') as f:
            print(dedent('''\
                global TCOMP
                set TCOMP tb_{}_unit_test/tb/i_test
                '''.format(name)), file=f)
        init_files = get_common_modelsim_init_files()
        init_files += [str(tcl)]
        tb.set_sim_option("modelsim.init_files.after_load", init_files)

        if 'wave' in cfg:
            path = base / cfg['wave']
            if not path.exists():
                log.warn('Wave file {} not found'.format(cfg['wave']))
            tb.set_sim_option("modelsim.init_file.gui", str(path))
        else:
            tcl = build / 'modelsim_gui_{}.tcl'.format(name)
            with tcl.open('wt', encoding='utf-8') as f:
                print(dedent('''\
                    start_CAN_simulation "dummy"
                    global IgnoreAddWaveErrors
                    puts "Automatically adding common waves. Failures are handled gracefully."
                    set IgnoreAddWaveErrors 1
                    add_test_status_waves
                    add_system_waves
                    set IgnoreAddWaveErrors 0
                    run_simulation
                    get_test_results
                    '''.format(name)), file=f)
            tb.set_sim_option("modelsim.init_file.gui", str(tcl))

    # check for unconfigured testbenches
    all_benches = lib.get_test_benches('*')
    configured = ['tb_{}_unit_test'.format(name) for name in config['tests'].keys()]
    log.debug('Configured tests: {}'.format(', '.join(configured)))
    unconfigured = [tb for tb in all_benches if tb.name not in configured]
    if len(unconfigured):
        log.warn("Testbenches with no configuration found: {}".format(', '.join(x.name for x in unconfigured)))

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

def dump_sim_options(lib):
    for tb in lib.get_test_benches('*'):
        for cfgs in tb._test_bench.get_configuration_dicts():
            for name, cfg in cfgs.items():
                print('{}#{}:'.format(tb.name, name))
                #pprint(cfg.__dict__)
                pprint(cfg.sim_options)
