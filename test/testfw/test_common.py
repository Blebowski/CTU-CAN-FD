from textwrap import dedent
from collections.abc import Iterable
from glob import glob
from os.path import join, abspath
import logging
from pathlib import Path
from jinja2 import Environment, PackageLoader
from pprint import pprint
import random
from .gtkwave import tcl2gtkw
from typing import List

__all__ = ['add_sources', 'add_common_sources', 'get_common_modelsim_init_files',
           'add_flags', 'dict_merge', 'vhdl_serialize', 'dump_sim_options',
           'TestsBase', 'get_seed']

d = Path(abspath(__file__)).parent
log = logging.getLogger(__name__)

jinja_env = Environment(loader=PackageLoader(__package__, 'data'), autoescape=False)


class TestsBase:
    def __init__(self, ui, lib, config, build, base):
        self.ui = ui
        self.lib = lib
        self.config = config
        self.build = build
        self.base = base

    @property
    def jinja_env(self):
        return jinja_env

    def add_sources(self):
        raise NotImplementedError()

    def configure(self):
        raise NotImplementedError()

    def add_modelsim_gui_file(self, tb, cfg, name, tcl_init_files: List[str] = None):
        if tcl_init_files is None:
            tcl_init_files = tb.get_sim_option("modelsim.init_files.after_load")
        if 'wave' in cfg:
            tcl = self.base / cfg['wave']
            if not tcl.exists():
                log.warn('Wave file {} not found'.format(cfg['wave']))
        else:
            tcl = self.build / 'modelsim_gui_{}.tcl'.format(name)
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
        if 'gtkw' in cfg:
            gtkw = self.base / cfg['gtkw']
            if not gtkw.exists():
                log.warn('GTKW wave file {} not found'.format(cfg['gtkw']))
        else:
            gtkw = tcl.with_suffix('.gtkw')
            tclfname = tcl.relative_to(self.base)
            log.info('Converting wave file {} to gtkw ...'.format(tclfname))
            tcl2gtkw(str(tcl), tcl_init_files, str(gtkw))

        if gtkw:
            tb.set_sim_option("ghdl.gtkw_file", str(gtkw))


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
    unit_tests = lib.get_test_benches('*_unit_test', allow_empty=True)
    for ut in unit_tests:
        ut.scan_tests_from_file(str(build / "../unit/vunittb_wrapper.vhd"))

    reference_tests = lib.get_test_benches('*reference*', allow_empty=True)
    for rt in reference_tests:
        rt.scan_tests_from_file(str(build / "../reference/vunit_reference_wrapper.vhd"))

    #lib.add_compile_option("ghdl.flags", ["-Wc,-g"])
    lib.add_compile_option("ghdl.flags", ["-fprofile-arcs", "-ftest-coverage", "-fpsl"])
    ui.set_sim_option("ghdl.elab_flags", ["-Wl,-lgcov", "-Wl,--coverage", "-Wl,-no-pie", "-fpsl"])
    ui.set_sim_option("ghdl.sim_flags", ["--ieee-asserts=disable-at-0"])
    modelsim_init_files = get_common_modelsim_init_files()
    ui.set_sim_option("modelsim.init_files.after_load", modelsim_init_files)


def get_seed(cfg):
    if 'seed' in cfg and 'randomize' in cfg:
        log.warning('Both "seed" and "randomize" are set - seed takes precedence')
    if 'seed' in cfg:
        seed = cfg['seed']
    elif cfg.get('randomize', False):
        # only 31 bits
        seed = int(random.random() * 2**31) & 0x7FFFFFFF
    else:
        seed = 0
    return seed


def dict_merge(up, *lowers):
    for lower in lowers:
        for k, v in lower.items():
            if k not in up:
                up[k] = v


def vhdl_serialize(o):
    if isinstance(o, Iterable):
        ss = []
        for x in o:
            ss.append(vhdl_serialize(x))
        return ''.join(['(', ', '.join(ss), ')'])
    else:
        return str(o)


def dump_sim_options(lib):
    for tb in lib.get_test_benches('*'):
        for cfgs in tb._test_bench.get_configuration_dicts():
            for name, cfg in cfgs.items():
                print('{}#{}:'.format(tb.name, name))
                #pprint(cfg.__dict__)
                pprint(cfg.sim_options)
