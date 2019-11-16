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
from typing import List, Tuple
import copy
import re

__all__ = ['add_sources', 'add_common_sources',
           'dict_merge', 'vhdl_serialize', 'dump_sim_options',
           'TestsBase', 'get_seed', 'OptionsDict']

d = Path(abspath(__file__)).parent
log = logging.getLogger(__name__)

jinja_env = Environment(loader=PackageLoader(__package__, 'data'), autoescape=False)


class OptionsDict(dict):
    # def __getattr__(self, key):
    #     return self[key]

    def __iadd__(self, upper: dict):
        self.__merge(self, upper)
        return self

    def __add__(self, upper: dict) -> 'OptionsDict':
        res = copy.deepcopy(self)
        res += upper
        return res

    def __radd__(self, lower: dict) -> 'OptionsDict':
        res = copy.deepcopy(lower)
        res += self
        return res

    @classmethod
    def __merge(cls, lower, upper) -> None:
        if isinstance(lower, OptionsDict):
            if not isinstance(upper, OptionsDict):
                raise TypeError('Cannot merge {} and {}'.format(type(lower), type('upper')))

            for k, v in upper.items():
                if k not in lower:
                    lower[k] = v
                else:
                    cls.__merge(lower[k], v)
        elif isinstance(lower, list):
            if not isinstance(upper, list):
                raise TypeError('Cannot merge {} and {}'.format(type(lower), type('upper')))
            lower.extend(upper)
        else:
            raise TypeError('Cannot merge {} and {}'.format(type(lower), type('upper')))


class TestsBase:
    def __init__(self, ui, lib, config, build, base, create_ghws: bool,
                 force_unrestricted_dump_signals: bool):
        self.ui = ui
        self.lib = lib
        self.config = config
        self.build = build
        self.base = base
        self.create_ghws = create_ghws
        self.force_unrestricted_dump_signals = force_unrestricted_dump_signals

    @property
    def jinja_env(self):
        return jinja_env

    def add_sources(self) -> None:
        raise NotImplementedError()

    def configure(self) -> bool:
        """Configure the tests.

        Return False if there were unconfigured tests found."""

        raise NotImplementedError()

    def generate_init_tcl(self, fname: str, tcomp: str) -> OptionsDict:
        tcl = self.build / fname
        with tcl.open('wt', encoding='utf-8') as f:
            print(dedent('''\
                global TCOMP
                set TCOMP {}
                '''.format(tcomp)), file=f)
        return OptionsDict({"modelsim.init_files.after_load": [str(tcl)]})

    def add_modelsim_gui_file(self, tb, cfg, name, tcl_init_files: List[str]) -> OptionsDict:
        """Return sim_options to add to the testcase."""
        sim_options = OptionsDict({'ghdl.sim_flags': []})
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

        sim_options["modelsim.init_file.gui"] = str(tcl)
        if 'gtkw' in cfg:
            gtkw = self.base / cfg['gtkw']
            if not gtkw.exists():
                log.warn('GTKW wave file {} not found'.format(cfg['gtkw']))
        else:
            tclfname = tcl.relative_to(self.base)
            base = str(tclfname.with_suffix("")).replace('/', '__')
            gtkw = self.build / (base+'.gtkw')
            ghw_file = self.build / (tb.name+'.elab.ghw')
            wave_opt_file = gtkw.with_suffix('.wevaopt.txt')
            # We need the GHW file for TCL -> GTKW conversion. If we are
            # generating them, there is no sense in actually doing
            # the conversion now.
            if self.create_ghws:
                log.info('Will generate {}'.format(ghw_file))
                sim_options["ghdl.sim_flags"] += ['--wave=' + str(ghw_file)]
            else:
                if not ghw_file.exists():
                    log.warning("Cannot convert wave file {} to gtkw, because"
                                " GHW file is missing. Run test with "
                                "--create-ghws.".format(tclfname))
                    gtkw = None
                else:
                    log.debug('Converting wave file {} to gtkw ...'.format(tclfname))
                    used_signals = tcl2gtkw(str(tcl), tcl_init_files, str(gtkw), ghw_file)
                    with wave_opt_file.open('wt') as f:
                        f.write('$ version 1.1\n')
                        f.writelines('\n'.join(used_signals))
                    if not cfg['dump_all_signals'] and not self.force_unrestricted_dump_signals:
                        log.info('Only signals included in the layout file '
                                 'will be dumped. To see them all, run with '
                                 '--dumpall.')
                        sim_options['ghdl.sim_flags'] += ['--read-wave-opt='+str(wave_opt_file)]
        if gtkw:
            try:
                tb.set_sim_option("ghdl.gtkwave_flags", [])
                sim_options["ghdl.gtkwave_flags"] = ['--save='+str(gtkw)]
            except ValueError:
                try:
                    tb.set_sim_option("ghdl.gtkw_file", "")
                    sim_options["ghdl.gtkw_file"] = str(gtkw)
                except ValueError:
                    log.warning('Setting GTKW file per test is not supported in this VUnit version.')
        return OptionsDict(sim_options)

    def get_default_sim_options(self) -> OptionsDict:
        c, s = get_default_compile_and_sim_options()
        return s

    def add_psl_cov(self, name) -> OptionsDict:
        name = re.sub(r'[^a-zA-Z0-9_-]', '_', name)
        psl_path = self.build / "functional_coverage" / "coverage_data" \
                    / "psl_cov_{}.json".format(name)
        sim_flags = ["--psl-report={}".format(psl_path)]
        return OptionsDict({"ghdl.sim_flags": sim_flags})

    @staticmethod
    def set_sim_options(tb, options: OptionsDict) -> None:
        for k, v in options.items():
            tb.set_sim_option(k, v)


def add_sources(lib, patterns) -> None:
    for pattern in patterns:
        p = join(str(d.parent), pattern)
        log.debug('Adding sources matching {}'.format(p))
        for f in glob(p, recursive=True):
            if f != "tb_wrappers.vhd":
                lib.add_source_file(str(f))


def add_common_sources(lib, ui) -> None:
    add_sources(lib, ['../src/**/*.vhd'])
    ui.enable_check_preprocessing()
    ui.enable_location_preprocessing() #(additional_subprograms=['log'])
    add_sources(lib, ['*.vhd', 'lib/*.vhd', 'models/*.vhd'])


def get_default_compile_and_sim_options() -> Tuple[OptionsDict, OptionsDict]:
    # TODO: move to config
    debug = True
    coverage = True
    psl = True

    compile_flags = []  # type: List[str]
    elab_flags = ["-Wl,-no-pie"]

    if debug:
        compile_flags += ['-g']
        elab_flags += ['-g']
    if coverage:
        compile_flags += ["-fprofile-arcs", "-ftest-coverage"]
        elab_flags += ["-Wl,-lgcov", "-Wl,--coverage"]
    if psl:
        compile_flags += ['-fpsl']
        elab_flags += ['-fpsl']

    compile_options = OptionsDict()
    compile_options["ghdl.flags"] = compile_flags

    cmif = ['../lib/test_lib.tcl', 'modelsim_init.tcl']
    common_modelsim_init_files = [str(d/x) for x in cmif]
    sim_options = OptionsDict({
        "ghdl.elab_flags": elab_flags,
        "modelsim.init_files.after_load": common_modelsim_init_files,
        "ghdl.sim_flags": ["--ieee-asserts=disable-at-0"],
    })
    return compile_options, sim_options


def get_compile_options() -> OptionsDict:
    c, s = get_default_compile_and_sim_options()
    return c


def get_seed(cfg) -> int:
    if 'seed' in cfg and 'randomize' in cfg:
        log.warning('Both "seed" and "randomize" are set - seed takes precedence')
    if 'seed' in cfg:
        seed = int(str(cfg['seed']), 0)
    elif cfg.get('randomize', False):
        # only 31 bits
        seed = int(random.random() * 2**31) & 0x7FFFFFFF
    else:
        seed = 0
    return seed


def dict_merge(up, *lowers) -> None:
    for lower in lowers:
        for k, v in lower.items():
            if k not in up:
                up[k] = v


def vhdl_serialize(o) -> str:
    if isinstance(o, Iterable):
        ss = []
        for x in o:
            ss.append(vhdl_serialize(x))
        return ''.join(['(', ', '.join(ss), ')'])
    else:
        return str(o)


def dump_sim_options(lib) -> None:
    for tb in lib.get_test_benches('*'):
        for cfgs in tb._test_bench.get_configuration_dicts():
            for name, cfg in cfgs.items():
                print('{}#{}:'.format(tb.name, name))
                #pprint(cfg.__dict__)
                pprint(cfg.sim_options)
