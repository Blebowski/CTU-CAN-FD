from textwrap import dedent
from collections.abc import Iterable
from glob import glob
from os.path import join, abspath
import logging
from pathlib import Path
from jinja2 import Environment, PackageLoader
from pprint import pprint
import random
from typing import List, Tuple
import copy
import re

__all__ = ['add_sources', 'add_rtl_sources', 'add_tb_sources',
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


def add_rtl_sources(lib) -> None:
    add_sources(lib, ['../src/**/*.vhd'])


def add_tb_sources(lib) -> None:
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
        "ghdl.sim_flags": ["--ieee-asserts=disable"],
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
