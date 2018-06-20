import yaml
import logging
import logging.config
import click
import os
import re
import sys
from pathlib import Path
from os.path import abspath
from .log import MyLogRecord

d = Path(abspath(__file__)).parent


def setup_logging() -> None:
    with Path(d / 'logging.yaml').open('rt', encoding='utf-8') as f:
        cfg = yaml.load(f)
    logging.setLogRecordFactory(MyLogRecord)
    logging.config.dictConfig(cfg)
    global log
    log = logging.getLogger('fw')


setup_logging()

from . import vunit_ifc
from . import test_unit, test_sanity, test_feature
from vunit.ui import VUnit
from .test_common import add_common_sources, add_flags


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


@click.group(cls=AliasedGroup)
@click.option('--compile', is_flag=True) #, hidden=True
@click.pass_context
def cli(ctx, compile):
    ctx.obj = {'compile': compile}
    sys.argv[0] = abspath(sys.argv[0])
    pass


@cli.command()
def create():
    pass


@cli.command()
@click.argument('config', type=click.Path())
@click.argument('vunit_args', nargs=-1)
@click.pass_obj
def test(obj, config, vunit_args):
    """Run the tests. Configuration is passed in YAML config file.

    You mas pass arguments directly to VUnit by appending them at the command end.
    NOTE: Be sure to include delimiter "--" before vunit options!

    Examples:

    # run all tests from tests_fast.yml\n
    ./run test tests_fast.yml

    # list tests\n
    ./run test tests_fast.yml -- --list

    # configure tests from tests_fast.yml, but run just one\n
    ./run test tests_fast.yml lib.tb_feature.retr_limit

    # run all feature tests (configured in config file)\n
    ./run test tests_fast.yml 'lib.tb_feature.*'
    """

    base = d.parent
    build = base / 'build'

    config_file = base / config
    out_basename = os.path.splitext(config)[0]
    with config_file.open('rt', encoding='utf-8') as f:
        config = yaml.load(f)
    build.mkdir(exist_ok=True)
    os.chdir(str(build))

    run_unit = 'unit' in config
    run_feature = 'feature' in config
    run_sanity = 'sanity' in config

    ui = create_vunit(obj, vunit_args, out_basename)

    lib = ui.add_library("lib")
    add_common_sources(lib, ui)

    tests = []
    if run_unit:
        tests.append(test_unit.UnitTests(ui, lib, config['unit'], build, base))
    if run_sanity:
        tests.append(test_sanity.SanityTests(ui, lib, config['sanity'], build, base))
    if run_feature:
        tests.append(test_feature.FeatureTests(ui, lib, config['feature'], build, base))

    for t in tests:
        t.add_sources()
    add_flags(ui, lib, build)
    for t in tests:
        t.configure()

    # check for unknown tests
    all_benches = lib.get_test_benches('*')
    unknown_tests = [tb for tb in all_benches if not re.match('tb_.*?_unit_test|tb_sanity|tb_feature', tb.name)]
    if len(unknown_tests):
        log.warn('Unknown tests (defaults will be used): {}'.format(', '.join(tb.name for tb in unknown_tests)))

    vunit_run(ui, build, out_basename)


def create_vunit(obj, vunit_args, out_basename):
    # fill vunit arguments
    args = []
    # hack for vunit_compile TCL command
    if obj['compile']:
        args += ['--compile']
    args += ['--xunit-xml', '../{}.xml1'.format(out_basename)] + list(vunit_args)
    ui = VUnit.from_argv(args)
    return ui


def vunit_run(ui, build, out_basename):
    try:
        vunit_ifc.run(ui)
        res = None
    except SystemExit as e:
        res = e.code
    out = build / '../{}.xml1'.format(out_basename)
    if out.exists():
        with out.open('rt', encoding='utf-8') as f:
            c = f.read()
        ofile = build / '../{}.xml'.format(out_basename)
        with ofile.open('wt', encoding='utf-8') as f:
            print('<?xml version="1.0" encoding="utf-8"?>', file=f)
            print('<?xml-stylesheet href="xunit.xsl" type="text/xsl"?>', file=f)
            f.write(c)
        out.unlink()
    sys.exit(res)


"""
+ vunit configurations
+ pass modelsim gui file via ui.set_sim_option("modelsim.init_file.gui", ...)
+ include the standard library files in ui.set_sim_option("modelsim.init_files.after_load", [...])
+ set TCOMP global variable

- allow preprocessed calls to log()
- use some log from vunit?
- use random from unit?

+ use per-test default configurations (with set tcl files etc.), different sanity configurations
x pass encoded composite generics (sanity test)

+ use watchdog - pass the time in config: test_runner_watchdog(runner, 10 ms);

- bash completion for files & tests:
    - click._bashcompletion.get_choices -> extend the if to check if the given argument is an instance of XXX
      and implement completion method for that instance. Complete test names.

- feature tests
- sanity - optimize bus delay shift registers
"""
