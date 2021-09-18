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
func_cov_dir = d.parent / "build/functional_coverage"


def setup_logging() -> None:
    with Path(d / 'logging.yaml').open('rt', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    logging.setLogRecordFactory(MyLogRecord)
    logging.config.dictConfig(cfg)
    global log
    log = logging.getLogger('fw')


setup_logging()

from . import vunit_ifc
from vunit.ui import VUnit
from .test_common import add_rtl_sources, add_post_syn_netlist, unit_configure, add_unit_sources, add_main_tb_sources, main_tb_configure, get_compile_options, dict_merge


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
def test(obj, *, config, vunit_args):
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
        config = yaml.safe_load(f)
    build.mkdir(exist_ok=True)
    os.chdir(str(build))

    ui = create_vunit(obj, vunit_args, out_basename)

    ctu_can_fd_rtl = ui.add_library("ctu_can_fd_rtl")
    ctu_can_fd_tb = ui.add_library("ctu_can_fd_tb")
    ctu_can_fd_tb_unit = ui.add_library("ctu_can_fd_tb_unit")

    add_rtl_sources(ctu_can_fd_rtl)
    
    # Compile gate level netlist to stand-alone library to avoid name conflict
    # with RTL version. If running gate, only DUT is gate level TEST_NODE in
    # feature tests is still on RTL, so we need whole RTL too!!
    if config["_default"]["gate_level"]:
        # We need separate library to compile in synthesized can_top_level
        # to avoid name clash and different interface!!
        ctu_can_fd_gates = ui.add_library("ctu_can_fd_gates")
        add_post_syn_netlist(ctu_can_fd_gates)

    ui.enable_check_preprocessing()
    #ui.enable_location_preprocessing()  # (additional_subprograms=['log'])

    tests = []
    
    ###########################################################################
    # Main TB
    ###########################################################################
    if ("compliance" in config or "feature" in config or "reference" in config):
        add_main_tb_sources(ctu_can_fd_tb, config)

        # Test-bench object is automatically detected on "tb_top" due to "runner_cfg"
        tb = ctu_can_fd_tb.get_test_benches()[0]
        main_tb_configure(tb, config, build)

    ###########################################################################
    # Unit tests
    ###########################################################################
    if ("unit" in config):
        add_unit_sources(ctu_can_fd_tb_unit, build)
        unit_configure(ctu_can_fd_tb_unit, config, build)


    (func_cov_dir / "html").mkdir(parents=True, exist_ok=True)
    (func_cov_dir / "coverage_data").mkdir(parents=True, exist_ok=True)

    # Global compile options
    c = get_compile_options(config['_default'])
    for k, v in c.items():
        ctu_can_fd_tb.set_compile_option(k, v)
        ctu_can_fd_rtl.set_compile_option(k, v)
        if config["_default"]["gate_level"]:
            ctu_can_fd_gates.set_compile_option(k, v)

    res = vunit_run(ui, build, out_basename)

    # Move code coverage results to stand-alone directory to avoid overwriting it by runs
    # of other configs
    if config['_default']['code_coverage']:
        code_coverage = build / 'code_coverage_{}'.format(out_basename)
        code_coverage.mkdir(exist_ok=True)
        os.system("mv *.gcda {}".format(code_coverage))
        os.system("mv *.gcno {}".format(code_coverage))

    ################################################################################################
    # Dump source file lists for RTL and main TB
    ################################################################################################
    rtl_sources = ui.get_source_files(library_name="ctu_can_fd_rtl")
    tb_sources = ui.get_source_files(library_name="ctu_can_fd_tb")
    rtl_sources_ordered = ui.get_compile_order(rtl_sources)
    tb_sources_ordered = ui.get_compile_order(tb_sources)

    rtl_lf = open(base / "rtl_lst.txt", 'w')
    tb_lf = open(base / "tb_lst.txt", 'w')

    # Correct list files to match file layout in export package
    rtl_names = []
    for item in rtl_sources_ordered:
        file_name = "rtl/{}".format(item.name.split("/")[-1])
        rtl_names.append(file_name)

    tb_names = []
    for item in tb_sources_ordered:
        # Skip Vunit internals and files from RTL added also to TB lib
        if item.name.startswith("../.."):
            continue

        file_name = "tb/{}".format(item.name.split("/")[-1])
        tb_names.append(file_name)

    # Dump to files
    for item in rtl_names:
        rtl_lf.write(item)
        rtl_lf.write("\n")

    for item in tb_names:     
        tb_lf.write(item)
        tb_lf.write("\n")

    sys.exit(res)


def create_vunit(obj, vunit_args, out_basename):
    # fill vunit arguments
    args = []
    # hack for vunit_compile TCL command
    if obj['compile']:
        args += ['--compile']
    args += ['--xunit-xml', '../{}.xml1'.format(out_basename)] + list(vunit_args)
    ui = VUnit.from_argv(args)
    return ui


def vunit_run(ui, build, out_basename) -> int:
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
    return res
