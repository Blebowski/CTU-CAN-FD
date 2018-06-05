from textwrap import dedent
import re
import logging
from .test_common import *

log = logging.getLogger(__name__)

class UnitTests(TestsBase):
    def add_sources(self):
        add_sources(self.lib, ['unit/**/*.vhd'])
        create_wrapper(self.lib, self.build / "tb_wrappers.vhd")

    def configure(self):
        ui, lib, config, build = self.ui, self.lib, self.config, self.build
        default = config['default']
        unit_tests = lib.get_test_benches('*_unit_test')
        for name, cfg in config['tests'].items():
            dict_merge(cfg, default)
            tb = lib.get_test_benches('*tb_{}_unit_test'.format(name), allow_empty=True)
            if not len(tb):
                pprint([x.name for x in unit_tests])
                raise RuntimeError('Testbench {}_unit_test does not exist (but specified in config).'.format(name))
            assert len(tb) == 1
            tb = tb[0]
            tb.set_generic('timeout', cfg['timeout'])
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
            self.add_modelsim_gui_file(tb, cfg, name)
            self._check_for_unconfigured()

    def _check_for_unconfigured(self):
        lib, config = self.lib, self.config
        # check for unconfigured unit tests
        unit_tests = lib.get_test_benches('*tb_*_unit_test')
        configured = ['tb_{}_unit_test'.format(name) for name in config['tests'].keys()]
        log.debug('Configured unit tests: {}'.format(', '.join(configured)))
        unconfigured = [tb for tb in unit_tests if tb.name not in configured]
        if len(unconfigured):
            log.warn("Unit tests with no configuration found (defaults will be used): {}".format(', '.join(tb.name for tb in unconfigured)))

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
                error_tol  : natural := 0;
                timeout    : string := "0 ms"
            ); end entity;
            architecture tb of tb_{test} is
                component vunittb_wrapper is generic (
                    nested_runner_cfg : string;
                    iterations : natural;
                    log_level  : log_lvl_type;
                    error_beh  : err_beh_type;
                    error_tol  : natural;
                    timeout    : string
                ); end component;
                for all:vunittb_wrapper use configuration work.tbconf_{test};
            begin
                tb:vunittb_wrapper generic map(
                    nested_runner_cfg => runner_cfg,
                    iterations        => iterations,
                    log_level         => log_level,
                    error_beh         => error_beh,
                    error_tol         => error_tol,
                    timeout           => timeout);
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
