import re
import logging
from textwrap import dedent
from .test_common import add_sources, dict_merge, TestsBase, \
                         get_common_modelsim_init_files, get_seed
from pprint import pprint

log = logging.getLogger(__name__)


class UnitTests(TestsBase):
    def add_sources(self):
        add_sources(self.lib, ['unit/**/*.vhd'])
        self._create_wrapper(self.build / "tb_wrappers.vhd")

    def configure(self):
        lib, config, build = self.lib, self.config, self.build
        default = config['default']
        unit_tests = lib.get_test_benches('*_unit_test')
        for name, cfg in config['tests'].items():
            dict_merge(cfg, default)
            tb = lib.get_test_benches('*tb_{}_unit_test'.format(name),
                                      allow_empty=True)
            if not len(tb):
                pprint([x.name for x in unit_tests])
                raise RuntimeError('Testbench {}_unit_test does not exist'
                                   + ' (but specified in config).'.format(name))
            assert len(tb) == 1
            tb = tb[0]
            tb.set_generic('timeout', cfg['timeout'])
            tb.set_generic('iterations', cfg['iterations'])
            tb.set_generic('log_level', cfg['log_level'] + '_l')
            tb.set_generic('error_tol', cfg['error_tolerance'])
            tb.set_generic('seed', get_seed(cfg))

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

    def _create_wrapper(self, fname):
        fname = str(fname)
        files = self.lib.get_source_files()
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
        c = self.jinja_env.get_template('unit_wrappers.vhd').render(tests=tests)
        with open(fname, "wt", encoding='utf-8') as f:
            f.write(c)

        self.lib.add_source_file(fname)
