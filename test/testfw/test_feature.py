import logging
from pathlib import Path
from .test_common import add_sources, TestsBase, dict_merge, \
                         get_seed, OptionsDict
from textwrap import dedent
import re

log = logging.getLogger(__name__)


class FeatureTests(TestsBase):
    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self._tests = self.config['tests'].keys()

    def add_sources(self) -> None:
        sources = ['feature/{}_feature_tb.vhd'.format(name) for name in self._tests]
        sources.append('feature/tb_feature.vhd')
        sources.append('feature/pkg_feature_exec_dispatch.vhd')
        add_sources(self.lib, sources)

        wrname = self.build / 'pkg_feature_exec_dispatch-body.vhd'
        self._create_wrapper(wrname)
        self.lib.add_source_file(str(wrname))
        tb = self.lib.get_test_benches('*tb_feature')[0]
        tb.scan_tests_from_file(str(wrname))

    def configure(self) -> bool:
        tb = self.lib.get_test_benches('*tb_feature')[0]
        default = self.config['default']

        sim_options = self.get_sim_options(default)

        for name, cfg in self.config['tests'].items():
            if cfg is None:
                cfg = dict()

            dict_merge(cfg, default)

            generics = {
                'timeout'      : cfg['timeout'],
                'iterations'   : cfg['iterations'],
                'log_level'    : cfg['log_level'] + '_l',
                'error_tol'    : cfg['error_tolerance'],
                'test_name'    : name,
                'seed'         : get_seed(cfg)
            }

            local_sim_options = OptionsDict()
            if cfg['functional_coverage']:
                local_sim_options += self.add_psl_cov('{}.{}'.format(tb.name, name))

            local_sim_options = sim_options + local_sim_options
            tb.add_config(name, generics=generics, sim_options=local_sim_options)

        return self._check_for_unconfigured()

    def _check_for_unconfigured(self) -> bool:
        config = self.config
        # check for unconfigured unit tests
        test_files = self.base.glob('feature/*_feature_tb.vhd')
        all_tests = [re.match('(.*?)_feature_tb.vhd', f.name).group(1) for f in test_files]
        configured = list(config['tests'].keys())
        log.debug('Configured feature tests: {}'.format(', '.join(configured)))
        unconfigured = [tb for tb in all_tests if tb not in configured]
        if len(unconfigured):
            log.warn("Feature tests with no configuration found (will not be run): {}".format(', '.join(unconfigured)))
        return len(unconfigured) == 0

    def _create_wrapper(self, ofile: Path) -> None:
        template = self.jinja_env.get_template('pkg_feature_exec_dispath-body.vhd')
        c = template.render(tests=self._tests)
        with ofile.open('wt', encoding='utf-8') as f:
            f.write(c)
