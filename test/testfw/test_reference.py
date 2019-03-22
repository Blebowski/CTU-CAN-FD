import logging
from pathlib import Path
from .test_common import add_sources, TestsBase, dict_merge, \
                         get_seed, OptionsDict

log = logging.getLogger(__name__)


class ReferenceTests(TestsBase):
    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self._tests = self.config['tests'].keys()

    def add_sources(self) -> None:
        sources = []
        sources.append('reference/tb_reference.vhd')
        sources.append('reference/vunit_reference_wrapper.vhd')
        add_sources(self.lib, sources)

    def configure(self) -> bool:
        tb = self.lib.get_test_benches('*reference*')[0]
        default = self.config['default']

        # TODO: is this necessary?
        tb.scan_tests_from_file(str(self.base / "reference/vunit_reference_wrapper.vhd"))

        sim_options = self.get_default_sim_options()
        # generate & set per-test modelsim tcl file
        sim_options += self.generate_init_tcl('modelsim_init_reference.tcl', 'tb_reference_wrapper/i_test')
        sim_options += self.add_modelsim_gui_file(tb, default, 'reference', sim_options['modelsim.init_files.after_load'])

        for data_set, cfg in self.config['tests'].items():
            dict_merge(cfg, default)
            generics = {
                'timeout'      : cfg['timeout'],
                'iterations'   : cfg['iterations'],
                'log_level'    : cfg['log_level'] + '_l',
                'error_tol'    : cfg['error_tolerance'],
                'seed'         : get_seed(cfg),
                'data_path'    : str(self.build) + '/../' + cfg['data_path'],
            }
            local_sim_options = OptionsDict()
            if cfg['psl_coverage']:
                local_sim_options += self.add_psl_cov('{}.{}'.format(tb.name, data_set))
            local_sim_options = sim_options + local_sim_options
            tb.add_config(data_set, generics=generics, sim_options=local_sim_options)

        return True
