import logging
from pathlib import Path
from .test_common import add_sources, TestsBase, dict_merge, \
                         get_seed, OptionsDict
from textwrap import dedent
import re

log = logging.getLogger(__name__)


class ComplianceTests(TestsBase):

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self._tests = self.config['tests'].keys()

    def add_sources(self) -> None:
        sources = [];
        #sources.append('compliance/com/*.vhd');
        sources.append('compliance/clk_gen_agent/*.vhd');
        sources.append('compliance/rst_gen_agent/*.vhd');
        sources.append('compliance/mem_bus_agent/*.vhd');
        sources.append('compliance/can_agent/*.vhd');
        sources.append('compliance/test_controller_agent/*.vhd');
        sources.append('compliance/*.vhd')
        add_sources(self.lib, sources)

        #wrname = self.build / 'pkg_feature_exec_dispatch-body.vhd'
        #self._create_wrapper(wrname)
        # add_sources(self.lib, [str(wrname)])
        #self.lib.add_source_file(str(wrname))
        tb = self.lib.get_test_benches('*can_compliance_tb')[0]
        #tb.scan_tests_from_file(str(wrname))

    def configure(self) -> bool:
        tb = self.lib.get_test_benches('*can_compliance_tb')[0]

        default = self.config['default']
        sim_options = self.get_default_sim_options()
        # generate & set per-test modelsim tcl file
        sim_options += self.generate_init_tcl('modelsim_init_feature.tcl', 'tb_feature/test_comp')
        sim_options += self.add_modelsim_gui_file(tb, default, 'feature', sim_options['modelsim.init_files.after_load'])
        sim_options['ghdl.sim_flags'] += ["--vpi=../compliance/sw_model/build/simulator_interface/libSIMULATOR_INTERFACE_LIB.so"]
        #sim_options['ghdl.elab_flags'] += ["-Wl,../compliance/sw_model/build/simulator_interface/libSIMULATOR_INTERFACE_LIB.so"]

        for name, cfg in self.config['tests'].items():
            if cfg is None:
                cfg = dict()

            dict_merge(cfg, default)

            generics = {
                'vpi_test_name'     : name
            }
            ##generics = {
            ##    'timeout'      : cfg['timeout'],
            ##    'iterations'   : cfg['iterations'],
            ##    'log_level'    : cfg['log_level'] + '_l',
            ##    'error_tol'    : cfg['error_tolerance'],
            ##    'test_name'    : name,
            ##    'seed'         : get_seed(cfg)
            ##}

            local_sim_options = OptionsDict()
            
            ##if cfg['psl_coverage']:
            ##    local_sim_options += self.add_psl_cov('{}.{}'.format(tb.name, name))

            local_sim_options = sim_options + local_sim_options
            #tb.add_config(name, sim_options=local_sim_options)
            tb.add_config(name, generics=generics, sim_options=local_sim_options)

        return True
