import logging
from pathlib import Path
from .test_common import add_sources, TestsBase, dict_merge, \
                         get_seed, OptionsDict
from textwrap import dedent
import re

log = logging.getLogger(__name__)


class MainTests(TestsBase):

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self._tests = self.config['tests'].keys()

    def add_sources(self) -> None:
        sources = [];
        #sources.append('compliance/com/*.vhd');
        sources.append('main_tb/pkg/*.vhd');
        sources.append('main_tb/common/*.vhd');

        sources.append('main_tb/agents/reset_agent/*.vhd');
        sources.append('main_tb/agents/clock_agent/*.vhd');
        sources.append('main_tb/agents/memory_bus_agent/*.vhd');
        sources.append('main_tb/agents/timestamp_agent/*.vhd');
        sources.append('main_tb/agents/interrupt_agent/*.vhd');
        sources.append('main_tb/agents/can_agent/*.vhd');

        sources.append('main_tb/agents/test_controller_agent/*.vhd');

        sources.append('main_tb/agents/feature_test_agent/*.vhd');
        sources.append('main_tb/agents/compliance_test_agent/*.vhd');
        sources.append('main_tb/agents/reference_test_agent/*.vhd');

        sources.append('main_tb/contexts/*.vhd');
        sources.append('main_tb/ctu_can_fd_vip.vhd');
        sources.append('main_tb/tb_top_ctu_can_fd.vhd');

        add_sources(self.lib, sources)

        #wrname = self.build / 'pkg_feature_exec_dispatch-body.vhd'
        #self._create_wrapper(wrname)
        # add_sources(self.lib, [str(wrname)])
        #self.lib.add_source_file(str(wrname))
        tb = self.lib.get_test_benches('*tb_top*')[0]
        #tb.scan_tests_from_file(str(wrname))

    def configure(self) -> bool:
        tb = self.lib.get_test_benches('*tb_top*')[0]

        global_config = self.config
        sim_options = self.get_sim_options(global_config)

        #sim_options['ghdl.sim_flags'] += ["--vpi=../compliance/libSIMULATOR_INTERFACE_LIB.so"]
        #sim_options['ghdl.elab_flags'] += ["-Wl,../compliance/sw_model/build/simulator_interface/libSIMULATOR_INTERFACE_LIB.so"]

        for name, cfg in self.config['tests'].items():
            if cfg is None:
                cfg = dict()

            dict_merge(cfg, global_config)

            generics = {
                'test_name'             : name,
                'test_type'             : cfg["test_type"],
                'iterations'            : cfg["iterations"],
                'cfg_sys_clk_period'    : cfg["system_clock_period"],

                'cfg_brp'               : cfg["brp"],
                'cfg_prop'              : cfg["prop"],
                'cfg_ph_1'              : cfg["ph_1"],
                'cfg_ph_2'              : cfg["ph_2"],
                'cfg_sjw'               : cfg["sjw"],
                'cfg_brp_fd'            : cfg["brp_fd"],
                'cfg_prop_fd'           : cfg["prop_fd"],
                'cfg_ph_1_fd'           : cfg["ph_1_fd"],
                'cfg_ph_2_fd'           : cfg["ph_2_fd"],
                'cfg_sjw_fd'            : cfg["sjw_fd"],

                'rx_buffer_size'        : cfg['rx_buffer_size'],
                'txt_buffer_count'      : cfg['txt_buffer_count'],
                'sup_filtA'             : cfg['sup_filtA'],
                'sup_filtB'             : cfg['sup_filtB'],
                'sup_filtC'             : cfg['sup_filtC'],
                'sup_range'             : cfg['sup_range'],
                'sup_traffic_ctrs'      : cfg['sup_traffic_ctrs'],
                'target_technology'     : cfg['target_technology'],

                'seed'                  : get_seed(cfg)
            }

            local_sim_options = OptionsDict()
            
            if ('functional_coverage' in cfg) and cfg['functional_coverage'] == True:
                local_sim_options += self.add_psl_cov('{}.{}'.format(tb.name, name))

            local_sim_options = sim_options + local_sim_options
            #tb.add_config(name, sim_options=local_sim_options)
            tb.add_config(name, generics=generics, sim_options=local_sim_options)

        return True
