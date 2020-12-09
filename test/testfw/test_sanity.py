import logging
from textwrap import dedent
from .test_common import TestsBase, add_sources, dict_merge, vhdl_serialize, \
                         get_seed, OptionsDict

log = logging.getLogger(__name__)


class SanityTests(TestsBase):
    def add_sources(self):
        add_sources(self.lib, ['sanity/**/*.vhd'])

    def format_valid_test_name(self, name):
        valid_name = name.replace(" ", '_')
        valid_name = valid_name.replace("/", "_")
        return valid_name

    def configure(self):
        tb = self.lib.get_test_benches('*tb_sanity')[0]
        default = self.config['default']

        sim_options = self.get_sim_options(default)

        for name, cfg in self.config['tests'].items():

            dict_merge(cfg, default)
            generics = {
                'timeout'      : cfg['timeout'],
                'iterations'   : cfg['iterations'],
                'log_level'    : cfg['log_level'] + '_l',
                'error_tol'    : cfg['error_tolerance'],
                'seed'         : get_seed(cfg),
                'topology'     : cfg['topology'],
                'bus_len_v'    : vhdl_serialize(cfg['bus_len_v']),
                'trv_del_v'    : vhdl_serialize(cfg['trv_del_v']),
                'osc_tol_v'    : vhdl_serialize(cfg['osc_tol_v']),
                'nw_mean'      : vhdl_serialize(cfg['nw_mean']),
                'nw_var'       : vhdl_serialize(cfg['nw_var']),
                'ng_mean'      : vhdl_serialize(cfg['ng_mean']),
                'ng_var'       : vhdl_serialize(cfg['ng_var']),
                'timing_config': vhdl_serialize(cfg['timing_config']),
                'gauss_iter'   : vhdl_serialize(cfg['gauss_iter']),
            }

            local_sim_options = OptionsDict()
            if ('functional_coverage' in cfg) and cfg['functional_coverage'] == True:
                local_sim_options += self.add_psl_cov('{}.{}'.format(tb.name, name))
            local_sim_options = sim_options + local_sim_options
            tb.add_config(name, generics=generics, sim_options=local_sim_options)
        return True
