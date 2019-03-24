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

        sim_options = self.get_default_sim_options()
        # generate & set per-test modelsim tcl file
        sim_options += self.generate_init_tcl('modelsim_init_sanity.tcl', 'tb_sanity')
        sim_options += self.add_modelsim_gui_file(tb, default, 'sanity', sim_options['modelsim.init_files.after_load'])

        for name, cfg in self.config['tests'].items():
            if 'wave' in cfg:
                log.warn('"wave" in sanity test config {} is ignored' +
                         ' (set it in default instead)'.format(name))

            dict_merge(cfg, default)
            generics = {
                'timeout'      : cfg['timeout'],
                'iterations'   : cfg['iterations'],
                'log_level'    : cfg['log_level'] + '_l',
                'error_tol'    : cfg['error_tolerance'],
                'seed'         : get_seed(cfg),
                'topology'     : cfg['topology'],
                #'bm'           : vhdl_serialize(bm),
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
            if cfg['psl_coverage']:
                local_sim_options += self.add_psl_cov('{}.{}'.format(tb.name, name))
            local_sim_options = sim_options + local_sim_options
            tb.add_config(name, generics=generics, sim_options=local_sim_options)
        return True
