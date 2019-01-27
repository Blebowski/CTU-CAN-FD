import logging
from .test_common import TestsBase, add_sources, dict_merge, vhdl_serialize, \
                         get_seed

log = logging.getLogger(__name__)


def len_to_matrix(topology, bus_len):
    l = bus_len
    if topology == 'bus':
        bm = [[0.0,            l[1],           l[1]+l[2],      l[1]+l[2]+l[3]],
              [l[1],           0.0,            l[2],           l[2]+l[3]],
              [l[1]+l[2],      l[2],           0.0,            l[3]],
              [l[1]+l[2]+l[3], l[2]+l[3],      l[3],           0.0]]
    elif topology == 'star':
        bm = [[0.0,            l[1]+l[2],      l[1]+l[3],      l[1]+l[4]],
              [l[1]+l[2],      0.0,            l[2]+l[3],      l[2]+l[4]],
              [l[1]+l[3],      l[2]+l[3],      0.0,            l[3]+l[4]],
              [l[1]+l[4],      l[2]+l[4],      l[3]+l[4],      0.0]]
    elif topology == 'tree':
        bm = [[0.0,            l[1]+l[2],      l[1]+l[3]+l[5], l[1]+l[4]+l[5]],
              [l[1]+l[2],      0.0,            l[2]+l[3]+l[5], l[2]+l[4]+l[5]],
              [l[1]+l[3]+l[5], l[2]+l[3]+l[5], 0.0,            l[3]+l[4]],
              [l[1]+l[4]+l[5], l[2]+l[4]+l[5], l[3]+l[4],      0.0]]
    elif topology == 'ring':
        bm = [[0.0, min(l[1], l[2]+l[3]+l[4]), min(l[1]+l[2],l[3]+l[4]), min(l[4],l[1]+l[2]+l[3])],
              [min(l[1], l[2]+l[3]+l[4]), 0.0, min(l[2], l[1]+l[3]+l[4]), min(l[2]+l[3],l[1]+l[4])],
              [min(l[1]+l[2],l[3]+l[4]), min(l[2], l[1]+l[3]+l[4]), 0.0, min(l[3],l[1]+l[2]+l[4])],
              [min(l[4],l[1]+l[2]+l[3]), min(l[2]+l[3],l[1]+l[4]), min(l[3],l[1]+l[2]+l[4]), 0.0]]
    elif topology == 'custom':
        bm = [[0.0,  l[1], l[2], l[3]],
              [l[1], 0.0,  l[4], l[5]],
              [l[2], l[4], 0.0,  l[6]],
              [l[3], l[6], l[6], 0.0]]
    else:
        raise ValueError("Invalid bus topology.")

    return bm


class SanityTests(TestsBase):
    def add_sources(self):
        add_sources(self.lib, ['sanity/**/*.vhd'])

    def create_psl_cov_file_opt(self, name):
        psl_flag = "--psl-report=psl_cov_sanity_{}.json".format(name)
        return {"ghdl.sim_flags" : [psl_flag]}

    def configure(self):
        # TODO: wave
        tb = self.lib.get_test_benches('*tb_sanity')[0]
        default = self.config['default']
        self.add_modelsim_gui_file(tb, default, 'sanity')
        for name, cfg in self.config['tests'].items():
            if 'wave' in cfg:
                log.warn('"wave" in sanity test config {} is ignored' +
                         ' (set it in default instead)'.format(name))

            dict_merge(cfg, default)
            # bm = len_to_matrix(cfg['topology'], cfg['bus_len_v'])
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

            sanity_cfg_name = name.replace(" ", "_").replace("/", "_").strip('"')
            psl_opts = self.create_psl_cov_file_opt(sanity_cfg_name)
            tb.add_config(name, generics=generics, sim_options=psl_opts)
            return True
