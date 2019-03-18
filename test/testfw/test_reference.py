import logging
from pathlib import Path
from .test_common import add_sources, TestsBase, dict_merge, \
                         get_common_modelsim_init_files, get_seed
from textwrap import dedent

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

    def create_psl_cov_file_opt(self, name):
        psl_path = "functional_coverage/coverage_data/psl_cov_reference_{}.json".format(name)
        psl_flag = "--psl-report={}".format(psl_path)
        return {"ghdl.sim_flags" : [psl_flag]}

    def configure(self) -> bool:
        tb = self.lib.get_test_benches('*reference*')[0]
        default = self.config['default']

        tcl = self.build / 'modelsim_init_reference.tcl'
        with tcl.open('wt', encoding='utf-8') as f:
            print(dedent('''\
                global TCOMP
                set TCOMP tb_reference_wrapper/i_test
                '''), file=f)

        init_files = get_common_modelsim_init_files()
        init_files += [str(tcl)]

        for data_set,cfg in self.config['tests'].items():
            dict_merge(cfg, default)
            # bm = len_to_matrix(cfg['topology'], cfg['bus_len_v'])
            generics = {
                'timeout'      : cfg['timeout'],
                'iterations'   : cfg['iterations'],
                'log_level'    : cfg['log_level'] + '_l',
                'error_tol'    : cfg['error_tolerance'],
                'seed'         : get_seed(cfg),
                'data_path'    : str(self.build) + '/../' + cfg['data_path'],
            }

            if (cfg['psl_coverage']):
                psl_opts = self.create_psl_cov_file_opt(data_set)
                tb.add_config(data_set, generics=generics, sim_options=psl_opts)
            else:
                tb.add_config(data_set, generics=generics)

        tb.set_sim_option("modelsim.init_files.after_load", init_files)
        self.add_modelsim_gui_file(tb, default, 'reference', init_files)
        return True
