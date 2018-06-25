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
        add_sources(self.lib, ['reference/tb_reference.vhd'])

    def configure(self) -> None:
        tb = self.lib.get_test_benches('*tb_reference')[0]
        default = self.config['default']
        self.add_modelsim_gui_file(tb, default, 'sanity')

        for data_set,cfg in config['tests'].items():
            dict_merge(cfg, default)
            # bm = len_to_matrix(cfg['topology'], cfg['bus_len_v'])
            generics = {
                'timeout'      : cfg['timeout'],
                'iterations'   : cfg['iterations'],
                'log_level'    : cfg['log_level'] + '_l',
                'error_tol'    : cfg['error_tolerance'],
                'seed'         : get_seed(cfg),
                'data_path'	   : cfg['data_path'],
            }
            tb.add_config(data_set, generics=generics)
