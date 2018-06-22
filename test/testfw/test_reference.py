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
        self._create_wrapper(self.build / "tb_reference_wrappers.vhd")
        add_sources(self.lib, sources)

    def configure(self) -> None:
        lib, config, build = self.lib, self.config, self.build
        default = config['default']
        reference_tests = lib.get_test_benches('*reference_wrapper*')

        for name, cfg in config['tests'].items():
            dict_merge(cfg, default)
            tb = lib.get_test_benches('*vunittb_reference_wrapper_{}'.format(name),
                                      allow_empty=True)
            if not len(tb):
                pprint([x.name for x in unit_tests])
                raise RuntimeError('Testbench vunittb_reference_wrapper_{} does not exist'
                                   + ' (but specified in config).'.format(name))
            assert len(tb) == 1
            tb = tb[0]
            tb.set_generic('timeout', cfg['timeout'])
            tb.set_generic('iterations', cfg['iterations'])
            tb.set_generic('log_level', cfg['log_level'] + '_l')
            tb.set_generic('error_tol', cfg['error_tolerance'])
            tb.set_generic('seed', get_seed(cfg))
            tb.set_generic('config_path', cfg['path'])

        # Generate & set modelsim TCL file. One is enough, each test instance
        # differs only by "config_path"!
        tcl = build / 'modelsim_init_{}.tcl'.format("reference_tb")
        with tcl.open('wt', encoding='utf-8') as f:
            print(dedent('''\
                global TCOMP
                set TCOMP tb/i_test
                '''.format(name)), file=f)
        init_files = get_common_modelsim_init_files()
        init_files += [str(tcl)]
        tb.set_sim_option("modelsim.init_files.after_load", init_files)
        self.add_modelsim_gui_file(tb, cfg, name)

    def _create_wrapper(self, fname):
        fname = str(fname)
        files = self.lib.get_source_files()
        data_sets = []

        # Create wrapper for each "data_set"
        for name,cfg in self.config["tests"].items():
            data_sets.append(name);

        c = self.jinja_env.get_template('vunittb_reference_wrapper.vhd').render(data_sets=data_sets)
        with open(fname, "wt", encoding='utf-8') as f:
            f.write(c)

        self.lib.add_source_file(fname)

