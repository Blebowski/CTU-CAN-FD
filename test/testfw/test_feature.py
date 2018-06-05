import logging
from .test_common import *

log = logging.getLogger(__name__)

class FeatureTests(TestsBase):
    def add_sources(self):
        add_sources(self.lib, ['feature/**/*.vhd'])

    def configure(self):
        pass
