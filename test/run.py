#!/usr/bin/env python3
# run sanity test using vunit
# Requires: vunit
# $ python run.py # select automatically
# $ # select manually
# $ VUNIT_SIMULATOR=modelsim PATH=$PATH:$MODELSIM_BIN python run.py
# $ VUNIT_SIMULATOR=ghdl python run.py


from testfw import cli


if __name__ == '__main__':
    cli()

# ------------------------------------------------------------------------------


#ui.enable_location_preprocessing(additional_subprograms=['log'])


