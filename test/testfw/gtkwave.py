from vcd.gtkw import GTKWSave
import tkinter
from typing import List
import logging
import traceback
import functools

log = logging.getLogger('gtkwave')


def logexc(f):
    @functools.wraps(f)
    def wrapper(*args, **kwds):
        try:
            return f(*args, **kwds)
        except:
            traceback.print_exc()
            raise
    return wrapper


class TclFuncs:
    def __init__(self, gtkw: str):
        self.gtkw = gtkw

        # set up TCL
        tcl = tkinter.Tcl()
        self.tcl = tcl
        tcl.createcommand("add", self.tcl_add)
        tcl.createcommand('quietly', self.tcl_quietly)

    def source(self, file: str):
        log.debug('Sourcing '+file)
        self.tcl.eval('source "{}"'.format(file))

    def finalize(self):
        pass

    def tcl_quietly(self, *args):
        return self.tcl.call(*args)

    @staticmethod
    def convsig(sig: str) -> str:
        return 'top.' + sig.replace('/', '.').replace('(', '[').replace(')', ']')

    @logexc
    def tcl_add(self, *args):
        i = 0
        if args[i] != 'wave':
            raise ValueError("Unsupported add TCL command")
        i += 1

        label = None
        format = 'hex'
        signal = None
        isdivider = False
        group = None

        while i < len(args):
            a0 = args[i]
            i += 1
            if a0 == '-label':
                label = args[i]
                i += 1
            elif a0 == '-hexadecimal':
                format = 'hex'
            elif a0 == '-unsigned':
                format = 'dec'
            elif a0 == '-signed':  # ????
                format = 'signed'
            elif a0 == '-noupdate':
                pass
            elif a0 == '-divider':
                isdivider = True
            elif a0 == '-height':
                i += 1
            elif a0 == '-group':
                if group:
                    self.gtkw.end_group(group, closed=False)
                group = args[i]
                self.gtkw.begin_group(group, closed=False)
                i += 1
            elif a0[0] == '-':
                raise ValueError("Unknown TCL add wave arg " + a0)
            else:
                signal = a0
                if isdivider:
                    self.gtkw.blank(label=signal)
                else:
                    self.gtkw.trace(self.convsig(signal), alias=label, datafmt=format)
                label = None
                format = 'hex'
                signal = None
                isdivider = False
        if group:
            self.gtkw.end_group(group)


def tcl2gtkw(tcl_wave, tcl_init_files: List[str], gtkw):
    with open(gtkw, 'wt') as f:
        gtkw = GTKWSave(f)
        gtkw.zoom_markers(-27.0)
        c = TclFuncs(gtkw)
        c.tcl.createcommand('vunit_help', lambda: None)
        for tcl in tcl_init_files:
            c.source(tcl)
        c.tcl.createcommand('run_simulation', lambda: None)
        c.source(tcl_wave)
        c.finalize()
