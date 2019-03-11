from vcd.gtkw import GTKWSave
import tkinter
from typing import List
import logging
import traceback
import functools
import re
from pathlib import Path
from . import ghw_parse

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
    def __init__(self, gtkw: str, hierarchy):
        self.gtkw = gtkw
        self.hierarchy = hierarchy

        # set up TCL
        tcl = tkinter.Tcl()
        self.tcl = tcl
        tcl.createcommand("add", self.tcl_add)
        tcl.createcommand('quietly', self.tcl_quietly)

    def source(self, file: str):
        log.debug('Sourcing '+file)
        self.tcl.eval('source "{}"'.format(file))

    def eval(self, code: str):
        self.tcl.eval(code)

    def finalize(self):
        pass

    def tcl_quietly(self, *args):
        return self.tcl.call(*args)

    def sigtype(self, sig: str):
        fqn = sig.replace('/', '.')
        fqn = re.sub(r'__([0-9]+)', r'(\1)', fqn)
        type = ghw_parse.find(self.hierarchy, fqn)
        return type

    def convsig(self, sig: str) -> str:
        fqn = sig.replace('/', '.')
        fqn = re.sub(r'__([0-9]+)', r'(\1)', fqn)
        type = ghw_parse.find(self.hierarchy, fqn)
        if ghw_parse.is_array(type):
            ranges, type = ghw_parse.strip_array(type)
            if len(ranges) > 1:
                raise NotImplementedError("Multidimensional arrays are not supported (yet)")
            l, r = ranges[0].left, ranges[0].right
            fqn += '({}:{})'.format(l, r)
        fqn = 'top.' + fqn
        return fqn.replace('(', '[').replace(')', ']').lower()

    def _add_trace(self, signal, type, *, label: str, datafmt: str, expand: bool, **kwds):
        if ghw_parse.is_record(type):
            with self.gtkw.group(label, closed=not expand):
                for iname, itype in type.items.items():
                    # do not pass label
                    self._add_trace(signal+'/'+iname, itype, datafmt=datafmt, expand=False, label=None, **kwds)
        else:
            signal = self.convsig(signal)
            self.gtkw.trace(signal, alias=label, datafmt=datafmt, **kwds)

    class Opts:
        def reset(self):
            self.label = None
            self.format = 'hex'
            self.signal = None
            self.isdivider = False
            self.group = None
            self.expand = False
            self.color = None

        def __init__(self):
            self.reset()

    @staticmethod
    def conv_color(clr: str) -> int:
        colors = [
            'normal',
            'red',
            'orange',
            'yellow',
            'green',
            'blue',
            'indigo',
            'violet',
            'cycle',
        ]
        mapping = {
            'cyan': 'yellow'
        }
        clr = clr.lower()
        clr = mapping.get(clr)
        return colors.index(clr)

    @logexc
    def tcl_add(self, *args):
        i = 0
        if args[i] != 'wave':
            raise ValueError("Unsupported add TCL command")
        i += 1

        o = self.Opts()
        while i < len(args):
            a0 = args[i]
            i += 1
            if a0 == '-label':
                o.label = args[i]
                i += 1
            elif a0 == '-hexadecimal':
                o.format = 'hex'
            elif a0 == '-unsigned':
                o.format = 'dec'
            elif a0 == '-decimal':
                o.format = 'dec'
            elif a0 == '-signed':  # ????
                o.format = 'signed'
            elif a0 == '-noupdate':
                pass
            elif a0 == '-color':
                o.color = self.conv_color(args[i])
                i += 1
            elif a0 == '-divider':
                o.isdivider = True
            elif a0 == '-height':
                i += 1
            elif a0 == '-expand':
                o.expand = True
            elif a0 == '-event':
                pass
            elif a0 == '-group':
                if o.group:
                    self.gtkw.end_group(o.group, closed=False)
                o.group = args[i]
                self.gtkw.begin_group(o.group, closed=False)
                i += 1
            elif a0[0] == '-':
                raise ValueError("Unknown TCL add wave arg " + a0)
            else:
                signal = a0
                if o.isdivider:
                    self.gtkw.blank(label=signal)
                else:
                    try:
                        type = self.sigtype(signal)
                    except KeyError as e:
                        log.warning(e.args[0])
                        break
                    self._add_trace(signal, type, label=o.label, datafmt=o.format, expand=o.expand, color=o.color)
                o.reset()
        if o.group:
            self.gtkw.end_group(o.group)


def tcl2gtkw(tcl_wave, tcl_init_files: List[str], gtkw, ghw: Path):
    hierarchy = ghw_parse.parse(ghw)
    with open(gtkw, 'wt') as f:
        gtkw = GTKWSave(f)
        gtkw.zoom_markers(-27.0)
        c = TclFuncs(gtkw, hierarchy)
        c.tcl.eval('set SILENT_SANITY "false"')  # TODO: propagate from YML config
        c.tcl.createcommand('vunit_help', lambda: None)
        for tcl in tcl_init_files:
            c.source(tcl)
        c.tcl.createcommand('run_simulation', lambda: None)
        c.source(tcl_wave)
        c.finalize()
