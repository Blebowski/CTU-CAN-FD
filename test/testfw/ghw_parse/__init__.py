"""Tools to parse hierarchy from GHW files and find out array ranges.
"""

from . import hierarchy
from . import types

import subprocess as sp
from pathlib import Path
from pprint import pprint
import re
import pickle
from functools import wraps
from typing import Tuple, List
import hashlib

GHWDUMP = 'ghwdump'


def cached(f):
    @wraps(f)
    def wrapper(path: Path):
        cache = path.with_suffix('.{}.pck'.format(f.__name__))
        h = hashlib.sha256()
        with path.open('rb') as ff:
            h.update(ff.read())
        curr_hash = h.digest()
        old_hash = None

        if cache.is_file():
            with cache.open('rb') as ff:
                old_hash, data = pickle.load(ff)

        if old_hash != curr_hash:
            data = f(path)
            with cache.open('wb') as ff:
                pickle.dump((curr_hash, data), ff)
        return data
    return wrapper


#@cached
def parse_hierarchy(ghw: Path):
    res = sp.run([GHWDUMP, '-h', str(ghw)], stdout=sp.PIPE)
    return hierarchy.parse(res.stdout.decode('ascii'))


#@cached
def parse_types(ghw: Path):
    res = sp.run([GHWDUMP, '-t', str(ghw)], stdout=sp.PIPE)
    return types.parse(res.stdout.decode('latin1'))



"""
TODO:
- parse types
- represent the hierarchy as proper tree
    - may be dict, keys are id()s, values the whole thing
- find
    - separates the name into name and (optional) index
    - searches for both name-only and name-with-index
    - in case the matching item is a signal
        - descend into type
"""


def inject_types(h, ts):
    if hasattr(h, 'type'):
        parsed = types.parse_type(h.type)
        t = types.resolve(parsed, ts)
        h.type = t
    if hasattr(h, 'children'):
        for c in h.children.values():
            inject_types(c, ts)


def find(h, fqn):
    names = fqn.split('.')
    curr = h
    def children(o):
        return o.items if isinstance(o, types.t_record) else o.children

    for name in names:
        if name in children(curr):
            curr = children(curr)[name]
        elif isinstance(curr, hierarchy.signal):
            # the name may refer to a signal, which is a record ...
            type = curr.type
            assert isinstance(type, types.t_record)
            curr = type.items[name]
        else:
            # the name may refer to a signal, which is an array ...
            m = re.match(r'^([^(]+)\(([0-9]+)\)$', name)
            if not m:
                raise KeyError('{} (from {}) not found in {}'
                               .format(name, fqn, sorted(curr.children.keys())),
                               name, fqn, list(curr.children.keys()))
            #print(name, m.groups())
            name, index = m.groups()
            index = int(index)
            if name in children(curr):
                signal = children(curr)[name]
                type = signal.type
                #print(type)
                range, type = strip_array(type)
                # TODO: check range: type.range.contains(index)
                while isinstance(type, types.t_subarray):
                    type = type.type
                assert isinstance(type, types.t_array)
                type = type.type
                curr = type
            else:
                raise KeyError('{} (from {}) not found in {}'
                               .format(name, fqn, list(curr.children.keys())))

    if not isinstance(curr, types.t_base):
        curr = curr.type
    return curr


def is_array(type) -> bool:
    return isinstance(type, types.t_subarray) or \
           isinstance(type, types.t_array)


def is_record(type) -> bool:
    return isinstance(type, types.t_record)


def strip_array(type) -> Tuple[List[types.t_range], types.t_base]:
    ranges = type.ranges
    while isinstance(type, types.t_subarray):
        type = type.type
    return ranges, type


@cached
def parse(path: Path):
    h = parse_hierarchy(path)
    t = parse_types(path)
    inject_types(h, t)
    return h



if __name__ == '__main__':
    h = parse(Path('wave.ghw'))
    #print(type(h))
    #pprint(attr.asdict(h))

    x = find(h, 'tb_feature.test_comp.g_inst(1).can_inst.drv_bus')
    pprint(x)
    pprint(strip_array(x))
    x = find(h, 'tb_feature.test_comp.p(1).clk_sys')
    pprint(x)
    #pprint(strip_array(x))
