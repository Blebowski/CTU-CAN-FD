import subprocess as sp
from pathlib import Path
from pprint import pprint
import attr
from typing import Dict


@attr.s
class hbase:
    name = attr.ib()  # type: str
    children = attr.ib(factory=dict, kw_only=True)  # type: Dict[str, hbase]


@attr.s
class design(hbase):
    @classmethod
    def create(cls, dummy) -> 'design':
        return cls(name='')


@attr.s
class signal(hbase):
    type = attr.ib(kw_only=True)
    extra = attr.ib(kw_only=True)
    kind = attr.ib(kw_only=True)  # signal / port-in / port-out / port-inout
    @classmethod
    def create(cls, type, name, extra) -> 'signal':
        tp, id = extra.split(':', 1)
        return cls(name=name, type=tp, extra=id.strip(), kind=type)


@attr.s
class container(hbase):
    kind = attr.ib(kw_only=True)  # package / process / instance / generate-if
    @classmethod
    def create(cls, type, name) -> 'container':
        return cls(name=name, kind=type)


@attr.s
class generate_for(hbase):
    index = attr.ib(kw_only=True)
    @classmethod
    def create(cls, type, name, extra) -> 'generate_for':
        return cls(name=name+extra, index=extra)  # TODO


def factory(line: str):
    type, rest = line.split(' ', 1) if ' ' in line else (line, '')
    name_rest = rest.split(':', 1)
    r = tuple(x.strip() for x in [type]+name_rest if x.strip())
    #print(r)
    factories = {
        'design':       design.create,
        'signal':       signal.create,
        'port-in':      signal.create,
        'port-out':     signal.create,
        'port-inout':   signal.create,
        'generate-for': generate_for.create,
        'package':      container.create,
        'process':      container.create,
        'instance':     container.create,
        'generate-if':  container.create,
        'block':        container.create,
    }
    fact = factories[r[0]]
    return fact(*r)


def parse(s: str) -> dict:
    """Parse signal hierarchy from `ghwdump -h`."""
    lines = s.splitlines()

    curr_ws = 0
    last = dict()
    stack = [last]
    for i, line in enumerate(lines[:4096]):
        l = line.lstrip()
        if not l:
            continue
        ws = len(line) - len(l)
        if ws > curr_ws:
            assert curr_ws + 1 == ws
            stack.append(last)
        elif ws < curr_ws:
            for _ in range(curr_ws-ws):
                stack.pop()
        #print("{:4d}: ws {:2d}; depth {:2d}: |{:<100} -- parent = {}".format(i, ws, len(stack)-1, line, ""))
        obj = factory(l)
        last = obj.children
        stack[-1][obj.name] = obj
        curr_ws = ws
    #pprint(stack)
    return next(stack[0].values().__iter__())
