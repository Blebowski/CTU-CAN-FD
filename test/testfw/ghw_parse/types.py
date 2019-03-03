import subprocess as sp
from pathlib import Path
from pprint import pprint
import attr
from typing import Dict, OrderedDict, List, Any, Optional
import re
from enum import Enum
from parsy import string, regex, seq


@attr.s
class t_base:
    name: Optional[str] = attr.ib(default=None, kw_only=True)


@attr.s
class t_ref:
    """Unresolved reference to another type."""
    name: str = attr.ib(kw_only=True)


class Direction(Enum):
    to = 'to'
    downto = 'downto'


@attr.s
class t_range:
    left: int = attr.ib(kw_only=True)
    direction: Direction = attr.ib(kw_only=True)
    right: int = attr.ib(kw_only=True)
    range_type: t_base = attr.ib(kw_only=True, default=t_ref(name='integer'))


@attr.s
class t_array(t_base):
    type: t_base = attr.ib(kw_only=True)
    ranges: List[t_range] = attr.ib(kw_only=True)


@attr.s
class t_subarray(t_base):
    type: t_array = attr.ib(kw_only=True)
    ranges: List[t_range] = attr.ib(kw_only=True)


@attr.s
class t_record(t_base):
    items: OrderedDict[str, t_base] = attr.ib(kw_only=True)


@attr.s
class t_enum(t_base):
    items: List[Any] = attr.ib(kw_only=True)


@attr.s
class t_number(t_base):
    type: t_base = attr.ib(kw_only=True)
    range: t_range = attr.ib(kw_only=True)


@attr.s
class t_unconstrained_range(t_base):
    pass


@attr.s
class type_binding:
    name: str = attr.ib(kw_only=True)
    type: t_base = attr.ib(kw_only=True)
    @classmethod
    def create(cls, factory, *, name, **kwds) -> 'type_binding':
        return cls(name=name, type=factory(**kwds))


def tb(factory):
    def f(**kwds):
        return type_binding.create(factory=factory, **kwds)
    return f


def tbs(type_bindings: List[type_binding]) -> Dict[str,t_base]:
    res = dict()
    for t in type_bindings:
        # some array types are defined in 2 steps:
        #  type mem_bus_arr_t is array (integer range <>) of avalon_mem_type;
        #  subtype mem_bus_arr_t is mem_bus_arr_t (1 to 2);
        # This takes care of that.
        if t.name in res:
            res[t.name] = resolve(t.type, res)
        else:
            res[t.name] = t.type
    return res


comment = regex('--.*$', re.MULTILINE).desc('comment')
ws = (regex(r'\s+') >> comment.optional()).at_least(1).desc('whitespace')
ows = ws.optional().desc('ows')

_type = string('type')
_subtype = string('subtype')
_is = string('is')
_end = string('end')
_record = string('record')
_to = string('to')
_downto = string('downto')
_range = string('range')
_array = string('array')
_of = string('of')
_units = string('units')
_rng_unconstrained = string('<>').result(t_unconstrained_range())
LP = ows >> string('(') << ows
RP = ows >> string(')') << ows
COMMA = ows >> string(',') << ows
INT = regex(r'[+-]?\d+').map(int).desc('integer')

identifier = regex(r'[a-zA-Z_][a-zA-Z_0-9]*').desc('identifier')
t_name = identifier.tag('name')
T_REF = seq(t_name).combine_dict(t_ref)

list_item = INT | identifier | regex(r"'.'")
comma_sep_list = LP >> list_item.sep_by(COMMA) << RP
enum = seq(_type >> ws >> t_name,
           ws >> _is >> ows >> comma_sep_list.tag('items')
           ).combine_dict(tb(t_enum))


# --- Enum

# 1 to 2, N downto M, <>
range = seq(list_item.tag('left'),
            ws >> (_to | _downto).map(Direction).tag('direction') << ws,
            list_item.tag('right')
            ).combine_dict(t_range).desc('Range') | _rng_unconstrained

# std_logic, std_logic_vector(1 to 2)
Type = T_REF.desc('type')
type_int_range = seq(Type.tag('type'),
                     ws >> _range >> ws >> range.tag('range')
                     ).combine_dict(t_number)
type_or_array = seq(Type.tag('type'),
                    (LP >> range.sep_by(COMMA, min=1).tag('ranges') << RP)
                    ).combine_dict(t_subarray).desc('array') | type_int_range | Type


# --- Record
record_item = seq(ows >> t_name,
                  regex(r'\s*:\s*') >> type_or_array.tag('type') << string(';')
                  ).combine_dict(type_binding)

record = seq(_type >> ws >> t_name,
             ws >> _is >> ws >> _record >> ws >> record_item.many().map(tbs).tag('items') <<
             (ows >> _end >> ws >> _record)).combine_dict(tb(t_record))

# type integer is range <>
td_rng_1 = seq(_type >> ws >> t_name <<
               ws << _is << ws << _range << ws << _rng_unconstrained).combine_dict(tb(t_base))

# type natural is integer range 0 to 123
td_rng = seq((_type | _subtype) >> ws >> t_name,
             ws >> _is >> ws >> T_REF.tag('type'),
             ws >> _range >> ws >> range.tag('range')).combine_dict(tb(t_number)) \
         | td_rng_1

# type std_ulogic_vector is array (natural range <>) of std_ulogic;
td_arr = seq(_type >> ws >> t_name,
             ws >> _is >> ws >> _array >> LP >> (T_REF.tag('range_type') >>
                ws >> _range >> ws >> range).sep_by(COMMA, min=1).tag('ranges'),
             RP >> _of >> ws >> type_or_array.tag('type')).combine_dict(tb(t_array)).desc('arr')

# type A is B
td_alias = seq((_type | _subtype) >> ws >> t_name,
               ws >> _is >> ws >> T_REF.tag('type')
               ).combine_dict(type_binding)

# subtype runner_sync_t is std_ulogic_vector (0 to 2);
td_subarr = seq(_subtype >> ws >> t_name,
                ws >> _is >> ws >> T_REF.tag('type'),
                LP >> range.sep_by(COMMA, min=1).tag('ranges') << RP).combine_dict(tb(t_subarray)).desc('subarr')

# type time is range <> units ... end units;
td_units = seq(_type >> ws >> t_name <<
               ws << _is << ws << _range << ws << range << ws << _units <<
               regex('.*?end units', re.DOTALL)).combine_dict(tb(t_base)).desc('units')

type_def = td_units | enum | record | td_rng | td_arr | td_subarr | td_alias

top = (ows >> type_def << ows << regex(r';')).many().map(tbs) << ows


def resolve(o, top: Dict[str, t_base]):
    """Resolve the `t_ref` type references."""
    if isinstance(o, t_ref):
        res = resolve(top[o.name], top)
        res.name = o.name  # set name, so that the information is keps
        return res
    elif isinstance(o, t_base):
        return type(o)(**resolve(o.__dict__, top))
    elif isinstance(o, dict):
        return dict((k, resolve(v, top)) for k, v in o.items())
    else:
        return o


def parse_type(s: str) -> t_base:
    return (type_int_range | type_or_array).parse(s)


def parse(s: str) -> Dict[str, t_base]:
    """Parse a list of VHDL types into type hierarchy."""
    s = re.sub(r'end units', 'end units;', s)
    s = re.sub(r'\(\(null\) to.*?\);', '(0 to 0);', s)
    # print('\n'.join(s.splitlines()[14:17]))
    bindings = top.parse(s)
    # now resolve the forward references ...
    bindings = resolve(bindings, bindings)

    return bindings


if __name__ == '__main__':
    from pprint import pprint
    """
    print(ws.parse(' '))
    print(ws.parse('\n'))
    print(ws.parse('   -- asdasd\n'))
    print(ws.parse('   -- asdasd\n   '))
    print(ws.parse('   -- asdasd\n   -- xxx\n'))
    print(enum.parse('type aaa is (a,b, c)'))
    for s in ['1 to 65', 'N downto M']:
        print(range.parse(s))
    for s in ['std_logic', 'std_logic_vector (31 downto 0)']:
        print(type_or_array.parse(s))
    pprint(record.parse(r'''type avalon_mem_type is record
          clk_sys: std_logic;
          data_in: std_ulogic_vector (31 downto 0);
          data_out: std_ulogic_vector (31 downto 0);
          address: std_ulogic_vector (23 downto 0);
          scs: std_logic;
          swr: std_logic;
          srd: std_logic;
          sbe: std_ulogic_vector (3 downto 0);
        end record'''))
    for s in ["subtype std_logic is std_ulogic range 'U' to '-'", "type integer is range <>", "subtype natural is integer range 0 to 2147483647"]:
        print(s)
        pprint(td_rng.parse(s))
    for s in ["type std_ulogic_vector is array (natural range <>) of std_ulogic"]:
        print(s)
        pprint(td_arr.parse(s))

    pprint(top.parse('''type time is range <> units
      fs = 1 fs;
      ps = 1000 fs;
      ns = 1000000 fs;
      us = 1000000000 fs;
      ms = 1000000000000 fs;
      sec = 1000000000000000 fs;
      min = 60000000000000000 fs;
      hr = 3600000000000000000 fs;
    end units;'''))

    pprint(top.parse('subtype logger_memory_type is logger_memory_type (0 to 15);'))
    pprint(td_alias.parse('subtype phase_locks_t is phase_locks_unresolved_t'))

    pprint((type_int_range|type_or_array).parse('integer range 0 to 100'))
    pprint(top.parse('type mem_bus_arr_t is array (integer range <>) of avalon_mem_type;'))
    #"""

    # pprint(range.sep_by(COMMA, min=1).parse('integer range <>'))
    # pprint(range.sep_by(COMMA, min=1).parse('integer range <>, integer range <>'))
    pprint(td_arr.parse('type delay_matrix_type is array (integer range <>) of time'))
    pprint(td_arr.parse('type delay_matrix_type is array (integer range <>, integer range <>) of time'))
    pprint(td_subarr.parse('subtype delay_matrix_type is delay_matrix_type (1 to 4, 1 to 4)'))


    res = parse(open('aaa', encoding='latin1').read())
    @attr.s
    class w:
        d = attr.ib()
    pprint(res['mem_bus_arr_t'])
    #pprint(attr.asdict(w(res)))
