#!/usr/bin/python3
"""
Generate vivado component file in /src/component.xml.

Serves to update the list of source files -- run when you add/delete/rename
a src vhdl file.
"""

from jinja2 import Environment, FileSystemLoader, select_autoescape
from pathlib import Path

d = Path(__file__).parent

jinja_env = Environment(
                loader=FileSystemLoader(str(d)),
                autoescape=select_autoescape(['html', 'xml']))

template = jinja_env.get_template('component.xml.j2')

src_dir = d / '..' / 'src'
files = [str(f.relative_to(src_dir)) for f in src_dir.glob('**/*.vhd')]
files = sorted(files)

contents = template.render(files=files)
with (src_dir / 'component.xml').open('wt', encoding='utf-8') as f:
    f.write(contents)
