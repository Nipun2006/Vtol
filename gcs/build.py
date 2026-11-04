#!/usr/bin/env python3
"""Bundle the ground station into one self-contained file: ../gcs.html.

Usage: python3 gcs/build.py
Every /*@include path*/ marker in src/gcs.html is replaced by that file's contents
(path relative to src/), so the output needs no other files and no internet.
"""
import pathlib
import re

SRC = pathlib.Path(__file__).parent / 'src'
OUT = pathlib.Path(__file__).parent.parent / 'gcs.html'


def include(match):
    path = (SRC / match.group(1)).resolve()
    text = path.read_text()
    # A literal "</script" or "</style" inside an inlined file would end the tag early.
    return text.replace('</script', '<\\/script').replace('</style', '<\\/style')


html = re.sub(r'/\*@include ([^*]+?)\*/', include, (SRC / 'gcs.html').read_text())
OUT.write_text(html)
print(f'wrote {OUT} ({len(html.encode()) // 1024} KB)')
