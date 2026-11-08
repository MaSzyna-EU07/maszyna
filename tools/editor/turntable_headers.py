#!/usr/bin/env python3
"""Writes //$e descriptions into the turntable templates of the scenery, so the editor places them as turntables.

usage: turntable_headers.py <data root> [--write] [--zip <file>] [--sidecar]
without --write it only reports what it would write; --zip packs every described turntable template with its scenery/ path
"""

import os
import re
import sys

from editor_headers import LINEMARK, body_lines, decode, load, options, pack, sidecar, write

FOLDER = 'scenery'
ROTATION = re.compile(r'^event\s+\S*tor(\d+)rot\s+animation\s+\S+\s+\S+\s+rotate\s+\S+\s+\S+\s+\S+\s+(\S+)', re.I | re.M)
MODEL = re.compile(r'model\s+0\s+0\s+0\s+0\s+\S*?obrotnica(\d+)m\.t3d\s+\(p6\)', re.I)


def examine(path, relative):
    raw = load(path)
    if re.search(rb'tor\d+rot\s+animation', raw, re.I) is None:
        return None, None
    text, encoding = decode(raw)
    marks = [line for line in text.splitlines() if line.startswith(LINEMARK)]
    if any('category: turntable' in line for line in marks):
        return {'raw': raw, 'header': None, 'described': True}, ''
    body = '\n'.join(body_lines(text))
    lower = body.lower()
    if 'origin (p2) (p3) (p4)' not in lower or 'rotate 0 (p5) 0' not in lower:
        return None, 'not origin (p2) (p3) (p4) rotate 0 (p5) 0'
    model = MODEL.search(body)
    if model is None:
        return None, 'no obrotnica<length>m.t3d model with surface (p6)'
    positions = {}
    for number, angle in ROTATION.findall(body):
        positions.setdefault(int(number), float(angle))
    if not positions:
        return None, 'no positions'
    count = max(int(n) for n in re.findall(r'\(p(\d+)\)', lower))
    if count != 6:
        return None, 'uses %d parameters, not 6' % count
    length = int(model.group(1))
    angles = [positions[n] for n in sorted(positions)]
    folder, file = os.path.split(relative)
    name = 'obrotnica %d m, %d %s' % (length, len(angles), 'stałe położenia' if len(angles) < 5 else 'stałych położeń')
    if folder != 'eng':
        name += ' (%s)' % os.path.splitext(file)[0]
    description = 'Obrotnica %d m o stałych położeniach mostu: %s' % (length, ' '.join('%g' % a for a in angles))
    header = [
        'version: 1',
        'name: ' + name,
        'category: turntable',
        'description: "' + description + '"',
        'turntable: {length: %d}' % length,
        'params:',
        '  - {id: 1, role: track, label: Name}',
        '  - {id: 2, role: pos.x}',
        '  - {id: 3, role: pos.y}',
        '  - {id: 4, role: pos.z}',
        '  - {id: 5, role: rot.y}',
        '  - {id: 6, label: Surface, default: eng/most1-jezdniablach}',
    ]
    return {'raw': raw, 'text': text, 'encoding': encoding, 'header': header, 'described': False, 'length': length, 'angles': angles}, ''


def main():
    chosen = options(sys.argv, __doc__)
    if chosen is None:
        return 1
    data, writing, target = chosen
    folder = os.path.join(data, FOLDER)
    done = []
    skipped = []
    for root, _, files in os.walk(folder):
        for file in sorted(files):
            if not file.lower().endswith('.inc'):
                continue
            relative = os.path.relpath(os.path.join(root, file), folder).replace(os.sep, '/')
            info, reason = examine(os.path.join(root, file), relative)
            if info is not None:
                done.append((relative, info))
            elif reason:
                skipped.append((relative, reason))
    done.sort()
    for file, info in done:
        if info['described']:
            print('%-40s already described' % file)
        else:
            print('%-40s %d m, positions %s' % (file, info['length'], ' '.join('%g' % a for a in info['angles'])))
    for file, reason in skipped:
        print('skipped %s: %s' % (file, reason))
    fresh = [(file, info) for file, info in done if not info['described']]
    print('%d described now, %d already described, %d skipped' % (len(fresh), len(done) - len(fresh), len(skipped)))
    if writing and fresh:
        write(data, FOLDER, 'turntables', fresh)
    if target:
        pack(target, FOLDER, done)
    if '--sidecar' in sys.argv:
        sidecar(data, FOLDER, done)
    return 0


if __name__ == '__main__':
    sys.exit(main())
