#!/usr/bin/env python3
"""Writes //$e descriptions into the switch drive templates (scenery/tory/napedy), so the editor places them with a switch.

usage: drive_headers.py <data root> [--write] [--zip <file>] [--sidecar]
without --write it only reports what it would write; --zip packs the written templates with their scenery/ paths
"""

import os
import re
import sys

from editor_headers import body_lines, decode, load, options, pack, sidecar, write

FOLDER = 'scenery/tory/napedy'
PATTERN = re.compile(r'^(ele1?|mech[12]?)-rz([lp])-([lp])((?:_[a-z]+)*)\.inc$')
KINDS = {'ele': 'elektryczny', 'mech': 'mechaniczny'}
DEFAULT = 'ele'
HEIGHTS = {'ele': 0.05, 'ele1': 0.05}


def examine(path, file):
    m = PATTERN.match(file)
    if m is None:
        return None, 'not a drive of a single switch'
    raw = load(path)
    text, encoding = decode(raw)
    body = '\n'.join(body_lines(text)).lower()
    count = max([int(n) for n in re.findall(r'\(p(\d+)\)', body)] or [0])
    if count != 7 or 'origin (p2) (p3) (p4)' not in body or 'rotate (p6) (p5) (p7)' not in body:
        return None, 'not origin (p2) (p3) (p4) rotate (p6) (p5) (p7)'

    model, hand, side, extras = m.groups()
    family = model + extras
    hand = 'left' if hand == 'l' else 'right'
    side = 'left' if side == 'l' else 'right'
    kind = KINDS[model.rstrip('12')]
    words = ' '.join(w.replace('_', '') for w in extras.split('_') if w)
    description = 'Napęd %s %s%s' % (kind, model, (' (' + words + ')') if words else '')
    description += ', rozjazd %s, napęd z %s strony' % ('lewy' if hand == 'left' else 'prawy', 'lewej' if side == 'left' else 'prawej')
    header = [
        'version: 1',
        'name: ' + family,
        'category: switch drive',
        'description: "' + description + '"',
        'switch: {hand: %s, drive: %s, height: %s%s}' % (hand, side, HEIGHTS.get(family, 0), ', default: true' if family == DEFAULT else ''),
        'params:',
        '  - {id: 1, role: track, label: Switch}',
        '  - {id: 2, role: pos.x}',
        '  - {id: 3, role: pos.y}',
        '  - {id: 4, role: pos.z}',
        '  - {id: 5, role: rot.y}',
        '  - {id: 6, role: rot.x}',
        '  - {id: 7, role: rot.z}',
    ]
    return {'raw': raw, 'text': text, 'encoding': encoding, 'header': header, 'family': family, 'hand': hand, 'side': side}, ''


def main():
    chosen = options(sys.argv, __doc__)
    if chosen is None:
        return 1
    data, writing, target = chosen
    folder = os.path.join(data, FOLDER)
    done = []
    skipped = []
    for file in sorted(os.listdir(folder)):
        if not file.endswith('.inc'):
            continue
        info, reason = examine(os.path.join(folder, file), file)
        if info is None:
            skipped.append((file, reason))
            continue
        done.append((file, info))
    for file, info in done:
        print('%-55s %-30s %-5s drive %s' % (file, info['family'], info['hand'], info['side']))
    for file, reason in skipped:
        print('skipped %s: %s' % (file, reason))
    print('%d described, %d skipped' % (len(done), len(skipped)))
    if writing:
        write(data, FOLDER, 'drives', done)
    if '--sidecar' in sys.argv:
        sidecar(data, FOLDER, done)
    if target:
        pack(target, FOLDER, done)
    return 0


if __name__ == '__main__':
    sys.exit(main())
