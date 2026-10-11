#!/usr/bin/env python3
"""Writes //$e descriptions into the switch drive templates (scenery/tory/napedy), so the editor places them with a switch.

usage: drive_headers.py <data root> [--write] [--zip <file>]
without --write it only reports what it would write; --zip packs the written templates with their scenery/ paths
"""

import os
import re
import sys
import tarfile
import time
import zipfile

LINEMARK = '//$e'
FOLDER = 'scenery/tory/napedy'
PATTERN = re.compile(r'^(ele1?|mech[12]?)-rz([lp])-([lp])((?:_[a-z]+)*)\.inc$')
KINDS = {'ele': 'elektryczny', 'mech': 'mechaniczny'}


def examine(path, file):
    m = PATTERN.match(file)
    if m is None:
        return None, 'not a drive of a single switch'
    raw = open(path, 'rb').read()  # NOSONAR
    try:
        text = raw.decode('utf-8')
        encoding = 'utf-8' if raw.startswith(b'\xef\xbb\xbf') or max(raw, default=0) > 127 else 'cp1250'
    except UnicodeDecodeError:
        text = raw.decode('cp1250')
        encoding = 'cp1250'
    lines = [line for line in text.splitlines() if not line.startswith(LINEMARK)]
    body = '\n'.join(lines).lower()
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
        'switch: {hand: %s, drive: %s}' % (hand, side),
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


def rewrite(info):
    text = info['text']
    eol = '\r\n' if '\r\n' in text else '\n'
    bom = text.startswith('﻿')
    if bom:
        text = text[1:]
    kept = [line for line in text.split(eol) if not line.startswith(LINEMARK)]
    block = [LINEMARK + ' ' + line for line in info['header']]
    out = eol.join(block + kept)
    return (('﻿' if bom else '') + out).encode(info['encoding'])


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    data = sys.argv[1]
    folder = os.path.join(data, FOLDER)
    write = '--write' in sys.argv
    pack = sys.argv[sys.argv.index('--zip') + 1] if '--zip' in sys.argv else None
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
    if write:
        backup = os.path.join(data, 'editor_backup')
        os.makedirs(backup, exist_ok=True)
        archive = os.path.join(backup, 'drives_%s.tar' % time.strftime('%Y%m%d_%H%M%S'))
        with tarfile.open(archive, 'w') as tar:
            for file, _ in done:
                tar.add(os.path.join(folder, file), arcname=FOLDER + '/' + file)
        for file, info in done:
            with open(os.path.join(folder, file), 'wb') as out:
                out.write(rewrite(info))
        print('written, originals in ' + archive)
    if pack:
        with zipfile.ZipFile(pack, 'w', zipfile.ZIP_DEFLATED) as out:
            for file, info in done:
                out.writestr(FOLDER + '/' + file, rewrite(info))
        print('packed into ' + pack)
    return 0


if __name__ == '__main__':
    sys.exit(main())
