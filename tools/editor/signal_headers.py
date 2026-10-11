#!/usr/bin/env python3
"""Writes //$e descriptions into the signal templates of the scenery directory, so the editor can place them.

usage: signal_headers.py <data root> [--write]
without --write it only reports what it would write
"""

import os
import re
import sys
import tarfile
import time

FAMILIES = [
    (re.compile(r'^ss[dow]?\d'), 'main'),
    (re.compile(r'^sk(\d|-un)'), 'main'),
    (re.compile(r'^sbl\d'), 'block'),
    (re.compile(r'^(ts\d|tk\d|top|tsosp)'), 'warning'),
    (re.compile(r'^(ps\d|sp3)'), 'repeater'),
    (re.compile(r'^(ms\d|zs2nb)'), 'shunt'),
    (re.compile(r'^(zs1c|zs2cb|tzm|z1)'), 'stop'),
    (re.compile(r'^sz(i|k|y|yn|-ksztalt\d?)\.inc$'), 'substitute'),
]

COLOURS = [('zielon', 'z'), ('czerwon', 'c'), ('pomara', 'p'), ('żółt', 'p'), ('biał', 'b'), ('mleczn', 'b'), ('niebiesk', 'n'), ('ślep', 'x'), ('brak', 'x')]

LINEMARK = '//$e'


def tokens(text):
    text = re.sub(r'//[^\n]*', ' ', text)
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    return re.split(r'[\s;]+', text.lower())


def statements(words, keyword):
    out = []
    idx = 0
    while idx < len(words):
        if words[idx] == keyword:
            if keyword == 'event':
                stop = idx
                while stop < len(words) and words[stop] != 'endevent':
                    stop += 1
            else:
                stop = idx + 10
            out.append(words[idx:stop])
        idx += 1
    return out


def examine(path, file):
    raw = open(path, 'rb').read()  # NOSONAR
    try:
        text = raw.decode('utf-8')
        encoding = 'utf-8'
    except UnicodeDecodeError:
        text = raw.decode('cp1250')
        encoding = 'cp1250'
    lines = [line for line in text.splitlines() if not line.startswith(LINEMARK)]
    body = '\n'.join(lines)
    words = tokens(body)
    count = max([int(m) for m in re.findall(r'\(p(\d+)\)', body)] or [0])

    if 'origin' not in words or words[words.index('origin') + 1:words.index('origin') + 4] != ['(p2)', '(p3)', '(p4)']:
        return None, 'no origin (p2) (p3) (p4)'
    if 'rotate' not in words or words[words.index('rotate') + 1:words.index('rotate') + 4] != ['0', '(p5)', '0']:
        return None, 'no rotate 0 (p5) 0'
    if count < 5:
        return None, 'fewer than 5 parameters'

    models = [n for n in statements(words, 'node') if len(n) > 9 and n[4] == 'model']
    head = next((n for n in models if n[3].startswith('(p1)')), None)
    if head is None:
        return None, 'no model named after (p1)'
    model = head[9]

    cells = {}
    for n in statements(words, 'node'):
        if len(n) > 8 and n[4] == 'memcell':
            cells[n[3]] = n[8]
    read = ''
    for e in statements(words, 'event'):
        velocity = (len(e) > 4 and e[2] == 'getvalues' and cells.get(e[4]) in ('setvelocity', 'shuntvelocity')) or (len(e) > 8 and e[2] == 'putvalues' and e[8] in ('setvelocity', 'shuntvelocity'))
        if velocity and e[1].startswith('(p1)'):
            if not read or e[1].endswith('_sem_info'):
                read = e[1]

    kind = next((k for pattern, k in FAMILIES if pattern.search(file)), None)
    if kind is None:
        return None, 'not a signal family'
    if kind == 'repeater' and cells:
        return None, 'repeater with a memory cell'
    if kind in ('main', 'block', 'shunt', 'stop', 'substitute') and not read and cells:
        return None, 'memory cell which no event reads for the train'
    if kind in ('warning', 'repeater'):
        read = ''

    models_used = [n[9] for n in models]
    lamps = ''
    lean = ''
    m = re.match(r'sem/(?:[a-z]+)_(\d)_([lpyk])_([a-z]+)[_.]', model)
    if m:
        lamps = m.group(3)
        lean = {'l': 'left', 'y': 'right', 'p': '', 'k': ''}[m.group(2)]
    else:
        m = re.match(r'sem/karzelki/k(?:[a-z]*?\d|tm|to|s)([zcpbnx]+)[.+]', model) or re.match(r'sem/glowice/(?:s|to)\d(?:i|y[lp])([zcpbnx]+)_dd\.', model)
        if m:
            lamps = m.group(1)
    if not lamps:
        for line in lines:
            if line.lower().lstrip('/ ').startswith('komory:'):
                for word in line.split(':', 1)[1].split(','):
                    word = word.strip().lower()
                    lamps += next((c for prefix, c in COLOURS if word.startswith(prefix)), '')
                break

    if 'karzelki/' in model or re.search(r'k(w\d+)?(\+.*)?\.inc$', file) or 'pod_betonowa' in body:
        mount = 'dwarf'
    elif '_drb_' in model and not any(('slupy/' in m or 'post' in m or 'drabinki/' in m) for m in models_used):
        mount = 'gantry'
    else:
        mount = 'mast'

    description = ''
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('//'):
            stripped = stripped.lstrip('/').strip()
            if stripped.lower().startswith('typ:'):
                stripped = stripped[4:].strip()
            if stripped:
                description = stripped
                break
        elif stripped:
            break

    said = description.lower()
    if 'w lewo' in said:
        lean = 'left'
    elif 'w prawo' in said:
        lean = 'right'
    elif 'prost' in said:
        lean = ''

    params = [
        "{id: 1, role: name, label: Name, default: A}",
        "{id: 2, role: pos.x}",
        "{id: 3, role: pos.y}",
        "{id: 4, role: pos.z}",
        "{id: 5, role: rot.y}",
    ]
    if count >= 6:
        params.append("{id: 6, role: texture, label: Plate, default: 0_nothing}" if 'tabl/(p6)' in body.lower() else "{id: 6, role: text, label: Symbol, default: none}")
    for id in range(7, count + 1):
        label = 'Linked signal' if id == 7 else 'Parameter %d' % id
        params.append("{id: %d, role: text, label: %s, default: none}" % (id, label))

    signal = ['kind: ' + kind, 'mount: ' + mount]
    if lamps:
        signal.append('lamps: ' + lamps)
    if lean:
        signal.append('lean: ' + lean)
    if read:
        signal.append('read: "' + read + '"')
    header = [
        'version: 1',
        'name: ' + file[:-4],
        'category: signal',
    ]
    if description:
        header.append('description: "' + description.replace('\\', '\\\\').replace('"', '\\"') + '"')
    header.append('signal: {' + ', '.join(signal) + '}')
    header.append('params:')
    header += ['  - ' + p for p in params]
    return {'raw': raw, 'text': text, 'encoding': encoding, 'header': header, 'kind': kind, 'model': model, 'read': read, 'lamps': lamps, 'mount': mount, 'lean': lean, 'count': count}, ''


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
    root = os.path.join(sys.argv[1], 'scenery')
    write = '--write' in sys.argv
    files = sorted(f for f in os.listdir(root) if f.endswith('.inc'))
    files += ['pkp/semafory/' + f for f in sorted(os.listdir(os.path.join(root, 'pkp', 'semafory'))) if f.endswith('.inc')]
    done = []
    skipped = []
    for file in files:
        name = os.path.basename(file).lower()
        if not any(pattern.search(name) for pattern, _ in FAMILIES):
            continue
        info, reason = examine(os.path.join(root, file), name)
        if info is None:
            skipped.append((file, reason))
            continue
        done.append((file, info))

    for file, info in done:
        print('%-34s %-10s %-6s %-6s %-6s p%d %-22s %s' % (file, info['kind'], info['mount'], info['lean'] or '-', info['lamps'] or '-', info['count'], info['read'] or '-', info['model']))
    for file, reason in skipped:
        print('SKIP %-34s %s' % (file, reason))
    kinds = {}
    for _, info in done:
        kinds[info['kind']] = kinds.get(info['kind'], 0) + 1
    print('described: %d %s, skipped: %d' % (len(done), kinds, len(skipped)))

    if write:
        backup = os.path.join(sys.argv[1], 'editor_backup')
        os.makedirs(backup, exist_ok=True)  # NOSONAR
        archive = os.path.join(backup, time.strftime('signals_%Y%m%d_%H%M%S.tar'))
        with tarfile.open(archive, 'w') as tar:  # NOSONAR
            for file, _ in done:
                tar.add(os.path.join(root, file), arcname='scenery/' + file)  # NOSONAR
        for file, info in done:
            content = rewrite(info)
            if content != info['raw']:
                with open(os.path.join(root, file), 'wb') as out:  # NOSONAR
                    out.write(content)
        print('written, originals in', archive)
    return 0


if __name__ == '__main__':
    sys.exit(main())
