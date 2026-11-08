#!/usr/bin/env python3
"""Writes //$e descriptions into the signal templates of the scenery directory, so the editor can place them.

usage: signal_headers.py <data root> [--write] [--sidecar]
without --write it only reports what it would write; --sidecar writes them beside the data, in editor/templates, the
templates left as they are
"""

import os
import re
import sys

from editor_headers import LINEMARK, body_lines, decode, load, sidecar, write

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


GUESSED = ('kind', 'mount', 'lamps', 'lean', 'read')


def flow_items(inner):
    """The key: value pieces of a YAML flow map written on one line, its nested maps and quoted strings kept whole."""
    items, depth, quote, start = [], 0, '', 0
    for idx, char in enumerate(inner):
        if quote:
            if char == quote and inner[idx - 1] != '\\':
                quote = ''
        elif char in '"\'':
            quote = char
        elif char in '{[':
            depth += 1
        elif char in '}]':
            depth -= 1
        elif char == ',' and depth == 0:
            items.append(inner[start:idx].strip())
            start = idx + 1
    if inner[start:].strip():
        items.append(inner[start:].strip())
    return items


def kept_of(text, file=''):
    """What a description written before has that the guesses mustn't replace: the keys of its signal map other than the guessed
    ones (arms, aspects, repeats...), its description, its parameters from 6 on, by id. Only a signal map written on one line is
    kept; one written as a block is told of, its keys are lost"""
    lines = [line[len(LINEMARK):].strip() for line in text.splitlines() if line.startswith(LINEMARK)]
    signal, description, params = [], None, {}
    for line in lines:
        if line.startswith('signal:'):
            inner = line[len('signal:'):].strip()
            if inner.startswith('{') and inner.endswith('}'):
                signal = [item for item in flow_items(inner[1:-1]) if item.split(':', 1)[0].strip() not in GUESSED]
            else:
                print('WARNING %s: its signal map isn\'t written on one line as {...}, its keys written by hand aren\'t kept' % (file or '?'), file=sys.stderr)
        elif line.startswith('description:'):
            description = line[len('description:'):].strip()
        elif line.startswith('- {id:'):
            found = re.match(r'- \{id:\s*(\d+)', line)
            if found and int(found.group(1)) >= 6:
                params[int(found.group(1))] = line[2:]
    return signal, description, params


def examine(path, file):
    raw = load(path)
    text, encoding = decode(raw, 'utf-8')
    kept_signal, kept_description, kept_params = kept_of(text, file)
    lines = body_lines(text)
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
        params.append(kept_params.get(6) or ("{id: 6, role: texture, label: Plate, default: 0_nothing}" if 'tabl/(p6)' in body.lower() else "{id: 6, role: text, label: Symbol, default: none}"))
    for id in range(7, count + 1):
        label = 'Linked signal' if id == 7 else 'Parameter %d' % id
        params.append(kept_params.get(id) or "{id: %d, role: text, label: %s, default: none}" % (id, label))

    signal = ['kind: ' + kind, 'mount: ' + mount]
    if lamps:
        signal.append('lamps: ' + lamps)
    if lean:
        signal.append('lean: ' + lean)
    if read:
        signal.append('read: "' + read + '"')
    # the keys written by hand stay: the arms, the aspects, the events a disc repeats
    signal += kept_signal
    header = [
        'version: 1',
        'name: ' + file[:-4],
        'category: signal',
    ]
    if kept_description:
        header.append('description: ' + kept_description)
    elif description:
        header.append('description: "' + description.replace('\\', '\\\\').replace('"', '\\"') + '"')
    header.append('signal: {' + ', '.join(signal) + '}')
    header.append('params:')
    header += ['  - ' + p for p in params]
    return {'raw': raw, 'text': text, 'encoding': encoding, 'header': header, 'kind': kind, 'model': model, 'read': read, 'lamps': lamps, 'mount': mount, 'lean': lean, 'count': count}, ''


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    root = os.path.join(sys.argv[1], 'scenery')
    writing = '--write' in sys.argv
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

    if writing:
        write(sys.argv[1], 'scenery', 'signals', done)
    if '--sidecar' in sys.argv:
        sidecar(sys.argv[1], 'scenery', done)
    return 0


if __name__ == '__main__':
    sys.exit(main())
