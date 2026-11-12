#!/usr/bin/env python3
"""Writes //$e descriptions into the indicators of the signals in scenery/pkp/wskazniki, and makes there, as copies, the
indicators which so far came only built into the signal templates: W1 of the warning discs and of the block signals, W18 of
the block signals and the I, II and III plates of the repeaters. The templates keep theirs: one of these put on the template
it was copied from shows twice. The editor puts them on a signal it places.

usage: indicator_headers.py <data root> [--write] [--sidecar]
without --write it only reports what it would write; --sidecar writes the descriptions of the lit ones beside the data, in
editor/templates, the files of the game left as they are. The ones made are new files either way, a replaced one is kept in
editor_backup first
"""

import os
import re
import sys
import tarfile
import time

from editor_headers import LINEMARK, decode, load, sidecar, write

FOLDER = 'pkp/wskazniki'

PARAMS = [
    'params:',
    '  - {id: 1, role: link, label: Signal}',
    '  - {id: 2, role: pos.x}',
    '  - {id: 3, role: pos.y}',
    '  - {id: 4, role: pos.z}',
    '  - {id: 5, role: rot.y}',
    '  - {id: 6, role: rot.x, default: 0}',
    '  - {id: 7, role: rot.z, default: 0}',
]

# the lit indicators under the head of the signal: the file, the sign, the name and what it says
LIT = [
    ('w19led.inc', 'W19', 'W19 ledowy', 'Wskaźnik W19 ledowy, na maszcie semafora pod komorami.'),
    ('w20led.inc', 'W20', 'W20 ledowy', 'Wskaźnik W20 ledowy (jazda po torze lewym), na maszcie semafora pod komorami.'),
    ('w21xled.inc', 'W21', 'W21 ledowy jednocyfrowy', 'Wskaźnik W21 ledowy jednocyfrowy (50-90 km/h), na maszcie semafora pod komorami. Każda jego prędkość wpisuje się do komórki semafora, (p1) to nazwa semafora.'),
    ('w21xyled.inc', 'W21', 'W21 ledowy dwucyfrowy', 'Wskaźnik W21 ledowy dwucyfrowy, na maszcie semafora pod komorami. Każda jego prędkość wpisuje się do komórki semafora, (p1) to nazwa semafora.'),
    ('w24led.inc', 'W24', 'W24 ledowy', 'Wskaźnik W24 ledowy (kierunku), na maszcie semafora pod komorami.'),
    ('w26led.inc', 'W26', 'W26 ledowy', 'Wskaźnik W26 ledowy (P albo D), na maszcie semafora pod komorami.'),
]

# how far the plates fixed flat to the mast go ahead of where their models put them: their back is 5 cm from the axis of the
# mast, the mast is 6.1 cm thick and its clamp under them 8.3 cm, so they went through the plate, in the templates too
AHEAD = '0.04'

# the indicators taken out of the signal templates: the model has its place by the foot of the signal built in.
# the file made, the sign, the name, the template it comes from, the model with its offset and textures, the node name suffix
MADE = [
    ('w1_nowy.inc', 'W1', 'W1 nowy, tarcza', 'ts1pi.inc', 'model 0 0 0.1 0 sem/wskazniki/w1new_d.t3d sem/slupy endmodel', 'w1',
     'Wskaźnik W1 nowego typu na maszcie tarczy ostrzegawczej.'),
    ('w1_nowy_mz.inc', 'W1', 'W1 nowy, tarcza z pasem', 'ts1py+1sp.inc', 'model 0 0 0 0 sem/wskazniki/w1new_mz.t3d sem/slupy endmodel', 'w1',
     'Wskaźnik W1 nowego typu na maszcie tarczy ostrzegawczej z pasem świetlnym, niżej niż zwykły.'),
    ('w1_stary.inc', 'W1', 'W1 stary, tarcza', 'ts1py+2sp.inc', 'model 0 0 0.1 0 sem/wskazniki/w1old_d.t3d sem/slupy endmodel', 'w1',
     'Wskaźnik W1 starego typu na maszcie tarczy ostrzegawczej.'),
    ('w1_sbl.inc', 'W1', 'W1 na SBL', 'sbl4pw1.inc', 'model 0 0 0.1 0 pkp/w1_sbl.t3d pkp/w1 endmodel', 'w1',
     'Wskaźnik W1 na maszcie semafora SBL, przedostatniego przed wjazdowym.'),
    ('w18.inc', 'W18', 'W18 na SBL', 'sbl3w18.inc', 'model 0 0 %s 0 sem/wskazniki/w18new.t3d sem/slupy endmodel' % AHEAD, 'w18',
     'Wskaźnik W18 nowego typu na maszcie semafora SBL, ostatniego przed wjazdowym.'),
    ('w18_stary.inc', 'W18', 'W18 stary na SBL', 'sbl3yw18u.inc', 'model 0 0 %s 0 sem/wskazniki/w18old.t3d sem/slupy endmodel' % AHEAD, 'w18',
     'Wskaźnik W18 starego typu na maszcie semafora SBL, ostatniego przed wjazdowym.'),
    ('wsk_i.inc', 'I', 'Tablica I', 'ps2pbi.inc', 'model 0 0 %s 0 sem/wskazniki/isp.t3d sem/slupy endmodel' % AHEAD, 'i',
     'Tablica I na maszcie powtarzacza, pierwszego od semafora.'),
    ('wsk_ii.inc', 'II', 'Tablica II', 'ps2pbii.inc', 'model 0 0 %s 0 sem/wskazniki/iisp.t3d sem/slupy endmodel' % AHEAD, 'ii',
     'Tablica II na maszcie powtarzacza, drugiego od semafora.'),
    ('wsk_iii.inc', 'III', 'Tablica III', 'ps2pbiii.inc', 'model 0 0 %s 0 sem/wskazniki/iiisp.t3d sem/slupy endmodel' % AHEAD, 'iii',
     'Tablica III na maszcie powtarzacza, trzeciego od semafora.'),
]

EVENT = re.compile(r'^\s*event\s+\(p1\)_(\S+)\s+(\S+)', re.I | re.M)


def quoted(text):
    return '"' + text.replace('\\', '\\\\').replace('"', '\\"') + '"'


def lit_events(text):
    """the events the indicator is shown by, by name: on, off, a letter or a speed; the parts of a multiple one left out"""
    events = {}
    for name, kind in EVENT.findall(text):
        low = name.lower()
        if low.endswith('_off') or low.endswith('off'):
            events['off'] = name
        elif kind.lower() == 'multiple' and re.search(r'_(\d+)$', low):
            events[re.search(r'_(\d+)$', low).group(1)] = name
        elif kind.lower() == 'lights' and re.search(r'_([a-z])$', low) and not re.search(r'_w\d+$', low):
            events[low[-1].upper()] = name
        elif kind.lower() == 'lights' and re.search(r'^w\d+$', low):
            events['on'] = name
    # shown first, then the letters and the speeds from the lowest, the one putting it out last
    order = lambda key: (0, 0, '') if key == 'on' else (3, 0, '') if key == 'off' else (2, int(key), '') if key.isdigit() else (1, 0, key)
    return {key: events[key] for key in sorted(events, key=order)}


def header(sign, name, description, place, events=None):
    lines = ['version: 1', 'name: ' + name, 'category: indicator', 'description: ' + quoted(description)]
    entry = '{sign: %s, place: %s' % (sign, place)
    if events:
        entry += ', events: {' + ', '.join('%s: %s' % (key, value) for key, value in events.items()) + '}'
    lines.append('indicator: ' + entry + '}')
    return lines + PARAMS


def made_text(entry, eol):
    file, sign, name, source, model, suffix, description = entry
    body = [
        '//-- %s skopiowany z %s przez tools/editor/indicator_headers.py: model ma swoje położenie względem stopy semafora' % (name, source),
        '//-- p1=nazwa semafora, p2,p3,p4=położenie jak semafora, p5=obrót jak semafora, p6,p7=pochylenie (0)',
        'origin (p2) (p3) (p4)',
        'rotate (p6) (p5) (p7)',
        'node 400 0 (p1)_%s %s' % (suffix, model),
        'rotate 0 0 0',
        'endorigin',
    ]
    block = [LINEMARK + ' ' + line for line in header(sign, name, description + ' Bez eventów, model stawia edytor przy semaforze.', 'foot')]
    return eol.join(block + body) + eol


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    root = os.path.join(sys.argv[1], 'scenery', FOLDER)
    writing = '--write' in sys.argv
    described = []
    for file, sign, name, description in LIT:
        path = os.path.join(root, file)
        if not os.path.isfile(path):
            print('SKIP %-18s missing' % file)
            continue
        raw = load(path)
        text, encoding = decode(raw)
        events = lit_events(text)
        if 'off' not in events or len(events) < 2:
            print('SKIP %-18s events not found: %s' % (file, events))
            continue
        lines = header(sign, name, description + ' Gaszony eventem (p1)_%s.' % events['off'], 'head', events)
        described.append((file, {'raw': raw, 'text': text, 'encoding': encoding, 'header': lines}))
        print('%-18s %-4s head %s' % (file, sign, ', '.join('%s=%s' % item for item in events.items())))
    made = []
    for entry in MADE:
        path = os.path.join(root, entry[0])
        content = made_text(entry, '\r\n').encode('cp1250')
        state = 'new' if not os.path.isfile(path) else ('same' if load(path) == content else 'changed')
        made.append((entry[0], content, state))
        print('%-18s %-4s foot %s (%s)' % (entry[0], entry[1], entry[4].split()[5], state))
    print('described: %d, made: %d' % (len(described), len([m for m in made if m[2] != 'same'])))
    if '--sidecar' in sys.argv:
        sidecar(sys.argv[1], os.path.join('scenery', FOLDER), described)
    elif writing and described:
        write(sys.argv[1], os.path.join('scenery', FOLDER), 'indicators', described)
    if writing or '--sidecar' in sys.argv:
        write_made(sys.argv[1], root, made)
    return 0


def write_made(data, root, made):
    """the indicators made, the ones there with other content kept in editor_backup first"""
    replaced = [file for file, _, state in made if state == 'changed']
    if replaced:
        backup = os.path.join(data, 'editor_backup')
        os.makedirs(backup, exist_ok=True)  # NOSONAR
        archive = os.path.join(backup, time.strftime('indicators_made_%Y%m%d_%H%M%S.tar'))
        with tarfile.open(archive, 'w') as tar:  # NOSONAR
            for file in replaced:
                tar.add(os.path.join(root, file), arcname=FOLDER + '/' + file)  # NOSONAR
        print('replaced ones kept in', archive)
    for file, content, state in made:
        if state == 'same':
            continue
        if state == 'changed':
            print('REPLACED', file)
        with open(os.path.join(root, file), 'wb') as out:  # NOSONAR
            out.write(content)


if __name__ == '__main__':
    sys.exit(main())
