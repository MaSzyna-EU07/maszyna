#!/usr/bin/env python3
"""Writes descriptions of the scenery templates beside the game data (editor/templates), from how the sceneries use them:
what each template is (its kind, by its path, its name and the comment at its top), and how it typically stands by the
track (how far from the axis, how high over the rail top), out of the tables the editor writes with the self-test command
"infrascan <corridor> <top> <file>". The templates with a description of their own, or one beside the data already, are
left alone; so are the ones no rule knows.

usage: usage_headers.py <data root> <table>... [--write] [--least <uses>]
without --write it only reports what it would write
"""

import os
import re
import statistics
import sys

from editor_headers import decode, load

# kind of a template by the words of its path, its name and the comment at its top, the first rule which matches
RULES = [
    ('signal', r'semafor|sem_|sem-|tarcz|/tz_|/tm_|powtarz|sygnaliz'),
    ('equipment', r'rezonator|shp|przejazd|plyty|zwrotni|zwrl|zwrp|wzm|-wz_|2-5-13/|6-9-9/|1435mm/|podsypk|kozio|most|wiadukt|przepust|track\.inc|cbp-'),
    ('environment', r'tree|grass|krzak|plants/|posers/|drzew|trawa|/las_|lawka|plot_|wiata|bud/|ludzi|nastawni|budyn'),
    ('sign', r'wskaz|wsk_|wsk-|(^|/)\d?w\d|/we\d|\+we\d|kilometr|hektometr|hekt|slhm|slupek|znak|tablic|napis'),
    ('pole', r'slup|słup|maszt|wysieg|wysięg|bramk|kotw|odciag|stb1|sts1|zwis|tra/sb|tr/-[pl]-'),
    ('catenary', r'trakcj|siec_|siec-|przewod|lina_nosna'),
]
CATEGORY_WORDS = {
    'signal': 'sygnał',
    'environment': 'otoczenie',
    'equipment': 'wyposażenie toru',
    'sign': 'wskaźnik, znak',
    'pole': 'słup, wysięgnik sieci',
    'catenary': 'sieć trakcyjna',
}


def heading(text):
    """the first line of the comment at the top of the template, as its name"""
    for line in text.splitlines()[:12]:
        line = line.strip()
        if not line.startswith('//') or line.startswith('//$'):
            continue
        line = line.lstrip('/').strip(' -=')
        if len(line) >= 6 and not re.match(r'^(p\d|param|autor|author|origin|rotate|node|ra \w+:)', line, re.I):
            return line[:90]
    return ''


def kind_of(path, text):
    """by the path first, it says the most; then by the comment at the top too"""
    for words in (path.lower(), (path + ' ' + '\n'.join(text.splitlines()[:6])).lower()):
        for kind, pattern in RULES:
            if re.search(pattern, words):
                return kind
    return None


def described(data, file):
    template = os.path.join(data, 'scenery', file)
    beside = os.path.join(data, 'editor', 'templates', file)
    if os.path.exists(beside):
        # a description this script wrote may be written again
        with open(beside, encoding='utf-8', errors='replace') as source:
            return 'usage: {' not in source.read()
    try:
        text, _ = decode(load(template))
    except OSError:
        return True
    return any(line.startswith('//$e') for line in text.splitlines()[:40])


def read_tables(tables):
    uses = {}
    for table in tables:
        scenery = os.path.basename(table).replace('use_', '').replace('.tsv', '')
        with open(table, encoding='utf-8') as source:
            next(source)
            for row in source:
                cells = row.rstrip('\n').split('\t')
                if len(cells) < 7 or cells[1] != 'include':
                    continue
                file = cells[0].replace('\\', '/')
                entry = uses.setdefault(file.lower(), {'file': file, 'offsets': [], 'heights': [], 'sceneries': set()})
                entry['offsets'] += [float(v) for v in cells[4].split(',') if v]
                entry['heights'] += [float(v) for v in cells[5].split(',') if v]
                entry['sceneries'].add(scenery)
    return uses


def header(entry, kind, name):
    distances = [abs(v) for v in entry['offsets']]
    offset = statistics.median(distances)
    spread = statistics.pstdev(distances) if len(distances) > 1 else 0.0
    height = statistics.median(entry['heights'])
    count = len(entry['offsets'])
    where = 'na osi toru' if offset < 0.6 else '%.1f m od osi toru' % offset
    sceneries = sorted(entry['sceneries'])
    description = '%s; typowo %s, %.2f m względem główki szyny; użyty %d razy (%s)' % (CATEGORY_WORDS[kind], where, height, count, ', '.join(sceneries))
    lines = [
        'version: 1',
        'name: "%s"' % name.replace('"', "'"),
        'category: %s' % kind,
        'description: "%s"' % description,
        'usage: {count: %d, offset: %.2f, spread: %.2f, height: %.2f, sceneries: [%s]}' % (count, offset, spread, height, ', '.join(sceneries)),
    ]
    return lines


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 1
    data = argv[1]
    tables = [arg for arg in argv[2:] if arg.endswith('.tsv')]
    write = '--write' in argv
    least = int(argv[argv.index('--least') + 1]) if '--least' in argv else 3
    uses = read_tables(tables)
    counts = {}
    written = 0
    unknown = []
    for key, entry in sorted(uses.items(), key=lambda item: -len(item[1]['offsets'])):
        file = entry['file']
        if len(entry['offsets']) < least or described(data, file):
            continue
        try:
            text, _ = decode(load(os.path.join(data, 'scenery', file)))
        except OSError:
            continue
        kind = kind_of(file, text)
        if kind is None:
            unknown.append((len(entry['offsets']), file))
            continue
        name = heading(text) or os.path.splitext(os.path.basename(file))[0]
        counts[kind] = counts.get(kind, 0) + 1
        lines = header(entry, kind, name)
        if write:
            target = os.path.join(data, 'editor', 'templates', file)
            os.makedirs(os.path.dirname(target), exist_ok=True)  # NOSONAR
            with open(target, 'w', encoding='utf-8', newline='\n') as out:  # NOSONAR
                out.write('\n'.join(lines) + '\n')
            written += 1
        else:
            print('%-45s %-12s %s' % (file, kind, lines[3][13:120]))
    print('by kind:', ', '.join('%s %d' % item for item in sorted(counts.items())))
    print('%d descriptions %s beside the data' % (sum(counts.values()), 'written' if write else 'would be written'))
    print('not known to the rules, the most used: ' + ', '.join('%s (%d)' % (file, count) for count, file in sorted(unknown, reverse=True)[:25]))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
