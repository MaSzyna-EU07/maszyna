"""Shared parts of the scripts writing //$e descriptions into the templates of the scenery."""

import os
import tarfile
import time
import zipfile

LINEMARK = '//$e'
BOM = '﻿'


def load(path):
    with open(path, 'rb') as source:  # NOSONAR
        return source.read()


def decode(raw, plain='cp1250'):
    try:
        text = raw.decode('utf-8')
        encoding = 'utf-8' if raw.startswith(b'\xef\xbb\xbf') or max(raw, default=0) > 127 else plain
    except UnicodeDecodeError:
        text = raw.decode('cp1250')
        encoding = 'cp1250'
    return text, encoding


def body_lines(text):
    return [line for line in text.splitlines() if not line.startswith(LINEMARK)]


def rewrite(info):
    if info.get('header') is None:
        return info['raw']
    text = info['text']
    eol = '\r\n' if '\r\n' in text else '\n'
    bom = text.startswith(BOM)
    if bom:
        text = text[1:]
    kept = [line for line in text.split(eol) if not line.startswith(LINEMARK)]
    block = [LINEMARK + ' ' + line for line in info['header']]
    out = eol.join(block + kept)
    return ((BOM if bom else '') + out).encode(info['encoding'])


def options(argv, usage):
    if len(argv) < 2:
        print(usage)
        return None
    pack = argv[argv.index('--zip') + 1] if '--zip' in argv else None
    return argv[1], '--write' in argv, pack


def write(data, folder, name, entries):
    backup = os.path.join(data, 'editor_backup')
    os.makedirs(backup, exist_ok=True)  # NOSONAR
    archive = os.path.join(backup, time.strftime(name + '_%Y%m%d_%H%M%S.tar'))
    root = os.path.join(data, folder)
    with tarfile.open(archive, 'w') as tar:  # NOSONAR
        for file, _ in entries:
            tar.add(os.path.join(root, file), arcname=folder + '/' + file)  # NOSONAR
    for file, info in entries:
        content = rewrite(info)
        if content != info['raw']:
            with open(os.path.join(root, file), 'wb') as out:  # NOSONAR
                out.write(content)
    print('written, originals in', archive)


def pack(target, folder, entries):
    with zipfile.ZipFile(target, 'w', zipfile.ZIP_DEFLATED) as out:  # NOSONAR
        for file, info in entries:
            out.writestr(folder + '/' + file, rewrite(info))
    print('packed into ' + target)

