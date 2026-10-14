#!/usr/bin/env python3
"""Checks .mat files and their textures against the material norm of the current shaders.

The norm (see tools/materials/README.md):
  specgloss R * param_specular  dielectric 0.02-0.12 (0.06), metal 0.7-1.5 (1.0)
  specgloss G                   1 - perceptual roughness (param_glossiness cancels out when >= 1)
  specgloss B                   metalness, 0 or 1; a metal takes its colour from the diffuse texture
  normalmap A                   reflectivity, 1 where the surface reflects, lower only in cavities
  param_specular                1, the strength of the highlight lives in R

usage: validate_materials.py GAMEDIR [SUBDIR ...] [--fallback DIR ...] [--shaders DIR] [--csv FILE] [--info] [-j N]
"""

import argparse
import csv
import os
import re
import sys

import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None

SIZE = 512

DIELECTRIC_SPEC = (0.02, 0.12)
METAL_SPEC = (0.7, 1.5)
METAL_ALBEDO_MIN = 0.2
METAL_REFLECTIVITY_MIN = 0.5
MIRROR_GLOSS = 0.9
NOISE_MAX = 0.06
ALBEDO_SRGB = (20, 245)
PARAM_SPECULAR_MAX = 2.0

ERROR, WARN, INFO = 'BLAD', 'UWAGA', 'info'


def srgb_to_linear(c):
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def blur(a, r):
    k = 2 * r + 1
    p = np.pad(a, r, mode='edge')
    c = np.cumsum(np.cumsum(p, 0), 1)
    c = np.pad(c, ((1, 0), (1, 0)))
    return (c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]) / (k * k)


class Shaders:
    def __init__(self, directory):
        self.directory = directory
        self.cache = {}
        self.metal_fix = 'metal_dark' in self.source('light_common.glsl')

    def source(self, name):
        path = os.path.join(self.directory, name)
        if not os.path.isfile(path):
            return ''
        text = open(path, encoding='utf-8', errors='replace').read()
        text = re.sub(r'/\*.*?\*/', '', strip_comments(text), flags=re.S)
        for include in re.findall(r'#include\s*<(mat_[^>]+)>', text):
            text += '\n' + self.source(include)
        return text

    def get(self, name):
        name = name.lower()
        if name in self.cache:
            return self.cache[name]
        text = self.source('mat_' + name + '.frag')
        info = None
        if text:
            textures = {m[0]: int(m[1]) for m in re.findall(r'#texture\s*\(\s*(\w+)\s*,\s*(\d+)', text)}
            params = {m[0]: m[1] for m in re.findall(r'#param\s*\(\s*(\w+)\s*,\s*\d+\s*,\s*\d+\s*,\s*\d+\s*,\s*(\w+)', text)}
            gloss = re.search(r'\bglossiness\s*=\s*([^;]+);', text)
            spec = re.search(r'\bspecularity\s*=\s*([^;]+);', text)
            call = re.search(r'apply_lights\s*\(([^;]+)\);', text)
            gloss = gloss.group(1) if gloss else ''
            spec = spec.group(1) if spec else ''
            if call and re.search(r',\s*0\.0\s*,\s*[\w.]+\s*$', call.group(1)):
                spec = '0.0'
            info = {
                'textures': textures,
                'reflection_default': params.get('reflection', 'one'),
                'gloss': 'specgloss' if 'specgloss' in gloss else ('phong' if 'phong_glossiness' in gloss else ('param' if 'param' in gloss else 'unset')),
                'spec': 'specgloss' if 'specgloss' in spec else ('diffuse' if 'tex_color' in spec else ('none' if spec.strip() in ('', '0.0') else 'const')),
                'metal': bool(re.search(r'metalic\s*=\s*[^;]*specgloss', text)),
                'reflectivity': 'normalmap' if re.search(r'reflectivity\s*=[^;]*normal', text) else ('reflmap' if re.search(r'reflectivity\s*=[^;]*reflmap', text) else 'param'),
            }
        self.cache[name] = info
        return info


def strip_comments(text):
    return re.sub(r'//[^\n]*', '', text)


def parse_mat(path):
    text = strip_comments(open(path, encoding='cp1250', errors='replace').read())
    values = {}
    depth = 0
    nested = False
    for line in text.splitlines():
        keys = list(re.finditer(r'(?:(?<=\s)|^)([A-Za-z_][A-Za-z_0-9]*):', line))
        for i, m in enumerate(keys):
            end = keys[i + 1].start() if i + 1 < len(keys) else len(line)
            value = line[m.end():end].strip()
            if depth == 0 and not value.startswith('{'):
                values.setdefault(m.group(1).lower(), value)
            if value.startswith('{'):
                nested = True
        depth += line.count('{') - line.count('}')
        depth = max(depth, 0)
    return values, nested


def first_of_set(value):
    value = value.strip()
    if value.startswith('['):
        value = value[1:].split(']')[0]
    parts = value.split()
    return parts[0] if parts else ''


def number(value):
    m = re.search(r'-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', value or '')
    return float(m.group(0)) if m else None


class Resolver:
    def __init__(self, roots):
        self.roots = roots
        self.listings = {}

    def exists(self, path):
        if os.path.isfile(path):
            return path
        directory, name = os.path.split(path)
        if directory not in self.listings:
            try:
                self.listings[directory] = {f.lower(): f for f in os.listdir(directory)}
            except OSError:
                self.listings[directory] = {}
        found = self.listings[directory].get(name.lower())
        return os.path.join(directory, found) if found else None

    def texture(self, ref, matdir):
        ref = first_of_set(ref).replace('\\', '/').split('|')[0]
        ref = ref.split(':')[0].strip().lstrip('/')
        if not ref or ref.lower() in ('none', '-'):
            return None
        bases = []
        directory = matdir
        for root in self.roots:
            if directory.startswith(root):
                while directory.startswith(root) and directory != root:
                    bases.append(directory)
                    directory = os.path.dirname(directory)
        bases += self.roots + [os.path.join(r, 'textures') for r in self.roots]
        stem, ext = os.path.splitext(ref)
        names = [ref] if ext.lower() in ('.tga', '.dds', '.png', '.bmp', '.jpg', '.ktx') else []
        names += [ref + e for e in ('.tga', '.png', '.bmp', '.jpg', '.dds')]
        for base in bases:
            for name in names:
                found = self.exists(os.path.join(base, name))
                if found:
                    return found
        return None


class Images:
    def __init__(self):
        self.cache = {}

    def get(self, path):
        if path in self.cache:
            return self.cache[path]
        result = None
        try:
            image = Image.open(path)
            image.load()
            if image.mode != 'RGBA':
                image = image.convert('RGBA')
            channels = [np.asarray(c.resize((SIZE, SIZE), Image.BOX), dtype=np.float32) / 255 for c in image.split()]
            result = np.dstack(channels)
        except Exception as e:
            result = str(e)
        self.cache[path] = result
        return result


def pct(a, q):
    return float(np.percentile(a, q)) if a.size else 0.0


def check(mat, values, nested, shaders, resolver, images, rel):
    findings = []

    def add(level, text):
        findings.append((level, text))

    matdir = os.path.dirname(mat)
    refs = {}
    for key, value in values.items():
        if key.startswith('texture') and not key.endswith('_variants'):
            refs[key[len('texture'):].lstrip('_')] = value

    shadername = first_of_set(values.get('shader', ''))
    explicit = bool(shadername)
    if not explicit:
        slots = set()
        for name in refs:
            if name.isdigit():
                slots.add(int(name) - 1)
            elif name in ('diffuse',):
                slots.add(0)
            elif name in ('normals', 'normalmap'):
                slots.add(1)
        shadername = 'default_0' if 0 not in slots else ('default_1' if 1 not in slots else ('default_2' if 2 not in slots else 'default_2'))
    shader = shaders.get(shadername)
    if shader is None:
        add(ERROR, 'nieznany shader "%s"' % shadername)
        return shadername, findings

    bound = {}
    names_by_slot = {v: k for k, v in shader['textures'].items()}
    for name, value in refs.items():
        if name.isdigit():
            name = names_by_slot.get(int(name) - 1, name)
        elif name == 'normals':
            name = 'normalmap'
        bound[name] = value

    textures = {}
    for name, value in bound.items():
        if name not in shader['textures'] or first_of_set(value).strip('/').lower() in ('', 'none', '-'):
            continue
        path = resolver.texture(value, matdir)
        if path is None:
            add(ERROR, 'brak tekstury %s "%s"' % (name, first_of_set(value)))
        else:
            textures[name] = path

    ps = number(values.get('param_specular'))
    pg = number(values.get('param_glossiness'))
    if pg is None:
        pg = number(values.get('glossiness'))
    pg_effective = abs(pg) if pg is not None else 10.0
    pr = number(values.get('param_reflection'))
    if pr is None:
        pr = 1.0 if shader['reflection_default'] == 'one' else 0.0

    if ps is None and shader['spec'] != 'none':
        add(INFO, 'param_specular nie ustawiony, bierze jasnosc specular submodelu z e3d/t3d')
    if ps is not None and ps > PARAM_SPECULAR_MAX and shadername.lower().startswith('water'):
        add(INFO, 'param_specular %g w shaderze %s: odblask slonca na lustrze wody, poza norma' % (ps, shadername))
    elif ps is not None and ps > PARAM_SPECULAR_MAX:
        add(ERROR if ps > 2 * PARAM_SPECULAR_MAX else WARN,
            'param_specular %g mnozy polysk %gx (norma 1, sile niesie R; w starszych shaderach rozjasnia tez w reflektorach)' % (ps, ps))
    psv = ps if ps is not None else 1.0

    def image(name):
        if name not in textures:
            return None
        data = images.get(textures[name])
        if isinstance(data, str):
            add(ERROR, 'nie da sie wczytac %s: %s' % (os.path.basename(textures[name]), data))
            return None
        return data

    diffuse = image('diffuse')
    opaque = None
    if diffuse is not None:
        opaque = diffuse[..., 3] > 0.5
        if not opaque.any():
            opaque = np.ones(diffuse.shape[:2], bool)

    if shader['gloss'] == 'specgloss':
        sg = image('specgloss')
        if sg is not None:
            mask = opaque if opaque is not None else np.ones(sg.shape[:2], bool)
            R, G, B = sg[..., 0], sg[..., 1], sg[..., 2]
            metal = (B >= 0.5) & mask
            dielectric = (B < 0.5) & mask
            if pg is not None and abs(pg) < 1:
                add(WARN, 'param_glossiness %g < 1 zmienia roughness na 1 - G*%g' % (pg, abs(pg)))
            if dielectric.mean() > 0.02:
                e = R[dielectric] * psv
                p50, p95 = pct(e, 50), pct(e, 95)
                if p50 > DIELECTRIC_SPEC[1] * 4:
                    add(ERROR, 'polysk niemetalu R*param_specular %.2f (norma %.2f-%.2f), slonce wypala' % (p50, *DIELECTRIC_SPEC))
                elif p50 > DIELECTRIC_SPEC[1]:
                    add(WARN, 'polysk niemetalu R*param_specular %.2f (norma %.2f-%.2f)' % (p50, *DIELECTRIC_SPEC))
                elif p95 > DIELECTRIC_SPEC[1] * 4:
                    add(WARN, 'czesc niemetalu ma R*param_specular %.2f (95. percentyl)' % p95)
                g = G[dielectric]
                mirror = float((g > MIRROR_GLOSS).mean())
                if mirror > 0.5:
                    add(WARN, 'G > %.1f na %d%% niemetalu: lustro, dobre tylko dla szkla i swiezego lakieru' % (MIRROR_GLOSS, mirror * 100))
                noise = float(np.mean((np.abs(G - blur(G, 2)) + np.abs(R - blur(R, 2)))[dielectric]))
                if noise > NOISE_MAX:
                    add(WARN, 'szum w R/G %.3f: polysk iskrzy plamkami' % noise)
            if metal.mean() > 0.01:
                e = R[metal] * psv
                p50 = pct(e, 50)
                if p50 < METAL_SPEC[0] or p50 > METAL_SPEC[1]:
                    add(WARN, 'polysk metalu R*param_specular %.2f (norma %.1f-%.1f)' % (p50, *METAL_SPEC))
                if diffuse is not None:
                    albedo = srgb_to_linear(diffuse[..., :3]) @ np.array([0.2126, 0.7152, 0.0722])
                    a50 = pct(albedo[metal], 50)
                    if a50 < METAL_ALBEDO_MIN and shaders.metal_fix:
                        add(WARN, 'metal z ciemnym diffuse (liniowo %.2f, norma >= %.1f): shader liczy go jak lakier z R 0.06; metal - rozjasnij diffuse, lakier - ustaw B = 0' % (a50, METAL_ALBEDO_MIN))
                    elif a50 < METAL_ALBEDO_MIN:
                        add(ERROR if a50 < METAL_ALBEDO_MIN / 2 else WARN,
                            'metal z ciemnym diffuse (liniowo %.2f, norma >= %.1f): wyjdzie czarny' % (a50, METAL_ALBEDO_MIN))
                normal = image('normalmap') if shader['reflectivity'] == 'normalmap' else None
                if normal is not None:
                    reflect = float(np.median(normal[..., 3][metal])) * pr
                    if shaders.metal_fix:
                        reflect = max(reflect, 0.5)
                    if reflect < METAL_REFLECTIVITY_MIN:
                        add(WARN, 'metal z odbiciem %.2f (alfa normalmapy * param_reflection, norma >= %.1f): bez otoczenia jest ciemny' % (reflect, METAL_REFLECTIVITY_MIN))
            middle = ((B > 0.15) & (B < 0.85) & mask)
            anymetal = ((B > 0.15) & mask)
            if anymetal.sum() > 0.01 * mask.sum() and middle.sum() > 0.3 * anymetal.sum():
                add(WARN, 'B (metalicznosc) nie jest 0/1 na %d%% metalu' % (100 * middle.sum() / anymetal.sum()))
            bb = blur(B, 3)
            speck = float(((bb > 0.08) & (bb < 0.92) & (np.abs(B - bb) > 0.25) & mask).mean())
            if speck > 0.01:
                add(WARN, 'B w plamki (%d%% powierzchni): metal migocze' % (speck * 100))
    else:
        roughness = None
        if shader['gloss'] == 'param':
            roughness = 0.04
        elif shader['gloss'] == 'unset':
            roughness = min(max(1 - 1 / max(pg_effective, 1), 0.04), 1)
        elif shader['gloss'] == 'phong':
            roughness = min(max((2 / (pg_effective + 2)) ** 0.25, 0.04), 1)
        if shader['spec'] == 'diffuse' and diffuse is not None:
            lin = srgb_to_linear(diffuse[..., :3])
            e = pct((lin.sum(axis=2) * 0.5)[opaque], 50) * psv
            if e > DIELECTRIC_SPEC[1] and roughness is not None and roughness < 0.7:
                suggestion = DIELECTRIC_SPEC[1] / 2 / max(e / psv, 1e-3)
                kind = 'lustrzany punkt (shader %s zawsze ma roughness 0.04)' % shadername if shader['gloss'] == 'param' else 'roughness %.2f z param_glossiness %g' % (roughness, pg_effective)
                if shader['gloss'] == 'phong' and pg is None:
                    kind += ' (domyslne glossiness 10)'
                add(WARN, 'polysk z jasnosci diffuse %.2f, %s; param_specular ok. %.2f' % (e, kind, suggestion))
        if shader['gloss'] == 'unset' and pg is not None and pg_effective <= 2:
            add(INFO, 'param_glossiness %g w shaderze %s: tu wieksza wartosc = bardziej matowo (roughness %.2f)' % (pg, shadername, roughness))
        if shader['gloss'] == 'param' and pr > 0.2 and shader['reflectivity'] == 'param':
            add(INFO, 'param_reflection %g w shaderze %s: odbicie lustrzane calej powierzchni' % (pr, shadername))

    if diffuse is not None and shader['gloss'] == 'specgloss':
        srgb = diffuse[..., :3] @ np.array([0.2126, 0.7152, 0.0722]) * 255
        sg = image('specgloss')
        nonmetal = opaque & ((sg[..., 2] < 0.5) if sg is not None else True)
        if nonmetal.any():
            out = float(((srgb < ALBEDO_SRGB[0]) | (srgb > ALBEDO_SRGB[1]))[nonmetal].mean())
            if out > 0.15:
                add(INFO, 'diffuse poza sRGB %d-%d na %d%% niemetalu (wypalone swiatlo albo czern)' % (*ALBEDO_SRGB, out * 100))

    if shader['reflectivity'] == 'normalmap' and 'normalmap' in textures and shader['gloss'] == 'specgloss':
        normal = image('normalmap')
        if normal is not None:
            a = normal[..., 3]
            noise = float(np.mean(np.abs(a - blur(a, 2))))
            if noise > NOISE_MAX:
                add(WARN, 'szum w alfie normalmapy (odbicie) %.3f' % noise)
    if nested:
        add(INFO, 'ma bloki pory roku/pogody, sprawdzono tylko wartosci podstawowe')
    return shadername, findings


WORKER = {}


def init_worker(roots, shaderdir):
    WORKER['shaders'] = Shaders(shaderdir)
    WORKER['resolver'] = Resolver(roots)
    WORKER['images'] = Images()
    WORKER['gamedir'] = roots[0]


def check_group(mats):
    results = []
    images = WORKER['images']
    for mat in mats:
        rel = os.path.relpath(mat, WORKER['gamedir'])
        try:
            values, nested = parse_mat(mat)
            shadername, findings = check(mat, values, nested, WORKER['shaders'], WORKER['resolver'], images, rel)
        except Exception as e:
            shadername, findings = '?', [(ERROR, 'blad walidatora: %s' % e)]
        results.append((rel, shadername, findings))
    images.cache.clear()
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('gamedir', help='katalog gry albo kopii roboczej z assetami (np. pctga)')
    parser.add_argument('subdirs', nargs='*', default=['dynamic'], help='podkatalogi do sprawdzenia (domyslnie dynamic)')
    parser.add_argument('--fallback', action='append', default=[], help='dodatkowy katalog gry, w ktorym szukac tekstur')
    parser.add_argument('--shaders', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'shaders'), help='katalog shaderow')
    parser.add_argument('--csv', help='zapisz wyniki do pliku CSV')
    parser.add_argument('--info', action='store_true', help='pokaz tez informacje, nie tylko bledy i uwagi')
    parser.add_argument('--jobs', '-j', type=int, default=os.cpu_count() or 1, help='liczba procesow (domyslnie wszystkie rdzenie)')
    args = parser.parse_args()

    gamedir = os.path.abspath(args.gamedir)
    roots = [gamedir] + [os.path.abspath(f) for f in args.fallback]
    shaderdir = os.path.abspath(args.shaders)
    if Shaders(shaderdir).get('default') is None:
        sys.exit('nie ma shaderow w %s, podaj --shaders' % shaderdir)

    groups = {}
    for sub in args.subdirs:
        for root, _, files in os.walk(os.path.join(gamedir, sub), followlinks=True):
            mats = sorted(os.path.join(root, f) for f in files if f.lower().endswith('.mat'))
            for i in range(0, len(mats), 32):
                groups[(root, i)] = mats[i:i + 32]
    groups = [groups[k] for k in sorted(groups)]
    total = sum(len(g) for g in groups)

    if args.jobs > 1:
        import multiprocessing
        pool = multiprocessing.Pool(args.jobs, init_worker, (roots, shaderdir))
        results = pool.imap(check_group, groups)
    else:
        init_worker(roots, shaderdir)
        pool = None
        results = map(check_group, groups)

    rows = []
    counts = {ERROR: 0, WARN: 0, INFO: 0}
    for group in results:
        for rel, shadername, findings in group:
            shown = [f for f in findings if args.info or f[0] != INFO]
            for level, text in findings:
                counts[level] += 1
                rows.append((rel, shadername, level, text))
            if shown:
                print('%s  [%s]' % (rel, shadername))
                for level, text in shown:
                    print('  %-5s %s' % (level, text))
    if pool:
        pool.close()
        pool.join()
    print('\n%d materialow: %d bledow, %d uwag, %d informacji' % (total, counts[ERROR], counts[WARN], counts[INFO]))
    if args.csv:
        with open(args.csv, 'w', newline='', encoding='utf-8') as out:
            writer = csv.writer(out)
            writer.writerow(['material', 'shader', 'poziom', 'opis'])
            writer.writerows(rows)
    return 1 if counts[ERROR] else 0


if __name__ == '__main__':
    sys.exit(main())
