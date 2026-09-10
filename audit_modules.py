#!/usr/bin/env python3
"""Audit the tree for the problems that only some toolchains diagnose.

Every check here corresponds to something that broke a real build:

  1. export-scope    `export` on an out-of-line member definition. Clang rejects
                     it; GCC accepted it. Nothing to do with which compiler is
                     "right" - the declaration introduces no namespace-scope
                     name, so it cannot be exported.

  2. fwd-decl        A forward declaration of a class another module owns.
                     Ill-formed across a module boundary.

  3. iwyu            A unit naming std:: things without including the header.
                     libstdc++ 16 drags most of them in; libstdc++ 13 and
                     MSVC's STL do not, and ADL then misses the free operators
                     declared beside the type.

  4. bare-c          Unqualified uint8_t / size_t and friends, same story.

  5. include-order   An #include after an import. The header's guard, if it
                     first arrived through the imported module's global module
                     fragment, is not visible here - macros do not cross a
                     module boundary - so the body is parsed twice and MSVC
                     rejects the redefinition.

  6. template-hdr    A template defined in a module interface is instantiated
                     in the importing unit, so the types its body names must be
                     complete THERE. Reaching them through the interface's
                     fragment is not enough for MSVC.

  7. macro-damage    An include or import sitting inside a #define's line
                     continuation. Only ever caused by a botched rewrite.

Read-only by default. Pass --fix to apply what is mechanically fixable
(3, 4, 5, 6); 1, 2 and 7 are reported for a human to look at.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
SKIP = ('ref', 'betterRenderer/thirdparty', 'betterRenderer/shaders', '.git',
        'imgui', 'stb', 'ci_shadervalidator', 'build')
# single-header libraries pasted into the tree: their includes are often behind
# an implementation guard, so leave them alone
VENDORED = {'tinyexr.h', 'xegtao.h', 'nanozlib.h', 'inteltaa.h', 'stb_image.h',
            'stb_image_resize2.h', 'ddspp.h', 'ma_dds.h'}

HDR = {}
for _hdr, _names in {
    'array': 'array', 'atomic': 'atomic atomic_bool atomic_int atomic_flag',
    'bitset': 'bitset', 'chrono': 'chrono', 'condition_variable': 'condition_variable',
    'cstddef': 'size_t ptrdiff_t byte nullptr_t',
    'cstdint': 'uint8_t uint16_t uint32_t uint64_t int8_t int16_t int32_t int64_t '
               'uintptr_t intptr_t',
    'deque': 'deque', 'filesystem': 'filesystem', 'forward_list': 'forward_list',
    'fstream': 'ifstream ofstream fstream',
    'functional': 'function bind ref cref reference_wrapper hash less greater invoke',
    'future': 'future async promise packaged_task shared_future',
    'initializer_list': 'initializer_list', 'iomanip': 'setprecision setw setfill',
    'ios': 'ios ios_base streamsize', 'iostream': 'cout cerr cin clog endl',
    'istream': 'istream',
    'iterator': 'back_inserter front_inserter inserter distance advance next prev '
                'iterator_traits',
    'limits': 'numeric_limits', 'list': 'list', 'map': 'map multimap',
    'memory': 'shared_ptr unique_ptr weak_ptr make_shared make_unique '
              'enable_shared_from_this addressof default_delete static_pointer_cast '
              'dynamic_pointer_cast const_pointer_cast',
    'mutex': 'mutex lock_guard unique_lock recursive_mutex scoped_lock once_flag call_once',
    'numeric': 'accumulate iota inner_product',
    'optional': 'optional nullopt make_optional', 'ostream': 'ostream',
    'queue': 'queue priority_queue',
    'random': 'mt19937 mt19937_64 random_device uniform_real_distribution '
              'uniform_int_distribution normal_distribution',
    'set': 'set multiset', 'sstream': 'stringstream ostringstream istringstream',
    'stack': 'stack',
    'stdexcept': 'runtime_error logic_error out_of_range invalid_argument length_error',
    'string': 'string wstring to_string stoi stof stod stol stoul stoll string_view getline',
    'thread': 'thread jthread this_thread',
    'tuple': 'tuple make_tuple tie tuple_size tuple_element apply',
    'type_traits': 'is_same enable_if underlying_type decay remove_reference conditional '
                   'is_base_of declval same_as',
    'typeinfo': 'type_info bad_cast',
    'unordered_map': 'unordered_map unordered_multimap',
    'unordered_set': 'unordered_set unordered_multiset',
    'utility': 'pair make_pair move forward swap exchange',
    'variant': 'variant visit holds_alternative get_if monostate', 'vector': 'vector',
}.items():
    for _n in _names.split():
        HDR.setdefault(_n, _hdr)

# names too overloaded to attribute to one header
AMBIGUOUS = {'get', 'begin', 'end', 'size', 'data', 'min', 'max', 'abs', 'swap',
             'move', 'forward'}
BARE = {'uint8_t': 'cstdint', 'uint16_t': 'cstdint', 'uint32_t': 'cstdint',
        'uint64_t': 'cstdint', 'int8_t': 'cstdint', 'int16_t': 'cstdint',
        'int32_t': 'cstdint', 'int64_t': 'cstdint', 'uintptr_t': 'cstdint',
        'intptr_t': 'cstdint', 'size_t': 'cstddef', 'ptrdiff_t': 'cstddef'}
STD_HEADERS = set(HDR.values()) | set(BARE.values())

MODULE_DECL = re.compile(r'^(export )?module ([\w.:]+);', re.M)
IMPORT = re.compile(r'^[ \t]*import[ \t]+[\w.:]+[ \t]*;', re.M)
INCLUDE = re.compile(r'^[ \t]*#[ \t]*include\b')


def sources():
    for dp, dn, fs in os.walk(ROOT):
        rel = os.path.relpath(dp, ROOT)
        if rel != '.' and any(rel == s or rel.startswith(s + os.sep) or rel.startswith(s)
                              for s in SKIP):
            dn[:] = []
            continue
        for f in sorted(fs):
            if f.endswith(('.cpp', '.cppm', '.h', '.hpp')) and f.lower() not in VENDORED:
                yield os.path.join(dp, f)


def read(p):
    raw = open(p, encoding='utf-8', errors='replace').read()
    bom = raw.startswith('﻿')
    return raw[1:] if bom else raw, bom


def split_unit(src):
    """(global module fragment, purview, module name or None)."""
    m = MODULE_DECL.search(src)
    if not m:
        return src, src, None
    return src[:m.start()], src[m.end():], m.group(2)


def template_bodies(purview):
    out = []
    for m in re.finditer(r'^[ \t]*template[ \t]*<', purview, re.M):
        i, depth, started = m.start(), 0, False
        while i < len(purview):
            c = purview[i]
            if c == '{':
                depth += 1
                started = True
            elif c == '}':
                depth -= 1
                if started and depth == 0:
                    i += 1
                    break
            elif c == ';' and not started:
                break
            i += 1
        out.append(purview[m.start():i])
    return out


def insert_includes(src):
    """Where a new #include belongs.

    In a module unit that is the global module fragment - after `module;` and
    before the module declaration. Putting it after the declaration would place
    the include in the purview, attaching those declarations to the module.
    In an ordinary translation unit it is the top level of the preprocessor,
    before the first import, never inside a #define's continuation.
    """
    decl = MODULE_DECL.search(src)
    if decl:
        g = re.search(r'^module;[ \t]*\n', src, re.M)
        if g:
            return src[:g.end()], src[g.end():]
        return src[:decl.start()] + 'module;\n', '\n' + src[decl.start():]
    lines, at = _plain_insert_point(src)
    return '\n'.join(lines[:at]) + ('\n' if at else ''), '\n'.join(lines[at:])


def _plain_insert_point(src):
    """Top level of the preprocessor, before the first import, never inside a
    #define's continuation."""
    depth, last_include, first_import, i = 0, None, None, 0
    lines = src.split('\n')
    while i < len(lines):
        l = lines[i]
        if re.match(r'^[ \t]*#[ \t]*(?:if|ifdef|ifndef)\b', l):
            depth += 1
        elif re.match(r'^[ \t]*#[ \t]*endif\b', l):
            depth -= 1
        elif l.lstrip().startswith('#'):
            start = i
            while lines[i].rstrip().endswith('\\') and i + 1 < len(lines):
                i += 1
            if INCLUDE.match(l) and depth == 0 and start == i and first_import is None:
                last_include = i
        elif IMPORT.match(l) and depth == 0 and first_import is None:
            first_import = i
        i += 1
    at = first_import if first_import is not None else (
        last_include + 1 if last_include is not None else 0)
    return lines, at


def main():
    fix = '--fix' in sys.argv
    units, provides, imports_of = [], {}, {}
    findings = {k: [] for k in ('export-scope', 'fwd-decl', 'iwyu', 'bare-c',
                                'include-order', 'template-hdr', 'macro-damage')}
    owners = {}

    files = list(sources())
    for p in files:
        src, bom = read(p)
        gmf, purview, mod = split_unit(src)
        units.append((p, src, gmf, purview, mod, bom))
        if mod:
            deps = {x.split(':')[0] for x in re.findall(r'^import ([\w.:]+);', src, re.M)}
            imports_of.setdefault(mod, set()).update(deps)
        if p.endswith('.cppm') and mod:
            base = mod.split(':')[0]
            for c in re.findall(r'^\s*(?:export\s+)?(?:class|struct)\s+(\w+)\s*'
                                r'(?:final\s*)?(?::[^;{]*)?\{', src, re.M):
                owners.setdefault(c, base)
            need = set()
            for body in template_bodies(purview):
                for n in set(re.findall(r'\bstd::(\w+)', body)):
                    if n not in AMBIGUOUS and HDR.get(n):
                        need.add(HDR[n])
            if need:
                provides.setdefault(base, set()).update(need)

    def closure(names):
        seen, todo = set(), list(names)
        while todo:
            n = todo.pop()
            if n in seen:
                continue
            seen.add(n)
            todo.extend(imports_of.get(n, ()))
        return seen

    edits = {}
    for p, src, gmf, purview, mod, bom in units:
        rel = os.path.relpath(p, ROOT)
        lines = src.split('\n')

        # 7. an include or import swallowed by a macro continuation
        for i, l in enumerate(lines):
            if i and lines[i - 1].rstrip().endswith('\\') and \
                    (INCLUDE.match(l) or IMPORT.match(l)):
                findings['macro-damage'].append('%s:%d' % (rel, i + 1))
                break

        # 1. export on an out-of-line member definition
        if p.endswith('.cppm'):
            cls = set(re.findall(r'^\s*(?:export\s+)?(?:class|struct)\s+(\w+)\s*'
                                 r'(?:final\s*)?(?::[^;{]*)?\{', src, re.M))
            depth, inexp = 0, False
            for i, l in enumerate(src.split('\n')):
                if re.match(r'\s*export\s*\{', l):
                    inexp, depth = True, 0
                    continue
                # `export inline glm::vec3 f(...)` is a free function whose return
                # type happens to be qualified, so match the declarator's class
                # against the ones this file defines rather than any `::` at all
                m = re.match(r'^\s*export\s+.*?\b(\w+)(?:<[^<>]*>)?::(~?\w+|operator[^\s(]*)\s*[\(<]', l)
                if m and m.group(1) in cls:
                    findings['export-scope'].append('%s:%d' % (rel, i + 1))
                if inexp:
                    if depth == 0:
                        m = re.match(r'\s*(?:template\s*<.*>\s*)?(?:inline\s+|constexpr\s+|'
                                     r'virtual\s+|static\s+)*(?:[\w:<>,&*\s]*?\s)?'
                                     r'(\w+)(?:<[^<>]*>)?::(~?\w+|operator[^\s(]*)\s*[\(<]', l)
                        if m and m.group(1) in cls:
                            findings['export-scope'].append('%s:%d' % (rel, i + 1))
                    depth += l.count('{') - l.count('}')
                    if depth < 0:
                        inexp, depth = False, 0

            # 2. forward declaration of a class another module owns
            for m in re.finditer(r'^\s*(?:class|struct)\s+(\w+)\s*;', src, re.M):
                o = owners.get(m.group(1))
                if o and mod and o != mod.split(':')[0]:
                    findings['fwd-decl'].append(
                        '%s:%d %s -> %s' % (rel, src[:m.start()].count('\n') + 1,
                                            m.group(1), o))

        # 5. an include after an import
        depth, first_import, after = 0, None, []
        i, ls = 0, lines
        while i < len(ls):
            l = ls[i]
            if re.match(r'^[ \t]*#[ \t]*(?:if|ifdef|ifndef)\b', l):
                depth += 1
            elif re.match(r'^[ \t]*#[ \t]*endif\b', l):
                depth -= 1
            elif l.lstrip().startswith('#'):
                start = i
                while ls[i].rstrip().endswith('\\') and i + 1 < len(ls):
                    i += 1
                # Only a standard header is a hazard here. A project header in the
                # purview is normal and sometimes required: an out-of-line member
                # of a module-attached class has to be defined inside the purview.
                std_after = re.match(r'^[ \t]*#[ \t]*include[ \t]*<([\w.]+)>', l)
                if std_after and std_after.group(1) in STD_HEADERS \
                        and depth == 0 and start == i and first_import is not None:
                    after.append(i)
            elif IMPORT.match(l) and depth == 0 and first_import is None:
                first_import = i
            i += 1
        if after:
            findings['include-order'].append('%s (%d)' % (rel, len(after)))

        # 3/4/6. headers this unit should include itself
        if mod is None and not re.search(r'^import ', src, re.M):
            continue                       # a plain header with no module contact
        need = set()
        for n in set(re.findall(r'\bstd::(\w+)', purview)):
            if n not in AMBIGUOUS and HDR.get(n) and '<%s>' % HDR[n] not in gmf:
                need.add(HDR[n])
        for n, h in BARE.items():
            if re.search(r'(?<!:)\b%s\b' % n, purview) and '<%s>' % h not in gmf \
                    and '<%s.h>' % h[1:] not in gmf:
                need.add(h)
        if need:
            findings['iwyu' if any(h not in BARE.values() for h in need)
                     else 'bare-c'].append('%s %s' % (rel, ' '.join(sorted(need))))

        direct = {x.split(':')[0] for x in re.findall(r'^import ([\w.:]+);', src, re.M)}
        if mod:
            direct.add(mod.split(':')[0])
        want = set()
        for im in closure(direct):
            want |= provides.get(im, set())
        miss = sorted(h for h in want if '<%s>' % h not in gmf)
        if miss:
            findings['template-hdr'].append('%s %s' % (rel, ' '.join(miss)))
            need |= set(miss)

        if fix and need:
            edits[p] = (sorted(need), bom)

    for k in ('macro-damage', 'export-scope', 'fwd-decl', 'include-order',
              'iwyu', 'bare-c', 'template-hdr'):
        v = findings[k]
        print('%-15s %d' % (k, len(v)))
        for row in v[:12]:
            print('                  %s' % row)
        if len(v) > 12:
            print('                  ... i %d wiecej' % (len(v) - 12))

    if fix and edits:
        touched = 0
        for p, (need, bom) in edits.items():
            src, _ = read(p)
            gmf = split_unit(src)[0]
            add = ''.join('#include <%s>\n' % h for h in need
                          if '<%s>' % h not in gmf)
            if not add:
                continue
            head, tail = insert_includes(src)
            open(p, 'w', encoding='utf-8').write(
                ('﻿' if bom else '') + head + add + tail)
            touched += 1
        print('\nnaprawiono %d plikow' % touched)


main()
