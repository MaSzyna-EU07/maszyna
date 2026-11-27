#!/usr/bin/env python3
# compiles the shaders of shaders/ogl46 and shaders/vk14 to SPIR-V with glslc, into shaders/ogl46bin and shaders/vk14bin,
# which the renderers load before they compile the sources (gfx.shaders.binaries). each directory of binaries gets
# manifest.txt: the source files each binary was compiled from, with their hashes, so the game doesn't take a binary older
# than its sources.
#
#   python shaders/compile_spirv.py [--only ogl46|vk14]
#
# the shaders are those of the directory of this script, and glslc is the one of the Vulkan SDK (VULKAN_SDK is looked at) or
# the one on PATH (the shaderc package); nothing on the command line names a file. the includes read stay inside the directory
# of the sources. the shaders of opengl are compiled
# for --target-env=opengl4.5 (SPIR-V 1.0, which opengl 4.6 takes), with the variant of the multi draws (MODEL_DRAWS) of the
# vertex and fragment shaders; those of vulkan for --target-env=vulkan1.4. a single file can be compiled by hand the same way:
#   glslc --target-env=opengl4.5 shaders/ogl46/mat_default.frag -o shaders/ogl46bin/mat_default.frag.spv
#   glslc --target-env=opengl4.5 -DMODEL_DRAWS=1 shaders/ogl46/vertex.vert -o shaders/ogl46bin/vertex.vert.MODEL_DRAWS.spv
#   glslc --target-env=vulkan1.4 shaders/vk14/imgui.frag -o shaders/vk14bin/imgui.frag.spv
# (a binary compiled by hand isn't in the manifest, so the game takes it without comparing it with its sources)

import argparse
import os
import re
import shutil
import subprocess
import sys

STAGES = ('.vert', '.frag', '.geom', '.comp')
TREES = {
    # source directory, binary directory, target environment, variants of the vertex and fragment shaders, whether a variant is
    # made only for the files which use its name (opengl links a program from the binaries of all of its stages or from none)
    'ogl46': ('ogl46', 'ogl46bin', 'opengl4.5', ['MODEL_DRAWS'], False),
    'vk14': ('vk14', 'vk14bin', 'vulkan1.4', ['NO_MOTION_OUTPUT'], True),
}
INCLUDE = re.compile(r'^[ \t]*#include[ \t]*[<"]([^>"]+)[>"]', re.M)
GLSLC_NAMES = ('glslc', 'glslc.exe')


def inside(base, path):
    # the path without links and .., when it is the directory base or a path under it; None when it leaves base
    base = os.path.realpath(base)
    path = os.path.realpath(path)
    try:
        common = os.path.commonpath([base, path])
    except ValueError:  # another drive
        return None
    return path if os.path.normcase(common) == os.path.normcase(base) else None


def file_hash(path):
    # fnv-1a 64 of the text with the line ends made \n, the same as gfx::shader_file_hash()
    with open(path, 'rb') as f:
        data = f.read().replace(b'\r\n', b'\n')
    h = 14695981039346656037
    for b in data:
        h ^= b
        h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return '%016x' % h


def dependencies(directory, name, found=None):
    # the file and the files it includes, the same way as gfx::preprocess_shader()
    if found is None:
        found = []
    if name in found:
        return found
    path = inside(directory, os.path.join(directory, name))
    if path is None:
        raise OSError('%s is outside %s' % (name, directory))
    found.append(name)
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        text = f.read()
    for include in INCLUDE.findall(text):
        if include == 'common':
            include = 'common.glsl'
        dependencies(directory, include, found)
    return found


def find_glslc():
    # glslc of the Vulkan SDK (VULKAN_SDK), else the one on PATH; never a path given on the command line
    sdk = os.environ.get('VULKAN_SDK')
    if sdk:
        for sub in ('Bin', 'bin'):
            for exe in GLSLC_NAMES:
                path = os.path.join(sdk, sub, exe)
                if os.path.isfile(path):
                    return path
    return shutil.which('glslc')


def variant_jobs(sources, name, deps, variants, onlyused):
    # the plain binary, and for the vertex and fragment shaders one per variant (only the variants the file uses, if onlyused)
    jobs = [('', [])]
    if os.path.splitext(name)[1] not in ('.vert', '.frag'):
        return jobs
    text = ''
    for dep in deps:
        with open(os.path.join(sources, dep), 'r', encoding='utf-8', errors='replace') as f:
            text += f.read()
    return jobs + [(v, ['-D%s=1' % v]) for v in variants if (not onlyused) or (v in text)]


def compile_tree(glslc, shaders, tree, failed):
    # compiles one tree of TREES into its directory of binaries with its manifest.txt; returns how many binaries were made
    sourcedir, binarydir, env, variants, onlyused = tree
    sources = os.path.join(shaders, sourcedir)
    binaries = os.path.join(shaders, binarydir)
    if not os.path.isdir(sources):
        return 0
    os.makedirs(binaries, exist_ok=True)
    manifest = ['# made by compile_spirv.py: binary, source file it was compiled from, fnv-1a 64 of the source (line ends as \\n)']
    count = 0
    for name in sorted(n for n in os.listdir(sources) if os.path.splitext(n)[1] in STAGES):
        try:
            deps = dependencies(sources, name)
        except OSError as e:
            failed.append('%s/%s (%s)' % (sourcedir, name, e))
            continue
        for variant, defines in variant_jobs(sources, name, deps, variants, onlyused):
            binary = name + ('.' + variant if variant else '') + '.spv'
            command = [glslc, '--target-env=' + env] + defines + [os.path.join(sources, name), '-o', os.path.join(binaries, binary)]
            result = subprocess.run(command, capture_output=True, text=True)
            if result.stderr.strip():
                sys.stderr.write(result.stderr)
            if result.returncode != 0:
                failed.append('%s/%s' % (binarydir, binary))
                continue
            count += 1
            manifest += ['%s %s %s' % (binary, dep, file_hash(os.path.join(sources, dep))) for dep in deps]
    with open(os.path.join(binaries, 'manifest.txt'), 'w', newline='\n') as f:
        f.write('\n'.join(manifest) + '\n')
    return count


def main():
    parser = argparse.ArgumentParser(description='compiles the shaders of MaSzyna (the directory of this script) to SPIR-V with glslc')
    parser.add_argument('--only', choices=sorted(TREES), help='only the shaders of one renderer')
    args = parser.parse_args()

    glslc = find_glslc()
    if not glslc or not os.path.isfile(glslc):
        print('glslc not found: install the Vulkan SDK (VULKAN_SDK) or shaderc, or put glslc on PATH', file=sys.stderr)
        return 1
    # the shaders directory is the one of this script: nothing on the command line names a file or a directory
    shaders = os.path.dirname(os.path.realpath(__file__))

    failed = []
    count = sum(compile_tree(glslc, shaders, tree, failed) for key, tree in TREES.items() if args.only in (None, key))

    print('%d binaries compiled' % count)
    if failed:
        print('failed: ' + ' '.join(failed), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
