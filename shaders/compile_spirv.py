#!/usr/bin/env python3
# compiles the shaders of shaders/ogl46 and shaders/vk14 to SPIR-V with glslc, into shaders/ogl46bin and shaders/vk14bin,
# which the renderers load before they compile the sources (gfx.shaders.binaries). each directory of binaries gets
# manifest.txt: the source files each binary was compiled from, with their hashes, so the game doesn't take a binary older
# than its sources.
#
#   python shaders/compile_spirv.py [--glslc <path>] [--only ogl46|vk14] [<shaders directory>]
#
# glslc comes with the Vulkan SDK (VULKAN_SDK is looked at) or with the shaderc package. the shaders of opengl are compiled
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
    # source directory, binary directory, target environment, variants of the vertex and fragment shaders
    'ogl46': ('ogl46', 'ogl46bin', 'opengl4.5', ['MODEL_DRAWS']),
    'vk14': ('vk14', 'vk14bin', 'vulkan1.4', []),
}
INCLUDE = re.compile(r'^[ \t]*#include[ \t]*[<"]([^>"]+)[>"]', re.M)


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
    found.append(name)
    with open(os.path.join(directory, name), 'r', encoding='utf-8', errors='replace') as f:
        text = f.read()
    for include in INCLUDE.findall(text):
        if include == 'common':
            include = 'common.glsl'
        dependencies(directory, include, found)
    return found


def find_glslc(explicit):
    if explicit:
        return explicit
    sdk = os.environ.get('VULKAN_SDK')
    if sdk:
        for sub in ('Bin', 'bin'):
            for exe in ('glslc.exe', 'glslc'):
                path = os.path.join(sdk, sub, exe)
                if os.path.isfile(path):
                    return path
    return shutil.which('glslc')


def main():
    parser = argparse.ArgumentParser(description='compiles the shaders of MaSzyna to SPIR-V with glslc')
    parser.add_argument('shaders', nargs='?', default=os.path.dirname(os.path.abspath(__file__)), help='the shaders directory (the one of this script by default)')
    parser.add_argument('--glslc', help='path of glslc')
    parser.add_argument('--only', choices=sorted(TREES), help='only the shaders of one renderer')
    args = parser.parse_args()

    glslc = find_glslc(args.glslc)
    if not glslc:
        print('glslc not found: install the Vulkan SDK (VULKAN_SDK) or shaderc, or give --glslc', file=sys.stderr)
        return 1

    failed = []
    count = 0
    for key, (sourcedir, binarydir, env, variants) in TREES.items():
        if args.only and args.only != key:
            continue
        sources = os.path.join(args.shaders, sourcedir)
        binaries = os.path.join(args.shaders, binarydir)
        if not os.path.isdir(sources):
            continue
        os.makedirs(binaries, exist_ok=True)
        manifest = ['# made by compile_spirv.py: binary, source file it was compiled from, fnv-1a 64 of the source (line ends as \\n)']
        names = sorted(n for n in os.listdir(sources) if os.path.splitext(n)[1] in STAGES)
        for name in names:
            jobs = [('', [])]
            if os.path.splitext(name)[1] in ('.vert', '.frag'):
                jobs += [(v, ['-D%s=1' % v]) for v in variants]
            try:
                deps = dependencies(sources, name)
            except OSError as e:
                failed.append('%s/%s (%s)' % (sourcedir, name, e))
                continue
            for variant, defines in jobs:
                binary = name + ('.' + variant if variant else '') + '.spv'
                command = [glslc, '--target-env=' + env] + defines + [os.path.join(sources, name), '-o', os.path.join(binaries, binary)]
                result = subprocess.run(command, capture_output=True, text=True)
                if result.returncode != 0:
                    failed.append('%s/%s' % (binarydir, binary))
                    sys.stderr.write(result.stderr)
                    continue
                if result.stderr.strip():
                    sys.stderr.write(result.stderr)
                count += 1
                for dep in deps:
                    manifest.append('%s %s %s' % (binary, dep, file_hash(os.path.join(sources, dep))))
        with open(os.path.join(binaries, 'manifest.txt'), 'w', newline='\n') as f:
            f.write('\n'.join(manifest) + '\n')

    print('%d binaries compiled' % count)
    if failed:
        print('failed: ' + ' '.join(failed), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
