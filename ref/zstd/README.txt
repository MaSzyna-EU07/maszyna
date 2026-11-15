Zstandard 1.5.6 (https://github.com/facebook/zstd), BSD licence (see LICENSE).

zstd.c is the single file amalgamation made with build/single_file_libs/combine.py from
zstd-in.c, without the dictionary builder and without multithreading:

  sed -e "s/^#ifndef __EMSCRIPTEN__$/#if 0/" -e "/dictBuilder/d" zstd-in.c > zstd-maszyna-in.c
  python3 combine.py -r ../../lib -x legacy/zstd_legacy.h -o zstd.c zstd-maszyna-in.c

Used by the heightmap terrain files (.tch).
