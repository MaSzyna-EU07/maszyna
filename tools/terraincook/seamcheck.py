#!/usr/bin/env python3
"""Checks that terrain tiles at neighbouring levels meet without gaps.

The renderer picks each tile's level from its distance at the last rescan, and morphs every
vertex towards the next level as the camera moves (rendering/terrainclipmap.vert). Two
neighbours at levels L and L+1 share their edge exactly when, all along it, the finer tile
has finished morphing and the coarser one has not started. This walks random camera
positions, lets the camera drift up to a rescan's distance from where the levels were
chosen, and counts every edge vertex where that does not hold.

The constants mirror rendering/terrainstatus.cppm; change them together. Exits non-zero
when a gap is found.
"""

import math
import random
import sys

# finest ranges tried, in tile sides: terrain_finest_tiles is the minimum, and the screen
# can push it further (terrain_finest_range)
FINEST_TILES = (4.0, 5.7, 11.3)
RESCAN_TILES = 0.25  # terrain_rescan_tiles
MORPH_BAND = 0.2     # terrain_morph_band
LEVELS = 6


def check(tilesize, step, trials, reach, finest_tiles):
    finest = finest_tiles * tilesize

    def level_range(level):
        return finest * 2 ** level

    def level_for(distance):
        if distance <= finest:
            return 0
        return min(int(math.log2(distance / finest)) + 1, LEVELS - 1)

    def nearest(x, z, tilex, tilez):
        nx = min(max(x, tilex * tilesize), (tilex + 1) * tilesize)
        nz = min(max(z, tilez * tilesize), (tilez + 1) * tilesize)
        return math.hypot(nx - x, nz - z)

    def morph(level, distance):
        if level + 1 >= LEVELS:
            return 0.0
        end = level_range(level) - RESCAN_TILES * tilesize
        begin = end - MORPH_BAND * level_range(level)
        return min(max((distance - begin) / (end - begin), 0.0), 1.0)

    gaps = 0
    example = ""
    for _ in range(trials):
        scanx, scanz = random.uniform(-5000, 5000), random.uniform(-5000, 5000)
        drift, heading = random.uniform(0, RESCAN_TILES * tilesize), random.uniform(0, 2 * math.pi)
        camx, camz = scanx + drift * math.cos(heading), scanz + drift * math.sin(heading)
        basex, basez = math.floor(camx / tilesize), math.floor(camz / tilesize)
        for tilex in range(basex - reach, basex + reach):
            for tilez in range(basez - reach, basez + reach):
                here = level_for(nearest(scanx, scanz, tilex, tilez))
                for stepx, stepz in ((1, 0), (0, 1)):
                    there = level_for(nearest(scanx, scanz, tilex + stepx, tilez + stepz))
                    if here == there:
                        continue
                    if abs(here - there) > 1:
                        gaps += 1
                        continue
                    fine, coarse = min(here, there), max(here, there)
                    count = int(tilesize / step) >> fine
                    for index in range(count + 1):
                        along = index * tilesize / count
                        if stepx:
                            x, z = (tilex + 1) * tilesize, tilez * tilesize + along
                        else:
                            x, z = tilex * tilesize + along, (tilez + 1) * tilesize
                        distance = math.hypot(x - camx, z - camz)
                        # an odd vertex of the finer tile is not on the coarser grid and has
                        # to have finished its move; the coarser edge must not be moving yet
                        if (index % 2 == 1 and morph(fine, distance) < 1.0) or morph(coarse, distance) > 0.0:
                            gaps += 1
                            if not example:
                                example = (f"camera {camx:.0f},{camz:.0f}, levels {fine}/{coarse}, "
                                           f"distance {distance:.0f} m")
    return gaps, example


def main():
    random.seed(1)
    failed = False
    for finest_tiles in FINEST_TILES:
        for tilesize, step, trials, reach in ((128, 1.0, 150, 60), (256, 2.0, 150, 60), (12800, 100.0, 30, 8)):
            gaps, example = check(tilesize, step, trials, reach, finest_tiles)
            print(f"finest {finest_tiles} tiles, tile {tilesize} m, grid {step} m: {gaps} gaps"
                  + (f" (e.g. {example})" if gaps else ""))
            failed = failed or gaps > 0
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
