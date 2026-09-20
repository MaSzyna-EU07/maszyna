#!/usr/bin/env python3
"""Builds the terrain test bed for the td scenery.

It stands in for td's own ground, which was sixty-two huge triangles of grass, and gives the cook and
the renderer the three things a real scenery has and a synthetic one usually lacks: ground drawn at
several densities, boundaries between materials of several shapes, and enough length that travelling
along it loads tiles and lets them go.

Run it with the game's directory as its argument. It writes scenery/td/tinpoligon.scm, which
scenery/$td.scn includes.
"""
import math
import os
import sys

FIELDX0, FIELDX1 = -1200.0, 2200.0
FIELDZ0, FIELDZ1 = -1400.0, 3000.0
CORRIDORX0, CORRIDORX1 = -200.0, 200.0
CORRIDORZ1 = 120000.0
COARSE, FINE, DENSE = 40.0, 4.0, 1.0
TILING = 5.0

# where the ground is drawn small, as survey-derived ground is: one by the track and one far along, to
# look at again after travelling
FINESTRETCHES = ( ( -120.0, 400.0 ), ( 20000.0, 20400.0 ) )
# A range of hills the corridor runs through, six kilometres across and ten long, with a valley down
# the middle to travel along. Peaks four to six hundred metres: high enough to be read as hills from
# twenty kilometres away, which is what makes the coarse levels worth looking at - a level that keeps
# the height but loses the ridges shows it at once from that distance.
MOUNTAINS = ( -3000.0, 3000.0, 24000.0, 34000.0 )

# And a far larger range at the end of the corridor, to be looked at from a hundred kilometres. Peaks
# of two thousand metres: at that distance one still covers some twenty pixels, so whether a coarse
# level keeps the ridges or flattens them into a lump is plain to see from where the corridor starts.
FARRANGE = ( -12000.0, 12000.0, 88000.0, 112000.0 )

# and where it is drawn at the density an airborne scan gives, a metre between points. Two of them:
# one by the track, and one far along the corridor, so the same kind of ground can be looked at after
# travelling as well as from the start
DENSEPATCHES = (
    ( -200.0, 200.0, 600.0, 1000.0 ),
    ( -200.0, 200.0, 10000.0, 10300.0 ),
)


def inside( x, z, rectangle ):
    x0, x1, z0, z1 = rectangle
    return ( x0 <= x < x1 ) and ( z0 <= z < z1 )


def dense_here( x, z ):
    for patch in DENSEPATCHES:
        if inside( x, z, patch ):
            return patch
    return None


def far_height( x, z ):
    # the same shape at five times the size, so that it reads as a range from a hundred kilometres
    across = min( 1.0, max( 0.0, ( abs( x ) - 400.0 ) / 7000.0 ) )
    def ridged( value ):
        return 1.0 - abs( math.sin( value ) )
    crests = ( 1200.0 * ridged( z / 4200.0 + x / 9000.0 )
             + 600.0 * ridged( z / 1700.0 - x / 5200.0 )
             + 240.0 * ridged( z / 700.0 + x / 2100.0 ) )
    rolling = 180.0 * math.sin( z / 9000.0 ) * math.cos( x / 7000.0 )
    x0, x1, z0, z1 = FARRANGE
    edge = min( x - x0, x1 - x, z - z0, z1 - z )
    return ( across ** 1.5 ) * ( crests + rolling ) * min( 1.0, edge / 3000.0 )


def mountain_height( x, z ):
    # a valley along the middle, rising to ridges either side
    across = min( 1.0, max( 0.0, ( abs( x ) - 250.0 ) / 2000.0 ) )
    # ridged waves: sharp crests where a plain sine would give round tops
    def ridged( value ):
        return 1.0 - abs( math.sin( value ) )
    crests = ( 220.0 * ridged( z / 900.0 + x / 2600.0 )
             + 130.0 * ridged( z / 380.0 - x / 1500.0 )
             + 60.0 * ridged( z / 160.0 + x / 520.0 ) )
    rolling = 40.0 * math.sin( z / 2200.0 ) * math.cos( x / 1700.0 )
    # faded to nothing at the edges of the range, so it meets the flat ground around it
    x0, x1, z0, z1 = MOUNTAINS
    edge = min( x - x0, x1 - x, z - z0, z1 - z )
    return ( across ** 1.6 ) * ( crests + rolling ) * min( 1.0, edge / 600.0 )


def height( x, z ):
    if inside( x, z, FARRANGE ):
        return far_height( x, z )
    if inside( x, z, MOUNTAINS ):
        return mountain_height( x, z )
    # near nothing where the track runs, so the rails still sit on the ground
    away = min( 1.0, ( abs( x ) / 120.0 ) ** 2 )
    rolling = ( 3.0 * math.sin( z / 260.0 ) + 2.0 * math.cos( x / 140.0 )
              + 8.0 * math.sin( z / 1700.0 ) * math.cos( x / 400.0 ) )
    ridge = 6.0 * math.exp( -( ( x - 60.0 ) ** 2 + ( z - 200.0 ) ** 2 ) / 6000.0 )
    shape = away * ( rolling + ridge )
    # Detail only a dense mesh can hold, and only where the mesh is dense enough to hold it. Sampling
    # a wave four metres long with cells forty metres apart gives every corner an arbitrary height,
    # and the ground comes out as noise rather than as terrain.
    patch = dense_here( x, z )
    if patch is not None:
        dx0, dx1, dz0, dz1 = patch
        grain = 0.25 * math.sin( x * 1.7 ) * math.sin( z * 1.9 ) + 0.4 * math.sin( ( x + z ) / 7.0 )
        # faded at the patch's own edge, so it meets the coarse ground around it evenly
        grain *= min( 1.0, min( x - dx0, dx1 - x, z - dz0, dz1 - z ) / 10.0 )
        return shape + grain
    return shape


def material( x, z ):
    if inside( x, z, FARRANGE ):
        above = far_height( x, z )
        if above > 1100.0:
            return "concrete1"
        if above > 500.0:
            return "rockmossy1"
        if above > 150.0:
            return "ziemia"
        return "grassdarkgreen2"
    if inside( x, z, MOUNTAINS ):
        # by height, as ground cover goes: grass in the valley, bare earth on the slopes, rock on top
        above = mountain_height( x, z )
        if above > 260.0:
            return "rockmossy1"
        if above > 90.0:
            return "ziemia"
        return "grassdarkgreen2"
    if dense_here( x, z ) is not None:
        # patches a few metres across: material detail finer than the levels that will be drawn from
        # far away, which is what the cook has to be able to merge
        return "grass" if int( ( x + z ) // 6.0 ) % 2 == 0 else "ziemia"
    for from_, to in FINESTRETCHES:
        if from_ <= z < to:
            local = z - from_
            if ( x - 60.0 ) ** 2 + ( local - 150.0 ) ** 2 < 30.0 ** 2:
                return "concrete1"
            if local > 200.0 and ( x + 200.0 ) > ( local - 200.0 ) * 2.0:
                return "piach_plaza"
            return "grass" if x < 0.0 else "ziemia"
    band = int( ( z + 1400.0 ) // 500.0 )
    return [ "grass", "grassdarkgreen2", "ziemia", "piach_plaza", "asphaltgray1", "concrete1" ][ band % 6 ]


def hole( x, z ):
    # gaps the scenery leaves, as it does for water or a building, every two kilometres - but not
    # through the hills, where a hole would only be confusing
    if inside( x, z, MOUNTAINS ) or inside( x, z, FARRANGE ):
        return False
    return ( z > 3200.0 ) and ( math.fmod( z - 3000.0, 2000.0 ) < 80.0 ) and ( -80.0 < x < 80.0 )


lines = [
    "// Poligon doswiadczalny terenu TIN. Zastepuje wlasny plaski teren td.",
    "// Pisany przez tools/terraincook/makepoligon.py - nie poprawiac recznie.",
    "//",
    "// Plat 3400 x 4400 m wokol toru, a od z = 3000 korytarz 400 m szerokosci do z = 43000, czyli",
    "// 40 km - dwa razy wiecej niz zasieg wczytywania terenu, zeby bylo widac doladowywanie kafli.",
    "//",
    "// Trzy gestosci siatki, bo teren w sceneriach jest rysowany na trzy sposoby:",
    "//   duze trojkaty co 40 m         - tlo, jak z 3ds Maxa",
    "//   drobne co 4 m                 - x -200..200, z -120..400 przy torze, i z 20000..20400",
    "//   gesta siatka co 1 m jak z NMT - x -200..200, z 600..1000 oraz z 10000..10300",
    "// Granice materialow: prosta, skosna, okragla, a w gestym placie plamy co 6 m.",
    "// Sciana nasypu na x 160, z 100..300. Dziury w ziemi co 2 km wzdluz korytarza.",
    "// Gory na x -3000..3000, z 24000..34000: dolina posrodku, grzbiety do 400 m.",
    "// Wielkie pasmo na x -12000..12000, z 88000..112000, szczyty do 2000 m - do ogladania ze 100 km.",
    "// Korytarz siega z = 120000, czyli 120 km od toru.",
    "",
]


def emit( triangle, name ):
    lines.append( "node -1 0 none triangles " + name )
    for index, ( x, y, z ) in enumerate( triangle ):
        tail = " end" if index < 2 else ""
        lines.append( "{:.3f} {:.3f} {:.3f} 0 1 0 {:.4f} {:.4f}{}".format(
            x, y, z, x / TILING, z / TILING, tail ) )
    lines.append( "endtri" )


def quad( x0, z0, step ):
    x1, z1 = x0 + step, z0 + step
    if hole( x0 + step * 0.5, z0 + step * 0.5 ):
        return
    corner = { ( cx, cz ): ( cx, height( cx, cz ), cz ) for cx in ( x0, x1 ) for cz in ( z0, z1 ) }
    name = material( x0 + step * 0.5, z0 + step * 0.5 )
    emit( [ corner[ ( x0, z0 ) ], corner[ ( x0, z1 ) ], corner[ ( x1, z1 ) ] ], name )
    emit( [ corner[ ( x0, z0 ) ], corner[ ( x1, z1 ) ], corner[ ( x1, z0 ) ] ], name )


def lay( x0, x1, z0, z1, step, skip = () ):
    # each patch is a rectangle on the coarse grid, and the coarse ground is not laid where a finer
    # patch takes over. Where two densities meet the source has a T-junction, and so the cook
    # faithfully has one - which is what scenery drawn in pieces looks like
    z = z0
    while z < z1 - 0.001:
        x = x0
        while x < x1 - 0.001:
            if not any( inside( x + step * 0.5, z + step * 0.5, rectangle ) for rectangle in skip ):
                quad( x, z, step )
            x += step
        z += step


FINEA = ( -200.0, 200.0, -120.0, 400.0 )
FINEB = ( -200.0, 200.0, 20000.0, 20400.0 )

lay( FIELDX0, FIELDX1, FIELDZ0, FIELDZ1, COARSE, ( FINEA, ) + DENSEPATCHES )
lay( FINEA[ 0 ], FINEA[ 1 ], FINEA[ 2 ], FINEA[ 3 ], FINE )
lay( CORRIDORX0, CORRIDORX1, FIELDZ1, CORRIDORZ1, COARSE, ( FINEB, MOUNTAINS, FARRANGE ) + DENSEPATCHES )
lay( MOUNTAINS[ 0 ], MOUNTAINS[ 1 ], MOUNTAINS[ 2 ], MOUNTAINS[ 3 ], COARSE )
# the far range is drawn with cells of its own size: it is never seen from near, and a hundred
# thousand triangles there would be a hundred thousand nobody ever looks at closely
lay( FARRANGE[ 0 ], FARRANGE[ 1 ], FARRANGE[ 2 ], FARRANGE[ 3 ], 120.0 )
lay( FINEB[ 0 ], FINEB[ 1 ], FINEB[ 2 ], FINEB[ 3 ], FINE )
for patch in DENSEPATCHES:
    lay( patch[ 0 ], patch[ 1 ], patch[ 2 ], patch[ 3 ], DENSE )

# an embankment side: upright triangles, which are ground as much as anything else
step, wallheight, wallx = 8.0, 6.0, 160.0
z = 100.0
while z < 300.0 - 0.001:
    z2 = z + step
    emit( [ ( wallx, height( wallx, z ), z ), ( wallx, height( wallx, z2 ), z2 ),
            ( wallx, height( wallx, z2 ) + wallheight, z2 ) ], "concrete1" )
    emit( [ ( wallx, height( wallx, z ), z ), ( wallx, height( wallx, z2 ) + wallheight, z2 ),
            ( wallx, height( wallx, z ) + wallheight, z ) ], "concrete1" )
    z += step

out = os.path.join( sys.argv[ 1 ], "scenery", "td", "tinpoligon.scm" )
open( out, "w" ).write( "\n".join( lines ) + "\n" )
print( "{}: {} triangles".format( out, sum( 1 for line in lines if line.startswith( "node" ) ) ) )
