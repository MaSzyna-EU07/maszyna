# Track unevenness

An optional attribute of a track entry, before `endtrack`:

```
node -1 0 tor1 track normal 100.0 1.435 0.15 25.0 20 0 flat vis
rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1
0.0 0.2 0.0 0.0
0.0 0.0 0.0
0.0 0.0 0.0
0.0 0.2 100.0 0.0
0.0
uneven 18;;;25;6
endtrack
```

The numbers are in mm, 0-100, separated by semicolons with no spaces:

| position | `normal` track                                | `switch`                 |
|----------|-----------------------------------------------|--------------------------|
| 1        | peak deviation of a rail on a 10 m chord      | the same, path p1-p2     |
| 2        | vertical                                      | path p3-p4               |
| 3        | sideways                                      | path p1-p2, vertical     |
| 4        | cross level                                   | path p1-p2, sideways     |
| 5        | dip at rail joints, by default 0.4 of vertical | path p3-p4, vertical     |
| 6        |                                               | path p3-p4, sideways     |

A position left out takes the first number, and the dip follows the vertical value. Positions at the end can be dropped. Sideways alone is `uneven 0;;30`, and dips at the joints alone are `uneven 0;;;;6`. A switch with no components of its own on path p3-p4 takes the values of path p1-p2. A switch for 100 km/h on the straight path and 40 km/h on the diverging one is, for example, `uneven 14;35`.

All of them are also in the editor, in the track panel, section "Ride and sound". On roads, rivers and turntables they are ignored.

## Choosing a value

`uneven` corresponds to the "vertical irregularities" column of the PLK Id-14 instruction, appendix 1 (class C):

| speed [km/h]  | 20 | 40 | 60 | 80 | 100 | 120 | 160 |
|---------------|----|----|----|----|-----|-----|-----|
| `uneven` [mm] | 50 | 35 | 25 | 18 | 14  | 10  | 6   |

According to the instruction, a track in good condition is half of this value (class A), and one in fair condition 3/4 (class B).

At these limits a pantograph may not keep up at speed, which trips the main breaker: see the end of "How it works". `tools/track_uneven_grade.py` keeps the values below that.

A track with `uneven 18` measures as follows (`tests/unit/test_track_uneven.cpp`), against the Id-14 limit for 80 km/h:

| parameter                | result | Id-14     |
|--------------------------|--------|-----------|
| vertical irregularities  | 18 mm  | 18 mm     |
| horizontal irregularities | 19 mm | 17 mm     |
| twist on a 5 m base      | 15 mm  | 16 mm     |
| gauge                    | ±10 mm | +10/-8 mm |
| cross level              | 20 mm  | 20 mm     |
| J index                  | 6.1    | 5.3       |

## How it works

Unevenness is a displacement field that depends on the position on the map, made of waves 5-42 m long. Vertically there are fewer short waves, so the rails come near the peak value more often, and there are also waves of 47-83 m: they hardly count on the 10 m chord, the body rises and falls on them, and a pantograph keeps up with them. Nothing in it is random: one place bends the same way on every run. A track that is moved gets the waves of its new place.

Each component joins separately:

- **Track to track:** the lower of the two values applies at the joint, and each track comes to it smoothly over 20 m. A free end of a track stays as designed.
- **Switch:** each path has its own value, and the lower of the two applies at their common end. Directly joined switches take the lowest value of the same path, and where two paths meet, the lower of the two. A joined track comes to the value of the switch at the end it meets, even if that is higher than its own. A track with none of a component keeps the whole switch as designed in it.
- **Joints:** the dips come every `fSoundDistance`, over 6 m, and run on through following tracks with the same spacing. The wheel clatter falls where the dip is. With a spacing of 6 m or less, and on switches, there are no dips.
- **Cross level:** adds to the designed cant. It is a tilt of the map plan, so tracks laid towards each other meet at the same cant. On switches there is only the part coming from the waves, with no separate settling of the rails.

The ends of joined paths meet in the middle between them, with one vector across, if they are less than 20 cm apart. This closes the cracks from coordinates that don't match in the data, on tracks with no unevenness as well. At such a joint a switch takes the cant of the plain track, and two tracks or two switches meet halfway. This applies to the physics too.

The rails, the top of the ballast, `sleepermodel`, the axles, the roll of the body and the cab shake follow the unevenness. The slope of the ballast takes up the difference, and its foot stays as designed, on the ground.

Models from includes (e.g. switch drives and timbers), the terrain and the catenary don't follow it, so at large values a rail shifts away from such a model by a few centimetres.

On a track with unevenness the track itself shakes the cab, in place of two of the sources of shake that were there before. It goes by the track under the vehicle, and a track with only dips at its joints counts too. On tracks with no unevenness everything works as before.

| source of the cab shake                                            | on a track with unevenness |
|--------------------------------------------------------------------|----------------------------|
| side sway from the lateral acceleration: curve radius and cant, jolts sideways on a damaged switch | off |
| random jolts over 25 km/h                                          | off                        |
| the track itself, from the bend under the axles                    | added                      |
| vertical jolts at the joints of a damaged track                    | stays                      |
| fore and aft, when accelerating and braking                        | stays                      |
| diesel engine vibration                                            | stays                      |
| hunting (`huntingspring`), set by the vehicle                      | stays                      |

This is about the cab only. The physics, derailment included, and the noise of the wheels still use the lateral acceleration.

A pantograph rises at the rate set by the pressure, about 0.14 m/s, and a gap to the wire over 1 cm cuts the voltage and trips the main breaker. Unevenness adds to this at high speed. On the tracks of Bałtyk a simulation gives a gap of 0.6 cm when the vertical value times the speed in km/h is 925, for example 8 mm at 120 km/h, 13 mm at 70 km/h and 23 mm at 40 km/h. The values in the Id-14 table are higher than that. The wire alone can do it too: at `tor0290` on Bałtyk it rises by about 1%, so a pantograph keeps up to about 46 km/h there, and above that the breaker trips on a straight track as well.

The physics takes the curve radius and the grade from the design, so unevenness doesn't change them. It adds only the cross level towards derailment, about 0.06 m/s² at `uneven 18` against a threshold of about 3.2 m/s².

## Grading the tracks of any scenery

`tools/track_uneven_grade.py` works out the values from what the scenery shows: ballast and rail textures, grass on the track, curves, speed and road crossings. It reads a `.scn`, `.scm` or `.inc` file with its includes and leaves the originals as they are, writing copies of the files with tracks to `--out`. With no `--out` it only prints a report.

```
python tools/track_uneven_grade.py scenery/lk357.scn --out graded
python tools/track_uneven_grade.py scenery/baltyk_skm1.scn --out graded --route -107,2300 -4120,2491
```

- `--route X,Z X,Z`: tracks get worse from the first point to the second, measured along the tracks.
- `--gain`: multiplier of the state of the tracks, 1 by default.
- `--near`: the share of tracks on wooden sleepers and at changes of curve that go nearer the Id-14 limit, 0.6 by default. The choice is fixed.
- `--straight`: names of tracks with no unevenness, separated by commas.
- `--root`: the `scenery` folder, when the given file lies deeper.

The values stay under what a pantograph can follow, by a simulation over those tracks, but the rise of the wire itself isn't counted. Texture wear is read from the names, by the tables at the top of the script, and an unknown name gets 0.5. Tracks in `.inc` files taking parameters are skipped and `rotate` blocks aren't applied; the report says so in both cases. A track that already has unevenness stays as it is.

## Performance

A track with unevenness has geometry every 2 m instead of every 10 m on straights. Switches keep their division.

Measured (camera moving or in the cab, 1920×1080):

| scene                               | no flags              | flags 18/35            |
|-------------------------------------|-----------------------|------------------------|
| Bałtyk, 56 km of tracks: vertices   | 0.56 M (about 27 MB)  | 2.17 M (about 104 MB)  |
| Bałtyk: longest frame in a fly-through | 31.5 ms            | 64 ms                  |
| `td.scn`, driving at 120 km/h: frame | 2.73 ms              | 2.93 ms                |

The cost grows with the number of flagged tracks, and the longest frames occur when the geometry of further sections is built. Driving on such a track costs practically nothing: about 0.3 µs per axle in a physics step.
