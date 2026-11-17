# Weather: clouds, fog and precipitation

The weather of a scenery is a set of cloud layers, a fog and a precipitation. The simulation keeps it
(`environment/weather.h`), the OpenGL 4.6 renderer draws it: volumetric clouds with their shadows on the ground, and a
fog with the shafts of the sun and the light of lamps and headlights in it (see `opengl46-renderer.md`, *Weather*).

A scenery describes it in a `weather` section. A scenery without one is drawn from its `atmo` section as before: the
overcast gives the clouds and the precipitation, the fog range the visibility. Whatever sets the old overcast or fog range
while the simulation runs (the sliders of the user interface, the `__simulation.weather` memory cell, the `setweather`
command of the network) does so too, and the clouds and the visibility follow it.

While the weather is described, the overcast and the fog range the rest of the simulation goes by (lamps turning on, the
sound and the look of the rain, friction, the `__simulation.weather` memory cell) are worked out from it.

## The weather section

```
weather
  metar "27015KT 3000 -RA BR BKN008 OVC015 08/06"
  visibility 12000
  haze 800
  groundfog 40 180
  fognoise 0.6
  clouds cumulus sct 1100 2600
  clouds cirrus 3 8500 9200 0.8
  precipitation rain 0.5
  cloudwind 250 12
  temperature 8
  elevation 310
  seed 7
endweather
```

Every entry is optional and they may come in any order; a later entry changes what an earlier one (or the report) set.
The section wins over an `atmo` section, wherever the two are in the scenery. Heights are in metres above the ground
reference (see `elevation`).

| Entry | Meaning |
|---|---|
| `location latitude longitude` | where the scenery lies, in degrees, for `climate`. Without it the place the editor keeps for the scenery (the origin of its aerial photos) is taken, and without that the middle of Poland |
| `climate [kind]` | the weather the place has in the month of the simulation (see *The weather of the climate*), in place of everything set before it; the entries after it change it. Give `location` and `seed` before it |
| `metar "report"` | a weather report (METAR) read first: the wind at the ground with its gusts (`27015G25KT`), the visibility, the present weather (rain, snow, drizzle, mist, fog, shallow and patchy fog, haze, showers, thunderstorms), the clouds with their heights in feet, the vertical visibility, the temperature and the dew point. The station, time, pressure, runway visual range and anything after `TEMPO`, `BECMG`, `NOSIG` or `RMK` are skipped. Reports name only the convective genera (`CB`, `TCU`); the others are taken from the height, the amount and the precipitation (low and broken: stratocumulus; low, overcast and precipitating: nimbostratus; very low: stratus; middle: altocumulus or altostratus; high: cirrus or cirrostratus) |
| `visibility m` | meteorological visibility at the ground: how far a dark object is still seen against the horizon (5% contrast) |
| `haze m` | how fast the haze thins out with the height: it is e times less dense this many metres higher. Low for a mist, high for a dusty or smoky air |
| `groundfog thickness visibility` | fog lying on the ground, this many metres thick, with this visibility inside it. It fills the valleys below the ground reference too. `0` removes it |
| `fognoise 0..1` | how patchy the ground fog is |
| `clouds genus amount base [top [density]]` | a layer of clouds (at most 4, the ones covering the most are kept). The genus: `cumulus` (`cu`), `towering_cumulus` (`tcu`), `cumulonimbus` (`cb`), `stratocumulus` (`sc`), `stratus` (`st`), `nimbostratus` (`ns`), `altocumulus` (`ac`), `altostratus` (`as`), `cirrus` (`ci`), `cirrostratus` (`cs`), `cirrocumulus` (`cc`). The amount: `few`, `sct`, `bkn`, `ovc`, or oktas 0..8. Without the top the layer gets the typical depth of the genus; the density multiplies the typical one (0..4) |
| `noclouds` | removes the layers set before (by a report, say) |
| `precipitation kind [intensity]` | `rain`, `snow`, `auto` or `none`, intensity 0..1. With `auto`, where the moisture of the air is described (a dew point, a humidity, a report with one, the climate), snow falls up to a wet-bulb temperature of 1 degree: in dry air at a few degrees above zero, in saturated air only just above it; without the moisture up to an air temperature of 1 degree, as the overcast above 1 always did. `rain` in a frost is freezing rain, which glazes the wires |
| `wind direction speed [gust]` | the wind at the ground: where it blows from (degrees from the north, clockwise), its mean speed and the speed of its gusts in m/s (without the gusts: half as much again). The wind of the simulation (the smoke, the grass, the rain, the drifting snow) follows it, wandering between lulls and gusts, and the clouds go the same way half as fast again. Without it (and without a report) the simulation keeps its light changing wind of up to 4 m/s |
| `cloudwind direction speed` | the wind at the height of the clouds: where it blows from (degrees from the north, clockwise) and its speed in m/s. The higher the layer, the faster it goes |
| `temperature degrees` | the air temperature, applied with the weather, the same all day |
| `temperaturerange lowest highest` | the lowest and the highest temperature of the day: the air temperature follows the time of the day between them, lowest about 5 and highest about 15 o'clock |
| `dewpoint degrees` | the dew point: the moisture of the air |
| `humidity percent` | the same given as the relative humidity at the temperature set before it |
| `elevation m` | world height (y) of the ground the heights of the weather are counted from. By default the ground under the camera where the simulation starts; give it for hilly sceneries, so the clouds and the valley fog stay at the same height whichever place the simulation starts at |
| `seed n` | another arrangement of the clouds |
| `snowcover depth [drift [loose]]` | snow lying from the start: its depth over open flat ground in metres, how far the wind has worked it into drifts (0..1, by default 0.5, the drifts turned to the wind at the ground of the section) and how much of it the wind can still carry (0..1, by default 0.5). Applied once: a later change of the weather keeps the snow as it lies then |

What each genus looks like:

| Genus | Drawn as |
|---|---|
| cumulus, towering cumulus | heaps with flat bases and rounded tops of different heights; towering ones a few kilometres tall |
| cumulonimbus | storm towers up to the tropopause, whose tops spread into anvils |
| stratocumulus, altocumulus | lumpy sheets; altocumulus higher, thinner and of smaller lumps |
| stratus, nimbostratus | even, grey sheets; nimbostratus thick and dark, the rain cloud |
| altostratus | a thin, even veil the sun shines through dimly |
| cirrus, cirrostratus, cirrocumulus | thin sheets of ice high up: strands drawn out along the wind, an even veil, small ripples |

## The weather of the climate

`climate kind` in the weather section (or in the text given to `__simulation.metar`), and the **Weather** window, make the weather
a place typically has in a month: the temperatures of the day, the dew point, the wind at the ground with its gusts, drawn from the
wind rose of the season, the clouds, the visibility, what falls and the snow lying. The kinds:

| Kind | Weather |
|---|---|
| `typical` | an average day of the month (the default) |
| `fair` | a high: little cloud, a wide range of the temperature, a light wind |
| `overcast` | a low grey sky, dry, the temperature nearly the same all day |
| `precipitation` | steady rain or snow from a thick layer |
| `showers` | passing showers from heaped clouds, gusty |
| `storm` | a thunderstorm: a cumulonimbus, heavy rain, strong gusts |
| `fog` | a still foggy day |
| `frost` | a winter high: clear, still, well below the average, from the east, over snow |
| `thaw` | mild wet air from the south-west over old snow, drizzle |
| `blizzard` | heavy snow in a strong wind over deep drifting snow |
| `gale` | a deep low passing: a very strong gusty wind from the west |
| `heat` | a hot spell |
| `random` | one of the above, each as often as the month has it: by its days with precipitation, fog and storms, its cloudiness, its snow |

The winter kinds come only in the months cold enough for them, the storms and the heat in the warm ones. `seed` before `climate`
makes the same day each time; without it it is another day each time.

The months are the climate normals of the weather station nearest to the scenery (within 150 km), read from
`climate/stations.txt` in the folder of the simulator. `tools/weather/imgw_normals.py` makes that file from the public monthly
measurements of the synoptic stations of IMGW-PIB for 1991-2020 (see the script for how; it needs the network once). Without the
file, or with no station near, Lodz-Lublinek stands in: the lowlands in the middle of Poland, away from the sea and the mountains.
Its temperatures built into the simulator are the 1991-2020 normals; its other elements are typical values of central Poland
until the file is made.

```
weather
  location 50.06 19.94
  seed 12
  climate blizzard
  visibility 400
endweather
```

## Snow lying

The snow falls, lies, drifts and melts with the weather (`environment/snowcover.h`, kept by the simulation):

- it piles up while it snows, about 7 cm an hour in the heaviest snowfall of a frost, less near zero, where the new snow is
  denser (by the wet-bulb temperature of the air it falls through); it settles as it lies, faster in the warmth, melts above
  zero (3 mm of water a degree and a day, more in the rain), and dry air and wind take a little of it straight off;
- the wind at the ground (the wind of the simulation, gusts included) carries the loose snow along the ground once
  it is over about 4 m/s for new dry snow, more for old, crusted or wet snow, and the more the stronger it is (the cube of the wind
  over the threshold). As long as it carries it, the drifts grow and turn to it; new snow falling in a calm evens them out;
- in a wind, the falling snow sticks to the faces turned to it (poles, signals, the fronts of the vehicles), best when it is wet,
  at a wet-bulb temperature about zero, and goes in the warmth and, slowly, in dry air;
- the ice on the wires grows too: the glaze of freezing rain, the rime of a freezing fog or of saturated air in the wind and a sleeve of wet snow, with
  its thickness and density. Nothing reads it yet; it is there for the look of the catenary and the contact of the pantographs.

The OpenGL 4.6 renderer draws the snow (`gfx.snow`, by default `drifts`):

- on every surface lit in the open, by how much it faces the sky and how deep the snow is: thin snow lies in patches on flat ground
  only, deeper snow on steeper slopes too. Nothing lies under a roof, a bridge or in a tunnel, inside the cabs and the interiors,
  on the water and on the grass, whose blades stick out of it;
- around the camera (as far as the maps of `gfx.skyvisibility` reach: 192 m at the medium level), from the shape of the scenery
  seen from above, how the wind worked the snow: the drifts in the lee of what stands in its way (as long as about 10 to 15 times
  the height of the obstacle, never higher than it), less in front of it with a gap swept at its foot, the hollows, ditches and
  cuttings filled, the crests and the slopes facing the wind blown bare; over the tracks the snow is kept 3 cm under the top of the
  rails, whose heads stay clean, the ploughs leave slopes beside them, and a road keeps a packed layer. Where the snow is deeper than
  a few centimetres it is a surface of its own over the ground, which buries what is lower than it (the foot of the poles, the
  sleepers, the grass, low fences);
- the snow is whiter when new and dry, greyer when old and wet, soiled over the tracks and roads; its crystals glint in the sun up
  close, the wind leaves ripples on the drifts and blows streaks along it in a strong wind.

`gfx.snow surfaces` leaves out the surface of the drifts, `gfx.snow no` all of it. Without `gfx.skyvisibility` the snow lies evenly
on the surfaces turned up, and there are no drifts. In the **Weather** window (and in the editor, **Weather and time**) the depth,
the drifts and the looseness of the snow can be set by hand, and the snow made to go faster than the simulation, to watch it pile up,
drift and melt.

## Changing the weather while the simulation runs

The `__simulation.metar` memory cell holds a report of the weather the simulation goes to. Setting its text (with an
`updatevalues` event, or from a script) changes the weather: the text is a report, or entries of the weather section
(`visibility 800 groundfog 60 300 clouds stratus ovc 150 400`); its value 1 is the time of the change in seconds. A text
of an event can't hold spaces: commas, semicolons or underscores separate the words as well
(`clouds,cumulonimbus,bkn,900`, `24015G25KT_4000_TSRA_BKN030CB`). The change goes smoothly: the layers of the same genera
change their sizes and heights, different ones fade out and in. A change of the old overcast or fog range takes over the
clouds or the fog at once, as it always did.

The weather can also be changed in the editor (**Weather and time**, *Clouds, fog and weather report*) and in the
**Weather** debug window of the renderer. These change the weather of this computer only; they are not sent over the
network.

## What the weather made of the old overcast looks like

| Overcast | Clouds |
|---|---|
| up to 0.5 | fair weather cumulus, more and taller with the overcast, and a high cirrus |
| 0.5 to 0.9 | a broken stratocumulus sheet and altocumulus above it |
| 0.9 to 1 | an overcast stratocumulus sheet and altostratus |
| above 1 | nimbostratus, lower and denser the heavier the precipitation, with cumulonimbus above 1.6 |

The visibility is 2.5 times the old fog range, divided by the overcast as the old fog was. The old fog thinned out
a dark object to 5% at 1.7 fog ranges, the new one at the visibility: the two cross about where the old fog took a
third.

The cloud model of the `sky` section (`Global.asSky`) and the model of the starry dome (`skydome_stars.t3d`) are no
longer loaded: the clouds come from the weather, the stars from a catalogue built into the simulator (the Yale Bright Star
Catalogue, 5th edition, about 9100 stars down to magnitude 6.5), turned with the sidereal time of the scenery.
