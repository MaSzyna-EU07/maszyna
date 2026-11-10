# The black box

A binary recording of a whole session: the map of the scenery (the rail tracks, the signals, the stop
points W4 and the horn points W6), every vehicle over time, the switches and the aspects of the signals as
they change, and the train the user is in, in detail. The detail includes what the AI driver would have
done at the same moment, every command the user gave, when the autopilot took over, the signals passed,
the timetable and how the train kept to it at each station, and where it was told to sound the horn. From it another program (the
starter, a viewer) can replay the session on the map, or set the user's driving against the AI's.

It is written along with the physics logs: `physicslog yes` in `eu07.ini`, not in an editor session.
`blackbox no` turns it off while the physics logs are kept (it is on by default). The file goes next to
them, `physicslog/<scenery>_<YYYY-MM-DD_HH-MM-SS>.blackbox` (the local time the recording started at).
The `.dat` logs stay as they are.

The recording starts once the scenery is loaded and ends when the simulator closes. Going into the
editor (F11) ends it too, since the editor may take away the tracks and the vehicles it refers to; no
new one starts when driving goes on after that.

It is written as the session goes and flushed once a second, so the file can be read while the simulator
still writes it, and after a crash all but the last second is there. A file without the `end` record
was cut short.

## Layout

Little-endian throughout. The types:

| type    | bytes | what                                                                      |
|---------|-------|---------------------------------------------------------------------------|
| `u8`    | 1     | unsigned                                                                  |
| `i8`    | 1     | signed                                                                    |
| `u16`   | 2     | unsigned                                                                  |
| `u32`   | 4     | unsigned                                                                  |
| `f32`   | 4     | IEEE 754 single                                                           |
| `f64`   | 8     | IEEE 754 double                                                           |
| `vec3`  | 12    | `f32 x, f32 y, f32 z`                                                     |
| `dvec3` | 24    | `f64 x, f64 y, f64 z`                                                     |
| `text`  | 1+    | UTF-8, after its length in bytes in 7 bit groups, low first, the high bit set on all but the last |

This is what .NET `BinaryReader` reads: `ReadByte`, `ReadSByte`, `ReadUInt16`, `ReadUInt32`,
`ReadSingle`, `ReadDouble` and `ReadString`.

Positions are in the frame of the scenery: metres, `y` up, the same as in the `.scn` files.

The file starts with a header of 12 bytes:

| offset | type     | value                         |
|--------|----------|-------------------------------|
| 0      | 8 bytes  | `EU07BBX` and a zero byte     |
| 8      | `u16`    | version of the format, now 1  |
| 10     | `u16`    | reserved, 0                   |

Then come the records, to the end of the file. Each record is:

| type  | what                                  |
|-------|---------------------------------------|
| `u8`  | the kind of the record (below)        |
| `u32` | length of the content, in bytes       |
|       | the content                           |

A reader skips a record of a kind it doesn't know, and the content past the fields it knows (later
versions may add fields at the end of a record). A record whose length runs past the end of the file is
still being written: stop there and read on from it later.

`t`, the time of a record, is an `f64` of seconds of simulation since the recording started. It doesn't
run while the simulation is paused. The `clock` records give the time of day it corresponds to.

Vehicles are given numbers from 1 up, in the `vehicle` records; 0 means no vehicle. Tracks, signals,
stop points and horn points are given numbers from 0 up, in the order of their `track`, `signal`,
`stop point` and `horn point` records.

## Records

### 0x01 info

Once, first.

| type   | field                                                              |
|--------|--------------------------------------------------------------------|
| `text` | scenario file, as the simulator was given it (the starter gives `$<name>.scn`) |
| `text` | local time the recording started at, `YYYY-MM-DDTHH:MM:SS`         |
| `u16`  | simulation year at the start                                       |
| `u16`  | simulation day of the year, 1..366                                 |
| `f64`  | simulation second of the day at the start                          |
| `text` | the vehicle the user was to start in, `ghostview` for none         |
| `text` | version of the simulator                                           |
| `f32`  | seconds between the `cab` records (0.1)                            |
| `f32`  | seconds between the `vehicles` records (1)                         |
| `f32`  | seconds between the `vehicles` records which list all the vehicles (60) |

### 0x10 track

The map: one for each rail track, two for a switch, one for each of its ways. Written at the start.

| type   | field                                                     |
|--------|-----------------------------------------------------------|
| `u32`  | number of the track                                       |
| `text` | name, may be empty                                        |
| `u8`   | kind: 0 a plain track, 1 a way of a switch                |
| `u8`   | the switch position this way is taken in (0 for a plain track) |
| `u8`   | category: 1 rail                                          |
| `u32`  | count of the points                                       |
| `vec3` | the points, along the track, as many as counted           |

### 0x11 signal

The map: one for each signal. Written at the start.

| type    | field                                              |
|---------|----------------------------------------------------|
| `u32`   | number of the signal                               |
| `text`  | name (that of its memory cell, without `_sem_mem`) |
| `dvec3` | position                                           |

### 0x14 stop point

The map: one for each stop point of a passenger train (W4), a `putvalues` event with the text
`PassengerStopPoint:<station>` (or a memory cell with that text). Written at the start.

| type    | field                                                                         |
|---------|-------------------------------------------------------------------------------|
| `u32`   | number of the stop point                                                      |
| `text`  | name of the station as the scenery gives it; what follows a `#` tells the stop points of one station apart |
| `dvec3` | position                                                                      |
| `f32`   | where the train is to stop, the first value of the cell: below 0 its head that many metres before the W4, above 0 the middle of the train that many metres before it |
| `f32`   | the second value of the cell: above 0 the length of the platform [m], below 0 the stop is for trains shorter than that (W32), 0 not given |
| `f32`   | heading of the trains it is for [rad, from +z towards +x], as the track it is read off gives it: the driver reads the events of the second end of a track going from its first end to the second; NaN on no track, or on tracks of both ways |

### 0x15 horn point

The map: one for each event telling the trains to sound the horn (W6), a `putvalues` event with the text
`Warning_signal`. Written at the start.

| type    | field                                     |
|---------|-------------------------------------------|
| `u32`   | number of the horn point                  |
| `text`  | name of the event                         |
| `dvec3` | position                                  |
| `f32`   | how long the horn is to sound [s]         |
| `f32`   | the tone, bits: 1 low, 2 high             |

### 0x12 vehicle

Every vehicle present at the start, and each one which shows up later, before it appears in any other
record.

| type   | field                                                    |
|--------|----------------------------------------------------------|
| `u32`  | number of the vehicle                                    |
| `text` | name                                                     |
| `text` | type                                                     |
| `text` | train number of its timetable, if it leads a train with one |
| `f32`  | length [m]                                               |
| `f32`  | width [m]                                                |
| `u8`   | category: 1 rail, 2 road                                 |

### 0x13 vehicle removed

| type  | field                 |
|-------|-----------------------|
| `f64` | t                     |
| `u32` | number of the vehicle |

### 0x20 cab

The vehicle the user is in, every 0.1 s, also while the user looks around outside it. 136 bytes.

| type    | field                                                                  |
|---------|------------------------------------------------------------------------|
| `f64`   | t                                                                      |
| `u32`   | number of the vehicle                                                  |
| `dvec3` | position                                                               |
| `f32`   | speed [km/h], positive along the front of the vehicle                  |
| `f32`   | acceleration along the track [m/s²]                                    |
| `f32`   | odometer [km]                                                          |
| `u8`    | driver: 0 none, 1 the user, 2 the AI (the autopilot)                   |
| `u8`    | flags, below                                                           |
| `i8`    | master controller position                                             |
| `i8`    | secondary controller (field shunting) position                         |
| `i8`    | reverser: -1, 0, 1                                                     |
| `f32`   | driver's brake valve position                                          |
| `f32`   | local brake position, 0..1                                             |
| `f32`   | brake cylinder pressure [bar]                                          |
| `f32`   | brake pipe pressure [bar]                                              |
| `f32`   | motor current [A]                                                      |
| `f32`   | AI: the speed it would hold here [km/h]                                |
| `f32`   | AI: the next speed limit it sees [km/h], -1 none                       |
| `f32`   | AI: distance to that limit [m], -1 none                                |
| `f32`   | AI: the acceleration it would go for [m/s²], below 0 it would brake    |
| `f32`   | AI: braking distance at this speed [m]                                 |
| `f32`   | top speed of the timetable [km/h], -1 none                             |
| `f32`   | top speed of the track [km/h], -1 none                                 |
| `u32`   | what is switched on, below                                             |
| `f32`   | main reservoir pressure [bar]                                          |
| `f32`   | voltage given to the motors [V]                                        |
| `f32`   | revolutions of the engine or the motors [rpm]                          |
| `u16`   | lights lit at the head of the train, below                             |
| `u16`   | lights lit at its tail                                                 |
| `i8`    | the cab the user is in: 1 the front one, -1 the rear one, 0 none       |
| `u8`    | the train protection, below                                            |
| `f32`   | acceleration across the track [m/s²], in the curves                    |
| `u8`    | what the driver of the train goes by: 1 a train run, 2 shunting, 0 anything else or no driver |
| `i16`   | the light pattern the timetable or the scenery asks for at the head, bits as the lights; -1 the usual one |
| `i16`   | the same at the tail                                                   |
| `u16`   | the lamps the vehicle at the head has at its outer end, bits as the lights |
| `u16`   | the same of the vehicle at the tail                                    |

The lights the train is to show follow from the movement and the two patterns after it: in a train run the head the pattern asked for,
the usual one Pc 1 (both headlights and the upper one; Pc 2 is the left red marker with the right and the
upper headlight), and the tail the pattern asked for, the usual one Pc 5 (both red markers); shunting
Tb 1, one headlight at each end. A pattern is checked against the lamps the vehicle has, as the driver
checks it: one without the upper headlight shows Pc 1 with the two others, one with auxiliary lamps may
show them instead of the headlights, and one with no lamps at all the rear-end signals; the tail shows
Pc 5 with the red markers or the rear-end signals it has. A recording made before the movement ends at
the acceleration across, and one made before the lamps at the light pattern of the tail.

The flags:

| bit  | meaning                                                         |
|------|-----------------------------------------------------------------|
| 0x01 | the user is in the cab                                          |
| 0x02 | the AI fields hold what the AI would do: the consist is started |
| 0x04 | the alerter or the cab signalling blinks                        |
| 0x08 | the alerter or the cab signalling beeps                         |
| 0x10 | the wheels slip                                                 |
| 0x20 | radio-stop                                                      |
| 0x40 | the alarm chain was pulled                                      |

What is switched on: the radio, the horn, the doors and the spring brake are those of the
vehicle the user is in; the battery is on if it is in either; the rest, and the pressures, the voltage and the revolutions, of the vehicle
whose motors the cab drives (the motor car of a unit).

| bit    | meaning                              |
|--------|--------------------------------------|
| 0x0001 | battery                              |
| 0x0002 | main breaker closed                  |
| 0x0004 | front pantograph raised              |
| 0x0008 | rear pantograph raised               |
| 0x0010 | converter                            |
| 0x0020 | compressor                           |
| 0x0040 | line contactors: the motors are on   |
| 0x0080 | overload relay tripped               |
| 0x0100 | sanding                              |
| 0x0200 | horn                                 |
| 0x0400 | left doors open                      |
| 0x0800 | right doors open                     |
| 0x1000 | radio                                |
| 0x2000 | train heating                        |
| 0x4000 | spring brake applied                 |
| 0x8000 | shunting mode                        |

The lights: 0x001 left headlight, 0x002 left red marker, 0x004 upper headlight, 0x010 right
headlight, 0x020 right red marker, 0x040 rear-end signals, 0x080 left and 0x100 right auxiliary
light, 0x200 left and 0x400 right high beam. Those of the head are at the outer end of the first vehicle
of the train, those of the tail at the outer end of the last one, whichever vehicle the user is in.

The train protection, of the vehicle whose motors the cab drives; the alerter (CA) and the cab signalling
(SHP) apart:

| bit  | meaning                                                                 |
|------|-------------------------------------------------------------------------|
| 0x01 | the alerter lamp blinks: the driver is to acknowledge                   |
| 0x02 | the cab signalling lamp blinks: an SHP magnet was passed                |
| 0x04 | the alerter beeps                                                       |
| 0x08 | the cab signalling beeps                                                |
| 0x10 | not acknowledged in time: the train protection brakes the train         |

The AI fields are what the AI driver works out from the place and the speed the train has at the time.
It works them out as it goes, also while the user drives, but doesn't act on them then. It is not a
separate run of the AI. They are refreshed every half a second or so, and only once the consist is
started (flag 0x02); before that the speed held, the acceleration and the braking distance are 0, the
next limit, its distance and the top speeds of the timetable and the track -1.

### 0x21 vehicles

Every second: the vehicles which moved more than 0.1 m, changed speed or got another driver since they
were last listed. Every 60 s all of them (a keyframe), so a reader can start at any keyframe.

| type  | field                                     |
|-------|-------------------------------------------|
| `f64` | t                                         |
| `u8`  | 1 if a keyframe, else 0                   |
| `u32` | count of the vehicles                     |
|       | as many times, 25 bytes:                  |
| `u32` | number of the vehicle                     |
| `vec3`| position                                  |
| `f32` | heading [rad] of its front, from +z towards +x: the front points to (sin, cos) in (x, z) |
| `f32` | speed [km/h], positive along its front    |
| `u8`  | driver: 0 none, 1 a user, 2 the AI        |

### 0x22 switch

The position of a switch: all of them at the start, then as they change (checked every second).

| type  | field                                                         |
|-------|---------------------------------------------------------------|
| `f64` | t                                                             |
| `u32` | number of the track                                           |
| `u8`  | position: the way taken is the `track` record with this branch |

### 0x23 signal aspect

What a signal tells: all of them at the start, then as they change (checked every second). The values
are those of its memory cell.

| type   | field                                                          |
|--------|----------------------------------------------------------------|
| `f64`  | t                                                              |
| `u32`  | number of the signal                                           |
| `f32`  | speed at the signal [km/h]: 0 stop, -1 no limit                |
| `f32`  | speed at the next signal [km/h]                                |
| `text` | command of the memory cell, e.g. `SetVelocity`, `ShuntVelocity` |

### 0x24 timetable

The timetable of the train the user is in: once the user is in a train which has one, and again whenever
it changes, the user in another train or the train given its next timetable (one recording may hold any
number of them). It is written before the first station record (`0x35`) under it, even when the train
comes to that station at the moment it gets the timetable. The times are minutes of the day (10:05 is 605),
-1 none.

| type   | field                                                   |
|--------|---------------------------------------------------------|
| `f64`  | t                                                       |
| `u32`  | number of the vehicle the user was in                   |
| `text` | train number                                            |
| `text` | category                                                |
| `text` | from                                                    |
| `text` | to                                                      |
| `f32`  | top speed of the train [km/h]                           |
| `u32`  | count of the rows, the stations                         |
|        | as many times:                                          |
| `text` | station                                                 |
| `f32`  | kilometre                                               |
| `f32`  | top speed before the station [km/h], -1 none            |
| `f32`  | arrival                                                 |
| `f32`  | departure                                               |
| `u8`   | 1 a technical stop, else 0                              |

The rows are numbered from 1, as the `station` records give them.

### 0x25 camera

Where the user looks from: at the start, then when it changed, looked at twice a second. From the cab and
in the outside views the camera goes with the train, whose way is written anyway (0x20), so only going to
another mode, view or vehicle counts; the free and the debug camera count as changed as well when they
moved more than 0.5 m or turned more than 0.05 rad.

| type    | field                                                                        |
|---------|------------------------------------------------------------------------------|
| `f64`   | t                                                                            |
| `u8`    | mode: 0 from the cab, 1 an outside view of the user's train, 2 the free camera, 3 the debug camera |
| `u8`    | the outside view: 0 the front of the train, 1 its rear, 2 a bogie, 3 drive-by; 0 in the other modes |
| `u32`   | number of the vehicle the camera goes with, 0 for none                       |
| `dvec3` | position                                                                     |
| `f32`   | heading [rad] of where it looks, as that of a vehicle: towards (sin, cos) in (x, z) |
| `f32`   | pitch [rad], up from the horizon                                             |

### 0x30 command

Every command the user gave: each press and release of a control of the cab, of the keyboard, mouse or a
device. Held controls repeat their command while held; only the press and the release are written, the
`cab` records show the effect. Also the commands to the whole simulation, like the time jumps.

| type   | field                                                       |
|--------|-------------------------------------------------------------|
| `f64`  | t                                                           |
| `u32`  | number of the vehicle, 0 for a command to the simulation    |
| `u8`   | 1 pressed, 0 released                                       |
| `f64`  | first parameter                                             |
| `f64`  | second parameter                                            |
| `text` | name of the command, as in `eu07_input-keyboard.ini`        |

### 0x31 driver change

The user or the AI takes over a vehicle: the autopilot switched on (Shift+Q) or off (Q), and entering a
vehicle the AI was driving.

| type  | field                          |
|-------|--------------------------------|
| `f64` | t                              |
| `u32` | number of the vehicle          |
| `u8`  | driver now: 1 the user, 2 the AI |

### 0x32 signal passed

A train, any, passed a signal. It is noticed when the AI driver updates its table, up to half a second
later, so the train is somewhat past the signal by then: the position is that of the signal and the
overrun tells how far past it the train had got.

| type    | field                                                         |
|---------|---------------------------------------------------------------|
| `f64`   | t                                                             |
| `u32`   | number of the vehicle leading the train                       |
| `u32`   | number of the signal, `0xFFFFFFFF` if it isn't on the map     |
| `text`  | name of the event telling the aspect                          |
| `dvec3` | position of the signal                                        |
| `f32`   | the aspect when passed: speed at the signal [km/h], 0 stop, -1 no limit |
| `f32`   | speed at the next signal [km/h], as the signal shows it when the pass is noticed |
| `f32`   | speed of the train [km/h]                                     |
| `f32`   | overrun [m]                                                   |
| `u8`    | flags: 0x01 a shunting signal, 0x02 passed at stop, 0x04 the stop of an automatic block signal; a shunting signal binds only shunting, so passed at stop by a train it has 0x01 alone |

### 0x33 user vehicle

The vehicle the user is in changed, the user got out of the cab or back into it, or went to the other cab.

| type  | field                                       |
|-------|---------------------------------------------|
| `f64` | t                                           |
| `u32` | number of the vehicle, 0 for none           |
| `u8`  | 1 in the cab, 0 outside it                  |
| `i8`  | the cab: 1 the front one, -1 the rear one, 0 none |

### 0x34 clock

Every second, and so after a time jump too.

| type  | field                          |
|-------|--------------------------------|
| `f64` | t                              |
| `u16` | year                           |
| `u16` | day of the year, 1..366        |
| `f64` | second of the day              |

### 0x35 station

A train, any, at a station of its timetable, as its driver (the AI, or the AI watching the user drive)
sees it at the stop point (W4).

| type    | field                                                                    |
|---------|--------------------------------------------------------------------------|
| `f64`   | t                                                                        |
| `u32`   | number of the vehicle leading the train                                  |
| `u8`    | what happened, below                                                     |
| `u16`   | row of the timetable, from 1                                             |
| `text`  | station                                                                  |
| `dvec3` | position of the stop point                                               |
| `f32`   | distance of the head of the train before the stop point [m], below 0 past it |
| `f32`   | where the head was to stop: that many metres before the stop point       |
| `f32`   | length of the train [m]                                                  |
| `f32`   | the second value of the stop point (see `stop point`): the platform      |
| `f32`   | arrival of the timetable, in minutes of the day, -1 none                 |
| `f32`   | departure of the timetable                                               |

What happened:

| value | meaning                                                                                   |
|-------|-------------------------------------------------------------------------------------------|
| 1     | the train stopped at the stop point of a stop of its timetable                            |
| 2     | it passed a station where the timetable has no stop                                       |
| 3     | it left the stop point of a stop behind before it was time to go: without stopping, or leaving before the departure; only for a user's train |
| 4     | it was still at the stop point when it was time to go: it may leave                       |

How far the distance is from the place the head was to stop at tells how well the train was stopped.

### 0x36 horn request

A train, any, was told to sound the horn (it passed a horn point W6). Whether the user did is in the
`cab` records (the horn among what is switched on) and the `command` records.

| type    | field                                           |
|---------|-------------------------------------------------|
| `f64`   | t                                               |
| `u32`   | number of the vehicle leading the train         |
| `f32`   | how long the horn is to sound [s]               |
| `f32`   | the tone, bits: 1 low, 2 high                   |
| `dvec3` | position of the horn point, or of the train's head if not known |

### 0x37 coupling

A coupler of a vehicle, any: at the start those coupled, then each change, looked at every second. A
change is two vehicles coupled, uncoupled, or the hoses and the cables between them connected or not. Both
vehicles of a coupling see it; the one with the lower number tells it.

| type    | field                                                                  |
|---------|------------------------------------------------------------------------|
| `f64`   | t                                                                      |
| `u32`   | number of the vehicle                                                  |
| `u8`    | its coupler: 0 the front one, 1 the rear one                           |
| `u32`   | number of the vehicle coupled to it now, 0 for none                    |
| `u8`    | the coupler of that vehicle                                            |
| `u8`    | what is coupled now, below; 0 nothing                                  |
| `u8`    | what was coupled before                                                |
| `u32`   | number of the vehicle coupled before, 0 for none                       |
| `dvec3` | position of the coupler                                                |

What is coupled: 0x01 the coupler, 0x02 the brake pipe, 0x04 the control (multiple unit), 0x08 the high
voltage, 0x10 the gangway, 0x20 the main reservoir pipe, 0x40 the train heating, 0x80 a coupling not to be
undone in shunting (between the cars of a unit).

### 0x3F end

Last, when the simulator closes.

| type  | field |
|-------|-------|
| `f64` | t     |

## What it doesn't hold

- The kilometrage of the lines: it is known only in an editor session.
- Roads and rivers on the map, only the rail tracks.
- A run of the AI of its own: the AI fields tell what the AI would do from where the user's train is.

## Reading it

A minimal reader in C#:

```csharp
using var reader = new BinaryReader(File.Open(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite));
if (new string(reader.ReadChars(7)) != "EU07BBX" || reader.ReadByte() != 0)
    throw new InvalidDataException("not a black box");
ushort version = reader.ReadUInt16();
reader.ReadUInt16();

var stream = reader.BaseStream;
while (stream.Length - stream.Position >= 5)
{
    long start = stream.Position;
    byte kind = reader.ReadByte();
    uint length = reader.ReadUInt32();
    if (stream.Length - stream.Position < length)
    {
        stream.Position = start; // still being written, read on from here later
        break;
    }
    long next = stream.Position + length;
    switch (kind)
    {
        case 0x20:
            double t = reader.ReadDouble();
            uint vehicle = reader.ReadUInt32();
            double x = reader.ReadDouble(), y = reader.ReadDouble(), z = reader.ReadDouble();
            float speed = reader.ReadSingle();
            // ...
            break;
        // ...
    }
    stream.Position = next; // past what wasn't read, and past fields added later
}
```
