# Bones of the vehicle models

A vehicle's `.mmd` can pin submodels of its model to points that move, and turn them to keep facing another such
point: the rods of a steam locomotive, which go round with the cranks of the wheels and slide with the crossheads,
and the couplers, which follow the hook of the next vehicle round a curve. The math is in `vehicle/bones.h` /
`bones.cpp`, the vehicle runs it in `TDynamicObject::update_bones()` along with the wheels. A model without these
entries is drawn as before.

## The entries

In the `models:` section, after `animwheelprefix:`, the points before the bones and before the points sliding with
them:

```
bonepoint: <name> fixed <x> <y> <z>
bonepoint: <name> crank <wheel submodel> <x> <y> <z>
bonepoint: <name> slide <driving point> <x> <y> <z> <guide dx> <dy> <dz>
bonepoint: <name> coupler <front|rear> <x> <y> <z>
bonepoint: <name> mid <point> <point>
bonechain: <name> <start point> <end point> <nodes> <x> <y> <z> (of each node)
bone: <submodel> <anchor point> <point it faces, or none>
```

- The coordinates are those of the model file: x across, y along the vehicle (the front, `coupler1`, at +y), z up.
  Each position is where the point is in the model as made: the wheels at their angle 0, the next vehicle straight
  in line.
- `fixed`: a point of the body.
- `crank`: a point going round with the wheel submodel (one of `animwheelprefix:`), about its axle the way the wheel
  turns: a crank pin.
- `slide`: a point on a guide through its position, in the direction given, which keeps the distance it has from
  the driving point as made: a crosshead, the length of the connecting rod kept.
- `coupler`: where the next vehicle's hook is, given for the two vehicles standing straight in line (as far past
  this vehicle's end as the hook is inside the other's). Coupled, it follows the other vehicle round a curve, and its
  buffers pressed in or pulled out; not coupled, it stays where it was made.
- `mid`: halfway between two points defined before it: where the hoses of two vehicles meet.
- `bonechain`: a chain hanging between two points, a rubber hose. Its nodes, given where they are in the model, are
  the points `<name>.1`, `<name>.2` and so on, which bones can use but other points can't. The pieces between the
  nodes keep their lengths; the nodes swing with gravity and their inertia (Verlet integration in the frame of the
  world, in substeps of at most 1/240 s, the swing dying down by half in about half a second), with the ends pinned
  where their points are. So a hose swings when the vehicle jerks, brakes or rocks on uneven track, and sags as the
  buffers are pressed in. Made of pieces, a submodel each, it bends at the nodes.
- `bone`: the submodel moves with its anchor point and turns about it to keep facing the other point. With `none`
  it only moves: a coupling rod, which stays level. The submodel's own origin and axes don't matter.

## Examples

The rods of one side of a two cylinder locomotive, the connecting rods driving the third coupled axle (`wheel4`),
the crank of 330 mm over the axle at the wheel angle 0 (the placeholder `pkp/ty2_v1`):

```
bonepoint: czop_p crank wheel4 1.0 -1.3 1.03
bonepoint: wodzik_p slide czop_p 1.06 2.7 1.0 0 1 0
bone: wiazar_p czop_p none
bone: korbowod_p czop_p wodzik_p
bone: wodzik_p wodzik_p none
```

The screw couplings of an EU07, hanging from the pin of the hook and reaching over the next vehicle's hook:

```
bonepoint: hak1_sworzen fixed 0 7.45 1.03
bonepoint: hak1_sasiada coupler front 0 8.30 1.03
bone: coupler1_on hak1_sworzen hak1_sasiada
bonepoint: hak2_sworzen fixed 0 -7.45 1.03
bonepoint: hak2_sasiada coupler rear 0 -8.30 1.03
bone: coupler2_on hak2_sworzen hak2_sasiada
```

A brake hose hanging from its cock to where it meets the next vehicle's, in four pieces (`waz_1` to `waz_4`):

```
bonepoint: kurek fixed 0.45 -6.35 0.95
bonepoint: kurek_tendra coupler rear 0.45 -7.05 0.95
bonepoint: spotkanie mid kurek kurek_tendra
bonechain: waz kurek spotkanie 3 0.45 -6.43 0.65 0.45 -6.53 0.55 0.45 -6.62 0.65
bone: waz_1 kurek waz.1
bone: waz_2 waz.1 waz.2
bone: waz_3 waz.2 waz.3
bone: waz_4 waz.3 spotkanie
```

## Still to do

- The valve gear (Heusinger / Walschaerts): the expansion link swung by the eccentric rod, the radius rod lifted by
  the reverser (`TAnimValveGear` holds the old, never finished figures for the Pt47). It needs a point turning on a
  pivot (a link) and a point at the crossing of two rods, besides those above.
- A hose bending smoothly rather than at its nodes: skinning, the vertices of one submodel weighted to several
  bones, in the shaders of the OpenGL 4.6 and Vulkan renderers. The chains stay as they are, only the drawing
  changes. The hoses of the existing models (`pneumatic*`) are one piece each and would need it, or cutting up.
- A screw coupling hanging free (not coupled) as a pendulum on its hook, and its sag with the buffers pressed in.
- The old entries `animpistonprefix:`, `animconrodprefix:`, `animreturnprefix:` and `pistonfactors:` are still read
  and do nothing, as before.
