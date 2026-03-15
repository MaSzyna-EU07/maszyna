
# File structure
| Directory| Description|
|--|--|
| McZapkie/ | Train Physics Engine |
| gl/ |  OpenGL Abstraction Layer|
| network/ |  Multiplayer Support
| launcher/ |  Scenario/Vehicle Selection (built in starter)|
| widgets/ |  HUD Elements |
| vr/ |  Virtual Reality Support |
| Console/ |  Hardware I/O |
| application/ | Core simulation loop, time, environment, state serializer, sounds |
| scene/ | Scene graph, scene nodes, scene editor, utilities |
| rendering/ | All renderers (OpenGL legacy and full), geometry banks, particles, lights, frustum |
| audio/ | Sound, audio, and audio renderer |
| environment/ | Sky, sun, moon, stars, skydome | 
| application/ | Application modes, UI layers/panels, driver/editor/scenario modes | 
| input/ | Keyboard, mouse, gamepad, command, messaging, ZMQ input |
| scripting/ | Lua, Python, ladder logic, screen viewer |
| model/ | 3D models, materials, textures, resource manager |
| utilities/ | Logging, timing, parsing, math, globals, crash reporter |
| world/ | Tracks, traction, events, memory cells, stations | 
| vehicle/ | Train, dynamic objects, driver, gauges, camera |
| entitysystem/ | EnTT wrapper, components, helpers |
