# Script screens

Screens drawn by scripts, alongside the Python screens (`pyscreen:`, `pydestinationsign:`): Lua scripts, run by
LuaJIT, and WebAssembly modules, run by the WAMR interpreter (`ref/wamr`). A script gets the same input a Python
screen gets (the table in `python_var_doc.md`), returns an RGBA image and, in the cab, may send commands.

A Lua script is text, kept with the vehicles and the `scripts/` folder like the Python screens, and changed with any
editor. A WebAssembly module is compiled from another language (Rust, C, Zig and the like).

Both run sandboxed: a script reaches only the functions of `eu07` below, and reads files only from its own folder and
below. A Lua script gets no `io`, `os`, `ffi`, `debug` or `require`, and no precompiled code is loaded.

Unlike a Python screen, which refreshes after a time (`python.updatetime`, `updatetime:`), a script screen refreshes
every so many rendered frames, set in `eu07.ini`. A call into a script may run at most 200 million instructions
(Lua runs without the JIT compiler so that the limit holds). A script that fails, or runs over, is switched off, and
the reason goes to the log.

## eu07.ini

| key                      | default | meaning                                                                        |
|--------------------------|---------|--------------------------------------------------------------------------------|
| `luascreens.enabled`     | 1       | 0: no Lua screens; the vehicles show what they did before                      |
| `wasm.enabled`           | 1       | 0: no WebAssembly screens; the vehicles show what they did before              |
| `screens.cabframes`      | 4       | a screen in the cab is refreshed every so many frames (1-1000)                 |
| `screens.externalframes` | 30      | a screen outside the cab (destination sign) is refreshed every so many frames  |
| `screens.gpupost`        | 1       | 0: `led_post` and `resize` of the Lua screens always run on the cpu, as they did before |

`wasm.cabframes` and `wasm.externalframes`, the first names of the last two, are read as well.

A screen may ask for a longer interval of its own (`frames:` below); the longer of the two counts. Screens sharing
an interval are spread over its frames, so they don't all refresh on the same one. A screen in the cab is refreshed
only while the player sits in that cab, a screen outside only while its vehicle is drawn.

## .mmd

In the cab section, next to `pyscreen:`:

```
luascreen: { ekran.lua target: screen01 frames: 6 parameters: id=1&mode=full }
wasmscreen: { ekran.wasm target: screen02 }
```

| key           | meaning                                                                                           |
|---------------|---------------------------------------------------------------------------------------------------|
| (first token) | the script, relative to the vehicle folder when it has no path of its own                         |
| `target:`     | the submodel the screen is drawn on (its texture is replaced); `none` for a screen that only sends commands |
| `frames:`     | the screen's own refresh interval, used when longer than `screens.cabframes`                      |
| `parameters:` | extra `key=value` pairs, `&`-separated, added to the input                                        |

A touch on the screen's submodel comes in the input as the list of points `touches`, as for the Python screens.

In `internaldata:`, next to `pydestinationsign:`:

```
luadestinationsign: { scripts/led_111a20.lua parameters: kod_e=IC&kod_p=TLK&kod_o=R instance: name }
destinationsignbackground: make:scripts/led_111a20.lua?dummy=1
```

`luadestinationsign:` and `wasmdestinationsign:` take the keys of `pydestinationsign:` (`instance:`, `parameters:`,
`background:`) and `frames:`. The input is what a Python sign gets: the timetable (`$timetable`), `_id1` (the
train), `_id2` (the instance) and the parameters, read anew on every refresh, so the sign can follow the journey. The
vehicles of one train showing the same request share one screen and one texture. A static destination texture, when
there is one, still comes first.

A blank sign (`background:`, or the older `destinationsignbackground:`) named `make:<script>.lua?<parameters>` or
`make:<script>.wasm?<parameters>` is drawn once by that script, with the parameters as its input.

A vehicle may have `pydestinationsign:`, `luadestinationsign:` and `wasmdestinationsign:` side by side. The Lua sign is
used while the Lua screens are on, else the WebAssembly one while those are on, else the Python one; a release without
the script screens reads only the Python one. An unknown key in any of the blocks is skipped together with its value.

## The image

`width * height` pixels of 4 bytes, R G B A, in sRGB, row by row from the top, as an image is drawn (PIL's
`tobytes()` of what a Python screen's `_render` returns). The game turns it bottom row first for the texture, as the
Python screens' `abstractscreenrenderer` does with `FLIP_TOP_BOTTOM`. Up to 4096 pixels on either side.

## Lua scripts

A script returns a table with `render(state)`, and optionally `init()`, called once after loading. Each screen has its
own Lua state, so what a script keeps in its variables stays between the refreshes of its screen.

```lua
local screen = {}

function screen.render(state)
	local image = eu07.image(256, 128, { 32, 32, 32 })
	image:fill_rect(0, 0, math.floor(state.velocity * 256 / 160), 128, { 0, 192, 0 })
	return image -- or nil: nothing changed, the previous image stays
end

return screen
```

`state` is a table of the input: numbers, booleans and strings (UTF-8), and `touches`, a list of `{ x, y }`. A
colour is a table `{ r, g, b }`, each 0-255. Coordinates are in pixels, `(0, 0)` the top left corner.

| function                                   | meaning                                                                       |
|--------------------------------------------|-------------------------------------------------------------------------------|
| `eu07.image(width, height [, color])`      | a new image, black or in the colour, up to 4096 on either side                 |
| `eu07.font(path)`                          | a font of the Python LED screens: `path .. ".png"` and `path .. ".txt"`, as `led_util.load_font()` reads them |
| `eu07.sprite(path)`                        | a sprite: `path .. ".png"`, as `led_util.load_spr()` reads it                  |
| `eu07.include(path)`                       | runs another script and returns what it returns (`local util = eu07.include("led_util.lua")`) |
| `eu07.read_file(path)`                     | the contents of a file, or `nil`                                               |
| `eu07.time()`                              | seconds of a steady clock shared by all screens, for animations (scrolling) that keep their pace whatever the refresh interval |
| `eu07.command(name, param1, param2)`       | sends a command (the names of `simulation::commandMap`) to the vehicle of the cab; ignored outside the cab |
| `eu07.log(...)`, `print(...)`              | writes a line to the log                                                       |

Paths are relative to the script's folder and lead no higher. A character a font lacks is drawn with the font's first
glyph (`led_util` picks a random one).

| method                                                     | meaning                                                         |
|------------------------------------------------------------|-----------------------------------------------------------------|
| `image:width()`, `image:height()`                          | the size                                                        |
| `image:fill(color)`                                        | fills the image                                                 |
| `image:set(x, y, color)`, `image:get(x, y)`                | a pixel; `get` returns `r, g, b`                                |
| `image:fill_rect(x, y, width, height, color)`              | a rectangle, clipped                                            |
| `image:draw_text(font, x, y, text, color [, spacing])`     | text from `x` rightwards, `spacing` columns between the characters (1); returns the `x` after it (`led_util.draw_str`) |
| `image:draw_text_right(...)`, `image:draw_text_centered(...)` | the same, ending at `x` or centred on it (`draw_strl`, `draw_strc`) |
| `image:draw_sprite(sprite, x, y, color)`                   | the lit pixels of the sprite (`draw_spr`)                       |
| `image:draw_image(image, x, y)`                            | another image, clipped                                          |
| `image:led_post([bloom])`                                  | `led_util.render_post()`: a new image, each pixel a 2x2 diode with a darker grid, glowing (`bloom`, on by default), in a 1 pixel black frame. see below for when it runs |
| `image:resize(width, height)`                              | a new image, scaled as Pillow's `resize(..., Image.LANCZOS)`, with the same pixels. see below for when it runs |
| `image:copy()`                                             | a new image, the same                                           |
| `font:height()`, `font:text_width(text [, spacing])`       | the height, the width of a text (`get_str_len`)                 |
| `sprite:width()`, `sprite:height()`                        | the size                                                        |

The drawing, `led_post` and `resize` run natively in the game, so a script only says what goes where.

`led_post` and `resize` only note what is to be done; the image's size is known right away. They run when the image is
used:

- on the image `render` returns, on the gpu, into the screen's texture. Only the screens that ask for them pay for
  them, and the texture gets the small image as drawn rather than the large one. The gpu gives the pixels of the cpu,
  give or take one step of brightness here and there, from rounding in single precision.
- on the cpu, as soon as the script reads the image (`get`), draws on it, or draws it on another image.
- on the cpu, in the screens' worker, when the renderer can't do it on the gpu (the NVRHI renderer, OpenGL ES, a
  shader that failed to load) or `screens.gpupost` is 0. A shader that fails is logged once, and the cpu takes over.

## WebAssembly modules

A module exports:

| export                        | meaning                                                                                  |
|-------------------------------|------------------------------------------------------------------------------------------|
| `memory`                      | the linear memory                                                                        |
| `screen_init() -> i32`        | optional; called once after loading. anything but 0 disables the screen                  |
| `screen_render() -> i32`      | called on every refresh. returns the address of the image, or 0 to keep the previous one |
| `screen_width() -> i32`       | the width of the image, asked after every `screen_render` that returned an image         |
| `screen_height() -> i32`      | the height of the image                                                                  |

Imports, module `eu07` (keys and texts are a pointer and a length in bytes, in the module's memory):

| import                                                          | meaning                                                         |
|-----------------------------------------------------------------|-----------------------------------------------------------------|
| `value_type(key, key_len) -> i32`                               | 0 no such value, 1 float, 2 int, 3 bool, 4 string, 5 list of points |
| `get_float(key, key_len) -> f64`                                | the value as a number (int and bool too); 0 when there is none  |
| `get_int(key, key_len) -> i32`                                  | the value as an integer (a float is truncated)                   |
| `get_bool(key, key_len) -> i32`                                 | 1 or 0                                                           |
| `get_string(key, key_len, buffer, capacity) -> i32`             | copies the text, in UTF-8; returns its whole length, -1 when there is none |
| `get_vec2_count(key, key_len) -> i32`                           | the number of points in a list (`touches`)                       |
| `get_vec2(key, key_len, index, out) -> i32`                     | writes the point as two f32 at `out`; returns 1, or 0 when there is none |
| `command(name, name_len, param1, param2)`                       | sends a command (the names of `simulation::commandMap`) to the vehicle of the cab; ignored outside the cab |
| `log(text, text_len)`                                           | writes a line to the log                                         |
| `time() -> f64`                                                 | seconds of a steady clock shared by all screens                  |
| `led_post(source, width, height, target, bloom) -> i32`         | `led_util.render_post()`, run natively: `target` takes `(2 width + 2) x (2 height + 2)`; `bloom` not 0 for the glow. returns 1, or 0 for sizes over 4096 |
| `resize_lanczos(source, width, height, target, target_width, target_height) -> i32` | Pillow's `resize(..., Image.LANCZOS)`, run natively, giving the same pixels. returns 1, or 0 for sizes over 4096 |
| `file_read(path, path_len, buffer, capacity) -> i32`            | reads a file from the module's folder or below; returns the file's whole size, -1 when there is none or the path leads outside |

`led_post` and `resize_lanczos` take RGBA images in the module's memory and write alpha 255; the source and the
target may overlap. They are what makes LED boards fast enough: in the interpreter the same filters take some 30
times longer.

The module gets 64 KiB of stack and no heap from the host: it manages its own memory.

### Building a module

With clang (any version with the `wasm32` target), no C library:

```c
// ekran.c
#include <stdint.h>

#define IMPORT(name) __attribute__((import_module("eu07"), import_name(name)))
IMPORT("get_float") double get_float(const char *key, int key_len);
IMPORT("command") void command(const char *name, int name_len, int param1, int param2);

#define WIDTH 256
#define HEIGHT 128
static uint32_t pixels[WIDTH * HEIGHT];

__attribute__((export_name("screen_width"))) int screen_width(void) { return WIDTH; }
__attribute__((export_name("screen_height"))) int screen_height(void) { return HEIGHT; }

__attribute__((export_name("screen_render"))) int screen_render(void) {
	double const velocity = get_float("velocity", 8);
	int const filled = (int)(velocity * WIDTH / 160.0);
	for (int y = 0; y < HEIGHT; ++y)
		for (int x = 0; x < WIDTH; ++x)
			pixels[y * WIDTH + x] = x < filled ? 0xFF00C000u : 0xFF202020u; // A B G R, little-endian
	return (int)(uintptr_t)pixels;
}
```

```
clang --target=wasm32 -O2 -nostdlib -Wl,--no-entry -o ekran.wasm ekran.c
```

Any language that compiles to WebAssembly works the same way (Rust `wasm32-unknown-unknown`, Zig, AssemblyScript),
as long as the module imports nothing but the `eu07` functions above.
