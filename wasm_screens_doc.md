# WebAssembly screens

Screens drawn by WebAssembly modules, alongside the Python screens (`pyscreen:`, `pydestinationsign:`). A module
gets the same input a Python screen gets (the table in `python_var_doc.md`), returns an RGBA image and, in the cab,
may send commands. The modules run in the WAMR interpreter (`ref/wamr`), in a sandbox: a module reaches nothing but
the functions of the `eu07` module below, and reads files only from its own folder.

Unlike a Python screen, which refreshes after a time (`python.updatetime`, `updatetime:`), a WebAssembly screen
refreshes every so many rendered frames, set in `eu07.ini`.

## eu07.ini

| key                   | default | meaning                                                                        |
|-----------------------|---------|--------------------------------------------------------------------------------|
| `wasm.enabled`        | 1       | 0: no WebAssembly screens; the vehicles show what they did before              |
| `wasm.cabframes`      | 4       | a screen in the cab is refreshed every so many frames (1-1000)                 |
| `wasm.externalframes` | 30      | a screen outside the cab (destination sign) is refreshed every so many frames  |

A screen may ask for a longer interval of its own (`frames:` below); the longer of the two counts. Screens sharing
an interval are spread over its frames, so they don't all refresh on the same one. A screen in the cab is refreshed
only while the player sits in that cab, a screen outside only while its vehicle is drawn.

## .mmd

In the cab section, next to `pyscreen:`:

```
wasmscreen: { ekran.wasm target: screen01 frames: 6 parameters: id=1&mode=full }
```

| key           | meaning                                                                                           |
|---------------|---------------------------------------------------------------------------------------------------|
| (first token) | the module, relative to the vehicle folder when it has no path of its own                         |
| `target:`     | the submodel the screen is drawn on (its texture is replaced); `none` for a screen that only sends commands |
| `frames:`     | the screen's own refresh interval, used when longer than `wasm.cabframes`                         |
| `parameters:` | extra `key=value` pairs, `&`-separated, added to the input                                        |

A touch on the screen's submodel comes in the input as the list of points `touches`, as for the Python screens.

In `internaldata:`, next to `pydestinationsign:`:

```
wasmdestinationsign: { tablica.wasm frames: 60 instance: name background: tablica_off parameters: style=lcd }
```

The keys are those of `pydestinationsign:` (`instance:`, `parameters:`, `background:`) and `frames:`. The input is
what a Python sign gets: the timetable (`$timetable`), `_id1` (the train), `_id2` (the instance) and the
parameters, read anew on every refresh, so the sign can follow the journey. The vehicles of one train showing the
same request share one screen and one texture. A static destination texture, when there is one, still comes first.

A vehicle may have both `pydestinationsign:` and `wasmdestinationsign:`: with the WebAssembly screens on, the
WebAssembly one is used; with them off (or in a release without them) the Python one. An unknown key in either block
is skipped together with its value.

## The module

Exports:

| export                        | meaning                                                                                  |
|-------------------------------|------------------------------------------------------------------------------------------|
| `memory`                      | the linear memory                                                                        |
| `screen_init() -> i32`        | optional; called once after loading. anything but 0 disables the screen                  |
| `screen_render() -> i32`      | called on every refresh. returns the address of the image, or 0 to keep the previous one |
| `screen_width() -> i32`       | the width of the image, asked after every `screen_render` that returned an image         |
| `screen_height() -> i32`      | the height of the image, up to 4096 on either side                                       |

The image is `width * height` pixels of 4 bytes, R G B A, in sRGB, its rows in the order the Python screens give
them (PIL's `tobytes()`).

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
| `time() -> f64`                                                 | seconds of a steady clock shared by all screens, for animations (scrolling) that keep their pace whatever the refresh interval |
| `led_post(source, width, height, target, bloom) -> i32`         | `led_util.render_post()` of the Python screens, run natively: each diode of the `width x height` image becomes 2x2 pixels with a darker grid, glowing into its neighbours when `bloom` is not 0, in a 1 pixel black frame; `target` takes `(2 width + 2) x (2 height + 2)`. returns 1, or 0 for sizes over 4096 |
| `resize_lanczos(source, width, height, target, target_width, target_height) -> i32` | Pillow's `resize(..., Image.LANCZOS)`, run natively, giving the same pixels. returns 1, or 0 for sizes over 4096 |
| `file_read(path, path_len, buffer, capacity) -> i32`            | reads a file from the module's folder or below; returns the file's whole size, -1 when there is none or the path leads outside |

`led_post` and `resize_lanczos` take RGBA images in the module's memory and write alpha 255; the source and the
target may overlap. They are what makes LED boards fast enough: in the interpreter the same filters take some 30
times longer.

A call into the module may run at most 200 million instructions; a module that traps or runs over is disabled and
the reason goes to the log. The module gets 64 KiB of stack and no heap from the host: it manages its own memory.

## Building a module

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
