#include "stdafx.h"
#include "doctest.h"

// the script screen host (scripting/scriptscreens.cpp) with the real runtimes: a script reads its input, draws an
// image that goes to the screen's texture, bottom row first, and sends a command; a script stuck in a loop is stopped;
// a Lua script can't reach files, the system or precompiled code. the Lua cases need WITH_LUA, the WebAssembly ones
// WITH_WASM

#include "scripting/scriptscreens.h"
#include "input/command.h"
#include "interfaces/ITexture.h"
#include "utilities/Globals.h"
#include "utilities/dictionary.h"

#include <array>
#include <chrono>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <span>
#include <string_view>
#include <thread>
#include <vector>

// the engine's command queue and command names, which the test build doesn't link: the commands the screens send
// are kept here instead

namespace {

std::vector<std::pair<command_data, std::uint32_t>> &sent_commands() {
	static std::vector<std::pair<command_data, std::uint32_t>> commands;
	return commands;
}

} // namespace

namespace simulation {
command_queue Commands; // NOSONAR the engine's extern global, input/command.h
std::unordered_map<std::string, user_command> const commandMap{{"cmdtest", user_command::aidriverenable}}; // NOSONAR the engine's extern global, input/command.h
} // namespace simulation

void command_queue::push(command_data const &Command, uint32_t const Recipient) { // NOSONAR signature from input/command.h
	sent_commands().emplace_back(Command, Recipient);
}

namespace {

// a texture that keeps what was uploaded to it
struct recording_texture : ITexture {
	bool create(bool) override { return true; }
	int get_width() const override { return width; }
	int get_height() const override { return height; }
	size_t get_id() const override { return id; }
	void release() override { /* nothing held */ }
	void make_stub() override { /* nothing held */ }
	std::string_view get_traits() const override { return ""; }
	std::string_view get_name() const override { return "recording"; }
	std::string_view get_type() const override { return "make:"; }
	bool is_stub() const override { return false; }
	bool get_has_alpha() const override { return true; }
	bool get_is_ready() const override { return ready; }
	void set_components_hint(int) override { /* not used */ }
	void make_from_memory(size_t const Width, size_t const Height, const uint8_t *Data) override { update_from_memory(Width, Height, Data); }
	void update_from_memory(size_t const Width, size_t const Height, const uint8_t *Data) override {
		width = static_cast<int>(Width);
		height = static_cast<int>(Height);
		pixels.assign(Data, Data + Width * Height * 4);
		++uploads;
	}

	bool ready{true};
	size_t id{1};
	int width{0};
	int height{0};
	int uploads{0};
	std::vector<std::uint8_t> pixels;
};

// a folder of its own for the scripts of a test, removed with the object. it's made in the build folder of the tests
// (EU07_UNIT_TESTS_SCRATCH, see CMakeLists.txt), not in the system's temporary folder, which anyone can write to. the
// scripts are one level down in it, so a file a test puts next to their folder ("../name") is still inside the folder
// of the test, and is removed with it
class script_folder {
  public:
	explicit script_folder(std::string const &Name)
	    : m_root{std::filesystem::path{EU07_UNIT_TESTS_SCRATCH} / std::format("eu07_script_screen_test_{}", Name)}, m_folder{m_root / "scripts"} {
		std::filesystem::remove_all(m_root);
		std::filesystem::create_directories(m_folder);
	}
	~script_folder() {
		std::error_code error;
		std::filesystem::remove_all(m_root, error);
	}
	script_folder(script_folder const &) = delete;
	script_folder &operator=(script_folder const &) = delete;

	// writes the file, returns its path as the game names scripts
	std::string write(std::string const &Name, std::span<std::uint8_t const> const Bytes) const {
		auto const path{m_folder / Name};
		std::filesystem::create_directories(path.parent_path());
		std::ofstream file{path, std::ios::binary};
		file.write(reinterpret_cast<char const *>(Bytes.data()), static_cast<std::streamsize>(Bytes.size()));
		return path.generic_string();
	}
	std::string write(std::string const &Name, std::string_view const Text) const {
		return write(Name, std::span{reinterpret_cast<std::uint8_t const *>(Text.data()), Text.size()});
	}

  private:
	std::filesystem::path m_root; // the folder of the test
	std::filesystem::path m_folder; // the folder of its scripts
};

// runs frames until the condition holds, or gives up after a few seconds
bool run_until(script_screen_queue &Queue, std::function<bool()> const &Condition) {
	auto const deadline{std::chrono::steady_clock::now() + std::chrono::seconds(20)};
	while (std::chrono::steady_clock::now() < deadline) {
		Queue.update();
		if (Condition()) {
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return false;
}

std::shared_ptr<dictionary_source> speed(double const Value) {
	auto input{std::make_shared<dictionary_source>()};
	input->insert("speed", Value);
	return input;
}

script_screen_queue::screen_request request(script_language const Language, std::string const &Script, std::string const &Key, ITexture *Texture,
                                            script_screen_kind const Kind = script_screen_kind::cab) {
	return {.kind = Kind, .language = Language, .script = Script, .key = Key, .texture = Texture, .frames = 0};
}

// submits a refresh as soon as the screen is due
void refresh(script_screen_queue &Queue, std::shared_ptr<script_screen> const &Screen, std::shared_ptr<dictionary_source> Input, std::uint32_t const Vehicle) {
	REQUIRE(run_until(Queue, [&Queue, &Screen] { return Queue.due(*Screen); }));
	Queue.submit(Screen, std::move(Input), Vehicle);
}

} // namespace

TEST_CASE("script screens: a language that doesn't run gets no screens") {
	script_screen_queue queue;
	// not started
	CHECK_FALSE(queue.runs(script_language::lua));
	CHECK(queue.acquire(request(script_language::lua, "x.lua", "x", nullptr)) == nullptr);
}

TEST_CASE("script screens: the language of a script is told by its extension") {
	CHECK(script_language_of("scripts/led_111a20.lua") == script_language::lua);
	CHECK(script_language_of("dynamic/pkp/x/ekran.wasm") == script_language::wasm);
	CHECK_FALSE(script_language_of("scripts/led_111a20").has_value());
}

#ifdef WITH_WASM

namespace {

// the modules, encoded by hand. "paint": a 2x2 screen in the colour 0xFF0000vv, vv being the input value
// "speed", which sends the command "cmdtest" with the parameters (vv, 7):
//   (import "eu07" "get_float" (func $get_float (param i32 i32) (result f64)))
//   (import "eu07" "command" (func $command (param i32 i32 i32 i32)))
//   (memory (export "memory") 1)
//   (data (i32.const 0) "speed") (data (i32.const 16) "cmdtest")
//   (func (export "screen_width") (result i32) i32.const 2)
//   (func (export "screen_height") (result i32) i32.const 2)
//   (func (export "screen_render") (result i32) (local $v i32)
//     (local.set $v (i32.trunc_f64_s (call $get_float (i32.const 0) (i32.const 5))))
//     (i32.store (i32.const 1024) (i32.or (local.get $v) (i32.const 0xFF000000))) ... the same at 1028, 1032, 1036
//     (call $command (i32.const 16) (i32.const 7) (local.get $v) (i32.const 7))
//     (i32.const 1024))
constexpr std::array<std::uint8_t, 254> paint_module{
	0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00, 0x01, 0x12, 0x03, 0x60, 0x02, 0x7f, 0x7f, 0x01,
	0x7c, 0x60, 0x04, 0x7f, 0x7f, 0x7f, 0x7f, 0x00, 0x60, 0x00, 0x01, 0x7f, 0x02, 0x21, 0x02, 0x04,
	0x65, 0x75, 0x30, 0x37, 0x09, 0x67, 0x65, 0x74, 0x5f, 0x66, 0x6c, 0x6f, 0x61, 0x74, 0x00, 0x00,
	0x04, 0x65, 0x75, 0x30, 0x37, 0x07, 0x63, 0x6f, 0x6d, 0x6d, 0x61, 0x6e, 0x64, 0x00, 0x01, 0x03,
	0x04, 0x03, 0x02, 0x02, 0x02, 0x05, 0x03, 0x01, 0x00, 0x01, 0x07, 0x39, 0x04, 0x06, 0x6d, 0x65,
	0x6d, 0x6f, 0x72, 0x79, 0x02, 0x00, 0x0c, 0x73, 0x63, 0x72, 0x65, 0x65, 0x6e, 0x5f, 0x77, 0x69,
	0x64, 0x74, 0x68, 0x00, 0x02, 0x0d, 0x73, 0x63, 0x72, 0x65, 0x65, 0x6e, 0x5f, 0x68, 0x65, 0x69,
	0x67, 0x68, 0x74, 0x00, 0x03, 0x0d, 0x73, 0x63, 0x72, 0x65, 0x65, 0x6e, 0x5f, 0x72, 0x65, 0x6e,
	0x64, 0x65, 0x72, 0x00, 0x04, 0x0a, 0x5e, 0x03, 0x04, 0x00, 0x41, 0x02, 0x0b, 0x04, 0x00, 0x41,
	0x02, 0x0b, 0x52, 0x01, 0x01, 0x7f, 0x41, 0x00, 0x41, 0x05, 0x10, 0x00, 0xaa, 0x21, 0x00, 0x41,
	0x80, 0x08, 0x20, 0x00, 0x41, 0x80, 0x80, 0x80, 0x78, 0x72, 0x36, 0x02, 0x00, 0x41, 0x84, 0x08,
	0x20, 0x00, 0x41, 0x80, 0x80, 0x80, 0x78, 0x72, 0x36, 0x02, 0x00, 0x41, 0x88, 0x08, 0x20, 0x00,
	0x41, 0x80, 0x80, 0x80, 0x78, 0x72, 0x36, 0x02, 0x00, 0x41, 0x8c, 0x08, 0x20, 0x00, 0x41, 0x80,
	0x80, 0x80, 0x78, 0x72, 0x36, 0x02, 0x00, 0x41, 0x10, 0x41, 0x07, 0x20, 0x00, 0x41, 0x07, 0x10,
	0x01, 0x41, 0x80, 0x08, 0x0b, 0x0b, 0x17, 0x02, 0x00, 0x41, 0x00, 0x0b, 0x05, 0x73, 0x70, 0x65,
	0x65, 0x64, 0x00, 0x41, 0x10, 0x0b, 0x07, 0x63, 0x6d, 0x64, 0x74, 0x65, 0x73, 0x74};

// "spin": exports the same functions as "paint", but screen_render never returns: (loop $l (br $l))
constexpr std::array<std::uint8_t, 108> spin_module{
	0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00, 0x01, 0x05, 0x01, 0x60, 0x00, 0x01, 0x7f, 0x03,
	0x04, 0x03, 0x00, 0x00, 0x00, 0x05, 0x03, 0x01, 0x00, 0x01, 0x07, 0x39, 0x04, 0x06, 0x6d, 0x65,
	0x6d, 0x6f, 0x72, 0x79, 0x02, 0x00, 0x0c, 0x73, 0x63, 0x72, 0x65, 0x65, 0x6e, 0x5f, 0x77, 0x69,
	0x64, 0x74, 0x68, 0x00, 0x00, 0x0d, 0x73, 0x63, 0x72, 0x65, 0x65, 0x6e, 0x5f, 0x68, 0x65, 0x69,
	0x67, 0x68, 0x74, 0x00, 0x01, 0x0d, 0x73, 0x63, 0x72, 0x65, 0x65, 0x6e, 0x5f, 0x72, 0x65, 0x6e,
	0x64, 0x65, 0x72, 0x00, 0x02, 0x0a, 0x15, 0x03, 0x04, 0x00, 0x41, 0x02, 0x0b, 0x04, 0x00, 0x41,
	0x02, 0x0b, 0x09, 0x00, 0x03, 0x40, 0x0c, 0x00, 0x0b, 0x41, 0x00, 0x0b};

} // namespace

TEST_CASE("wasm screens: a cab screen draws from its input and sends commands to its vehicle") {
	sent_commands().clear();
	script_folder const folder{"wasm_paint"};
	auto const module_path{folder.write("paint.wasm", paint_module)};
	script_screen_queue queue;
	REQUIRE(queue.init());
	REQUIRE(queue.runs(script_language::wasm));

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::wasm, module_path, "cab", &texture))};
	REQUIRE(screen != nullptr);
	// the same key, the same screen
	CHECK(queue.acquire(request(script_language::wasm, module_path, "cab", &texture)) == screen);

	refresh(queue, screen, speed(5.0), 42);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0 && false == sent_commands().empty(); }));

	CHECK(texture.width == 2);
	CHECK(texture.height == 2);
	REQUIRE(texture.pixels.size() == 16);
	std::array<std::uint8_t, 4> const expected{0x05, 0x00, 0x00, 0xFF};
	for (std::size_t pixel = 0; pixel < 4; ++pixel) {
		CAPTURE(pixel);
		CHECK(std::equal(expected.begin(), expected.end(), texture.pixels.begin() + static_cast<std::ptrdiff_t>(pixel * 4)));
	}

	REQUIRE(sent_commands().size() == 1);
	auto const &[command, recipient]{sent_commands().front()};
	CHECK(command.command == user_command::aidriverenable);
	CHECK(command.param1 == 5.0);
	CHECK(command.param2 == 7.0);
	CHECK(recipient == (static_cast<std::uint32_t>(command_target::vehicle) | 42u));

	queue.exit();
}

TEST_CASE("wasm screens: the image goes up again when the texture is made anew, and waits while it isn't there") {
	sent_commands().clear();
	script_folder const folder{"wasm_reupload"};
	auto const module_path{folder.write("paint.wasm", paint_module)};
	script_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	texture.ready = false;
	auto const screen{queue.acquire(request(script_language::wasm, module_path, "cab", &texture))};
	refresh(queue, screen, speed(9.0), 1);
	REQUIRE(run_until(queue, [] { return false == sent_commands().empty(); }));
	queue.update();
	CHECK(texture.uploads == 0);

	texture.ready = true;
	queue.update();
	CHECK(texture.uploads == 1);
	queue.update();
	CHECK(texture.uploads == 1);

	// released and made again: a new gl texture
	texture.id = 2;
	queue.update();
	CHECK(texture.uploads == 2);

	queue.exit();
}

TEST_CASE("wasm screens: a screen outside the cab draws, but its commands are dropped") {
	sent_commands().clear();
	script_folder const folder{"wasm_sign"};
	auto const module_path{folder.write("paint.wasm", paint_module)};
	script_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::wasm, module_path, "sign", &texture, script_screen_kind::external))};
	refresh(queue, screen, speed(3.0), 0);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	queue.update();
	CHECK(sent_commands().empty());

	queue.exit();
}

TEST_CASE("wasm screens: a screen stuck in a loop is stopped, and the others keep working") {
	script_folder const folder{"wasm_spin"};
	auto const spinning{folder.write("spin.wasm", spin_module)};
	auto const painting{folder.write("paint.wasm", paint_module)};
	script_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture stuck;
	recording_texture working;
	auto const first{queue.acquire(request(script_language::wasm, spinning, "spin", &stuck))};
	auto const second{queue.acquire(request(script_language::wasm, painting, "paint", &working))};
	refresh(queue, first, speed(1.0), 1);
	refresh(queue, second, speed(2.0), 1);

	REQUIRE(run_until(queue, [&working] { return working.uploads > 0; }));
	CHECK(stuck.uploads == 0);
	// the stuck screen ran first, into the instruction limit, and isn't due any more
	CHECK_FALSE(queue.due(*first));

	queue.exit();
}

TEST_CASE("wasm screens: the screens outlive the queue without harm") {
	script_folder const folder{"wasm_outlive"};
	auto const module_path{folder.write("paint.wasm", paint_module)};
	std::shared_ptr<script_screen> screen;
	recording_texture texture;
	{
		script_screen_queue queue;
		REQUIRE(queue.init());
		screen = queue.acquire(request(script_language::wasm, module_path, "cab", &texture));
		refresh(queue, screen, speed(4.0), 1);
		REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	}
	// the queue and the runtime are gone, the vehicle still holds its screen
	CHECK(screen != nullptr);
	screen.reset();
}

#endif

#ifdef WITH_LUA

#include "stb/stb_image.h"
#include "stb/stb_image_write.h"

namespace {

// the game's texture loader turns every image stb_image reads bottom row first (model/Texture.cpp); for the length of
// a test, the same here
class images_flipped_as_in_the_game {
  public:
	images_flipped_as_in_the_game() { stbi_set_flip_vertically_on_load(1); }
	~images_flipped_as_in_the_game() { stbi_set_flip_vertically_on_load(0); }
	images_flipped_as_in_the_game(images_flipped_as_in_the_game const &) = delete;
	images_flipped_as_in_the_game &operator=(images_flipped_as_in_the_game const &) = delete;
};

// a 2x2 screen in the colour (speed, 0, 0), sending the command "cmdtest" with (speed, 7)
constexpr std::string_view paint_script{R"(
local screen = {}
function screen.render(state)
	eu07.command("cmdtest", state.speed, 7)
	return eu07.image(2, 2, { state.speed, 0, 0 })
end
return screen
)"};

// 1x2: a red row above a green one
constexpr std::string_view rows_script{R"(
return {
	render = function(state)
		local image = eu07.image(1, 2, { 0, 0, 0 })
		image:set(0, 0, { 255, 0, 0 })
		image:set(0, 1, { 0, 255, 0 })
		return image
	end
}
)"};

// 1x1, white when nothing of the world outside the sandbox is reachable, black otherwise
constexpr std::string_view sandbox_script{R"(
local reachable = io or os or ffi or debug or package or require or load or loadstring or dofile or loadfile or string.dump or jit
local escapes = eu07.read_file("../outside.txt") or eu07.read_file("/etc/hosts") or eu07.read_file("c:/windows/win.ini")
local inside = eu07.read_file("data/inside.txt")
local ok = not reachable and not escapes and inside == "inside"
return {
	render = function(state)
		return eu07.image(1, 1, ok and { 255, 255, 255 } or { 0, 0, 0 })
	end
}
)"};

// never returns from render
constexpr std::string_view spin_script{R"(
return { render = function(state) while true do end end }
)"};

// draws once, then reports nothing new
constexpr std::string_view once_script{R"(
local drawn = false
return {
	render = function(state)
		if drawn then return nil end
		drawn = true
		return eu07.image(1, 1, { 1, 2, 3 })
	end
}
)"};

// uses a helper script and a font next to it
constexpr std::string_view include_script{R"(
local helper = eu07.include("helper.lua")
local font = eu07.font("font/test")
return {
	render = function(state)
		local image = eu07.image(8, 2, { 0, 0, 0 })
		local x = image:draw_text(font, 0, 0, helper.text, { 255, 0, 0 })
		image:set(7, 1, { x, font:text_width("ab"), font:height() })
		return image
	end
}
)"};

constexpr std::string_view helper_script{R"(
return { text = "ab" }
)"};

// a font of two glyphs a column apart: "a" two columns wide, lit in its top row, "b" one column, lit in both rows.
// written as a .png by stb_image_write, the way the game's fonts are
std::vector<std::uint8_t> test_font_png() {
	constexpr int width{4};
	constexpr int height{2};
	std::array<std::uint8_t, width * height * 4> pixels{};
	pixels.fill(255);
	auto const lit{[&pixels](int const X, int const Y) { pixels[(Y * width + X) * 4] = 0; }};
	lit(0, 0);
	lit(1, 0);
	lit(3, 0);
	lit(3, 1);
	std::vector<std::uint8_t> result;
	auto const append{[](void *Target, void *Data, int const Size) {
		auto *bytes{static_cast<std::uint8_t const *>(Data)};
		static_cast<std::vector<std::uint8_t> *>(Target)->insert(static_cast<std::vector<std::uint8_t> *>(Target)->end(), bytes, bytes + Size);
	}};
	stbi_write_png_to_func(append, &result, width, height, 4, pixels.data(), width * 4);
	return result;
}

} // namespace

TEST_CASE("lua screens: a cab screen draws from its input and sends commands to its vehicle") {
	sent_commands().clear();
	script_folder const folder{"lua_paint"};
	auto const script{folder.write("paint.lua", paint_script)};
	script_screen_queue queue;
	REQUIRE(queue.init());
	REQUIRE(queue.runs(script_language::lua));

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::lua, script, "cab", &texture))};
	REQUIRE(screen != nullptr);
	refresh(queue, screen, speed(5.0), 42);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0 && false == sent_commands().empty(); }));

	CHECK(texture.width == 2);
	CHECK(texture.height == 2);
	REQUIRE(texture.pixels.size() == 16);
	CHECK(texture.pixels[0] == 5);
	CHECK(texture.pixels[1] == 0);
	CHECK(texture.pixels[3] == 255);

	REQUIRE(sent_commands().size() == 1);
	auto const &[command, recipient]{sent_commands().front()};
	CHECK(command.command == user_command::aidriverenable);
	CHECK(command.param1 == 5.0);
	CHECK(command.param2 == 7.0);
	CHECK(recipient == (static_cast<std::uint32_t>(command_target::vehicle) | 42u));
	queue.exit();
}

TEST_CASE("lua screens: the texture gets the bottom row first, as from the python screens") {
	script_folder const folder{"lua_rows"};
	auto const script{folder.write("rows.lua", rows_script)};
	script_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::lua, script, "rows", &texture))};
	refresh(queue, screen, speed(0.0), 1);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	REQUIRE(texture.pixels.size() == 8);
	// green, the bottom row, first
	CHECK(texture.pixels[0] == 0);
	CHECK(texture.pixels[1] == 255);
	CHECK(texture.pixels[4] == 255);
	CHECK(texture.pixels[5] == 0);
	queue.exit();
}

TEST_CASE("lua screens: a script reaches no files outside its folder, nor the system, nor ffi") {
	script_folder const folder{"lua_sandbox"};
	// next to the folder of the scripts, where the script mustn't reach
	folder.write("../outside.txt", std::string_view{"outside"});
	folder.write("data/inside.txt", std::string_view{"inside"});
	auto const script{folder.write("sandbox.lua", sandbox_script)};
	script_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::lua, script, "sandbox", &texture))};
	refresh(queue, screen, speed(0.0), 1);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	REQUIRE(texture.pixels.size() == 4);
	CHECK(texture.pixels[0] == 255);
	queue.exit();
}

TEST_CASE("lua screens: precompiled code isn't loaded") {
	script_folder const folder{"lua_bytecode"};
	// LuaJIT bytecode starts with ESC "LJ"
	auto const script{folder.write("compiled.lua", std::string_view{"\x1bLJ\x02\x00 not a script"})};
	script_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::lua, script, "compiled", &texture))};
	refresh(queue, screen, speed(0.0), 1);
	// it fails to load, and isn't refreshed again: a screen that runs would be due again within a few frames
	for (int frame = 0; frame < 40; ++frame) {
		queue.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	CHECK_FALSE(queue.due(*screen));
	CHECK(texture.uploads == 0);
	queue.exit();
}

TEST_CASE("lua screens: a script stuck in a loop is stopped, and the others keep working") {
	script_folder const folder{"lua_spin"};
	auto const spinning{folder.write("spin.lua", spin_script)};
	auto const painting{folder.write("paint.lua", paint_script)};
	script_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture stuck;
	recording_texture working;
	auto const first{queue.acquire(request(script_language::lua, spinning, "spin", &stuck))};
	auto const second{queue.acquire(request(script_language::lua, painting, "paint", &working))};
	refresh(queue, first, speed(1.0), 1);
	refresh(queue, second, speed(2.0), 1);

	REQUIRE(run_until(queue, [&working] { return working.uploads > 0; }));
	CHECK(stuck.uploads == 0);
	CHECK_FALSE(queue.due(*first));
	queue.exit();
}

TEST_CASE("lua screens: nil from render keeps the image there is") {
	script_folder const folder{"lua_once"};
	auto const script{folder.write("once.lua", once_script)};
	script_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::lua, script, "once", &texture))};
	refresh(queue, screen, speed(0.0), 1);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	refresh(queue, screen, speed(0.0), 1);
	refresh(queue, screen, speed(0.0), 1);
	for (int frame = 0; frame < 20; ++frame) {
		queue.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	CHECK(texture.uploads == 1);
	CHECK(texture.pixels[2] == 3);
	queue.exit();
}

TEST_CASE("lua screens: a script includes another and reads a font of the python screens' format") {
	script_folder const folder{"lua_include"};
	auto const script{folder.write("include.lua", include_script)};
	folder.write("helper.lua", helper_script);
	folder.write("font/test.png", test_font_png());
	folder.write("font/test.txt", std::string_view{"// a comment line\r\na2\r\nb1\r\n"});
	// the fonts are read top row first all the same
	images_flipped_as_in_the_game const flipped;
	script_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::lua, script, "include", &texture))};
	refresh(queue, screen, speed(0.0), 1);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	REQUIRE(texture.pixels.size() == 8 * 2 * 4);
	// bottom row first: the top row of the image is the second row of the texture
	auto const top{[&texture](int const X) { return texture.pixels[(8 + X) * 4]; }};
	auto const bottom{[&texture](int const X) { return texture.pixels[X * 4]; }};
	CHECK(top(0) == 255);
	CHECK(top(1) == 255);
	CHECK(top(2) == 0); // the spacing column
	CHECK(top(3) == 255);
	CHECK(bottom(0) == 0);
	CHECK(bottom(3) == 255);
	// the x after the text (2 + 1 + 1 + 1), its width and the font's height
	CHECK(texture.pixels[7 * 4] == 5);
	CHECK(texture.pixels[7 * 4 + 1] == 5);
	CHECK(texture.pixels[7 * 4 + 2] == 2);
	queue.exit();
}

namespace {

// 2x1, red and green, shown through led_post without the glow and a resize to 4x3
constexpr std::string_view posted_script{R"(
return {
	render = function(state)
		local image = eu07.image(2, 1, { 255, 0, 0 })
		image:set(1, 0, { 0, 255, 0 })
		return image:led_post(false):resize(4, 3)
	end
}
)"};

// reads an image after led_post: its size, a pixel of the frame and one of a diode
constexpr std::string_view read_posted_script{R"(
return {
	render = function(state)
		local posted = eu07.image(2, 1, { 255, 255, 255 }):led_post(false)
		local width, height = posted:width(), posted:height()
		local frame = posted:get(0, 0)
		local diode = posted:get(1, 1)
		return eu07.image(1, 1, { width * 10 + height, frame, diode })
	end
}
)"};

// a gpu that keeps what it was given, or one that can't do it
struct recording_processor : script_screens::post_processor {
	bool apply(ITexture & /*Target*/, std::span<std::uint8_t const> const Pixels, int const Width, int const Height,
	           std::span<script_screens::post_step const> const Steps) override {
		++calls;
		pixels.assign(Pixels.begin(), Pixels.end());
		width = Width;
		height = Height;
		steps.assign(Steps.begin(), Steps.end());
		return works;
	}

	bool works{true};
	int calls{0};
	std::vector<std::uint8_t> pixels;
	int width{0};
	int height{0};
	std::vector<script_screens::post_step> steps;
};

// what posted_script shows, done on the cpu, bottom row first
std::vector<std::uint8_t> posted_on_cpu() {
	script_screens::post_step led;
	led.type = script_screens::post_step::kind::led;
	led.bloom = false;
	script_screens::post_step resize;
	resize.type = script_screens::post_step::kind::resize;
	resize.width = 4;
	resize.height = 3;
	std::array const steps{led, resize};
	script_screens::post_image image{{255, 0, 0, 255, 0, 255, 0, 255}, 2, 1};
	REQUIRE(script_screens::apply_post(image, steps));
	std::vector<std::uint8_t> result;
	for (auto row = image.height - 1; row >= 0; --row) {
		auto const first{image.pixels.begin() + static_cast<std::ptrdiff_t>(script_screens::image_size(image.width, row))};
		result.insert(result.end(), first, first + image.width * 4);
	}
	return result;
}

} // namespace

TEST_CASE("lua screens: led_post and resize of the image shown go to the gpu, with the image as drawn") {
	script_folder const folder{"lua_post_gpu"};
	auto const script{folder.write("posted.lua", posted_script)};
	script_screen_queue queue;
	REQUIRE(queue.init());
	recording_processor gpu;
	queue.set_post_processor(&gpu);

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::lua, script, "posted", &texture, script_screen_kind::external))};
	refresh(queue, screen, speed(0.0), 1);
	REQUIRE(run_until(queue, [&gpu] { return gpu.calls > 0; }));
	CHECK(texture.uploads == 0);
	CHECK(gpu.width == 2);
	CHECK(gpu.height == 1);
	CHECK(gpu.pixels == std::vector<std::uint8_t>{255, 0, 0, 255, 0, 255, 0, 255});
	REQUIRE(gpu.steps.size() == 2);
	CHECK(gpu.steps[0].type == script_screens::post_step::kind::led);
	CHECK_FALSE(gpu.steps[0].bloom);
	CHECK(gpu.steps[1].type == script_screens::post_step::kind::resize);
	CHECK(gpu.steps[1].width == 4);
	CHECK(gpu.steps[1].height == 3);
	queue.exit();
}

TEST_CASE("lua screens: without a gpu, led_post and resize run on the cpu") {
	script_folder const folder{"lua_post_cpu"};
	auto const script{folder.write("posted.lua", posted_script)};
	script_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::lua, script, "posted", &texture, script_screen_kind::external))};
	refresh(queue, screen, speed(0.0), 1);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	CHECK(texture.width == 4);
	CHECK(texture.height == 3);
	CHECK(texture.pixels == posted_on_cpu());
	queue.exit();
}

TEST_CASE("lua screens: when the gpu can't do the post-processing, the cpu does it from then on") {
	script_folder const folder{"lua_post_fallback"};
	auto const script{folder.write("posted.lua", posted_script)};
	script_screen_queue queue;
	REQUIRE(queue.init());
	recording_processor gpu;
	gpu.works = false;
	queue.set_post_processor(&gpu);

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::lua, script, "posted", &texture, script_screen_kind::external))};
	refresh(queue, screen, speed(0.0), 1);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	CHECK(gpu.calls == 1);
	CHECK(texture.width == 4);
	CHECK(texture.pixels == posted_on_cpu());

	refresh(queue, screen, speed(0.0), 1);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 1; }));
	CHECK(gpu.calls == 1);
	CHECK(texture.pixels == posted_on_cpu());
	queue.exit();
}

TEST_CASE("lua screens: an image read after led_post has its post-processing done first") {
	script_folder const folder{"lua_post_read"};
	auto const script{folder.write("read.lua", read_posted_script)};
	script_screen_queue queue;
	REQUIRE(queue.init());
	recording_processor gpu;
	queue.set_post_processor(&gpu);

	recording_texture texture;
	auto const screen{queue.acquire(request(script_language::lua, script, "read", &texture, script_screen_kind::external))};
	refresh(queue, screen, speed(0.0), 1);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	CHECK(gpu.calls == 0);
	REQUIRE(texture.pixels.size() == 4);
	CHECK(texture.pixels[0] == 64); // 6 x 4
	CHECK(texture.pixels[1] == 0);
	CHECK(texture.pixels[2] == 255);
	queue.exit();
}

#endif
