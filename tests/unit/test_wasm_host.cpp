#include "stdafx.h"
#include "doctest.h"

// the wasm screen host (scripting/wasmscreens.cpp) with the real runtime: a module reads its input, draws an
// image that goes to the screen's texture and sends a command; a module stuck in a loop is stopped. built with
// WITH_WASM only

#ifdef WITH_WASM

#include "scripting/wasmscreens.h"
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

// writes a module into a folder of its own, removed with the object
class module_file {
  public:
	template <std::size_t Size_>
	module_file(std::string const &Name, std::array<std::uint8_t, Size_> const &Bytes)
	    : m_folder{std::filesystem::temp_directory_path() / std::format("eu07_wasm_test_{}", Name)} {
		std::filesystem::create_directories(m_folder);
		m_path = (m_folder / (Name + ".wasm")).generic_string();
		std::ofstream file{m_path, std::ios::binary};
		file.write(reinterpret_cast<char const *>(Bytes.data()), static_cast<std::streamsize>(Bytes.size()));
	}
	~module_file() {
		std::error_code error;
		std::filesystem::remove_all(m_folder, error);
	}
	module_file(module_file const &) = delete;
	module_file &operator=(module_file const &) = delete;

	std::string const &path() const { return m_path; }

  private:
	std::filesystem::path m_folder;
	std::string m_path;
};

// runs frames until the condition holds, or gives up after a few seconds
bool run_until(wasm_screen_queue &Queue, std::function<bool()> const &Condition) {
	auto const deadline{std::chrono::steady_clock::now() + std::chrono::seconds(10)};
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

} // namespace

TEST_CASE("wasm host: a cab screen draws from its input and sends commands to its vehicle") {
	sent_commands().clear();
	module_file const module{"paint", paint_module};
	wasm_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	auto const screen{queue.acquire({.kind = wasm_screen_kind::cab, .script = module.path(), .key = "cab", .texture = &texture, .frames = 0})};
	REQUIRE(screen != nullptr);
	// the same key, the same screen
	CHECK(queue.acquire({.kind = wasm_screen_kind::cab, .script = module.path(), .key = "cab", .texture = &texture, .frames = 0}) == screen);

	REQUIRE(run_until(queue, [&queue, &screen] { return queue.due(*screen); }));
	queue.submit(screen, speed(5.0), 42);
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

TEST_CASE("wasm host: the image goes up again when the texture is made anew, and waits while it isn't there") {
	sent_commands().clear();
	module_file const module{"paint", paint_module};
	wasm_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	texture.ready = false;
	auto const screen{queue.acquire({.kind = wasm_screen_kind::cab, .script = module.path(), .key = "cab", .texture = &texture, .frames = 0})};
	REQUIRE(run_until(queue, [&queue, &screen] { return queue.due(*screen); }));
	queue.submit(screen, speed(9.0), 1);
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

TEST_CASE("wasm host: a screen outside the cab draws, but its commands are dropped") {
	sent_commands().clear();
	module_file const module{"paint", paint_module};
	wasm_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture texture;
	auto const screen{queue.acquire({.kind = wasm_screen_kind::external, .script = module.path(), .key = "sign", .texture = &texture, .frames = 0})};
	REQUIRE(run_until(queue, [&queue, &screen] { return queue.due(*screen); }));
	queue.submit(screen, speed(3.0), 0);
	REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	queue.update();
	CHECK(sent_commands().empty());

	queue.exit();
}

TEST_CASE("wasm host: a screen stuck in a loop is stopped, and the others keep working") {
	sent_commands().clear();
	module_file const spinning{"spin", spin_module};
	module_file const painting{"paint", paint_module};
	wasm_screen_queue queue;
	REQUIRE(queue.init());

	recording_texture stuck;
	recording_texture working;
	auto const first{queue.acquire({.kind = wasm_screen_kind::cab, .script = spinning.path(), .key = "spin", .texture = &stuck, .frames = 0})};
	auto const second{queue.acquire({.kind = wasm_screen_kind::cab, .script = painting.path(), .key = "paint", .texture = &working, .frames = 0})};
	REQUIRE(run_until(queue, [&queue, &first] { return queue.due(*first); }));
	REQUIRE(run_until(queue, [&queue, &second] { return queue.due(*second); }));
	queue.submit(first, speed(1.0), 1);
	queue.submit(second, speed(2.0), 1);

	REQUIRE(run_until(queue, [&working] { return working.uploads > 0; }));
	CHECK(stuck.uploads == 0);
	// the stuck screen ran first, into the instruction limit, and isn't due any more
	CHECK_FALSE(queue.due(*first));

	queue.exit();
}

TEST_CASE("wasm host: the screens outlive the queue without harm") {
	module_file const module{"paint", paint_module};
	std::shared_ptr<wasm_screen> screen;
	recording_texture texture;
	{
		wasm_screen_queue queue;
		REQUIRE(queue.init());
		screen = queue.acquire({.kind = wasm_screen_kind::cab, .script = module.path(), .key = "cab", .texture = &texture, .frames = 0});
		REQUIRE(run_until(queue, [&queue, &screen] { return queue.due(*screen); }));
		queue.submit(screen, speed(4.0), 1);
		REQUIRE(run_until(queue, [&texture] { return texture.uploads > 0; }));
	}
	// the queue and the runtime are gone, the vehicle still holds its screen
	CHECK(screen != nullptr);
	screen.reset();
}

#endif
