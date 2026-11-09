/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scripting/wasmscreens.h"
#include "scripting/wasmscreenkernels.h"

#include "input/command.h"
#include "interfaces/ITexture.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/dictionary.h"
#include "utilities/utilities.h"

#include <wasm_export.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <format>
#include <fstream>
#include <iterator>
#include <limits>
#include <mutex>
#include <span>
#include <stop_token>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

namespace
{

// the wasm stack of a screen, in bytes
constexpr std::uint32_t stack_size{64 * 1024};
// the most instructions one call into a screen may run, so that a screen stuck in a loop can't hang the worker
constexpr int instruction_limit{200'000'000};
// the largest side of a screen image, in pixels
constexpr int max_image_side{4096};
constexpr std::size_t no_texture{std::numeric_limits<std::size_t>::max()};

// the type of a value in the input of a screen, as eu07.value_type() reports it
enum class value_type : std::int32_t
{
	none = 0,
	floating = 1,
	integer = 2,
	boolean = 3,
	string = 4,
	vec2_list = 5
};

// the input of one refresh, indexed by key. when two values share a key the later one counts, as it does for the
// python screens, which get the values in the same order
class input_view
{
  public:
	struct entry
	{
		value_type type{value_type::none};
		std::size_t index{0};
	};

	explicit input_view(dictionary_source const &Source) : m_source{Source}
	{
		index(Source.floats, value_type::floating);
		index(Source.integers, value_type::integer);
		index(Source.bools, value_type::boolean);
		index(Source.strings, value_type::string);
		index(Source.vec2_lists, value_type::vec2_list);
	}

	entry find(std::string_view const Key) const
	{
		if (auto const lookup{m_entries.find(Key)}; lookup != m_entries.end())
		{
			return lookup->second;
		}
		return {};
	}

	dictionary_source const &source() const
	{
		return m_source;
	}

  private:
	template <typename Sequence_> void index(Sequence_ const &Values, value_type const Type)
	{
		for (std::size_t idx = 0; idx < Values.size(); ++idx)
		{
			m_entries.insert_or_assign(std::string_view{Values[idx].first}, entry{Type, idx});
		}
	}

	dictionary_source const &m_source;
	std::unordered_map<std::string_view, entry> m_entries;
};

struct pending_command
{
	std::string name;
	std::int32_t param1{0};
	std::int32_t param2{0};
	std::uint32_t vehicle{0};
};

// the length of the UTF-8 sequence starting with the byte, 0 for a byte that can't start one
std::size_t utf8_length(std::uint8_t const Lead)
{
	if (Lead < 0x80)
	{
		return 1;
	}
	if (Lead >= 0xC2 && Lead <= 0xDF)
	{
		return 2;
	}
	if (Lead >= 0xE0 && Lead <= 0xEF)
	{
		return 3;
	}
	if (Lead >= 0xF0 && Lead <= 0xF4)
	{
		return 4;
	}
	return 0;
}

// true for text in valid UTF-8
bool is_utf8(std::string_view const Text)
{
	std::size_t idx{0};
	while (idx < Text.size())
	{
		auto const length{utf8_length(static_cast<std::uint8_t>(Text[idx]))};
		if (length == 0 || idx + length > Text.size())
		{
			return false;
		}
		auto const sequence{Text.substr(idx + 1, length - 1)};
		if (false == std::ranges::all_of(sequence, [](char const Byte) { return (static_cast<std::uint8_t>(Byte) & 0xC0) == 0x80; }))
		{
			return false;
		}
		idx += length;
	}
	return true;
}

std::int32_t to_int32(double const Value)
{
	if (std::isnan(Value))
	{
		return 0;
	}
	return static_cast<std::int32_t>(std::clamp(Value, static_cast<double>(std::numeric_limits<std::int32_t>::min()), static_cast<double>(std::numeric_limits<std::int32_t>::max())));
}

std::int32_t to_int32(std::size_t const Value)
{
	return static_cast<std::int32_t>(std::min<std::size_t>(Value, std::numeric_limits<std::int32_t>::max()));
}

} // namespace

class wasm_screen
{
  public:
	wasm_screen(wasm_screen_queue::screen_request const &Request, wasm_screens::frame_schedule const &Schedule)
	    : kind{Request.kind}, script{Request.script}, folder{substr_path(Request.script)}, texture{Request.texture}, m_schedule{Schedule}
	{
	}
	~wasm_screen()
	{
		release_runtime();
	}
	wasm_screen(wasm_screen const &) = delete;
	wasm_screen &operator=(wasm_screen const &) = delete;

	bool poll(std::uint64_t const Frame)
	{
		std::scoped_lock lock{m_schedule_mutex};
		return m_schedule.poll(Frame);
	}

	// frees the module instance. done by the queue when it stops, as the runtime goes with it
	void release_runtime()
	{
		if (exec_env != nullptr)
		{
			wasm_runtime_destroy_exec_env(exec_env);
			exec_env = nullptr;
		}
		if (instance != nullptr)
		{
			wasm_runtime_deinstantiate(instance);
			instance = nullptr;
		}
		failed = true;
	}

	// set when made, read-only afterwards
	wasm_screen_kind const kind;
	std::string const script;
	std::string const folder;
	ITexture *const texture;

	// set by the worker when the screen can't run (any more); a failed screen isn't refreshed
	std::atomic<bool> failed{false};

	// the worker's part
	wasm_module_inst_t instance{nullptr};
	wasm_exec_env_t exec_env{nullptr};
	wasm_function_inst_t render{nullptr};
	wasm_function_inst_t width{nullptr};
	wasm_function_inst_t height{nullptr};
	bool commands_ignored{false}; // a screen outside the cab tried to send a command, and it was logged
	// the call in progress
	input_view const *input{nullptr};
	std::vector<pending_command> *commands{nullptr};
	std::uint32_t vehicle{0};

	// the image, handed from the worker to the main thread
	std::mutex image_mutex;
	std::vector<std::uint8_t> image;
	int image_width{0};
	int image_height{0};
	bool image_pending{false}; // drawn and not yet uploaded
	std::size_t uploaded_texture{no_texture}; // the gl texture the image went to, to upload it again when it's made anew

  private:
	std::mutex m_schedule_mutex;
	wasm_screens::frame_schedule m_schedule;
};

namespace
{

wasm_screen *screen_of(wasm_exec_env_t Env)
{
	return static_cast<wasm_screen *>(wasm_runtime_get_custom_data(wasm_runtime_get_module_inst(Env)));
}

struct lookup_result
{
	input_view const *view{nullptr};
	input_view::entry entry;
};

lookup_result lookup(wasm_exec_env_t Env, char const *Key, std::uint32_t const Length)
{
	auto const *screen{screen_of(Env)};
	if (screen == nullptr || screen->input == nullptr || Key == nullptr)
	{
		return {};
	}
	return {screen->input, screen->input->find(std::string_view{Key, Length})};
}

// the natives of the "eu07" module, which a screen imports. see wasm_screens_doc.md

std::int32_t native_value_type(wasm_exec_env_t Env, char const *Key, std::uint32_t const Length)
{
	return static_cast<std::int32_t>(lookup(Env, Key, Length).entry.type);
}

double native_get_float(wasm_exec_env_t Env, char const *Key, std::uint32_t const Length)
{
	auto const [view, entry]{lookup(Env, Key, Length)};
	using enum value_type;
	switch (entry.type)
	{
	case floating:
		return view->source().floats[entry.index].second;
	case integer:
		return view->source().integers[entry.index].second;
	case boolean:
		return view->source().bools[entry.index].second ? 1.0 : 0.0;
	default:
		return 0.0;
	}
}

std::int32_t native_get_int(wasm_exec_env_t Env, char const *Key, std::uint32_t const Length)
{
	auto const [view, entry]{lookup(Env, Key, Length)};
	using enum value_type;
	switch (entry.type)
	{
	case floating:
		return to_int32(view->source().floats[entry.index].second);
	case integer:
		return view->source().integers[entry.index].second;
	case boolean:
		return view->source().bools[entry.index].second ? 1 : 0;
	default:
		return 0;
	}
}

std::int32_t native_get_bool(wasm_exec_env_t Env, char const *Key, std::uint32_t const Length)
{
	auto const [view, entry]{lookup(Env, Key, Length)};
	using enum value_type;
	switch (entry.type)
	{
	case floating:
		return view->source().floats[entry.index].second != 0.0 ? 1 : 0;
	case integer:
		return view->source().integers[entry.index].second != 0 ? 1 : 0;
	case boolean:
		return view->source().bools[entry.index].second ? 1 : 0;
	default:
		return 0;
	}
}

// copies the string into the buffer, as UTF-8 (names of sceneries and vehicles may be in windows-1250).
// returns the length of the whole string, which may be more than the buffer took, or -1 when there is no such string
std::int32_t native_get_string(wasm_exec_env_t Env, char const *Key, std::uint32_t const Length, char *Buffer, std::uint32_t const Capacity)
{
	auto const [view, entry]{lookup(Env, Key, Length)};
	if (entry.type != value_type::string)
	{
		return -1;
	}
	auto const &value{view->source().strings[entry.index].second};
	auto const text{is_utf8(value) ? value : win1250_to_utf8(value)};
	if (Buffer != nullptr)
	{
		std::memcpy(Buffer, text.data(), std::min<std::size_t>(text.size(), Capacity));
	}
	return to_int32(text.size());
}

std::int32_t native_get_vec2_count(wasm_exec_env_t Env, char const *Key, std::uint32_t const Length)
{
	auto const [view, entry]{lookup(Env, Key, Length)};
	if (entry.type != value_type::vec2_list)
	{
		return 0;
	}
	return to_int32(view->source().vec2_lists[entry.index].second.size());
}

// writes the point as two f32 at Output. returns 1 on success, 0 when there is no such point
std::int32_t native_get_vec2(wasm_exec_env_t Env, char const *Key, std::uint32_t const Length, std::int32_t const Index, std::uint32_t const Output)
{
	auto const [view, entry]{lookup(Env, Key, Length)};
	if (entry.type != value_type::vec2_list || Index < 0)
	{
		return 0;
	}
	auto const &points{view->source().vec2_lists[entry.index].second};
	if (static_cast<std::size_t>(Index) >= points.size())
	{
		return 0;
	}
	auto *instance{wasm_runtime_get_module_inst(Env)};
	std::array<float, 2> const point{points[Index].x, points[Index].y};
	if (false == wasm_runtime_validate_app_addr(instance, Output, sizeof(point)))
	{
		// the runtime has raised an exception, which ends the call
		return 0;
	}
	std::memcpy(wasm_runtime_addr_app_to_native(instance, Output), point.data(), sizeof(point));
	return 1;
}

void native_command(wasm_exec_env_t Env, char const *Name, std::uint32_t const Length, std::int32_t const Param1, std::int32_t const Param2)
{
	auto *screen{screen_of(Env)};
	if (screen == nullptr || screen->commands == nullptr || Name == nullptr)
	{
		return;
	}
	if (screen->kind != wasm_screen_kind::cab)
	{
		if (false == screen->commands_ignored)
		{
			screen->commands_ignored = true;
			WriteLog(std::format("WASM screen \"{}\": commands are only taken from screens in the cab", screen->script));
		}
		return;
	}
	screen->commands->emplace_back(std::string{Name, Length}, Param1, Param2, screen->vehicle);
}

void native_log(wasm_exec_env_t Env, char const *Text, std::uint32_t const Length)
{
	auto const *screen{screen_of(Env)};
	if (screen == nullptr || Text == nullptr)
	{
		return;
	}
	WriteLog(std::format("WASM screen \"{}\": {}", screen->script, std::string_view{Text, Length}));
}

// reads a file from the screen's folder into the buffer. returns the size of the whole file, which may be more
// than the buffer took, or -1 when there is no such file or the path leads outside the folder
std::int32_t native_file_read(wasm_exec_env_t Env, char const *Path, std::uint32_t const Length, char *Buffer, std::uint32_t const Capacity)
{
	auto const *screen{screen_of(Env)};
	if (screen == nullptr || Path == nullptr)
	{
		return -1;
	}
	std::string_view const path{Path, Length};
	if (false == wasm_screens::is_safe_relative_path(path))
	{
		return -1;
	}
	std::ifstream file{screen->folder + std::string{path}, std::ios::binary | std::ios::ate};
	if (false == file.is_open())
	{
		return -1;
	}
	auto const size{static_cast<std::size_t>(std::max<std::streamoff>(file.tellg(), 0))};
	if (Buffer != nullptr && Capacity > 0)
	{
		file.seekg(0);
		file.read(Buffer, static_cast<std::streamsize>(std::min<std::size_t>(size, Capacity)));
	}
	return to_int32(size);
}

// seconds of a steady clock, the same for every screen, for animations that don't depend on how often the
// screen is refreshed
double native_time(wasm_exec_env_t /*Env*/)
{
	static auto const start{std::chrono::steady_clock::now()};
	return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

// the bytes of an image in the module's memory; an empty span for one that lies outside (the runtime then raises an
// exception, which ends the call)
std::span<std::uint8_t> module_image(wasm_exec_env_t Env, std::uint32_t const Offset, int const Width, int const Height)
{
	auto *instance{wasm_runtime_get_module_inst(Env)};
	auto const size{wasm_screens::image_size(Width, Height)};
	if (false == wasm_runtime_validate_app_addr(instance, Offset, size))
	{
		return {};
	}
	return {static_cast<std::uint8_t *>(wasm_runtime_addr_app_to_native(instance, Offset)), size};
}

bool valid_side(std::int32_t const Side)
{
	return Side > 0 && Side <= max_image_side;
}

// led_util's render_post, natively: Target takes (2 Width + 2) x (2 Height + 2) pixels. returns 1, or 0 for an image
// that doesn't fit
std::int32_t native_led_post(wasm_exec_env_t Env, std::uint32_t const Source, std::int32_t const Width, std::int32_t const Height, std::uint32_t const Target, std::int32_t const Bloom)
{
	if (false == valid_side(Width) || false == valid_side(Height))
	{
		return 0;
	}
	auto const source{module_image(Env, Source, Width, Height)};
	auto const target{module_image(Env, Target, Width * 2 + 2, Height * 2 + 2)};
	if (source.empty() || target.empty())
	{
		return 0;
	}
	return wasm_screens::led_post(source, Width, Height, target, Bloom != 0) ? 1 : 0;
}

// Pillow's resize(..., LANCZOS), natively. returns 1, or 0 for an image that doesn't fit
std::int32_t native_resize_lanczos(wasm_exec_env_t Env, std::uint32_t const Source, std::int32_t const Width, std::int32_t const Height, std::uint32_t const Target,
                                   std::int32_t const TargetWidth, std::int32_t const TargetHeight)
{
	if (false == valid_side(Width) || false == valid_side(Height) || false == valid_side(TargetWidth) || false == valid_side(TargetHeight))
	{
		return 0;
	}
	auto const source{module_image(Env, Source, Width, Height)};
	auto const target{module_image(Env, Target, TargetWidth, TargetHeight)};
	if (source.empty() || target.empty())
	{
		return 0;
	}
	return wasm_screens::resize_lanczos(source, Width, Height, target, TargetWidth, TargetHeight) ? 1 : 0;
}

template <typename Function_> NativeSymbol native(char const *Name, Function_ *Function, char const *Signature)
{
	return NativeSymbol{Name, reinterpret_cast<void *>(Function), Signature, nullptr};
}

} // namespace

struct wasm_screen_queue::implementation
{
	struct module_entry
	{
		std::vector<std::uint8_t> bytes; // the runtime reads the module from here as long as it is loaded
		wasm_module_t module{nullptr};
	};

	struct task
	{
		std::shared_ptr<wasm_screen> screen;
		std::shared_ptr<dictionary_source> input;
		std::uint32_t vehicle{0};
	};

	implementation()
	    : natives{native("value_type", &native_value_type, "(*~)i"),
	              native("get_float", &native_get_float, "(*~)F"),
	              native("get_int", &native_get_int, "(*~)i"),
	              native("get_bool", &native_get_bool, "(*~)i"),
	              native("get_string", &native_get_string, "(*~*~)i"),
	              native("get_vec2_count", &native_get_vec2_count, "(*~)i"),
	              native("get_vec2", &native_get_vec2, "(*~ii)i"),
	              native("command", &native_command, "(*~ii)"),
	              native("log", &native_log, "(*~)"),
	              native("file_read", &native_file_read, "(*~*~)i"),
	              native("time", &native_time, "()F"),
	              native("led_post", &native_led_post, "(iiiii)i"),
	              native("resize_lanczos", &native_resize_lanczos, "(iiiiii)i")}
	{
	}

	void run(std::stop_token const &Stop);
	void process(task const &Task);
	bool instantiate(wasm_screen &Screen);
	wasm_module_t load_module(std::string const &Script);
	bool call(wasm_screen &Screen, wasm_function_inst_t Function, std::string_view Name, std::int32_t &Result) const;
	void present(wasm_screen &Screen, std::uint32_t Pixels) const;
	static void upload(wasm_screen &Screen);
	static void execute(pending_command const &Command);

	// the runtime keeps the array and sorts it, so it lives as long as the queue
	std::array<NativeSymbol, 13> natives;
	bool initialized{false};
	std::atomic<std::uint64_t> frame{0};

	std::mutex screens_mutex;
	std::unordered_map<std::string, std::weak_ptr<wasm_screen>> screens;

	std::mutex tasks_mutex;
	std::condition_variable_any tasks_condition;
	std::deque<task> tasks;
	std::jthread worker;

	std::mutex commands_mutex;
	std::vector<pending_command> commands;

	// the worker's, and the queue's once the worker stopped
	std::unordered_map<std::string, module_entry> modules;
};

void wasm_screen_queue::implementation::run(std::stop_token const &Stop)
{
	wasm_runtime_init_thread_env();
	while (false == Stop.stop_requested())
	{
		task next;
		{
			std::unique_lock lock{tasks_mutex};
			if (false == tasks_condition.wait(lock, Stop, [this] { return false == tasks.empty(); }))
			{
				break;
			}
			next = std::move(tasks.front());
			tasks.pop_front();
		}
		process(next);
	}
	wasm_runtime_destroy_thread_env();
}

void wasm_screen_queue::implementation::process(task const &Task)
{
	auto &screen{*Task.screen};
	if (screen.failed)
	{
		return;
	}
	if (screen.instance == nullptr && false == instantiate(screen))
	{
		screen.failed = true;
		return;
	}

	input_view const view{*Task.input};
	std::vector<pending_command> sent;
	screen.input = &view;
	screen.commands = &sent;
	screen.vehicle = Task.vehicle;
	std::int32_t pixels{0};
	auto const rendered{call(screen, screen.render, "screen_render", pixels)};
	screen.input = nullptr;
	screen.commands = nullptr;
	if (false == rendered)
	{
		return;
	}
	// 0: the screen didn't change, keep the image there is
	if (pixels != 0)
	{
		present(screen, static_cast<std::uint32_t>(pixels));
	}
	if (false == sent.empty())
	{
		std::scoped_lock lock{commands_mutex};
		commands.insert(commands.end(), std::make_move_iterator(sent.begin()), std::make_move_iterator(sent.end()));
	}
}

bool wasm_screen_queue::implementation::instantiate(wasm_screen &Screen)
{
	auto *module{load_module(Screen.script)};
	if (module == nullptr)
	{
		return false;
	}
	std::array<char, 256> error{};
	Screen.instance = wasm_runtime_instantiate(module, stack_size, 0, error.data(), static_cast<std::uint32_t>(error.size()));
	if (Screen.instance == nullptr)
	{
		ErrorLog(std::format("WASM screen \"{}\": instantiation failed: {}", Screen.script, error.data()));
		return false;
	}
	wasm_runtime_set_custom_data(Screen.instance, &Screen);
	Screen.exec_env = wasm_runtime_create_exec_env(Screen.instance, stack_size);
	if (Screen.exec_env == nullptr)
	{
		ErrorLog(std::format("WASM screen \"{}\": failed to create the execution environment", Screen.script));
		return false;
	}
	Screen.render = wasm_runtime_lookup_function(Screen.instance, "screen_render");
	Screen.width = wasm_runtime_lookup_function(Screen.instance, "screen_width");
	Screen.height = wasm_runtime_lookup_function(Screen.instance, "screen_height");
	if (Screen.render == nullptr || Screen.width == nullptr || Screen.height == nullptr)
	{
		ErrorLog(std::format("WASM screen \"{}\": the module has to export screen_render, screen_width and screen_height", Screen.script));
		return false;
	}
	if (auto *init{wasm_runtime_lookup_function(Screen.instance, "screen_init")}; init != nullptr)
	{
		std::int32_t result{0};
		if (false == call(Screen, init, "screen_init", result))
		{
			return false;
		}
		if (result != 0)
		{
			ErrorLog(std::format("WASM screen \"{}\": screen_init returned {}", Screen.script, result));
			return false;
		}
	}
	WriteLog(std::format("WASM screen \"{}\": ready", Screen.script));
	return true;
}

wasm_module_t wasm_screen_queue::implementation::load_module(std::string const &Script)
{
	if (auto const lookup{modules.find(Script)}; lookup != modules.end())
	{
		return lookup->second.module;
	}
	// a failure is kept as well, so the module isn't tried again for every screen using it
	auto &entry{modules[Script]};
	std::ifstream file{Script, std::ios::binary};
	if (false == file.is_open())
	{
		ErrorLog(std::format("WASM screen \"{}\": file not found", Script));
		return nullptr;
	}
	entry.bytes.assign(std::istreambuf_iterator<char>{file}, std::istreambuf_iterator<char>{});
	if (entry.bytes.empty() || entry.bytes.size() > std::numeric_limits<std::uint32_t>::max())
	{
		ErrorLog(std::format("WASM screen \"{}\": invalid module size", Script));
		return nullptr;
	}
	std::array<char, 256> error{};
	entry.module = wasm_runtime_load(entry.bytes.data(), static_cast<std::uint32_t>(entry.bytes.size()), error.data(), static_cast<std::uint32_t>(error.size()));
	if (entry.module == nullptr)
	{
		ErrorLog(std::format("WASM screen \"{}\": failed to load the module: {}", Script, error.data()));
	}
	return entry.module;
}

// calls a function of the screen that takes nothing and returns an i32. a failed call disables the screen
bool wasm_screen_queue::implementation::call(wasm_screen &Screen, wasm_function_inst_t Function, std::string_view const Name, std::int32_t &Result) const
{
	wasm_runtime_set_instruction_count_limit(Screen.exec_env, instruction_limit);
	std::array<std::uint32_t, 1> cells{};
	if (false == wasm_runtime_call_wasm(Screen.exec_env, Function, 0, cells.data()))
	{
		auto const *exception{wasm_runtime_get_exception(Screen.instance)};
		ErrorLog(std::format("WASM screen \"{}\": {} failed: {}", Screen.script, Name, exception != nullptr ? exception : "unknown error"));
		Screen.failed = true;
		return false;
	}
	Result = static_cast<std::int32_t>(cells[0]);
	return true;
}

// copies the image the screen drew, Pixels being its address in the module's memory
void wasm_screen_queue::implementation::present(wasm_screen &Screen, std::uint32_t const Pixels) const
{
	std::int32_t width{0};
	std::int32_t height{0};
	if (false == call(Screen, Screen.width, "screen_width", width) || false == call(Screen, Screen.height, "screen_height", height))
	{
		return;
	}
	if (width <= 0 || height <= 0 || width > max_image_side || height > max_image_side)
	{
		ErrorLog(std::format("WASM screen \"{}\": invalid image size {}x{}", Screen.script, width, height));
		Screen.failed = true;
		return;
	}
	auto const size{static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4};
	if (false == wasm_runtime_validate_app_addr(Screen.instance, Pixels, size))
	{
		ErrorLog(std::format("WASM screen \"{}\": the image lies outside the module's memory", Screen.script));
		Screen.failed = true;
		return;
	}
	auto const *pixels{static_cast<std::uint8_t const *>(wasm_runtime_addr_app_to_native(Screen.instance, Pixels))};

	std::scoped_lock lock{Screen.image_mutex};
	Screen.image.assign(pixels, pixels + size);
	Screen.image_width = width;
	Screen.image_height = height;
	Screen.image_pending = true;
}

void wasm_screen_queue::implementation::upload(wasm_screen &Screen)
{
	if (Screen.texture == nullptr)
	{
		return;
	}
	std::scoped_lock lock{Screen.image_mutex};
	if (Screen.image.empty() || false == Screen.texture->get_is_ready())
	{
		// nothing drawn yet, or the texture isn't made yet or was released: the image goes up once it's there
		return;
	}
	if (false == Screen.image_pending && Screen.texture->get_id() == Screen.uploaded_texture)
	{
		return;
	}
	Screen.texture->update_from_memory(Screen.image_width, Screen.image_height, Screen.image.data());
	Screen.uploaded_texture = Screen.texture->get_id();
	Screen.image_pending = false;
}

void wasm_screen_queue::implementation::execute(pending_command const &Command)
{
	auto const lookup{simulation::commandMap.find(Command.name)};
	if (lookup == simulation::commandMap.end())
	{
		ErrorLog(std::format("WASM screen: command [{}] not found", Command.name));
		return;
	}
	command_data data{};
	data.command = lookup->second;
	data.action = GLFW_PRESS;
	data.param1 = Command.param1;
	data.param2 = Command.param2;
	WriteLog(std::format("WASM screen: executing command [{}] with params: P1={} P2={} Target ID={}", Command.name, Command.param1, Command.param2, Command.vehicle));
	simulation::Commands.push(data, static_cast<std::uint32_t>(command_target::vehicle) | Command.vehicle);
}

wasm_screen_queue::wasm_screen_queue() : m_impl{std::make_unique<implementation>()} {}

wasm_screen_queue::~wasm_screen_queue()
{
	exit();
}

bool wasm_screen_queue::init()
{
	if (m_impl->initialized)
	{
		return true;
	}
	RuntimeInitArgs arguments{};
	arguments.mem_alloc_type = Alloc_With_System_Allocator;
	arguments.native_module_name = "eu07";
	arguments.native_symbols = m_impl->natives.data();
	arguments.n_native_symbols = static_cast<std::uint32_t>(m_impl->natives.size());
	arguments.running_mode = Mode_Interp;
	if (false == wasm_runtime_full_init(&arguments))
	{
		ErrorLog("WASM screens: failed to initialize the runtime");
		return false;
	}
	wasm_runtime_set_log_level(WASM_LOG_LEVEL_ERROR);
	m_impl->initialized = true;
	auto *impl{m_impl.get()};
	m_impl->worker = std::jthread([impl](std::stop_token const &Stop) { impl->run(Stop); });
	WriteLog(std::format("WASM screens: setup complete, refreshed every {} frames in the cab and every {} frames outside", Global.wasm_cabframes, Global.wasm_externalframes));
	return true;
}

void wasm_screen_queue::exit()
{
	if (false == m_impl->initialized)
	{
		return;
	}
	if (m_impl->worker.joinable())
	{
		m_impl->worker.request_stop();
		m_impl->worker.join();
	}
	{
		std::scoped_lock lock{m_impl->tasks_mutex};
		m_impl->tasks.clear();
	}
	{
		std::scoped_lock lock{m_impl->commands_mutex};
		m_impl->commands.clear();
	}
	{
		// the vehicles may hold their screens past this point, but the runtime goes now
		std::scoped_lock lock{m_impl->screens_mutex};
		for (auto const &[key, screen] : m_impl->screens)
		{
			if (auto const live{screen.lock()}; live != nullptr)
			{
				live->release_runtime();
			}
		}
		m_impl->screens.clear();
	}
	for (auto const &[script, entry] : m_impl->modules)
	{
		if (entry.module != nullptr)
		{
			wasm_runtime_unload(entry.module);
		}
	}
	m_impl->modules.clear();
	wasm_runtime_destroy();
	m_impl->initialized = false;
}

std::shared_ptr<wasm_screen> wasm_screen_queue::acquire(screen_request const &Request)
{
	if (false == m_impl->initialized || Request.script.empty())
	{
		return nullptr;
	}
	auto const key{Request.key.empty() ? Request.script : Request.key};
	std::scoped_lock lock{m_impl->screens_mutex};
	if (auto const lookup{m_impl->screens.find(key)}; lookup != m_impl->screens.end())
	{
		if (auto existing{lookup->second.lock()}; existing != nullptr)
		{
			return existing;
		}
	}
	auto const configured{Request.kind == wasm_screen_kind::cab ? Global.wasm_cabframes : Global.wasm_externalframes};
	auto const interval{wasm_screens::frame_schedule::effective_interval(configured, Request.frames)};
	auto screen{std::make_shared<wasm_screen>(Request, wasm_screens::frame_schedule{m_impl->frame.load(), interval, wasm_screens::frame_schedule::phase(key, interval)})};
	m_impl->screens.insert_or_assign(key, screen);
	return screen;
}

bool wasm_screen_queue::due(wasm_screen &Screen) const
{
	if (false == m_impl->initialized || Screen.failed)
	{
		return false;
	}
	return Screen.poll(m_impl->frame.load());
}

void wasm_screen_queue::submit(std::shared_ptr<wasm_screen> const &Screen, std::shared_ptr<dictionary_source> Input, std::uint32_t const Vehicle)
{
	if (false == m_impl->initialized || Screen == nullptr || Input == nullptr || Screen->failed)
	{
		return;
	}
	{
		std::scoped_lock lock{m_impl->tasks_mutex};
		auto const pending{std::ranges::find(m_impl->tasks, Screen, &implementation::task::screen)};
		if (pending != m_impl->tasks.end())
		{
			// the refresh still waiting gets the newer input
			pending->input = std::move(Input);
			pending->vehicle = Vehicle;
		}
		else
		{
			m_impl->tasks.emplace_back(Screen, std::move(Input), Vehicle);
		}
	}
	m_impl->tasks_condition.notify_one();
}

void wasm_screen_queue::update()
{
	if (false == m_impl->initialized)
	{
		return;
	}
	++m_impl->frame;
	{
		std::scoped_lock lock{m_impl->screens_mutex};
		std::erase_if(m_impl->screens, [](auto const &Entry) { return Entry.second.expired(); });
		for (auto const &[key, screen] : m_impl->screens)
		{
			if (auto const live{screen.lock()}; live != nullptr)
			{
				implementation::upload(*live);
			}
		}
	}
	std::vector<pending_command> sent;
	{
		std::scoped_lock lock{m_impl->commands_mutex};
		sent.swap(m_impl->commands);
	}
	for (auto const &command : sent)
	{
		implementation::execute(command);
	}
}
