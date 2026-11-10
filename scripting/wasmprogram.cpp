/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the WebAssembly screens: modules run by the WAMR interpreter (ref/wamr), with the "eu07" import module of
// script_screens_doc.md

#include "stdafx.h"
#include "scripting/scriptscreenprogram.h"
#include "scripting/scriptscreenkernels.h"

#include "utilities/Logs.h"
#include "utilities/dictionary.h"
#include "utilities/utilities.h"

#include <wasm_export.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
#include <iterator>
#include <limits>
#include <span>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace script_screens
{

namespace
{

// the wasm stack of a screen, in bytes
constexpr std::uint32_t stack_size{64 * 1024};

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

class wasm_program final : public program
{
  public:
	wasm_program(std::string const &Script, script_screen_kind const Kind) : kind{Kind}, script{Script}, folder{substr_path(Script)} {}
	~wasm_program() override
	{
		if (exec_env != nullptr)
		{
			wasm_runtime_destroy_exec_env(exec_env);
		}
		if (instance != nullptr)
		{
			wasm_runtime_deinstantiate(instance);
		}
	}
	wasm_program(wasm_program const &) = delete;
	wasm_program &operator=(wasm_program const &) = delete;

	bool instantiate(wasm_module_t Module);
	bool render(render_call &Call) override;

	script_screen_kind const kind;
	std::string const script;
	std::string const folder;
	// the call in progress
	input_view const *input{nullptr};
	render_call *call{nullptr};
	bool commands_ignored{false}; // a screen outside the cab tried to send a command, and it was logged

  private:
	bool call_i32(wasm_function_inst_t Function, std::string_view Name, std::int32_t &Result);
	bool present(std::uint32_t Pixels, render_call &Call);

	wasm_module_inst_t instance{nullptr};
	wasm_exec_env_t exec_env{nullptr};
	wasm_function_inst_t m_render{nullptr};
	wasm_function_inst_t m_width{nullptr};
	wasm_function_inst_t m_height{nullptr};
};

wasm_program *program_of(wasm_exec_env_t Env)
{
	return static_cast<wasm_program *>(wasm_runtime_get_custom_data(wasm_runtime_get_module_inst(Env)));
}

struct lookup_result
{
	input_view const *view{nullptr};
	input_view::entry entry;
};

lookup_result lookup(wasm_exec_env_t Env, char const *Key, std::uint32_t const Length)
{
	auto const *screen{program_of(Env)};
	if (screen == nullptr || screen->input == nullptr || Key == nullptr)
	{
		return {};
	}
	return {screen->input, screen->input->find(std::string_view{Key, Length})};
}

// the natives of the "eu07" module, which a screen imports. see script_screens_doc.md

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

// copies the string into the buffer, as UTF-8. returns the length of the whole string, which may be more than the
// buffer took, or -1 when there is no such string
std::int32_t native_get_string(wasm_exec_env_t Env, char const *Key, std::uint32_t const Length, char *Buffer, std::uint32_t const Capacity)
{
	auto const [view, entry]{lookup(Env, Key, Length)};
	if (entry.type != value_type::string)
	{
		return -1;
	}
	auto const text{utf8_text(view->source().strings[entry.index].second)};
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
	auto *screen{program_of(Env)};
	if (screen == nullptr || screen->call == nullptr || Name == nullptr)
	{
		return;
	}
	if (screen->kind != script_screen_kind::cab)
	{
		if (false == screen->commands_ignored)
		{
			screen->commands_ignored = true;
			WriteLog(std::format("WASM screen \"{}\": commands are only taken from screens in the cab", screen->script));
		}
		return;
	}
	screen->call->commands.emplace_back(std::string{Name, Length}, Param1, Param2, screen->call->vehicle);
}

void native_log(wasm_exec_env_t Env, char const *Text, std::uint32_t const Length)
{
	auto const *screen{program_of(Env)};
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
	auto const *screen{program_of(Env)};
	if (screen == nullptr || Path == nullptr)
	{
		return -1;
	}
	auto const contents{read_screen_file(screen->folder, std::string_view{Path, Length})};
	if (false == contents.has_value())
	{
		return -1;
	}
	if (Buffer != nullptr)
	{
		std::memcpy(Buffer, contents->data(), std::min<std::size_t>(contents->size(), Capacity));
	}
	return to_int32(contents->size());
}

double native_time(wasm_exec_env_t /*Env*/)
{
	return screen_time();
}

// the bytes of an image in the module's memory; an empty span for one that lies outside (the runtime then raises an
// exception, which ends the call)
std::span<std::uint8_t> module_image(wasm_exec_env_t Env, std::uint32_t const Offset, int const Width, int const Height)
{
	auto *instance{wasm_runtime_get_module_inst(Env)};
	auto const size{image_size(Width, Height)};
	if (false == wasm_runtime_validate_app_addr(instance, Offset, size))
	{
		return {};
	}
	return {static_cast<std::uint8_t *>(wasm_runtime_addr_app_to_native(instance, Offset)), size};
}

// led_util's render_post, natively: Target takes (2 Width + 2) x (2 Height + 2) pixels. returns 1, or 0 for an image
// that doesn't fit
std::int32_t native_led_post(wasm_exec_env_t Env, std::uint32_t const Source, std::int32_t const Width, std::int32_t const Height, std::uint32_t const Target, std::int32_t const Bloom)
{
	if (false == valid_image_side(Width) || false == valid_image_side(Height))
	{
		return 0;
	}
	auto const source{module_image(Env, Source, Width, Height)};
	auto const target{module_image(Env, Target, Width * 2 + 2, Height * 2 + 2)};
	if (source.empty() || target.empty())
	{
		return 0;
	}
	return led_post(source, Width, Height, target, Bloom != 0) ? 1 : 0;
}

// Pillow's resize(..., LANCZOS), natively. returns 1, or 0 for an image that doesn't fit
std::int32_t native_resize_lanczos(wasm_exec_env_t Env, std::uint32_t const Source, std::int32_t const Width, std::int32_t const Height, std::uint32_t const Target,
                                   std::int32_t const TargetWidth, std::int32_t const TargetHeight)
{
	if (false == valid_image_side(Width) || false == valid_image_side(Height) || false == valid_image_side(TargetWidth) || false == valid_image_side(TargetHeight))
	{
		return 0;
	}
	auto const source{module_image(Env, Source, Width, Height)};
	auto const target{module_image(Env, Target, TargetWidth, TargetHeight)};
	if (source.empty() || target.empty())
	{
		return 0;
	}
	return resize_lanczos(source, Width, Height, target, TargetWidth, TargetHeight) ? 1 : 0;
}

template <typename Function_> NativeSymbol native(char const *Name, Function_ *Function, char const *Signature)
{
	return NativeSymbol{Name, reinterpret_cast<void *>(Function), Signature, nullptr};
}

bool wasm_program::instantiate(wasm_module_t const Module)
{
	std::array<char, 256> error{};
	instance = wasm_runtime_instantiate(Module, stack_size, 0, error.data(), static_cast<std::uint32_t>(error.size()));
	if (instance == nullptr)
	{
		ErrorLog(std::format("WASM screen \"{}\": instantiation failed: {}", script, error.data()));
		return false;
	}
	wasm_runtime_set_custom_data(instance, this);
	exec_env = wasm_runtime_create_exec_env(instance, stack_size);
	if (exec_env == nullptr)
	{
		ErrorLog(std::format("WASM screen \"{}\": failed to create the execution environment", script));
		return false;
	}
	m_render = wasm_runtime_lookup_function(instance, "screen_render");
	m_width = wasm_runtime_lookup_function(instance, "screen_width");
	m_height = wasm_runtime_lookup_function(instance, "screen_height");
	if (m_render == nullptr || m_width == nullptr || m_height == nullptr)
	{
		ErrorLog(std::format("WASM screen \"{}\": the module has to export screen_render, screen_width and screen_height", script));
		return false;
	}
	if (auto *init{wasm_runtime_lookup_function(instance, "screen_init")}; init != nullptr)
	{
		std::int32_t result{0};
		if (false == call_i32(init, "screen_init", result))
		{
			return false;
		}
		if (result != 0)
		{
			ErrorLog(std::format("WASM screen \"{}\": screen_init returned {}", script, result));
			return false;
		}
	}
	WriteLog(std::format("WASM screen \"{}\": ready", script));
	return true;
}

// calls a function of the screen that takes nothing and returns an i32
bool wasm_program::call_i32(wasm_function_inst_t Function, std::string_view const Name, std::int32_t &Result)
{
	wasm_runtime_set_instruction_count_limit(exec_env, static_cast<int>(instruction_limit));
	std::array<std::uint32_t, 1> cells{};
	if (false == wasm_runtime_call_wasm(exec_env, Function, 0, cells.data()))
	{
		auto const *exception{wasm_runtime_get_exception(instance)};
		ErrorLog(std::format("WASM screen \"{}\": {} failed: {}", script, Name, exception != nullptr ? exception : "unknown error"));
		return false;
	}
	Result = static_cast<std::int32_t>(cells[0]);
	return true;
}

bool wasm_program::render(render_call &Call)
{
	input_view const view{Call.input};
	input = &view;
	call = &Call;
	std::int32_t pixels{0};
	auto const rendered{call_i32(m_render, "screen_render", pixels)};
	input = nullptr;
	call = nullptr;
	if (false == rendered)
	{
		return false;
	}
	// 0: the screen didn't change, keep the image there is
	return pixels == 0 || present(static_cast<std::uint32_t>(pixels), Call);
}

// copies the image the screen drew, Pixels being its address in the module's memory
bool wasm_program::present(std::uint32_t const Pixels, render_call &Call)
{
	std::int32_t width{0};
	std::int32_t height{0};
	if (false == call_i32(m_width, "screen_width", width) || false == call_i32(m_height, "screen_height", height))
	{
		return false;
	}
	if (false == valid_image_side(width) || false == valid_image_side(height))
	{
		ErrorLog(std::format("WASM screen \"{}\": invalid image size {}x{}", script, width, height));
		return false;
	}
	auto const size{image_size(width, height)};
	if (false == wasm_runtime_validate_app_addr(instance, Pixels, size))
	{
		ErrorLog(std::format("WASM screen \"{}\": the image lies outside the module's memory", script));
		return false;
	}
	auto const *pixels{static_cast<std::uint8_t const *>(wasm_runtime_addr_app_to_native(instance, Pixels))};
	Call.pixels.assign(pixels, pixels + size);
	Call.width = width;
	Call.height = height;
	return true;
}

class wasm_runtime_host final : public runtime
{
  public:
	wasm_runtime_host()
	    : m_natives{native("value_type", &native_value_type, "(*~)i"),
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
	~wasm_runtime_host() override
	{
		for (auto const &[script, entry] : m_modules)
		{
			if (entry.module != nullptr)
			{
				wasm_runtime_unload(entry.module);
			}
		}
		if (m_started)
		{
			wasm_runtime_destroy();
		}
	}
	wasm_runtime_host(wasm_runtime_host const &) = delete;
	wasm_runtime_host &operator=(wasm_runtime_host const &) = delete;

	bool start();
	std::unique_ptr<program> load(std::string const &Script, script_screen_kind Kind) override;
	void attach_thread() override
	{
		wasm_runtime_init_thread_env();
	}
	void detach_thread() override
	{
		wasm_runtime_destroy_thread_env();
	}

  private:
	struct module_entry
	{
		std::vector<std::uint8_t> bytes; // the runtime reads the module from here as long as it is loaded
		wasm_module_t module{nullptr};
	};

	wasm_module_t load_module(std::string const &Script);

	// the runtime keeps the array and sorts it, so it lives as long as the runtime
	std::array<NativeSymbol, 13> m_natives;
	bool m_started{false};
	std::unordered_map<std::string, module_entry> m_modules; // the worker's
};

bool wasm_runtime_host::start()
{
	RuntimeInitArgs arguments{};
	arguments.mem_alloc_type = Alloc_With_System_Allocator;
	arguments.native_module_name = "eu07";
	arguments.native_symbols = m_natives.data();
	arguments.n_native_symbols = static_cast<std::uint32_t>(m_natives.size());
	arguments.running_mode = Mode_Interp;
	if (false == wasm_runtime_full_init(&arguments))
	{
		ErrorLog("WASM screens: failed to initialize the runtime");
		return false;
	}
	wasm_runtime_set_log_level(WASM_LOG_LEVEL_ERROR);
	m_started = true;
	return true;
}

wasm_module_t wasm_runtime_host::load_module(std::string const &Script)
{
	if (auto const lookup{m_modules.find(Script)}; lookup != m_modules.end())
	{
		return lookup->second.module;
	}
	// a failure is kept as well, so the module isn't tried again for every screen using it
	auto &entry{m_modules[Script]};
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

std::unique_ptr<program> wasm_runtime_host::load(std::string const &Script, script_screen_kind const Kind)
{
	auto *module{load_module(Script)};
	if (module == nullptr)
	{
		return nullptr;
	}
	auto loaded{std::make_unique<wasm_program>(Script, Kind)};
	if (false == loaded->instantiate(module))
	{
		return nullptr;
	}
	return loaded;
}

} // namespace

std::unique_ptr<runtime> make_wasm_runtime()
{
	auto started{std::make_unique<wasm_runtime_host>()};
	if (false == started->start())
	{
		return nullptr;
	}
	return started;
}

} // namespace script_screens
