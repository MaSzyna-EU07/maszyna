/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scripting/scriptscreens.h"
#include "scripting/scriptscreenprogram.h"
#include "scripting/scriptscreenkernels.h"

#include "input/command.h"
#include "interfaces/ITexture.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/dictionary.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <limits>
#include <mutex>
#include <span>
#include <stop_token>
#include <string_view>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <vector>

namespace script_screens
{

namespace
{

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

} // namespace

std::string utf8_text(std::string const &Text)
{
	return is_utf8(Text) ? Text : win1250_to_utf8(Text);
}

std::optional<std::string> read_screen_file(std::string const &Folder, std::string_view const Path)
{
	if (false == is_safe_relative_path(Path))
	{
		return std::nullopt;
	}
	return read_regular_file(Folder + std::string{Path});
}

std::optional<std::string> read_regular_file(std::string const &Path)
{
	// a folder or a device (CON, NUL, COM1...) isn't a regular file
	std::error_code error;
	// in the code page of the system, as the game opens its files
	std::filesystem::path const path{Path};
	if (false == std::filesystem::is_regular_file(path, error))
	{
		return std::nullopt;
	}
	auto const size{std::filesystem::file_size(path, error)};
	if (error || size > max_screen_file)
	{
		return std::nullopt;
	}
	std::ifstream file{path, std::ios::binary};
	if (false == file.is_open())
	{
		return std::nullopt;
	}
	std::string contents(static_cast<std::size_t>(size), '\0');
	file.read(contents.data(), static_cast<std::streamsize>(contents.size()));
	// what was read, should the file have shrunk meanwhile
	contents.resize(static_cast<std::size_t>(file.gcount()));
	return contents;
}

bool output_limits::take_command(render_call const &Call, std::string_view const Name)
{
	if (Call.commands.size() < max_commands_per_call && Name.size() <= max_command_name)
	{
		return true;
	}
	if (false == m_commands_dropped)
	{
		m_commands_dropped = true;
		ErrorLog(std::format("{}: commands over {} in a refresh, or with names over {} characters, are dropped", m_name, max_commands_per_call, max_command_name));
	}
	return false;
}

std::optional<std::string> output_limits::take_log(std::string_view Text)
{
	if (m_log_lines >= max_log_lines_per_call)
	{
		if (false == m_logs_dropped)
		{
			m_logs_dropped = true;
			ErrorLog(std::format("{}: log lines over {} in a call are dropped", m_name, max_log_lines_per_call));
		}
		return std::nullopt;
	}
	++m_log_lines;
	if (Text.size() <= max_log_line)
	{
		return std::format("{}: {}", m_name, Text);
	}
	// cut on the start of a UTF-8 character
	auto end{max_log_line};
	while (end > 0 && (static_cast<std::uint8_t>(Text[end]) & 0xC0) == 0x80)
	{
		--end;
	}
	return std::format("{}: {} [cut at {} of {} bytes]", m_name, Text.substr(0, end), end, Text.size());
}

double screen_time()
{
	static auto const start{std::chrono::steady_clock::now()};
	return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

bool valid_image_side(int const Side)
{
	return Side > 0 && Side <= max_image_side;
}

} // namespace script_screens

namespace
{

constexpr std::size_t no_texture{std::numeric_limits<std::size_t>::max()};

std::string_view language_name(script_language const Language)
{
	return Language == script_language::lua ? "Lua" : "WASM";
}

// the rows of the image the other way round. the script draws from the top row down; the texture takes the bottom row
// first, as the python screens give it (abstractscreenrenderer flips the image before handing it over)
std::vector<std::uint8_t> flipped(std::span<std::uint8_t const> const Pixels, int const Width, int const Height)
{
	auto const row{static_cast<std::size_t>(Width) * 4};
	auto const height{static_cast<std::size_t>(Height)};
	std::vector<std::uint8_t> result(Pixels.size());
	for (std::size_t y = 0; y < height; ++y)
	{
		std::memcpy(result.data() + (height - 1 - y) * row, Pixels.data() + y * row, row);
	}
	return result;
}

// the post-processing of the image done on the cpu. false when the kernels refused a size
bool post_on_cpu(std::vector<std::uint8_t> &Pixels, int &Width, int &Height, std::vector<script_screens::post_step> &Steps)
{
	script_screens::post_image image{std::move(Pixels), Width, Height};
	auto const done{script_screens::apply_post(image, Steps)};
	Pixels = std::move(image.pixels);
	Width = image.width;
	Height = image.height;
	Steps.clear();
	return done;
}

} // namespace

class script_screen
{
  public:
	script_screen(script_screen_queue::screen_request const &Request, script_screens::frame_schedule const &Schedule)
	    : kind{Request.kind}, language{Request.language}, script{Request.script}, texture{Request.texture}, m_schedule{Schedule}
	{
	}
	~script_screen() = default;
	script_screen(script_screen const &) = delete;
	script_screen &operator=(script_screen const &) = delete;

	bool poll(std::uint64_t const Frame)
	{
		std::scoped_lock lock{m_schedule_mutex};
		return m_schedule.poll(Frame);
	}

	// set when made, read-only afterwards
	script_screen_kind const kind;
	script_language const language;
	std::string const script;
	ITexture *const texture;

	// set by the worker when the screen can't run (any more); a failed screen isn't refreshed
	std::atomic<bool> failed{false};
	// the worker's, and the queue's once the worker stopped
	std::unique_ptr<script_screens::program> program;

	// the image, handed from the worker to the main thread: bottom row first as the texture takes it, or, while it has
	// post-processing to run on the gpu, from the top row down as the script drew it
	std::mutex image_mutex;
	std::vector<std::uint8_t> image;
	int image_width{0};
	int image_height{0};
	std::vector<script_screens::post_step> post;
	bool image_pending{false}; // drawn and not yet uploaded
	std::size_t uploaded_texture{no_texture}; // the gl texture the image went to, to upload it again when it's made anew

  private:
	std::mutex m_schedule_mutex;
	script_screens::frame_schedule m_schedule;
};

struct script_screen_queue::implementation
{
	struct task
	{
		std::shared_ptr<script_screen> screen;
		std::shared_ptr<dictionary_source> input;
		std::uint32_t vehicle{0};
	};

	void run(std::stop_token const &Stop);
	void process(task const &Task);
	script_screens::runtime *runtime_of(script_language const Language) const
	{
		return Language == script_language::lua ? lua.get() : wasm.get();
	}
	static void present(script_screen &Screen, script_screens::render_call &Call);
	void upload(script_screen &Screen);
	// the post-processing of the image the gpu couldn't do, done on the cpu. false when it failed too
	static bool finish_on_cpu(script_screen &Screen);
	static void execute(script_screens::pending_command const &Command);

	std::unique_ptr<script_screens::runtime> lua;
	std::unique_ptr<script_screens::runtime> wasm;
	bool initialized{false};
	std::atomic<std::uint64_t> frame{0};

	// the main thread's: the gpu post-processing, none when the renderer has none, or it failed
	script_screens::post_processor *post_processor{nullptr};
	// read by the worker: the post-processing is left to the main thread, for the gpu
	std::atomic<bool> post_on_gpu{false};

	std::mutex screens_mutex;
	std::unordered_map<std::string, std::weak_ptr<script_screen>> screens;

	std::mutex tasks_mutex;
	std::condition_variable_any tasks_condition;
	std::deque<task> tasks;
	std::jthread worker;

	std::mutex commands_mutex;
	std::vector<script_screens::pending_command> commands;
};

void script_screen_queue::implementation::run(std::stop_token const &Stop)
{
	for (auto *runtime : {lua.get(), wasm.get()})
	{
		if (runtime != nullptr)
		{
			runtime->attach_thread();
		}
	}
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
	for (auto *runtime : {lua.get(), wasm.get()})
	{
		if (runtime != nullptr)
		{
			runtime->detach_thread();
		}
	}
}

void script_screen_queue::implementation::process(task const &Task)
{
	auto &screen{*Task.screen};
	if (screen.failed)
	{
		return;
	}
	if (screen.program == nullptr)
	{
		auto *runtime{runtime_of(screen.language)};
		screen.program = runtime != nullptr ? runtime->load(screen.script, screen.kind) : nullptr;
		if (screen.program == nullptr)
		{
			screen.failed = true;
			return;
		}
	}

	script_screens::render_call call{*Task.input, Task.vehicle};
	if (false == screen.program->render(call))
	{
		screen.failed = true;
		return;
	}
	if (false == call.pixels.empty())
	{
		if (false == script_screens::valid_image_side(call.width) || false == script_screens::valid_image_side(call.height) ||
		    call.pixels.size() != script_screens::image_size(call.width, call.height))
		{
			ErrorLog(std::format("{} screen \"{}\": invalid image size {}x{}", language_name(screen.language), screen.script, call.width, call.height));
			screen.failed = true;
			return;
		}
		if (false == call.post.empty() && false == post_on_gpu && false == post_on_cpu(call.pixels, call.width, call.height, call.post))
		{
			ErrorLog(std::format("{} screen \"{}\": the post-processing of the image failed", language_name(screen.language), screen.script));
			screen.failed = true;
			return;
		}
		present(screen, call);
	}
	if (false == call.commands.empty())
	{
		std::scoped_lock lock{commands_mutex};
		commands.insert(commands.end(), std::make_move_iterator(call.commands.begin()), std::make_move_iterator(call.commands.end()));
	}
}

// keeps the image the screen drew, turned bottom row first for the texture; with post-processing left for the gpu, as
// it was drawn
void script_screen_queue::implementation::present(script_screen &Screen, script_screens::render_call &Call)
{
	std::scoped_lock lock{Screen.image_mutex};
	if (Call.post.empty())
	{
		Screen.image = flipped(Call.pixels, Call.width, Call.height);
	}
	else
	{
		Screen.image = std::move(Call.pixels);
	}
	Screen.post = std::move(Call.post);
	Screen.image_width = Call.width;
	Screen.image_height = Call.height;
	Screen.image_pending = true;
}

bool script_screen_queue::implementation::finish_on_cpu(script_screen &Screen)
{
	if (false == post_on_cpu(Screen.image, Screen.image_width, Screen.image_height, Screen.post))
	{
		ErrorLog(std::format("{} screen \"{}\": the post-processing of the image failed", language_name(Screen.language), Screen.script));
		Screen.image.clear();
		return false;
	}
	Screen.image = flipped(Screen.image, Screen.image_width, Screen.image_height);
	return true;
}

void script_screen_queue::implementation::upload(script_screen &Screen)
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
	auto const on_gpu{false == Screen.post.empty() && post_processor != nullptr &&
	                  post_processor->apply(*Screen.texture, Screen.image, Screen.image_width, Screen.image_height, Screen.post)};
	if (false == on_gpu)
	{
		if (false == Screen.post.empty())
		{
			// the gpu can't do it: the cpu does, here this time, and on the worker from now on
			if (post_processor != nullptr)
			{
				WriteLog("Script screens: the post-processing of the images goes to the cpu");
			}
			post_processor = nullptr;
			post_on_gpu = false;
			if (false == finish_on_cpu(Screen))
			{
				return;
			}
		}
		Screen.texture->update_from_memory(Screen.image_width, Screen.image_height, Screen.image.data());
	}
	Screen.uploaded_texture = Screen.texture->get_id();
	Screen.image_pending = false;
}

void script_screen_queue::implementation::execute(script_screens::pending_command const &Command)
{
	auto const lookup{simulation::commandMap.find(Command.name)};
	if (lookup == simulation::commandMap.end())
	{
		ErrorLog(std::format("Script screen: command [{}] not found", Command.name));
		return;
	}
	command_data data{};
	data.command = lookup->second;
	data.action = GLFW_PRESS;
	data.param1 = Command.param1;
	data.param2 = Command.param2;
	WriteLog(std::format("Script screen: executing command [{}] with params: P1={} P2={} Target ID={}", Command.name, Command.param1, Command.param2, Command.vehicle));
	simulation::Commands.push(data, static_cast<std::uint32_t>(command_target::vehicle) | Command.vehicle);
}

script_screen_queue::script_screen_queue() : m_impl{std::make_unique<implementation>()} {}

script_screen_queue::~script_screen_queue()
{
	exit();
}

bool script_screen_queue::init()
{
	if (m_impl->initialized)
	{
		return true;
	}
	if (Global.luascreens_enabled)
	{
		m_impl->lua = script_screens::make_lua_runtime();
	}
	if (Global.wasm_enabled)
	{
		m_impl->wasm = script_screens::make_wasm_runtime();
	}
	if (m_impl->lua == nullptr && m_impl->wasm == nullptr)
	{
		return false;
	}
	m_impl->initialized = true;
	auto *impl{m_impl.get()};
	m_impl->worker = std::jthread([impl](std::stop_token const &Stop) { impl->run(Stop); });
	WriteLog(std::format("Script screens: setup complete (lua: {}, wasm: {}), refreshed every {} frames in the cab and every {} frames outside", m_impl->lua != nullptr ? "yes" : "no",
	                     m_impl->wasm != nullptr ? "yes" : "no", Global.screens_cabframes, Global.screens_externalframes));
	return true;
}

void script_screen_queue::exit()
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
		// the vehicles may hold their screens past this point, but the runtimes go now
		std::scoped_lock lock{m_impl->screens_mutex};
		for (auto const &[key, screen] : m_impl->screens)
		{
			if (auto const live{screen.lock()}; live != nullptr)
			{
				live->program.reset();
				live->failed = true;
			}
		}
		m_impl->screens.clear();
	}
	m_impl->lua.reset();
	m_impl->wasm.reset();
	m_impl->initialized = false;
}

bool script_screen_queue::runs(script_language const Language) const
{
	return m_impl->initialized && m_impl->runtime_of(Language) != nullptr;
}

std::shared_ptr<script_screen> script_screen_queue::acquire(screen_request const &Request)
{
	if (false == runs(Request.language) || Request.script.empty())
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
	auto const configured{Request.kind == script_screen_kind::cab ? Global.screens_cabframes : Global.screens_externalframes};
	auto const interval{script_screens::frame_schedule::effective_interval(configured, Request.frames)};
	auto screen{std::make_shared<script_screen>(Request, script_screens::frame_schedule{m_impl->frame.load(), interval, script_screens::frame_schedule::phase(key, interval)})};
	m_impl->screens.insert_or_assign(key, screen);
	WriteLog(std::format("{} screen \"{}\": refreshed every {} frames", language_name(Request.language), Request.script, interval));
	return screen;
}

bool script_screen_queue::due(script_screen &Screen) const
{
	if (false == m_impl->initialized || Screen.failed)
	{
		return false;
	}
	return Screen.poll(m_impl->frame.load());
}

void script_screen_queue::submit(std::shared_ptr<script_screen> const &Screen, std::shared_ptr<dictionary_source> Input, std::uint32_t const Vehicle)
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

void script_screen_queue::update()
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
				m_impl->upload(*live);
			}
		}
	}
	std::vector<script_screens::pending_command> sent;
	{
		std::scoped_lock lock{m_impl->commands_mutex};
		sent.swap(m_impl->commands);
	}
	for (auto const &command : sent)
	{
		implementation::execute(command);
	}
}

void script_screen_queue::set_post_processor(script_screens::post_processor *Processor)
{
	m_impl->post_processor = Processor;
	m_impl->post_on_gpu = Processor != nullptr;
}

#ifndef WITH_LUA
std::unique_ptr<script_screens::runtime> script_screens::make_lua_runtime()
{
	return nullptr;
}
#endif

#ifndef WITH_WASM
std::unique_ptr<script_screens::runtime> script_screens::make_wasm_runtime()
{
	return nullptr;
}
#endif
