/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/physicslog.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <format>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string_view>
#include <system_error>
#include <utility>

#if defined(__linux__)
#include <sys/syscall.h>
#include <unistd.h>
#endif

physicslog_writer PhysicsLogs;

namespace {

// the most of all the logs kept while their files can't be written, shared by those, a log never less than the
// least: the oldest lines go beyond it
constexpr std::size_t keptwhilefailing{64 * 1024 * 1024};
constexpr std::size_t keptwhilefailingleast{64 * 1024};

// the text cut to its last lines within Most
void keep_last(std::string &Text, std::size_t const Most) {
	if (Text.size() <= Most)
		return;
	auto const cut{Text.find('\n', Text.size() - Most)};
	Text.erase(0, cut == std::string::npos ? Text.size() : cut + 1);
}

// the name of the file of the log Name, its extension too. what no file name may have on Windows is replaced, on every
// system, so the logs are named alike everywhere: a spawned vehicle is named <spawn>:<number>, and on NTFS "a:1.dat"
// isn't even an error but the stream 1.dat of the file a, which nothing lists
std::string file_name(std::string const &Name) {
	std::string name{Name};
	for (auto &character : name) {
		if (static_cast<unsigned char>(character) < 0x20 || std::string_view{R"(<>:"/\|?*)"}.find(character) != std::string_view::npos)
			character = '_';
	}
	// a device rather than a file, with any extension: con.dat is the console. told by the name before its first dot
	auto stem{name.substr(0, name.find('.'))};
	while (false == stem.empty() && stem.back() == ' ')
		stem.pop_back();
	std::ranges::transform(stem, stem.begin(), [](unsigned char const Character) { return static_cast<char>(std::toupper(Character)); });
	auto const numbered{stem.size() == 4 && (stem.starts_with("COM") || stem.starts_with("LPT")) && stem[3] >= '1' && stem[3] <= '9'};
	if (numbered || stem == "CON" || stem == "PRN" || stem == "AUX" || stem == "NUL")
		name.insert(0, 1, '_');
	return name + ".dat";
}

// told in the log of the simulator, if there's the memory for it
void tell(std::string const &Text) noexcept {
	try {
		ErrorLog(Text);
	} catch (...) {
		// out of memory for a line of the log too: nothing more to be done about it
	}
}

// the disk work of the calling thread after everyone else's: the simulation reading its textures and sounds, and the
// pages of its memory, from the same disk doesn't wait behind the logs, which on a hard disk are seconds of seeking
// when there are hundreds of them. what the thread does otherwise is the same
void lower_disk_priority() noexcept {
#ifdef _WIN32
	// a thread which can't have it writes as it did before
	static_cast<void>(::SetThreadPriority(::GetCurrentThread(), THREAD_MODE_BACKGROUND_BEGIN));
#elif defined(__linux__) && defined(SYS_ioprio_set)
	// the lowest of the best effort class rather than the idle one, which a disk always busy would never serve: the
	// lines would be kept in memory with no end. the constants of linux/ioprio.h, which not every system has
	constexpr int whothread{1}; // IOPRIO_WHO_PROCESS, of the calling thread with 0
	constexpr int classbesteffort{2};
	constexpr int classshift{13};
	constexpr int lowest{7};
	static_cast<void>(::syscall(SYS_ioprio_set, whothread, 0, (classbesteffort << classshift) | lowest));
#endif
}

} // namespace

void physicslog_values::append_to(std::string &Text) const {
	std::ostringstream line;
	line << std::fixed << std::setprecision(4) << time << " " << wheelspeed << " " << acceleration << " " << couplerdistance << " " << couplerforce << " "
	     << tractionforce << " " << frictionforce << " " << brakeforce << " " << brakepressure << " " << pipepressure << " " << motorcurrent << " "
	     << mainctrl << " " << scndctrl << " " << brakectrl << " " << localbrake << " " << direction << " " << command << " " << value1 << " "
	     << value2 << " " << security << " " << wheelslip << " " << enginetemperature << " " << oiltemperature << " " << watertemperature << " "
	     << wateraux << " " << vehiclespeed << "\r\n";
	Text.append(line.str());
}

void physicslog_writer::pending::to_text() {
	for (auto const &line : values)
		line.append_to(text);
	values.clear();
}

physicslog_writer::physicslog_writer(std::filesystem::path Folder, std::chrono::milliseconds const Interval) :
    m_folder{std::move(Folder)}, m_shownfolder{m_folder.generic_string()}, m_interval{Interval} {}

physicslog_writer::~physicslog_writer() {
	try {
		stop();
	} catch (...) {
		// at the end of the process: what the logs couldn't keep, memory or a file, isn't to end it worse
	}
}

void physicslog_writer::begin(std::string const &Name, std::string_view const Header) {
	std::scoped_lock lock{m_mutex};
	auto &lines{m_pending[Name]};
	lines.fresh = true;
	lines.header.assign(Header);
	lines.text.clear();
	lines.values.clear();
	start();
}

void physicslog_writer::append(std::string const &Name, std::string_view const Line) {
	std::scoped_lock lock{m_mutex};
	auto &lines{m_pending[Name]};
	// after the lines given as values before it
	lines.to_text();
	lines.text.append(Line);
	start();
}

void physicslog_writer::append(std::string const &Name, physicslog_values Values) {
	std::scoped_lock lock{m_mutex};
	m_pending[Name].values.push_back(std::move(Values));
	start();
}

void physicslog_writer::flush() {
	std::scoped_lock files{m_filemutex};
	write();
}

bool physicslog_writer::flush_for(std::chrono::milliseconds const Timeout) {
	// on a thread of its own, left behind if it doesn't end in time: a disk which doesn't answer holds it up in the
	// writing, not only the lock. for the end of the process, which doesn't wait for it
	auto done{std::make_shared<std::promise<void>>()};
	auto ended{done->get_future()};
	std::thread([this, done] {
		flush_safely();
		done->set_value();
	}).detach();
	return ended.wait_for(Timeout) == std::future_status::ready;
}

void physicslog_writer::stop() {
	std::jthread thread;
	bool itself;
	{
		std::scoped_lock lock{m_mutex};
		m_stopped = true;
		// taken by the first to stop, so it's joined once
		thread = std::move(m_thread);
		itself = m_writer == std::this_thread::get_id();
	}
	// the stop asked outside the lock, which waking the writer takes
	if (thread.joinable())
		thread.request_stop();
	// stopped from the writer itself, by the end of a run which failed in it: the files may be held by it
	if (itself) {
		if (thread.joinable())
			thread.detach();
		return;
	}
	if (thread.joinable())
		thread.join();
	flush_safely();
}

std::filesystem::path physicslog_writer::file(std::string const &Name) const {
	auto const name{file_name(Name)};
#ifdef _WIN32
	// in the code page of the system, as the standard library would convert it, but with the bytes not of it replaced
	// instead of an exception thrown: a vehicle of a scenery written in 1250 on a Windows set to UTF-8, say
	auto const size{static_cast<int>(name.size())};
	std::wstring wide(static_cast<std::size_t>(::MultiByteToWideChar(CP_ACP, 0, name.data(), size, nullptr, 0)), L'\0');
	::MultiByteToWideChar(CP_ACP, 0, name.data(), size, wide.data(), static_cast<int>(wide.size()));
	return m_folder / wide;
#else
	// the bytes are the name of the file as they are
	return m_folder / name;
#endif
}

std::string physicslog_writer::shown(std::string const &Name) const {
	return m_shownfolder + '/' + file_name(Name);
}

std::chrono::steady_clock::duration physicslog_writer::wait_after(std::chrono::milliseconds const Interval, std::chrono::steady_clock::duration const Took) {
	return std::max<std::chrono::steady_clock::duration>(Interval, Took * 3);
}

void physicslog_writer::start() {
	if (m_stopped || m_thread.joinable())
		return;
	m_thread = std::jthread([this](std::stop_token const &Stop) { run(Stop); });
	m_writer = m_thread.get_id();
}

void physicslog_writer::run(std::stop_token const &Stop) {
	lower_disk_priority();
	// woken by every stop asked, by stop() or by the end of the thread itself. the lock taken once first: the writer is
	// either about to wait, and sees the stop, or waiting already
	std::stop_callback const wake{Stop, [this] {
		{ std::scoped_lock lock{m_mutex}; }
		m_wake.notify_all();
	}};
	std::chrono::steady_clock::duration wait{m_interval};
	while (false == Stop.stop_requested()) {
		{
			std::unique_lock lock{m_mutex};
			// woken early only to stop, which writes the rest itself
			if (m_wake.wait_for(lock, wait, [&Stop] { return Stop.stop_requested(); }))
				return;
		}
		auto const start{std::chrono::steady_clock::now()};
		auto const logs{flush_safely()};
		auto const took{std::chrono::steady_clock::now() - start};
		wait = wait_after(m_interval, took);
		if (wait > m_interval && false == m_slowtold) {
			m_slowtold = true;
			using seconds = std::chrono::duration<double>;
			try {
				WriteLog(std::format("Physics logs: writing {} of them took {:.1f} s, over a third of the {:.1f} s between writings; they're "
				                     "written less often from now on, the disk busy with them a quarter of the time at most",
				    logs, seconds{took}.count(), seconds{m_interval}.count()));
			} catch (...) {
				// out of memory for a line of the log: the logs are written all the same
			}
		}
	}
}

std::size_t physicslog_writer::flush_safely() noexcept {
	try {
		std::scoped_lock files{m_filemutex};
		return write();
	} catch (std::exception const &Error) {
		tell(std::string{"Bad physics log: the writing failed ("} + Error.what() + "), the lines taken for it are lost");
	} catch (...) {
		tell("Bad physics log: the writing failed, the lines taken for it are lost");
	}
	return 0;
}

std::size_t physicslog_writer::write() {
	std::map<std::string, pending> taken;
	{
		std::scoped_lock lock{m_mutex};
		taken.swap(m_pending);
	}
	if (taken.empty())
		return 0;
	auto const logs{taken.size()};
	std::error_code error;
	std::filesystem::create_directories(m_folder, error);
	std::map<std::string, pending> failed;
	for (auto &[name, lines] : taken) {
		// the text of the lines given as values, here on the writer's thread rather than the simulation's
		lines.to_text();
		if (lines.text.empty() && false == lines.fresh)
			continue;
		// a log going wrong otherwise, out of memory say, is kept for the next try as one whose file can't be written:
		// the others are written still, and the writer goes on
		auto written{false};
		std::string problem{"can't write"};
		try {
			written = write_log(name, lines);
		} catch (std::exception const &Error) {
			problem = std::string{"failed ("} + Error.what() + ") to write";
		}
		if (written) {
			if (m_failed.erase(name) > 0)
				WriteLog("Physics log: " + shown(name) + " written again");
			continue;
		}
		if (m_failed.insert(name).second)
			ErrorLog("Bad physics log: " + problem + " " + shown(name) + ", its last lines are kept for the next try");
		failed.emplace(name, std::move(lines));
	}
	if (failed.empty())
		return logs;
	// cut before the lock, which the simulation waits on to give its lines: only what came since is added under it
	auto const most{std::max(keptwhilefailingleast, keptwhilefailing / failed.size())};
	for (auto &[name, lines] : failed)
		keep_last(lines.text, most);
	// tried again the next time, before what came since; a file held by another program for a while doesn't lose
	// them, nor its header. a log begun anew since drops them
	std::scoped_lock lock{m_mutex};
	for (auto &[name, lines] : failed) {
		auto &now{m_pending[name]};
		if (now.fresh)
			continue;
		lines.text.append(now.text);
		lines.values = std::move(now.values);
		now = std::move(lines);
	}
	return logs;
}

bool physicslog_writer::write_log(std::string const &Name, pending const &Lines) const {
	auto const path{file(Name)};
	// one write to the file: the header of a log begun with its lines, not the one after the other
	std::string begun;
	if (Lines.fresh)
		begun = Lines.header + Lines.text;
	std::string_view const text{Lines.fresh ? std::string_view{begun} : std::string_view{Lines.text}};
	// in text mode, as the logs were always written: on Windows their lines, which end with "\r\n", end with "\r\r\n".
	// a log added to is opened at its end (ate), and where that is, its length, is what it's cut back to if the lines
	// are written in part only, a disk filling up say: else they'd be written twice, the next time. the disk isn't
	// asked for the length before, one access less of hundreds of logs
	std::ofstream stream{path, Lines.fresh ? std::ios::out | std::ios::trunc : std::ios::out | std::ios::app | std::ios::ate};
	if (false == stream.is_open())
		return false;
	auto const before{Lines.fresh ? std::streamoff{0} : static_cast<std::streamoff>(stream.tellp())};
	stream.write(text.data(), static_cast<std::streamsize>(text.size()));
	stream.close();
	if (false == stream.fail())
		return true;
	// a log begun is written anew from its header the next time anyway
	if (false == Lines.fresh && before >= 0) {
		std::error_code error;
		std::filesystem::resize_file(path, static_cast<std::uintmax_t>(before), error);
	}
	return false;
}
