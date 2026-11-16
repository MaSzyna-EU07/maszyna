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
#include <exception>
#include <fstream>
#include <future>
#include <memory>
#include <system_error>
#include <utility>

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

// told in the log of the simulator, if there's the memory for it
void tell(std::string const &Text) noexcept {
	try {
		ErrorLog(Text);
	} catch (...) {
		// out of memory for a line of the log too: nothing more to be done about it
	}
}

} // namespace

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
	start();
}

void physicslog_writer::append(std::string const &Name, std::string_view const Line) {
	std::scoped_lock lock{m_mutex};
	m_pending[Name].text.append(Line);
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
	auto const name{Name + ".dat"};
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
	return m_shownfolder + '/' + Name + ".dat";
}

void physicslog_writer::start() {
	if (m_stopped || m_thread.joinable())
		return;
	m_thread = std::jthread([this](std::stop_token const &Stop) { run(Stop); });
	m_writer = m_thread.get_id();
}

void physicslog_writer::run(std::stop_token const &Stop) {
	// woken by every stop asked, by stop() or by the end of the thread itself. the lock taken once first: the writer is
	// either about to wait, and sees the stop, or waiting already
	std::stop_callback const wake{Stop, [this] {
		{ std::scoped_lock lock{m_mutex}; }
		m_wake.notify_all();
	}};
	while (false == Stop.stop_requested()) {
		{
			std::unique_lock lock{m_mutex};
			// woken early only to stop, which writes the rest itself
			if (m_wake.wait_for(lock, m_interval, [&Stop] { return Stop.stop_requested(); }))
				return;
		}
		flush_safely();
	}
}

void physicslog_writer::flush_safely() noexcept {
	try {
		flush();
	} catch (std::exception const &Error) {
		tell(std::string{"Bad physics log: the writing failed ("} + Error.what() + "), the lines taken for it are lost");
	} catch (...) {
		tell("Bad physics log: the writing failed, the lines taken for it are lost");
	}
}

void physicslog_writer::write() {
	std::map<std::string, pending> taken;
	{
		std::scoped_lock lock{m_mutex};
		taken.swap(m_pending);
	}
	if (taken.empty())
		return;
	std::error_code error;
	std::filesystem::create_directories(m_folder, error);
	std::map<std::string, pending> failed;
	for (auto &[name, lines] : taken) {
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
		return;
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
		now = std::move(lines);
	}
}

bool physicslog_writer::write_log(std::string const &Name, pending const &Lines) const {
	auto const path{file(Name)};
	std::error_code error;
	// how long it was, to be cut back to if the lines are written in part only, a disk filling up say: else they'd be
	// written twice, the next time
	auto const before{Lines.fresh ? 0 : std::filesystem::file_size(path, error)};
	auto const existed{false == Lines.fresh && false == static_cast<bool>(error)};
	// in text mode, as the logs were always written: on Windows their lines, which end with "\r\n", end with "\r\r\n"
	std::ofstream stream{path, Lines.fresh ? std::ios::out | std::ios::trunc : std::ios::out | std::ios::app};
	if (stream.is_open()) {
		if (Lines.fresh)
			stream << Lines.header;
		stream << Lines.text;
	}
	stream.close();
	if (false == stream.fail())
		return true;
	if (existed)
		std::filesystem::resize_file(path, before, error);
	return false;
}
