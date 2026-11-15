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

} // namespace

physicslog_writer::physicslog_writer(std::filesystem::path Folder, std::chrono::milliseconds const Interval) : m_folder{std::move(Folder)}, m_interval{Interval} {}

physicslog_writer::~physicslog_writer() {
	stop();
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
		flush();
		done->set_value();
	}).detach();
	return ended.wait_for(Timeout) == std::future_status::ready;
}

void physicslog_writer::stop() {
	std::jthread thread;
	{
		std::scoped_lock lock{m_mutex};
		m_stopped = true;
		// stopped from the writer itself, by the end of a run which failed in it: the files may be held by it, and
		// another thread may be joining it already
		if (m_writer == std::this_thread::get_id()) {
			if (m_thread.joinable()) {
				m_thread.request_stop();
				m_thread.detach();
			}
			return;
		}
		// taken by the first to stop, so it's joined once
		thread = std::move(m_thread);
	}
	if (thread.joinable()) {
		thread.request_stop();
		thread.join();
	}
	flush();
}

void physicslog_writer::start() {
	if (m_stopped || m_thread.joinable())
		return;
	m_thread = std::jthread([this](std::stop_token const &Stop) { run(Stop); });
	m_writer = m_thread.get_id();
}

void physicslog_writer::run(std::stop_token const &Stop) {
	while (false == Stop.stop_requested()) {
		{
			std::unique_lock lock{m_mutex};
			// woken early only to stop, which writes the rest itself
			if (m_wake.wait_for(lock, Stop, m_interval, [] { return false; }) || Stop.stop_requested())
				return;
		}
		flush();
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
		auto const path{m_folder / (name + ".dat")};
		// how long it was, to be cut back to if the lines are written in part only, a disk filling up say: else
		// they'd be written twice, the next time
		auto const before{lines.fresh ? 0 : std::filesystem::file_size(path, error)};
		auto const existed{false == lines.fresh && false == static_cast<bool>(error)};
		// in text mode, as the logs were always written: on Windows their lines, which end with "\r\n", end with "\r\r\n"
		std::ofstream file{path, lines.fresh ? std::ios::out | std::ios::trunc : std::ios::out | std::ios::app};
		if (file.is_open()) {
			if (lines.fresh)
				file << lines.header;
			file << lines.text;
		}
		file.close();
		if (false == file.fail()) {
			if (m_failed.erase(name) > 0)
				WriteLog("Physics log: " + path.string() + " written again");
			continue;
		}
		if (existed)
			std::filesystem::resize_file(path, before, error);
		if (m_failed.insert(name).second)
			ErrorLog("Bad physics log: can't write " + path.string() + ", its last lines are kept for the next try");
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
