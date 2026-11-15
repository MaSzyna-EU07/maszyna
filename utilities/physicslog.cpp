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

#include <fstream>
#include <system_error>
#include <utility>

physicslog_writer PhysicsLogs;

namespace {

// the most of a log kept while its file can't be written; the oldest lines go beyond it
constexpr std::size_t keptwhilefailing{4 * 1024 * 1024};

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
	std::unique_lock files{m_filemutex, Timeout};
	if (false == files.owns_lock())
		return false;
	write();
	return true;
}

void physicslog_writer::stop() {
	std::jthread thread;
	{
		std::scoped_lock lock{m_mutex};
		m_stopped = true;
		// taken by the first to stop, so it's joined once
		thread = std::move(m_thread);
	}
	if (thread.joinable()) {
		if (thread.get_id() == std::this_thread::get_id()) {
			// stopped from the thread itself, by the end of a run which failed in it: the files may be held by it
			thread.detach();
			return;
		}
		thread.request_stop();
		thread.join();
	}
	flush();
}

void physicslog_writer::start() {
	if (m_stopped || m_thread.joinable())
		return;
	m_thread = std::jthread([this](std::stop_token const &Stop) { run(Stop); });
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
		if (m_failed.insert(name).second)
			ErrorLog("Bad physics log: can't write " + path.string() + ", its lines are kept for the next try");
		failed.emplace(name, std::move(lines));
	}
	if (failed.empty())
		return;
	// tried again the next time, before what came since; a file held by another program for a while doesn't lose
	// them, nor its header. a log begun anew since drops them
	std::scoped_lock lock{m_mutex};
	for (auto &[name, lines] : failed) {
		auto &now{m_pending[name]};
		if (now.fresh)
			continue;
		lines.text.append(now.text);
		if (lines.text.size() > keptwhilefailing) {
			auto const cut{lines.text.find('\n', lines.text.size() - keptwhilefailing)};
			lines.text.erase(0, cut == std::string::npos ? lines.text.size() : cut + 1);
		}
		now = std::move(lines);
	}
}
