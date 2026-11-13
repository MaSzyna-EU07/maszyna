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

physicslog_writer::physicslog_writer(std::filesystem::path Folder, std::chrono::milliseconds const Interval) : m_folder{std::move(Folder)}, m_interval{Interval} {}

physicslog_writer::~physicslog_writer() {
	stop();
}

void physicslog_writer::begin(std::string const &Name, std::string_view const Header) {
	std::scoped_lock lock{m_mutex};
	auto &lines{m_pending[Name]};
	lines.fresh = true;
	lines.text.assign(Header);
	start();
}

void physicslog_writer::append(std::string const &Name, std::string_view const Line) {
	std::scoped_lock lock{m_mutex};
	m_pending[Name].text.append(Line);
	start();
}

void physicslog_writer::flush() {
	write();
}

void physicslog_writer::stop() {
	{
		std::scoped_lock lock{m_mutex};
		m_stopped = true;
	}
	// started only before m_stopped was set, so no one else touches it now
	if (m_thread.joinable()) {
		m_thread.request_stop();
		m_thread.join();
	}
	write();
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
		write();
	}
}

void physicslog_writer::write() {
	std::scoped_lock files{m_filemutex};
	std::map<std::string, pending> taken;
	{
		std::scoped_lock lock{m_mutex};
		taken.swap(m_pending);
	}
	if (taken.empty())
		return;
	std::error_code error;
	std::filesystem::create_directories(m_folder, error);
	for (auto const &[name, lines] : taken) {
		if (lines.text.empty() && false == lines.fresh)
			continue;
		auto const path{m_folder / (name + ".dat")};
		// in text mode, as the logs were always written: on Windows their lines, which end with "\r\n", end with "\r\r\n"
		std::ofstream file{path, lines.fresh ? std::ios::out | std::ios::trunc : std::ios::out | std::ios::app};
		if (file.is_open())
			file << lines.text;
		file.close();
		if (file.fail() && m_failed.insert(name).second)
			ErrorLog("Bad physics log: can't write " + path.string());
	}
}
