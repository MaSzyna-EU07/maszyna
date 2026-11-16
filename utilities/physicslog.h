/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <map>
#include <mutex>
#include <set>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>

// the physics logs, <folder>/<name>.dat. the lines given are kept, and written every two seconds by a thread of its
// own, each file opened only for the moment it is written to: the logs of hundreds of vehicles hold no files open, and
// the simulation doesn't wait for the disk. a crash loses the last two seconds of real time of them. lines whose file
// can't be written, held by another program say, are kept for the next try: the last 64 MB of them all at most
class physicslog_writer {
public:
	explicit physicslog_writer(std::filesystem::path Folder = "physicslog", std::chrono::milliseconds Interval = std::chrono::seconds{2});
	physicslog_writer(physicslog_writer const &) = delete;
	physicslog_writer &operator=(physicslog_writer const &) = delete;
	// stops, writing what is kept
	~physicslog_writer();

	// starts the log Name anew: its file is emptied and Header written into it, the next time the logs are written.
	// the lines of it not written yet are dropped
	void begin(std::string const &Name, std::string_view Header);
	// Line at the end of the log Name. a log not begun is added to as its file stands
	void append(std::string const &Name, std::string_view Line);
	// writes what is kept, now
	void flush();
	// as flush, waited for no longer than Timeout: then false, and the writing goes on by itself, on a thread left
	// behind. for the end of the process, the files held up or the disk not answering
	bool flush_for(std::chrono::milliseconds Timeout);
	// writes what is kept and ends the thread. what is given after is written by a flush, or at the end
	void stop();
	// the file of the log Name, made of its bytes as they are: those of the code page of a scenery (1250) too, on a
	// Windows set to UTF-8, where the standard library throws making a path of them. there they become U+FFFD
	std::filesystem::path file(std::string const &Name) const;

private:
	struct pending {
		bool fresh{false}; // its file is to be emptied first, and the header written into it
		std::string header;
		std::string text;
	};

	// the thread, on the first line given, unless stopped. called with m_mutex held
	void start();
	void run(std::stop_token const &Stop);
	// as flush, letting nothing out: on the writer's thread, or at the end of the process, an exception would end the
	// simulator. what was taken for a writing which failed as a whole is lost, the next one goes on with what came since
	void flush_safely() noexcept;
	// called with m_filemutex held
	void write();
	// the lines of the log Name written into its file. returns: true if they all were
	bool write_log(std::string const &Name, pending const &Lines) const;
	// the file of the log Name, for the log of the simulator
	std::string shown(std::string const &Name) const;

	std::filesystem::path const m_folder;
	std::string const m_shownfolder;
	std::chrono::milliseconds const m_interval;
	std::mutex m_mutex; // the lines kept and the thread
	std::map<std::string, pending> m_pending;
	bool m_stopped{false};
	std::condition_variable m_wake;
	std::jthread m_thread;
	std::thread::id m_writer; // of the thread, kept after it is taken to be joined: the writer knows itself by it
	std::mutex m_filemutex; // the files are written one batch at a time, so the lines of a log stay in order
	std::set<std::string> m_failed; // the logs whose file couldn't be written, told once
};

// the physics logs of the drivers (physicslog yes), and those the black box writes of the user's vehicles
extern physicslog_writer PhysicsLogs;
