/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <map>
#include <mutex>
#include <set>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

// a line of a driver's physics log as its values, taken as the simulation goes and made into text by the thread which
// writes the logs: the text, a stream of 26 values, is what a line costs, and the drivers of a big scenery give twenty
// lines a second each. the columns as physicslog_header of the driver names them
struct physicslog_values {
	double time{0.0};
	double wheelspeed{0.0};
	double acceleration{0.0};
	double couplerdistance{0.0};
	double couplerforce{0.0};
	double tractionforce{0.0};
	double frictionforce{0.0};
	double brakeforce{0.0};
	double brakepressure{0.0};
	double pipepressure{0.0};
	double motorcurrent{0.0};
	int mainctrl{0};
	int scndctrl{0};
	double brakectrl{0.0};
	double localbrake{0.0};
	int direction{0};
	std::string command;
	double value1{0.0};
	double value2{0.0};
	int security{0};
	int wheelslip{0};
	double enginetemperature{0.0};
	double oiltemperature{0.0};
	double watertemperature{0.0};
	double wateraux{0.0};
	double vehiclespeed{0.0};

	// the line, as the logs always had it, at the end of Text
	void append_to(std::string &Text) const;
};

// the physics logs, <folder>/<name>.dat, what no file name may have on Windows replaced in the name (a spawned vehicle's
// has a ':'), the same on every system. the lines given are kept, and written every two seconds by a thread of its own,
// each file opened only for the moment it is written to: the logs of hundreds of vehicles hold no files open, and the
// simulation doesn't wait for the disk. a crash loses the last two seconds of real time of them, up to ten on a disk
// which can't keep up (below). lines whose file can't be written, held by another program say, are kept for the next
// try: the last 64 MB of them all at most. the thread's disk work goes after everyone else's, and a disk slower than the
// logs need (a hard disk, hundreds of them) gets them less often, so the simulation reading from it doesn't wait behind
// them. the closing waits for the writing a second at most (below)
class physicslog_writer {
public:
	explicit physicslog_writer(std::filesystem::path Folder = "physicslog", std::chrono::milliseconds Interval = std::chrono::seconds{2},
	    std::chrono::milliseconds ClosingWait = std::chrono::seconds{1});
	physicslog_writer(physicslog_writer const &) = delete;
	physicslog_writer &operator=(physicslog_writer const &) = delete;
	// stops, writing what is kept, and waits for the writings a stop left behind: the object goes. the simulator's,
	// global, never goes this way: the process ends with _Exit
	~physicslog_writer();

	// starts the log Name anew: its file is emptied and Header written into it, the next time the logs are written.
	// the lines of it not written yet are dropped
	void begin(std::string const &Name, std::string_view Header);
	// Line at the end of the log Name. a log not begun is added to as its file stands
	void append(std::string const &Name, std::string_view Line);
	// the same, a line of the values, made into text by the writer's thread when it writes the log
	void append(std::string const &Name, physicslog_values Values);
	// the log Name written before the others, at each writing: the logs of the vehicles the user was in, and of the
	// tested trains, which a closing short of time would otherwise drop by the order of their names
	void put_first(std::string const &Name);
	// writes what is kept, now
	void flush();
	// as flush, waited for no longer than Timeout: then false, and the writing goes on by itself, on a thread left
	// behind. for the end of the process, the files held up or the disk not answering
	bool flush_for(std::chrono::milliseconds Timeout);
	// writes what is kept and ends the thread, waiting for it ClosingWait at most: the logs whose turn hasn't come by
	// then are dropped, and the writing of the one at hand (a disk not answering) is left behind, going on by itself. a
	// writing of the thread under way ends at the log at hand, leaving the others to the closing's, which has them with
	// the lines given since, those put first first. what is given after is written by a flush, or by the next stop
	void stop();
	// the file of the log Name: <folder>/<name>.dat, with < > : " / \ | ? * and the control characters of the name
	// replaced by _, and _ before a name Windows keeps for a device (con, nul, com1...). the bytes of the code page of a
	// scenery (1250) stay as they are, also on a Windows set to UTF-8, where the standard library would throw making a
	// path of them: there they become U+FFFD
	std::filesystem::path file(std::string const &Name) const;
	// the same, for the log of the simulator
	std::string shown(std::string const &Name) const;
	// the wait of the thread before its next writing, after one which took Took: the interval, or three times as long
	// as the writing was when that's longer, five intervals at most. a disk which can't keep up is then busy with the
	// logs a quarter of the time rather than all of it, and their lines wait in memory the longer
	static std::chrono::steady_clock::duration wait_after(std::chrono::milliseconds Interval, std::chrono::steady_clock::duration Took);
	// the logs taken for a writing, pointers to the entries of a map by their names, put in the order they're written:
	// those put first (First) before the others, each in the order they were
	template <typename Log> static void writing_order(std::vector<Log *> &Logs, std::set<std::string> const &First) {
		std::ranges::stable_partition(Logs, [&First](Log const *Each) { return First.contains(Each->first); });
	}

private:
	// a writing: a flush; the thread's, which a stop ends at the log at hand, leaving the others to the closing's; or the
	// closing's, which drops the logs past the time the stop gives it
	enum class writing { flush, thread, closing };

	// the end of the time a stop gives the writing. fresh: given by this stop, not by one before it still under way
	struct closing_time {
		std::chrono::steady_clock::time_point by;
		bool fresh{true};
	};

	struct pending {
		bool fresh{false}; // its file is to be emptied first, and the header written into it
		std::string header;
		std::string text;
		// the lines given as values since, after the text: made into text when written
		std::vector<physicslog_values> values;

		// the lines of the values at the end of the text
		void to_text();
	};

	// the time of a stop: the one of a closing still under way, if there's one. called with m_mutex held
	closing_time closing_begun();
	// the writer ended and the rest written by a thread of its own, waited for until Closeby. returns: false if no
	// thread could be had for it, out of memory say; Thread may be the writer still then
	bool close_aside(std::jthread &Thread, closing_time Closeby) noexcept;
	// the thread, on the first line given, unless stopped. called with m_mutex held
	void start();
	void run(std::stop_token const &Stop);
	// as flush, letting nothing out: on the writer's thread, or at the end of the process, an exception would end the
	// simulator. what was taken for a writing which failed as a whole is lost, the next one goes on with what came since.
	// returns: the number of logs there were to write
	std::size_t flush_safely(writing Mode) noexcept;
	// called with m_filemutex held. returns: the number of logs there were to write
	std::size_t write(writing Mode);
	// the log Name taken for a writing written into its file, the problem told once if it can't be. returns: false if
	// it wasn't, its lines to be kept for the next try
	bool write_taken(std::string const &Name, pending &Lines);
	// whether the time a stop gave the writing is over
	bool past_closing() const;
	// the logs taken for a writing and not written kept for the next, before what came since, their values too: Failed,
	// whose file couldn't be written, cut to their last lines, and Left, which a stop left to the closing's writing, as
	// they were taken: their values are made into text by that writing, which drops them past its time, not on the way
	// unless text was given since
	void keep(std::map<std::string, pending> &Failed, std::vector<std::pair<std::string const, pending> *> const &Left);
	// Lines of the log Name, taken for a writing, put back before the lines given since. called with m_mutex held
	void put_back(std::string const &Name, pending &Lines);
	// the lines of the log Name written into its file. returns: true if they all were
	bool write_log(std::string const &Name, pending const &Lines) const;

	std::filesystem::path const m_folder;
	std::string const m_shownfolder;
	std::chrono::milliseconds const m_interval;
	std::chrono::milliseconds const m_closingwait;
	// the end of the time the last stop gave the writing, of the steady clock; 0 before the first stop
	std::atomic<std::chrono::steady_clock::rep> m_closeby{0};
	std::mutex m_mutex; // the lines kept and the thread
	std::map<std::string, pending> m_pending;
	std::set<std::string> m_first; // the logs put first
	bool m_stopped{false};
	std::condition_variable m_wake;
	std::jthread m_thread;
	std::thread::id m_writer; // of the thread, kept after it is taken to be joined: the writer knows itself by it
	std::vector<std::shared_future<void>> m_closings; // the writings of the stops, which may outlast them
	std::mutex m_filemutex; // the files are written one batch at a time, so the lines of a log stay in order
	std::set<std::string> m_failed; // the logs whose file couldn't be written, told once
	bool m_slowtold{false}; // the writings were told to take longer than the interval allows, once; the thread's own
};

// the physics logs of the drivers (physicslog yes), and those the black box writes of the user's vehicles
extern physicslog_writer PhysicsLogs;
