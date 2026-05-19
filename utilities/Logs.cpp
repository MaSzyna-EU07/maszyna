/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/Logs.h"

#include "utilities/Globals.h"
#include "winheaders.h"
#include "utilities/utilities.h"
#include "application/uilayer.h"
#include <deque>

std::ofstream output; // standardowy "log.txt", można go wyłączyć
std::ofstream errors; // lista błędów "errors.txt", zawsze działa
std::ofstream comms; // lista komunikatow "comms.txt", można go wyłączyć
char logbuffer[ 256 ];

char endstring[10] = "\n";

std::deque<std::string> log_scrollback;

namespace
{
std::string sanitize_filename_component(std::string value)
{
	for (auto &character : value)
	{
		switch (character)
		{
		case '\\':
		case '/':
		case ':':
		case '*':
		case '?':
		case '\"':
		case '<':
		case '>':
		case '|':
			character = '_';
			break;
		default:
			break;
		}
	}
	return value;
}

std::string filename_session_timestamp()
{
	::SYSTEMTIME st {};

#ifdef __unix__
	timespec ts {};
	clock_gettime(CLOCK_REALTIME, &ts);
	tm *tms = localtime(&ts.tv_sec);
	if (tms != nullptr)
	{
		st.wYear = static_cast<unsigned short>(tms->tm_year + 1900);
		st.wMonth = static_cast<unsigned short>(tms->tm_mon + 1);
		st.wDay = static_cast<unsigned short>(tms->tm_mday);
		st.wHour = static_cast<unsigned short>(tms->tm_hour);
		st.wMinute = static_cast<unsigned short>(tms->tm_min);
		st.wSecond = static_cast<unsigned short>(tms->tm_sec);
		st.wMilliseconds = static_cast<unsigned short>(ts.tv_nsec / 1000000);
	}
#elif _WIN32
	::GetLocalTime(&st);
#endif

	std::snprintf(
	    logbuffer,
	    sizeof(logbuffer),
	    "%04u%02u%02u_%02u%02u%02u_%03u",
	    static_cast<unsigned>(st.wYear),
	    static_cast<unsigned>(st.wMonth),
	    static_cast<unsigned>(st.wDay),
	    static_cast<unsigned>(st.wHour),
	    static_cast<unsigned>(st.wMinute),
	    static_cast<unsigned>(st.wSecond),
	    static_cast<unsigned>(st.wMilliseconds));

	return std::string(logbuffer);
}

std::string const &log_session_id()
{
	static std::string const session_id = []()
	{
		std::string session = filename_session_timestamp();
#ifdef _WIN32
		char pid_suffix[32];
		std::snprintf(pid_suffix, sizeof(pid_suffix), "_%lu", static_cast<unsigned long>(::GetCurrentProcessId()));
		session += pid_suffix;
#endif
		return session;
	}();
	return session_id;
}

std::string filename_scenery()
{
	auto extension = Global.SceneryFile.rfind('.');
	if (extension != std::string::npos)
	{
		return sanitize_filename_component(Global.SceneryFile.substr(0, extension));
	}

	return sanitize_filename_component(Global.SceneryFile);
}

std::string log_output_filename()
{
	if (Global.MultipleLogs && false == Global.SceneryFile.empty())
	{
		return "logs/log (" + filename_scenery() + ") " + log_session_id() + ".txt";
	}

	return "logs/log_" + log_session_id() + ".txt";
}

std::string log_errors_filename()
{
	if (Global.MultipleLogs && false == Global.SceneryFile.empty())
	{
		return "logs/errors (" + filename_scenery() + ") " + log_session_id() + ".txt";
	}

	return "logs/errors_" + log_session_id() + ".txt";
}
} // namespace

// log service stacks
std::deque < std::pair<std::string, bool>> InfoStack;
std::deque<std::string> ErrorStack;

// lock for log stacks
std::mutex logMutex;


void LogService()
{
	// prevent crash if mutex is not initialized
	while (true)
	{
		try
		{
			logMutex.lock();
			break;
		}
		catch (...) {}
	}
	logMutex.unlock();

	while (!Global.applicationQuitOrder)
	{
		{
			// --- Obsługa InfoStack ---
			while (!InfoStack.empty())
			{
				logMutex.lock();
				std::string msg = InfoStack.front().first;
				bool isError = InfoStack.front().second;
				InfoStack.pop_front();
				logMutex.unlock();

				// log to file
				if (Global.iWriteLogEnabled & 1)
				{
					if (!output.is_open())
					{
						output.open(log_output_filename(), std::ios::trunc);
					}
					output << msg << "\n";
					output.flush();
				}

				// log to scrollback imgui
				log_scrollback.emplace_back(msg);
				if (log_scrollback.size() > 200)
					log_scrollback.pop_front();

				// log to console
				if (Global.iWriteLogEnabled & 2)
				{
					if (isError)
						printf("\033[1;37;41m%s\033[0m\n", msg.c_str());
					else
						printf("\033[32m%s\033[0m\n", msg.c_str());
				}
			}

			// --- Obsługa ErrorStack ---
			while (!ErrorStack.empty())
			{
				logMutex.lock();
				std::string msg = ErrorStack.front();
				ErrorStack.pop_front();
				logMutex.unlock();

				if (!(Global.iWriteLogEnabled & 1))
					continue;

				if (!errors.is_open())
				{
					errors.open(log_errors_filename(), std::ios::trunc);
					errors << "EU07.EXE " + Global.asVersion << "\n";
				}

				errors << msg << "\n";
				errors.flush();
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
}


bool ShouldSkipLog(std::string_view str, logtype type)
{
	return str.empty() ||
		   TestFlag(Global.DisabledLogTypes, static_cast<unsigned int>(type));
}

std::string FormatLogMessage(std::string_view str)
{
	const auto now = std::chrono::steady_clock::now();
	const auto elapsed = now - Global.startTimestamp;
	const double seconds = std::chrono::duration<double>(elapsed).count();

	return std::format("[ {:8.3f} ]\t\t{}", seconds, str);
}

void WriteLog(std::string_view str, logtype type, bool isError)
{
	if (ShouldSkipLog(str, type))
		return;

	const auto message = FormatLogMessage(str);

	std::lock_guard<std::mutex> lock(logMutex);
	InfoStack.push_back({message, isError});
}

void ErrorLog(std::string_view str, logtype type)
{
	if (ShouldSkipLog(str, type))
		return;

	const auto message = FormatLogMessage(str);

	std::lock_guard<std::mutex> lock(logMutex);
	ErrorStack.push_back(message);
}

void WriteLog(const char* str, logtype type, bool isError)
{
	if (str == nullptr || *str == '\0')
		return;

	WriteLog(std::string_view{str}, type, isError);
}

void ErrorLog(const char* str, logtype type)
{
	if (str == nullptr || *str == '\0')
		return;

	ErrorLog(std::string_view{str}, type);
}


void Error(const std::string &asMessage, bool box)
{
    // if (box)
    //	MessageBox(NULL, asMessage.c_str(), string("EU07 " + Global.asRelease).c_str(), MB_OK);
    ErrorLog(asMessage.c_str());
}

void Error(const char *&asMessage, bool box)
{
    // if (box)
    //	MessageBox(NULL, asMessage, string("EU07 " + Global.asRelease).c_str(), MB_OK);
    ErrorLog(asMessage);
    WriteLog(asMessage);
}

void ErrorLog(const std::string &str, logtype const Type )
{
    ErrorLog( str.c_str(), Type );
    WriteLog( str.c_str(), Type, true );
}

void WriteLog(const std::string &str, logtype const Type )
{ // Ra: wersja z AnsiString jest zamienna z Error()
    WriteLog( str.c_str(), Type );
};

void CommLog(const char *str)
{ // Ra: warunkowa rejestracja komunikatów
    WriteLog(str);
    /*    if (Global.iWriteLogEnabled & 4)
    {
    if (!comms.is_open())
    {
    comms.open("comms.txt", std::ios::trunc);
    comms << AnsiString("EU07.EXE " + Global.asRelease).c_str() << "\n";
    }
    if (str)
    comms << str;
    comms << "\n";
    comms.flush();
    }*/
};

void CommLog(const std::string &str)
{ // Ra: wersja z AnsiString jest zamienna z Error()
    WriteLog(str);
};

//---------------------------------------------------------------------------
