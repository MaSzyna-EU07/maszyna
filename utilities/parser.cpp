/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/parser.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

#include "scene/scenenodegroups.h"

/*
    MaSzyna EU07 locomotive simulator parser
    Copyright (C) 2003  TOLARIS

*/

/////////////////////////////////////////////////////////////////////////////////////////////////////
// cParser -- generic class for parsing text data.

namespace
{
inline std::array<bool, 256> makeBreakTable(const char *brk)
{
	std::array<bool, 256> arr{};
	for (unsigned char c : std::string_view(brk ? brk : ""))
	{
		arr[c] = true;
	}
	return arr;
}

inline char toLowerChar(char c)
{
	return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
}

inline bool startsWithBOM(const std::string &s)
{
	return s.size() >= 3
		&& static_cast<unsigned char>(s[0]) == 0xEF
		&& static_cast<unsigned char>(s[1]) == 0xBB
		&& static_cast<unsigned char>(s[2]) == 0xBF;
}

std::atomic<unsigned int> g_parser_file_cache_depth { 0 };
bool g_parser_file_cache_enabled { false };
std::mutex g_parser_file_cache_mutex;
std::unordered_map<std::string, std::shared_ptr<std::string const>> g_parser_file_cache;

struct parser_file_cache_metrics
{
	std::uint64_t open_attempts { 0 };
	std::uint64_t disk_reads { 0 };
	std::uint64_t cache_hits { 0 };
	std::uint64_t cache_misses { 0 };
	std::uint64_t failed_opens { 0 };
	std::uint64_t bytes_read { 0 };
	std::uint64_t untracked_path_calls { 0 };
	std::uint64_t zero_copy_opens { 0 };
	std::chrono::steady_clock::duration open_read_time {};
	std::unordered_map<std::string, std::uint64_t> path_counts;

	void reset()
	{
		*this = {};
	}
};

parser_file_cache_metrics g_parser_file_cache_metrics;

class parser_memory_streambuf final : public std::streambuf
{
public:
	explicit parser_memory_streambuf(std::shared_ptr<std::string const> data)
	    : data_(std::move(data))
	{
	}

protected:
	int underflow() override
	{
		if (!data_ || pos_ >= data_->size())
		{
			return traits_type::eof();
		}
		return static_cast<unsigned char>((*data_)[pos_]);
	}

	int uflow() override
	{
		if (!data_ || pos_ >= data_->size())
		{
			return traits_type::eof();
		}
		return static_cast<unsigned char>((*data_)[pos_++]);
	}

	std::streampos seekoff(std::streamoff off, std::ios_base::seekdir dir, std::ios_base::openmode which) override
	{
		if (!data_ || ((which & std::ios_base::in) == 0))
		{
			return std::streampos(std::streamoff(-1));
		}

		auto const size = static_cast<std::streamoff>(data_->size());
		std::streamoff newpos = 0;
		switch (dir)
		{
		case std::ios_base::beg:
			newpos = off;
			break;
		case std::ios_base::cur:
			newpos = static_cast<std::streamoff>(pos_) + off;
			break;
		case std::ios_base::end:
			newpos = size + off;
			break;
		default:
			return std::streampos(std::streamoff(-1));
		}

		if (newpos < 0)
		{
			newpos = 0;
		}
		if (newpos > size)
		{
			newpos = size;
		}
		pos_ = static_cast<std::size_t>(newpos);
		return std::streampos(newpos);
	}

	std::streampos seekpos(std::streampos pos, std::ios_base::openmode which) override
	{
		return seekoff(std::streamoff(pos), std::ios_base::beg, which);
	}

private:
	std::shared_ptr<std::string const> data_;
	std::size_t pos_ { 0 };
};

class parser_memory_istream final : public std::istream
{
public:
	explicit parser_memory_istream(std::shared_ptr<std::string const> data)
	    : std::istream(nullptr)
	    , buffer_(std::move(data))
	{
		rdbuf(&buffer_);
	}

private:
	parser_memory_streambuf buffer_;
};

std::shared_ptr<std::istream> ParserFileCacheMakeMemoryStream(std::shared_ptr<std::string const> const &Content)
{
	return std::make_shared<parser_memory_istream>(Content);
}

std::string ParserFileCacheKey(std::string const &Filename)
{
	if (Filename.empty())
	{
		return Filename;
	}

	try
	{
		auto path = std::filesystem::path(Filename);
		if (path.is_relative())
		{
			path = std::filesystem::absolute(path);
		}

		auto key = path.lexically_normal().generic_string();
#ifdef _WIN32
		for (auto &character : key)
		{
			auto const value = static_cast<unsigned char>(character);
			if (value < 128)
			{
				character = static_cast<char>(std::tolower(value));
			}
		}
#endif
		return key;
	}
	catch (std::filesystem::filesystem_error const &)
	{
		auto key = Filename;
		std::replace(key.begin(), key.end(), '\\', '/');
		return key;
	}
}

double ParserFileCacheDurationMilliseconds(std::chrono::steady_clock::duration const Duration)
{
	return std::chrono::duration<double, std::milli>(Duration).count();
}

std::string ParserFileCacheFormatMilliseconds(std::chrono::steady_clock::duration const Duration)
{
	std::ostringstream output;
	output << std::fixed << std::setprecision(3) << ParserFileCacheDurationMilliseconds(Duration);
	return output.str();
}

void ParserFileCacheTrackPath(std::string const &Key)
{
	if (auto lookup = g_parser_file_cache_metrics.path_counts.find(Key); lookup != g_parser_file_cache_metrics.path_counts.end())
	{
		++lookup->second;
	}
	else if (g_parser_file_cache_metrics.path_counts.size() < 4096)
	{
		g_parser_file_cache_metrics.path_counts.emplace(Key, 1);
	}
	else
	{
		++g_parser_file_cache_metrics.untracked_path_calls;
	}
}

std::vector<std::string> ParserFileCacheReport()
{
	std::vector<std::string> report;
	report.emplace_back(
	    std::string("Parser file scope: mode=") + (g_parser_file_cache_enabled ? "cached" : "direct")
	    + ", opens=" + std::to_string(g_parser_file_cache_metrics.open_attempts)
	    + ", disk_reads=" + std::to_string(g_parser_file_cache_metrics.disk_reads)
	    + ", hits=" + std::to_string(g_parser_file_cache_metrics.cache_hits)
	    + ", misses=" + std::to_string(g_parser_file_cache_metrics.cache_misses)
	    + ", failed=" + std::to_string(g_parser_file_cache_metrics.failed_opens)
	    + ", bytes_read=" + std::to_string(g_parser_file_cache_metrics.bytes_read)
	    + ", open_read_ms=" + ParserFileCacheFormatMilliseconds(g_parser_file_cache_metrics.open_read_time)
	    + ", zero_copy_opens=" + std::to_string(g_parser_file_cache_metrics.zero_copy_opens)
	    + ", untracked_paths=" + std::to_string(g_parser_file_cache_metrics.untracked_path_calls)
	    + ", cached_files=" + std::to_string(g_parser_file_cache.size()));

	std::vector<std::pair<std::string, std::uint64_t>> top_paths;
	top_paths.reserve(g_parser_file_cache_metrics.path_counts.size());
	for (auto const &entry : g_parser_file_cache_metrics.path_counts)
	{
		if (entry.second > 1)
		{
			top_paths.emplace_back(entry);
		}
	}

	constexpr std::size_t top_path_limit = 5;
	auto const top_count = std::min(top_path_limit, top_paths.size());
	std::partial_sort(
	    top_paths.begin(),
	    top_paths.begin() + top_count,
	    top_paths.end(),
	    [](auto const &Left, auto const &Right)
	    {
		    if (Left.second != Right.second)
		    {
			    return Left.second > Right.second;
		    }
		    return Left.first < Right.first;
	    });

	for (std::size_t index = 0; index < top_count; ++index)
	{
		report.emplace_back("  " + std::to_string(top_paths[index].second) + "x " + top_paths[index].first);
	}

	return report;
}

void ParserFileCacheLogReport(std::vector<std::string> const &Report)
{
	for (auto const &line : Report)
	{
		WriteLog(line);
	}
}

std::shared_ptr<std::istream> ParserFileCacheOpenDirect(std::string const &Path, std::string const &Key)
{
	{
		std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
		++g_parser_file_cache_metrics.open_attempts;
		ParserFileCacheTrackPath(Key);
	}

	auto const timestart = std::chrono::steady_clock::now();
	auto stream = std::make_shared<std::ifstream>(Path, std::ios_base::binary);
	if (stream->fail())
	{
		std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
		++g_parser_file_cache_metrics.failed_opens;
		g_parser_file_cache_metrics.open_read_time += std::chrono::steady_clock::now() - timestart;
		return stream;
	}

	std::error_code error;
	auto const size = std::filesystem::file_size(Path, error);
	{
		std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
		++g_parser_file_cache_metrics.disk_reads;
		if (!error)
		{
			g_parser_file_cache_metrics.bytes_read += size;
		}
		g_parser_file_cache_metrics.open_read_time += std::chrono::steady_clock::now() - timestart;
	}

	return stream;
}

std::shared_ptr<std::istream> ParserFileCacheOpenCached(std::string const &Path, std::string const &Key)
{
	{
		std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
		++g_parser_file_cache_metrics.open_attempts;
		ParserFileCacheTrackPath(Key);
		if (auto lookup = g_parser_file_cache.find(Key); lookup != g_parser_file_cache.end())
		{
			++g_parser_file_cache_metrics.cache_hits;
			++g_parser_file_cache_metrics.zero_copy_opens;
			return ParserFileCacheMakeMemoryStream(lookup->second);
		}
		++g_parser_file_cache_metrics.cache_misses;
	}

	auto const timestart = std::chrono::steady_clock::now();
	auto file = std::ifstream(Path, std::ios_base::binary);
	if (file.fail())
	{
		std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
		++g_parser_file_cache_metrics.failed_opens;
		g_parser_file_cache_metrics.open_read_time += std::chrono::steady_clock::now() - timestart;
		return std::make_shared<std::ifstream>(std::move(file));
	}

	file.seekg(0, std::ios_base::end);
	auto const size = file.tellg();
	file.seekg(0, std::ios_base::beg);
	if (size < 0)
	{
		{
			std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
			++g_parser_file_cache_metrics.failed_opens;
			g_parser_file_cache_metrics.open_read_time += std::chrono::steady_clock::now() - timestart;
		}
		WriteLog("Parser file cache: falling back to direct read for \"" + Path + "\"");
		return std::make_shared<std::ifstream>(Path, std::ios_base::binary);
	}

	auto content = std::make_shared<std::string>();
	if (size > 0)
	{
		content->resize(static_cast<std::size_t>(size));
		file.read(content->data(), static_cast<std::streamsize>(size));
	}
	if (file.bad() || ((size > 0) && (file.gcount() != static_cast<std::streamsize>(size))))
	{
		{
			std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
			++g_parser_file_cache_metrics.failed_opens;
			g_parser_file_cache_metrics.open_read_time += std::chrono::steady_clock::now() - timestart;
		}
		WriteLog("Parser file cache: falling back to direct read for \"" + Path + "\"");
		return std::make_shared<std::ifstream>(Path, std::ios_base::binary);
	}

	{
		std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
		++g_parser_file_cache_metrics.disk_reads;
		g_parser_file_cache_metrics.bytes_read += static_cast<std::uint64_t>(content->size());
		g_parser_file_cache_metrics.open_read_time += std::chrono::steady_clock::now() - timestart;
		g_parser_file_cache.emplace(Key, content);
	}

	return ParserFileCacheMakeMemoryStream(content);
}

std::shared_ptr<std::istream> ParserFileCacheOpen(std::string const &Path)
{
	if (g_parser_file_cache_depth.load(std::memory_order_acquire) == 0)
	{
		return std::make_shared<std::ifstream>(Path, std::ios_base::binary);
	}

	bool cache_enabled = false;
	std::string key;
	{
		std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
		if (g_parser_file_cache_depth.load(std::memory_order_relaxed) == 0)
		{
			return std::make_shared<std::ifstream>(Path, std::ios_base::binary);
		}
		cache_enabled = g_parser_file_cache_enabled;
		key = cache_enabled ? ParserFileCacheKey(Path) : Path;
	}

	if (cache_enabled)
	{
		return ParserFileCacheOpenCached(Path, key);
	}

	return ParserFileCacheOpenDirect(Path, key);
}

std::atomic<unsigned int> g_parser_metrics_depth { 0 };

struct parser_cpu_metrics
{
	std::atomic<std::uint64_t> read_token_calls { 0 };
	std::atomic<std::uint64_t> tokens_read { 0 };
	std::atomic<std::uint64_t> get_tokens_calls { 0 };
	std::atomic<std::uint64_t> includes_opened { 0 };
	std::atomic<std::uint64_t> parsers_file_buffer { 0 };
	std::atomic<std::uint64_t> parsers_text_buffer { 0 };
	std::atomic<std::uint64_t> tokenize_ns { 0 };
	std::atomic<std::uint64_t> read_token_ns { 0 };
	std::atomic<std::uint64_t> convert_ns { 0 };
	std::atomic<std::uint64_t> get_tokens_ns { 0 };
	std::atomic<std::uint64_t> include_ns { 0 };
	std::atomic<std::uint64_t> fp_parse_calls { 0 };
	std::atomic<std::uint64_t> skip_until_calls { 0 };
	std::atomic<std::uint64_t> fast_skip_tokens { 0 };
	std::atomic<std::uint64_t> skip_until_ns { 0 };

	void reset()
	{
		read_token_calls.store(0, std::memory_order_relaxed);
		tokens_read.store(0, std::memory_order_relaxed);
		get_tokens_calls.store(0, std::memory_order_relaxed);
		includes_opened.store(0, std::memory_order_relaxed);
		parsers_file_buffer.store(0, std::memory_order_relaxed);
		parsers_text_buffer.store(0, std::memory_order_relaxed);
		tokenize_ns.store(0, std::memory_order_relaxed);
		read_token_ns.store(0, std::memory_order_relaxed);
		convert_ns.store(0, std::memory_order_relaxed);
		get_tokens_ns.store(0, std::memory_order_relaxed);
		include_ns.store(0, std::memory_order_relaxed);
		fp_parse_calls.store(0, std::memory_order_relaxed);
		skip_until_calls.store(0, std::memory_order_relaxed);
		fast_skip_tokens.store(0, std::memory_order_relaxed);
		skip_until_ns.store(0, std::memory_order_relaxed);
	}
};

namespace
{
bool parse_token_as_float(std::string const &Token, float &Value)
{
	if (Token.empty())
	{
		return false;
	}

	char *end = nullptr;
	Value = std::strtof(Token.c_str(), &end);
	return end != Token.c_str();
}

bool parse_token_as_double(std::string const &Token, double &Value)
{
	if (Token.empty())
	{
		return false;
	}

	char *end = nullptr;
	Value = std::strtod(Token.c_str(), &end);
	return end != Token.c_str();
}
} // namespace

parser_cpu_metrics g_parser_cpu_metrics;
std::mutex g_parser_metrics_mutex;

bool ParserMetricsActive()
{
	return g_parser_metrics_depth.load(std::memory_order_acquire) > 0;
}

void ParserMetricsAddNs(std::atomic<std::uint64_t> &Counter, std::chrono::steady_clock::duration const Duration)
{
	Counter.fetch_add(
	    static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Duration).count()),
	    std::memory_order_relaxed);
}

class ParserMetricsReadTokenTimer
{
public:
	explicit ParserMetricsReadTokenTimer(bool const Enabled)
	    : m_enabled(Enabled)
	{
		if (m_enabled)
		{
			g_parser_cpu_metrics.read_token_calls.fetch_add(1, std::memory_order_relaxed);
			m_start = std::chrono::steady_clock::now();
		}
	}

	~ParserMetricsReadTokenTimer()
	{
		if (m_enabled)
		{
			ParserMetricsAddNs(g_parser_cpu_metrics.read_token_ns, std::chrono::steady_clock::now() - m_start);
		}
	}

private:
	bool const m_enabled;
	std::chrono::steady_clock::time_point m_start {};
};

class ParserMetricsTokenizeTimer
{
public:
	explicit ParserMetricsTokenizeTimer(bool const Enabled)
	    : m_enabled(Enabled)
	{
		if (m_enabled)
		{
			m_start = std::chrono::steady_clock::now();
		}
	}

	~ParserMetricsTokenizeTimer()
	{
		if (m_enabled)
		{
			ParserMetricsAddNs(g_parser_cpu_metrics.tokenize_ns, std::chrono::steady_clock::now() - m_start);
		}
	}

private:
	bool const m_enabled;
	std::chrono::steady_clock::time_point m_start {};
};

class ParserMetricsGetTokensTimer
{
public:
	explicit ParserMetricsGetTokensTimer(bool const Enabled)
	    : m_enabled(Enabled)
	{
		if (m_enabled)
		{
			g_parser_cpu_metrics.get_tokens_calls.fetch_add(1, std::memory_order_relaxed);
			m_start = std::chrono::steady_clock::now();
		}
	}

	~ParserMetricsGetTokensTimer()
	{
		if (m_enabled)
		{
			ParserMetricsAddNs(g_parser_cpu_metrics.get_tokens_ns, std::chrono::steady_clock::now() - m_start);
		}
	}

private:
	bool const m_enabled;
	std::chrono::steady_clock::time_point m_start {};
};

class ParserMetricsIncludeTimer
{
public:
	explicit ParserMetricsIncludeTimer(bool const Enabled)
	    : m_enabled(Enabled)
	{
		if (m_enabled)
		{
			m_start = std::chrono::steady_clock::now();
		}
	}

	~ParserMetricsIncludeTimer()
	{
		if (m_enabled)
		{
			ParserMetricsAddNs(g_parser_cpu_metrics.include_ns, std::chrono::steady_clock::now() - m_start);
		}
	}

private:
	bool const m_enabled;
	std::chrono::steady_clock::time_point m_start {};
};

double ParserMetricsNanosecondsToMilliseconds(std::uint64_t const Nanoseconds)
{
	return static_cast<double>(Nanoseconds) / 1'000'000.0;
}

std::string ParserMetricsFormatMilliseconds(std::uint64_t const Nanoseconds)
{
	std::ostringstream output;
	output << std::fixed << std::setprecision(3) << ParserMetricsNanosecondsToMilliseconds(Nanoseconds);
	return output.str();
}

std::vector<std::string> ParserMetricsReport()
{
	auto const read_token_calls = g_parser_cpu_metrics.read_token_calls.load(std::memory_order_relaxed);
	auto const tokens_read = g_parser_cpu_metrics.tokens_read.load(std::memory_order_relaxed);
	auto const get_tokens_calls = g_parser_cpu_metrics.get_tokens_calls.load(std::memory_order_relaxed);
	auto const includes_opened = g_parser_cpu_metrics.includes_opened.load(std::memory_order_relaxed);
	auto const parsers_file_buffer = g_parser_cpu_metrics.parsers_file_buffer.load(std::memory_order_relaxed);
	auto const parsers_text_buffer = g_parser_cpu_metrics.parsers_text_buffer.load(std::memory_order_relaxed);
	auto const tokenize_ns = g_parser_cpu_metrics.tokenize_ns.load(std::memory_order_relaxed);
	auto const read_token_ns = g_parser_cpu_metrics.read_token_ns.load(std::memory_order_relaxed);
	auto const convert_ns = g_parser_cpu_metrics.convert_ns.load(std::memory_order_relaxed);
	auto const get_tokens_ns = g_parser_cpu_metrics.get_tokens_ns.load(std::memory_order_relaxed);
	auto const include_ns = g_parser_cpu_metrics.include_ns.load(std::memory_order_relaxed);
	auto const fp_parse_calls = g_parser_cpu_metrics.fp_parse_calls.load(std::memory_order_relaxed);
	auto const skip_until_calls = g_parser_cpu_metrics.skip_until_calls.load(std::memory_order_relaxed);
	auto const fast_skip_tokens = g_parser_cpu_metrics.fast_skip_tokens.load(std::memory_order_relaxed);
	auto const skip_until_ns = g_parser_cpu_metrics.skip_until_ns.load(std::memory_order_relaxed);

	auto const cpu_total_ns = tokenize_ns + convert_ns + include_ns;
	std::string tokens_per_sec = "n/a";
	if (cpu_total_ns > 0 && tokens_read > 0)
	{
		auto const rate = static_cast<double>(tokens_read) / (static_cast<double>(cpu_total_ns) / 1'000'000'000.0);
		tokens_per_sec = to_string(rate, 0);
	}

	std::vector<std::string> report;
	report.emplace_back(
	    std::string("Parser CPU scope: read_token=") + std::to_string(read_token_calls)
	    + ", tokens=" + std::to_string(tokens_read)
	    + ", get_tokens=" + std::to_string(get_tokens_calls)
	    + ", includes=" + std::to_string(includes_opened)
	    + ", parsers_file=" + std::to_string(parsers_file_buffer)
	    + ", parsers_text=" + std::to_string(parsers_text_buffer)
	    + ", tokenize_ms=" + ParserMetricsFormatMilliseconds(tokenize_ns)
	    + ", read_token_ms=" + ParserMetricsFormatMilliseconds(read_token_ns)
	    + ", convert_ms=" + ParserMetricsFormatMilliseconds(convert_ns)
	    + ", get_tokens_ms=" + ParserMetricsFormatMilliseconds(get_tokens_ns)
	    + ", include_ms=" + ParserMetricsFormatMilliseconds(include_ns)
	    + ", fp_parse=" + std::to_string(fp_parse_calls)
	    + ", skip_until=" + std::to_string(skip_until_calls)
	    + ", fast_skip_tokens=" + std::to_string(fast_skip_tokens)
	    + ", skip_until_ms=" + ParserMetricsFormatMilliseconds(skip_until_ns)
	    + ", tokens_per_sec=" + tokens_per_sec);
	return report;
}

void ParserMetricsLogReport(std::vector<std::string> const &Report)
{
	for (auto const &line : Report)
	{
		WriteLog(line);
	}
}
} // namespace

namespace parser_metrics
{
bool active()
{
	return ParserMetricsActive();
}

convert_timer::convert_timer()
{
	enabled = active();
	if (enabled)
	{
		start = std::chrono::steady_clock::now();
	}
}

convert_timer::~convert_timer()
{
	if (enabled)
	{
		ParserMetricsAddNs(g_parser_cpu_metrics.convert_ns, std::chrono::steady_clock::now() - start);
	}
}
} // namespace parser_metrics

// constructors
cParser::cParser(std::string const &Stream, buffertype const Type, std::string Path, bool const Loadtraction, std::vector<std::string> Parameters, bool allowRandom)
    : allowRandomIncludes(allowRandom), LoadTraction(Loadtraction), mPath(Path)
{
	if (ParserMetricsActive())
	{
		if (Type == buffertype::buffer_FILE)
		{
			g_parser_cpu_metrics.parsers_file_buffer.fetch_add(1, std::memory_order_relaxed);
		}
		else
		{
			g_parser_cpu_metrics.parsers_text_buffer.fetch_add(1, std::memory_order_relaxed);
		}
	}

	// store to calculate sub-sequent includes from relative path
	if (Type == buffertype::buffer_FILE)
	{
		mFile = Stream;
	}
	// reset pointers and attach proper type of buffer
	switch (Type)
	{
	case buffer_FILE:
	{
		Path.append(Stream);
		mStream = ParserFileCacheOpen(Path);
		// content of *.inc files is potentially grouped together
		if ((Stream.size() >= 4) && (ToLower(Stream.substr(Stream.size() - 4)) == ".inc"))
		{
			mIncFile = true;
			scene::Groups.create();
		}
		break;
	}
	case buffer_TEXT:
	{
		mStream = std::make_shared<std::istringstream>(Stream);
		break;
	}
	default:
	{
		break;
	}
	}
	// calculate stream size
	if (mStream)
	{
		if (true == mStream->fail())
		{
			ErrorLog("Failed to open file \"" + Path + "\"");
		}
		else
		{
			mSize = mStream->rdbuf()->pubseekoff(0, std::ios_base::end);
			mStream->rdbuf()->pubseekoff(0, std::ios_base::beg);
			mLine = 1;
		}
	}
	// set parameter set if one was provided
	if (false == Parameters.empty())
	{
		parameters.swap(Parameters);
	}
}

// destructor
cParser::~cParser()
{

	if (true == mIncFile)
	{
		// wrap up the node group holding content of processed file
		scene::Groups.close();
	}
}

ParserFileCacheScope::ParserFileCacheScope()
{
	std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
	if (g_parser_file_cache_depth.load(std::memory_order_relaxed) == 0)
	{
		g_parser_file_cache.clear();
		g_parser_file_cache_enabled = Global.ScenarioParserFileCache;
		g_parser_file_cache_metrics.reset();
	}
	g_parser_file_cache_depth.fetch_add(1, std::memory_order_release);
}

ParserFileCacheScope::~ParserFileCacheScope()
{
	end();
}

void ParserFileCacheScope::end()
{
	if (false == m_active)
	{
		return;
	}

	std::vector<std::string> report;
	{
		std::lock_guard<std::mutex> lock(g_parser_file_cache_mutex);
		auto const depth = g_parser_file_cache_depth.load(std::memory_order_relaxed);
		if (depth <= 1)
		{
			report = ParserFileCacheReport();
			g_parser_file_cache_depth.store(0, std::memory_order_release);
			g_parser_file_cache.clear();
			g_parser_file_cache_enabled = false;
			g_parser_file_cache_metrics.reset();
		}
		else
		{
			g_parser_file_cache_depth.store(depth - 1, std::memory_order_release);
		}
		m_active = false;
	}

	ParserFileCacheLogReport(report);
}

ParserMetricsScope::ParserMetricsScope()
{
	std::lock_guard<std::mutex> lock(g_parser_metrics_mutex);
	if (g_parser_metrics_depth.load(std::memory_order_relaxed) == 0)
	{
		g_parser_cpu_metrics.reset();
	}
	g_parser_metrics_depth.fetch_add(1, std::memory_order_release);
}

ParserMetricsScope::~ParserMetricsScope()
{
	end();
}

void ParserMetricsScope::end()
{
	if (false == m_active)
	{
		return;
	}

	std::vector<std::string> report;
	{
		std::lock_guard<std::mutex> lock(g_parser_metrics_mutex);
		auto const depth = g_parser_metrics_depth.load(std::memory_order_relaxed);
		if (depth <= 1)
		{
			report = ParserMetricsReport();
			g_parser_metrics_depth.store(0, std::memory_order_release);
			g_parser_cpu_metrics.reset();
		}
		else
		{
			g_parser_metrics_depth.store(depth - 1, std::memory_order_release);
		}
		m_active = false;
	}

	ParserMetricsLogReport(report);
}

template <> glm::vec3 cParser::getToken(bool const ToLower, char const *Break)
{
	// NOTE: this specialization ignores default arguments
	getTokens(3, false, "\n\r\t  ,;[]");
	glm::vec3 output;
	*this >> output.x >> output.y >> output.z;
	return output;
};

template <> cParser &cParser::operator>>(std::string &Right)
{

	if (true == this->tokens.empty())
	{
		return *this;
	}

	parser_metrics::convert_timer timer;
	Right = this->tokens.front();
	this->tokens.pop_front();

	return *this;
}

template <> cParser &cParser::operator>>(bool &Right)
{

	if (true == this->tokens.empty())
	{
		return *this;
	}

	parser_metrics::convert_timer timer;
	Right = ((this->tokens.front() == "true") || (this->tokens.front() == "yes") || (this->tokens.front() == "1"));
	this->tokens.pop_front();

	return *this;
}

template <> bool cParser::getToken<bool>(bool const ToLower, const char *Break)
{

	auto const token = getToken<std::string>(true, Break);
	return ((token == "true") || (token == "yes") || (token == "1"));
}

// methods
cParser &cParser::autoclear(bool const Autoclear)
{

	m_autoclear = Autoclear;
	if (mIncludeParser)
	{
		mIncludeParser->autoclear(Autoclear);
	}

	return *this;
}

bool cParser::getTokens(unsigned int Count, bool ToLower, const char *Break)
{
	auto const metrics_active = ParserMetricsActive();
	ParserMetricsGetTokensTimer const get_tokens_timer(metrics_active);

	if (true == m_autoclear)
	{
		// legacy parser behaviour
		tokens.clear();
	}
	/*
	 if (LoadTraction==true)
	  trtest="niemaproblema"; //wczytywać
	 else
	  trtest="x"; //nie wczytywać
	*/
	/*
	    int i;
	    this->str("");
	    this->clear();
	*/
	std::string token; 
	for (unsigned int i = tokens.size(); i < Count; ++i)
	{
		readToken(token, ToLower, Break);
		if (token.empty())
		{
			// no more tokens
			break;
		}
		tokens.emplace_back(std::move(token));
		// collect parameters
		/*
		        if (i == 0)
		            this->str(token);
		        else
		        {
		            std::string temp = this->str();
		            temp.append("\n");
		            temp.append(token);
		            this->str(temp);
		        }
		*/
	}
	if (tokens.size() < Count)
		return false;
	else
		return true;
}

bool cParser::readTokenFloat(float &Value, bool ToLower, const char *Break)
{
	std::string token;
	readToken(token, ToLower, Break);
	if (false == parse_token_as_float(token, Value))
	{
		return false;
	}

	if (ParserMetricsActive())
	{
		g_parser_cpu_metrics.fp_parse_calls.fetch_add(1, std::memory_order_relaxed);
	}
	return true;
}

bool cParser::readTokenDouble(double &Value, bool ToLower, const char *Break)
{
	std::string token;
	readToken(token, ToLower, Break);
	if (false == parse_token_as_double(token, Value))
	{
		return false;
	}

	if (ParserMetricsActive())
	{
		g_parser_cpu_metrics.fp_parse_calls.fetch_add(1, std::memory_order_relaxed);
	}
	return true;
}

void cParser::readNextToken(std::string &Token, bool ToLower, const char *Break)
{
	readToken(Token, ToLower, Break);
}

std::string cParser::readTokenFromStream(bool ToLower, const char *Break)
{
	auto const metrics_active = ParserMetricsActive();
	ParserMetricsTokenizeTimer const tokenize_timer(metrics_active);

	std::string token;
	token.reserve(64);

	const auto breakTable = makeBreakTable(Break);
	char c = 0;


	while (token.empty() && mStream->peek() != EOF) {
		while (mStream->peek() != EOF) { // idk why but with mStream->get(c) not all cars are loaded
			c = static_cast<char>(mStream->get());
			if (c == '\n') {
				++mLine;
			}

			const unsigned char uc = static_cast<unsigned char>(c);
			if (breakTable[uc]) {
				// separator ends token (or continues skipping if token empty)
				if (!token.empty())
					break;
				continue;
			}

			if (ToLower) c = toLowerChar(c);
			token.push_back(c);

			if (findQuotes(token)) {
				continue; // glue quoted content
			}
			if (skipComments && trimComments(token)) {
				break; // don't glue tokens separated by comment
			}
		}
	}

	return token;
}

std::string cParser::readTokenFromStreamFast(bool ToLower, const char *Break)
{
	std::string token;
	token.reserve(32);

	const auto breakTable = makeBreakTable(Break);
	char c = 0;

	while (token.empty() && mStream->peek() != EOF)
	{
		while (mStream->peek() != EOF)
		{
			c = static_cast<char>(mStream->get());
			if (c == '\n')
			{
				++mLine;
			}

			const unsigned char uc = static_cast<unsigned char>(c);
			if (breakTable[uc])
			{
				if (!token.empty())
				{
					break;
				}
				continue;
			}

			if (ToLower)
			{
				c = toLowerChar(c);
			}
			token.push_back(c);

			if (token.find('\"') != std::string::npos && findQuotes(token))
			{
				continue;
			}
			if (token.find('/') != std::string::npos && skipComments && trimComments(token))
			{
				break;
			}
		}
	}

	return token;
}

void cParser::readTokenForSkip(std::string &out, bool ToLower, const char *Break)
{
	if (mIncludeParser)
	{
		mIncludeParser->readTokenForSkip(out, ToLower, Break);
		if (out.empty())
		{
			mIncludeParser = nullptr;
			readTokenForSkip(out, ToLower, Break);
		}
		return;
	}

	out = readTokenFromStreamFast(ToLower, Break);

	stripFirstTokenBOM(out, ToLower, Break);

	if (out.find('(') != std::string::npos)
	{
		substituteParameters(out, ToLower);
	}

	if (handleIncludeIfPresent(out, ToLower, Break))
	{
		if (ParserMetricsActive())
		{
			g_parser_cpu_metrics.fast_skip_tokens.fetch_add(1, std::memory_order_relaxed);
		}
		return;
	}

	if (ParserMetricsActive() && false == out.empty())
	{
		g_parser_cpu_metrics.fast_skip_tokens.fetch_add(1, std::memory_order_relaxed);
	}
}

void cParser::skipUntilKeyword(std::string const &Keyword, bool ToLower, const char *Break)
{
	auto const metrics_active = ParserMetricsActive();
	auto const timestart = metrics_active ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point {};

	if (metrics_active)
	{
		g_parser_cpu_metrics.skip_until_calls.fetch_add(1, std::memory_order_relaxed);
	}

	std::string token;
	std::string keyword_match = Keyword;
	if (ToLower)
	{
		keyword_match = ::ToLower(keyword_match);
	}

	for (;;)
	{
		readTokenForSkip(token, ToLower, Break);
		if (token.empty() || token == keyword_match)
		{
			break;
		}
	}

	if (metrics_active)
	{
		ParserMetricsAddNs(g_parser_cpu_metrics.skip_until_ns, std::chrono::steady_clock::now() - timestart);
	}
}

void cParser::stripFirstTokenBOM(std::string& token, bool ToLower, const char* Break) {
	if (!mFirstToken) return;
	mFirstToken = false;

	if (startsWithBOM(token)) {
		token.erase(0, 3);
	}

	// if first "token" was standalone BOM, read the next real token (avoid recursion)
	while (token.empty() && mStream->peek() != EOF) {
		readToken(token, ToLower, Break);
		// readToken will not re-enter BOM stripping because mFirstToken is now false
		break;
	}
}

void cParser::substituteParameters(std::string& token, bool ToLower) {
	if (parameters.empty()) return;

	// Replace occurrences of "(pN)" anywhere in token.
	// Keep behavior: if missing parameter -> "none".
	size_t pos = 0;
	while ((pos = token.find("(p", pos)) != std::string::npos) {
		const size_t close = token.find(')', pos);
		if (close == std::string::npos) break; // malformed -> stop like old behavior (it would substr weirdly)

		const std::string idxStr = token.substr(pos + 2, close - (pos + 2));
		token.erase(pos, (close - pos) + 1);

		const size_t nr = static_cast<size_t>(std::atoi(idxStr.c_str()));
		const std::string repl = (nr >= 1 && (nr - 1) < parameters.size())
			? parameters[nr - 1]
			: std::string("none");

		const size_t insertPos = pos;
		token.insert(insertPos, repl);

		if (ToLower) {
			// Lowercase only what we inserted (same intent as original)
			for (size_t i = insertPos; i < insertPos + repl.size(); ++i) {
				token[i] = toLowerChar(token[i]);
			}
		}

		pos = insertPos + repl.size(); // continue after inserted text
	}
}

void cParser::skipIncludeBlock() {
	// mimic original: while token != "end" readToken(true)
	std::string t;
	do {
		readToken(t, true);
	} while (t != "end" && !t.empty());
}

void cParser::startIncludeFromParser(cParser& srcParser, bool ToLower, std::string includefile) {
	auto const metrics_active = ParserMetricsActive();
	ParserMetricsIncludeTimer const include_timer(metrics_active);

	replace_slashes(includefile);

	const bool allowTraction =
		(true == LoadTraction) ||
		((false == contains(includefile, "tr/")) && (false == contains(includefile, "tra/")));

	if (!allowTraction) {
		// skip include block until "end" (original behavior in token-mode include)
		skipIncludeBlock();
		return;
	}

	const bool isTerrain = contains(includefile, "_ter.scm");
	if (isTerrain && true == Global.file_binary_terrain_state) {
		WriteLog("SBT found, ignoring: " + includefile);
		readParameters(srcParser); // preserve original side-effect: still consume parameters
		return;
	}

	if (Global.ParserLogIncludes) {
		if (isTerrain) WriteLog("including terrain: " + includefile);
		else {
			// WriteLog("including: " + includefile);
		}
	}

	if (metrics_active)
	{
		g_parser_cpu_metrics.includes_opened.fetch_add(1, std::memory_order_relaxed);
	}

	mIncludeParser = std::make_shared<cParser>(
		includefile, /*buffer_FILE*/ static_cast<buffertype>(/*buffer_FILE*/ 0), mPath, LoadTraction, readParameters(srcParser)
	);
	mIncludeParser->allowRandomIncludes = allowRandomIncludes;
	mIncludeParser->autoclear(m_autoclear);

	if (mIncludeParser->mSize <= 0) {
		ErrorLog("Bad include: can't open file \"" + includefile + "\"");
	}
}

bool cParser::handleIncludeIfPresent(std::string& token, bool ToLower, const char* Break) {
	// token-mode include: token == "include"
	if (expandIncludes && token == "include") {
		std::string includefile;
		if (allowRandomIncludes)
			includefile = deserialize_random_set(*this);
		else
			readToken(includefile, ToLower);

		startIncludeFromParser(*this, ToLower, std::move(includefile));

		// after processing include, return next token from current parser
		readToken(token, ToLower, Break);
		return true;
	}

	// line-mode HACK: Break == "\n\r" and line begins with "include"
	if ((std::strcmp(Break, "\n\r") == 0) && token.compare(0, 7, "include") == 0) {
		cParser includeparser(token.substr(7));
		std::string includefile;
		if (allowRandomIncludes)
			includefile = deserialize_random_set(includeparser);
		else
			includeparser.readToken(includefile, ToLower);

		startIncludeFromParser(includeparser, ToLower, std::move(includefile));

		readToken(token, ToLower, Break);
		return true;
	}

	return false;
}

void cParser::readToken(std::string &out, bool ToLower, const char *Break)
{
	auto const metrics_active = ParserMetricsActive();
	ParserMetricsReadTokenTimer const read_token_timer(metrics_active);

	if (mIncludeParser)
	{
		mIncludeParser->readToken(out, ToLower, Break);
		if (out.empty())
		{
			mIncludeParser = nullptr;
			out = readTokenFromStream(ToLower, Break);
		}
	}
	else
	{
		out = readTokenFromStream(ToLower, Break);
	}

	stripFirstTokenBOM(out, ToLower, Break);

	substituteParameters(out, ToLower);

	handleIncludeIfPresent(out, ToLower, Break);

	if (metrics_active && false == out.empty())
	{
		g_parser_cpu_metrics.tokens_read.fetch_add(1, std::memory_order_relaxed);
	}
}

std::vector<std::string> cParser::readParameters(cParser &Input)
{

	std::vector<std::string> includeparameters;
	std::string parameter;
	Input.readToken(parameter, false); // w parametrach nie zmniejszamy
	while ((parameter.empty() == false) && (parameter != "end"))
	{
		includeparameters.emplace_back(parameter);
		Input.readToken(parameter, false);
	}
	return includeparameters;
}

std::string cParser::readQuotes(char const Quote)
{ // read the stream until specified char or stream end
	std::string token;
	char c{0};
	bool escaped = false;
	while (mStream->get(c))
	{ // get all chars until the quote mark
		if (escaped)
		{
			escaped = false;
		}
		else
		{
			if (c == '\\')
			{
				escaped = true;
				continue;
			}
			else if (c == Quote)
				break;
		}

		if (c == '\n')
			++mLine; // update line counter
		token += c;
	}

	return token;
}

void cParser::skipComment(std::string const &Endmark)
{ // pobieranie znaków aż do znalezienia znacznika końca
	std::string input;
	char c{0};
	auto const endmarksize = Endmark.size();
	while (mStream->get(c))
	{
		if (c == '\n')
		{
			// update line counter
			++mLine;
		}
		input += c;
		if (input == Endmark) // szukanie znacznika końca
			break;
		if (input.size() >= endmarksize)
		{
			// keep the read text short, to avoid pointless string re-allocations on longer comments
			input = input.substr(1);
		}
	}
	return;
}

bool cParser::findQuotes(std::string &String)
{

	if (String.back() == '\"')
	{

		String.pop_back();
		String += readQuotes();
		return true;
	}
	return false;
}

bool cParser::trimComments(std::string &String)
{
	for (auto const &comment : mComments)
	{
		if (String.size() < comment.first.size())
		{
			continue;
		}

		if (String.compare(String.size() - comment.first.size(), comment.first.size(), comment.first) == 0)
		{
			skipComment(comment.second);
			String.resize(String.rfind(comment.first));
			return true;
		}
	}
	return false;
}

void cParser::injectString(const std::string &str)
{
	if (mIncludeParser)
	{
		mIncludeParser->injectString(str);
	}
	else
	{
		mIncludeParser = std::make_shared<cParser>(str, buffer_TEXT, "", LoadTraction, std::vector<std::string>(), allowRandomIncludes);
		mIncludeParser->autoclear(m_autoclear);
	}
}

int cParser::getProgress() const
{
	return static_cast<int>(mStream->rdbuf()->pubseekoff(0, std::ios_base::cur) * 100 / mSize);
}

int cParser::getFullProgress() const
{

	int progress = getProgress();
	if (mIncludeParser)
		return progress + ((100 - progress) * (mIncludeParser->getProgress()) / 100);
	else
		return progress;
}

std::size_t cParser::countTokens(std::string const &Stream, std::string Path)
{

	return cParser(Stream, buffer_FILE, Path).count();
}

std::size_t cParser::count()
{

	std::string token;
	size_t count{0};
	do
	{
		token.clear();
		readToken(token, false);
		++count;
	} while (false == token.empty());

	return count - 1;
}

void cParser::addCommentStyle(std::string const &Commentstart, std::string const &Commentend)
{

	mComments.insert(commentmap::value_type(Commentstart, Commentend));
}

// returns name of currently open file, or empty string for text type stream
std::string cParser::Name() const
{

	if (mIncludeParser)
	{
		return mIncludeParser->Name();
	}
	else
	{
		return mPath + mFile;
	}
}

// returns number of currently processed line
std::size_t cParser::Line() const
{

	if (mIncludeParser)
	{
		return mIncludeParser->Line();
	}
	else
	{
		return mLine;
	}
}

int cParser::LineMain() const
{
	return mIncludeParser ? -1 : mLine;
}
