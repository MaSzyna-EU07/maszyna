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
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "utilities/Globals.h"

#include "scene/scenenodegroups.h"
#include "scene/scenerybinary.h"

#include <charconv>
#include <cmath>
#include <filesystem>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <unordered_set>

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

// true if the whole token is a finite decimal number; on success Value is set.
// used to store numeric tokens as typed doubles in the binary twin instead of text.
inline bool sniffNumber(const std::string &token, double &Value)
{
	if (token.empty())
	{
		return false;
	}
	const char *first = token.data();
	const char *last = token.data() + token.size();
	double value = 0.0;
	auto const result = std::from_chars(first, last, value);
	// require the entire token to be consumed and the value to be finite (reject
	// "inf"/"nan"/overflow, and identifiers like "12abc" or "1.2.3")
	if ((result.ec != std::errc()) || (result.ptr != last) || (false == std::isfinite(value)))
	{
		return false;
	}
	return (Value = value, true);
}

// shortest representation of a double that round-trips back to the same value,
// used when serving a numeric twin entry to the (text-oriented) deserializer
inline std::string formatNumber(double Value)
{
	char buffer[32];
	auto const result = std::to_chars(buffer, buffer + sizeof(buffer), Value);
	return std::string(buffer, result.ptr);
}

// classifies a scenery node by its `type` token (lower-cased) for v6 markers: sets Visual
// and the node's terminator token. returns false for an unrecognized type. mapping
// confirmed against the node deserializers.
inline bool classifyNodeType(std::string const &Type, bool &Visual, std::string &Endtoken)
{
	struct entry { char const *type; bool visual; char const *end; };
	static const entry table[] = {
		{ "model",               true,  "endmodel" },
		{ "triangles",           true,  "endtri" },
		{ "triangle_strip",      true,  "endtri" },
		{ "triangle_fan",        true,  "endtri" },
		{ "lines",               true,  "endline" },
		{ "line_strip",          true,  "endline" },
		{ "line_loop",           true,  "endline" },
		{ "track",               false, "endtrack" },
		{ "traction",            false, "endtraction" },
		{ "tractionpowersource", false, "end" },
		{ "memcell",             false, "endmemcell" },
		{ "eventlauncher",       false, "endevent" },
		{ "sound",               false, "endsound" },
		{ "dynamic",             false, "enddynamic" },
	};
	for (auto const &e : table) {
		if (Type == e.type) { Visual = e.visual; Endtoken = e.end; return true; }
	}
	return false;
}

inline bool endsWithLower(const std::string &s, const char *suffix)
{
	const std::string suf(suffix);
	return s.size() >= suf.size() && ToLower(s.substr(s.size() - suf.size())) == suf;
}

// scenery source files (and only these) get binary twins
inline bool isSceneryFile(const std::string &name)
{
	// text scenery component files share one syntax (per wiki: SCN/SCM/CTR/INC).
	// .eu7 is intentionally excluded: it is not a documented text format (it can be an
	// editor/binary file), so tokenizing it into a twin would be wrong.
	return endsWithLower(name, ".scn") || endsWithLower(name, ".inc")
	    || endsWithLower(name, ".scm") || endsWithLower(name, ".ctr");
}

inline scene::scenery_file_kind sceneryKind(const std::string &name)
{
	if (endsWithLower(name, ".inc")) return scene::scenery_file_kind::inc;
	if (endsWithLower(name, ".scm")) return scene::scenery_file_kind::scm;
	return scene::scenery_file_kind::scn;
}

// the twin may be replayed only if it is at least as new as its source text, so that
// editing a scenery file forces a recompile instead of replaying a stale twin
inline bool twinIsFresh(const std::string &sourcefull, const std::string &twinfull)
{
	std::error_code ec;
	auto const sourcetime = std::filesystem::last_write_time(sourcefull, ec);
	if (ec)
	{
		// can't read the source time (e.g. packed/missing): trust the twin
		return true;
	}
	auto const twintime = std::filesystem::last_write_time(twinfull, ec);
	if (ec)
	{
		return false;
	}
	return (twintime >= sourcetime);
}
} // namespace

// constructors
cParser::cParser(std::string const &Stream, buffertype const Type, std::string Path, bool const Loadtraction, std::vector<std::string> Parameters, bool allowRandom, bool BakeOnly)
    : allowRandomIncludes(allowRandom), LoadTraction(Loadtraction), mPath(Path), m_bakeonly(BakeOnly)
{
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
		// bake-only: always compile this one file's twin from text, never touch scene
		// groups and never replay; includes are recorded but not opened (the parallel
		// baker compiles them separately)
		if (m_bakeonly)
		{
			Path.append(Stream);
			mStream = std::make_shared<std::ifstream>(Path, std::ios_base::binary);
			if (false == mStream->fail())
			{
				m_compiling = true;
				std::string twinrel = Stream;
				erase_extension(twinrel);
				twinrel += scene::scenerybinary_extension_for(Stream);
				m_binarytwinpath = mPath + twinrel;
				m_binarykind = sceneryKind(Stream);
				m_writer = std::make_unique<scene::scenery_binary_writer>();
			}
			break;
		}

		// content of *.inc files is grouped together (same for text and binary replay)
		if (endsWithLower(Stream, ".inc"))
		{
			mIncFile = true;
			scene::Groups.create();
		}

		bool opened = false;
		// scenery files (.scn/.inc/.scm) are backed by a binary twin: replay it if
		// present, otherwise parse the text and compile a twin alongside it.
		// rainsted-created '$' overrides are always parsed from text.
		if (Global.file_binary_scenery && isSceneryFile(Stream) && (false == Stream.empty()) && (Stream[0] != '$'))
		{
			std::string twinrel = Stream;
			erase_extension(twinrel);
			twinrel += scene::scenerybinary_extension_for(Stream);
			std::string const twinfull = mPath + twinrel;
			std::string const sourcefull = mPath + Stream;

			std::ifstream probe(twinfull, std::ios_base::binary);
			bool replaying = false;
			if (probe.good() && twinIsFresh(sourcefull, twinfull))
			{
				// slurp the whole twin once; the reader parses it by pointer and serves
				// entries on demand (string tokens as views into this buffer)
				std::ostringstream slurp;
				slurp << probe.rdbuf();
				m_twinbuf = std::move(slurp).str();
				m_reader = std::make_unique<scene::scenery_binary_reader>();
				if (m_reader->open(m_twinbuf))
				{
					m_replay = true;
					mStream = std::make_shared<std::istringstream>(std::string());
					replaying = true;
					opened = true;
				}
				else
				{
					m_reader.reset();
				}
			}
			if (false == replaying)
			{
				Path.append(Stream);
				mStream = std::make_shared<std::ifstream>(Path, std::ios_base::binary);
				if (false == mStream->fail())
				{
					// no usable twin: compile one while parsing the text
					m_compiling = true;
					m_binarytwinpath = twinfull;
					m_binarykind = sceneryKind(Stream);
					m_writer = std::make_unique<scene::scenery_binary_writer>();
				}
				opened = true;
			}
		}
		if (false == opened)
		{
			// non-scenery file, or binary scenery disabled: plain text parse
			Path.append(Stream);
			mStream = std::make_shared<std::ifstream>(Path, std::ios_base::binary);
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
	// slurp the whole source into memory and tokenize from the buffer; reading char
	// by char from the stream (get()/peek()) goes through the virtual streambuf on
	// every character, which dominates parse time on large sceneries.
	if (mStream)
	{
		if (true == mStream->fail())
		{
			// bake-only parsers run on worker threads; the logger is not thread-safe, so
			// stay silent here (the driver checks ok() and simply skips a file it couldn't
			// open). missing includes still surface during the normal text/replay load.
			if (false == m_bakeonly)
			{
				ErrorLog("Failed to open file \"" + Path + "\"");
			}
		}
		else if (false == m_replay)
		{
			std::ostringstream slurp;
			slurp << mStream->rdbuf();
			m_buffer = std::move(slurp).str();
			m_bufferpos = 0;
			mSize = static_cast<std::streamoff>(m_buffer.size());
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
	// fallback flush in case the twin wasn't explicitly finalized
	flushBinaryTwin();

	if (true == mIncFile)
	{
		// wrap up the node group holding content of processed file
		scene::Groups.close();
	}
}

void cParser::bakeFinishNode()
{
	// flush a node still open at end-of-file or one whose type was unrecognized
	if (m_bakenode_active && m_writer)
	{
		m_writer->end_node(m_bakenode_visual, m_bakenode_haspos, m_bakenode_pos[0], m_bakenode_pos[1], m_bakenode_pos[2]);
	}
	m_bakenode_active = false;
}

std::vector<std::string> cParser::bakeFile()
{
	// drain the file: every token is captured into the twin, include directives are
	// recorded (their candidate filenames collected) but not opened
	std::string token;
	do { token = getToken<std::string>(); } while (false == token.empty());
	bakeFinishNode();

	// write the twin synchronously -- the parallel baker already runs one file per
	// worker thread, so there is no point handing this to the async pool
	if (m_compiling && m_writer && (false == m_twinwritten) && (m_bufferpos >= m_buffer.size()))
	{
		m_twinwritten = true;
		std::ofstream output(m_binarytwinpath, std::ios_base::binary);
		// errors are intentionally not logged here (worker thread); a missing/short twin
		// is simply recompiled on the next normal load
		(void)(output.good() && m_writer->write(output, m_binarykind));
	}
	return std::move(m_bakeincludes);
}

void cParser::flushBinaryTwin()
{
	// write the twin only once, only when compiling, and only if the source text was
	// fully consumed -- an aborted parse must not leave a truncated twin to be replayed
	if ((false == m_compiling) || (false == static_cast<bool>(m_writer)) || m_twinwritten)
	{
		return;
	}
	if (m_bufferpos < m_buffer.size())
	{
		// source not fully consumed (aborted parse): don't leave a truncated twin
		return;
	}
	bakeFinishNode(); // close a node still open at end-of-file
	m_twinwritten = true;

	// hand the finished writer to the background pool; serialization and file I/O then
	// overlap with the rest of the scene build instead of blocking it. ownership of the
	// writer transfers to the task, so this cParser no longer touches it.
	scene::scenerybinary_write_async(std::move(m_writer), m_binarytwinpath, m_binarykind);
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

std::string cParser::readTokenFromStream(bool ToLower, const char *Break)
{
	// the token is produced in its ORIGINAL case; lower-casing (when requested) is applied
	// to the consumer copy in readToken. this keeps the binary twin's captured tokens
	// case-faithful regardless of how a given read is cased, which is what lets the
	// headless/standalone bake tokenize correctly without knowing the grammar.
	(void)ToLower;
	m_lastquoted = false;

	std::string token;
	token.reserve(64);

	const auto breakTable = makeBreakTable(Break);
	char c = 0;


	while (token.empty() && m_bufferpos < m_buffer.size()) {
		while (m_bufferpos < m_buffer.size()) {
			c = m_buffer[m_bufferpos++];
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

			token.push_back(c);

			if (findQuotes(token)) {
				m_lastquoted = true; // came from a quoted span; never lower-cased
				continue; // glue quoted content
			}
			if (skipComments && trimComments(token)) {
				break; // don't glue tokens separated by comment
			}
		}
	}

	return token;
}

void cParser::stripFirstTokenBOM(std::string& token, bool ToLower, const char* Break) {
	if (!mFirstToken) return;
	mFirstToken = false;

	if (startsWithBOM(token)) {
		token.erase(0, 3);
	}

	// if first "token" was standalone BOM, read the next real token (avoid recursion)
	while (token.empty() && m_bufferpos < m_buffer.size()) {
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

void cParser::processInclude(cParser& srcParser, bool ToLower) {
	// the filename expression and parameters are part of the directive, not file
	// content, so keep them out of this file's own captured token stream
	bool const prevsuppress = m_capturesuppress;
	m_capturesuppress = true;

	std::vector<std::string> fileexpr;
	std::string pick;
	if (allowRandomIncludes) {
		// capture the verbatim expression (incl. random-set brackets) and pick one
		pick = deserialize_random_set_capture(srcParser, fileexpr);
	}
	else {
		std::string filename;
		srcParser.readToken(filename, ToLower);
		std::replace(filename.begin(), filename.end(), '\\', '/');
		fileexpr.emplace_back(filename);
		pick = filename;
	}
	// consume the directive's parameter list (up to "end")
	std::vector<std::string> params = readParameters(srcParser);

	m_capturesuppress = prevsuppress;

	// record the include reference verbatim so replay can re-randomize the choice
	if (m_compiling && m_writer) {
		m_writer->add_include(fileexpr, params);
	}

	if (m_bakeonly) {
		// standalone/parallel bake: don't open the child here. record every candidate
		// filename (a random set may list several) so the baker can compile each twin
		// independently on its own worker thread.
		for (auto const &token : fileexpr) {
			if ((token != "[") && (token != "]") && (false == token.empty())) {
				std::string candidate = token;
				replace_slashes(candidate);
				m_bakeincludes.emplace_back(std::move(candidate));
			}
		}
		return;
	}

	// open the include for the live load with a freshly evaluated filename
	replace_slashes(pick);
	startIncludeDirect(std::move(pick), std::move(params));
}

void cParser::startIncludeDirect(std::string includefile, std::vector<std::string> Params) {

	const bool allowTraction =
		(true == LoadTraction) ||
		((false == contains(includefile, "tr/")) && (false == contains(includefile, "tra/")));

	if (!allowTraction) {
		// traction loading disabled: the include is simply not opened
		return;
	}

	if (Global.ParserLogIncludes) {
		// WriteLog("including: " + includefile);
	}

	mIncludeParser = std::make_shared<cParser>(
		includefile, buffer_FILE, mPath, LoadTraction, std::move(Params)
	);
	mIncludeParser->allowRandomIncludes = allowRandomIncludes;
	mIncludeParser->autoclear(m_autoclear);
	// the child inherits the current load pass so the whole include tree is filtered
	// consistently (e.g. visuals-only on the second pass)
	mIncludeParser->setReplayPass(m_replaypass);

	// a binary-twin replay child reports mSize 0 but is still valid
	if (mIncludeParser->mSize <= 0 && (false == mIncludeParser->m_replay)) {
		ErrorLog("Bad include: can't open file \"" + includefile + "\"");
	}
}

void cParser::setReplayPass(scene::scenery_load_pass Pass)
{
	m_replaypass = Pass;
	if (m_reader)
	{
		m_reader->set_pass(Pass);
	}
}

bool cParser::restartReplay(scene::scenery_load_pass Pass)
{
	if ((false == m_replay) || (false == static_cast<bool>(m_reader)))
	{
		return false;
	}
	m_reader->open(m_twinbuf);
	m_reader->set_pass(Pass);
	m_replaypass = Pass;
	m_replayexhausted = false;
	mIncludeParser = nullptr;
	return true;
}

bool cParser::handleIncludeIfPresent(std::string& token, bool ToLower, const char* Break) {
	// token-mode include: token == "include". NOTE: we only process the directive here
	// and report it; readToken loops to fetch the next token. fetching it here (the old
	// behaviour) recursed once per consecutive include, overflowing the stack on files
	// with long runs of includes (e.g. signaling .scm) -- especially in bake-only mode
	// where the child isn't opened to break the chain.
	if (expandIncludes && token == "include") {
		processInclude(*this, ToLower);
		return true;
	}

	// line-mode HACK: Break == "\n\r" and line begins with "include"
	if ((std::strcmp(Break, "\n\r") == 0) && token.compare(0, 7, "include") == 0) {
		cParser includeparser(token.substr(7));
		processInclude(includeparser, ToLower);
		return true;
	}

	return false;
}

void cParser::readToken(std::string &out, bool ToLower, const char *Break)
{
	if (m_replay)
	{
		// served from the binary twin; includes are entered transparently
		readReplayToken(out, ToLower, Break);
		return;
	}

	// include directives are handled iteratively: a run of consecutive includes loops
	// here instead of recursing through readToken (which overflowed the stack on files
	// with hundreds of back-to-back include lines).
	for (;;)
	{
		bool fromOwn;
		bool quoted = false;
		if (mIncludeParser)
		{
			mIncludeParser->readToken(out, ToLower, Break);
			if (out.empty())
			{
				mIncludeParser = nullptr;
				out = readTokenFromStream(ToLower, Break);
				quoted = m_lastquoted;
				fromOwn = true;
			}
			else
			{
				fromOwn = false;
			}
		}
		else
		{
			out = readTokenFromStream(ToLower, Break);
			quoted = m_lastquoted;
			fromOwn = true;
		}

		stripFirstTokenBOM(out, ToLower, Break);

		// snapshot the original-case token (BOM stripped, before substitution) for the
		// twin, then produce the consumer copy: only unquoted tokens are lower-cased
		// (quoted spans keep their case, matching the legacy tokenizer)
		std::string rawtoken;
		bool rawquoted = false;
		if (fromOwn)
		{
			rawtoken = out;
			rawquoted = quoted;
			if (ToLower && (false == quoted))
			{
				for (auto &ch : out) { ch = toLowerChar(ch); }
			}
		}

		substituteParameters(out, ToLower);

		if (handleIncludeIfPresent(out, ToLower, Break))
		{
			// the include directive was consumed; fetch the next token
			continue;
		}

		// capture this file's own content into its twin. include directive tokens are
		// excluded (handled above); child-include tokens (fromOwn == false) belong to the
		// child's own twin.
		if (m_compiling && m_writer && fromOwn
		 && (false == m_capturesuppress) && (false == rawtoken.empty()))
		{
			// v6: wrap each top-level node in a marker so the reader can serve/skip it per
			// load pass. a node starts at the "node" keyword and ends at its type-specific
			// terminator; nodes are never nested.
			std::string const lowered = ::ToLower(rawtoken); // ::-qualified; the param `ToLower` shadows it
			if ((false == m_bakenode_active) && (lowered == "node"))
			{
				m_writer->begin_node();
				m_bakenode_active = true;
				m_bakenode_count = 0;
				m_bakenode_visual = false;
				m_bakenode_haspos = false;
				m_bakenode_end.clear();
			}
			else if (m_bakenode_active && m_bakenode_end.empty() && (lowered == "node"))
			{
				// previous node had an unrecognized type (no terminator found): close it
				// just before this new node starts
				bakeFinishNode();
				m_writer->begin_node();
				m_bakenode_active = true;
				m_bakenode_count = 0;
				m_bakenode_visual = false;
				m_bakenode_haspos = false;
				m_bakenode_end.clear();
			}

			// numeric tokens -> typed values; quoted strings preserve case at replay;
			// other strings (names, paths, keywords, "(pN)") may be lower-cased per consumer
			double value = 0.0;
			if ((false == rawquoted) && sniffNumber(rawtoken, value))
			{
				m_writer->add_number(value);
			}
			else
			{
				m_writer->add_token(rawtoken, rawquoted);
			}

			if (m_bakenode_active)
			{
				++m_bakenode_count;
				if ((m_bakenode_count == 5) && m_bakenode_end.empty())
				{
					// 5th node entry is the type token (node, range_max, range_min, name, type)
					classifyNodeType(lowered, m_bakenode_visual, m_bakenode_end);
					// a model node's entries 6,7,8 are its local X Y Z -- record them in the
					// marker so the camera-ring load can skip the node without reading its body
					m_bakenode_haspos = (lowered == "model");
				}
				else if (m_bakenode_haspos && (m_bakenode_count >= 6) && (m_bakenode_count <= 8))
				{
					m_bakenode_pos[m_bakenode_count - 6] = value;
				}
				else if ((false == m_bakenode_end.empty()) && (lowered == m_bakenode_end))
				{
					// terminator captured: close the node
					m_writer->end_node(m_bakenode_visual, m_bakenode_haspos, m_bakenode_pos[0], m_bakenode_pos[1], m_bakenode_pos[2]);
					m_bakenode_active = false;
				}
			}
		}
		return;
	}
}

void cParser::readReplayToken(std::string &out, bool ToLower, const char *Break)
{
	// drain an active child include first, exactly like the text path
	if (mIncludeParser)
	{
		mIncludeParser->readToken(out, ToLower, Break);
		if (false == out.empty())
		{
			return;
		}
		mIncludeParser = nullptr;
	}

	scene::scenery_entry_view entry;
	while (m_reader && m_reader->next(entry))
	{
		if (entry.type == scene::scenery_entry_type::token)
		{
			// stored in original case: lower-case per the consumer (unquoted tokens only),
			// then re-apply this file's include parameters, mirroring the text path
			out.assign(entry.text);
			if (ToLower) { for (auto &ch : out) { ch = toLowerChar(ch); } }
			substituteParameters(out, ToLower);
			return;
		}
		if (entry.type == scene::scenery_entry_type::qtoken)
		{
			// quoted token: case preserved verbatim
			out.assign(entry.text);
			substituteParameters(out, ToLower);
			return;
		}
		if (entry.type == scene::scenery_entry_type::number)
		{
			// typed numeric entry: hand back its shortest round-tripping text form
			// (no parameter substitution applies to a literal number)
			out = formatNumber(entry.number);
			return;
		}
		// include entry: re-evaluate the filename expression (re-randomizing any
		// random set), then enter the child (its own twin or text) and serve its
		// tokens; an empty/skipped child just advances to the next entry
		std::vector<std::string> fileexpr;
		fileexpr.reserve(entry.fileexpr.size());
		for (auto const sv : entry.fileexpr) { fileexpr.emplace_back(sv); }
		std::vector<std::string> params;
		params.reserve(entry.params.size());
		for (auto const sv : entry.params) { params.emplace_back(sv); }

		std::size_t pos = 0;
		std::string includefile = resolve_random_set(fileexpr, pos);
		replace_slashes(includefile);
		startIncludeDirect(std::move(includefile), std::move(params));
		if (mIncludeParser)
		{
			mIncludeParser->readToken(out, ToLower, Break);
			if (false == out.empty())
			{
				return;
			}
			mIncludeParser = nullptr;
		}
	}

	m_replayexhausted = true;
	out.clear();
}

bool cParser::skipReplayNode()
{
	// a node's entries are contiguous within a single file, served by the deepest active
	// include child -- delegate down to whichever parser is actually replaying it
	if (mIncludeParser) { return mIncludeParser->skipReplayNode(); }
	if (m_replay && m_reader && m_reader->skip_to_node_end())
	{
		// any tokens already pulled into the lookahead deque belong to the node we just
		// skipped past (or earlier) -- drop them so the next read starts at the next node
		tokens.clear();
		return true;
	}
	return false;
}

bool cParser::currentNodePosition(double &X, double &Y, double &Z)
{
	// delegate to the deepest active include child (it serves the current node), like skip
	if (mIncludeParser) { return mIncludeParser->currentNodePosition(X, Y, Z); }
	return (m_replay && m_reader && m_reader->node_position(X, Y, Z));
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
{ // read the buffer until specified char or end
	std::string token;
	char c{0};
	bool escaped = false;
	while (m_bufferpos < m_buffer.size())
	{ // get all chars until the quote mark
		c = m_buffer[m_bufferpos++];
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
	while (m_bufferpos < m_buffer.size())
	{
		c = m_buffer[m_bufferpos++];
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
	if (m_replay)
	{
		return ( m_reader ? m_reader->progress() : 100 );
	}
	if (m_buffer.empty())
	{
		return 100;
	}
	return static_cast<int>(m_bufferpos * 100 / m_buffer.size());
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

namespace scene
{
// Headless bake: drains the scenario through cParser so the existing include machinery
// tokenizes and (re)compiles every reachable file's binary twin, with no deserializer,
// scene, renderer or window involved. Because tokens are captured in original case
// (v5 format), draining with a uniform case is correct.
bool scenerybinary_bake_headless(std::string const &Scenariofile, std::string const &Path, bool Loadtraction)
{
	// parallel bake: discover the file graph by tokenizing each file in isolation (no
	// scene state, no includes opened -- just their references collected) and compile
	// each twin on a worker thread. each file is baked exactly once (deduped), so twin
	// writes never race.
	std::mutex mutex;
	std::condition_variable cv;
	std::unordered_set<std::string> visited;
	std::deque<std::string> queue;
	std::size_t active = 0;
	std::atomic<std::size_t> baked { 0 };

	auto const enqueue = [ & ]( std::string const &File ) {
		// only scenery component files get twins; dedupe so shared includes bake once
		if ( ( false == isSceneryFile( File ) ) || File.empty() || ( File[ 0 ] == '$' ) ) { return; }
		if ( visited.insert( File ).second ) { queue.push_back( File ); }
	};
	{ std::lock_guard<std::mutex> lock( mutex ); enqueue( Scenariofile ); }

	unsigned const workercount = std::max( 2u, std::min( 16u, std::thread::hardware_concurrency() ) );
	auto const worker = [ & ] {
		for ( ;; )
		{
			std::string file;
			{
				std::unique_lock<std::mutex> lock( mutex );
				cv.wait( lock, [ & ] { return ( false == queue.empty() ) || ( active == 0 ); } );
				if ( queue.empty() )
				{
					if ( active == 0 ) { cv.notify_all(); return; }
					continue;
				}
				file = std::move( queue.front() );
				queue.pop_front();
				++active;
			}

			std::vector<std::string> includes;
			{
				cParser parser( file, cParser::buffer_FILE, Path, Loadtraction, std::vector<std::string>(), false, /*BakeOnly*/ true );
				if ( parser.ok() )
				{
					includes = parser.bakeFile();
					++baked;
				}
			}

			{
				std::unique_lock<std::mutex> lock( mutex );
				for ( auto const &inc : includes ) { enqueue( inc ); }
				--active;
				cv.notify_all();
			}
		}
	};

	std::vector<std::thread> workers;
	workers.reserve( workercount );
	for ( unsigned i = 0; i < workercount; ++i ) { workers.emplace_back( worker ); }
	for ( auto &thread : workers ) { thread.join(); }

	WriteLog( "Bake: compiled " + std::to_string( baked.load() ) + " binary scenery twins from \"" + Scenariofile + "\"" );
	return ( baked.load() > 0 );
}
} // namespace scene
