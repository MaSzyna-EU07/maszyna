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
cParser::cParser(std::string const &Stream, buffertype const Type, std::string Path, bool const Loadtraction, std::vector<std::string> Parameters, bool allowRandom)
    : allowRandomIncludes(allowRandom), LoadTraction(Loadtraction), mPath(Path)
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
			scene::scenery_binary_reader reader;
			if (probe.good() && twinIsFresh(sourcefull, twinfull) && reader.open(probe))
			{
				// replay: serve tokens from the twin, no text tokenization at all
				m_entries = reader.entries();
				m_entrycount = m_entries.size();
				m_replay = true;
				m_entryindex = 0;
				mStream = std::make_shared<std::istringstream>(std::string());
				opened = true;
			}
			else
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
	// fallback flush in case the twin wasn't explicitly finalized
	flushBinaryTwin();

	if (true == mIncFile)
	{
		// wrap up the node group holding content of processed file
		scene::Groups.close();
	}
}

void cParser::flushBinaryTwin()
{
	// write the twin only once, only when compiling, and only if the source text was
	// fully consumed -- an aborted parse must not leave a truncated twin to be replayed
	if ((false == m_compiling) || (false == static_cast<bool>(m_writer)) || m_twinwritten)
	{
		return;
	}
	if ((nullptr == mStream) || (false == mStream->eof()))
	{
		return;
	}
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

	// a binary-twin replay child reports mSize 0 but is still valid
	if (mIncludeParser->mSize <= 0 && (false == mIncludeParser->m_replay)) {
		ErrorLog("Bad include: can't open file \"" + includefile + "\"");
	}
}

bool cParser::handleIncludeIfPresent(std::string& token, bool ToLower, const char* Break) {
	// token-mode include: token == "include"
	if (expandIncludes && token == "include") {
		processInclude(*this, ToLower);
		// after processing include, return next token from current parser
		readToken(token, ToLower, Break);
		return true;
	}

	// line-mode HACK: Break == "\n\r" and line begins with "include"
	if ((std::strcmp(Break, "\n\r") == 0) && token.compare(0, 7, "include") == 0) {
		cParser includeparser(token.substr(7));
		processInclude(includeparser, ToLower);
		readToken(token, ToLower, Break);
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

	bool fromOwn;
	if (mIncludeParser)
	{
		mIncludeParser->readToken(out, ToLower, Break);
		if (out.empty())
		{
			mIncludeParser = nullptr;
			out = readTokenFromStream(ToLower, Break);
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
		fromOwn = true;
	}

	stripFirstTokenBOM(out, ToLower, Break);

	// snapshot the raw token (BOM stripped, parameters NOT yet substituted) for the
	// binary twin, so parameterised includes can be re-substituted at replay time
	std::string const rawtoken = out;

	substituteParameters(out, ToLower);

	bool const wasInclude = handleIncludeIfPresent(out, ToLower, Break);

	// capture this file's own content into its twin. include directive tokens
	// (the keyword/filename/parameters) are excluded via wasInclude/suppression,
	// and child-include tokens (fromOwn == false) belong to the child's own twin.
	if (m_compiling && m_writer && fromOwn && (false == wasInclude)
	 && (false == m_capturesuppress) && (false == rawtoken.empty()))
	{
		// store numeric tokens as typed doubles (genuinely binary), everything else
		// (names, paths, keywords, "(pN)" placeholders) as strings
		double value = 0.0;
		if (sniffNumber(rawtoken, value))
		{
			m_writer->add_number(value);
		}
		else
		{
			m_writer->add_token(rawtoken);
		}
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

	while (m_entryindex < m_entries.size())
	{
		scene::scenery_entry const &entry = m_entries[m_entryindex++];
		if (entry.type == scene::scenery_entry_type::token)
		{
			out = entry.text;
			// re-apply this file's include parameters, mirroring the text path
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
		std::size_t pos = 0;
		std::string includefile = resolve_random_set(entry.fileexpr, pos);
		replace_slashes(includefile);
		startIncludeDirect(std::move(includefile), entry.params);
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

	out.clear();
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
	if (m_replay)
	{
		return ( m_entries.empty()
		    ? 100
		    : static_cast<int>(m_entryindex * 100 / m_entries.size()) );
	}
	if (mSize <= 0)
	{
		return 100;
	}
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
