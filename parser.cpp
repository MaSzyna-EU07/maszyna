/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "parser.h"
#include "utilities.h"
#include "Logs.h"

#include "scenenodegroups.h"

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
		Path.append(Stream);
		mStream = std::make_shared<std::ifstream>(Path, std::ios_base::binary);
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
	for (unsigned int i = tokens.size(); i < Count; ++i)
	{
		std::string token = readToken(ToLower, Break);
		if (true == token.empty())
		{
			// no more tokens
			break;
		}
		// collect parameters
		tokens.emplace_back(token);
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

std::string cParser::readTokenFromDelegate(bool ToLower, const char *Break)
{
	if (!mIncludeParser)
		return {};
	std::string token = mIncludeParser->readToken(ToLower, Break);
	if (token.empty())
	{
		mIncludeParser = nullptr;
	}
	return token;
}

std::string cParser::readTokenFromStream(bool ToLower, const char *Break)
{
	std::string token;
	// get the token yourself if the delegation attempt failed

	const auto breakTable = makeBreakTable(Break);
	char c = 0;


	while (token.empty() && mStream->peek() != EOF) {
		while (mStream->peek() != EOF) {
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
		token = readToken(ToLower, Break);
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
		t = readToken(true);
	} while (t != "end" && !t.empty());
}

void cParser::startIncludeFromParser(cParser& srcParser, bool ToLower, std::string includefile) {
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
		std::string includefile =
			allowRandomIncludes ? deserialize_random_set(*this) : readToken(ToLower);

		startIncludeFromParser(*this, ToLower, std::move(includefile));

		// after processing include, return next token from current parser
		token = readToken(ToLower, Break);
		return true;
	}

	// line-mode HACK: Break == "\n\r" and line begins with "include"
	if ((std::strcmp(Break, "\n\r") == 0) && token.compare(0, 7, "include") == 0) {
		cParser includeparser(token.substr(7));
		std::string includefile =
			allowRandomIncludes ? deserialize_random_set(includeparser) : includeparser.readToken(ToLower);

		startIncludeFromParser(includeparser, ToLower, std::move(includefile));

		token = readToken(ToLower, Break);
		return true;
	}

	return false;
}

std::string cParser::readToken(bool ToLower, const char *Break)
{
	std::string token;

	token = readTokenFromDelegate(ToLower, Break);
	if (token.empty())
	{
		token = readTokenFromStream(ToLower, Break);
	}

	stripFirstTokenBOM(token, ToLower, Break);

	// 4) parameter substitution
	substituteParameters(token, ToLower);

	// 5) include directive handling (may mutate token to next token)
	handleIncludeIfPresent(token, ToLower, Break);


	return token;
}

std::vector<std::string> cParser::readParameters(cParser &Input)
{

	std::vector<std::string> includeparameters;
	std::string parameter = Input.readToken(false); // w parametrach nie zmniejszamy
	while ((parameter.empty() == false) && (parameter != "end"))
	{
		includeparameters.emplace_back(parameter);
		parameter = Input.readToken(false);
	}
	return includeparameters;
}

std::string cParser::readQuotes(char const Quote)
{ // read the stream until specified char or stream end
	std::string token = "";
	char c{0};
	bool escaped = false;
	while (mStream->peek() != EOF)
	{ // get all chars until the quote mark
		c = mStream->get();

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
	std::string input = "";
	char c{0};
	auto const endmarksize = Endmark.size();
	while (mStream->peek() != EOF)
	{
		// o ile nie koniec pliku
		c = mStream->get(); // pobranie znaku
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
		token = "";
		token = readToken(false);
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
