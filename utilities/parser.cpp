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

#include "scene/scenenodegroups.h"
#include "scene/scenelayers.h"

/*
    MaSzyna EU07 locomotive simulator parser
    Copyright (C) 2003  TOLARIS

*/

/////////////////////////////////////////////////////////////////////////////////////////////////////
// cParser -- generic class for parsing text data.

namespace
{
inline char toLowerChar(char c)
{
	// Only fold ASCII letters. Bytes >= 0x80 belong to multibyte UTF-8
	// sequences and must be passed through untouched, otherwise a non-"C"
	// global locale could remap them and corrupt UTF-8 encoded text.
	// The remaining ASCII characters have no lower case form, and don't need the call.
	if (c >= 'A' && c <= 'Z')
		return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return c;
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
		// the file is read in one go, tokens are then extracted straight from the memory
		std::ifstream file(Path, std::ios_base::binary);
		if (file.fail())
		{
			mFailBit = true;
		}
		else
		{
			auto const size{file.rdbuf()->pubseekoff(0, std::ios_base::end)};
			file.rdbuf()->pubseekoff(0, std::ios_base::beg);
			if (size > 0 && static_cast<std::uintmax_t>(size) < mBuffer.max_size())
			{
				mBuffer.resize(static_cast<std::size_t>(size));
				file.read(mBuffer.data(), static_cast<std::streamsize>(mBuffer.size()));
				mBuffer.resize(static_cast<std::size_t>(std::max<std::streamsize>(0, file.gcount())));
			}
			else if (size != 0)
			{
				// not a file which can be read, e.g. a directory, which some systems allow to open
				mFailBit = true;
			}
		}
		// content of *.inc files is potentially grouped together
		if (Stream.size() >= 4 && ToLower(Stream.substr(Stream.size() - 4)) == ".inc")
		{
			mIncFile = true;
			scene::Groups.create();
		}
		break;
	}
	case buffer_TEXT:
	{
		mBuffer = Stream;
		break;
	}
	default:
	{
		break;
	}
	}
	// calculate stream size
	if (true == mFailBit)
	{
		ErrorLog("Failed to open file \"" + Path + "\"");
	}
	else
	{
		mSize = static_cast<std::streamoff>(mBuffer.size());
		mLine = 1;
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
	if (true == mLayerFile)
	{
		// content of the processed file ended, following nodes belong to the parent layer again
		scene::Layers.close();
	}
	if (true == mInstanceFile)
	{
		scene::Layers.instance_end();
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

	Right = std::move(this->tokens.front());
	this->tokens.pop_front();

	return *this;
}

template <> cParser &cParser::operator>>(bool &Right)
{

	if (true == this->tokens.empty())
	{
		return *this;
	}

	Right = this->tokens.front() == "true" || this->tokens.front() == "yes" || this->tokens.front() == "1";
	this->tokens.pop_front();

	return *this;
}

template <> std::string cParser::getToken<std::string>(bool const ToLower, const char *Break)
{
	// does what the generic version would, minus the trip of the token through the queue
	if (true == m_autoclear)
	{
		tokens.clear();
	}
	std::string token;
	if (false == tokens.empty())
	{
		*this >> token;
	}
	else
	{
		readToken(token, ToLower, Break);
	}
	return token;
}

template <> bool cParser::getToken<bool>(bool const ToLower, const char *Break)
{

	auto const token = getToken<std::string>(true, Break);
	return token == "true" || token == "yes" || token == "1";
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

// mimics std::istream::peek() != EOF of the stream the parser used to read from
bool cParser::hasChar()
{
	if (mEofBit || mFailBit)
	{
		// an attempt to read from a stream which isn't in good state marks it as failed
		mFailBit = true;
		return false;
	}
	if (static_cast<std::size_t>(mPosition) >= mBuffer.size())
	{
		mEofBit = true;
		return false;
	}
	return true;
}

// mimics std::istream::get(char&). NOTE: doesn't advance the read position, the caller does
bool cParser::getChar(char &Char)
{
	if (mEofBit || mFailBit)
	{
		mFailBit = true;
		return false;
	}
	if (static_cast<std::size_t>(mPosition) >= mBuffer.size())
	{
		// failed extraction at the end of data sets both flags
		mEofBit = true;
		mFailBit = true;
		return false;
	}
	Char = mBuffer[static_cast<std::size_t>(mPosition)];
	return true;
}

// prepares character lookup for specified set of token separators
void cParser::updateCharClasses(char const *Break)
{
	if (Break == nullptr)
	{
		Break = "";
	}
	if (mCharClassesValid && std::strcmp(mCharClassesBreak.c_str(), Break) == 0)
	{
		// the set of separators rarely changes from one token to another
		return;
	}
	mCharClassesBreak = Break;
	mCharClasses.fill(0);
	for (unsigned char const c : mCharClassesBreak)
	{
		mCharClasses[c] |= char_break;
	}
	// characters which need more than being added to the token:
	// line ends are counted,
	mCharClasses[static_cast<unsigned char>('\n')] |= char_special;
	// quotes glue the text they enclose into a single token,
	mCharClasses[static_cast<unsigned char>('\"')] |= char_special;
	// and a comment can start only once the last character of its opening mark arrives
	for (auto const &comment : mComments)
	{
		for (auto character{0}; character < 256; ++character)
		{
			// NOTE: the mark is compared with the token, which can hold the character converted to lower case
			if (comment.first.empty() || comment.first.back() == static_cast<char>(character) || comment.first.back() == toLowerChar(static_cast<char>(character)))
			{
				mCharClasses[character] |= char_special;
			}
		}
	}
	mCharClassesValid = true;
}

void cParser::readTokenFromStream(std::string &token, bool ToLower, const char *Break)
{
	token.clear();

	updateCharClasses(Break);
	char c = 0;
	bool separated = false; // the token was ended by a separator, which isn't a part of it


	while (token.empty() && hasChar()) {
		while (hasChar()) {
			auto const *const data{mBuffer.data()};
			auto const size{mBuffer.size()};
			auto const begin{static_cast<std::size_t>(mPosition)};
			if (mCharClasses[static_cast<unsigned char>(data[begin])] == 0)
			{
				// run of regular characters, which only need to be added to the token
				auto end{begin + 1};
				while (end < size && mCharClasses[static_cast<unsigned char>(data[end])] == 0)
				{
					++end;
				}
				if (token.empty())
				{
					mTokenBegin = mPosition;
				}
				auto const converted{token.size()};
				token.append(data + begin, end - begin);
				if (ToLower)
				{
					for (auto index{converted}; index < token.size(); ++index)
					{
						token[index] = toLowerChar(token[index]);
					}
				}
				mPosition = static_cast<std::streamoff>(end);
				continue;
			}

			c = data[begin];
			++mPosition;
			if (c == '\n') {
				++mLine;
			}

			const unsigned char uc = static_cast<unsigned char>(c);
			if (mCharClasses[uc] & char_break) {
				// separator ends token (or continues skipping if token empty)
				if (!token.empty()) {
					separated = true;
					break;
				}
				// the rest of a run of separators is skipped right away
				auto next{begin + 1};
				while (next < size && (mCharClasses[static_cast<unsigned char>(data[next])] & char_break)) {
					if (data[next] == '\n') {
						++mLine;
					}
					++next;
				}
				mPosition = static_cast<std::streamoff>(next);
				continue;
			}

			if (token.empty()) {
				mTokenBegin = mPosition - 1;
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
	// NOTE: comment glued to the end of a token is counted as a part of it
	mTokenEnd = separated ? mPosition - 1 : mPosition;
}

void cParser::stripFirstTokenBOM(std::string& token, bool ToLower, const char* Break) {
	if (!mFirstToken) return;
	mFirstToken = false;

	if (startsWithBOM(token)) {
		token.erase(0, 3);
		mTokenBegin += 3;
	}

	// if first "token" was standalone BOM, read the next real token (avoid recursion)
	if (token.empty() && hasChar()) {
		// readToken will not re-enter BOM stripping because mFirstToken is now false
		readToken(token, ToLower, Break);
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
		token.erase(pos, close - pos + 1);

		const size_t nr = static_cast<size_t>(std::atoi(idxStr.c_str()));
		const std::string repl = nr >= 1 && nr - 1 < parameters.size()
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
	replace_slashes(includefile);

	const bool allowTraction =
		true == LoadTraction ||
		(false == contains(includefile, "tr/") && false == contains(includefile, "tra/"));

	if (!allowTraction) {
		// skip include block until "end" (original behavior in token-mode include)
		skipIncludeBlock();
		return;
	}

	// layers are named by their files relative to the scenery folder; the terrain files (.txtf) come from includes made up by
	// the loader, relative to the folder of the simulator
	auto layername{mPath + includefile};
	layername = (layername.starts_with(Global.asCurrentSceneryPath) ? layername.substr(Global.asCurrentSceneryPath.size()) : includefile);

	const bool isTerrain = contains(includefile, "_ter.scm");
	if (isTerrain && true == Global.file_binary_terrain_state) {
		WriteLog("SBT found, ignoring: " + includefile);
		readParameters(srcParser); // preserve original side-effect: still consume parameters
		++Global.file_binary_terrain_skipped;
		if (sceneryLayers)
		{
			// the file is still a part of the scenery, even though its content comes from the binary terrain file
			scene::Layers.layer(scene::Layers.open(layername, include_site())).binary = true;
			scene::Layers.close();
		}
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

	if (sceneryLayers)
	{
		mIncludeParser->sceneryLayers = true;
		// content of an included scenery file forms a layer of its own. *.inc files are node templates
		// reused all over the scenery, so their content stays in the layer of the file which includes them
		if (mIncludeParser->mIncFile)
		{
			scene::Layers.template_used(includefile);
			// include of a template made directly by a layer file is handled by the editor as a whole. templates
			// included by that template are a part of it
			auto const site{include_site()};
			if (false == site.fixed && scene::Layers.instance() == 0 && mIncludeParser->mSize > 0)
			{
				scene::Layers.instance_begin(includefile, site.span);
				mIncludeParser->mInstanceFile = true;
			}
		}
		else if (mIncludeParser->mSize > 0)
		{
			auto site{include_site()};
			site.parameters = false == mIncludeParser->parameters.empty();
			scene::Layers.open(layername, site);
			mIncludeParser->mLayerFile = true;
		}
	}
}

// describes the include directive being processed, for the scenery layer bookkeeping
scene::include_site cParser::include_site() const
{
	scene::include_site site;
	// the directive can be rewritten on scenery save only if we know where it is, and it sits in a layer file
	site.fixed = mIncludeBegin < 0 || mFile.empty() || mIncFile;
	if (false == site.fixed)
	{
		site.span = {mIncludeBegin, mTokenEnd};
	}
	return site;
}

bool cParser::handleIncludeIfPresent(std::string& token, bool ToLower, const char* Break) {
	// token-mode include: token == "include"
	if (expandIncludes && token == "include") {
		mIncludeBegin = mTokenBegin;
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
	if (std::strcmp(Break, "\n\r") == 0 && token.compare(0, 7, "include") == 0) {
		mIncludeBegin = -1; // the directive is parsed out of a line of text, its exact location isn't known
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
	if (mIncludeParser)
	{
		mIncludeParser->readToken(out, ToLower, Break);
		if (out.empty())
		{
			mIncludeParser = nullptr;
			readTokenFromStream(out, ToLower, Break);
		}
	}
	else
	{
		readTokenFromStream(out, ToLower, Break);
	}

	// NOTE: the checks repeat what the called methods test first, to skip the calls for vast majority of the tokens
	if (mFirstToken)
	{
		stripFirstTokenBOM(out, ToLower, Break);
	}

	if (false == parameters.empty())
	{
		substituteParameters(out, ToLower);
	}

	if (out.size() >= 7 && out[0] == 'i')
	{
		// nothing else can be an include directive
		handleIncludeIfPresent(out, ToLower, Break);
	}
}

std::vector<std::string> cParser::readParameters(cParser &Input)
{

	std::vector<std::string> includeparameters;
	std::string parameter;
	Input.readToken(parameter, false); // w parametrach nie zmniejszamy
	while (parameter.empty() == false && parameter != "end")
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
	while (getChar(c))
	{ // get all chars until the quote mark
		++mPosition;
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
	char c{0};
	if (false == getChar(c))
	{
		return;
	}
	auto const begin{static_cast<std::size_t>(mPosition)};
	// szukanie znacznika końca
	// NOTE: with nothing to look for the comment runs until the end of data
	auto const found{Endmark.empty() ? std::string::npos : mBuffer.find(Endmark, begin)};
	auto const end{found != std::string::npos ? found + Endmark.size() : mBuffer.size()};
	// update line counter
	mLine += static_cast<std::size_t>(std::count(mBuffer.data() + begin, mBuffer.data() + end, '\n'));
	mPosition = static_cast<std::streamoff>(end);
	if (found == std::string::npos)
	{
		// the data ended before the comment did; record the attempt to read past the end
		getChar(c);
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
		mIncludeParser->sceneryLayers = sceneryLayers;
	}
}

int cParser::getProgress() const
{
	return (mSize > 0 ? static_cast<int>(mPosition * 100 / mSize) : 100);
}

int cParser::getFullProgress() const
{

	int progress = getProgress();
	if (mIncludeParser)
		return progress + (100 - progress) * mIncludeParser->getProgress() / 100;
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
	// the character lookup depends on the comment marks
	mCharClassesValid = false;
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

std::streamoff cParser::TokenBegin() const
{
	return mIncludeParser ? mIncludeParser->TokenBegin() : mTokenBegin;
}

std::streamoff cParser::TokenEnd() const
{
	return mIncludeParser ? mIncludeParser->TokenEnd() : mTokenEnd;
}

bool cParser::InLayerFile() const
{
	return mIncludeParser ? mIncludeParser->InLayerFile() : (false == mFile.empty() && false == mIncFile);
}

std::string_view cParser::remainingText() const
{
	if (false == tokens.empty())
	{
		return {};
	}
	// each parser on the way gets to change the tokens of the file being read
	auto const *source{this};
	while (true)
	{
		if (false == source->parameters.empty() || false == source->skipComments || source->mComments.size() != 2)
		{
			return {};
		}
		if (source->mIncludeParser == nullptr)
		{
			break;
		}
		source = source->mIncludeParser.get();
	}
	// NOTE: the first token of a file is left to the regular code, which deals with the byte order mark
	if (source->mFirstToken || source->mEofBit || source->mFailBit || static_cast<std::size_t>(source->mPosition) >= source->mBuffer.size())
	{
		return {};
	}
	return std::string_view{source->mBuffer}.substr(static_cast<std::size_t>(source->mPosition));
}

void cParser::skipText(std::size_t const Count, std::size_t const Lines, std::size_t const Tokenlength)
{
	if (mIncludeParser)
	{
		mIncludeParser->skipText(Count, Lines, Tokenlength);
		return;
	}
	mPosition = static_cast<std::streamoff>(std::min(static_cast<std::size_t>(mPosition) + Count, mBuffer.size()));
	mLine += Lines;
	mTokenEnd = mPosition;
	mTokenBegin = mPosition - static_cast<std::streamoff>(std::min(Tokenlength, static_cast<std::size_t>(mPosition)));
}
