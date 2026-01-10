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
#include <limits>

#include "scenenodegroups.h"

/*
    MaSzyna EU07 locomotive simulator parser
    Copyright (C) 2003  TOLARIS

*/

/////////////////////////////////////////////////////////////////////////////////////////////////////
// cParser -- generic class for parsing text data.

// constructors
cParser::cParser(std::string const &Stream, buffertype const Type, std::string Path, bool const Loadtraction, std::vector<std::string> Parameters, bool allowRandom)
    : mPath(Path), LoadTraction(Loadtraction), allowRandomIncludes(allowRandom)
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

std::string cParser::readToken(bool ToLower, const char *Break)
{

	std::string token;
	token.clear();
	token.reserve(64);

	if (mIncludeParser)
	{
		// see if there's include parsing going on. clean up when it's done.
		token = mIncludeParser->readToken(ToLower, Break);
		if (true == token.empty())
		{
			mIncludeParser = nullptr;
		}
	}
	if (true == token.empty())
	{
		cleanComments(Break, ToLower, token);
	}
	// check the first token for potential presence of utf bom
	if (mFirstToken)
	{
		mFirstToken = false;
		if (token.size() >= 3 
			&& static_cast<unsigned char>(token[0]) == 0xEF 
			&& static_cast<unsigned char>(token[1]) == 0xBB 
			&& static_cast<unsigned char>(token[2]) == 0xBF)
		{
			if (token.size() == 3)
			{
				// Entire token was just BOM - read next token
				return readToken(ToLower, Break);
			}
			else
			{
				// Remove BOM from beginning
				token.erase(0, 3);
			}
		}
	}

	if (false == parameters.empty())
	{
		// if there's parameter list, check the token for potential parameters to replace
		replaceParameters(token, ToLower);
	}

	// launch child parser if include directive found.
	// NOTE: parameter collecting uses default set of token separators.
	if (expandIncludes && token == "include")
	{
		std::string includefile = allowRandomIncludes ? deserialize_random_set(*this) : readToken(ToLower); // nazwa pliku
		replace_slashes(includefile);
		if ((true == LoadTraction) || ((false == contains(includefile, "tr/")) && (false == contains(includefile, "tra/"))))
		{
			if (false == contains(includefile, "_ter.scm"))
			{
				if (Global.ParserLogIncludes)
				{
					// WriteLog("including: " + includefile);
				}
				mIncludeParser = std::make_shared<cParser>(includefile, buffer_FILE, mPath, LoadTraction, readParameters(*this));
				mIncludeParser->allowRandomIncludes = allowRandomIncludes;
				mIncludeParser->autoclear(m_autoclear);
				if (mIncludeParser->mSize <= 0)
				{
					ErrorLog("Bad include: can't open file \"" + includefile + "\"");
				}
			}
			else
			{
				if (true == Global.file_binary_terrain_state)
				{
					WriteLog("SBT found, ignoring: " + includefile);
					readParameters(*this);
				}
				else
				{
					if (Global.ParserLogIncludes)
					{
						WriteLog("including terrain: " + includefile);
					}
					mIncludeParser = std::make_shared<cParser>(includefile, buffer_FILE, mPath, LoadTraction, readParameters(*this));
					mIncludeParser->allowRandomIncludes = allowRandomIncludes;
					mIncludeParser->autoclear(m_autoclear);
					if (mIncludeParser->mSize <= 0)
					{
						ErrorLog("Bad include: can't open file \"" + includefile + "\"");
					}
				}
			}
		}
		else
		{
			while (token != "end")
			{
				token = readToken(true); // minimize risk of case mismatch on comparison
			}
		}
		token = readToken(ToLower, Break);
	}
	else if ((std::strcmp(Break, "\n\r") == 0) && (token.compare(0, 7, "include") == 0))
	{
		// HACK: if the parser reads full lines we expect this line to contain entire include directive, to make parsing easier
		cParser includeparser(token.substr(7));
		std::string includefile = allowRandomIncludes ? deserialize_random_set(includeparser) : includeparser.readToken(ToLower); // nazwa pliku
		replace_slashes(includefile);
		bool has_tr = contains(includefile, "tr/") || contains(includefile, "tra/");
		bool has_ter = contains(includefile, "_ter.scm");
		if (LoadTraction || !has_tr)
		{
			if (!has_ter)
			{
				if (Global.ParserLogIncludes)
				{
					// WriteLog("including: " + includefile);
				}
				mIncludeParser = std::make_shared<cParser>(includefile, buffer_FILE, mPath, LoadTraction, readParameters(includeparser));
				mIncludeParser->allowRandomIncludes = allowRandomIncludes;
				mIncludeParser->autoclear(m_autoclear);
				if (mIncludeParser->mSize <= 0)
				{
					ErrorLog("Bad include: can't open file \"" + includefile + "\"");
				}
			}
			else
			{
				if (true == Global.file_binary_terrain_state)
				{
					WriteLog("SBT found, ignoring: " + includefile);
					readParameters(includeparser);
				}
				else
				{
					if (Global.ParserLogIncludes)
					{
						WriteLog("including terrain: " + includefile);
					}
					mIncludeParser = std::make_shared<cParser>(includefile, buffer_FILE, mPath, LoadTraction, readParameters(includeparser));
					mIncludeParser->allowRandomIncludes = allowRandomIncludes;
					mIncludeParser->autoclear(m_autoclear);
					if (mIncludeParser->mSize <= 0)
					{
						ErrorLog("Bad include: can't open file \"" + includefile + "\"");
					}
				}
			}
		}
		token = readToken(ToLower, Break);
	}
	// all done
	return token;
}

void cParser::cleanComments(const char *Break, bool ToLower, std::string &token)
{
	do
	{
		prepareBreakTable(Break);

		int ic = 0;
		while ((ic = mStream->get()) != EOF)
		{
			unsigned char uc = static_cast<unsigned char>(ic);

			// Break check first
			if (isBreak(uc))
			{
				if (static_cast<char>(uc) == '\n')
					++mLine;
				break;
			}

			char c = ToLower ? static_cast<char>(std::tolower(uc)) : static_cast<char>(uc);
			token.push_back(c);

			// Handle quotes
			if (c == '"')
			{
				token.pop_back();
				token += readQuotes();
				continue;
			}

			// Comments: clean and fast check
			if (skipComments && token.size() >= 2)
			{
				// Pointer to last two characters
				const char *end = &token[token.size() - 2];

				// Check for "//" or "/*" in one clear line
				if (end[0] == '/' && (end[1] == '/' || end[1] == '*'))
				{
					if (trimComments(token))
						break;
				}
			}
		}
	} while (token.empty() && mStream->peek() != EOF);
}
void cParser::replaceParameters(std::string &token, bool ToLower)
{
	size_t pos = 0;
	while ((pos = token.find("(p")) != std::string::npos)
	{
		size_t end = token.find(')', pos);
		if (end == std::string::npos)
		{
			break; // big no no
		}
		size_t nr = 0;
		bool valid = true;
		for (size_t i = pos + 2; i < end; ++i)
		{
			unsigned char c = static_cast<unsigned char>(token[i]);
			if (!std::isdigit(c))
			{
				valid = false; //someone has shitty parameter or smth
				return;
			}
			nr = nr * 10 + (token[i] - '0');
		}

		if (!valid || nr == 0)
		{
			continue;
		}
		size_t nr_index = nr - 1;
		if (nr_index < parameters.size())
		{
			const std::string &rep = parameters[nr_index];
			// ONE replace instead of erase+insert
			token.replace(pos, end - pos + 1, rep);

			if (ToLower)
			{
				for (size_t i = pos; i < pos + rep.size(); ++i)
				{
					token[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(token[i])));
				}
			}
			pos += rep.size();
		}
		else
		{
			token.replace(pos, end - pos + 1, "none");
			pos += 4; // "none".length()
		}
	}
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
{
	std::string token;
	token.reserve(64);

	while (true)
	{
		int ic = mStream->get();
		if (ic == EOF)
			break;

		char c = static_cast<char>(ic);

		// Handle escape sequences
		if (c == '\\')
		{
			int next = mStream->get();
			if (next != EOF)
			{
				token.push_back(static_cast<char>(next));
			}
			continue;
		}

		// End of quoted string
		if (c == Quote)
			break;

		// Count lines
		if (c == '\n')
			++mLine;

		token.push_back(c);
	}

	return token;
}

void cParser::skipComment(std::string const &Endmark)
{
	if (Endmark == "\n")
	{
		mStream->ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		++mLine;
		return;
	}

	const size_t endlen = Endmark.size();
	if (endlen == 0)
		return;

	// Circular buffer on stack for small patterns
	char buffer[8]; // Max comment end size ("*/" = 2)
	size_t pos = 0;
	size_t filled = 0;

	while (true)
	{
		int ic = mStream->get();
		if (ic == EOF)
			break;

		char c = static_cast<char>(ic);
		if (c == '\n')
			++mLine;

		// Store in circular buffer
		buffer[pos] = c;
		pos = (pos + 1) % endlen;
		if (filled < endlen)
			++filled;

		// Check for match
		if (filled == endlen)
		{
			bool match = true;
			for (size_t i = 0; i < endlen; ++i)
			{
				if (buffer[(pos + i) % endlen] != Endmark[i])
				{
					match = false;
					break;
				}
			}

			if (match)
				break;
		}
	}
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

bool cParser::trimComments(std::string &token)
{
	// Must have at least 2 characters for a comment
	const size_t len = token.size();
	if (len < 2)
		return false;

	// Get the last two characters
	const char last_two[2] = {token[len - 2], token[len - 1]};

	// Handle "//" - line comment
	if (last_two[0] == '/' && last_two[1] == '/')
	{
		skipComment("\n");
		token.resize(len - 2);
		return true;
	}

	// Handle "/*" - block comment
	if (last_two[0] == '/' && last_two[1] == '*')
	{
		skipComment("*/");
		token.resize(len - 2);
		return true;
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
