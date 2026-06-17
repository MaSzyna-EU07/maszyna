/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <array>
#include <charconv>
#include <type_traits>

/////////////////////////////////////////////////////////////////////////////////////////////////////
// cParser -- generic class for parsing text data, either from file or provided string

namespace cparser_detail {
// Types for which std::from_chars matches the legacy stringstream extraction semantics.
// Character types are excluded on purpose: narrow-stream extraction of char/wchar_t reads a
// single character, whereas std::from_chars would parse an integer.
template<typename T>
inline constexpr bool use_from_chars_v =
    ( std::is_integral_v<T>
        && !std::is_same_v<T, bool>
        && !std::is_same_v<T, char>
        && !std::is_same_v<T, signed char>
        && !std::is_same_v<T, unsigned char>
        && !std::is_same_v<T, wchar_t>
        && !std::is_same_v<T, char16_t>
        && !std::is_same_v<T, char32_t> )
    || std::is_floating_point_v<T>;
} // namespace cparser_detail

class cParser //: public std::stringstream
{
  public:
    // parameters:
    enum buffertype
    {
        buffer_FILE,
        buffer_TEXT
    };
    // constructors:
    cParser(std::string const &Stream, buffertype const Type = buffer_TEXT, std::string Path = "", bool const Loadtraction = true, std::vector<std::string> Parameters = std::vector<std::string>(), bool allowRandom = false );
    // destructor:
    virtual ~cParser();
    // methods:
    template <typename Type_>
    cParser &
        operator>>( Type_ &Right );
    template <typename Output_>
	Output_
		getToken( bool const ToLower = true, char const *Break = "\n\r\t ;" ) {
            getTokens( 1, ToLower, Break );
		    Output_ output;
            *this >> output;
		    return output; };
    inline
    void
        ignoreToken() {
		std::string out;
            readToken(out); };
    inline
    bool
        expectToken( std::string const &Value ) {
		std::string out;
		readToken(out);
            return out == Value; };
    inline
    bool
        eof() {
            return mStream->eof(); };
    inline
    bool
        ok() {
            return ( !mStream->fail() ); };
    cParser &
        autoclear( bool const Autoclear );
    inline
    bool
        autoclear() const {
            return m_autoclear; }
    bool
        getTokens( unsigned int Count = 1, bool ToLower = true, char const *Break = "\n\r\t ;" );
	std::string readTokenFromStream(bool ToLower, const char *Break);
	void stripFirstTokenBOM(std::string &token, bool ToLower, const char *Break);
	void substituteParameters(std::string &token, bool ToLower);
	void skipIncludeBlock();
	// returns next incoming token, if any, without removing it from the set
    inline
    std::string
        peek() const {
            return (
                false == tokens.empty() ?
                    tokens.front() :
                    "" ); }
	// inject string as internal include
	void injectString(const std::string &str);

    // returns percentage of file processed so far
    int getProgress() const;
    int getFullProgress() const;
    //
    static std::size_t countTokens( std::string const &Stream, std::string Path = "" );
    // add custom definition of text which should be ignored when retrieving tokens
    void addCommentStyle( std::string const &Commentstart, std::string const &Commentend );
    // returns name of currently open file, or empty string for text type stream
    std::string Name() const;
    // returns number of currently processed line
    std::size_t Line() const;
	// returns number of currently processed line in main file, -1 if inside include
	int LineMain() const;
	bool expandIncludes = true;
	bool allowRandomIncludes = false;
    bool skipComments = true;

  private:
	void startIncludeFromParser(cParser &srcParser, bool ToLower, std::string includefile);
	bool handleIncludeIfPresent(std::string &token, bool ToLower, const char *Break);
	// methods:
    void readToken(std::string& out, bool ToLower = true, const char *Break = "\n\r\t ;");
	static std::vector<std::string> readParameters( cParser &Input );
    std::string readQuotes( char const Quote = '\"' );
    void skipComment( std::string const &Endmark );
    bool findQuotes( std::string &String );
    bool trimComments( std::string &String );
    std::size_t count();
    // members:
    bool m_autoclear { true }; // unretrieved tokens are discarded when another read command is issued (legacy behaviour)
    bool LoadTraction { true }; // load traction?
    std::shared_ptr<std::istream> mStream; // relevant kind of buffer is attached on creation.
    std::string mFile; // name of the open file, if any
    std::string mPath; // path to open stream, for relative path lookups.
    std::streamoff mSize { 0 }; // size of open stream, for progress report.
    std::size_t mLine { 0 }; // currently processed line
    bool mIncFile { false }; // the parser is processing an *.inc file
    bool mFirstToken { true }; // processing first token in the current file; helper used when checking for utf bom
    typedef std::map<std::string, std::string> commentmap;
    commentmap mComments {
        commentmap::value_type( "/*", "*/" ),
        commentmap::value_type( "//", "\n" ) };
    // cached separator lookup table to avoid rebuilding it on every token read
    char const *m_breakTableKey { nullptr };
    std::array<bool, 256> m_breakTable {};
    std::shared_ptr<cParser> mIncludeParser; // child class to handle include directives.
    std::vector<std::string> parameters; // parameter list for included file.
    std::deque<std::string> tokens;
};


template <>
glm::vec3
cParser::getToken( bool const ToLower, const char *Break );


template<typename Type_>
cParser&
cParser::operator>>( Type_ &Right ) {

    if( true == this->tokens.empty() ) { return *this; }

    if constexpr( cparser_detail::use_from_chars_v<Type_> ) {
        std::string const &token = this->tokens.front();
        char const *first { token.data() };
        char const *const last { first + token.size() };
        // legacy stream extraction accepts a leading '+', std::from_chars does not
        if( first != last && *first == '+' ) {
            ++first;
        }
        Type_ value {};
        auto const result { std::from_chars( first, last, value ) };
        if( result.ec == std::errc() ) {
            Right = value;
        }
        else {
            // fall back to the legacy path for inputs from_chars rejects (inf/nan/hex/...)
            std::stringstream converter( token );
            converter >> Right;
        }
        this->tokens.pop_front();
        return *this;
    }
    else {
        std::stringstream converter( this->tokens.front() );
        converter >> Right;
        this->tokens.pop_front();

        return *this;
    }
}

template<>
cParser&
cParser::operator>>( std::string &Right );

template<>
cParser&
cParser::operator>>( bool &Right );

template<>
bool
cParser::getToken<bool>( bool const ToLower, const char *Break );
