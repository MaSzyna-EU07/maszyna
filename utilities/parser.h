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
#include <string_view>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <array>
#include <charconv>
#include <type_traits>

namespace scene
{
struct include_site;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// cParser -- generic class for parsing text data, either from file or provided string

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
            return mEofBit; };
    inline
    bool
        ok() {
            return !mFailBit; };
    cParser &
        autoclear( bool const Autoclear );
    inline
    bool
        autoclear() const {
            return m_autoclear; }
    bool
        getTokens( unsigned int Count = 1, bool ToLower = true, char const *Break = "\n\r\t ;" );
	void readTokenFromStream(std::string &token, bool ToLower, const char *Break);
	void stripFirstTokenBOM(std::string &token, bool ToLower, const char *Break);
	void substituteParameters(std::string &token, bool ToLower);
	void skipIncludeBlock();
	// returns next incoming token, if any, without removing it from the set
    inline
    std::string
        peek() const {
            return false == tokens.empty() ? tokens.front() : ""; }
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
	// location of the most recently read token in its source file, as byte offsets
	std::streamoff TokenBegin() const;
	std::streamoff TokenEnd() const;
	// true if the most recently read token came straight from a scenery file which isn't an *.inc node template
	bool InLayerFile() const;
	// text which follows the most recently read token, for the readers which take entries of a known kind straight from
	// the text instead of token by token. empty if the tokens can't be taken that way: some wait to be picked up, there
	// are parameters of an include to put in them, or comment marks other than the standard ones are in use
	std::string_view remainingText() const;
	// moves past specified amount of the text returned by remainingText(): Lines is the number of line breaks in it,
	// Tokenlength the length of the token it ends with
	void skipText(std::size_t Count, std::size_t Lines, std::size_t Tokenlength);
	// converts text holding a plain number without involving a stream. returns: true on success, false if the text needs regular treatment
	template <typename Type_>
	static bool parseNumber( std::string_view Token, Type_ &Output );
	bool expandIncludes = true;
	bool allowRandomIncludes = false;
    bool skipComments = true;
	bool sceneryLayers = false; // included scenery files are registered as scenery layers (scenery opened for editing)

  private:
    // types:
    // retrieved tokens waiting for pick up. a vector with moving head instead of a deque,
    // as the latter can allocate memory for each stored item
    struct token_queue {
        std::vector<std::string> items;
        std::size_t head { 0 };

        bool empty() const {
            return head == items.size(); }
        std::size_t size() const {
            return items.size() - head; }
        std::string & front() {
            return items[ head ]; }
        std::string const & front() const {
            return items[ head ]; }
        void emplace_back( std::string &&Token ) {
            items.emplace_back( std::move( Token ) ); }
        void pop_front() {
            ++head;
            if( head == items.size() ) {
                clear(); }
            else if( head >= 64 && head * 2 >= items.size() ) {
                // keep the queue from growing without end if it never gets fully emptied
                items.erase( items.begin(), items.begin() + static_cast<std::ptrdiff_t>( head ) );
                head = 0; } }
        void clear() {
            items.clear();
            head = 0; }
    };
	void startIncludeFromParser(cParser &srcParser, bool ToLower, std::string includefile);
	scene::include_site include_site() const;
	bool handleIncludeIfPresent(std::string &token, bool ToLower, const char *Break);
	// methods:
    void readToken(std::string& out, bool ToLower = true, const char *Break = "\n\r\t ;");
	static std::vector<std::string> readParameters( cParser &Input );
    std::string readQuotes( char const Quote = '\"' );
    void skipComment( std::string const &Endmark );
    bool findQuotes( std::string &String );
    bool trimComments( std::string &String );
    std::size_t count();
    // input access. these mimic peek() and get() of the stream the parser used to read from,
    // including their effect on the state reported by eof() and ok()
    bool hasChar();
    bool getChar( char &Char );
    void updateCharClasses( char const *Break );
    // members:
    bool m_autoclear { true }; // unretrieved tokens are discarded when another read command is issued (legacy behaviour)
    bool LoadTraction { true }; // load traction?
    std::string mBuffer; // content of the open file or the provided text, loaded on creation.
    bool mEofBit { false }; // input state, maintained the way a stream would do it
    bool mFailBit { false };
    enum charclass : unsigned char {
        char_break = 1, // separates tokens
        char_special = 2 }; // can't be simply added to the token: line end, quote, or potential end of a comment mark
    std::array<unsigned char, 256> mCharClasses {}; // lookup built for the most recently used set of token separators
    std::string mCharClassesBreak; // set of separators the lookup was built for
    bool mCharClassesValid { false };
    std::string mFile; // name of the open file, if any
    std::string mPath; // path to open stream, for relative path lookups.
    std::streamoff mSize { 0 }; // size of open stream, for progress report.
    std::size_t mLine { 0 }; // currently processed line
    bool mIncFile { false }; // the parser is processing an *.inc file
    bool mLayerFile { false }; // the parser is processing a file registered as scenery layer
    bool mInstanceFile { false }; // the parser is processing a template whose include is registered with the scenery layers
    std::streamoff mPosition { 0 }; // amount of bytes read from the stream so far
    std::streamoff mTokenBegin { 0 }; // location of the most recently read token
    std::streamoff mTokenEnd { 0 };
    std::streamoff mIncludeBegin { -1 }; // location of the include directive being processed, -1 if unknown
    bool mFirstToken { true }; // processing first token in the current file; helper used when checking for utf bom
    typedef std::map<std::string, std::string> commentmap;
    commentmap mComments {
        commentmap::value_type( "/*", "*/" ),
        commentmap::value_type( "//", "\n" ) };
    std::shared_ptr<cParser> mIncludeParser; // child class to handle include directives.
    std::vector<std::string> parameters; // parameter list for included file.
    token_queue tokens;
};


template <>
glm::vec3
cParser::getToken( bool const ToLower, const char *Break );


template<typename Type_>
bool
cParser::parseNumber( std::string_view const Token, Type_ &Output ) {

    using type = std::remove_cv_t<Type_>;
    if constexpr(
#ifdef __cpp_lib_to_chars
        std::is_floating_point_v<type> ||
#endif
        ( std::is_integral_v<type>
       && false == std::is_same_v<type, bool>
       && false == std::is_same_v<type, char>
       && false == std::is_same_v<type, signed char>
       && false == std::is_same_v<type, unsigned char>
       && false == std::is_same_v<type, wchar_t>
       && false == std::is_same_v<type, char8_t>
       && false == std::is_same_v<type, char16_t>
       && false == std::is_same_v<type, char32_t> ) ) {

        auto const *first { Token.data() };
        auto const *last { first + Token.size() };
        // only text which is a number from start to end is converted here. anything else, including forms
        // the stream extraction doesn't recognize (inf, nan), is left for the stream to make sense of
        auto const *digits { ( first != last && *first == '-' ) ? first + 1 : first };
        if( digits == last ) { return false; }
        if( ( *digits < '0' || *digits > '9' ) && *digits != '.' ) { return false; }
        type value {};
        auto const result { std::from_chars( first, last, value ) };
        if( result.ec != std::errc() || result.ptr != last ) { return false; }
        Output = value;
        return true;
    }
    else {
        return false;
    }
}

template<typename Type_>
cParser&
cParser::operator>>( Type_ &Right ) {

    if( true == this->tokens.empty() ) { return *this; }

    if( false == parseNumber( this->tokens.front(), Right ) ) {
        std::stringstream converter( this->tokens.front() );
        converter >> Right;
    }
    this->tokens.pop_front();

    return *this;
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

template<>
std::string
cParser::getToken<std::string>( bool const ToLower, const char *Break );
