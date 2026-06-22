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
#include <memory>
#include <cstdint>

// binary scenery twin support (full definitions in scene/scenerybinary.h, included
// from parser.cpp). only forward declarations here to avoid pulling utilities.h in
// before cParser is defined.
namespace scene {
    enum class scenery_file_kind : std::uint8_t;
    enum class scenery_load_pass : std::uint8_t;
    class scenery_binary_writer;
    class scenery_binary_reader;
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
    cParser(std::string const &Stream, buffertype const Type = buffer_TEXT, std::string Path = "", bool const Loadtraction = true, std::vector<std::string> Parameters = std::vector<std::string>(), bool allowRandom = false, bool BakeOnly = false );
    // destructor:
    virtual ~cParser();
    // bake-only: tokenize this file into its binary twin without opening includes or
    // touching scene state, and return the (relative) filenames it includes so a parallel
    // baker can compile each of those twins on its own thread. requires BakeOnly ctor.
    std::vector<std::string> bakeFile();
    // select which class of nodes the replay serves (propagated to include children).
    // only meaningful in replay (twin) mode; ignored for text parsing.
    void setReplayPass( scene::scenery_load_pass Pass );
    // rewind the replay to the start of the twin with a (possibly new) pass, dropping any
    // open include child. used to run a second pass (visual) over an already-loaded twin.
    // returns false if this parser isn't replaying a twin (no second pass possible).
    bool restartReplay( scene::scenery_load_pass Pass );
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
            // in replay mode there is no character stream; exhaustion is reaching the
            // end of the twin's entries with no include still being drained
            if( m_replay ) {
                return m_replayexhausted && ( !mIncludeParser );
            }
            // text is tokenized from an in-memory buffer; eof is the read cursor
            // reaching its end
            return ( m_bufferpos >= m_buffer.size() ); };
    inline
    bool
        ok() {
            // historically !stream.fail(); with the in-memory buffer the stream is read
            // only once at construction, so its fail bit never flips at end-of-input.
            // ok() now means "there is still input to read", which is what the common
            // `while( parser.ok() )` loops rely on, and is true right after opening a
            // non-empty file (the `if( !ok() )` open checks) / false if the open failed
            // (empty buffer) or once everything has been consumed.
            if( m_replay ) { return ( false == m_replayexhausted ); }
            return ( m_bufferpos < m_buffer.size() ); };
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

	// writes the compiled binary twin to disk if this parser was compiling one and the
	// source was fully consumed. safe to call multiple times; the destructor also calls
	// it as a fallback. used to flush the top-level twin promptly once loading finishes.
	void flushBinaryTwin();

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
	// reads an include directive (filename expression + parameters) from srcParser,
	// records it for the binary twin when compiling, and opens the include with a
	// freshly evaluated filename (re-randomizing any random set).
	void processInclude(cParser &srcParser, bool ToLower);
	// opens an include directly from a (filename, parameters) pair, as reconstructed
	// from a binary twin's include entry, minus the stream-driven parameter parsing.
	void startIncludeDirect(std::string includefile, std::vector<std::string> Params);
	bool handleIncludeIfPresent(std::string &token, bool ToLower, const char *Break);
	// serves the next token when replaying a binary twin, transparently entering
	// child includes when an include entry is reached.
	void readReplayToken(std::string &out, bool ToLower, const char *Break);
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
    std::shared_ptr<std::istream> mStream; // attached on creation; kept for open/fail state.
    std::string m_buffer; // whole source text read into memory; tokenized from here
    std::size_t m_bufferpos { 0 }; // read cursor into m_buffer
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
    std::shared_ptr<cParser> mIncludeParser; // child class to handle include directives.
    std::vector<std::string> parameters; // parameter list for included file.
    std::deque<std::string> tokens;
    // --- binary scenery twin support ---
    // last token produced by readTokenFromStream was (partly) quoted -> case preserved
    bool m_lastquoted { false };
    // replay: this file is served from its binary twin instead of text
    bool m_replay { false };
    bool m_replayexhausted { false }; // all twin entries consumed (for inline eof())
    scene::scenery_load_pass m_replaypass {}; // value-init == scenery_load_pass::all
    std::string m_twinbuf; // whole twin file held in memory; the reader views into it
    std::unique_ptr<scene::scenery_binary_reader> m_reader; // streams entries from m_twinbuf
    // compile: this file is being captured into a binary twin alongside the text read
    bool m_compiling { false };
    std::unique_ptr<scene::scenery_binary_writer> m_writer;
    std::string m_binarytwinpath; // destination path for the compiled twin
    scene::scenery_file_kind m_binarykind {}; // value-initialised (== scenery_file_kind::scn)
    bool m_capturesuppress { false }; // suppress token capture (include directive internals)
    bool m_twinwritten { false }; // guards against writing the twin more than once
    // standalone/parallel bake: capture this file only, collect its include filenames
    bool m_bakeonly { false };
    std::vector<std::string> m_bakeincludes;
    // v6 node-marker tracking during bake (wrap each top-level node for pass selection)
    bool m_bakenode_active { false };
    int m_bakenode_count { 0 };       // entries captured since "node" (1=node,...,5=type)
    bool m_bakenode_visual { false };
    std::string m_bakenode_end;       // terminator token; empty until type classified
    // flushes a node still open at end-of-file (or unknown type), so its buffer is written
    void bakeFinishNode();
};


template <>
glm::vec3
cParser::getToken( bool const ToLower, const char *Break );


template<typename Type_>
cParser&
cParser::operator>>( Type_ &Right ) {

    if( true == this->tokens.empty() ) { return *this; }

    std::stringstream converter( this->tokens.front() );
    converter >> Right;
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
