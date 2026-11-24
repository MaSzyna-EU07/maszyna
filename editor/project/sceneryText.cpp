/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/project/sceneryText.hpp"
#include "utilities/utilities.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <set>
#include <utility>

namespace scenery_text
{

namespace
{

bool separator(char const C)
{
	return C == ' ' || C == '\t' || C == '\n' || C == '\r' || C == ';';
}

// the statements which take more than one word, with the words ending them, as state_serializer::preview_include() has them
std::array<std::pair<std::string_view, std::string_view>, 20> const node_ends{{
    {"dynamic", "enddynamic"},
    {"track", "endtrack"},
    {"road", "endroad"},
    {"junction", "endjunction"},
    {"crossing", "endcrossing"},
    {"spawn", "endspawn"},
    {"despawn", "enddespawn"},
    {"crosswalk", "endcrosswalk"},
    {"traction", "endtraction"},
    {"tractionpowersource", "end"},
    {"model", "endmodel"},
    {"triangles", "endtri"},
    {"triangle_strip", "endtri"},
    {"triangle_fan", "endtri"},
    {"lines", "endline"},
    {"line_strip", "endline"},
    {"line_loop", "endline"},
    {"memcell", "endmemcell"},
    {"eventlauncher", "end"},
    {"sound", "endsound"},
}};

std::array<std::pair<std::string_view, std::string_view>, 16> const statement_ends{{
    {"event", "endevent"},
    {"trainset", ""}, // its words are counted, its vehicles are nodes of their own up to endtrainset
    {"isolated", "endisolated"},
    {"area", "endarea"},
    {"assignment", "endassignment"},
    {"atmo", "endatmo"},
    {"camera", "endcamera"},
    {"config", "endconfig"},
    {"description", "enddescription"},
    {"light", "endlight"},
    {"sky", "endsky"},
    {"test", "endtest"},
    {"time", "endtime"},
    {"terrain", "endterrain"},
    {"heightmap_terrain", "endheightmap_terrain"},
    {"editorterrain", "endeditorterrain"},
}};

// the statements of a few words, their count
std::size_t word_count(std::string_view const Keyword)
{
	if (Keyword == "origin" || Keyword == "rotate" || Keyword == "scale")
		return 3;
	if (Keyword == "trainset")
		return 4;
	if (Keyword == "lua")
		return 1;
	return 0;
}

std::string normalized(std::string_view const Name)
{
	auto result{lowered(Name)};
	std::replace(result.begin(), result.end(), '\\', '/');
	while (result.starts_with("./"))
		result.erase(0, 2);
	return result;
}


// (p1), (p2)... put in place by the parameters of the include, a missing one as none (cParser::substituteParameters())
void substitute(std::string &Text, std::vector<std::string> const &Parameters)
{
	if (Parameters.empty())
		return;
	std::size_t at{0};
	while ((at = Text.find("(p", at)) != std::string::npos)
	{
		auto const close{Text.find(')', at)};
		if (close == std::string::npos)
			break;
		auto const index{static_cast<std::size_t>(std::atoi(Text.substr(at + 2, close - at - 2).c_str()))};
		auto const value{index >= 1 && index - 1 < Parameters.size() ? Parameters[index - 1] : std::string{"none"}};
		Text.replace(at, close - at + 1, value);
		at += value.size();
	}
}

// the next word of the text from At on, its bytes, the line it starts on counted on; none at the end
struct word_span
{
	std::size_t begin{0};
	std::size_t end{0};
	std::size_t line{0};
};
std::optional<word_span> next_word(std::string_view const Text, std::size_t &At, std::size_t &Line)
{
	// the byte order mark of a file in utf-8
	if (At == 0 && Text.starts_with("\xEF\xBB\xBF"))
		At = 3;
	auto const lines = [&](std::size_t const From, std::size_t const To) { Line += static_cast<std::size_t>(std::count(Text.begin() + static_cast<std::ptrdiff_t>(From), Text.begin() + static_cast<std::ptrdiff_t>(To), '\n')); };
	while (At < Text.size())
	{
		auto const c{Text[At]};
		if (separator(c))
		{
			if (c == '\n')
				++Line;
			++At;
			continue;
		}
		if (Text.compare(At, 2, "//") == 0)
		{
			auto const end{Text.find('\n', At)};
			At = (end == std::string_view::npos ? Text.size() : end);
			continue;
		}
		if (Text.compare(At, 2, "/*") == 0)
		{
			auto const end{Text.find("*/", At + 2)};
			auto const stop{end == std::string_view::npos ? Text.size() : end + 2};
			lines(At, stop);
			At = stop;
			continue;
		}
		word_span word;
		word.line = Line;
		word.begin = At;
		auto const comment = [&]() { return Text[At] == '/' && At + 1 < Text.size() && (Text[At + 1] == '/' || Text[At + 1] == '*'); };
		while (At < Text.size() && false == separator(Text[At]) && false == comment())
		{
			if (Text[At] == '"')
			{
				// the quotes glue the text between them into the word, blanks and line ends too
				auto const close{Text.find('"', At + 1)};
				auto const stop{close == std::string_view::npos ? Text.size() : close + 1};
				lines(At, stop);
				At = stop;
				continue;
			}
			++At;
		}
		word.end = At;
		return word;
	}
	return std::nullopt;
}

// the word is the keyword, its letters A to Z of either size
bool is(std::string_view const Word, std::string_view const Keyword)
{
	if (Word.size() != Keyword.size())
		return false;
	for (std::size_t i = 0; i < Word.size(); ++i)
	{
		auto const c{Word[i] >= 'A' && Word[i] <= 'Z' ? static_cast<char>(Word[i] - 'A' + 'a') : Word[i]};
		if (c != Keyword[i])
			return false;
	}
	return true;
}

double real(token const &Token)
{
	return std::atof(Token.text.c_str());
}

std::size_t constexpr kDepth{32}; // the includes nested no deeper

// the words of the files read one after another, the includes followed where they're met
class reader
{
  public:
	reader(std::filesystem::path Folder, read_options const &Options, scenery &Output) : m_folder{std::move(Folder)}, m_options{Options}, m_output{Output} {}

	void open(std::string const &Name, std::size_t const Parent, std::size_t const Line, std::vector<std::string> Parameters)
	{
		included_file file;
		file.name = Name;
		std::replace(file.name.begin(), file.name.end(), '\\', '/');
		file.parent = Parent;
		file.line = Line;
		file.context = context();
		file.parameters = std::move(Parameters);
		file.map = (Parent != static_cast<std::size_t>(-1) && m_output.files[Parent].map) || (false == m_options.map.empty() && same_file(file.name, m_options.map));
		auto const index{m_output.files.size()};
		if (m_options.skip_terrain && Parent != static_cast<std::size_t>(-1) && lowered(file.name).find("_ter.scm") != std::string::npos)
		{
			file.skipped = true;
			m_output.files.push_back(std::move(file));
			return;
		}
		// a file including itself, on its own or through others, would never end
		if (m_frames.size() >= kDepth)
		{
			file.missing = true;
			m_output.problems.push_back("includes nested deeper than " + std::to_string(kDepth) + " at " + file.name + ", included from " + m_output.files[Parent].name + " line " + std::to_string(Line));
			m_output.files.push_back(std::move(file));
			return;
		}
		auto text{file_text(m_folder / file.name)};
		if (false == text.has_value())
		{
			file.missing = true;
			// told the first time, a template missing is included many times
			if (m_missing.insert(lowered(file.name)).second)
				m_output.problems.push_back(Parent == static_cast<std::size_t>(-1) ? "can't read " + file.name : "can't read " + file.name + ", included from " + m_output.files[Parent].name + " line " + std::to_string(Line));
			m_output.files.push_back(std::move(file));
			return;
		}
		frame top;
		top.file = index;
		top.text = std::move(*text);
		top.parameters = file.parameters;
		m_output.files.push_back(std::move(file));
		m_frames.push_back(std::move(top));
	}

	// the next word, an include met on the way followed; none at the end of the files
	std::optional<token> next()
	{
		auto result{raw()};
		while (result.has_value() && is(result->text, "include"))
		{
			auto const name{raw()};
			if (false == name.has_value())
				break;
			std::vector<std::string> parameters;
			for (auto parameter{raw()}; parameter.has_value() && false == is(parameter->text, "end"); parameter = raw())
				parameters.push_back(parameter->text);
			open(name->text, result->file, result->line, std::move(parameters));
			result = raw();
		}
		return result;
	}

	bool in_map(std::size_t const File) const { return m_output.files[File].map; }

	// the placement directives, as state_serializer keeps them: the offsets of origin summed up, the scales multiplied, the rotation set
	void origin(glm::dvec3 const &Offset) { m_offsets.push_back(Offset + (m_offsets.empty() ? glm::dvec3{0.0} : m_offsets.back())); }
	void end_origin()
	{
		if (false == m_offsets.empty())
			m_offsets.pop_back();
	}
	void scale(glm::vec3 const &Factor)
	{
		auto const factor{Factor.x <= 0.f || Factor.y <= 0.f || Factor.z <= 0.f ? glm::vec3{1.f} : Factor};
		m_scales.push_back(factor * (m_scales.empty() ? glm::vec3{1.f} : m_scales.back()));
	}
	void end_scale()
	{
		if (false == m_scales.empty())
			m_scales.pop_back();
	}
	void rotate(glm::vec3 const &Rotation) { m_rotation = Rotation; }

  private:
	// a file being read, its words taken one by one as they're needed
	struct frame
	{
		std::size_t file{0};
		std::string text;
		std::size_t at{0};
		std::size_t line{1};
		std::vector<std::string> parameters;
	};

	scene::layer_context context() const
	{
		scene::layer_context result;
		result.offset = m_offsets.empty() ? glm::dvec3{0.0} : m_offsets.back();
		result.rotation = m_rotation;
		result.scale = m_scales.empty() ? glm::vec3{1.f} : m_scales.back();
		return result;
	}

	std::optional<token> raw()
	{
		while (false == m_frames.empty())
		{
			auto &top{m_frames.back()};
			auto const span{next_word(top.text, top.at, top.line)};
			if (false == span.has_value())
			{
				m_frames.pop_back();
				continue;
			}
			token result;
			result.file = top.file;
			result.line = span->line;
			result.begin = span->begin;
			result.end = span->end;
			result.text = top.text.substr(span->begin, span->end - span->begin);
			substitute(result.text, top.parameters);
			return result;
		}
		return std::nullopt;
	}

	std::filesystem::path m_folder;
	read_options const &m_options;
	scenery &m_output;
	std::vector<frame> m_frames;
	std::set<std::string> m_missing;
	std::vector<glm::dvec3> m_offsets;
	std::vector<glm::vec3> m_scales;
	glm::vec3 m_rotation{0.f};
};

} // namespace

std::vector<token> split(std::string_view const Text)
{
	std::vector<token> result;
	std::size_t line{1};
	std::size_t at{0};
	while (auto const span{next_word(Text, at, line)})
	{
		token word;
		word.line = span->line;
		word.begin = span->begin;
		word.end = span->end;
		word.text = std::string{Text.substr(span->begin, span->end - span->begin)};
		result.push_back(std::move(word));
	}
	return result;
}

std::vector<std::string> includes_in(std::string_view const Text)
{
	// the word include wherever it stands, the comments not told apart: a few more files looked at, none of the included missed, and no
	// word of the many of the terrain made
	std::vector<std::string> result;
	std::string_view constexpr keyword{"include"};
	for (std::size_t at = 0; at + keyword.size() < Text.size(); ++at)
	{
		if ((Text[at] | 0x20) != 'i' || (at > 0 && false == separator(Text[at - 1])) || false == separator(Text[at + keyword.size()]) || false == is(Text.substr(at, keyword.size()), keyword))
			continue;
		auto begin{at + keyword.size()};
		while (begin < Text.size() && separator(Text[begin]))
			++begin;
		auto end{begin};
		while (end < Text.size() && false == separator(Text[end]))
			++end;
		if (end > begin)
			result.emplace_back(Text.substr(begin, end - begin));
		at = end;
	}
	return result;
}

std::string lowered(std::string_view const Text)
{
	std::string result{Text};
	for (auto &c : result)
		if (c >= 'A' && c <= 'Z')
			c = static_cast<char>(c - 'A' + 'a');
	return result;
}

std::string joined(std::vector<token> const &Tokens, std::size_t const From, std::size_t const To)
{
	std::string result;
	for (auto i{From}; i < std::min(To, Tokens.size()); ++i)
	{
		if (false == result.empty())
			result += ' ';
		result += Tokens[i].text;
	}
	return result;
}

std::string_view node_end(std::string_view const Type)
{
	for (auto const &[type, end] : node_ends)
		if (type == Type)
			return end;
	return {};
}

std::string_view statement_end(std::string_view const Keyword)
{
	for (auto const &[keyword, end] : statement_ends)
		if (keyword == Keyword)
			return end;
	return {};
}

std::optional<std::string> node_text(std::string_view const Text, std::string_view const Name)
{
	auto const tokens{split(Text)};
	auto const name{lowered(Name)};
	for (std::size_t i = 0; i + 4 < tokens.size(); ++i)
	{
		if (lowered(tokens[i].text) != "node")
			continue;
		auto const end{node_end(lowered(tokens[i + 4].text))};
		if (end.empty())
			continue;
		auto last{i + 5};
		while (last < tokens.size() && lowered(tokens[last].text) != end)
			++last;
		if (lowered(tokens[i + 3].text) == name)
			return joined(tokens, i, last + 1);
		// the words of the node aren't nodes
		i = last;
	}
	return std::nullopt;
}

std::optional<std::string> file_text(std::filesystem::path const &Path)
{
	std::ifstream file{Path, std::ios::binary};
	if (false == file.is_open())
	{
		// named in another case, as the game finds it on Linux
		auto const cased{FileCased(Path)};
		if (cased.empty())
			return std::nullopt;
		file.open(cased, std::ios::binary);
		if (false == file.is_open())
			return std::nullopt;
	}
	return std::string{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

bool same_file(std::string_view const A, std::string_view const B)
{
	return normalized(A) == normalized(B);
}

bool scenery::uses_map() const
{
	return std::ranges::any_of(files, [](included_file const &File) { return File.map && false == File.missing; });
}

scenery read(std::filesystem::path const &Folder, std::string const &Root, read_options const &Options)
{
	scenery result;
	reader input{Folder, Options, result};
	input.open(Root, static_cast<std::size_t>(-1), 0, {});
	while (auto first{input.next()})
	{
		statement current;
		current.keyword = lowered(first->text);
		current.file = first->file;
		current.line = first->line;
		current.map = input.in_map(first->file);
		std::string type;
		std::string_view end;
		std::size_t count{word_count(current.keyword)};
		if (current.keyword == "node")
		{
			// the range, the name and the type, then the words up to the end of the type
			for (std::size_t i = 0; i < 4; ++i)
				if (auto word{input.next()})
					current.words.push_back(std::move(*word));
			type = current.word(3);
			end = node_end(type);
			count = 0;
		}
		else
			end = statement_end(current.keyword);
		for (std::size_t i = 0; i < count; ++i)
			if (auto word{input.next()})
				current.words.push_back(std::move(*word));
		// the words of a statement which isn't kept are only gone through, the ones of the terrain may be many
		auto const kept{false == static_cast<bool>(Options.keep) || Options.keep(current.keyword, type)};
		if (false == end.empty())
		{
			bool ended{false};
			while (auto word{input.next()})
			{
				auto const done{is(word->text, end)};
				if (kept)
					current.words.push_back(std::move(*word));
				if (done)
				{
					ended = true;
					break;
				}
			}
			if (false == ended)
				result.problems.push_back(result.files[current.file].name + " line " + std::to_string(current.line) + ": " + current.keyword + " " + type + " without " + std::string{end});
		}
		// the placement, for the includes after it
		auto const vector = [&]() { return glm::dvec3{current.words.size() > 0 ? real(current.words[0]) : 0.0, current.words.size() > 1 ? real(current.words[1]) : 0.0, current.words.size() > 2 ? real(current.words[2]) : 0.0}; };
		if (current.keyword == "origin")
			input.origin(vector());
		else if (current.keyword == "endorigin")
			input.end_origin();
		else if (current.keyword == "scale")
			input.scale(glm::vec3{vector()});
		else if (current.keyword == "endscale")
			input.end_scale();
		else if (current.keyword == "rotate")
			input.rotate(glm::vec3{vector()});
		if (kept)
			result.statements.push_back(std::move(current));
	}
	return result;
}

} // namespace scenery_text
