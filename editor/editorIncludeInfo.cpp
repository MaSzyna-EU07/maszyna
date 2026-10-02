/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorIncludeInfo.hpp"

#include "utilities/Globals.h"
#include "utilities/utilities.h"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <random>
#include <sstream>
#include <tuple>
#include <unordered_map>

namespace
{

std::string const linemark{"//$e"};

// template file split into the description and the rest
struct template_text
{
	std::string content; // whole file
	std::string yaml; // description, with the line marks taken off; utf-8
	std::vector<std::pair<std::size_t, std::size_t>> lines; // ranges of the content taken by the description lines, line ends included
	std::string eol{"\r\n"};
	std::size_t start{0}; // location of the first character past the byte order mark
	bool utf8{false}; // the file is utf-8 encoded, rather than windows-1250 like most of the scenery files
};

// true if specified text is valid utf-8. HasExtended: set if the text has any characters outside of ascii
bool is_utf8(std::string const &Text, bool &HasExtended)
{
	HasExtended = false;
	for (std::size_t idx = 0; idx < Text.size();)
	{
		auto const lead{static_cast<unsigned char>(Text[idx])};
		if (lead < 0x80)
		{
			++idx;
			continue;
		}
		HasExtended = true;
		auto const length{(lead & 0xE0) == 0xC0 ? 2 : (lead & 0xF0) == 0xE0 ? 3 : (lead & 0xF8) == 0xF0 ? 4 : 0};
		if (length == 0 || idx + length > Text.size())
		{
			return false;
		}
		for (auto offset = 1; offset < length; ++offset)
		{
			if ((static_cast<unsigned char>(Text[idx + offset]) & 0xC0) != 0x80)
			{
				return false;
			}
		}
		idx += length;
	}
	return true;
}

// counterpart of win1250_to_utf8(). characters outside of the supported set are replaced with question marks
std::string utf8_to_win1250(std::string const &Text)
{
	// polish letters, as utf-8 sequences; the same set win1250_to_utf8() handles
	static std::unordered_map<std::string, char> const charmap{{"\xC4\x84", '\xA5'}, {"\xC4\x86", '\xC6'}, {"\xC4\x98", '\xCA'}, {"\xC5\x81", '\xA3'}, {"\xC5\x83", '\xD1'}, {"\xC3\x93", '\xD3'},
	                                                           {"\xC5\x9A", '\x8C'}, {"\xC5\xB9", '\x8F'}, {"\xC5\xBB", '\xAF'}, {"\xC4\x85", '\xB9'}, {"\xC4\x87", '\xE6'}, {"\xC4\x99", '\xEA'},
	                                                           {"\xC5\x82", '\xB3'}, {"\xC5\x84", '\xF1'}, {"\xC3\xB3", '\xF3'}, {"\xC5\x9B", '\x9C'}, {"\xC5\xBA", '\x9F'}, {"\xC5\xBC", '\xBF'}};
	std::string output;
	for (std::size_t idx = 0; idx < Text.size();)
	{
		auto const lead{static_cast<unsigned char>(Text[idx])};
		if (lead < 0x80)
		{
			output += Text[idx++];
			continue;
		}
		auto const length{std::min<std::size_t>((lead & 0xE0) == 0xC0 ? 2 : (lead & 0xF0) == 0xE0 ? 3 : (lead & 0xF8) == 0xF0 ? 4 : 1, Text.size() - idx)};
		auto const lookup{charmap.find(Text.substr(idx, length))};
		output += (lookup != charmap.end() ? lookup->second : '?');
		idx += length;
	}
	return output;
}

bool read_template(std::string const &File, template_text &Text, std::string &Error)
{
	std::ifstream input{Global.asCurrentSceneryPath + File, std::ios_base::binary};
	if (false == input.is_open())
	{
		Error = "can't open file \"" + File + "\"";
		return false;
	}
	auto &content{Text.content};
	content.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());

	auto const bom{content.compare(0, 3, "\xEF\xBB\xBF") == 0};
	Text.start = bom ? 3 : 0;
	bool extended;
	Text.utf8 = bom || (is_utf8(content, extended) && extended);
	if (auto const linebreak{content.find('\n')}; linebreak != std::string::npos)
	{
		Text.eol = (linebreak > 0 && content[linebreak - 1] == '\r') ? "\r\n" : "\n";
	}

	for (auto linebegin{Text.start}; linebegin < content.size();)
	{
		auto const linebreak{content.find('\n', linebegin)};
		auto const next{linebreak == std::string::npos ? content.size() : linebreak + 1};
		if (content.compare(linebegin, linemark.size(), linemark) == 0)
		{
			auto line{content.substr(linebegin + linemark.size(), next - linebegin - linemark.size())};
			line.erase(line.find_last_not_of("\r\n") + 1);
			// a single space separates the mark from the document; the rest is indentation, which carries meaning
			if (false == line.empty() && line.front() == ' ')
			{
				line.erase(0, 1);
			}
			Text.yaml += line + '\n';
			Text.lines.emplace_back(linebegin, next);
		}
		linebegin = next;
	}
	if (false == is_utf8(Text.yaml, extended))
	{
		Text.yaml = win1250_to_utf8(Text.yaml);
	}
	return true;
}

// number of the parameter if specified text holds (pN) placeholder at specified location, 0 otherwise. Length: size of the placeholder
int placeholder(std::string const &Text, std::size_t const Location, std::size_t &Length)
{
	if (Text.compare(Location, 2, "(p") != 0)
	{
		return 0;
	}
	auto idx{Location + 2};
	auto id{0};
	while (idx < Text.size() && std::isdigit(static_cast<unsigned char>(Text[idx])) && id < 1000)
	{
		id = id * 10 + (Text[idx] - '0');
		++idx;
	}
	if (idx == Location + 2 || idx >= Text.size() || Text[idx] != ')')
	{
		return 0;
	}
	Length = idx + 1 - Location;
	return id;
}

// number of the parameter if the token is nothing but its placeholder, 0 otherwise
int exact_placeholder(std::string const &Token)
{
	std::size_t length{0};
	auto const id{placeholder(Token, 0, length)};
	return length == Token.size() ? id : 0;
}

// number of the first parameter whose placeholder is a part of the token, 0 if there's none
int first_placeholder(std::string const &Token)
{
	std::size_t length{0};
	for (auto location{Token.find("(p")}; location != std::string::npos; location = Token.find("(p", location + 1))
	{
		if (auto const id{placeholder(Token, location, length)}; id != 0)
		{
			return id;
		}
	}
	return 0;
}

// tokens of scenery text, in lower case and with the comments left out
std::vector<std::string> tokenize(std::string const &Text)
{
	std::vector<std::string> tokens;
	std::string token;
	auto const flush = [&]() {
		if (false == token.empty())
		{
			tokens.emplace_back(ToLower(token));
			token.clear();
		}
	};
	for (std::size_t idx = 0; idx < Text.size(); ++idx)
	{
		if (Text.compare(idx, 2, "//") == 0)
		{
			flush();
			idx = Text.find('\n', idx);
			if (idx == std::string::npos)
			{
				break;
			}
		}
		else if (Text.compare(idx, 2, "/*") == 0)
		{
			flush();
			idx = Text.find("*/", idx + 2);
			if (idx == std::string::npos)
			{
				break;
			}
			++idx;
		}
		else if (Text[idx] == ' ' || Text[idx] == '\t' || Text[idx] == '\n' || Text[idx] == '\r' || Text[idx] == ';')
		{
			flush();
		}
		else
		{
			token += Text[idx];
		}
	}
	flush();
	return tokens;
}

// reads description held by specified template text
bool parse(template_text const &Text, include_info &Info, std::string &Error)
{
	Info = include_info();
	auto const &text{Text};
	try
	{
		auto const document{YAML::Load(text.yaml)};
		if (document.IsNull())
		{
			return true;
		}
		if (false == document.IsMap())
		{
			Error = "the description isn't a list of named values";
			return false;
		}
		Info.name = document["name"].as<std::string>("");
		Info.category = document["category"].as<std::string>("");
		Info.description = document["description"].as<std::string>("");
		if (auto const parameters{document["params"]}; parameters.IsDefined() && parameters.IsSequence())
		{
			for (auto const &entry : parameters)
			{
				auto const id{entry.IsMap() ? entry["id"].as<int>(0) : 0};
				if (id <= 0)
				{
					continue;
				}
				auto &parameter{Info.parameter(id)};
				parameter.role = entry["role"].as<std::string>("free");
				parameter.label = entry["label"].as<std::string>("");
				parameter.value = entry["default"].as<std::string>("");
			}
		}
	}
	catch (YAML::Exception const &exception)
	{
		Error = std::string{"the description is malformed: "} + exception.what();
		return false;
	}
	return true;
}

// highest number of (pN) placeholder used by specified template text
int count_parameters(template_text const &Text)
{
	// the description lines may mention parameters the template doesn't use
	auto content{Text.content};
	for (auto line{Text.lines.rbegin()}; line != Text.lines.rend(); ++line)
	{
		content.erase(line->first, line->second - line->first);
	}
	auto count{0};
	std::size_t length{0};
	for (auto location{content.find("(p")}; location != std::string::npos; location = content.find("(p", location + 1))
	{
		count = std::max(count, placeholder(content, location, length));
	}
	return count;
}

} // namespace

include_parameter &include_info::parameter(int const Id)
{
	auto lookup{std::find_if(std::begin(parameters), std::end(parameters), [=](include_parameter const &Parameter) { return Parameter.id >= Id; })};
	if (lookup == std::end(parameters) || lookup->id != Id)
	{
		lookup = parameters.emplace(lookup);
		lookup->id = Id;
	}
	return *lookup;
}

namespace editor_includes
{

std::vector<std::string> const roles{"free", "name", "pos.x", "pos.y", "pos.z", "rot.x", "rot.y", "rot.z", "track", "memcell", "event", "model", "texture", "number", "text"};

bool load(std::string const &File, include_info &Info, std::string &Error)
{
	template_text text;
	return read_template(File, text, Error) && parse(text, Info, Error);
}

bool save(std::string const &File, include_info const &Info, std::string &Error)
{
	template_text text;
	if (false == read_template(File, text, Error))
	{
		return false;
	}
	std::string yaml;
	try
	{
		// start with the existing description, to keep the entries we don't know
		auto document{YAML::Load(text.yaml)};
		if (false == document.IsNull() && false == document.IsMap())
		{
			Error = "the existing description isn't a list of named values";
			return false;
		}
		if (false == document["version"].IsDefined())
		{
			document["version"] = 1;
		}
		auto const set = [&](char const *Key, std::string const &Value) {
			if (false == Value.empty())
			{
				document[Key] = Value;
			}
			else
			{
				document.remove(Key);
			}
		};
		set("name", Info.name);
		set("category", Info.category);
		set("description", Info.description);

		YAML::Node parameters{YAML::NodeType::Sequence};
		for (auto const &parameter : Info.parameters)
		{
			YAML::Node entry{YAML::NodeType::Map};
			if (auto const existing{document["params"]}; existing.IsDefined() && existing.IsSequence())
			{
				for (auto const &candidate : existing)
				{
					if (candidate.IsMap() && candidate["id"].as<int>(0) == parameter.id)
					{
						entry = YAML::Clone(candidate);
						break;
					}
				}
			}
			entry["id"] = parameter.id;
			entry["role"] = parameter.role.empty() ? std::string{"free"} : parameter.role;
			if (false == parameter.label.empty())
			{
				entry["label"] = parameter.label;
			}
			else
			{
				entry.remove("label");
			}
			if (false == parameter.value.empty())
			{
				entry["default"] = parameter.value;
			}
			else
			{
				entry.remove("default");
			}
			entry.SetStyle(YAML::EmitterStyle::Flow);
			parameters.push_back(entry);
		}
		if (parameters.size() > 0)
		{
			document["params"] = parameters;
		}
		else
		{
			document.remove("params");
		}

		YAML::Emitter emitter;
		emitter << document;
		yaml = emitter.c_str();
	}
	catch (YAML::Exception const &exception)
	{
		Error = std::string{"the existing description is malformed, fix or remove it first: "} + exception.what();
		return false;
	}
	if (false == text.utf8)
	{
		yaml = utf8_to_win1250(yaml);
	}

	std::string block;
	std::istringstream lines{yaml};
	for (std::string line; std::getline(lines, line);)
	{
		block += linemark + (line.empty() ? "" : " " + line) + text.eol;
	}
	// the description replaces the old one, or goes to the top of the file if there was none
	auto content{text.content};
	auto location{text.lines.empty() ? text.start : text.lines.front().first};
	for (auto line{text.lines.rbegin()}; line != text.lines.rend(); ++line)
	{
		content.erase(line->first, line->second - line->first);
	}
	content.insert(location, block);

	auto const filepath{Global.asCurrentSceneryPath + File};
	std::ofstream output{filepath + ".tmp", std::ios_base::binary | std::ios_base::trunc};
	output << content;
	output.close();
	std::error_code error;
	if (output.fail())
	{
		std::filesystem::remove(filepath + ".tmp", error);
		Error = "can't write file \"" + File + "\"";
		return false;
	}
	// the first save of a file leaves its original content next to it
	if (false == std::filesystem::exists(filepath + ".bak", error))
	{
		std::filesystem::copy_file(filepath, filepath + ".bak", error);
	}
	std::filesystem::rename(filepath + ".tmp", filepath, error);
	if (error)
	{
		Error = "can't replace file \"" + File + "\": " + error.message();
		std::filesystem::remove(filepath + ".tmp", error);
		return false;
	}
	return true;
}

int parameter_count(std::string const &File)
{
	template_text text;
	std::string error;
	return read_template(File, text, error) ? count_parameters(text) : 0;
}

void suggest(std::string const &File, include_info &Info)
{
	template_text text;
	std::string error;
	if (false == read_template(File, text, error))
	{
		return;
	}
	auto const tokens{tokenize(text.content)};
	auto const assign = [&](std::size_t const Index, char const *Role, bool const Exact = true) {
		if (Index >= tokens.size())
		{
			return;
		}
		auto const id{Exact ? exact_placeholder(tokens[Index]) : first_placeholder(tokens[Index])};
		if (id != 0 && Info.parameter(id).role == "free")
		{
			Info.parameter(id).role = Role;
		}
	};
	for (std::size_t idx = 0; idx < tokens.size(); ++idx)
	{
		auto const &token{tokens[idx]};
		if (token == "origin")
		{
			assign(idx + 1, "pos.x");
			assign(idx + 2, "pos.y");
			assign(idx + 3, "pos.z");
		}
		else if (token == "rotate")
		{
			assign(idx + 1, "rot.x");
			assign(idx + 2, "rot.y");
			assign(idx + 3, "rot.z");
		}
		else if (token == "node" && idx + 4 < tokens.size())
		{
			// node <max> <min> <name> <type> <x> <y> <z> ...
			assign(idx + 3, "name", false);
			auto const &type{tokens[idx + 4]};
			if (type == "model" || type == "memcell" || type == "eventlauncher" || type == "sound")
			{
				assign(idx + 5, "pos.x");
				assign(idx + 6, "pos.y");
				assign(idx + 7, "pos.z");
			}
			if (type == "model")
			{
				assign(idx + 8, "rot.y");
				assign(idx + 9, "model");
				assign(idx + 10, "texture");
			}
			if (type == "memcell")
			{
				assign(idx + 11, "track");
			}
		}
	}
}

std::string const directive_mark{"include "};

bool automatic(std::string const &Role)
{
	return Role == "name" || Role.starts_with("pos.") || Role.starts_with("rot.");
}

bool complete(include_info const &Info, int const Parameters, std::string *Issue)
{
	std::string issue;
	if (Info.name.empty())
	{
		issue = "the template needs a name";
	}
	for (auto id = 1; id <= Parameters && issue.empty(); ++id)
	{
		auto const lookup{std::find_if(std::begin(Info.parameters), std::end(Info.parameters), [=](include_parameter const &Parameter) { return Parameter.id == id; })};
		if (lookup == std::end(Info.parameters) || (false == automatic(lookup->role) && lookup->value.empty()))
		{
			issue = "(p" + std::to_string(id) + ") needs a default value, or a role filled in by the editor (name, pos.*, rot.*)";
		}
	}
	if (false == issue.empty() && Issue != nullptr)
	{
		*Issue = issue;
	}
	return issue.empty();
}

std::string directive(std::string const &File, include_info const &Info, int const Parameters, glm::dvec3 const &Location, std::optional<float> Yaw)
{
	auto const number = [](double const Value) {
		std::ostringstream converter;
		converter.imbue(std::locale::classic());
		converter << std::fixed << std::setprecision(3) << Value;
		return converter.str();
	};
	std::string text{"include " + File};
	for (auto id = 1; id <= Parameters; ++id)
	{
		auto const lookup{std::find_if(std::begin(Info.parameters), std::end(Info.parameters), [=](include_parameter const &Parameter) { return Parameter.id == id; })};
		auto const role{lookup != std::end(Info.parameters) ? lookup->role : std::string{"free"}};
		auto value{lookup != std::end(Info.parameters) ? lookup->value : std::string{}};
		if (role == "pos.x")
		{
			value = number(Location.x);
		}
		else if (role == "pos.y")
		{
			value = number(Location.y);
		}
		else if (role == "pos.z")
		{
			value = number(Location.z);
		}
		else if (role == "rot.y" && Yaw)
		{
			value = number(*Yaw);
		}
		else if (role.starts_with("rot.") && value.empty())
		{
			value = "0";
		}
		else if (role == "name")
		{
			// names have to be unique in the scenery. the default value, or the name of the template, makes them recognizable
			static std::mt19937 engine{std::random_device{}()};
			char suffix[16];
			std::snprintf(suffix, sizeof(suffix), "_%08x", static_cast<unsigned int>(engine()));
			value = (value.empty() ? std::filesystem::path(File).stem().string() : value) + suffix;
		}
		if (value.empty())
		{
			// a parameter can't be left out, as the ones after it would take its place
			value = "none";
		}
		text += ' ' + (value.find_first_of(" \t;") != std::string::npos ? '\"' + value + '\"' : value);
	}
	return text + " end";
}

} // namespace editor_includes

editorIncludeBank EditorIncludes;

namespace
{

include_entry examine(std::string const &File)
{
	include_entry entry;
	entry.file = File;
	entry.statement = std::make_shared<std::string>(editor_includes::directive_mark + File);
	template_text text;
	if (false == read_template(File, text, entry.issue))
	{
		return entry;
	}
	entry.described = false == text.lines.empty();
	if (false == entry.described)
	{
		entry.issue = "the template has no description";
		return entry;
	}
	include_info info;
	if (false == parse(text, info, entry.issue))
	{
		return entry;
	}
	entry.name = info.name;
	entry.category = info.category;
	entry.complete = editor_includes::complete(info, count_parameters(text), &entry.issue);
	return entry;
}

} // namespace

void editorIncludeBank::scan()
{
	// the node bank may hold one of the templates selected, the statements stay in place across the scans
	std::unordered_map<std::string, std::shared_ptr<std::string>> statements;
	for (auto &entry : m_entries)
	{
		statements.emplace(entry.file, entry.statement);
	}
	m_entries.clear();
	m_scanned = true;
	std::error_code error;
	auto const root{std::filesystem::path(Global.asCurrentSceneryPath)};
	// NOTE: on an error the iterator turns into the end one
	for (std::filesystem::recursive_directory_iterator file{root, std::filesystem::directory_options::skip_permission_denied, error}, end; file != end; file.increment(error))
	{
		if (false == file->is_regular_file(error))
		{
			continue;
		}
		// the scenery refers to its files with lower case names, relative to the scenery directory
		std::string name;
		try
		{
			name = ToLower(file->path().lexically_relative(root).generic_string());
		}
		catch (std::exception const &)
		{
			// the name can't be expressed in the encoding the scenery files use, so nothing can include the file anyway
			continue;
		}
		if (false == name.ends_with(".inc"))
		{
			continue;
		}
		m_entries.emplace_back(examine(name));
		if (auto const lookup{statements.find(name)}; lookup != statements.end())
		{
			m_entries.back().statement = lookup->second;
		}
	}
	std::sort(std::begin(m_entries), std::end(m_entries), [](include_entry const &Left, include_entry const &Right) { return Left.file < Right.file; });
	index();
}

void editorIncludeBank::index()
{
	m_ready.clear();
	for (std::size_t idx = 0; idx < m_entries.size(); ++idx)
	{
		if (m_entries[idx].complete)
		{
			m_ready.emplace_back(idx);
		}
	}
	std::sort(std::begin(m_ready), std::end(m_ready), [this](std::size_t const Left, std::size_t const Right) {
		auto const &left{m_entries[Left]};
		auto const &right{m_entries[Right]};
		return std::tie(left.category, left.name, left.file) < std::tie(right.category, right.name, right.file);
	});
	++m_revision;
}

void editorIncludeBank::update(std::string const &File)
{
	auto const lookup{std::find_if(std::begin(m_entries), std::end(m_entries), [&](include_entry const &Entry) { return Entry.file == File; })};
	if (lookup == std::end(m_entries))
	{
		m_entries.insert(std::upper_bound(std::begin(m_entries), std::end(m_entries), File, [](std::string const &Name, include_entry const &Entry) { return Name < Entry.file; }), examine(File));
	}
	else
	{
		// the node bank may hold the template selected, keep its statement
		auto const statement{lookup->statement};
		*lookup = examine(File);
		lookup->statement = statement;
	}
	index();
}
