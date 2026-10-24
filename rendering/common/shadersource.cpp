/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/shadersource.h"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>

#include "utilities/Logs.h"

namespace gfx
{

namespace
{

bool ends_with(std::string_view Value, std::string_view Ending)
{
	return (Ending.size() <= Value.size()) && (Value.compare(Value.size() - Ending.size(), Ending.size(), Ending) == 0);
}

// the content of a file, empty with Found false when it can't be read
std::string read_text(std::string const &Path, bool &Found)
{
	std::ifstream file(Path, std::ios::binary);
	Found = file.is_open();
	if (false == Found)
	{
		return {};
	}
	std::stringstream stream;
	stream << file.rdbuf();
	return stream.str();
}

std::string read_file(std::string const &Path)
{
	bool found{false};
	auto text{read_text(Path, found)};
	if ((false == found) || text.empty())
	{
		throw shader_exception("cannot read shader: " + Path);
	}
	return text;
}

std::unordered_map<std::string, shader_components> const components_mapping = {
    {"R", shader_components::R}, {"RG", shader_components::RG}, {"RGB", shader_components::RGB}, {"RGBA", shader_components::RGBA}, {"sRGB", shader_components::sRGB}, {"sRGB_A", shader_components::sRGB_A}};

std::unordered_map<std::string, shader_param_default> const defaultparams_mapping = {
    {"required", shader_param_default::required}, {"nan", shader_param_default::nan},         {"zero", shader_param_default::zero},         {"one", shader_param_default::one},
    {"ambient", shader_param_default::ambient},   {"diffuse", shader_param_default::diffuse}, {"specular", shader_param_default::specular}, {"glossiness", shader_param_default::glossiness}};

void log_error(std::string const &Name, std::string const &Message)
{
	ErrorLog("bad shader: " + Name + ": " + Message, logtype::shader);
}

// the position of the next directive Magic ("texture", "param") of the materials: #pragma <magic>(...), or the older #<magic> (...)
std::size_t find_directive(std::string const &Text, std::string const &Magic, std::size_t const From, std::size_t &Arguments)
{
	auto start{From};
	while ((start = Text.find('#', start)) != std::string::npos)
	{
		auto position{start + 1};
		auto const skip_blanks = [&]() {
			while ((position < Text.size()) && ((Text[position] == ' ') || (Text[position] == '\t')))
			{
				++position;
			}
		};
		skip_blanks();
		if (Text.compare(position, 6, "pragma") == 0)
		{
			position += 6;
			skip_blanks();
		}
		if ((Text.compare(position, Magic.size(), Magic) == 0) && (position + Magic.size() < Text.size()))
		{
			position += Magic.size();
			skip_blanks();
			if ((position < Text.size()) && (Text[position] == '('))
			{
				Arguments = position;
				return start;
			}
		}
		++start;
	}
	return std::string::npos;
}

// the arguments of a directive, between the brackets, split at the commas; the directive is taken out of the text
std::vector<std::string> take_directive(std::string &Text, std::size_t const Start, std::size_t const Arguments)
{
	std::vector<std::string> result;
	auto const end{Text.find(')', Arguments)};
	if (end == std::string::npos)
	{
		Text.erase(Start, 1);
		return result;
	}
	std::istringstream stream(Text.substr(Arguments + 1, end - Arguments - 1));
	std::string token;
	while (std::getline(stream, token, ','))
	{
		std::istringstream tokenstream(token);
		std::string value;
		tokenstream >> value;
		result.emplace_back(value);
	}
	Text.erase(Start, end - Start + 1);
	return result;
}

void parse_texture_entries(std::string &Text, shader_source &Source)
{
	std::size_t start{0}, arguments{0};
	while ((start = find_directive(Text, "texture", start, arguments)) != std::string::npos)
	{
		auto const args{take_directive(Text, start, arguments)};
		if (args.size() != 3)
		{
			log_error(Source.name, "invalid argument count to #pragma texture");
			continue;
		}
		shader_texture_entry conf{};
		try
		{
			conf.id = static_cast<std::size_t>(std::stoul(args[1]));
		}
		catch (std::exception const &)
		{
			log_error(Source.name, "invalid texture binding: " + args[1]);
			continue;
		}
		auto const components{components_mapping.find(args[2])};
		if (components == components_mapping.end())
		{
			log_error(Source.name, "unknown components: " + args[2]);
			continue;
		}
		conf.components = components->second;
		if (args[0].empty())
		{
			log_error(Source.name, "empty name");
		}
		else if (conf.id >= MAX_SHADER_TEXTURES)
		{
			log_error(Source.name, "invalid texture binding: " + std::to_string(conf.id));
		}
		else
		{
			Source.texture_conf.try_emplace(args[0], conf);
		}
	}
}

void parse_param_entries(std::string &Text, shader_source &Source)
{
	std::size_t start{0}, arguments{0};
	while ((start = find_directive(Text, "param", start, arguments)) != std::string::npos)
	{
		auto const args{take_directive(Text, start, arguments)};
		if (args.size() != 5)
		{
			log_error(Source.name, "invalid argument count to #pragma param");
			continue;
		}
		shader_param_entry conf{};
		try
		{
			conf.location = static_cast<std::size_t>(std::stoul(args[1]));
			conf.offset = static_cast<std::size_t>(std::stoul(args[2]));
			conf.size = static_cast<std::size_t>(std::stoul(args[3]));
		}
		catch (std::exception const &)
		{
			log_error(Source.name, "invalid param: " + args[0]);
			continue;
		}
		auto const defaultparam{defaultparams_mapping.find(args[4])};
		if (defaultparam == defaultparams_mapping.end())
		{
			log_error(Source.name, "unknown param default: " + args[4]);
			continue;
		}
		conf.defaultparam = defaultparam->second;
		if (args[0].empty())
		{
			log_error(Source.name, "empty name");
		}
		else if (conf.location >= MAX_SHADER_PARAMS)
		{
			log_error(Source.name, "invalid param binding: " + std::to_string(conf.location));
		}
		else if (conf.offset > 3)
		{
			log_error(Source.name, "invalid offset: " + std::to_string(conf.offset));
		}
		else if (conf.offset + conf.size > 4)
		{
			log_error(Source.name, "invalid size: " + std::to_string(conf.size));
		}
		else
		{
			Source.param_conf.try_emplace(args[0], conf);
		}
	}
}

std::size_t count_lines(std::string const &Text, std::size_t const End)
{
	return static_cast<std::size_t>(std::count(Text.begin(), Text.begin() + std::min(End, Text.size()), '\n'));
}

// the number of the file in the list of the files of the source, added when it isn't there
std::size_t file_number(shader_source &Source, std::string const &File)
{
	auto const lookup{std::find(Source.files.begin(), Source.files.end(), File)};
	if (lookup != Source.files.end())
	{
		return static_cast<std::size_t>(lookup - Source.files.begin());
	}
	Source.files.emplace_back(File);
	return Source.files.size() - 1;
}

// replaces the includes of Text (the content of the file number Number) with the content of the files they name, with #line
// directives which keep the numbers of the lines of each file in the messages of the compiler. an include is a line which
// starts with #include (blanks before it allowed)
void expand_includes(std::string &Text, std::size_t const Number, std::string const &Basedir, shader_source &Source, int const Depth)
{
	if (Depth > 32)
	{
		throw shader_exception("includes nested too deep in " + Source.name);
	}
	std::string result;
	result.reserve(Text.size());
	std::size_t linestart{0};
	std::size_t linenumber{1};
	while (linestart < Text.size())
	{
		auto lineend{Text.find('\n', linestart)};
		if (lineend == std::string::npos)
		{
			lineend = Text.size();
		}
		std::string_view const line{Text.data() + linestart, lineend - linestart};
		auto position{line.find_first_not_of(" \t")};
		bool included{false};
		if ((position != std::string_view::npos) && (line.compare(position, 8, "#include") == 0))
		{
			position = line.find_first_not_of(" \t", position + 8);
			if (position != std::string_view::npos)
			{
				auto const open{line[position]};
				auto const close{(open == '<') ? '>' : (open == '"') ? '"' : '\0'};
				auto const end{(close != '\0') ? line.find(close, position + 1) : std::string_view::npos};
				if (end != std::string_view::npos)
				{
					std::string filename{line.substr(position + 1, end - position - 1)};
					if (filename == "common")
					{
						// the name the shaders used for the block of the game before it became a file
						filename = "common.glsl";
					}
					auto const number{file_number(Source, filename)};
					auto content{read_file(Basedir + filename)};
					expand_includes(content, number, Basedir, Source, Depth + 1);
					result += "#line 1 " + std::to_string(number) + "\n";
					result += content;
					if ((false == content.empty()) && (content.back() != '\n'))
					{
						result += '\n';
					}
					result += "#line " + std::to_string(linenumber + 1) + " " + std::to_string(Number);
					included = true;
				}
			}
		}
		if (false == included)
		{
			result.append(line);
		}
		if (lineend < Text.size())
		{
			result += '\n';
		}
		linestart = lineend + 1;
		++linenumber;
	}
	Text = std::move(result);
}

std::mutex manifest_mutex;

// the manifest of the binaries of a directory: binary -> (source file, hash)
struct manifest_data
{
	std::filesystem::file_time_type time{};
	bool found{false};
	std::unordered_map<std::string, std::vector<std::pair<std::string, std::uint64_t>>> entries;
};

std::unordered_map<std::string, manifest_data> manifests;

manifest_data const &manifest_of(std::string const &Directory)
{
	auto &manifest{manifests[Directory]};
	auto const path{Directory + "manifest.txt"};
	std::error_code error;
	auto const time{std::filesystem::last_write_time(path, error)};
	if (error)
	{
		manifest = manifest_data{};
		return manifest;
	}
	if (manifest.found && (manifest.time == time))
	{
		return manifest;
	}
	manifest = manifest_data{};
	manifest.found = true;
	manifest.time = time;
	std::ifstream file(path);
	std::string line;
	while (std::getline(file, line))
	{
		if (line.empty() || (line[0] == '#'))
		{
			continue;
		}
		std::istringstream stream(line);
		std::string binary, source, hash;
		if (stream >> binary >> source >> hash)
		{
			try
			{
				manifest.entries[binary].emplace_back(source, std::stoull(hash, nullptr, 16));
			}
			catch (std::exception const &)
			{
			}
		}
	}
	return manifest;
}

} // namespace

char const *shader_api_name(shader_api const Api)
{
	return (Api == shader_api::vulkan14 ? "vulkan 1.4" : "opengl 4.6");
}

std::string shader_source_dir(shader_api const Api)
{
	return (Api == shader_api::vulkan14 ? "shaders/vk14/" : "shaders/ogl46/");
}

std::string shader_binary_dir(shader_api const Api)
{
	return (Api == shader_api::vulkan14 ? "shaders/vk14bin/" : "shaders/ogl46bin/");
}

shader_stage shader_stage_of(std::string const &Name)
{
	if (ends_with(Name, ".vert"))
	{
		return shader_stage::vertex;
	}
	if (ends_with(Name, ".frag"))
	{
		return shader_stage::fragment;
	}
	if (ends_with(Name, ".geom"))
	{
		return shader_stage::geometry;
	}
	if (ends_with(Name, ".comp"))
	{
		return shader_stage::compute;
	}
	throw shader_exception("unknown shader " + Name);
}

char const *shader_stage_define(shader_stage const Stage)
{
	switch (Stage)
	{
	case shader_stage::vertex:
		return "STAGE_VERTEX";
	case shader_stage::fragment:
		return "STAGE_FRAGMENT";
	case shader_stage::geometry:
		return "STAGE_GEOMETRY";
	case shader_stage::compute:
		return "STAGE_COMPUTE";
	}
	return "STAGE_VERTEX";
}

std::string shader_binary_name(std::string const &Name, std::string const &Variant)
{
	return Name + (Variant.empty() ? "" : "." + Variant) + ".spv";
}

std::string shader_variant_of(std::string const &Defines, bool &Plain)
{
	Plain = true;
	std::string result;
	std::istringstream stream(Defines);
	std::string line;
	while (std::getline(stream, line))
	{
		std::istringstream linestream(line);
		std::string directive, name, value, rest;
		if (false == static_cast<bool>(linestream >> directive))
		{
			continue;
		}
		if ((directive != "#define") || (false == static_cast<bool>(linestream >> name)))
		{
			Plain = false;
			continue;
		}
		linestream >> value;
		if ((false == value.empty()) && (value != "1"))
		{
			Plain = false;
		}
		if (linestream >> rest)
		{
			Plain = false;
		}
		result += (result.empty() ? "" : ".") + name;
	}
	return result;
}

std::uint64_t shader_file_hash(std::string_view const Text)
{
	std::uint64_t hash{14695981039346656037ull};
	for (std::size_t i = 0; i < Text.size(); ++i)
	{
		auto const c{Text[i]};
		if ((c == '\r') && (i + 1 < Text.size()) && (Text[i + 1] == '\n'))
		{
			continue;
		}
		hash ^= static_cast<std::uint8_t>(c);
		hash *= 1099511628211ull;
	}
	return hash;
}

shader_source preprocess_shader(shader_api const Api, std::string const &Name, std::string const &Defines, std::string const &Preamble, bool const Allowmissing, std::string const &Basedir)
{
	shader_source source;
	source.api = Api;
	source.name = Name;
	source.stage = shader_stage_of(Name);
	bool plain{true};
	source.variant = shader_variant_of(Defines, plain);
	source.has_binary_name = plain;

	auto const basedir{Basedir.empty() ? shader_source_dir(Api) : Basedir};
	bool found{false};
	auto text{read_text(basedir + Name, found)};
	if ((false == found) || text.empty())
	{
		if (Allowmissing)
		{
			source.files.emplace_back(Name);
			return source;
		}
		throw shader_exception("cannot read shader: " + basedir + Name);
	}
	source.has_source = true;
	source.files.emplace_back(Name);

	// what goes after #version: the variant, the renderer, the stage where the file doesn't say it
	std::string preamble{Defines};
	if ((false == preamble.empty()) && (preamble.back() != '\n'))
	{
		preamble += '\n';
	}
	preamble += Preamble;
	if ((false == preamble.empty()) && (preamble.back() != '\n'))
	{
		preamble += '\n';
	}
	if (text.find(std::string("#define ") + shader_stage_define(source.stage)) == std::string::npos)
	{
		preamble += std::string("#define ") + shader_stage_define(source.stage) + " 1\n";
	}

	// the #version line, which has to stay first
	std::size_t versionend{std::string::npos};
	{
		auto version{text.find("#version")};
		if ((version != std::string::npos) && ((version == 0) || (text[version - 1] == '\n')))
		{
			versionend = text.find('\n', version);
		}
	}
	expand_includes(text, 0, basedir, source, 0);
	parse_texture_entries(text, source);
	parse_param_entries(text, source);

	if (versionend != std::string::npos)
	{
		// the includes come after the #version line, which is where it was
		versionend = text.find('\n', text.find("#version"));
		auto const line{count_lines(text, versionend) + 2};
		text.insert(versionend + 1, preamble + "#line " + std::to_string(line) + " 0\n");
	}
	else
	{
		text = "#version 460 core\n" + preamble + "#line 1 0\n" + text;
	}
	source.text = std::move(text);
	return source;
}

std::vector<std::uint32_t> load_shader_binary(shader_source const &Source, std::string &Reason)
{
	Reason.clear();
	if (false == Source.has_binary_name)
	{
		Reason = "the variant isn't one of plain defines";
		return {};
	}
	auto const directory{shader_binary_dir(Source.api)};
	auto const name{shader_binary_name(Source.name, Source.variant)};
	bool found{false};
	auto const data{read_text(directory + name, found)};
	if (false == found)
	{
		return {};
	}
	if ((data.size() < 20) || (data.size() % 4 != 0))
	{
		Reason = name + " isn't SPIR-V";
		return {};
	}
	std::vector<std::uint32_t> words(data.size() / 4);
	std::memcpy(words.data(), data.data(), data.size());
	if (words[0] != 0x07230203u)
	{
		Reason = name + " isn't SPIR-V";
		return {};
	}
	// the sources the binary was compiled from, as the manifest has them: a binary older than its sources isn't taken
	std::lock_guard<std::mutex> lock(manifest_mutex);
	auto const &manifest{manifest_of(directory)};
	auto const entry{manifest.entries.find(name)};
	if (entry != manifest.entries.end())
	{
		auto const sourcedir{shader_source_dir(Source.api)};
		for (auto const &[file, hash] : entry->second)
		{
			bool sourcefound{false};
			auto const text{read_text(sourcedir + file, sourcefound)};
			if (false == sourcefound)
			{
				// the sources aren't there, so they can't be newer
				continue;
			}
			if (shader_file_hash(text) != hash)
			{
				Reason = name + " was compiled from another " + file + " than the one in " + sourcedir;
				return {};
			}
		}
	}
	return words;
}

std::vector<std::uint32_t> spirv_specialization_ids(std::vector<std::uint32_t> const &Words)
{
	std::vector<std::uint32_t> result;
	// the header is 5 words; each instruction starts with its word count (high 16 bits) and opcode (low 16 bits)
	std::size_t index{5};
	while (index < Words.size())
	{
		auto const count{Words[index] >> 16};
		auto const opcode{Words[index] & 0xffffu};
		if ((count == 0) || (index + count > Words.size()))
		{
			break;
		}
		// OpDecorate %target SpecId <id>
		if ((opcode == 71) && (count >= 4) && (Words[index + 2] == 1))
		{
			result.push_back(Words[index + 3]);
		}
		index += count;
	}
	return result;
}

std::unordered_map<std::string, std::string> shader_file_defines(shader_api const Api, std::string const &Name, std::string const &Basedir)
{
	std::unordered_map<std::string, std::string> result;
	bool found{false};
	auto const text{read_text((Basedir.empty() ? shader_source_dir(Api) : Basedir) + Name, found)};
	std::istringstream stream(text);
	std::string line;
	while (std::getline(stream, line))
	{
		std::istringstream linestream(line);
		std::string directive, name, value;
		if ((linestream >> directive >> name) && (directive == "#define"))
		{
			std::getline(linestream, value);
			auto const first{value.find_first_not_of(" \t")};
			auto const last{value.find_last_not_of(" \t\r")};
			result[name] = (first == std::string::npos ? std::string{} : value.substr(first, last - first + 1));
		}
	}
	return result;
}

bool shader_constants_match(shader_api const Api, std::string const &Name, std::string const &Preamble, std::string const &Basedir, std::vector<std::string> &Differences)
{
	Differences.clear();
	auto const file{shader_file_defines(Api, Name, Basedir)};
	if (file.empty())
	{
		Differences.emplace_back(Name + " can't be read");
		return false;
	}
	// the same number however it's written: 0.2 and 0.200000, 3 and 3U
	auto const normalized = [](std::string Value) {
		if ((false == Value.empty()) && ((Value.back() == 'U') || (Value.back() == 'u')))
		{
			Value.pop_back();
		}
		try
		{
			std::size_t used{0};
			auto const number{std::stod(Value, &used)};
			if (used == Value.size())
			{
				return std::to_string(number);
			}
		}
		catch (std::exception const &)
		{
		}
		return Value;
	};
	std::istringstream stream(Preamble);
	std::string line;
	while (std::getline(stream, line))
	{
		std::istringstream linestream(line);
		std::string directive, name, value;
		if (!(linestream >> directive >> name >> value) || (directive != "#define"))
		{
			continue;
		}
		auto const lookup{file.find(name)};
		if (lookup == file.end())
		{
			Differences.emplace_back(name + " is missing");
		}
		else if (normalized(lookup->second) != normalized(value))
		{
			Differences.emplace_back(name + " is " + lookup->second + " in the file and " + value + " in the game");
		}
	}
	return Differences.empty();
}

} // namespace gfx
