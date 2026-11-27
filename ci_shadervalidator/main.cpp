#include "stdafx.h"
#include "../gl/glsl_common.h"
#include "rendering/common/shadersource.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string_view>
#include <unordered_set>
#include <vector>
#ifdef _WIN32
#include <random>
#else
#include <unistd.h>
#endif

_global Global;

namespace
{

std::string const ogl46_dir{"../shaders/ogl46/"};
std::string const vk14_dir{"../shaders/vk14/"};
std::string const vk14_bindings_header{"../rendering/vulkan14/vulkan14bindings.h"};
// the vulkan environment glslang is asked for: 1.4 where it knows it, or the one given with --vulkan-env
std::string vulkan_env{"vulkan1.4"};

// writes the text to a new file with a unique name in the temporary directory of the system and returns its path, or an empty
// string when it cannot be written. the file is created exclusively, so nothing put in a shared /tmp under a name known in
// advance (a file or a link) is written through
std::string write_temp_file(std::string_view const Text)
{
	std::string path{(std::filesystem::temp_directory_path() / "maszyna-shader-XXXXXX").string()};
	std::FILE *file{nullptr};
#ifdef _WIN32
	// no mkstemp() in the CRT; a random name, opened with "x", which fails when the file is there already
	std::random_device random;
	for (int attempt = 0; (attempt < 100) && (file == nullptr); ++attempt)
	{
		path.replace(path.size() - 6, 6, std::to_string(100000 + random() % 900000));
		if (fopen_s(&file, path.c_str(), "wx") != 0)
			file = nullptr;
	}
#else
	if (int const descriptor{mkstemp(path.data())}; descriptor != -1)
	{
		file = fdopen(descriptor, "w");
		if (file == nullptr)
		{
			close(descriptor);
			std::remove(path.c_str());
		}
	}
#endif
	if (file == nullptr)
		return {};

	bool const written{std::fwrite(Text.data(), 1, Text.size(), file) == Text.size()};
	if ((std::fclose(file) != 0) || !written)
	{
		std::remove(path.c_str());
		return {};
	}
	return path;
}

// a file of the validator in the temporary directory, removed when it goes. glslangValidator writes its output and the shell
// its log into files made this way, so they too are made exclusively beforehand
class temp_file
{
  public:
	explicit temp_file(std::string_view const Text) : m_path{write_temp_file(Text)} {}
	~temp_file()
	{
		if (false == m_path.empty())
			std::remove(m_path.c_str());
	}
	temp_file(temp_file const &) = delete;
	temp_file &operator=(temp_file const &) = delete;

	std::string const &path() const
	{
		return m_path;
	}

  private:
	std::string m_path;
};

// runs glslangValidator with the arguments, what it writes into a log file. false when the file can't be made; Result is the
// exit code, Log what it wrote
bool run_glslang(std::string_view const Arguments, int &Result, std::string &Log)
{
	temp_file const log{std::string_view{}};
	if (log.path().empty())
	{
		std::cerr << "cannot make a file in the temporary directory" << std::endl;
		return false;
	}
	auto const command{std::format("glslangValidator {} > \"{}\" 2>&1", Arguments, log.path())};
	Result = system(command.c_str());
	std::ifstream file(log.path());
	std::ostringstream text;
	text << file.rdbuf();
	Log = text.str();
	return true;
}

char const *stage_name(gfx::shader_stage const Stage)
{
	switch (Stage)
	{
	case gfx::shader_stage::vertex:
		return "vert";
	case gfx::shader_stage::fragment:
		return "frag";
	case gfx::shader_stage::geometry:
		return "geom";
	case gfx::shader_stage::compute:
		return "comp";
	}
	return "vert";
}

// Mode: the options of glslangValidator (empty: glsl of opengl, compiled from the source as the game does)
bool check_text(gfx::shader_source const &Source, std::string const &Mode)
{
	if (ErrorLogged)
	{
		ErrorLogged = false;
		return false;
	}
	temp_file const source{Source.text};
	temp_file const output{std::string_view{}};
	if (source.path().empty() || output.path().empty())
	{
		std::cerr << "cannot write the shader to the temporary directory" << std::endl;
		return false;
	}

	// the output only when it fails (glslangValidator names the file it compiled to spir-v)
	std::string arguments{std::format("{} -S {} \"{}\"", Mode, stage_name(Source.stage), source.path())};
	if (false == Mode.empty())
		arguments += std::format(" -o \"{}\"", output.path());
	int ret{-1};
	std::string log;
	if (false == run_glslang(arguments, ret, log))
		return false;
	if (ret != 0)
	{
		std::cerr << std::endl << log;
		std::cerr << "(source strings:";
		for (std::size_t i = 0; i < Source.files.size(); ++i)
			std::cerr << " " << i << ": " << Source.files[i];
		std::cerr << ")" << std::endl;
	}
	return ret == 0;
}

void set_features(bool const Enabled)
{
	Global.gfx_shadowmap_enabled = Enabled;
	Global.gfx_envmap_enabled = Enabled;
	Global.gfx_postfx_motionblur_enabled = Enabled;
	Global.gfx_skippipeline = !Enabled;
	Global.gfx_extraeffects = Enabled;
	Global.gfx_shadergamma = Enabled;
}

// a shader of the opengl renderer: compiled from the source as the game does it (with the features on and off, and the variant
// of the multi draws), and compiled to SPIR-V for opengl from the file as it is, as glslc makes the binaries of shaders/ogl46bin
bool check_ogl46(std::string const &Name)
{
	bool suc = true;
	auto const stage{gfx::shader_stage_of(Name)};
	auto const source = [&](std::string const &Defines, std::string const &Preamble) { return gfx::preprocess_shader(gfx::shader_api::opengl46, Name, Defines, Preamble, false, ogl46_dir); };

	std::cout << "features on..." << std::flush;
	set_features(true);
	if (!check_text(source({}, gl::glsl_common + gl::glsl_settings_defines()), {}))
		suc = false;

	std::cout << "done, features off..." << std::flush;
	set_features(false);
	if (!check_text(source({}, gl::glsl_common + gl::glsl_settings_defines()), {}))
		suc = false;

	// the variant of the vertex and fragment stages the multi draws of the instances use (model blocks by gl_DrawID)
	bool const variants{(stage == gfx::shader_stage::vertex) || (stage == gfx::shader_stage::fragment)};
	if (variants)
	{
		std::cout << "done, model draws..." << std::flush;
		set_features(true);
		if (!check_text(source("#define MODEL_DRAWS 1\n", gl::glsl_common + gl::glsl_settings_defines()), {}))
			suc = false;
	}

	std::cout << "done, spir-v..." << std::flush;
	if (!check_text(source({}, {}), "-G"))
		suc = false;
	if (variants && !check_text(source("#define MODEL_DRAWS 1\n", {}), "-G"))
		suc = false;

	std::cout << "done" << std::flush;
	return suc;
}

// a shader of the vulkan renderer, compiled to SPIR-V for vulkan as the game does it (and as glslc makes the binaries of
// shaders/vk14bin)
bool check_vk14(std::string const &Name)
{
	std::cout << "spir-v..." << std::flush;
	auto const source{gfx::preprocess_shader(gfx::shader_api::vulkan14, Name, {}, {}, false, vk14_dir)};
	auto suc{check_text(source, "-V --target-env " + vulkan_env)};
	// the variant of the passes without the velocity target, of the shaders which have one
	if (source.text.find("NO_MOTION_OUTPUT") != std::string::npos)
	{
		std::cout << "done, no motion output..." << std::flush;
		if (!check_text(gfx::preprocess_shader(gfx::shader_api::vulkan14, Name, "#define NO_MOTION_OUTPUT 1\n", {}, false, vk14_dir), "-V --target-env " + vulkan_env))
			suc = false;
	}
	// the ray traced reflections (gfx.raytracing): the renderer puts the extensions and REFLECTION_RAYS before the sources
	// (vk14::binding_preamble()), the shaders which shade with light_common.glsl trace the rays
	if (source.text.find("REFLECTION_RAYS_ACTIVE") != std::string::npos)
	{
		std::cout << "done, ray traced reflections..." << std::flush;
		std::string const raytracing{
		    "#extension GL_EXT_ray_query : require\n#extension GL_EXT_buffer_reference2 : require\n#extension GL_EXT_buffer_reference_uvec2 : require\n"
		    "#extension GL_EXT_nonuniform_qualifier : require\n#define REFLECTION_RAYS 1\n"};
		if (!check_text(gfx::preprocess_shader(gfx::shader_api::vulkan14, Name, raytracing, {}, false, vk14_dir), "-V --target-env " + vulkan_env))
			suc = false;
	}
	std::cout << "done" << std::flush;
	return suc;
}

bool is_stage(std::filesystem::path const &Path)
{
	auto const ext = Path.extension();
	return (ext == ".frag" || ext == ".geom" || ext == ".vert" || ext == ".comp");
}

// whether glslangValidator compiles a shader for the vulkan environment
bool knows_vulkan_env(std::string_view const Env)
{
	temp_file const source{"#version 460\nvoid main() { gl_Position = vec4(0.0); }\n"};
	temp_file const output{std::string_view{}};
	if (source.path().empty() || output.path().empty())
		return false;
	int ret{-1};
	std::string log;
	return run_glslang(std::format("-V --target-env {} -S vert \"{}\" -o \"{}\"", Env, source.path(), output.path()), ret, log) && (ret == 0);
}

// the bindings of rendering/vulkan14/vulkan14bindings.h as the #define lines vk14::binding_preamble() makes of them, read from
// the header as it is (the code of the renderer needs vulkan and glslang, which the validator is built without). Names: all
// the constants of the header
std::string vk14_binding_defines(std::unordered_set<std::string> &Names)
{
	std::ifstream file(vk14_bindings_header);
	std::regex const constant{R"(^\s*constexpr\s+std::uint32_t\s+(\w+)\s*=\s*(\d+)\s*;)"};
	std::string defines;
	std::string line;
	while (std::getline(file, line))
	{
		std::smatch match;
		if (false == std::regex_search(line, match, constant))
			continue;
		Names.insert(match[1].str());
		// the number of the bindings with dynamic offsets is the renderer's own, the shaders aren't given it
		if (match[1].str() == "PASS_DYNAMIC_OFFSETS")
			continue;
		defines += std::format("#define {} {}\n", match[1].str(), match[2].str());
	}
	return defines;
}

// the constants of the game the vulkan renderer gives the shaders as the opengl one does (gl::glsl_common), those of them which
// shaders/vk14/common.glsl writes too; the bindings, which differ between the renderers, left out
std::string vk14_shared_defines(std::unordered_set<std::string> const &Bindings)
{
	auto const file{gfx::shader_file_defines(gfx::shader_api::vulkan14, "common.glsl", vk14_dir)};
	std::istringstream stream(gl::glsl_common);
	std::string defines;
	std::string line;
	while (std::getline(stream, line))
	{
		std::istringstream linestream(line);
		std::string directive;
		std::string name;
		if (!(linestream >> directive >> name) || (directive != "#define") || Bindings.contains(name) || !file.contains(name))
			continue;
		defines += line + '\n';
	}
	return defines;
}

// the constants written in common.glsl of a directory compared with those of the game: the binaries are compiled with the
// constants of the file, the game compiles the sources with its own
bool check_constants(std::string_view const Label, gfx::shader_api const Api, std::string const &Directory, std::string const &Preamble)
{
	std::vector<std::string> differences;
	std::cout << "== constants of " << Label << ": " << std::flush;
	if (gfx::shader_constants_match(Api, "common.glsl", Preamble, Directory, differences))
	{
		std::cout << "the same as in the game" << std::endl;
		return true;
	}
	std::cout << "not the same as in the game:" << std::endl;
	for (auto const &difference : differences)
		std::cout << "  " << difference << std::endl;
	return false;
}

// the vulkan environment from the arguments (--vulkan-env <env>), empty without it. false on arguments it doesn't know
bool parse_arguments(std::vector<std::string_view> const &Arguments, std::string &Required)
{
	if (Arguments.empty())
		return true;
	if ((Arguments.size() != 2) || (Arguments[0] != "--vulkan-env"))
		return false;
	Required = Arguments[1];
	return true;
}

} // namespace

// --vulkan-env <env>: the vulkan environment to check the shaders of shaders/vk14 against, which glslangValidator has to know;
// without it vulkan1.4, or vulkan1.3 (said at the start and at the end) where glslangValidator doesn't know 1.4
int main(int argc, char *argv[])
{
	std::string required;
	if (false == parse_arguments(std::vector<std::string_view>(argv + 1, argv + argc), required))
	{
		std::cerr << "usage: validateshaders [--vulkan-env <env>]" << std::endl;
		return 1;
	}

	int ret{-1};
	std::string version;
	if (!run_glslang("--version", ret, version) || (ret != 0))
	{
		std::cerr << "glslangValidator not found in PATH" << std::endl;
		return 1;
	}
	std::cout << "== " << version.substr(0, version.find('\n')) << std::endl;
	bool fallback{false};
	if (false == required.empty())
	{
		if (false == knows_vulkan_env(required))
		{
			std::cerr << "glslangValidator doesn't know the vulkan environment " << required << std::endl;
			return 1;
		}
		vulkan_env = required;
	}
	else if (false == knows_vulkan_env(vulkan_env))
	{
		vulkan_env = "vulkan1.3";
		fallback = true;
		std::cout << "== WARNING: glslangValidator doesn't know vulkan1.4, the vulkan shaders are checked against vulkan1.3 only" << std::endl;
	}
	std::cout << "== the vulkan shaders are checked against " << vulkan_env << std::endl;

	bool suc = true;
	std::vector<std::string> failed;

	gl::glsl_common_setup();
	if (false == check_constants("shaders/ogl46/common.glsl", gfx::shader_api::opengl46, ogl46_dir, gl::glsl_common))
	{
		suc = false;
		failed.emplace_back("ogl46/common.glsl");
	}
	std::unordered_set<std::string> bindings;
	if (auto const bindingdefines{vk14_binding_defines(bindings)}; bindingdefines.empty())
	{
		std::cout << "== constants of shaders/vk14/common.glsl: " << vk14_bindings_header << " can't be read" << std::endl;
		suc = false;
		failed.emplace_back("vk14/common.glsl");
	}
	else if (false == check_constants("shaders/vk14/common.glsl", gfx::shader_api::vulkan14, vk14_dir, bindingdefines + vk14_shared_defines(bindings)))
	{
		suc = false;
		failed.emplace_back("vk14/common.glsl");
	}

	for (auto const &[directory, api] : {std::pair{ogl46_dir, gfx::shader_api::opengl46}, std::pair{vk14_dir, gfx::shader_api::vulkan14}})
	{
		if (false == std::filesystem::is_directory(directory))
		{
			std::cout << "== the directory " << directory << " isn't there" << std::endl;
			suc = false;
			failed.push_back(directory);
			continue;
		}
		std::vector<std::string> names;
		for (auto &f : std::filesystem::directory_iterator(directory))
			if (f.is_regular_file() && is_stage(f.path()))
				names.push_back(f.path().filename().string());
		std::sort(names.begin(), names.end());
		for (auto const &name : names)
		{
			auto const label{std::string(api == gfx::shader_api::opengl46 ? "ogl46/" : "vk14/") + name};
			std::cout << "== validating shader " << label << ": " << std::flush;
			bool ok{false};
			try
			{
				ok = (api == gfx::shader_api::opengl46 ? check_ogl46(name) : check_vk14(name));
			}
			catch (std::exception const &e)
			{
				std::cerr << e.what() << std::endl;
			}
			std::cout << std::endl;
			if (!ok)
			{
				suc = false;
				failed.push_back(label);
			}
		}
	}

	if (fallback)
		std::cout << "==== the vulkan shaders were checked against vulkan1.3, not vulkan1.4" << std::endl;
	if (suc)
		std::cout << "==== all checks ok" << std::endl;
	else {
		std::cout << "==== checks failed:";
		for (auto const &name : failed)
			std::cout << " " << name;
		std::cout << std::endl;
		return 1;
	}

	return 0;
}
