#include "stdafx.h"
#include "../gl/glsl_common.h"
#include "rendering/common/shadersource.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <vector>

_global Global;

namespace
{

std::string const ogl46_dir{"../shaders/ogl46/"};
std::string const vk14_dir{"../shaders/vk14/"};
// the vulkan environment glslang is asked for: 1.4 where it knows it
std::string vulkan_env{"vulkan1.4"};

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
	std::ofstream file("/tmp/shader");
	file << Source.text;
	file.close();

	// the output only when it fails (glslangValidator names the file it compiled to spir-v)
	auto const command{"glslangValidator " + Mode + " -S " + stage_name(Source.stage) + " /tmp/shader" + (Mode.empty() ? "" : " -o /tmp/shader.spv") + " > /tmp/shader.log 2>&1"};
	int ret = system(command.c_str());
	if (ret != 0)
	{
		std::ifstream log("/tmp/shader.log");
		std::cerr << std::endl << log.rdbuf();
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
	auto const suc{check_text(gfx::preprocess_shader(gfx::shader_api::vulkan14, Name, {}, {}, false, vk14_dir), "-V --target-env " + vulkan_env)};
	std::cout << "done" << std::flush;
	return suc;
}

bool is_stage(std::filesystem::path const &Path)
{
	auto const ext = Path.extension();
	return (ext == ".frag" || ext == ".geom" || ext == ".vert" || ext == ".comp");
}

} // namespace

int main()
{
	if (system("glslangValidator --version > /dev/null 2>&1") != 0)
	{
		std::cerr << "glslangValidator not found in PATH" << std::endl;
		return 1;
	}
	{
		std::ofstream file("/tmp/shader_env.vert");
		file << "#version 460\nvoid main() { gl_Position = vec4(0.0); }\n";
	}
	if (system("glslangValidator -V --target-env vulkan1.4 /tmp/shader_env.vert -o /tmp/shader_env.spv > /dev/null 2>&1") != 0)
	{
		vulkan_env = "vulkan1.3";
		std::cout << "== glslangValidator doesn't know vulkan 1.4, the vulkan shaders are checked against vulkan 1.3" << std::endl;
	}

	bool suc = true;
	std::vector<std::string> failed;

	// the binaries are compiled with the constants written in common.glsl, the game compiles the sources with its own
	gl::glsl_common_setup();
	std::vector<std::string> differences;
	std::cout << "== constants of shaders/ogl46/common.glsl: " << std::flush;
	if (gl::glsl_common_matches(ogl46_dir, differences))
	{
		std::cout << "the same as in the game" << std::endl;
	}
	else
	{
		std::cout << "not the same as in the game:" << std::endl;
		for (auto const &difference : differences)
			std::cout << "  " << difference << std::endl;
		suc = false;
		failed.push_back("common.glsl");
	}

	for (auto const &[directory, api] : {std::pair{ogl46_dir, gfx::shader_api::opengl46}, std::pair{vk14_dir, gfx::shader_api::vulkan14}})
	{
		if (false == std::filesystem::is_directory(directory))
			continue;
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
