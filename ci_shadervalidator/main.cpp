#include "stdafx.h"
#include "../gl/shader.h"
#include "../gl/glsl_common.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <vector>

_global Global;

bool check_shader(std::string filename, std::string const &defines = {})
{
	gl::shader shader_processor;
	shader_processor.name = filename;

	gl::glsl_common_setup();
	std::pair<GLuint, std::string> source = shader_processor.process_source(filename, "../shaders/", defines);

	std::string stage;
	if (source.first == GL_VERTEX_SHADER)
		stage = "vert";
	else if (source.first == GL_FRAGMENT_SHADER)
		stage = "frag";
	else if (source.first == GL_GEOMETRY_SHADER)
		stage = "geom";
	else if (source.first == GL_COMPUTE_SHADER)
		stage = "comp";
	else
		return false;

	if (ErrorLogged)
	{
		ErrorLogged = false;
		return false;
	}

	std::ofstream file("/tmp/shader");
	file << source.second;
	file.close();

	int ret = system(("glslangValidator -S " + stage + " /tmp/shader").c_str());

	return ret == 0;
}

bool check_shader_allopts(std::string filename)
{
	bool suc = true;

	std::cout << "features on..." << std::flush;
	Global.gfx_shadowmap_enabled = true;	
	Global.gfx_envmap_enabled = true;
	Global.gfx_postfx_motionblur_enabled = true;
	Global.gfx_skippipeline = false;
	Global.gfx_extraeffects = true;
	Global.gfx_shadergamma = true;
	if (!check_shader(filename))
		suc = false;

	std::cout << "done, features off..." << std::flush;
	Global.gfx_shadowmap_enabled = false;
	Global.gfx_envmap_enabled = false;
	Global.gfx_postfx_motionblur_enabled = false;
	Global.gfx_skippipeline = true;
	Global.gfx_extraeffects = false;
	Global.gfx_shadergamma = false;
	if (!check_shader(filename))
		suc = false;

	// the variant of the vertex and fragment stages the multi draws of the instances use (model blocks by gl_DrawID)
	auto const ext{std::filesystem::path(filename).extension()};
	if (ext == ".vert" || ext == ".frag")
	{
		std::cout << "done, model draws..." << std::flush;
		Global.gfx_shadowmap_enabled = true;
		Global.gfx_envmap_enabled = true;
		Global.gfx_postfx_motionblur_enabled = true;
		Global.gfx_skippipeline = false;
		Global.gfx_extraeffects = true;
		Global.gfx_shadergamma = true;
		if (!check_shader(filename, "#define MODEL_DRAWS 1\n"))
			suc = false;
	}

	std::cout << "done" << std::flush;

	return suc;
}


bool check_shader_confs(std::string filename)
{
	bool suc = true;

	std::cout << "desktop GL: " << std::flush;
	if (!check_shader_allopts(filename))
		suc = false;

	std::cout << std::endl;

	return suc;
}

int main()
{
	if (system("glslangValidator --version > /dev/null 2>&1") != 0)
	{
		std::cerr << "glslangValidator not found in PATH" << std::endl;
		return 1;
	}

	bool suc = true;
	std::vector<std::string> failed;
	for (auto &f : std::filesystem::recursive_directory_iterator("../shaders/")) {
		if (!f.is_regular_file())
			continue;

		std::string relative = std::filesystem::relative(f.path(), "../shaders/").string();
		auto const ext = f.path().extension();
		if (ext != ".frag" && ext != ".geom" && ext != ".vert" && ext != ".comp")
			continue;

		std::cout << "== validating shader " << relative << ": " << std::flush;
		if (!check_shader_confs(relative))
		{
			suc = false;
			failed.push_back(relative);
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
