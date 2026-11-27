#include "stdafx.h"
#include "../gl/shader.h"
#include "../gl/glsl_common.h"
#include "utilities/Globals.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <vector>
#include <algorithm>
#ifdef _WIN32
#include <random>
#else
#include <unistd.h>
#endif

_global Global;

// writes the text to a new file with a unique name in the temporary directory of the system and returns its path,
// or an empty string when it cannot be written. the file is created exclusively, so nothing put in a shared /tmp
// under a name known in advance (a file or a link) is written through
std::string write_temp_file(std::string const &text)
{
	std::string path = (std::filesystem::temp_directory_path() / "maszyna-shader-XXXXXX").string();
	std::FILE *file = nullptr;
#ifdef _WIN32
	// no mkstemp() in the CRT; a random name, opened with "x", which fails when the file is there already
	std::random_device random;
	for (int attempt = 0; attempt < 100 && file == nullptr; ++attempt) {
		path.replace(path.size() - 6, 6, std::to_string(100000 + random() % 900000));
		if (fopen_s(&file, path.c_str(), "wx") != 0)
			file = nullptr;
	}
#else
	int const descriptor = mkstemp(path.data());
	if (descriptor != -1) {
		file = fdopen(descriptor, "w");
		if (file == nullptr) {
			close(descriptor);
			std::remove(path.c_str());
		}
	}
#endif
	if (file == nullptr)
		return {};

	bool const written = std::fwrite(text.data(), 1, text.size(), file) == text.size();
	if (std::fclose(file) != 0 || !written) {
		std::remove(path.c_str());
		return {};
	}
	return path;
}

bool check_shader(std::string filename)
{
	gl::shader shader_processor;

	gl::glsl_common_setup();
	std::pair<GLuint, std::string> source;
	try {
		source = shader_processor.process_source(filename, "../shaders/");
	}
	catch (gl::shader_exception const &e) {
		std::cout << "failed to preprocess: " << e.what() << std::endl;
		return false;
	}

	std::string stage;
	if (source.first == GL_VERTEX_SHADER)
		stage = "vert";
	else if (source.first == GL_FRAGMENT_SHADER)
		stage = "frag";
	else if (source.first == GL_GEOMETRY_SHADER)
		stage = "geom";
	else
		return false;

	// glslang defines the GL_ARB_gpu_shader5 macro without the #extension directive, so light_common.glsl
	// takes its textureGather() path, which needs the extension enabled on desktop GL 3.30. has_gl_extension()
	// of gl/shader.cpp has no GL context here and says no, so the directive which the simulator gives on the
	// drivers with the extension is added here
	std::string const gpu_shader5 = "#extension GL_ARB_gpu_shader5 : enable\n";
	if (!Global.gfx_usegles && source.second.find(gpu_shader5) == std::string::npos) {
		std::size_t const version_end = source.second.find('\n');
		if (version_end != std::string::npos)
			source.second.insert(version_end + 1, gpu_shader5);
	}

	// the stage is given with -S, so the file needs no extension
	std::string const path = write_temp_file(source.second);
	if (path.empty()) {
		std::cout << "cannot write the shader to the temporary directory" << std::endl;
		return false;
	}

	int ret = system(("glslangValidator -S " + stage + " \"" + path + "\"").c_str());
	std::remove(path.c_str());

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
	// the default of the simulator, which compiles the alpha to coverage branches of the shaders
	Global.iMultisampling = 2;
	Global.gfx_headlights_beamwidth = 2.f;
	if (!check_shader(filename))
		suc = false;

	std::cout << "done, features off..." << std::flush;
	Global.gfx_shadowmap_enabled = false;
	Global.gfx_envmap_enabled = false;
	Global.gfx_postfx_motionblur_enabled = false;
	Global.gfx_skippipeline = true;
	Global.gfx_extraeffects = false;
	Global.gfx_shadergamma = false;
	Global.iMultisampling = 0;
	Global.gfx_headlights_beamwidth = 1.f;
	if (!check_shader(filename))
		suc = false;

	std::cout << "done" << std::flush;

	return suc;
}


bool check_shader_confs(std::string filename)
{
	bool suc = true;

	std::cout << "desktop GL: " << std::flush;
	Global.gfx_usegles = false;
	if (!check_shader_allopts(filename))
		suc = false;

	std::cout << "; GLES: " << std::flush;
	GLAD_GL_ES_VERSION_3_1 = 1;
	Global.gfx_usegles = true;
	if (!check_shader_allopts(filename))
		suc = false;

	std::cout << std::endl;

	return suc;
}

int main()
{
	// in name order, so the runs can be compared
	std::vector<std::string> shaders;
	for (auto &f : std::filesystem::recursive_directory_iterator("../shaders/")) {
		if (!f.is_regular_file())
			continue;

		if (f.path().extension() != ".frag" && f.path().extension() != ".geom" && f.path().extension() != ".vert")
			continue;

		shaders.push_back(std::filesystem::relative(f.path(), "../shaders/").generic_string());
	}
	std::sort(shaders.begin(), shaders.end());

	std::vector<std::string> failed;
	for (auto const &relative : shaders) {
		std::cout << "== validating shader " << relative << ": " << std::flush;
		if (!check_shader_confs(relative))
			failed.push_back(relative);
	}

	if (failed.empty())
		std::cout << "==== all checks ok (" << shaders.size() << " shaders)" << std::endl;
	else {
		std::cout << "==== checks failed for " << failed.size() << " of " << shaders.size() << " shaders:" << std::endl;
		for (auto const &relative : failed)
			std::cout << relative << std::endl;
		return 1;
	}

	return 0;
}
