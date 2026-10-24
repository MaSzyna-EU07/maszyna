/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the shader files of the renderers, whichever api compiles them: where they are, how a file is made ready to compile (its
// includes, the directives of the materials, what the renderer puts before the source), and the SPIR-V binaries compiled from
// them ahead of time, which the renderers load before they compile the source.
//
// each api has a directory of sources and one of binaries: shaders/ogl46/ and shaders/ogl46bin/ for opengl 4.6,
// shaders/vk14/ and shaders/vk14bin/ for vulkan 1.4. a binary is named after its source, <file>.spv, and a variant of the
// shader (the defines of the renderer, e.g. MODEL_DRAWS) <file>.<DEFINE>.spv. the sources can be compiled by glslc as they are
// (shaders/compile_spirv.py), which also writes the manifest of the binaries: the sources each binary was compiled from, so a
// binary older than its sources isn't taken.
//
// no graphics api here: the file is also built into the shader validator (ci_shadervalidator)

#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace gfx
{

class shader_exception : public std::runtime_error
{
	using runtime_error::runtime_error;
};

enum class shader_api
{
	opengl46,
	vulkan14
};

enum class shader_stage
{
	vertex,
	fragment,
	geometry,
	compute
};

// textures and parameters a material shader can take, the same as gl::MAX_TEXTURES and gl::MAX_PARAMS
constexpr std::size_t MAX_SHADER_TEXTURES = 8;
constexpr std::size_t MAX_SHADER_PARAMS = 3;

// what the texture of a unit holds, as #pragma texture(name, unit, components) gives it. the values are those of opengl
// (GL_RED ... GL_SRGB_ALPHA), which the textures take as the hint of their format
enum class shader_components : int
{
	R = 0x1903,
	RG = 0x8227,
	RGB = 0x1907,
	RGBA = 0x1908,
	sRGB = 0x8C40,
	sRGB_A = 0x8C42
};

struct shader_texture_entry
{
	std::size_t id;
	shader_components components;
};

enum class shader_param_default
{
	required,
	nan,
	zero,
	one,
	ambient,
	diffuse,
	specular,
	glossiness
};

// #pragma param(name, location, offset, size, default): a part of one of the vec4 parameters of the model block
struct shader_param_entry
{
	std::size_t location;
	std::size_t offset;
	std::size_t size;
	shader_param_default defaultparam;
};

using shader_texture_conf = std::unordered_map<std::string, shader_texture_entry>;
using shader_param_conf = std::unordered_map<std::string, shader_param_entry>;

// a program of a material as the materials see it, whichever api draws it: the textures and the parameters it takes, the
// name it was fetched by (gfx_renderer::Fetch_Shader)
class shader_program
{
  public:
	virtual ~shader_program() = default;

	shader_texture_conf texture_conf;
	shader_param_conf param_conf;
	std::string name;
};

// a shader file made ready to compile
struct shader_source
{
	shader_api api{shader_api::opengl46};
	shader_stage stage{shader_stage::vertex};
	// the file as asked for, e.g. mat_default.frag
	std::string name;
	// the names of the defines of the variant joined with dots (MODEL_DRAWS), empty for the shader itself. a variant whose
	// defines aren't all plain flags (#define NAME or #define NAME 1) has no binary: invalid_variant
	std::string variant;
	bool has_binary_name{true};
	// whether the source file is there; a binary can be loaded without it, but the material directives come from the source
	bool has_source{false};
	// the source with the preamble put after #version and the includes expanded, for a compiler of the source. the files are
	// told apart in the messages of the compiler by their numbers in files (#line <line> <number>)
	std::string text;
	// the file and its includes, relative to the directory of the sources, by their numbers
	std::vector<std::string> files;
	shader_texture_conf texture_conf;
	shader_param_conf param_conf;
};

// "opengl 4.6", "vulkan 1.4"
char const *shader_api_name(shader_api Api);
// the directories of the sources and of the binaries of an api: shaders/ogl46/, shaders/ogl46bin/ ...
std::string shader_source_dir(shader_api Api);
std::string shader_binary_dir(shader_api Api);
// the stage of a file by its extension (.vert, .frag, .geom, .comp). throws shader_exception for another one
shader_stage shader_stage_of(std::string const &Name);
// the define the files set for their stage: STAGE_VERTEX ...
char const *shader_stage_define(shader_stage Stage);
// the file name of the binary of a shader: mat_default.frag.spv, vertex.vert.MODEL_DRAWS.spv
std::string shader_binary_name(std::string const &Name, std::string const &Variant);
// the variant named by the defines of a renderer ("#define MODEL_DRAWS 1\n" -> MODEL_DRAWS). Plain: false when a define isn't
// a plain flag (the variant can't have a binary then)
std::string shader_variant_of(std::string const &Defines, bool &Plain);
// the hash the manifest of the binaries keeps of a source file: fnv-1a 64 of the text with the line ends made \n
std::uint64_t shader_file_hash(std::string_view Text);

// reads the shader Name of Api and makes it ready to compile: the includes expanded (#include "file", #include <file>; <common>
// stands for common.glsl), #pragma texture(...) and #pragma param(...) read and taken out of the text, and after the #version
// line (#version 460 core put first when the file has none) the defines of the variant, the preamble of the renderer and,
// where the file doesn't set it, the define of its stage. Basedir: the directory of the sources, shader_source_dir() when empty.
// a missing file is an error unless Allowmissing (the source then comes back with has_source false). throws shader_exception
shader_source preprocess_shader(shader_api Api, std::string const &Name, std::string const &Defines, std::string const &Preamble, bool const Allowmissing = false,
                                std::string const &Basedir = {});

// the SPIR-V binary of a shader from the directory of the binaries of its api. empty when there's none, when the manifest of the
// binaries says the binary was compiled from other sources than those there now, or when the file isn't SPIR-V; Reason says
// which (empty when there's simply no binary)
std::vector<std::uint32_t> load_shader_binary(shader_source const &Source, std::string &Reason);

// the ids of the specialization constants of a SPIR-V module (OpDecorate ... SpecId)
std::vector<std::uint32_t> spirv_specialization_ids(std::vector<std::uint32_t> const &Words);

// the #define lines of a source, NAME -> value as written, for the checks of the constants the binaries were compiled with
std::unordered_map<std::string, std::string> shader_file_defines(shader_api Api, std::string const &Name, std::string const &Basedir = {});
// whether the constants a renderer puts before the sources (the #define lines of Preamble) are those the file Name of Api writes
// for itself (the binaries are compiled by glslc with the values of the file). Differences: what isn't the same
bool shader_constants_match(shader_api Api, std::string const &Name, std::string const &Preamble, std::string const &Basedir, std::vector<std::string> &Differences);

} // namespace gfx
