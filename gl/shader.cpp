#include "stdafx.h"

#include <fstream>
#include <sstream>
#include <cstring>
#include "shader.h"
#include "glsl_common.h"
#include "utilities/Logs.h"

inline bool strcend(std::string const &value, std::string const &ending)
{
    if (ending.size() > value.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

namespace {

// The project's glad config only generates GLAD_GL_ARB_texture_filter_anisotropic
// among the ARB/EXT extension flags -- GL_ARB_texture_gather (desktop) and
// GL_EXT_gpu_shader5 (GLES 3.1) aren't compiled in, so we can't gate the
// shadow textureGather() optimisation on a GLAD constant. Query the live
// extension string instead. The first call walks the extension list once
// (no extension count in the dozens is large enough to matter here) and
// the result is cached in the static bool inside has_gl_extension(), so
// every subsequent shader compile is a plain bool read.
//
// SHADERVALIDATOR_STANDALONE gates out the GL queries because the offline
// shader validator tool links glad.c but never calls gladLoadGL(); the
// function pointers stay null and a real call would crash. Returning
// false there means the standalone validator simply compiles the original
// 16-tap PCF fallback path in light_common.glsl, which is what we want.
bool has_gl_extension(char const *name) {
#ifdef SHADERVALIDATOR_STANDALONE
    (void)name;
    return false;
#else
    if (!glGetIntegerv || !glGetStringi) {
        return false;
    }
    GLint count = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &count);
    for (GLint i = 0; i < count; ++i) {
		const auto ext = reinterpret_cast<char const *>(glGetStringi(GL_EXTENSIONS, i));
        if (ext != nullptr && std::strcmp(ext, name) == 0) {
            return true;
        }
    }
    return false;
#endif
}

} // anonymous namespace

std::string gl::shader::read_file(const std::string &filename)
{
    std::stringstream stream;
    std::ifstream f;

    f.open(filename);
    stream << f.rdbuf();
    f.close();

    std::string str = stream.str();
    if (str.empty())
        throw shader_exception("cannot read shader: " + filename);

    return str;
}

void gl::shader::expand_includes(std::string &str, const std::string &basedir)
{
    size_t start_pos = 0;

	const std::string magic = "#include";
    while ((start_pos = str.find(magic, start_pos)) != str.npos)
    {
		const size_t fp = str.find('<', start_pos);
		const size_t fe = str.find('>', start_pos);
        if (fp == str.npos || fe == str.npos)
            return;

        std::string filename = str.substr(fp + 1, fe - fp - 1);
        std::string content;
        if (filename != "common")
            content = read_file(basedir + filename);
        else
            content = glsl_common;

        str.replace(start_pos, fe - start_pos + 1, content);
    }
}

std::unordered_map<std::string, gl::shader::components_e> gl::shader::components_mapping =
{
    { "R", components_e::R },
    { "RG", components_e::RG },
    { "RGB", components_e::RGB },
    { "RGBA", components_e::RGBA },
    { "sRGB", components_e::sRGB },
    { "sRGB_A", components_e::sRGB_A }
};

std::unordered_map<std::string, gl::shader::defaultparam_e> gl::shader::defaultparams_mapping =
{
    { "required", defaultparam_e::required },
    { "nan", defaultparam_e::nan },
    { "zero", defaultparam_e::zero },
    { "one", defaultparam_e::one },
    { "ambient", defaultparam_e::ambient },
    { "diffuse", defaultparam_e::diffuse },
    { "specular", defaultparam_e::specular },
    { "glossiness", defaultparam_e::glossiness }
};

std::pair<GLuint, std::string> gl::shader::process_source(const std::string &filename, const std::string &basedir)
{
    std::string str;

    GLuint type;
    if (strcend(filename, ".vert"))
        type = GL_VERTEX_SHADER;
    else if (strcend(filename, ".frag"))
        type = GL_FRAGMENT_SHADER;
    else if (strcend(filename, ".geom"))
        type = GL_GEOMETRY_SHADER;
    else
        throw shader_exception("unknown shader " + filename);

    if (!Global.gfx_usegles)
    {
        str += "#version 330 core\n";
        // textureGather() on sampler2DArrayShadow is core in GLSL 4.0. On 3.30
        // desktop it requires GL_ARB_gpu_shader5 -- the older
        // GL_ARB_texture_gather (2010) only adds the non-shadow and the plain
        // sampler2DShadow overloads, NOT the sampler2DArrayShadow one we need
        // for cascaded shadow PCF. Some drivers advertise texture_gather but
        // reject the shadow-array overload because the spec for it lives in
        // gpu_shader5; so emit only when gpu_shader5 is advertised. When the
        // extension is missing, calc_shadow() in light_common.glsl falls back
        // to the original 16-tap hardware-PCF loop via #ifndef.
        // (Project's glad config doesn't generate GLAD_GL_ARB_gpu_shader5,
        //  so we query the live extension list -- see has_gl_extension above.)
        static bool const have_gpu_shader5 = has_gl_extension("GL_ARB_gpu_shader5");
        if (have_gpu_shader5)
            str += "#extension GL_ARB_gpu_shader5 : enable\n";
    }
    else
    {
        if (GLAD_GL_ES_VERSION_3_1) {
            str += "#version 310 es\n";
            // GLES 3.1 lacks textureGather on shadow samplers in core; the
            // EXT_gpu_shader5 extension adds it. Only emit the directive when
            // the driver advertises support (see desktop comment above).
            // (Glad config doesn't generate GLAD_GL_EXT_gpu_shader5 -- query
            //  the live extension string the same way.)
            static bool const have_gpu_shader5 = has_gl_extension("GL_EXT_gpu_shader5");
            if (have_gpu_shader5)
                str += "#extension GL_EXT_gpu_shader5 : enable\n";
            if (type == GL_GEOMETRY_SHADER)
                str += "#extension GL_EXT_geometry_shader : require\n";
        } else {
            str += "#version 300 es\n";
        }
        str += "precision highp float;\n";
        str += "precision highp int;\n";
        str += "precision highp sampler2DShadow;\n";
        str += "precision highp sampler2DArrayShadow;\n";
    }
    // expose the shader stage to glsl_common so vertex-only built-ins
    // (gl_InstanceID, effective_modelview, etc.) can be guarded out of
    // fragment/geometry compilations.
    if (type == GL_VERTEX_SHADER)        str += "#define STAGE_VERTEX 1\n";
    else if (type == GL_FRAGMENT_SHADER) str += "#define STAGE_FRAGMENT 1\n";
    else if (type == GL_GEOMETRY_SHADER) str += "#define STAGE_GEOMETRY 1\n";
    str += "vec4 FBOUT(vec4 x) { return " + (Global.gfx_shadergamma ? std::string("vec4(pow(x.rgb, vec3(1.0 / 2.2)), x.a)") : std::string("x")) + "; }\n";

    str += read_file(basedir + filename);

    expand_includes(str, basedir);
    parse_texture_entries(str);
    parse_param_entries(str);

    return std::make_pair(type, str);
}

void gl::shader::parse_texture_entries(std::string &str)
{
    size_t start_pos = 0;

    std::string magic = "#texture";
    while ((start_pos = str.find(magic, start_pos)) != str.npos)
    {
        size_t fp = str.find('(', start_pos);
        size_t fe = str.find(')', start_pos);
        if (fp == str.npos || fe == str.npos)
            return;

        std::istringstream ss(str.substr(fp + 1, fe - fp - 1));
        std::string token;

        std::string name;
        texture_entry conf;

        size_t arg = 0;
        while (std::getline(ss, token, ','))
        {
            std::istringstream token_ss(token);
            if (arg == 0)
                token_ss >> name;
            else if (arg == 1)
                token_ss >> conf.id;
            else if (arg == 2)
            {
                std::string comp;
                token_ss >> comp;
                if (!components_mapping.contains(comp))
                    log_error("unknown components: " + comp);
                else
                    conf.components = components_mapping[comp];
            }
            arg++;
        }

        if (arg == 3)
        {
            if (name.empty())
                log_error("empty name");
            else if (conf.id >= MAX_TEXTURES)
                log_error("invalid texture binding: " + std::to_string(conf.id));
            else
                texture_conf.emplace(std::make_pair(name, conf));
        }
        else
            log_error("invalid argument count to #texture");

        str.erase(start_pos, fe - start_pos + 1);
    }
}

void gl::shader::parse_param_entries(std::string &str)
{
    size_t start_pos = 0;

    std::string magic = "#param";
    while ((start_pos = str.find(magic, start_pos)) != str.npos)
    {
        size_t fp = str.find('(', start_pos);
        size_t fe = str.find(')', start_pos);
        if (fp == str.npos || fe == str.npos)
            return;

        std::istringstream ss(str.substr(fp + 1, fe - fp - 1));
        std::string token;

        std::string name;
        param_entry conf;

        size_t arg = 0;
        while (std::getline(ss, token, ','))
        {
            std::istringstream token_ss(token);
            if (arg == 0)
                token_ss >> name;
            else if (arg == 1)
                token_ss >> conf.location;
            else if (arg == 2)
                token_ss >> conf.offset;
            else if (arg == 3)
                token_ss >> conf.size;
            else if (arg == 4)
            {
                std::string tok;
                token_ss >> tok;
                if (!defaultparams_mapping.contains(tok))
                    log_error("unknown param default: " + tok);
                conf.defaultparam = defaultparams_mapping[tok];
            }
            arg++;
        }

        if (arg == 5)
        {
            if (name.empty())
                log_error("empty name");
            else if (conf.location >= MAX_PARAMS)
                log_error("invalid param binding: " + std::to_string(conf.location));
            else if (conf.offset > 3)
                log_error("invalid offset: " + std::to_string(conf.offset));
            else if (conf.offset + conf.size > 4)
                log_error("invalid size: " + std::to_string(conf.size));
            else
                param_conf.emplace(std::make_pair(name, conf));
        }
        else
            log_error("invalid argument count to #param");

        str.erase(start_pos, fe - start_pos + 1);
    }
}

void gl::shader::log_error(const std::string &str)
{
    ErrorLog("bad shader: " + name + ": " + str, logtype::shader);
}

gl::shader::shader(const std::string &filename)
{
    name = filename;

	const std::pair<GLuint, std::string> source = process_source(filename, "shaders/");

    const GLchar *cstr = source.second.c_str();

    **this = glCreateShader(source.first);
    glShaderSource(*this, 1, &cstr, 0);
    glCompileShader(*this);

    GLint status;
    glGetShaderiv(*this, GL_COMPILE_STATUS, &status);
    if (!status)
    {
        GLchar info[512];
        glGetShaderInfoLog(*this, 512, 0, info);
        log_error(std::string(info));

        throw shader_exception("failed to compile " + filename + ": " + std::string(info));
    }
}

gl::shader::~shader()
{
#ifndef SHADERVALIDATOR_STANDALONE
    glDeleteShader(*this);
#endif
}

void gl::program::init()
{
    bind();

    for (auto it : texture_conf)
    {
		const shader::texture_entry &e = it.second;
		const GLuint loc = glGetUniformLocation(*this, it.first.c_str());
        glUniform1i(loc, e.id);
    }

    glUniform1i(glGetUniformLocation(*this, "shadowmap"), SHADOW_TEX);
    glUniform1i(glGetUniformLocation(*this, "envmap"), ENV_TEX);
    glUniform1i(glGetUniformLocation(*this, "headlightmap"), HEADLIGHT_TEX);

	GLuint index;

	if ((index = glGetUniformBlockIndex(*this, "scene_ubo")) != GL_INVALID_INDEX)
		glUniformBlockBinding(*this, index, 0);

	if ((index = glGetUniformBlockIndex(*this, "model_ubo")) != GL_INVALID_INDEX)
		glUniformBlockBinding(*this, index, 1);

	if ((index = glGetUniformBlockIndex(*this, "light_ubo")) != GL_INVALID_INDEX)
		glUniformBlockBinding(*this, index, 2);

	if ((index = glGetUniformBlockIndex(*this, "instance_ubo")) != GL_INVALID_INDEX)
		glUniformBlockBinding(*this, index, 3);
}

gl::program::program()
{
    **this = glCreateProgram();
}

gl::program::program(std::vector<std::reference_wrapper<const shader>> shaders) : program()
{
    for (const shader &s : shaders)
        attach(s);
    link();
}

void gl::program::attach(const shader &s)
{
    for (auto it : s.texture_conf)
        texture_conf.emplace(std::make_pair(it.first, std::move(it.second)));
    for (auto it : s.param_conf)
        param_conf.emplace(std::make_pair(it.first, std::move(it.second)));
    glAttachShader(*this, *s);
}

void gl::program::link()
{
    glLinkProgram(*this);

    GLint status;
    glGetProgramiv(*this, GL_LINK_STATUS, &status);
    if (!status)
    {
        GLchar info[512];
        glGetProgramInfoLog(*this, 512, 0, info);
        throw shader_exception("failed to link program: " + std::string(info));
    }

    init();
}

gl::program::~program()
{
    glDeleteProgram(*this);
}

void gl::program::bind(const GLuint i)
{
    glUseProgram(i);
}
