#include "stdafx.h"

#include "shader.h"
#include "glsl_common.h"
#include "utilities/Logs.h"

#include <cstring>

// the values of the components are those of opengl (material textures take them as the hint of their format)
static_assert(static_cast<GLint>(gfx::shader_components::R) == GL_RED);
static_assert(static_cast<GLint>(gfx::shader_components::RG) == GL_RG);
static_assert(static_cast<GLint>(gfx::shader_components::RGB) == GL_RGB);
static_assert(static_cast<GLint>(gfx::shader_components::RGBA) == GL_RGBA);
static_assert(static_cast<GLint>(gfx::shader_components::sRGB) == GL_SRGB);
static_assert(static_cast<GLint>(gfx::shader_components::sRGB_A) == GL_SRGB_ALPHA);
static_assert(gfx::MAX_SHADER_TEXTURES == gl::MAX_TEXTURES);
static_assert(gfx::MAX_SHADER_PARAMS == gl::MAX_PARAMS);

namespace
{

GLenum shader_type(gfx::shader_stage const Stage)
{
    switch (Stage)
    {
    case gfx::shader_stage::vertex:
        return GL_VERTEX_SHADER;
    case gfx::shader_stage::fragment:
        return GL_FRAGMENT_SHADER;
    case gfx::shader_stage::geometry:
        return GL_GEOMETRY_SHADER;
    case gfx::shader_stage::compute:
        return GL_COMPUTE_SHADER;
    }
    return GL_VERTEX_SHADER;
}

gfx::shader_source preprocess(std::string const &Name, std::string const &Defines, bool const Allowmissing)
{
    return gfx::preprocess_shader(gfx::shader_api::opengl46, Name, Defines, gl::glsl_common + gl::glsl_settings_defines(), Allowmissing);
}

// the files of the sources named by their numbers, for the messages of the compiler (#line <line> <number>)
std::string files_of(gfx::shader_source const &Source)
{
    std::string result;
    for (std::size_t i = 0; i < Source.files.size(); ++i)
        result += (i > 0 ? ", " : "") + std::to_string(i) + ": " + Source.files[i];
    return result;
}

std::string shader_log(GLuint const Shader)
{
    GLint length{0};
    glGetShaderiv(Shader, GL_INFO_LOG_LENGTH, &length);
    std::string info(static_cast<std::size_t>(std::max(length, 1)), '\0');
    glGetShaderInfoLog(Shader, static_cast<GLsizei>(info.size()), nullptr, info.data());
    info.resize(std::strlen(info.c_str()));
    return info;
}

std::string program_log(GLuint const Program)
{
    GLint length{0};
    glGetProgramiv(Program, GL_INFO_LOG_LENGTH, &length);
    std::string info(static_cast<std::size_t>(std::max(length, 1)), '\0');
    glGetProgramInfoLog(Program, static_cast<GLsizei>(info.size()), nullptr, info.data());
    info.resize(std::strlen(info.c_str()));
    return info;
}

} // namespace

bool gl::shader_binaries_enabled()
{
    // decided once, at the first shader: the settings and the driver don't change while the renderer runs
    static bool const enabled{[]() {
        if (false == Global.gfx_shader_binaries)
        {
            WriteLog("shaders: compiled from the sources (gfx.shaders.binaries no)", logtype::shader);
            return false;
        }
        if ((false == GLAD_GL_VERSION_4_6) || (glSpecializeShader == nullptr))
        {
            WriteLog("shaders: the driver can't load SPIR-V, compiled from the sources", logtype::shader);
            return false;
        }
        std::vector<std::string> differences;
        if (false == glsl_common_matches({}, differences))
        {
            std::string text;
            for (auto const &difference : differences)
                text += "\n  " + difference;
            ErrorLog("shaders: the constants in " + gfx::shader_source_dir(gfx::shader_api::opengl46) + "common.glsl aren't those of the game, so the SPIR-V binaries of " +
                         gfx::shader_binary_dir(gfx::shader_api::opengl46) + " can't be used; the shaders are compiled from the sources:" + text,
                     logtype::shader);
            return false;
        }
        WriteLog("shaders: the SPIR-V binaries of " + gfx::shader_binary_dir(gfx::shader_api::opengl46) + " are loaded where there are up to date ones", logtype::shader);
        return true;
    }()};
    return enabled;
}

gl::shader::shader(const std::string &filename, const std::string &defines) : name(filename), m_defines(defines)
{
    auto const source{preprocess(filename, defines, true)};
    texture_conf = source.texture_conf;
    param_conf = source.param_conf;
    m_type = shader_type(source.stage);

    if (shader_binaries_enabled() && load_binary(source))
    {
        if (false == source.has_source)
            ErrorLog("shader " + filename + ": the SPIR-V binary is used without its source, so the textures and the parameters of the materials (#pragma texture, #pragma param) aren't known", logtype::shader);
        return;
    }

    if (false == source.has_source)
        throw shader_exception("cannot read shader: " + gfx::shader_source_dir(gfx::shader_api::opengl46) + filename);
    **this = compile_source(source);
}

bool gl::shader::load_binary(gfx::shader_source const &Source)
{
    std::string reason;
    auto const words{gfx::load_shader_binary(Source, reason)};
    if (false == reason.empty())
        WriteLog("shader " + name + (Source.variant.empty() ? "" : " (" + Source.variant + ")") + ": " + reason + ", compiled from the source", logtype::shader);
    if (words.empty())
        return false;

    auto const id{glCreateShader(m_type)};
    glShaderBinary(1, &id, GL_SHADER_BINARY_FORMAT_SPIR_V, words.data(), static_cast<GLsizei>(words.size() * sizeof(std::uint32_t)));
    // the settings the module has, the others aren't given
    auto const values{glsl_settings_values()};
    std::vector<GLuint> indices;
    std::vector<GLuint> settings;
    for (auto const id : gfx::spirv_specialization_ids(words))
    {
        if (id < values.size())
        {
            indices.push_back(id);
            settings.push_back(values[id]);
        }
    }
    glSpecializeShader(id, "main", static_cast<GLuint>(indices.size()), indices.data(), settings.data());

    GLint status{GL_FALSE};
    glGetShaderiv(id, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE)
    {
        WriteLog("shader " + name + ": the SPIR-V binary can't be loaded (" + shader_log(id) + "), compiled from the source", logtype::shader);
        glDeleteShader(id);
        return false;
    }
    **this = id;
    m_binary = true;
    return true;
}

GLuint gl::shader::compile_source(gfx::shader_source const &Source) const
{
    const GLchar *cstr = Source.text.c_str();

    auto const id{glCreateShader(m_type)};
    glShaderSource(id, 1, &cstr, nullptr);
    glCompileShader(id);

    GLint status;
    glGetShaderiv(id, GL_COMPILE_STATUS, &status);
    if (!status)
    {
        auto const info{shader_log(id)};
        glDeleteShader(id);
        ErrorLog("bad shader: " + name + ": " + info + "(source strings: " + files_of(Source) + ")", logtype::shader);
        throw shader_exception("failed to compile " + name + ": " + info);
    }
    return id;
}

GLuint gl::shader::source_object() const
{
    if (false == m_binary)
        return *this;
    if (m_source_object == 0)
        m_source_object = compile_source(preprocess(name, m_defines, false));
    return m_source_object;
}

gl::shader::~shader()
{
    if (m_source_object != 0)
        glDeleteShader(m_source_object);
    glDeleteShader(*this);
}

void gl::program::init()
{
    bind();

    // the samplers and the blocks have their bindings in the shaders; the sources which don't say them (older ones) are given
    // them here. a program linked from SPIR-V doesn't know the names, nothing is found then
    for (auto const &[texturename, textureentry] : texture_conf)
    {
        GLint loc = glGetUniformLocation(*this, texturename.c_str());
        if (loc >= 0)
            glUniform1i(loc, static_cast<GLint>(textureentry.id));
    }

    auto const unit = [this](char const *Name, std::size_t const Unit) {
        GLint loc = glGetUniformLocation(*this, Name);
        if (loc >= 0)
            glUniform1i(loc, static_cast<GLint>(Unit));
    };
    unit("shadowmap", gl::SHADOW_TEX);
    unit("envmap", gl::ENV_TEX);
    unit("headlightmap", gl::HEADLIGHT_TEX);
    // the prefiltered environment map (envmapping.glsl); without a unit of its own the cube sampler would share unit 0 with
    // the 2d diffuse texture of the materials, which makes their draws fail (GL_INVALID_OPERATION)
    unit("envmap_ggx", gl::ENV_GGX_TEX);

	GLuint index;

	if ((index = glGetUniformBlockIndex(*this, "scene_ubo")) != GL_INVALID_INDEX)
		glUniformBlockBinding(*this, index, gl::SCENE_UBO_BINDING);

	if ((index = glGetUniformBlockIndex(*this, "model_ubo")) != GL_INVALID_INDEX)
		glUniformBlockBinding(*this, index, gl::MODEL_UBO_BINDING);
}

gl::program::program()
{
    **this = glCreateProgram();
}

gl::program::program(std::vector<std::reference_wrapper<const gl::shader>> const &shaders) : program()
{
    for (const gl::shader &s : shaders)
        attach(s);
    link();
}

void gl::program::attach(const gl::shader &s)
{
    for (auto [texturename, textureentry] : s.texture_conf)
        texture_conf.try_emplace(texturename, std::move(textureentry));
    for (auto [paramname, paramentry] : s.param_conf)
        param_conf.try_emplace(paramname, std::move(paramentry));
    m_attached.push_back(&s);
}

void gl::program::link()
{
    // a program is linked either from SPIR-V binaries or from sources only: when one of the shaders has no binary, the others are
    // compiled from the source too
    auto const allbinary{std::all_of(m_attached.begin(), m_attached.end(), [](shader const *Shader) { return Shader->is_binary(); })};
    GLint status{GL_FALSE};
    if (allbinary && (false == m_attached.empty()))
    {
        for (auto const *s : m_attached)
            glAttachShader(*this, *s);
        glLinkProgram(*this);
        glGetProgramiv(*this, GL_LINK_STATUS, &status);
        for (auto const *s : m_attached)
            glDetachShader(*this, *s);
        if (status == GL_TRUE)
        {
            m_binary = true;
        }
        else
        {
            std::string names;
            for (auto const *s : m_attached)
                names += (names.empty() ? "" : ", ") + s->name;
            WriteLog("program of " + names + ": the SPIR-V binaries can't be linked (" + program_log(*this) + "), compiled from the sources", logtype::shader);
        }
    }
    if (status == GL_FALSE)
    {
        std::vector<GLuint> objects;
        for (auto const *s : m_attached)
            objects.push_back(s->source_object());
        for (auto const object : objects)
            glAttachShader(*this, object);
        glLinkProgram(*this);
        glGetProgramiv(*this, GL_LINK_STATUS, &status);
        for (auto const object : objects)
            glDetachShader(*this, object);
        m_binary = false;
    }
    m_attached.clear();
    if (status == GL_FALSE)
        throw shader_exception("failed to link program: " + program_log(*this));

    init();
}

gl::program::~program()
{
    glDeleteProgram(*this);
}

void gl::program::bind(GLuint i)
{
    glUseProgram(i);
}

void gl::program::dispatch(GLuint groups_x, GLuint groups_y, GLuint groups_z)
{
    bind();
    glDispatchCompute(groups_x, groups_y, groups_z);
}
