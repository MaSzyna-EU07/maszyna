#pragma once

#include <array>
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>

#include "object.h"
#include "bindable.h"
#include "rendering/common/shadersource.h"

namespace gl
{
    using shader_exception = gfx::shader_exception;

    // the settings of the game the shaders take (shaders/ogl46/settings.glsl): the ids of the specialization constants of the
    // SPIR-V binaries; compiled from the source, the shaders get them as constants (gl::glsl_settings_defines())
    enum class shader_setting : GLuint
    {
        shadowmap = 0,
        envmap = 1,
        motionblur = 2,
        postfx = 3,
        extraeffects = 4,
        output_gamma = 5,
        count = 6
    };

    // a shader of shaders/ogl46/: the SPIR-V binary of shaders/ogl46bin/ when there's an up to date one (gfx.shaders.binaries),
    // compiled from the source otherwise
    class shader : public object
    {
    public:
        // Defines: lines put before the source (after the version), e.g. "#define MODEL_DRAWS 1\n" for a variant of the shader
        explicit shader(const std::string &filename, const std::string &defines = {});
        ~shader();

        using components_e = gfx::shader_components;
        using texture_entry = gfx::shader_texture_entry;
        using defaultparam_e = gfx::shader_param_default;
        using param_entry = gfx::shader_param_entry;

        gfx::shader_texture_conf texture_conf;
        gfx::shader_param_conf param_conf;
        std::string name;

        // whether the shader was loaded from a SPIR-V binary
        bool is_binary() const
        {
            return m_binary;
        }
        // the shader compiled from the source: the shader itself, or for a binary one the same shader compiled from the source
        // (a program can't be linked from binaries and sources together). throws shader_exception
        GLuint source_object() const;

    private:
        bool load_binary(gfx::shader_source const &Source);
        GLuint compile_source(gfx::shader_source const &Source) const;

        GLenum m_type{0};
        std::string m_defines;
        bool m_binary{false};
        mutable GLuint m_source_object{0};
    };

    class program : public gfx::shader_program, public object, public bindable<program>
    {
    public:
        program();
        explicit program(std::vector<std::reference_wrapper<const gl::shader>> const &);
        ~program();

        using bindable::bind;
        static void bind(GLuint i);

        // the shaders are linked by link(), which has to come while they are there
        void attach(const shader &);
        void link();

        // for a program linked from a compute shader only: binds it and runs the given number of work groups
        void dispatch(GLuint groups_x, GLuint groups_y = 1, GLuint groups_z = 1);

        // whether the program was linked from SPIR-V binaries
        bool is_binary() const
        {
            return m_binary;
        }

	private:
        void init();

        std::vector<shader const *> m_attached;
        bool m_binary{false};
	};

    // whether the shaders are loaded from the SPIR-V binaries where there are some: gfx.shaders.binaries, an opengl 4.6 driver,
    // and the constants of the game the same as those the binaries were compiled with (gl::glsl_common_matches())
    bool shader_binaries_enabled();
}
