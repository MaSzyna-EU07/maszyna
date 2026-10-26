/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46renderer.h"
#include "utilities/color.h"
#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "vehicle/Train.h"
#include "vehicle/Camera.h"
#include "simulation/simulation.h"
#include "model/MdlMngr.h"
#include "audio/audiorenderer.h"
#include "world/Track.h"
#include "world/Traction.h"
#include "world/MemCell.h"
#include "audio/sound.h"
#include "utilities/memorystats.h"
#include "utilities/Logs.h"
#include "simulation/simulationtime.h"
#include "application/application.h"
#include "model/AnimModel.h"
#include "rendering/opengl46/opengl46geometrybank.h"
#include "rendering/screenshot.h"
#include "rendering/editoroverlay.h"
#include <imgui/imgui_impl_opengl3.h>

//#define EU07_DEBUG_OPENGL

int constexpr EU07_PICKBUFFERSIZE{ 1024 }; // size of (square) textures bound with the pick framebuffer

void GLAPIENTRY
ErrorCallback( GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam ) {
/*
    auto const typestring {
        type == GL_DEBUG_TYPE_ERROR ? "error" :
        type == GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR ? "deprecated behavior" :
        type == GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR ? "undefined behavior" :
        type == GL_DEBUG_TYPE_PORTABILITY ? "portability" :
        type == GL_DEBUG_TYPE_PERFORMANCE ? "performance" :
        type == GL_DEBUG_TYPE_OTHER ? "other" :
        "unknown" };
*/
    auto const severitystring {
        severity == GL_DEBUG_SEVERITY_HIGH ? "high severity" :
        severity == GL_DEBUG_SEVERITY_MEDIUM ? "medium severity" :
        severity == GL_DEBUG_SEVERITY_LOW ? "low severity" :
        severity == GL_DEBUG_SEVERITY_NOTIFICATION ? "notification" :
        "unknown" };

    ErrorLog(
        "bad gfx code: " + std::string( type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR ** " : "" )
        + to_hex_str( id )
        + " (" + severitystring + ") "
        + message );
}

bool opengl46_renderer::Init(GLFWwindow *Window)
{
    if (Global.gfx_gldebug) {
        glEnable( GL_DEBUG_OUTPUT );
        glEnable( GL_DEBUG_OUTPUT_SYNCHRONOUS );
        glDebugMessageControl( GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE );
        glDebugMessageControl( GL_DONT_CARE, GL_DEBUG_TYPE_PERFORMANCE, GL_DONT_CARE, 0, nullptr, GL_FALSE );
        glDebugMessageCallback( ErrorCallback, 0 );
    }

	if (!Init_caps())
		return false;

	if (Global.gfx_antialiasing == global_settings::antialiasing_mode::dlss)
	{
		// dlss is of the vulkan renderer only: the temporal anti-aliasing of this one instead
		WriteLog("gfx.antialiasing dlss isn't available with opengl, using taa");
		Global.gfx_antialiasing = global_settings::antialiasing_mode::taa;
	}

	WriteLog("preparing renderer..");

	m_window = Window;

	gl::glsl_common_setup();
	m_windscreenwater.set_host({[this](material_handle const Material) { return &m_materials.material(Material); }});

	if (true == Global.ScaleSpecularValues)
	{
		m_specularopaquescalefactor = 0.25f;
		m_speculartranslucentscalefactor = 1.5f;
	}

	// rgb value for 5780 kelvin
	Global.DayLight.diffuse[0] = 1.0f;        // 255 / 255
	Global.DayLight.diffuse[1] = 0.9490196f;  // 242 / 255
	Global.DayLight.diffuse[2] = 0.9058824f;  // 231 / 255
	Global.DayLight.is_directional = true;
	m_sunlight.id = opengl46_renderer::sunlight;

	// preload some common textures
	WriteLog("Loading common gfx data...");
	m_glaretexture = Fetch_Texture("fx/lightglare");
	m_suntexture = Fetch_Texture("fx/sun");
	m_moontexture = Fetch_Texture("fx/moon");
	m_smoketexture = Fetch_Texture("fx/smoke");
    m_headlightstexture = Fetch_Texture("fx/headlights:st");
	WriteLog("...gfx data pre-loading done");

	// prepare basic geometry chunks
	float const size = 2.5f / 2.0f;
        auto const geometrybank = Create_Bank();

    gfx::vertex_array billboard_array{
        {{-size, size, 0.f}, glm::vec3(), {1.f, 1.f}}, {{size, size, 0.f}, glm::vec3(), {0.f, 1.f}}, {{-size, -size, 0.f}, glm::vec3(), {1.f, 0.f}}, {{size, -size, 0.f}, glm::vec3(), {0.f, 0.f}}};
    gfx::userdata_array userdata{};
	m_billboardgeometry = m_geometry.create_chunk(billboard_array, userdata, geometrybank, GL_TRIANGLE_STRIP);
	m_empty_vao = std::make_unique<gl::vao>();

	try
	{
		m_vertex_shader = std::make_unique<gl::shader>("vertex.vert");
		m_line_shader = make_shader("traction.vert", "traction.frag");
		m_freespot_shader = make_shader("freespot.vert", "freespot.frag");
		m_shadow_shader = make_shader("simpleuv.vert", "shadowmap.frag");
		m_alpha_shadow_shader = make_shader("simpleuv.vert", "alphashadowmap.frag");
        m_pick_shader = make_shader("vertexonly.vert", "pick.frag");
        m_pick_surface_shader = make_shader("simpleuv.vert", "pick_surface.frag");
		m_billboard_shader = make_shader("simpleuv.vert", "billboard.frag");
        m_celestial_shader = make_shader("celestial.vert", "celestial.frag");
        m_hiddenarea_shader = make_shader("hiddenarea.vert", "hiddenarea.frag");
        m_copy_shader = make_shader("quad.vert", "copy.frag");
        {
            gl::shader cluster_build("cluster_build.comp");
            m_cluster_build_shader = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{cluster_build});
        }
        {
            gl::shader cluster_cull("cluster_cull.comp");
            m_cluster_cull_shader = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{cluster_cull});
        }
        m_cluster_heatmap_shader = make_shader("quad.vert", "cluster_heatmap.frag");
        m_prepass_shader = make_shader("depthprepass.vert", "depthprepass.frag");
        m_prepass_alpha_shader = make_shader("depthprepass.vert", "depthprepass_alpha.frag");
		m_invalid_material = Fetch_Material("invalid");
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("invalid shader: " + std::string(e.what()));
		return false;
	}

	scene_ubo = std::make_unique<gl::ubo>(sizeof(gl::scene_ubs), gl::SCENE_UBO_BINDING);
	model_ubo = std::make_unique<gl::stream_ubo>(sizeof(gl::model_ubs), gl::MODEL_UBO_BINDING, Global.gfx_bufferstreaming);
	light_ssbo = std::make_unique<gl::ssbo>(sizeof(gl::light_data), gl::LIGHT_SSBO_BINDING);
	light_ssbo->bind_base();
	{
		// element 0 of the instance buffers is identity. This is the matrix sampled by
		// non-instanced draws, so existing rendering paths
		// continue to multiply effective_modelview = identity * modelview = modelview.
		// the buffers grow when a frame needs more
		glm::mat4 const identity(1.0f);
		m_instancestream = std::make_unique<gl::frame_stream>(
			static_cast<GLsizeiptr>(16384 * sizeof(glm::mat4)), static_cast<GLsizeiptr>(sizeof(glm::mat4)), Global.gfx_bufferstreaming, &identity, static_cast<GLsizeiptr>(sizeof(identity)));
	}
	WriteLog(std::string("Model data streaming: ") + (model_ubo->streamed() ? "persistently mapped buffers" : "glBufferSubData"));
	// multi draws of the instances drawn from the gpu culling (Render_Instances_Indirect()): the model blocks of their draws in a
	// storage buffer bound at whole blocks, and the variants of the programs which read them there. without them the draws are
	// made one by one, as before
	try
	{
		GLint alignment{256};
		glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &alignment);
		if ((alignment <= 0) || (sizeof(gl::model_ubs) % static_cast<std::size_t>(alignment) != 0))
			throw gl::shader_exception("storage buffer offset alignment of " + std::to_string(alignment) + " bytes");
		std::string const defines{"#define MODEL_DRAWS 1\n"};
		m_vertex_multidraw_shader = std::make_unique<gl::shader>("vertex.vert", defines);
		gl::shader prepassvertex("depthprepass.vert", defines);
		gl::shader prepassfragment("depthprepass.frag", defines);
		gl::shader prepassalphafragment("depthprepass_alpha.frag", defines);
		m_prepass_multidraw_shader = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{prepassvertex, prepassfragment});
		m_prepass_alpha_multidraw_shader = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{prepassvertex, prepassalphafragment});
		gl::shader shadowvertex("simpleuv.vert", defines);
		gl::shader shadowfragment("shadowmap.frag", defines);
		gl::shader alphashadowfragment("alphashadowmap.frag", defines);
		m_shadow_multidraw_shader = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shadowvertex, shadowfragment});
		m_alpha_shadow_multidraw_shader = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shadowvertex, alphashadowfragment});
		m_drawblockstream = std::make_unique<gl::frame_stream>(
			static_cast<GLsizeiptr>(4096 * sizeof(gl::model_ubs)), static_cast<GLsizeiptr>(sizeof(gl::model_ubs)), Global.gfx_bufferstreaming);
		// and the commands of the multi draws of the recorded depth pass of the scene (Render_Scene_Prepass())
		m_scenecommandstream = std::make_unique<gl::frame_stream>(
			static_cast<GLsizeiptr>(4096 * sizeof(gl::instance_draw_command)), static_cast<GLsizeiptr>(sizeof(gl::instance_draw_command)), Global.gfx_bufferstreaming);
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("multi draws of the instances not available: " + std::string(e.what()));
		m_vertex_multidraw_shader.reset();
		m_prepass_multidraw_shader.reset();
		m_prepass_alpha_multidraw_shader.reset();
		m_shadow_multidraw_shader.reset();
		m_alpha_shadow_multidraw_shader.reset();
		m_drawblockstream.reset();
		m_scenecommandstream.reset();
	}

	try
	{
		m_sky.init();
		m_weather.init();
		m_instancecull.init();
		for (auto &shadowcull : m_shadowcull)
			shadowcull.init(&m_instancecull);
		m_hiz.init();
		m_shadowfit.init();
		m_exposure.init();
		m_ao.init();
		m_localshadows.init();
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("invalid shader: " + std::string(e.what()));
		return false;
	}

	// better initialize with 0 to not crash driver/whole system
	// when we forget
	memset(&light_data, 0, sizeof(light_data));
	memset(&model_ubs, 0, sizeof(model_ubs));
	memset(&scene_ubs, 0, sizeof(scene_ubs));

	light_ssbo->update(light_data);
	model_ubo->update(model_ubs);
	scene_ubo->update(scene_ubs);

	Bind_Instance_Stream();

	int samples = 1 << Global.iMultisampling;
	if (samples > 1)
		glEnable(GL_MULTISAMPLE);

	m_pfx_motionblur = std::make_unique<gl::postfx>("motionblur");
	if (Global.gfx_taa())
		m_pfx_taa = std::make_unique<gl::postfx>("taa");
	m_pfx_tonemapping = std::make_unique<gl::postfx>("tonemapping");
    m_pfx_chromaticaberration = std::make_unique<gl::postfx>( "chromaticaberration" );

	m_pfx_ssao       = std::make_unique<gl::postfx>("ssao");
	m_pfx_ssao_blur  = std::make_unique<gl::postfx>("ssao_blur");

	// Generate hemisphere kernel (z > 0 = toward surface normal)
	std::uniform_real_distribution<float> rnd(0.0f, 1.0f);
	std::default_random_engine gen(42); // NOSONAR fixed seed on purpose, SSAO kernel has to be identical every run
	for (int i = 0; i < 32; i++) {
		glm::vec3 s(rnd(gen)*2.0f-1.0f, rnd(gen)*2.0f-1.0f, rnd(gen));
		s = glm::normalize(s) * rnd(gen);
		float scale = float(i) / 32.0f;
		scale = glm::mix(0.1f, 1.0f, scale * scale); // accelerating interpolant
		m_ssao_kernel[i] = s * scale;
	}

	// Generate 4x4 noise texture (random rotation vectors in XY)
	std::vector<glm::vec3> noise_data;
	for (int i = 0; i < 16; i++)
		noise_data.push_back(glm::normalize(glm::vec3(rnd(gen)*2.0f-1.0f, rnd(gen)*2.0f-1.0f, 0.0f)));

	// noise texture (4x4 random rotation vectors in tangent space)
	// noise texture (4x4 random rotation vectors in tangent space)
	m_ssao_noise_tex = std::make_unique<opengl_texture>();
	m_ssao_noise_tex->alloc_rendertarget(GL_RGB32F, GL_RGB, 4, 4, 1, 1, GL_REPEAT);
	m_ssao_noise_tex->bind(0);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 4, 4, GL_RGB, GL_FLOAT, noise_data.data());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	m_empty_cubemap = std::make_unique<gl::cubemap>();
	m_empty_cubemap->alloc(Global.gfx_format_color, 16, 16, GL_RGB, GL_FLOAT);

	m_viewports.push_back(std::make_unique<viewport_config>());
	viewport_config &default_viewport = *m_viewports.front().get();
	default_viewport.width = Global.gfx_framebuffer_width;
	default_viewport.height = Global.gfx_framebuffer_height;
	default_viewport.main = true;
    default_viewport.shadow = true;
	default_viewport.window = m_window;
	default_viewport.draw_range = 1.0f;

    if (Global.vr) {
        vr = vr_interface_factory::get_instance()->create(Global.vr_backend);
        crashreport_add_info("vr_backend", Global.vr_backend);
    }

    if (vr) {
        glm::ivec2 target_size = vr->get_target_size();
        WriteLog("using vr rendertarget: " + glm::to_string(target_size));

        // hijack main window for left eye
        default_viewport.width = target_size.x;
        default_viewport.height = target_size.y;
        default_viewport.custom_backbuffer = true;
        default_viewport.proj_type = viewport_config::vr_left;

        // create right eye viewport
        m_viewports.push_back(std::make_unique<viewport_config>());
        m_viewports.back()->width = target_size.x;
        m_viewports.back()->height = target_size.y;
        m_viewports.back()->window = m_window; // we can reuse context
        m_viewports.back()->real_window = false; // but don't draw anything to it
        m_viewports.back()->custom_backbuffer = true;
        m_viewports.back()->proj_type = viewport_config::vr_right;
        m_viewports.back()->draw_range = 1.0f;
        m_viewports.back()->shadow = true;

        if (!init_viewport(*m_viewports.back()))
            return false;
    }

	if (!init_viewport(default_viewport))
		return false;
	glfwMakeContextCurrent(m_window);

	if (Global.gfx_shadowmap_enabled)
	{
		m_shadow_fb = std::make_unique<gl::framebuffer>();
		m_shadow_tex = std::make_unique<opengl_texture>();
		m_shadow_tex->alloc_rendertarget(Global.gfx_format_depth, GL_DEPTH_COMPONENT, m_shadowbuffersize, m_shadowbuffersize, m_shadowpass.size());
		// only the shadow sampler reads through the depth comparison; depth targets read as plain values must not have it
		glTextureParameteri(m_shadow_tex->id, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
		// stand-in for the shadow map, for the shaders which are drawn without one: a shadow sampler with no depth texture
		// to read from is undefined. the stand-in is cleared to the far end, so it shadows nothing
		m_shadow_empty_tex = std::make_unique<opengl_texture>();
		m_shadow_empty_tex->alloc_rendertarget(Global.gfx_format_depth, GL_DEPTH_COMPONENT, 1, 1, m_shadowpass.size());
		glTextureParameteri(m_shadow_empty_tex->id, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
		float const farthest{0.f};
		glClearTexImage(m_shadow_empty_tex->id, 0, GL_DEPTH_COMPONENT, GL_FLOAT, &farthest);
		m_shadow_fb->attach(*m_shadow_tex, GL_DEPTH_ATTACHMENT, 0);
        m_shadow_fb->setup_drawing(0);

        if( !m_shadow_fb->is_complete() ) {
            ErrorLog( "shadow framebuffer setup failed" );
            return false;
        }

		WriteLog("shadows enabled");
	}

    if (Global.gfx_envmap_enabled)
	{
		m_env_rb = std::make_unique<gl::renderbuffer>();
		m_env_rb->alloc(Global.gfx_format_depth, gl::ENVMAP_SIZE, gl::ENVMAP_SIZE);
		m_env_tex = std::make_unique<gl::cubemap>();
		m_env_tex->alloc(Global.gfx_format_color, gl::ENVMAP_SIZE, gl::ENVMAP_SIZE, GL_RGB, GL_FLOAT);

		m_env_fb = std::make_unique<gl::framebuffer>();

		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		for (int i = 0; i < 6; i++)
		{
			m_env_fb->attach(*m_empty_cubemap, i, GL_COLOR_ATTACHMENT0);
			m_env_fb->clear(GL_COLOR_BUFFER_BIT);
		}

		m_env_fb->detach(GL_COLOR_ATTACHMENT0);
		m_env_fb->attach(*m_env_rb, GL_DEPTH_ATTACHMENT);

		WriteLog("envmap enabled");
	}

	m_pick_tex = std::make_unique<opengl_texture>();
	m_pick_tex->alloc_rendertarget(GL_RGB8, GL_RGB, EU07_PICKBUFFERSIZE, EU07_PICKBUFFERSIZE);
	m_pick_rb = std::make_unique<gl::renderbuffer>();
	m_pick_rb->alloc(Global.gfx_format_depth, EU07_PICKBUFFERSIZE, EU07_PICKBUFFERSIZE);
	m_pick_fb = std::make_unique<gl::framebuffer>();
	m_pick_fb->attach(*m_pick_tex, GL_COLOR_ATTACHMENT0);
	m_pick_fb->attach(*m_pick_rb, GL_DEPTH_ATTACHMENT);

    if( !m_pick_fb->is_complete() ) {
        ErrorLog( "pick framebuffer setup failed" );
        return false;
    }

	m_picking_pbo = std::make_unique<gl::pbo>();
	m_picking_node_pbo = std::make_unique<gl::pbo>();

    GLenum depth_pointer_format = Global.gfx_format_depth;
    if (Global.gfx_skippipeline)
    {
        gl::framebuffer::unbind();
        GLint bits, type;
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_DEPTH, GL_FRAMEBUFFER_ATTACHMENT_DEPTH_SIZE, &bits);
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_DEPTH, GL_FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE, &type);
        if (type == GL_FLOAT && bits == 32)
            depth_pointer_format = GL_DEPTH_COMPONENT32F;
        else if (bits == 16)
            depth_pointer_format = GL_DEPTH_COMPONENT16;
        else if (bits == 24)
            depth_pointer_format = GL_DEPTH_COMPONENT24;
        else if (bits == 32)
            depth_pointer_format = GL_DEPTH_COMPONENT32;
    }

	m_depth_pointer_pbo = std::make_unique<gl::pbo>();
	if (Global.iMultisampling)
	{
		m_depth_pointer_rb = std::make_unique<gl::renderbuffer>();
        m_depth_pointer_rb->alloc(depth_pointer_format, 1, 1);

		m_depth_pointer_fb = std::make_unique<gl::framebuffer>();
		m_depth_pointer_fb->attach(*m_depth_pointer_rb, GL_DEPTH_ATTACHMENT);
        m_depth_pointer_fb->setup_drawing(0);

        if( !m_depth_pointer_fb->is_complete() ) {
            ErrorLog( "depth pointer framebuffer setup failed" );
            return false;
        }
	}

    m_timequery.emplace(gl::query::TIME_ELAPSED);
    m_timequery->begin();

	WriteLog("picking objects created");

	if (Global.gfx_threads != 0)
	{
		// the threads of the renderers (gfx.threads) record the command buffers of the vulkan renderer; the opengl renderer walks and
		// draws the scene on the main thread
		WriteLog("Gfx Renderer: gfx.threads without effect with opengl");
	}
	WriteLog("Gfx Renderer: setup complete");

    return true;
}

void opengl46_renderer::Shutdown()
{
    vr.reset();
}

bool opengl46_renderer::AddViewport(const global_settings::extraviewport_config &conf)
{
    viewport_config *vp;

    if (conf.monitor == "MAIN") {
        vp = m_viewports.front().get();
    }
    else {
        m_viewports.push_back(std::make_unique<viewport_config>());
        vp = m_viewports.back().get();
        vp->window = Application.window(-1, true, conf.width, conf.height, Application.find_monitor(conf.monitor));
    }

    vp->width = conf.width;
    vp->height = conf.height;
    vp->projection = conf.projection;
    vp->proj_type = viewport_config::custom;
    vp->draw_range = conf.draw_range;

    bool ret = init_viewport(*vp);
	glfwMakeContextCurrent(m_window);

	return ret;
}

bool opengl46_renderer::init_viewport(viewport_config &vp)
{
    if (vp.initialized)
        return true;

	glfwMakeContextCurrent(vp.window);

	WriteLog("init viewport: " + std::to_string(vp.width) + " x " + std::to_string(vp.height));

    if (vp.real_window)
        glfwSwapInterval( Global.VSync ? 1 : 0 );

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);

    glClearColor( 51.0f / 255.f, 102.0f / 255.f, 85.0f / 255.f, 1.f ); // initial background Color

	glFrontFace(GL_CCW);
	glEnable(GL_CULL_FACE);

	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glClearDepth(0.0f);

	glDepthFunc(GL_GEQUAL);

	glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);

	glEnable(GL_PROGRAM_POINT_SIZE);

	if (!gl::vao::use_vao)
	{
		GLuint v;
		glGenVertexArrays(1, &v);
		glBindVertexArray(v);
	}

	int samples = 1 << Global.iMultisampling;

	model_ubo->bind_uniform();
	scene_ubo->bind_uniform();
	light_ssbo->bind_base();
	m_sky.bind();
	m_weather.bind();
	m_exposure.bind();

	vp.clusters = std::make_unique<gl::cluster_grid>();
	// the material shaders declare the cluster blocks whether they read them or not, and drawing with an active storage
	// block without a buffer behind it is undefined. the context starts with these buffers in place; every pass which
	// uses a grid binds its own
	vp.clusters->bind();

	if (!Global.gfx_skippipeline)
	{
		vp.msaa_rbc = std::make_unique<gl::renderbuffer>();
		vp.msaa_rbc->alloc(Global.gfx_format_color, vp.width, vp.height, samples);

		vp.msaa_rbd = std::make_unique<gl::renderbuffer>();
        vp.msaa_rbd->alloc(Global.gfx_format_depth, vp.width, vp.height, samples);

		vp.msaa_fb = std::make_unique<gl::framebuffer>();
		vp.msaa_fb->attach(*vp.msaa_rbc, GL_COLOR_ATTACHMENT0);
		vp.msaa_fb->attach(*vp.msaa_rbd, GL_DEPTH_ATTACHMENT);

		if (Global.gfx_postfx_chromaticaberration_enabled || Global.gfx_motion_vectors() || Global.gfx_postfx_ao != 0)
		{
			vp.main_tex = std::make_unique<opengl_texture>();
			vp.main_tex->alloc_rendertarget(Global.gfx_format_color, GL_RGB, vp.width, vp.height, 1, 1, GL_CLAMP_TO_EDGE);

			vp.main_texd = std::make_unique<opengl_texture>();
			vp.main_texd->alloc_rendertarget(Global.gfx_format_depth, GL_DEPTH_COMPONENT, vp.width, vp.height, 1, 1, GL_CLAMP_TO_EDGE);

			vp.main_fb = std::make_unique<gl::framebuffer>();
			vp.main_fb->attach(*vp.main_tex, GL_COLOR_ATTACHMENT0);
			vp.main_fb->attach(*vp.main_texd, GL_DEPTH_ATTACHMENT);

			if (Global.gfx_motion_vectors())
			{
				vp.main_texv = std::make_unique<opengl_texture>();
				vp.main_texv->alloc_rendertarget(Global.gfx_postfx_motionblur_format, GL_RG, vp.width, vp.height);
				vp.main_fb->attach(*vp.main_texv, GL_COLOR_ATTACHMENT1);
			}

			if( !vp.main_fb->is_complete() ) {
				ErrorLog( "main framebuffer setup failed" );
				return false;
			}
		}

		if (Global.gfx_motion_vectors())
		{
			vp.msaa_rbv = std::make_unique<gl::renderbuffer>();
			vp.msaa_rbv->alloc(Global.gfx_postfx_motionblur_format, vp.width, vp.height, samples);
			vp.msaa_fb->attach(*vp.msaa_rbv, GL_COLOR_ATTACHMENT1);

			if (Global.gfx_postfx_motionblur_enabled)
				WriteLog("motion blur enabled");
		}

        if( !vp.msaa_fb->is_complete() ) {
            ErrorLog( "msaa framebuffer setup failed" );
            return false;
        }

		vp.main2_tex = std::make_unique<opengl_texture>();
		vp.main2_tex->alloc_rendertarget(Global.gfx_format_color, GL_RGB, vp.width, vp.height);

		vp.main2_fb = std::make_unique<gl::framebuffer>();
		vp.main2_fb->attach(*vp.main2_tex, GL_COLOR_ATTACHMENT0);

		if (Global.gfx_postfx_ao != 0)
		{
			int ssao_w = std::max(1, vp.width  / 2);
			int ssao_h = std::max(1, vp.height / 2);

			vp.ssao_tex = std::make_unique<opengl_texture>();
			vp.ssao_tex->alloc_rendertarget(GL_R8, GL_RED, ssao_w, ssao_h, 1, 1, GL_CLAMP_TO_EDGE);
			vp.ssao_fb = std::make_unique<gl::framebuffer>();
			vp.ssao_fb->attach(*vp.ssao_tex, GL_COLOR_ATTACHMENT0);
			if (!vp.ssao_fb->is_complete()) { ErrorLog("ssao framebuffer setup failed"); return false; }

			vp.ssao_blur_tex = std::make_unique<opengl_texture>();
			vp.ssao_blur_tex->alloc_rendertarget(GL_R8, GL_RED, ssao_w, ssao_h, 1, 1, GL_CLAMP_TO_EDGE);
			vp.ssao_blur_fb = std::make_unique<gl::framebuffer>();
			vp.ssao_blur_fb->attach(*vp.ssao_blur_tex, GL_COLOR_ATTACHMENT0);
			if (!vp.ssao_blur_fb->is_complete()) { ErrorLog("ssao blur framebuffer setup failed"); return false; }

			if (Global.gfx_motion_vectors())
			{
				vp.ao_apply_fb = std::make_unique<gl::framebuffer>();
				vp.ao_apply_fb->attach(*vp.main_tex, GL_COLOR_ATTACHMENT0);
				if (!vp.ao_apply_fb->is_complete()) { ErrorLog("ambient occlusion framebuffer setup failed"); return false; }
			}
		}

		if (!vp.main2_fb->is_complete())
			return false;

		// made whether the bloom is on or not, it can be switched on in the debug window
		vp.bloom = opengl46_exposure::make_bloom_chain(vp.width, vp.height);
		if (!vp.bloom)
			ErrorLog("bloom framebuffer setup failed, no bloom");

		// the two images of the history of the temporal anti-aliasing, of the main view (not the eyes of the vr)
		if (Global.gfx_taa() && vp.main && (vp.proj_type != viewport_config::vr_left) && (vp.proj_type != viewport_config::vr_right))
		{
			for (std::size_t i = 0; i < vp.taa_tex.size(); ++i)
			{
				vp.taa_tex[i] = std::make_unique<opengl_texture>();
				vp.taa_tex[i]->alloc_rendertarget(GL_RGBA16F, GL_RGBA, vp.width, vp.height, 1, 1, GL_CLAMP_TO_EDGE);
				vp.taa_fb[i] = std::make_unique<gl::framebuffer>();
				vp.taa_fb[i]->attach(*vp.taa_tex[i], GL_COLOR_ATTACHMENT0);
				if (!vp.taa_fb[i]->is_complete())
				{
					ErrorLog("temporal anti-aliasing framebuffer setup failed");
					return false;
				}
			}
			WriteLog("temporal anti-aliasing enabled");
		}
	}

    if (vp.custom_backbuffer)
    {
        vp.backbuffer_tex = std::make_unique<opengl_texture>();
        vp.backbuffer_tex->alloc_rendertarget(GL_SRGB8, GL_RGB, vp.width, vp.height);

        vp.backbuffer_fb = std::make_unique<gl::framebuffer>();
        vp.backbuffer_fb->attach(*vp.backbuffer_tex, GL_COLOR_ATTACHMENT0);
        if (!vp.backbuffer_fb->is_complete())
            return false;
    }

    vp.initialized = true;
	return true;
}

std::unique_ptr<gl::program> opengl46_renderer::make_shader(std::string v, std::string f)
{
	gl::shader vert(v);
	gl::shader frag(f);
	gl::program *prog = new gl::program({vert, frag});
	return std::unique_ptr<gl::program>(prog);
}

bool opengl46_renderer::Render()
{
	// the texture names bound before may have been given to other textures since the last frame
	opengl_texture::reset_unit_cache();
	Timer::subsystem.gfx_total.start();

	std::optional<int64_t> result = m_timequery->result();
	if (result) {
		m_gllasttime = *result;
		m_timequery->begin();
	}
	// the context of the main view is current here, the one of the previous frame's end
	m_gputimeline.begin("frame setup");
	m_cputimeline.begin("frame setup");

	// fetch simulation data
	if (simulation::is_ready)
	{
		m_sunlight = Global.DayLight;
		// quantize sun angle to reduce shadow crawl
		auto const quantizationstep{0.004f};
		m_sunlight.direction = glm::normalize(quantizationstep * glm::roundEven(m_sunlight.direction * (1.f / quantizationstep)));
		// the light of the sky for the render: colours of the sun, ambient light and fog from the atmosphere where it is
		// used, the directional part of the ambient light
		m_fogcolor = Global.FogColor;
		// the volumetric clouds of the previous frame tell the sky whether to look overcast and dim the sun itself
		m_sky.set_clouds(m_weather.clouds_drawn(), m_weather.cloud_shadows());
		m_sky.update(m_sunlight, m_fogcolor, simulation::Environment.skydome(), simulation::Environment.sun().getDirection(), simulation::Environment.sun().getAngle());
		// fog, clouds and stars of the weather, in the light of the sky worked out above
		if (false == Global.gfx_skiprendering)
			m_weather.update(m_sky, -m_sunlight.direction, glm::vec3{m_sunlight.diffuse} * simulation::Environment.light_intensity(), glm::vec3{m_sunlight.ambient}, m_fogcolor, Global.pCamera.Pos);
		// lamps of models and vehicles around the camera and the cab lamp, for the views of the frame to take into their light buffers
		if ((false == Global.gfx_skiprendering) && (simulation::Region != nullptr))
			m_locallights.gather(Global.pCamera.Pos, *simulation::Region, m_geometry, m_framestamp);
	}
	else
	{
		// a scenery is being loaded: the cells, instances and models the lights were taken from may be gone
		m_locallights.reset();
		m_localshadows.reset_lamps();
		m_localshadows.reset_headlights();
		// and so may the tracks the sleeper records were made for, and the models drawn on their own
		Clear_Indirect_Records();
	}
	// generate new frame
    opengl_texture::reset_unit_cache();
	// textures of new heightmap terrain layers go to the arrays the terrain shader reads them from
	m_terrainmaterials.update();
	if( ( EditorModeFlag ) || ( m_nodestateversion != scene::node_state_version ) ) {
		// the editor moves and resizes model instances in place, events show and hide them and make them drawn on
		// their own: the instance records are out of date
		++m_instancerecordsversion;
		m_nodestateversion = scene::node_state_version;
	}
	// the sleeper records of tracks out of sight for a while, and the records of the models drawn on their own
	Expire_Indirect_Records();
	// scenery lamps which get shadows in this frame. their maps hold the scenery as of the version of the instance records
	if (simulation::is_ready && (false == Global.gfx_skiprendering))
		m_localshadows.update_lamps(m_locallights.lights(), Global.pCamera.Pos, m_instancerecordsversion, m_framestamp);
	// instanced draws of the frame place their matrices in the next of the instance buffers, the one used the longest ago;
	// the same for the model blocks of the draws
	m_instancestream->next_frame();
	Bind_Instance_Stream();
	model_ubo->next_frame();
	if (m_drawblockstream)
		m_drawblockstream->next_frame();
	if (m_scenecommandstream)
		m_scenecommandstream->next_frame();

	m_renderpass.draw_mode = rendermode::none; // force setup anew
	// the vehicles which moved in the last frame (motion blur)
	m_vehiclesmoved = m_vehiclesmoving;
	m_vehiclesmoving = false;
	m_renderpass.draw_stats = debug_stats();
    m_geometry.primitives_count() = 0;

    if (vr)
        vr->begin_frame();

	for (auto &viewport : m_viewports) {
		Render_pass(*viewport, rendermode::color);
	}

	glfwMakeContextCurrent(m_window);
	model_ubo->bind_uniform();
	Bind_Instance_Stream();

	gl::framebuffer::unbind();

	m_current_viewport = &(*m_viewports.front());
	m_gputimeline.end();
	m_cputimeline.end();
/*
	m_debugtimestext += "cpu: " + to_string(Timer::subsystem.gfx_color.average(), 2) + " ms (" + std::to_string(m_cellqueue.size()) + " sectors)\n" +=
	    "cpu swap: " + to_string(Timer::subsystem.gfx_swap.average(), 2) + " ms\n" += "uilayer: " + to_string(Timer::subsystem.gfx_gui.average(), 2) + "ms\n" +=
	    "mainloop total: " + to_string(Timer::subsystem.mainloop_total.average(), 2) + "ms\n";

*/
	m_timequery->end();

/*
	if (m_gllasttime)
		m_debugtimestext += "gpu: " + to_string((double)(m_gllasttime / 1000ULL) / 1000.0, 3) + "ms";
*/

    ++m_framestamp;
    Timer::subsystem.gfx_total.stop();

    // swapbuffers() could unbind current buffers so we prepare for it on our end
    gfx::opengl46_vaogeometrybank::reset();

    return true; // for now always succeed
}

void opengl46_renderer::SwapBuffers()
{
	Timer::subsystem.gfx_swap.start();

	for (auto &viewport : m_viewports) {
        if (viewport->window && viewport->real_window)
			glfwSwapBuffers(viewport->window);
	}

    if (vr)
        vr->finish_frame();

	Timer::subsystem.gfx_swap.stop();

    // the debug data, in sections for the panel. info_times() and info_stats() hold the same, joined
    m_debugsections.clear();
    Update_Memory_Info();
    // a line of a table of times: the name indented and padded to a column, then the time(s)
    auto const timeline = []( int const Indent, std::string const &Name, std::string const &Values ) {
        auto line { std::string( static_cast<std::size_t>( Indent ), ' ' ) + Name };
        line.resize( std::max<std::size_t>( line.size() + 1, 44 ), ' ' );
        return line + Values + "\n";
    };
    auto const ms = []( double const Value, int const Precision = 2 ) {
        auto text { to_string( Value, Precision ) };
        return std::string( std::max<int>( 0, 7 - static_cast<int>( text.size() ) ), ' ' ) + text;
    };

    // the whole frame of the main loop on the cpu and its parts, the simulation with its own; what's left is measured by none
    {
        auto const &timers { Timer::subsystem };
        auto const mainloop { timers.mainloop_total.average() };
        auto const simulation { timers.sim_total.average() };
        auto const render { timers.gfx_total.average() };
        auto const swap { timers.gfx_swap.average() };
        auto const simulationparts {
            timers.sim_scripting.average() + timers.sim_environment.average() + timers.sim_precipitation.average() + timers.sim_dynamics.average()
            + timers.sim_secondary.average() + timers.sim_trains.average() + timers.sim_events.average() + timers.sim_lights.average()
            + timers.sim_commands.average() + timers.sim_ui.average() + timers.sim_camera.average() };
        auto const loopparts {
            timers.loop_input.average() + timers.loop_ui_frame.average() + simulation + timers.loop_audio.average() + timers.loop_particles.average() + timers.loop_renderer_update.average()
            + timers.loop_terrain.average() + timers.loop_tasks.average() + render + swap };
        std::string text;
        text += timeline( 0, "main loop", ms( mainloop ) + " ms (" + std::to_string( static_cast<int>( std::round( 1000.f / std::max( mainloop, 0.001f ) ) ) ) + " fps)" );
        text += timeline( 1, "input, head tracking", ms( timers.loop_input.average() ) );
        text += timeline( 1, "start of the ui frame", ms( timers.loop_ui_frame.average() ) );
        text += timeline( 1, "simulation", ms( simulation ) );
        text += timeline( 2, "clocks, scripting", ms( timers.sim_scripting.average() ) );
        text += timeline( 2, "environment", ms( timers.sim_environment.average() + timers.sim_precipitation.average() ) );
        text += timeline( 2, "vehicles (physics, ai)", ms( timers.sim_dynamics.average() ) );
        {
            auto const &counts { timers.vehicleslast };
            auto const dynamicsparts {
                timers.sim_animations.average() + timers.sim_powergrid.average() + timers.sim_roads.average()
                + timers.vehicles_prepare.average() + timers.vehicles_forces.average() + timers.vehicles_update.average() };
            text += timeline( 3, "scenery animations", ms( timers.sim_animations.average() ) );
            text += timeline( 3, "power grid", ms( timers.sim_powergrid.average() ) );
            text += timeline( 3, "road junctions, spawn points", ms( timers.sim_roads.average() ) );
            text += timeline( 3, "wires, constants, neighbours", ms( timers.vehicles_prepare.average() ) );
            text += timeline( 3, "forces, " + std::to_string( counts.steps ) + " steps", ms( timers.vehicles_forces.average() ) );
            text += timeline( 3, "vehicle update", ms( timers.vehicles_update.average() ) );
            text += timeline( 4, "of it ai drivers", ms( timers.vehicles_ai.average() ) );
            text += timeline( 3, "rest", ms( timers.sim_dynamics.average() - dynamicsparts ) );
            text += timeline(
                3,
                std::to_string( counts.all ) + " vehicles: " + std::to_string( counts.enabled ) + " enabled, " + std::to_string( counts.active ) + " active, "
                    + std::to_string( counts.road ) + " road, " + std::to_string( counts.ai ) + " ai",
                "" );
            // by kind: how many, standing still, the time of the forces and of the update
            auto const count = []( unsigned int const Value ) {
                auto text { std::to_string( Value ) };
                return std::string( std::max<int>( 0, 6 - static_cast<int>( text.size() ) ), ' ' ) + text; };
            auto const nearlimit { std::to_string( static_cast<int>( Timer::subsystem.VEHICLE_NEAR ) ) + " m" };
            text += timeline(
                3,
                ( counts.threads > 1 ?
                    "steps on " + std::to_string( counts.threads ) + " of " + std::to_string( counts.capacity ) + " threads: " + std::to_string( counts.groups ) + " groups, largest "
                        + std::to_string( counts.largestgroup ) + ", " + std::to_string( counts.maingroups ) + " in the main thread (forces by kind not timed)" :
                    "steps on 1 thread of " + std::to_string( counts.capacity ) + ( counts.singlethread.empty() ? std::string{} : ": " + counts.singlethread ) ),
                "" );
            char const *const kindnames[] = { "rail, within ", "rail, farther", "road, within ", "road, farther" };
            text += timeline( 3, "by kind (near: " + nearlimit + ")", " count  stand  forces  update" );
            for( std::size_t kind = 0; kind < 4; ++kind ) {
                text += timeline(
                    4, kindnames[ kind ],
                    count( counts.kind[ kind ] ) + " " + count( counts.standing[ kind ] ) + " " + ms( timers.vehicles_forces_kind[ kind ].average() ) + " "
                        + ms( timers.vehicles_update_kind[ kind ].average() ) );
            }
            auto const &awake { counts.awake };
            text += timeline(
                3,
                std::to_string( awake.vehicles ) + " rail standing with physics on: cab " + std::to_string( awake.cab ) + ", speed " + std::to_string( awake.speed )
                    + ", acceleration " + std::to_string( awake.acceleration ) + ", switching " + std::to_string( awake.switching ) + ", ezt/dmu "
                    + std::to_string( awake.multipleunit ) + ", neighbour " + std::to_string( awake.neighbour ),
                "" );
            // what resets the switching time of the vehicles, in the last second
            auto const &switching { timers.switching.last() };
            using source = Timer::switching_counter::source;
            text += timeline(
                3,
                "switching per second: compressor on " + std::to_string( switching.events[ source::compressor_on ] ) + ", off "
                    + std::to_string( switching.events[ source::compressor_off ] ) + ", mains " + std::to_string( switching.events[ source::mains ] ) + ", diesel "
                    + std::to_string( switching.events[ source::diesel ] ),
                "" );
            text += timeline(
                3,
                "physics woken per second: brake pipe " + std::to_string( switching.events[ source::wake_mainpipe ] ) + ", main reservoir pipe "
                    + std::to_string( switching.events[ source::wake_scndpipe ] ) + ", command " + std::to_string( switching.events[ source::wake_command ] )
                    + ", speed " + std::to_string( switching.events[ source::wake_speed ] ) + ", acceleration "
                    + std::to_string( switching.events[ source::wake_acceleration ] ) + ", neighbour " + std::to_string( switching.events[ source::wake_neighbour ] )
                    + ", cab/ezt " + std::to_string( switching.events[ source::wake_rest ] ) + ", other " + std::to_string( switching.events[ source::wake ] ),
                "" );
            if( switching.busiestevents > 0 ) {
                text += timeline( 3, "most often: " + switching.busiest + " (" + std::to_string( switching.busiestevents ) + ")", "" );
            }
        }
        text += timeline( 2, "secondary steps", ms( timers.sim_secondary.average() ) );
        text += timeline( 2, "trains (cabs)", ms( timers.sim_trains.average() ) );
        text += timeline( 2, "events", ms( timers.sim_events.average() ) );
        text += timeline( 2, "lights", ms( timers.sim_lights.average() ) );
        text += timeline( 2, "commands", ms( timers.sim_commands.average() ) );
        text += timeline( 2, "ui panels, tooltips (50 Hz)", ms( timers.sim_ui.average() ) );
        text += timeline( 2, "camera", ms( timers.sim_camera.average() ) );
        text += timeline( 2, "rest", ms( simulation - simulationparts ) );
        text += timeline( 1, "sounds", ms( timers.loop_audio.average() ) );
        text += timeline( 1, "particles", ms( timers.loop_particles.average() ) );
        text += timeline( 1, "renderer update", ms( timers.loop_renderer_update.average() ) );
        text += timeline( 1, "terrain streaming", ms( timers.loop_terrain.average() ) );
        text += timeline( 1, "tasks", ms( timers.loop_tasks.average() ) );
        text += timeline( 1, "render", ms( render ) );
        text += timeline( 1, "swap", ms( swap ) );
        text += timeline( 1, "rest", ms( mainloop - loopparts ) );
        m_debugsections.push_back( { "CPU: main loop", std::move( text ), true } );
    }

    // the parts of the frame of the renderer, cpu and gpu side by side. the gpu ones follow one another and add up to the frame;
    // while the gpu waits for the cpu, the wait goes to the part drawn next
    {
        std::string text;
        text += timeline( 0, "part", "    cpu      gpu  ms" );
        auto const &gpuparts { m_gputimeline.result() };
        auto cputotal { 0.f };
        for( auto const &part : m_cputimeline.parts ) {
            cputotal += part.time;
            std::string gpu { "        -" };
            for( auto const &gpupart : gpuparts ) {
                if( std::strcmp( gpupart.name, part.name ) == 0 ) {
                    gpu = "  " + ms( static_cast<double>( gpupart.time ) / 1000000.0, 3 );
                    break;
                }
            }
            text += timeline( 1, part.name, ms( part.time ) + gpu );
        }
        // parts the gpu measured and the cpu didn't, if any
        for( auto const &gpupart : gpuparts ) {
            if( false == m_cputimeline.time( gpupart.name ).has_value() ) {
                text += timeline( 1, gpupart.name, "      -  " + ms( static_cast<double>( gpupart.time ) / 1000000.0, 3 ) );
            }
        }
        text += timeline(
            0, "total",
            ms( cputotal ) + "  " + ( gpuparts.empty() ? std::string{ "      -" } : ms( static_cast<double>( m_gputimeline.total() ) / 1000000.0, 3 ) ) );
        text += "\n";
        text += timeline( 0, "cpu of the passes", "" );
        text += timeline( 1, "colour (" + std::to_string( m_cellqueue.size() ) + " sectors)", ms( Timer::subsystem.gfx_color.average() ) );
        if( Global.gfx_shadowmap_enabled ) {
            text += timeline( 1, "shadows", ms( Timer::subsystem.gfx_shadows.average() ) );
        }
        if( Global.gfx_envmap_enabled ) {
            text += timeline( 1, "reflections", ms( Timer::subsystem.gfx_reflections.average() ) );
        }
        text += timeline( 1, "ui", ms( Timer::subsystem.gfx_gui.average() ) );
        if( m_gllasttime ) {
            text += timeline( 0, "gpu frame (query)", ms( static_cast<double>( m_gllasttime / 1000ULL ) / 1000.0, 3 ) );
        }
        m_debugsections.push_back( { "Render: cpu / gpu by part", std::move( text ), true } );
    }

    // the steps below are timed each on its own on the gpu and can hold the tail of the work before them
    {
        std::string text;
        if (auto const clusterstime = m_clusterstimer.result())
            text += " clusters: " + to_string(static_cast<double>(*clusterstime) / 1000000.0, 3) + " ms\n";
        if (auto const culltime = m_cullstimer.result())
            text += " cull: " + to_string(static_cast<double>(*culltime) / 1000000.0, 3) + " ms\n";
        text += m_sky.info_times();
        text += m_weather.info_times();
        text += m_exposure.info_times();
        text += m_ao.info_times();
        text += m_instancecull.info_times();
        // the cullings of the other passes; the environment map is drawn now and then, its numbers are of its last update
        if( Global.gfx_gpucull_shadows ) {
            for( std::size_t cascade = 0; cascade < m_shadowcull.size(); ++cascade ) {
                text += m_shadowcull[ cascade ].info_times( "instance cull shadows " + std::to_string( cascade + 1 ) );
            }
        }
        text += Light_Shadow_Cull_Info( true );
        if( Global.gfx_depthprepass_scene || m_contactshadowslast ) {
            if( auto const scenedepthtime = m_scenedepthtimer.result() )
                text += " scene depth: " + to_string( static_cast<double>( *scenedepthtime ) / 1000000.0, 3 ) + " ms\n";
        }
        if( m_contactshadowslast ) {
            if( auto const contacttime = m_contactshadowstimer.result() )
                text += " contact shadows depth: " + to_string( static_cast<double>( *contacttime ) / 1000000.0, 3 ) + " ms\n";
        }
        if( Global.gfx_ssr > 0 ) {
            if( auto const historytime = m_ssrhistorytimer.result() )
                text += " reflections, image of the frame: " + to_string( static_cast<double>( *historytime ) / 1000000.0, 3 ) + " ms\n";
        }
        if( Global.gfx_gpucull == 1 ) {
            auto const twophase { m_instancecull.two_phase() };
            text += m_hiz.info_times( twophase );
            if( auto const prepasstime = m_prepasstimer.result() ) {
                text += " depth prepass: " + to_string( static_cast<double>( *prepasstime ) / 1000000.0, 3 ) + " ms";
                auto const firsttime { m_prepassfirsttimer.result() };
                if( twophase && firsttime )
                    text += " (+" + to_string( static_cast<double>( *firsttime ) / 1000000.0, 3 ) + " ms of the instances drawn first)";
                text += "\n";
            }
        }
        text += m_localshadows.info_times();
        if( Global.gfx_shadow_sdsm )
            text += m_shadowfit.info_times();
        text += m_gpuparticles.info_times();
        if( DebugModeFlag )
            text += m_textures.info();
        m_debugsections.push_back( { "GPU steps", std::move( text ), false } );
    }

    debug_stats shadowstats;
    for( auto const &shadowpass : m_shadowpass ) {
        shadowstats += shadowpass.draw_stats;
    }

    // the cascades of the sun shadows: where they end and the size of a texel of their maps, fixed or fitted to the depth
    std::string shadowcascadestext;
    if( Global.gfx_shadowmap_enabled && ( m_shadowbuffersize > 0 ) ) {
        auto const &fit { m_shadowfitstate };
        shadowcascadestext = " sun shadow cascades to";
        for( std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade ) {
            shadowcascadestext += " " + to_string( fit.splits[ cascade + 1 ], 1 );
        }
        shadowcascadestext += " m, texels";
        for( std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade ) {
            shadowcascadestext += " " + to_string( 100.f * fit.drawn[ cascade ] / m_shadowbuffersize, 1 );
        }
        shadowcascadestext += " cm";
        if( Global.gfx_shadow_sdsm ) {
            if( fit.fitted ) {
                shadowcascadestext += " (whole slices";
                for( std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade ) {
                    shadowcascadestext += " " + to_string( 100.f * fit.slice[ cascade ] / m_shadowbuffersize, 1 );
                }
                shadowcascadestext +=
                    "), fitted to pixels " + to_string( fit.depthmin, 2 ) + "-" + to_string( fit.depthmax, 1 ) + " m "
                    + std::to_string( fit.age ) + " frames ago, view turned " + to_string( fit.turn, 1 ) + " deg since";
            }
            else {
                shadowcascadestext += ", fixed: " + std::string{ fit.reason != nullptr ? fit.reason : "" };
            }
        }
        shadowcascadestext += "\n";
    }

    std::string statstext =
        "triangles: " + to_string( static_cast<int>(m_geometry.primitives_count()), 7 ) + "\n"
        + "vehicles:  " + to_string( m_colorpass.draw_stats.dynamics, 7 ) + " +" + to_string( shadowstats.dynamics, 7 )
        + " =" + to_string( m_colorpass.draw_stats.dynamics + shadowstats.dynamics, 7 ) + "\n"
        + "models:    " + to_string( m_colorpass.draw_stats.models, 7 ) + " +" + to_string( shadowstats.models, 7 )
        + " =" + to_string( m_colorpass.draw_stats.models + shadowstats.models, 7 ) + "\n"
        + "drawcalls: " + to_string( m_colorpass.draw_stats.drawcalls, 7 ) + " +" + to_string( shadowstats.drawcalls, 7 )
        + " =" + to_string( m_colorpass.draw_stats.drawcalls + shadowstats.drawcalls, 7 ) + "\n"
        + " instanced:" + to_string( m_colorpass.draw_stats.instances, 7 ) + " +" + to_string( shadowstats.instances, 7 )
        + " =" + to_string( m_colorpass.draw_stats.instances + shadowstats.instances, 7 )
        + " (" + std::to_string( m_colorpass.draw_stats.instanced_drawcalls + shadowstats.instanced_drawcalls ) + " batches)\n"
        + ( ( Global.gfx_depthprepass_scene || m_contactshadowslast ) ?
            ( m_scenereplayed ?
                " scene depth: " + std::to_string( m_sceneprepassdraws ) + " of " + std::to_string( m_scenerecordedparts ) + " recorded parts in "
                    + std::to_string( m_sceneprepasscalls ) + " calls"
                    + ( m_scenebeyondreach > 0 ? ", " + std::to_string( m_scenebeyondreach ) + " beyond the contact shadows" : std::string{} )
                    + ", cpu " + to_string( m_scenedepthcputime, 2 ) + " ms with the walk of the colour pass\n" :
                " scene depth: " + std::to_string( m_scenedepthstats.drawcalls ) + " draws, cpu " + to_string( m_scenedepthcputime, 2 ) + " ms\n" ) :
            std::string{} )
        + ( Global.gfx_contactshadows > 0 ?
            " contact shadows: " + std::string{ Global.gfx_contactshadows == 3 ? "high" : Global.gfx_contactshadows == 2 ? "medium" : "low" }
                + ( m_contactshadowslast ?
                    ", rays " + to_string( Global.gfx_contactshadows_length, 2 ) + " m within " + to_string( Global.gfx_contactshadows_range, 0 ) + " m\n" :
                    ", off in this view\n" ) :
            std::string{} )
        + shadowcascadestext
        + " instance buffer: " + std::to_string( m_instancestream->used() / static_cast<GLsizeiptr>( sizeof( glm::mat4 ) ) ) + " matrices, "
        + std::to_string( m_instancestream->size() / 1024 ) + " KiB\n"
        + " model blocks: " + std::to_string( model_ubo->updates() )
        + ( model_ubo->streamed() ?
            ", " + std::to_string( model_ubo->stream()->used() / 1024 ) + "/" + std::to_string( model_ubo->stream()->size() / 1024 ) + " KiB, streamed (waits "
                + std::to_string( model_ubo->stream()->waits() + m_instancestream->waits() ) + ", grown "
                + std::to_string( model_ubo->stream()->grows() + m_instancestream->grows() ) + ")\n" :
            ", glBufferSubData\n" )
        + ( ( m_drawblockstream && ( Global.gfx_gpucull == 1 ) && Global.gfx_gpucull_multidraw ) ?
            " multi draw blocks: " + std::to_string( m_drawblockstream->used() / static_cast<GLsizeiptr>( sizeof( gl::model_ubs ) ) ) + ", "
                + std::to_string( m_drawblockstream->size() / 1024 ) + " KiB (waits " + std::to_string( m_drawblockstream->waits() ) + ", grown "
                + std::to_string( m_drawblockstream->grows() ) + ")\n" :
            std::string{} )
        + " inst-pool:" + std::to_string( TAnimModel::s_instanceable_total )
        + "/" + std::to_string( TAnimModel::s_classified_total )
        + " rej(noModel/lights/anim/animSubM): "
        + std::to_string( TAnimModel::s_rejected_no_pmodel ) + "/"
        + std::to_string( TAnimModel::s_rejected_lights ) + "/"
        + std::to_string( TAnimModel::s_rejected_animlist ) + "/"
        + std::to_string( TAnimModel::s_rejected_animated_submodel ) + "\n"
        + " submodels:" + to_string( m_colorpass.draw_stats.submodels, 7 ) + " +" + to_string( shadowstats.submodels, 7 )
        + " =" + to_string( m_colorpass.draw_stats.submodels + shadowstats.submodels, 7 ) + "\n"
        + " paths:    " + to_string( m_colorpass.draw_stats.paths, 7 ) + " +" + to_string( shadowstats.paths, 7 )
        + " =" + to_string( m_colorpass.draw_stats.paths + shadowstats.paths, 7 ) + "\n"
        + " shapes:   " + to_string( m_colorpass.draw_stats.shapes, 7 ) + " +" + to_string( shadowstats.shapes, 7 )
        + " =" + to_string( m_colorpass.draw_stats.shapes + shadowstats.shapes, 7 ) + "\n"
        + " traction: " + to_string( m_colorpass.draw_stats.traction, 7 ) + "\n"
        + " lines:    " + to_string( m_colorpass.draw_stats.lines, 7 ) + "\n"
        + "particles: " + to_string( m_colorpass.draw_stats.particles, 7 )
        + "\nlights:    " + to_string( static_cast<int>( m_lights_lit ), 7 )
        + " (" + std::to_string( m_lights_dropped ) + " over capacity), "
        + ( Global.gfx_lights_clustered ? "clustered" : "first " + std::to_string( std::max( Global.DynamicLightCount, 0 ) ) )
        + "\n" + m_locallights.info()
        + m_localshadows.info()
        + m_gpuparticles.info()
        + m_windscreenwater.info();
    if( auto const cullstats = ( m_viewports.empty() || !( Global.gfx_lights_clustered || Global.gfx_lights_clusterheatmap ) ? std::nullopt : m_viewports.front()->clusters->stats() ) ) {
        statstext +=
            "\ncluster lists: " + std::to_string( cullstats->index_count )
            + ", overflow " + std::to_string( cullstats->overflow_count )
            + " (peak " + std::to_string( cullstats->overflow_peak ) + ")";
    }
    auto culltext { m_instancecull.info() };
    if( Global.gfx_gpucull_shadows ) {
        for( std::size_t cascade = 0; cascade < m_shadowcull.size(); ++cascade ) {
            if( false == m_shadowcull[ cascade ].ran() ) { continue; }
            m_shadowcull[ cascade ].collect();
            culltext += m_shadowcull[ cascade ].info( "gpu cull shadows " + std::to_string( cascade + 1 ), false );
        }
    }
    culltext += Light_Shadow_Cull_Info( false );
    if( Global.gfx_gpucull == 1 ) {
        auto const &parts { m_cullparts };
        culltext +=
            "gpu cull, cpu by part: depth of the scene so far " + to_string( parts.scenedepth, 3 ) + ", depth pyramid " + to_string( parts.pyramid, 3 )
            + ", culling " + to_string( parts.culling, 3 ) + ", walks and order " + to_string( parts.record, 3 ) + ", depth draws " + to_string( parts.depth, 3 )
            + ", colour draws " + to_string( parts.colour, 3 ) + " ms\n";
        // all passes of the last frame
        auto const &walks { m_lastwalkcounts };
        culltext +=
            "walks of the submodel trees in the frame: " + std::to_string( walks.walked ) + " made (" + std::to_string( walks.changing ) + " not kept), "
            + std::to_string( walks.reused ) + " kept from earlier frames (" + std::to_string( m_walkrecords.size() ) + " groups), cpu " + to_string( walks.walktime, 3 )
            + " ms; the draws put in order and sent, cpu " + to_string( walks.ordertime, 3 ) + " ms\n";
    }
    m_debugsections.push_back( { "Scene statistics", std::move( statstext ), false } );
    m_debugsections.push_back( { "GPU culling", std::move( culltext ), false } );
    m_debugsections.push_back( { "Memory", m_memorytext, true } );

    // the parts of the sections start with a line break here and there, and end with one or not
    for( auto &section : m_debugsections ) {
        auto &text { section.text };
        text.erase( 0, text.find_first_not_of( '\n' ) == std::string::npos ? text.size() : text.find_first_not_of( '\n' ) );
        while( ( false == text.empty() ) && ( text.back() == '\n' ) ) { text.pop_back(); }
    }
    // the same joined, the first three sections times, the others statistics
    m_debugtimestext.clear();
    m_debugstatstext.clear();
    for( std::size_t index = 0; index < m_debugsections.size(); ++index ) {
        auto &target { index < 3 ? m_debugtimestext : m_debugstatstext };
        target += ( target.empty() ? "" : "\n" ) + m_debugsections[ index ].title + ":\n" + m_debugsections[ index ].text;
    }
}

// what the process and its parts keep in memory, for the debug panel (m_memorytext). gathered every 2 s: it goes through every
// texture, geometry chunk, model and scenery shape
void opengl46_renderer::Update_Memory_Info()
{
	auto const now { std::chrono::steady_clock::now() };
	if( ( false == m_memorytext.empty() ) && ( now - m_memorytime < std::chrono::seconds( 2 ) ) ) { return; }
	m_memorytime = now;

	auto const mb = []( std::uint64_t const Bytes ) {
		return to_string( static_cast<double>( Bytes ) / ( 1024.0 * 1024.0 ), 1 ) + " MB"; };
	auto const counted = []( memory_count const &Count, char const *Name ) {
		return std::to_string( Count.count ) + " " + Name; };
	std::string text;

	auto const process { query_process_memory() };
	if( process.valid ) {
		text += "process: " + mb( process.resident ) + " resident (peak " + mb( process.peak ) + ")"
			+ ( process.committed > 0 ? ", " + mb( process.committed ) + " private" : std::string{} ) + "\n";
	}
	// the memory of the card, all processes: GL_NVX_gpu_memory_info (in kB)
	if( m_nvxmemory < 0 ) {
		m_nvxmemory = 0;
		GLint extensions { 0 };
		::glGetIntegerv( GL_NUM_EXTENSIONS, &extensions );
		for( GLint index = 0; index < extensions; ++index ) {
			auto const *name { reinterpret_cast<char const *>( ::glGetStringi( GL_EXTENSIONS, static_cast<GLuint>( index ) ) ) };
			if( ( name != nullptr ) && ( std::strcmp( name, "GL_NVX_gpu_memory_info" ) == 0 ) ) {
				m_nvxmemory = 1;
				break;
			}
		}
	}
	if( m_nvxmemory > 0 ) {
		GLint dedicated { 0 }, available { 0 }, evicted { 0 };
		::glGetIntegerv( 0x9047, &dedicated ); // GL_GPU_MEMORY_INFO_DEDICATED_VIDMEM_NVX
		::glGetIntegerv( 0x9049, &available ); // GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX
		::glGetIntegerv( 0x904B, &evicted ); // GL_GPU_MEMORY_INFO_EVICTED_MEMORY_NVX
		text += "card memory, all programs: " + mb( static_cast<std::uint64_t>( std::max( dedicated - available, 0 ) ) * 1024 ) + " used of "
			+ mb( static_cast<std::uint64_t>( dedicated ) * 1024 ) + ( evicted > 0 ? ", " + mb( static_cast<std::uint64_t>( evicted ) * 1024 ) + " evicted" : std::string{} ) + "\n";
	}

	// the rest needs new and delete counted, which slows the loading down: only with debug.memorystats
	if( false == heap_counting() ) {
		text += "the memory of the parts of the program: off (debug.memorystats yes in eu07.ini, it slows the loading down)";
		m_memorytext = std::move( text );
		return;
	}
	// what the program took with new: the rest of the private memory is of the drivers, libraries and the c runtime
	auto const heap { query_heap_memory() };
	text += "taken with new: " + mb( heap.bytes ) + " in " + std::to_string( heap.blocks ) + " blocks"
		+ ( process.valid && ( process.committed > heap.bytes ) ? ", the rest of the private memory " + mb( process.committed - heap.bytes ) + " (drivers, libraries)" : std::string{} ) + "\n";
	// the same by the size of the blocks, in wider ranges
	{
		struct range { std::size_t last; char const *name; }; // classes up to last, see heap_memory
		static range const ranges[] = {
			{ 6, "below 64 B" }, { 8, "64-255 B" }, { 10, "256 B-1 KB" }, { 12, "1-4 KB" }, { 16, "4-64 KB" }, { 20, "64 KB-1 MB" }, { 24, "1-16 MB" }, { heap_memory::CLASSES - 1, "16 MB and more" } };
		std::size_t first { 0 };
		text += " by block size:";
		for( auto const &item : ranges ) {
			std::uint64_t bytes { 0 }, blocks { 0 };
			for( auto index { first }; index <= item.last; ++index ) {
				bytes += heap.classbytes[ index ];
				blocks += heap.classblocks[ index ];
			}
			first = item.last + 1;
			if( blocks == 0 ) { continue; }
			text += std::string( " " ) + item.name + ": " + mb( bytes ) + " in " + std::to_string( blocks ) + ";";
		}
		text += "\n";
	}
	// where the large blocks come from
	{
		auto const large { query_large_blocks() };
		text += " blocks of " + std::to_string( large_blocks::LARGE_BLOCK / 1024 ) + " KB and more by the place which took them:\n";
		std::size_t const shown { 15 };
		std::uint64_t restbytes { 0 }, restblocks { 0 };
		for( std::size_t index = 0; index < large.sites.size(); ++index ) {
			auto const &site { large.sites[ index ] };
			if( index >= shown ) {
				restbytes += site.bytes;
				restblocks += site.blocks;
				continue;
			}
			text += "  " + mb( site.bytes ) + " in " + std::to_string( site.blocks ) + ": " + site.place + "\n";
		}
		if( restblocks > 0 ) {
			text += "  " + mb( restbytes ) + " in " + std::to_string( restblocks ) + ": " + std::to_string( large.sites.size() - shown ) + " other places\n";
		}
		if( large.untracked > 0 ) {
			text += "  " + std::to_string( large.untracked ) + " blocks not followed, the table of them was full\n";
		}
	}
	std::uint64_t accounted { 0 };
	// textures: the copies of the data kept in the process after the upload
	auto const textures { m_textures.memory() };
	accounted += textures.cpu.bytes;
	accounted += textures.readahead.bytes;
	text += "textures: " + std::to_string( textures.cpu.count ) + " keep their data in ram, " + mb( textures.cpu.bytes ) + "; "
		+ std::to_string( textures.gpu.count ) + " on the card, ~" + mb( textures.gpu.bytes ) + "\n";
	if( textures.readahead.count + textures.reading + textures.waiting > 0 ) {
		text += " streamed: " + std::to_string( textures.readahead.count ) + " read ahead, not drawn yet, " + mb( textures.readahead.bytes ) + "; "
			+ std::to_string( textures.reading ) + " being read; " + std::to_string( textures.waiting ) + " not read until they're needed\n";
	}
	// geometry: the copies of the chunks sent to the card
	auto const geometry { m_geometry.memory() };
	accounted += geometry.vertices.bytes + geometry.indices.bytes + geometry.userdata.bytes;
	text += "geometry banks: " + std::to_string( geometry.banks ) + ", " + counted( geometry.vertices, "chunks" ) + ": vertices " + mb( geometry.vertices.bytes )
		+ ", indices " + mb( geometry.indices.bytes ) + ", user data " + mb( geometry.userdata.bytes ) + "\n";
	text += gfx::opengl46_vaogeometrybank::shared_info();
	// models: the geometry their submodels keep of their own
	auto const models { TModelsManager::memory() };
	accounted += models.submodels.bytes;
	text += "models: " + std::to_string( models.models ) + ", " + counted( models.submodels, "submodels" ) + ", their own geometry " + mb( models.submodels.bytes ) + "\n";
	// scenery: the world space vertices the shapes and lines keep, the transforms of the sleepers, the model instances
	memory_count shapes, lines, sleepers;
	std::uint64_t instances { 0 };
	if( simulation::Region != nullptr ) {
		auto const shape = [ &shapes ]( scene::shape_node const &Shape ) {
			++shapes.count;
			shapes.bytes += memory_of( Shape.data().vertices ) + memory_of( Shape.data().userdata ); };
		for( auto const *section : simulation::Region->m_sections ) {
			if( section == nullptr ) { continue; }
			for( auto const &item : section->m_shapes ) { shape( item ); }
			for( auto const &cell : section->m_cells ) {
				for( auto const &item : cell.m_shapesopaque ) { shape( item ); }
				for( auto const &item : cell.m_shapestranslucent ) { shape( item ); }
				for( auto const &item : cell.m_lines ) {
					++lines.count;
					lines.bytes += memory_of( item.data().vertices );
				}
				for( auto const *track : cell.m_paths ) {
					if( track->m_sleeper_local_transforms.empty() ) { continue; }
					++sleepers.count;
					sleepers.bytes += memory_of( track->m_sleeper_local_transforms );
				}
				instances += cell.m_instancesopaque.size() + cell.m_instancetranslucent.size();
			}
		}
	}
	accounted += shapes.bytes + lines.bytes + sleepers.bytes;
	text += "scenery: " + counted( shapes, "shapes" ) + " " + mb( shapes.bytes ) + ", " + counted( lines, "lines" ) + " " + mb( lines.bytes ) + ", sleepers of "
		+ counted( sleepers, "tracks" ) + " " + mb( sleepers.bytes ) + "\n";
	// the objects of the model instances themselves, without the strings and the containers they hold
	accounted += instances * sizeof( TAnimModel );
	text += "model instances: " + std::to_string( instances ) + ", their objects " + mb( instances * sizeof( TAnimModel ) ) + " (" + std::to_string( sizeof( TAnimModel ) ) + " bytes each)\n";
	// sounds: the data of the buffers, which the AL keeps in the process
	auto const sounds { audio::renderer.buffers_memory() };
	accounted += sounds.bytes;
	text += "sounds: " + counted( sounds, "buffers" ) + " " + mb( sounds.bytes ) + "\n";
	text += "vehicles: " + std::to_string( simulation::Vehicles.sequence().size() ) + "\n";
	// the objects of the tables of the simulation, without what they hold
	{
		auto const table = [ & ]( char const *Name, std::size_t const Count, std::size_t const Size ) {
			return std::string( Name ) + " " + std::to_string( Count ) + " (" + mb( static_cast<std::uint64_t>( Count ) * Size ) + ")"; };
		auto const tables {
			table( "tracks", simulation::Paths.sequence().size(), sizeof( TTrack ) ) + ", "
			+ table( "traction", simulation::Traction.sequence().size(), sizeof( TTraction ) ) + ", "
			+ table( "memory cells", simulation::Memory.sequence().size(), sizeof( TMemCell ) ) + ", "
			+ table( "instances in the table", simulation::Instances.sequence().size(), sizeof( TAnimModel ) ) + ", "
			+ table( "sounds", simulation::Sounds.sequence().size(), sizeof( sound_source ) ) };
		text += "tables: " + tables + "\n";
	}
	if( process.valid ) {
		auto const total { process.committed > 0 ? process.committed : process.resident };
		text += "counted above: " + mb( accounted ) + " of " + mb( total ) + ( process.committed > 0 ? " private" : " resident" ) + ", the rest "
			+ mb( total > accounted ? total - accounted : 0 ) + " (other data, the heaps, the driver)\n";
	}
	text += "gathered in " + to_string( std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - now ).count(), 2 ) + " ms, every 2 s";
	m_memorytext = std::move( text );
}

void opengl46_renderer::cpu_timeline::begin( char const *Name )
{
	if( active ) { end(); }
	active = true;
	mark = std::chrono::steady_clock::now();
	current = parts.size();
	next( Name );
}

void opengl46_renderer::cpu_timeline::next( char const *Name )
{
	if( false == active ) { return; }
	auto const now { std::chrono::steady_clock::now() };
	if( current < parts.size() ) {
		parts[ current ].frame += std::chrono::duration<float, std::milli>( now - mark ).count();
	}
	mark = now;
	current = 0;
	while( ( current < parts.size() ) && ( parts[ current ].name != Name ) && ( std::strcmp( parts[ current ].name, Name ) != 0 ) ) {
		++current;
	}
	if( current == parts.size() ) {
		parts.push_back( { Name } );
	}
}

void opengl46_renderer::cpu_timeline::end()
{
	if( false == active ) { return; }
	auto const now { std::chrono::steady_clock::now() };
	if( current < parts.size() ) {
		parts[ current ].frame += std::chrono::duration<float, std::milli>( now - mark ).count();
	}
	// the average of Timer::stopwatch: the last 20 frames or so. parts left out in the frame fade away
	for( auto &part : parts ) {
		part.time = 0.95f * part.time + 0.05f * part.frame;
		part.frame = 0.f;
	}
	active = false;
}

std::optional<float> opengl46_renderer::cpu_timeline::time( char const *Name ) const
{
	for( auto const &part : parts ) {
		if( ( part.name == Name ) || ( std::strcmp( part.name, Name ) == 0 ) ) { return part.time; }
	}
	return std::nullopt;
}

void opengl46_renderer::draw_debug_ui()
{
	if (!debug_ui_active)
		return;

    if (ImGui::Begin("Headlight config", &debug_ui_active))
	{
		ImGui::SetWindowSize(ImVec2(0, 0));

		headlight_config_s &conf = headlight_config;
		ImGui::SliderFloat("in_cutoff", &conf.in_cutoff, 0.9f, 1.1f);
		ImGui::SliderFloat("out_cutoff", &conf.out_cutoff, 0.9f, 1.1f);

		ImGui::SliderFloat("falloff_linear", &conf.falloff_linear, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
		ImGui::SliderFloat("falloff_quadratic", &conf.falloff_quadratic, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic);

		ImGui::SliderFloat("ambient", &conf.ambient, 0.0f, 3.0f);
		ImGui::SliderFloat("intensity", &conf.intensity, 0.0f, 10.0f);
	}
	ImGui::End();

    if (ImGui::Begin("Light clusters"))
    {
        ImGui::Checkbox("clustered lights (off: the first dynamiclights headlights)", &Global.gfx_lights_clustered);
        ImGui::Checkbox("heatmap", &Global.gfx_lights_clusterheatmap);
        ImGui::Checkbox("beams of vehicle headlights (gfx.lights.headlights)", &Global.gfx_lights_headlights);
        if (auto const &clusters{m_viewports.front()->clusters})
        {
            auto const &params{clusters->params()};
            ImGui::Text("grid %u x %u x %u, %s", gl::CLUSTER_GRID_X, gl::CLUSTER_GRID_Y, gl::CLUSTER_GRID_Z, params.valid ? "built" : "none (orthographic view)");
            ImGui::Text("near %.3f m, far %.1f m", params.znear, params.zfar);
            ImGui::Text("headlights %u, %u over capacity of %u", m_lights_lit, m_lights_dropped, static_cast<unsigned>(gl::MAX_LIGHTS - 1));
            if (auto const cullstats = clusters->stats())
            {
                ImGui::Text("index list %u of %u", cullstats->index_count, gl::CLUSTER_COUNT * gl::MAX_LIGHTS_PER_CLUSTER);
                ImGui::Text("overflow: %u lights dropped from full clusters (limit %u each), peak %u", cullstats->overflow_count, gl::MAX_LIGHTS_PER_CLUSTER, cullstats->overflow_peak);
            }
        }
        ImGui::Separator();
        m_locallights.draw_debug_ui();
        ImGui::Separator();
        m_localshadows.draw_debug_ui();
        ImGui::Separator();
        char const *contactlevels[]{"no", "low (the sun)", "medium (the sun and 2 lamps)", "high (the sun and 4 lamps)"};
        ImGui::Combo("contact shadows (gfx.contactshadows)", &Global.gfx_contactshadows, contactlevels, IM_ARRAYSIZE(contactlevels));
        ImGui::SliderFloat("length of their rays, m (gfx.contactshadows.length)", &Global.gfx_contactshadows_length, 0.05f, 5.f, "%.2f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("distance they reach, m (gfx.contactshadows.range)", &Global.gfx_contactshadows_range, 1.f, 1000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
    }
    ImGui::End();

    if (ImGui::Begin("Sun shadows"))
    {
        ImGui::Checkbox("cascades fitted to the depth of the view (gfx.shadow.sdsm)", &Global.gfx_shadow_sdsm);
        ImGui::SliderFloat("splits, 0 equal lengths .. 1 equal ratios (gfx.shadow.sdsm.split)", &Global.gfx_shadow_sdsm_split, 0.f, 1.f, "%.2f");
        auto const &fit{m_shadowfitstate};
        if (Global.gfx_shadow_sdsm)
        {
            if (fit.fitted)
                ImGui::Text("fitted to pixels %.2f - %.1f m, measured %u frames ago, view turned %.1f deg since", fit.depthmin, fit.depthmax, fit.age, fit.turn);
            else
                ImGui::Text("fixed cascades: %s", fit.reason != nullptr ? fit.reason : "");
        }
        if (m_shadowbuffersize > 0)
        {
            for (std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade)
                ImGui::Text("cascade %d: to %.1f m, map %.1f m across (whole slice %.1f m), texel %.1f cm", static_cast<int>(cascade + 1), fit.splits[cascade + 1],
                            fit.drawn[cascade], fit.slice[cascade], 100.f * fit.drawn[cascade] / m_shadowbuffersize);
        }
    }
    ImGui::End();

    if (ImGui::Begin("Weather"))
    {
        m_weather.draw_debug_ui();
    }
    ImGui::End();

    if (ImGui::Begin("Particles"))
    {
        ImGui::Checkbox("smoke simulated and drawn on the gpu (gfx.smoke.gpu)", &Global.gfx_smoke_gpu);
        if (Global.gfx_smoke_gpu)
            ImGui::Checkbox("smoke lit by the sun, the sky and the lamps (gfx.smoke.lighting)", &Global.gfx_smoke_lighting);
        ImGui::Checkbox("rain and snow as particles of the gpu (gfx.precipitation.gpu)", &Global.gfx_precipitation_gpu);
        if (Global.gfx_precipitation_gpu)
            ImGui::SliderInt("particles of the heaviest rain or snow (gfx.precipitation.particles)", &Global.gfx_precipitation_particles, 0, 262144);
        ImGui::SliderFloat("soft particles, share of their size, 0: off (gfx.particles.soft)", &Global.gfx_particles_soft, 0.f, 4.f, "%.2f");
        auto const info{m_gpuparticles.info()};
        if (!info.empty())
            ImGui::TextUnformatted(info.c_str() + 1);
    }
    ImGui::End();

    if (ImGui::Begin("Sky and exposure"))
    {
        m_sky.draw_debug_ui();
        ImGui::SliderFloat("sun disc, 1 = physical (gfx.sun.disc)", &Global.gfx_sun_disc, 0.f, 2.f);
        ImGui::SliderFloat("moon disc (gfx.moon.disc)", &Global.gfx_moon_disc, 0.f, 8.f);
        ImGui::SliderFloat("moonlight on the scenery (gfx.moonlight)", &Global.gfx_moonlight, 0.f, 10.f);
        {
            // where the sun and the moon are: azimuth from the north to the east (the sky has x to the west, z to the north)
            auto const azimuth = [](glm::vec3 const &Direction) { return glm::degrees(std::atan2(-Direction.x, Direction.z)) + (Direction.x > 0.f ? 360.f : 0.f); };
            auto const &sun{simulation::Environment.sun()};
            auto const &moon{simulation::Environment.moon()};
            ImGui::Text("sun: elevation %.1f, azimuth %.1f", sun.getAngle(), azimuth(sun.getDirection()));
            ImGui::Text("moon: elevation %.1f, azimuth %.1f, lit %.0f%%, age %.1f d", moon.getAngle(), azimuth(moon.getDirection()), 100.f * moon.getIllumination(), moon.getPhase());
        }
        ImGui::Separator();
        m_exposure.draw_debug_ui();
        ImGui::Separator();
        m_ao.draw_debug_ui(!m_viewports.empty() && m_viewports.front()->ssao_tex != nullptr);
        ImGui::Separator();
        m_ssr.draw_debug_ui(nullptr);
    }
    ImGui::End();

    if (ImGui::Begin("Instance culling"))
    {
        char const *modes[]{"cpu", "gpu, drawn from its result", "cpu, compared with the gpu"};
        ImGui::Combo("instance culling (gfx.gpucull)", &Global.gfx_gpucull, modes, IM_ARRAYSIZE(modes));
        if (Global.gfx_gpucull == 1)
        {
            ImGui::Checkbox("leave out instances hidden behind the depth drawn before them (gfx.gpucull.occlusion)", &Global.gfx_gpucull_occlusion);
            if (Global.gfx_gpucull_occlusion)
                ImGui::Checkbox("draw the instances seen in the last frame first, to hide the others (gfx.gpucull.occlusion.instances)", &Global.gfx_gpucull_occlusion_instances);
            ImGui::Checkbox("draw the depth of the instances before their colour (gfx.depthprepass)", &Global.gfx_depthprepass);
            if (m_drawblockstream)
                ImGui::Checkbox("draw the instances of the same material and model with one multi draw (gfx.gpucull.multidraw)", &Global.gfx_gpucull_multidraw);
            else
                ImGui::TextUnformatted("(multi draws not available, see the log)");
        }
        if (Global.gfx_gpucull != 0)
        {
            ImGui::Checkbox("the same for the cascades of the sun shadows (gfx.gpucull.shadows)", &Global.gfx_gpucull_shadows);
            ImGui::Checkbox("the same for the shadow maps of headlights (gfx.gpucull.shadows.headlights)", &Global.gfx_gpucull_shadows_headlights);
            ImGui::Checkbox("the same for the scenery of the shadow maps of lamps, when it's drawn next (gfx.gpucull.shadows.lamps)", &Global.gfx_gpucull_shadows_lamps);
            ImGui::Checkbox("the same for the sleepers of the tracks of the view (gfx.gpucull.sleepers)", &Global.gfx_gpucull_sleepers);
            ImGui::Checkbox("the same for the models drawn on their own, with lights, animations or skins of their own (gfx.gpucull.models)", &Global.gfx_gpucull_models);
            ImGui::Checkbox("leave out the models of a cell lying wholly out of view, beyond their range or too small before the culling (gfx.gpucull.bucketrange)", &Global.gfx_gpucull_bucketrange);
            ImGui::Checkbox("keep the walks of the submodel trees of the instances from frame to frame (gfx.gpucull.walkcache)", &Global.gfx_gpucull_walkcache);
        }
        ImGui::Checkbox("draw the depth of the whole scene before its colour (gfx.depthprepass.scene)", &Global.gfx_depthprepass_scene);
        if (Global.gfx_contactshadows > 0)
            ImGui::TextUnformatted("(drawn anyway for the contact shadows, gfx.contactshadows)");
        ImGui::Checkbox("walk the scene once for its depth and colour, drawing both from a record (gfx.depthprepass.replay)", &Global.gfx_depthprepass_replay);
        ImGui::Checkbox("grass of the heightmap terrain (gfx.grass)", &Global.gfx_grass);
        if (Global.gfx_grass)
        {
            ImGui::SliderFloat("the grass drawn whole up to, m (gfx.grass.distance)", &Global.gfx_grass_distance, 10.f, 300.f, "%.0f");
            ImGui::SliderFloat("thinning out up to, m (gfx.grass.fadedistance)", &Global.gfx_grass_fadedistance, 10.f, 350.f, "%.0f");
            ImGui::SliderFloat("density of the grass (gfx.grass.density)", &Global.gfx_grass_density, 0.1f, 2.f, "%.2f");
        }
        auto info{m_instancecull.info()};
        if (Global.gfx_gpucull_shadows)
        {
            for (std::size_t cascade = 0; cascade < m_shadowcull.size(); ++cascade)
            {
                if (m_shadowcull[cascade].ran())
                    info += m_shadowcull[cascade].info("gpu cull shadows " + std::to_string(cascade + 1), false);
            }
        }
        info += Light_Shadow_Cull_Info(false);
        if (!info.empty())
            ImGui::TextUnformatted(info.c_str() + 1);
    }
    ImGui::End();

    ImGui::SetNextWindowSize(ImVec2S(400, 400));
    if (ImGui::Begin("Pickbuffer") && m_pick_tex) {
		ImGui::Image((ImTextureID)(intptr_t)(m_pick_tex->id), ImGui::GetContentRegionAvail(), ImVec2(0, 1.0), ImVec2(1.0, 0));
    }
    ImGui::End();

}

// runs jobs needed to generate graphics for specified render pass
void opengl46_renderer::Render_pass(viewport_config &vp, rendermode const Mode)
{
	setup_pass(vp, m_renderpass, Mode);
    switch (m_renderpass.draw_mode)
	{

	case rendermode::color:
	{
		glDebug("rendermode::color");

		glDebug("context switch");
		static bool was_made = false;

		if (m_current_viewport != &vp)
		{
			glfwMakeContextCurrent(vp.window);
			m_current_viewport = &vp;
			// the streamed model block and the instance buffer of the frame move with every update, bindings are per context
			model_ubo->bind_uniform();
			Bind_Instance_Stream();
		}

		if ((!simulation::is_ready) || (Global.gfx_skiprendering))
		{
			gl::framebuffer::unbind();
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            if (vp.main) {
                // clear state for ui
                gl::program::unbind();
				Application.render_ui();
            }
			break;
		}

		// the temporal anti-aliasing: the projection of the frame moved by a part of a pixel, the velocities pointed to the last frame
		auto const taa{Taa_View(vp)};
		m_motionhistory = taa;
		m_cabfuture = glm::mat4{1.f};
		if (taa)
		{
			auto const *cab{((false == FreeFlyModeFlag) && Global.render_cab && (simulation::Train != nullptr)) ? simulation::Train->Dynamic() : nullptr};
			m_renderpass.pass_camera.projection() = m_taa.begin_frame(
			    m_renderpass.pass_camera.projection(), glm::dmat3{m_renderpass.pass_camera.modelview()}, m_renderpass.pass_camera.position(), cab, glm::ivec2{vp.width, vp.height});
			m_cabfuture = m_taa.cab_future();
		}
		// the screen space reflections: where the points the rays hit were in the image of the history
		m_ssrframe = Ssr_View(vp);
		if (m_ssrframe)
		{
			auto const *cab{((false == FreeFlyModeFlag) && Global.render_cab && (simulation::Train != nullptr)) ? simulation::Train->Dynamic() : nullptr};
			m_ssr.begin_frame(
			    m_renderpass.pass_camera.projection(), glm::dmat3{m_renderpass.pass_camera.modelview()}, m_renderpass.pass_camera.position(), cab, glm::ivec2{vp.width, vp.height}, vp.ssr_levels);
		}

        m_colorpass = m_renderpass; // cache pass data
		// parts of the gpu time of the frame; the marks go through the main view only, in the context they were begun in
		auto const gpupart = [this, &vp](char const *Name) {
			if (vp.main)
			{
				m_gputimeline.next(Name);
				m_cputimeline.next(Name);
			}
		};

		scene_ubs.time = Timer::GetTime();
		{
			float percipitation_intensity = glm::saturate(Global.Overcast - 1.);
			scene_ubs.rain_params.x = percipitation_intensity; // % amount of droplets
			scene_ubs.rain_params.y = glm::mix(15., 1., percipitation_intensity); // Regeneration time
			scene_ubs.rain_params.z = (Global.Weather == "snow:" ? 1.f : 0.f); // flakes on the glass in place of the drops
			scene_ubs.rain_params.w = Global.AirTemperature; // how fast the flakes melt
			{
				// the air the drops on the glass are blown with: the wind as the particles drift with it, less the velocity of the
				// vehicle (V is along the front of the vehicle)
				glm::dvec3 air{glm::dvec3{simulation::Environment.wind()} * 0.5};
				if (TDynamicObject const *owner = Global.pCamera.m_owner)
					air -= owner->VectorFront() * owner->MoverParameters->V;
				scene_ubs.rain_wind = glm::vec4{glm::vec3{air}, 0.f};
			}
			if (TDynamicObject const *owner = Global.pCamera.m_owner; owner && !!owner->MoverParameters->CabActive)
			{
				for (int i = 0; i < 4; ++i)
				{
					if (i < owner->dWiperPos.size())
					{
						int index = owner->MoverParameters->CabActive > 0 ? i : static_cast<int>(owner->dWiperPos.size() - 1) - i;
						scene_ubs.wiper_pos[i] = owner->dWiperPos[index];
						if (owner->dWiperPos[index] > 0. && owner->wiperDirection[index])
						{
							scene_ubs.wiper_pos[i] += 1.;
						}
						if (owner->dWiperPos[index] < .025)
						{
							scene_ubs.wiper_timer_out[i] = scene_ubs.time;
						}
						if (owner->dWiperPos[index] > .975)
						{
							scene_ubs.wiper_timer_return[i] = scene_ubs.time;
						}
					}
					else
					{
						scene_ubs.wiper_pos[i] = 0.;
						scene_ubs.wiper_timer_out[i] = -1000.;
						scene_ubs.wiper_timer_return[i] = -1000.;
					}
				}
			}
			else
			{
				scene_ubs.wiper_pos = glm::vec4{0.};
				scene_ubs.wiper_timer_out = glm::vec4{-1000.};
				scene_ubs.wiper_timer_return = glm::vec4{-1000.};
			}
		}
		// the reflections: the ground below the horizon of the environment map, and the screen space ones set up for the colour
		// pass of the main view only
		scene_ubs.env_params = glm::vec4{Global.gfx_ambient_groundalbedo, 0.f, 0.f, 0.f};
		gfx::screen_reflections::clear(scene_ubs);
		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
        scene_ubs.inv_view = glm::inverse( glm::mat4{ glm::mat3{ m_colorpass.pass_camera.modelview() } } );
        scene_ubo->update(scene_ubs);
		scene_ubo->bind_uniform();
		// the water on the windscreens of the cab: a step of its simulation once a frame, its storage for the glasses of every view
		if (vp.main)
			m_windscreenwater.update(Timer::GetDeltaTime());
		m_windscreenwater.bind();

		if (Global.bWireFrame)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        setup_shadow_empty_map();
		setup_env_map(nullptr);
		// the cube shadow maps of the lamps, or the stand-in for them
		m_localshadows.bind();

		if (Global.gfx_shadowmap_enabled && vp.main)
		{
			glDebug("render shadowmap start");
			Timer::subsystem.gfx_shadows.start();

            m_renderpass.draw_stats = {};
            Render_pass(vp, rendermode::shadows);
			m_renderpass = m_colorpass; // restore draw mode. TBD, TODO: render mode stack

			Timer::subsystem.gfx_shadows.stop();
			glDebug("render shadowmap end");
        }

		if (Global.gfx_envmap_enabled && vp.main)
		{
			gpupart("reflections");
			// potentially update environmental cube map
            m_renderpass.draw_stats = {};
			Timer::subsystem.gfx_reflections.start();
            if (Render_reflections(vp))
				m_renderpass = m_colorpass; // restore color pass settings
			// the prefiltered map is made with every update of the environment map, or once the switch is turned on
			if (Global.gfx_envmap_ggx && m_environmentready && (m_sky.prefiltered() == nullptr))
				m_sky.prefilter(*m_env_tex);
			Timer::subsystem.gfx_reflections.stop();
			setup_env_map(m_env_tex.get());
		}

		if (vp.main)
		{
			gpupart("lamp and headlight shadows");
			Render_local_shadows();
			Render_lamp_shadows();
			Render_headlight_shadows();
		}

		gpupart("clear, clusters, lights");
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

		setup_drawing(false);

		glm::ivec2 target_size(vp.width, vp.height);
        if (vp.main && !vp.custom_backbuffer) // TODO: update window sizes also for extra viewports
            target_size = Global.fb_size;

		// the motion blur only when something in the view moves: the vehicle of the camera, or a vehicle drawn in the last frame.
		// with nothing moving the velocity is zero everywhere, the blur takes a single sample of each pixel and gives back the
		// image it was given, so the velocity target, its resolve and the blur are left out and the picture stays the same.
		// a vehicle setting off is blurred from its second frame, when its velocity is still far below a pixel
		auto const motionblur{
		    Global.gfx_postfx_motionblur_enabled
		    && (m_vehiclesmoved
		        || ((Global.pCamera.m_owner != nullptr) && (Global.pCamera.m_owner->get_future_movement() != glm::dvec3{0.0})))};

		if (!Global.gfx_skippipeline)
		{
			vp.msaa_fb->bind();

			if (motionblur || taa)
				vp.msaa_fb->setup_drawing(2);
			else
				vp.msaa_fb->setup_drawing(1);

			glViewport(0, 0, vp.width, vp.height);

			vp.msaa_fb->clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		}
		else
		{
			if (!Global.gfx_shadergamma)
				glEnable(GL_FRAMEBUFFER_SRGB);
			glBindFramebuffer(GL_FRAMEBUFFER, 0);
			glViewport(0, 0, target_size.x, target_size.y);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

        glEnable(GL_DEPTH_TEST);

		Timer::subsystem.gfx_color.start();

        if (vr && (vp.proj_type == viewport_config::vr_left || vp.proj_type == viewport_config::vr_right)) {
            glDebug("vr hiddenarea");
            TModel3d *mask = vr->get_hiddenarea_mesh((vp.proj_type == viewport_config::vr_left) ? vr_interface::eye_left : vr_interface::eye_right);

            if (mask) {
                glDisable(GL_CULL_FACE);
                glDisable(GL_BLEND);
                glDepthMask(GL_TRUE);

                m_hiddenarea_shader->bind();
                draw(mask->Root->m_geometry.handle);

                glEnable(GL_CULL_FACE);
            }
        }

		setup_matrices();
		setup_drawing(true);
        m_renderpass.draw_stats = {};

		model_ubs.future = glm::mat4();
        glm::mat4 future;
		if (taa)
		{
			future = m_taa.world_future();
		}
		else if (Global.pCamera.m_owner != nullptr)
		{
			auto const *vehicle = Global.pCamera.m_owner;
			glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
			future = glm::translate(mv, -glm::vec3(vehicle->get_future_movement())) * glm::inverse(mv);
		}

        // the cluster grid and its lists of lights are kept up only while something reads them: the shading or the heatmap.
        // the lists exist for a perspective view; without them the shading goes through the first few lights, as does the pass
        // the environment map is drawn in
        auto const clusterlists{ Global.gfx_lights_clustered || Global.gfx_lights_clusterheatmap };
        if( clusterlists ) {
            // the tiles are laid over what the scene is drawn to: the buffers of the viewport, or the window itself without the post processing pipeline
            Update_Clusters( vp, Global.gfx_skippipeline ? target_size : glm::ivec2{ vp.width, vp.height } );
        }
        Update_Lights( simulation::Lights, true, Global.gfx_lights_clustered && vp.clusters->valid() );
        if( clusterlists ) {
            Cull_Lights( vp );
        }

        // the clouds of the view and the fog volume, which take the lights of the clusters and the shadow map of the pass
        gpupart("clouds, fog volume");
        {
            auto const fogtarget{ Global.gfx_skippipeline ? target_size : glm::ivec2{ vp.width, vp.height } };
            if( vp.main ) {
                if( Global.gfx_shadowmap_enabled )
                    setup_shadow_bind_map();
                m_weather.render_view(
                    OpenGLMatrices.data( GL_PROJECTION ), glm::mat4{ m_colorpass.pass_camera.modelview() }, m_colorpass.pass_camera.position(), fogtarget,
                    ( m_cablamplit && Global.gfx_lights_clustered && vp.clusters->valid() ) ? 1 : -1 );
            }
            m_weather.setup_pass( vp.main, fogtarget );
        }

		glDebug("render environment");
		gpupart("sky");

		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
		// back to the rotation of the camera after the faces of the environment map, which are drawn with their own
		scene_ubs.inv_view = glm::inverse(glm::mat4{glm::mat3{m_colorpass.pass_camera.modelview()}});
		scene_ubo->update(scene_ubs);
		Render(&simulation::Environment);

        if( Global.gfx_shadowmap_enabled )
            setup_shadow_bind_map();

		// the depth of the whole opaque scene before its colour, which the contact shadows need
		m_scenedepthdrawn = false;
		m_indirectcolourpending = false;
		m_scenerecorded = false;
		auto const contactshadows{Contact_Shadows_Wanted(vp)};
		// the screen space reflections trace the depth of the whole scene, not only of what the contact shadows reach
		auto const reflections{m_ssrframe && (false == Global.bWireFrame)};
		if ((Global.gfx_depthprepass_scene || contactshadows || reflections) && vp.main && (false == Global.bWireFrame))
		{
			glDebug("render scene depth");
			gpupart("scene depth");
			Render_Scene_Depth(future, contactshadows && (false == Global.gfx_depthprepass_scene) && (false == reflections));
			if (contactshadows)
			{
				gpupart("contact shadows depth");
				Setup_Contact_Shadows(vp);
			}
			if (reflections)
			{
				Setup_Screen_Reflections(vp, taa);
			}
		}
		if (vp.main)
			m_contactshadowslast = m_contactshadows;

		// opaque parts...
		gpupart("colour, opaque");
		setup_drawing(false);

		if (m_scenerecorded)
		{
			// the depth pass walked the scene for this pass, which draws what it recorded (the cab drawn early included)
			glDebug("render recorded opaque scene");
			Render_Scene_Colour();
			m_scenerecorded = false;
		}
		else
		{
			// without rain/snow we can render the cab early to limit the overdraw
			// precipitation happens when overcast is in 1-2 range
			if (!FreeFlyModeFlag && Global.Overcast <= 1.0f && Global.render_cab)
			{
				glDebug("render opaque cab");
	            model_ubs.future = m_cabfuture;
	            auto const *vehicle{ simulation::Train->Dynamic() };
	            // the lamp lighting the shadows of the sun, unless it is a light of its own
	            if( ( vehicle->InteriorLightLevel > 0.f ) && ( false == m_cablamplit ) ) {
	                setup_shadow_color( glm::min( colors::white, m_shadowcolor + glm::vec4( vehicle->InteriorLight * vehicle->InteriorLightLevel, 1.f ) ) );
	            }
	            Render_cab( vehicle, vehicle->InteriorLightLevel, false );
			}

	        setup_shadow_color( m_shadowcolor );

			glDebug("render opaque region");

			model_ubs.future = future;

			Render(simulation::Region);
		}
		m_scenedepthdrawn = false;
		m_indirectcolourpending = false;

		// the grass of the terrain, opaque, over the ground drawn
		if (vp.main && (false == Global.bWireFrame))
		{
			gpupart("grass");
			Render_grass(future);
		}

        Render_vr_models();

		// ...translucent parts
		glDebug("render translucent region");
		gpupart("colour, translucent and cab");
		setup_drawing(true);
		Render_Alpha(simulation::Region);
		Render_editor_overlay();

		// the depth drawn so far, which the particles of the gpu fade in front of
		m_particlesoft =
			( Global.gfx_particles_soft > 0.f )
			&& ( ( Global.Smoke && Global.gfx_smoke_gpu && ( false == simulation::Particles.sequence().empty() ) ) || ( Global.gfx_precipitation_gpu && ( Global.Overcast > 1.f ) ) )
			&& Setup_Particles_Depth( vp );

		// particles
		Render_particles();

		// precipitation; done at end, only before cab render
		Render_precipitation();

		// cab render
		if (false == FreeFlyModeFlag && Global.render_cab)
		{
			model_ubs.future = m_cabfuture;
            auto *vehicle { simulation::Train->Dynamic() };
            if( ( vehicle->InteriorLightLevel > 0.f ) && ( false == m_cablamplit ) ) {
                setup_shadow_color( glm::min( colors::white, m_shadowcolor + glm::vec4( vehicle->InteriorLight * vehicle->InteriorLightLevel, 1.f ) ) );
            }
			if (Global.Overcast > 1.0f)
			{
				// with active precipitation draw the opaque cab parts here to mask rain/snow placed 'inside' the cab
                glDebug( "render opaque cab" );
                setup_drawing(false);
				Render_cab(vehicle, vehicle->InteriorLightLevel, false);
                Render_interior(false);
				setup_drawing(true);
                Render_interior(true);
			}
            glDebug( "render translucent cab" );
            Render_cab(vehicle, vehicle->InteriorLightLevel, true);

            setup_shadow_color( m_shadowcolor );

            if( Global.Overcast > 1.0f ) {
                // with the cab in place we can (finally) safely draw translucent part of the occupied vehicle
                if( taa ) {
                    // the vehicle adds its own motion to the view of the world (the cab future has it already)
                    model_ubs.future = m_taa.world_future();
                }
                Render_Alpha( vehicle );
            }
        }

		if (m_contactshadows || m_ssractive)
		{
			// the depth they read is of this view only
			m_contactshadows = false;
			m_ssractive = false;
			scene_ubs.contact_shadow_params = glm::vec4{0.f};
			scene_ubs.contact_shadow_lamps = glm::vec4{0.f};
			gfx::screen_reflections::clear(scene_ubs);
			scene_ubo->update(scene_ubs);
		}

		Timer::subsystem.gfx_color.stop();

        // store draw stats
        m_colorpass.draw_stats = m_renderpass.draw_stats;

        setup_shadow_empty_map();
		setup_env_map(nullptr);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

if (!Global.gfx_skippipeline)
		{
			gpupart("resolve, ambient occlusion, motion blur");
			// the targets of the ambient occlusion are there only when it was on at the start
			auto const aomode{(vp.ssao_tex != nullptr) ? Global.gfx_postfx_ao : 0};
			if (motionblur || taa)
			{
				vp.main_fb->clear(GL_COLOR_BUFFER_BIT);
                vp.main_fb->setup_drawing(2);
				vp.msaa_fb->blit_to(vp.main_fb.get(), vp.width, vp.height, GL_COLOR_BUFFER_BIT, GL_COLOR_ATTACHMENT0);
				vp.msaa_fb->blit_to(vp.main_fb.get(), vp.width, vp.height, GL_COLOR_BUFFER_BIT, GL_COLOR_ATTACHMENT1);
				vp.msaa_fb->blit_to(vp.main_fb.get(), vp.width, vp.height, GL_DEPTH_BUFFER_BIT, GL_DEPTH_ATTACHMENT);

				// the occlusion goes into the image before the blur, which smears both alike
				if (aomode != 0)
					Render_ambient_occlusion(vp, *vp.ao_apply_fb, aomode);

				// the image the motion blur takes: the scene, or the scene blended into its history
				auto *image{vp.main_tex.get()};
				if (taa)
				{
					gpupart("temporal anti-aliasing");
					Render_taa(vp);
					image = vp.taa_tex[m_taa.target()].get();
				}
				// the image before the blur is the one the reflections of the next frame take
				if (m_ssrframe)
				{
					if (taa)
						Store_Ssr_History(vp, *vp.taa_fb[m_taa.target()], GL_COLOR_ATTACHMENT0);
					else
						Store_Ssr_History(vp, *vp.main_fb, GL_COLOR_ATTACHMENT0);
				}
				if (motionblur)
				{
					if (taa)
						gpupart("motion blur");
					model_ubs.param[0].x = m_framerate / (1.0 / Global.gfx_postfx_motionblur_shutter);
					model_ubo->update(model_ubs);
					m_pfx_motionblur->apply({image, vp.main_texv.get(), vp.main_texd.get()}, vp.main2_fb.get());
				}
				else
				{
					vp.taa_fb[m_taa.target()]->blit_to(vp.main2_fb.get(), vp.width, vp.height, GL_COLOR_BUFFER_BIT, GL_COLOR_ATTACHMENT0);
				}
			}
			else
			{
				vp.main2_fb->clear(GL_COLOR_BUFFER_BIT);
				vp.msaa_fb->blit_to(vp.main2_fb.get(), vp.width, vp.height, GL_COLOR_BUFFER_BIT, GL_COLOR_ATTACHMENT0);
				if (aomode != 0)
				{
					// the depth for the occlusion to read, the image stays in main2
					vp.msaa_fb->blit_to(vp.main_fb.get(), vp.width, vp.height, GL_DEPTH_BUFFER_BIT, GL_DEPTH_ATTACHMENT);
					Render_ambient_occlusion(vp, *vp.main2_fb, aomode);
				}
				if (m_ssrframe)
					Store_Ssr_History(vp, *vp.main2_fb, GL_COLOR_ATTACHMENT0);
			}
			m_ssrframe = false;

			// the depth of the finished view, which the cascades of the sun shadows of the next frames are fitted to
			// (gfx.shadow.sdsm). the main framebuffer holds it resolved for the motion blur or the ambient occlusion
			if (vp.main && Global.gfx_shadow_sdsm && (Shadow_Fit_Unavailable(vp) == nullptr))
			{
				auto const resolved{(vp.main_texd != nullptr) && (motionblur || taa || (aomode != 0))};
				opengl46_shadow_fit::view const view{
				    m_colorpass.pass_camera.position(),
				    m_colorpass.pass_camera.projection(),
				    glm::mat3{m_colorpass.pass_camera.modelview()},
				    Shadow_Light_Rotation(),
				    Shadow_Fixed_Splits().back()};
				m_shadowfit.measure(resolved ? vp.main_texd->id : 0, *vp.msaa_fb, vp.width, vp.height, view, m_framestamp);
			}

			gpupart("exposure, bloom");
			// the exposure follows the main view; the other viewports use it too
			if (Global.gfx_postfx_autoexposure && vp.main)
				m_exposure.measure(*vp.main2_tex, Timer::GetDeltaRenderTime());
			opengl_texture *bloomtexture{nullptr};
			std::size_t bloomlevels{0};
			if (Global.gfx_postfx_bloom && vp.bloom)
			{
				bloomtexture = m_exposure.bloom(*vp.bloom, *vp.main2_tex, *model_ubo, model_ubs);
				bloomlevels = vp.bloom->sizes.size();
			}
			model_ubs.param[0] = m_exposure.tonemap_params(bloomlevels);
			// the dither goes to the last pass before the window: the chromatic aberration, when it follows
			model_ubs.param[1] = glm::vec4{Global.gfx_postfx_chromaticaberration_enabled ? 0.f : 1.f, 0.f, 0.f, 0.f};
			model_ubo->update(model_ubs);
			std::vector<opengl_texture *> tonemapinput{vp.main2_tex.get()};
			if (bloomtexture != nullptr)
				tonemapinput.push_back(bloomtexture);

			gpupart("tonemapping, output");
			if (!Global.gfx_shadergamma)
				glEnable(GL_FRAMEBUFFER_SRGB);

            gl::framebuffer *target = nullptr;
            if (vp.custom_backbuffer)
                target = vp.backbuffer_fb.get();

            if( Global.gfx_postfx_chromaticaberration_enabled ) {
                glViewport(0, 0, vp.width, vp.height);
                vp.main_fb->setup_drawing(1);
                m_pfx_tonemapping->apply( tonemapinput, vp.main_fb.get() );

                glViewport(0, 0, target_size.x, target_size.y);
                m_pfx_chromaticaberration->apply( *vp.main_tex, target );
            }
            else {
                glViewport(0, 0, target_size.x, target_size.y);
                m_pfx_tonemapping->apply( tonemapinput, target );
            }

            if (Global.gfx_lights_clusterheatmap && vp.clusters->valid())
                Render_cluster_heatmap(vp, target, target_size);

            if (vr) {
                if (vp.proj_type == viewport_config::vr_left)
                    vr->submit(vr_interface::eye_left, vp.backbuffer_tex.get());
                if (vp.proj_type == viewport_config::vr_right)
                    vr->submit(vr_interface::eye_right, vp.backbuffer_tex.get());
            }

            if (vp.custom_backbuffer && vp.real_window) {
                if (vp.main)
                    target_size = Global.fb_size;
                glViewport(0, 0, target_size.x, target_size.y);

                vp.backbuffer_tex->bind(0);
                m_copy_shader->bind();
                m_empty_vao->bind();
                glBindFramebuffer(GL_FRAMEBUFFER, 0);

                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

                vp.backbuffer_tex->unbind(0);
            }

            opengl_texture::reset_unit_cache();
		}

		if (!Global.gfx_shadergamma)
			glDisable(GL_FRAMEBUFFER_SRGB);

		glDebug("uilayer render");
		gpupart("ui");
		Timer::subsystem.gfx_gui.start();

        if (vp.main) {
            // clear state for ui
            gl::program::unbind();
			draw_debug_ui();
			Application.render_ui();
		}

        Timer::subsystem.gfx_gui.stop();

		// viewports drawn after this one in the same context (vr) land in the last part; others aren't measured
		if (m_viewports.size() > 1)
			gpupart("other viewports");

		// restore binding
		scene_ubo->bind_uniform();

		glDebug("rendermode::color end");
		break;
	}

	case rendermode::shadows:
	{
		if (!simulation::is_ready)
			break;

		glDebug("rendermode::shadows");

        glEnable(GL_DEPTH_TEST);
        setup_drawing( false );

		glViewport(0, 0, m_shadowbuffersize, m_shadowbuffersize);
		auto csmstageboundaries { Shadow_Fixed_Splits() };
		// the cascades fitted to the depth of the view (gfx.shadow.sdsm) go between the distances of its pixels, each map
		// covering the area of the pixels its cascade takes
		std::array<std::optional<std::array<glm::vec3, 8>>, gl::MAX_CASCADES> fitareas;
		auto const fitted { Shadow_Fit( vp, csmstageboundaries, fitareas ) };
		scene_ubs.shadow_fit = glm::vec4{ fitted ? 1.f : 0.f, 0.f, 0.f, 0.f };
		m_shadowfitstate.splits = csmstageboundaries;

        for( auto idx = 0; idx < m_shadowpass.size(); ++idx ) {

            // the pass is drawn for the main view only
            static char const *const gpuparts[] = { "sun shadows 1", "sun shadows 2", "sun shadows 3" };
            static_assert( std::size( gpuparts ) == std::tuple_size_v<decltype( m_shadowpass )> );
            m_gputimeline.next( gpuparts[ idx ] );
            m_cputimeline.next( gpuparts[ idx ] );

            m_shadow_fb->attach( *m_shadow_tex, GL_DEPTH_ATTACHMENT, idx );
            m_shadow_fb->clear( GL_DEPTH_BUFFER_BIT );

            m_shadowfitarea = ( fitareas[ idx ] ? &( *fitareas[ idx ] ) : nullptr );
            setup_pass( vp, m_renderpass, rendermode::shadows, csmstageboundaries[ idx ], csmstageboundaries[ idx + 1 ] );
            m_shadowfitarea = nullptr;
            m_shadowpass[ idx ] = m_renderpass; // store pass config for reference in other stages
            m_shadowfitstate.slice[ idx ] = m_shadowslicesize;
            m_shadowfitstate.drawn[ idx ] = m_shadowdrawnsize;

            setup_matrices();
            scene_ubs.projection = OpenGLMatrices.data( GL_PROJECTION );
            auto const csmstagezfar {
                (csmstageboundaries[ idx + 1 ])
              + ( m_shadowpass.size() - idx ) }; // we can fairly safely add some extra padding in the early stages
            scene_ubs.cascade_end[ idx ] = csmstagezfar * csmstagezfar; // store squared to allow semi-optimized length2 comparisons in the shader
            scene_ubo->update( scene_ubs );

            if( m_shadowcolor != colors::white ) {
                m_shadowcascade = idx;
                Render( simulation::Region );
                m_shadowcascade = -1;
                if( idx > 0 ) { continue; } // render cab only in the closest csm stage
                if( !FreeFlyModeFlag && Global.render_cab ) {
                    Render_cab( simulation::Train->Dynamic(), 0.0f, false );
                    Render_cab( simulation::Train->Dynamic(), 0.0f, true );
                }
            }
        }
		m_shadow_fb->unbind();

		glDebug("rendermode::shadows end");

		break;
	}

	case rendermode::reflections:
	{
		if (!simulation::is_ready)
			break;

		glDebug("rendermode::reflections");

		// NOTE: buffer attachment and viewport setup in this mode is handled by the wrapper method
		glEnable(GL_DEPTH_TEST);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		m_env_fb->clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		m_env_fb->bind();

		setup_env_map(m_empty_cubemap.get());
		// the fog volume is the one of the main view
		m_weather.setup_pass(false, glm::ivec2{static_cast<int>(gl::ENVMAP_SIZE)});

		setup_matrices();
		// the light buffer holds what the last color pass left in it, in the view space of its camera, while the face is drawn
		// in its own: the sun goes in anew, for the fog and the clouds over the sky
		Update_Lights(simulation::Lights, false);

		// the sky and the clouds alone: the scenery is reflected by the screen space reflections (gfx.ssr), the environment map
		// gives what they don't find
		setup_drawing(true);

        setup_shadow_empty_map();
		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
		// world space directions of the shading of the face (the sky, the directional ambient light)
		scene_ubs.inv_view = glm::inverse(glm::mat4{glm::mat3{m_renderpass.pass_camera.modelview()}});
		scene_ubo->update(scene_ubs);
		Render(&simulation::Environment);
		// the passes after this one expect the state the opaque scenery left behind (depth writes on, no blending)
		setup_drawing(false);

		m_env_fb->unbind();

		glDebug("rendermode::reflections end");

		break;
	}

	case rendermode::pickcontrols:
	{
		if (!simulation::is_ready || !simulation::Train)
			break;

		glDebug("rendermode::pickcontrols");

		glEnable(GL_DEPTH_TEST);
		glViewport(0, 0, EU07_PICKBUFFERSIZE, EU07_PICKBUFFERSIZE);
		m_pick_fb->bind();
		m_pick_fb->clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        m_picksurfaceitems.clear();
		m_pickcontrolsitems.clear();
		setup_matrices();
		setup_drawing(false);

		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
		scene_ubo->update(scene_ubs);
		if (simulation::Train != nullptr) {
			Render_cab(simulation::Train->Dynamic(), 0.0f);
			Render(simulation::Train->Dynamic());
		}

		m_pick_fb->unbind();

		glDebug("rendermode::pickcontrols end");
		break;
	}

	case rendermode::pickscenery:
	{
		if (!simulation::is_ready)
			break;

		glEnable(GL_DEPTH_TEST);
		glViewport(0, 0, EU07_PICKBUFFERSIZE, EU07_PICKBUFFERSIZE);
		m_pick_fb->bind();
		m_pick_fb->clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		m_picksceneryitems.clear();
		setup_matrices();
		setup_drawing(false);

		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
		scene_ubo->update(scene_ubs);
 		Render(simulation::Region);

		break;
	}

	default:
	{
		break;
	}
	}
}



// creates dynamic environment cubemap
bool opengl46_renderer::Render_reflections(viewport_config &vp)
{
    if( Global.reflectiontune.update_interval == 0 ) { return false; }

    if( m_environmentupdateface >= 6 ) {
        // no update in progress, check whether it's time for another one
        auto const timestamp{ Timer::GetRenderTime() };
        if( ( timestamp - m_environmentupdatetime < Global.reflectiontune.update_interval )
         && ( glm::length2( m_renderpass.pass_camera.position() - m_environmentupdatelocation ) < sq(1000.0)) ) // length2 is better than length for comparing because it does not require sqrt function
        {
            // run update every 5+ mins of simulation time, or at least 1km from the last location
            return false;
        }
        m_environmentupdatetime = timestamp;
        m_environmentupdatelocation = m_renderpass.pass_camera.position();
        m_environmentupdateface = 0;
    }
	// an update draws the sky six times. its cost is spread by drawing one face of the map per frame, all of them as seen
	// from the place the update began at. the first update is done in one go, as there's nothing to show in the meantime
	auto const lastface{ m_environmentready ? m_environmentupdateface : 5 };
	glViewport(0, 0, gl::ENVMAP_SIZE, gl::ENVMAP_SIZE);
	for (; m_environmentupdateface <= lastface; ++m_environmentupdateface)
	{
		m_environmentcubetextureface = m_environmentupdateface;
		m_env_fb->attach(*m_env_tex, m_environmentcubetextureface, GL_COLOR_ATTACHMENT0);
		if (m_env_fb->is_complete())
			Render_pass(vp, rendermode::reflections);
	}
	if (m_environmentupdateface >= 6)
	{
		m_env_tex->generate_mipmaps();
		m_environmentready = true;
		if (Global.gfx_envmap_ggx)
			m_sky.prefilter(*m_env_tex);
	}
	m_env_fb->detach(GL_COLOR_ATTACHMENT0);

	return true;
}

// based on
// https://csc.lsu.edu/~kooima/articles/genperspective/index.html
glm::mat4 opengl46_renderer::perspective_projection(const viewport_proj_config &c,
                                                  float n, float f, glm::mat4 &frustum)
{
	glm::vec3 vr, vu, vn;
	vr = glm::normalize(c.pb - c.pa);
	vu = glm::normalize(c.pc - c.pa);
	vn = glm::normalize(glm::cross(vr, vu));

	glm::mat4 M;
	M[0] = glm::vec4(vr, 0.0f);
	M[1] = glm::vec4(vu, 0.0f);
	M[2] = glm::vec4(vn, 0.0f);
	M = glm::transpose(M);

	glm::vec3 va, vb, vc;
	va = c.pa - c.pe;
	vb = c.pb - c.pe;
	vc = c.pc - c.pe;

	float l, r, b, t, d;
	d = -glm::dot(va, vn);

	l = glm::dot(vr, va) * n / d;
	r = glm::dot(vr, vb) * n / d;
	b = glm::dot(vu, va) * n / d;
	t = glm::dot(vu, vc) * n / d;

	frustum = glm::frustum(l, r, b, t, n, f);
	glm::mat4 R = frustum;
    frustum = glm::translate(frustum * M, -c.pe);

	// clip_control: projection matrix with 1..0 Z range and infinite zfar
	R[2][2] = -1.0f;
	R[3][2] = -2.0f * n;
	R = glm::mat4( //
	           1.0f, 0.0f, 0.0f, 0.0f, //
	           0.0f, 1.0f, 0.0f, 0.0f, //
	           0.0f, 0.0f, -0.5f, 0.0f, //
	           0.0f, 0.0f, 0.5f, 1.0f //
	           ) * R;

	return glm::translate(R * M, -c.pe);
}

glm::mat4 opengl46_renderer::perspective_projection_raw(float fovy, float aspect, float znear, float zfar)
{
    const float f = 1.0f / std::tan(fovy / 2.0f);

    // clip_control: projection matrix with 1..0 Z range and infinite zfar
    return glm::mat4( //
        f / aspect, 0.0f, 0.0f, 0.0f, //
        0.0f, f, 0.0f, 0.0f, //
        0.0f, 0.0f, 0.0f, -1.0f, //
        0.0f, 0.0f, znear, 0.0f //
    );
}

glm::mat4 opengl46_renderer::ortho_projection(float l, float r, float b, float t, float znear, float zfar)
{
	glm::mat4 proj = glm::ortho(l, r, b, t, znear, zfar);
	// clip_control: projection matrix with 1..0 Z range
	return glm::mat4( //
	           1.0f, 0.0f, 0.0f, 0.0f, //
	           0.0f, 1.0f, 0.0f, 0.0f, //
	           0.0f, 0.0f, -0.5f, 0.0f, //
	           0.0f, 0.0f, 0.5f, 1.0f //
	           ) *
	       proj;
}

glm::mat4 opengl46_renderer::ortho_frustumtest_projection(float l, float r, float b, float t, float znear, float zfar)
{
	// for frustum calculation, use standard opengl matrix
	return glm::ortho(l, r, b, t, znear, zfar);
}

// the main viewport of the editor in the orthographic view replaces the perspective projection
void opengl46_renderer::editor_ortho_projection(viewport_config const &Viewport, float const Zfar, glm::mat4 &Projection, glm::mat4 &Frustum)
{
	if (false == (EditorModeFlag && Global.EditorOrtho && Viewport.main))
		return;
	auto const height{Global.EditorOrthoExtent};
	auto const width{height * Global.window_size.x / std::max(1.f, static_cast<float>(Global.window_size.y))};
	Projection = ortho_projection(-width, width, -height, height, -Zfar, Zfar);
	Frustum = ortho_frustumtest_projection(-width, width, -height, height, -Zfar, Zfar);
}

void opengl46_renderer::setup_pass(viewport_config &Viewport, renderpass_config &Config, rendermode const Mode,
                                 float const Znear, float const Zfar, bool const Ignoredebug)
{
	Config.draw_mode = Mode;

    if( false == simulation::is_ready ) { return; }
    // setup draw range
    switch( Mode ) {
        case rendermode::color:        { Config.draw_range = Global.BaseDrawRange * Global.fDistanceFactor * Viewport.draw_range; break; }
        case rendermode::shadows:      { Config.draw_range = Global.shadowtune.range; break; }
        case rendermode::reflections:  { Config.draw_range = Global.BaseDrawRange * Viewport.draw_range; break; }
        case rendermode::pickcontrols: { Config.draw_range = 50.f; break; }
        case rendermode::pickscenery:  { Config.draw_range = Global.BaseDrawRange * Viewport.draw_range * 0.5f; break; }
        default:                       { Config.draw_range = 0.f; break; }
    }

	// setup camera
	auto &camera = Config.pass_camera;

	glm::dmat4 viewmatrix(1.0);

	glm::mat4 frustumtest_proj;

	glm::ivec2 target_size(Viewport.width, Viewport.height);
	if (Viewport.main) // TODO: update window sizes also for extra viewports
        target_size = Global.fb_size;

	Config.viewport_camera.position() = Global.pCamera.Pos;

	switch (Mode)
	{
	case rendermode::color:
    {
        if (Viewport.proj_type == viewport_config::normal && Viewport.main) {
            // TODO: update window sizes also for extra viewports
            float const fovy = glm::radians(Global.FieldOfView / Global.ZoomFactor);

            // setup virtual screen
            glm::vec2 screen_h = glm::vec2(Global.window_size) / 2.0f;
            float const dist = screen_h.y / glm::tan(fovy / 2.0f);

            Viewport.projection.pa = glm::vec3(-screen_h.x, -screen_h.y, -dist);
            Viewport.projection.pb = glm::vec3( screen_h.x, -screen_h.y, -dist);
            Viewport.projection.pc = glm::vec3(-screen_h.x,  screen_h.y, -dist);
            Viewport.projection.pe = glm::vec3(0.0f, 0.0f, 0.0f);
        }

        if (vr && (Viewport.proj_type == viewport_config::vr_left || Viewport.proj_type == viewport_config::vr_right)) {
            Viewport.projection = vr->get_proj_config(Viewport.proj_type == viewport_config::vr_left ? vr_interface::eye_left : vr_interface::eye_right);
            Global.pCamera.Angle.x = 0.0;
            Global.pCamera.Angle.z = 0.0;
        }

        if (Global.headtrack_conf.magic_window)
            Viewport.projection.pe = Global.viewport_move;

        TCamera *cam = ((false == DebugCameraFlag) || (true == Ignoredebug)) ? &Global.pCamera : &Global.pDebugCamera;
        camera.position() = cam->Pos;
        cam->SetMatrix(viewmatrix);

        if (!Global.headtrack_conf.magic_window || vr) {
            camera.position() += Global.viewport_move * glm::mat3(viewmatrix);
            viewmatrix = glm::dmat4(glm::inverse(Global.viewport_rotate)) * viewmatrix;
        }

        // optimization: don't bother drawing things completely blended with the background
        // but ensure at least 2 km range to account for signals and lights
        Config.draw_range = std::min( Config.draw_range, m_fogrange * 2 );
        Config.draw_range = std::max( 2000.f, Config.draw_range );

		// projection
		auto zfar  = ( Zfar  > 1.f ? Zfar : Config.draw_range * Zfar );
		auto const znear = ( Znear > 1.f ? Znear : Znear > 0.f ? Znear * zfar : 0.1f * Global.ZoomFactor);
		if (EditorModeFlag && Viewport.main)
			zfar = editor_track_view_range(zfar, true);
		Config.znear = znear;
		Config.zfar = zfar;

		camera.projection() = perspective_projection(Viewport.projection, znear, zfar, frustumtest_proj);
		editor_ortho_projection(Viewport, zfar, camera.projection(), frustumtest_proj);
		break;
	}
	case rendermode::shadows:
	{
        // calculate lightview boundaries based on relevant area of the world camera frustum:
		// ...setup chunk of frustum we're interested in...
//        auto const zfar = std::min(1.f, Global.shadowtune.range / (Global.BaseDrawRange * Global.fDistanceFactor) * std::max(1.f, Global.ZoomFactor * 0.5f));
//        auto const zfar = ( Zfar > 1.f ? Zfar * std::max( 1.f, Global.ZoomFactor * 0.5f ) : Zfar );
        auto const zfar = ( Zfar > 1.f ? Zfar : Zfar * Global.shadowtune.range * std::max( Zfar, Global.ZoomFactor * 0.5f ) );

        size_t bounding_viewports_count = 0;
        std::vector<glm::vec4> frustumchunkshapepoints;
        glm::dvec3 pass_camera;

        for (std::unique_ptr<viewport_config> &conf : m_viewports) {
            if (!conf->shadow)
                continue;

            renderpass_config worldview;
            setup_pass(*conf, worldview, rendermode::color, Znear, zfar, true);

            auto &points = worldview.pass_camera.frustum_points();
            frustumchunkshapepoints.insert(frustumchunkshapepoints.end(), points.begin(), points.end());

            pass_camera += worldview.pass_camera.position();
            bounding_viewports_count++;
        }
        pass_camera /= bounding_viewports_count;

		// ...a cascade fitted to the depth of the view (gfx.shadow.sdsm) covers the area of the pixels it takes, within the
		// slice of the view...
		auto quantizationstep{gfx::shadow_fit::fit_slice(frustumchunkshapepoints, m_shadowfitarea, Shadow_Light_Rotation(), m_shadowslicesize)};

		// ...modelview matrix: determine the centre of frustum chunk in world space...
		glm::vec3 frustumchunkmin, frustumchunkmax;
		bounding_box(frustumchunkmin, frustumchunkmax, std::begin(frustumchunkshapepoints), std::end(frustumchunkshapepoints));
		auto const frustumchunkcentre = (frustumchunkmin + frustumchunkmax) * 0.5f;
		// ...cap the vertical angle to keep shadows from getting too long...
		auto const lightvector = Shadow_Light_Vector();
		// ...place the light source at the calculated centre and setup world space light view matrix...
        camera.position() = pass_camera + glm::dvec3{frustumchunkcentre};
		viewmatrix *= glm::lookAt(camera.position(), camera.position() + glm::dvec3{lightvector}, glm::dvec3{0.f, 1.f, 0.f});
		// ...projection matrix: calculate boundaries of the frustum chunk in light space...
		auto const lightviewmatrix = glm::translate(glm::mat4{glm::mat3{viewmatrix}}, -frustumchunkcentre);
		for (auto &point : frustumchunkshapepoints)
		{
			point = lightviewmatrix * point;
		}
		bounding_box(frustumchunkmin, frustumchunkmax, std::begin(frustumchunkshapepoints), std::end(frustumchunkshapepoints));
		// quantize the frustum points and add some padding, to reduce shadow shimmer on scale changes
        auto const frustumchunkdepth { zfar - ( Znear > 1.f ? Znear : 0.f ) };
		if (quantizationstep <= 0.f)
			quantizationstep = std::min(frustumchunkdepth, 50.f);
		frustumchunkmin = quantizationstep * glm::floor(frustumchunkmin * (1.f / quantizationstep));
		frustumchunkmax = quantizationstep * glm::ceil(frustumchunkmax * (1.f / quantizationstep));
		m_shadowdrawnsize = std::max(frustumchunkmax.x - frustumchunkmin.x, frustumchunkmax.y - frustumchunkmin.y);
		// ...use the dimensions to set up light projection boundaries...
		// NOTE: since we only have one cascade map stage, we extend the chunk forward/back to catch areas normally covered by other stages
		camera.projection() = ortho_projection(frustumchunkmin.x, frustumchunkmax.x, frustumchunkmin.y, frustumchunkmax.y, frustumchunkmin.z - 500.f, frustumchunkmax.z + 500.f);
		frustumtest_proj = ortho_frustumtest_projection(frustumchunkmin.x, frustumchunkmax.x, frustumchunkmin.y, frustumchunkmax.y, frustumchunkmin.z - 500.f, frustumchunkmax.z + 500.f);
		// ... and adjust the projection to sample complete shadow map texels:
		// get coordinates for a sample texel...
		auto shadowmaptexel = glm::vec2{camera.projection() * glm::mat4{viewmatrix} * glm::vec4{0.f, 0.f, 0.f, 1.f}};
		// ...convert result from clip space to texture coordinates, and calculate adjustment...
		shadowmaptexel *= m_shadowbuffersize * 0.5f;
		auto shadowmapadjustment = glm::round(shadowmaptexel) - shadowmaptexel;
		// ...transform coordinate change back to homogenous light space...
		shadowmapadjustment /= m_shadowbuffersize * 0.5f;
		// ... and bake the adjustment into the projection matrix
		camera.projection() = glm::translate(glm::mat4{1.f}, glm::vec3{shadowmapadjustment, 0.f}) * camera.projection();

		break;
	}
	case rendermode::pickcontrols:
	case rendermode::pickscenery:
    {
		// modelview
		camera.position() = Global.pCamera.Pos;
		Global.pCamera.SetMatrix(viewmatrix);

        viewport_proj_config proj = Viewport.projection;

        // projection
        float znear = 0.1f * Global.ZoomFactor;
        float zfar = Config.draw_range * Global.fDistanceFactor;
		if (EditorModeFlag && Viewport.main && Mode == rendermode::pickscenery)
			zfar = editor_track_view_range(zfar, true);

        if (vr) {
            glm::mat4 transform = vr->get_pick_transform();
            camera.position() += glm::vec3(transform[3]) * glm::mat3(viewmatrix);
            viewmatrix = glm::dmat4(glm::inverse(glm::mat3(transform))) * viewmatrix;

            znear = 0.01f;
            zfar = 10.0f;
            float const fovy = glm::radians(3.0f);

            glm::vec2 screen_h = glm::vec2(1000.0f, 1000.0f) / 2.0f;
            float const dist = screen_h.y / glm::tan(fovy / 2.0f);

            proj.pa = glm::vec3(-screen_h.x, -screen_h.y, -dist);
            proj.pb = glm::vec3( screen_h.x, -screen_h.y, -dist);
            proj.pc = glm::vec3(-screen_h.x,  screen_h.y, -dist);
            proj.pe = glm::vec3(0.0f, 0.0f, 0.0f);
        }

        camera.projection() = perspective_projection(proj, znear, zfar, frustumtest_proj);
		editor_ortho_projection(Viewport, zfar, camera.projection(), frustumtest_proj);
		break;
	}
	case rendermode::reflections:
	{
		// modelview
		// the faces of the map are drawn over a few frames. each is drawn from the place the camera was at
		// when the update began, so they fit together along the edges when the camera moves in the meantime
		camera.position() = m_environmentupdatelocation;
		glm::dvec3 const cubefacetargetvectors[6] = {{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}};
		glm::dvec3 const cubefaceupvectors[6] = {{0.0, -1.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}, {0.0, -1.0, 0.0}, {0.0, -1.0, 0.0}};
		auto const cubefaceindex = m_environmentcubetextureface;
		viewmatrix *= glm::lookAt(camera.position(), camera.position() + cubefacetargetvectors[cubefaceindex], cubefaceupvectors[cubefaceindex]);
		// projection
		float znear = 0.1f * Global.ZoomFactor;
		float zfar = Config.draw_range * Global.fDistanceFactor;

		viewport_proj_config proj({{ -0.5f, -0.5f, -0.5f },
		                           {  0.5f, -0.5f, -0.5f },
		                           { -0.5f,  0.5f, -0.5f },
		                           { 0.0f, 0.0f, 0.0f }});
		camera.projection() = perspective_projection(proj, znear, zfar, frustumtest_proj);
		break;
	}
	default:
	{
		break;
	}
	}
	camera.modelview() = viewmatrix;
	camera.update_frustum(frustumtest_proj);
}

void opengl46_renderer::setup_matrices()
{
    OpenGLMatrices.mode(GL_PROJECTION);
	OpenGLMatrices.load_matrix(m_renderpass.pass_camera.projection());
	// trim modelview matrix just to rotation, since rendering is done in camera-centric world space
    OpenGLMatrices.mode(GL_MODELVIEW);
    OpenGLMatrices.load_matrix(glm::mat4{glm::mat3{m_renderpass.pass_camera.modelview()}});
}

void opengl46_renderer::setup_drawing(bool const Alpha)
{
	if (Alpha)
	{
		glEnable(GL_BLEND);
		glDepthMask(GL_FALSE);
		m_blendingenabled = true;
	}
	else
	{
		glDisable(GL_BLEND);
		glDepthMask(GL_TRUE);
		m_blendingenabled = false;
	}

	switch (m_renderpass.draw_mode)
	{
	case rendermode::color:
	case rendermode::reflections:
	{
		glCullFace(GL_BACK);
		break;
	}
	case rendermode::shadows:
	{
		glCullFace(GL_FRONT);
		break;
	}
	case rendermode::pickcontrols:
	case rendermode::pickscenery:
	{
		glCullFace(GL_BACK);
		break;
	}
	default:
	{
		break;
	}
	}
}

// puts the empty shadow map in place of the shadow map
void opengl46_renderer::setup_shadow_empty_map()
{
    if( false == Global.gfx_shadowmap_enabled ) { return; }

    m_shadow_empty_tex->bind( gl::SHADOW_TEX );
}

// binds shadow map and updates shadow map uniform data
void opengl46_renderer::setup_shadow_bind_map()
{
    if( false == Global.gfx_shadowmap_enabled ) { return; }

    m_shadow_tex->bind(gl::SHADOW_TEX);

	glm::mat4 coordmove;

	// transform 1..-1 NDC xy coordinates to 1..0 (with clip_control z is already 1..0)
	coordmove = glm::mat4( //
		0.5, 0.0, 0.0, 0.0, //
		0.0, 0.5, 0.0, 0.0, //
		0.0, 0.0, 1.0, 0.0, //
		0.5, 0.5, 0.0, 1.0 //
	);

    for( auto idx = 0; idx < m_shadowpass.size(); ++idx ) {

        glm::mat4 depthproj = m_shadowpass[ idx ].pass_camera.projection();
        // NOTE: we strip transformations from camera projections to remove jitter that occurs
        // with large (and unneded as we only need the offset) transformations back and forth
        auto const depthcam{ glm::mat3{ m_shadowpass[ idx ].pass_camera.modelview()} };
        auto const worldcam{ glm::mat3{ m_renderpass.pass_camera.modelview()} };

        scene_ubs.lightview[ idx ] =
            coordmove
            * depthproj
            * glm::translate(
                glm::mat4{ depthcam },
                glm::vec3{ m_renderpass.pass_camera.position() - m_shadowpass[ idx ].pass_camera.position() } )
            * glm::mat4{ glm::inverse( worldcam ) };
    }
	scene_ubo->update(scene_ubs);
}

void opengl46_renderer::setup_shadow_color( glm::vec4 const &Shadowcolor ) {

    model_ubs.shadow_tone = glm::pow( Shadowcolor.x, 2.2f );
}

void opengl46_renderer::setup_env_map(gl::cubemap *tex)
{
	if (tex)
	{
		tex->bind(GL_TEXTURE0 + gl::ENV_TEX);
		glActiveTexture(GL_TEXTURE0);
	}
	else
	{
		glActiveTexture(GL_TEXTURE0 + gl::ENV_TEX);
		glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
		glActiveTexture(GL_TEXTURE0);
	}
	// rough reflections read the prefiltered map along with the real environment map; with the empty stand-in (the reflection
	// pass) or without the map the shading keeps to the mipmaps of what is bound
	auto const *prefiltered{(tex != nullptr && tex == m_env_tex.get()) ? m_sky.prefiltered() : nullptr};
	m_envmap_bound = (prefiltered != nullptr);
	if (prefiltered != nullptr)
		prefiltered->bind(GL_TEXTURE0 + gl::ENV_GGX_TEX);
	else if (m_empty_cubemap)
		m_empty_cubemap->bind(GL_TEXTURE0 + gl::ENV_GGX_TEX);
	glActiveTexture(GL_TEXTURE0);
	opengl_texture::reset_unit_cache();
}

void opengl46_renderer::setup_environment_light(TEnvironmentType const Environment)
{

	switch (Environment)
	{
	case e_flat:
	{
        setup_sunlight_intensity();
		//            m_environment = Environment;
		break;
	}
	case e_canyon:
	{
		setup_sunlight_intensity(0.4f);
		//            m_environment = Environment;
		break;
	}
	case e_tunnel:
	{
		setup_sunlight_intensity(0.2f);
		//            m_environment = Environment;
		break;
	}
	default:
	{
		break;
	}
	}
}

void opengl46_renderer::setup_sunlight_intensity( float const Factor ) {

    m_sunlight.apply_intensity( Factor );
    light_data.lights[ 0 ].intensity = m_sunlight.factor * (1.0f - Global.Overcast * 0.33f);
    light_data.ambient = m_sunlight.ambient * m_sunlight.factor;
    light_data.sky_ambient_factor = ( Global.gfx_ambient_sh ? m_sunlight.factor : 0.f );
    upload_light_header();
}

// the lighting values which change from object to object within a pass (the sun in tunnels, the ambient light in a cab):
// the header of the light buffer and the sun in its first slot. the headlights are written once per pass by Update_Lights()
void opengl46_renderer::upload_light_header()
{
    if( m_scenerecording ) {
        // the recorded walk of Render_Scene_Depth() draws nothing lit: the draws which follow go with this header, uploaded
        // when the colour pass draws them
        auto &header { m_sceneheaders.emplace_back() };
        std::memcpy( header.data(), &light_data, header.size() );
        return;
    }
    light_ssbo->update( &light_data, 0, offsetof( gl::light_data, lights ) + sizeof( gl::light_element ) );
}

bool opengl46_renderer::Render(world_environment *Environment)
{
    m_shadowcolor = colors::white; // prevent shadow from affecting sky
    setup_shadow_color( m_shadowcolor );

	if (Global.bWireFrame)
	{
		// bez nieba w trybie rysowania linii
		return false;
	}

    Bind_Material(null_handle);
	::glPushMatrix();
	// the sky gets its fog all at once, laid over it with the clouds (opengl46_weather::draw_sky)
	auto const fogdensity{model_ubs.fog_density};
	model_ubs.fog_density = 0.f;

	// skydome
	// drawn with 500m radius to blend in if the fog range is low
	glPushMatrix();
    {
        glScalef( 500.0f, 500.0f, 500.0f );
        model_ubs.set_modelview( OpenGLMatrices.data( GL_MODELVIEW ) );
        auto const atmosphere{ m_sky.atmosphere_weight() > 0.f };
        if( atmosphere ) {
            m_sky.setup_sky( model_ubs );
        }
        model_ubo->update( model_ubs );

        m_skydomerenderer.update();
        m_skydomerenderer.render( atmosphere );
        // skydome uses a custom vbo which could potentially confuse the main geometry system. hardly elegant but, eh
        gfx::opengl46_vaogeometrybank::reset();
    }
    glPopMatrix();

    ::glBlendFunc( GL_SRC_ALPHA, GL_ONE );

	auto const &modelview = OpenGLMatrices.data(GL_MODELVIEW);

    // stars of the catalogue, turned by the sidereal time; the fog and the clouds go over them
    m_weather.draw_stars(modelview, Environment->m_sun.getAngle(), Environment->m_sun.getSiderealTime(), Environment->m_sun.getLatitude());
    ::glBlendFunc( GL_SRC_ALPHA, GL_ONE );

	// celestial bodies
	m_celestial_shader->bind();
	m_empty_vao->bind();
	// the bodies give their colour premultiplied, and as alpha how much of the sky behind them they hide (celestial.frag)
	::glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    // the fog and the clouds laid over the sky hide the sun and the moon; without the clouds the overcast fades them
    auto const overcastfactor{m_weather.clouds_drawn() ? 1.f : std::clamp(1.5f - Global.Overcast, 0.f, 1.f)};
	float const duskfactor = 1.0f - std::clamp(std::abs(Environment->m_sun.getAngle()), 0.0f, 12.0f) / 12.0f;
	glm::vec3 suncolor = glm::mix(glm::vec3(1.0f, 242.0f / 255.0f, 231.0f / 255.0f), glm::vec3(235.0f / 255.0f, 140.0f / 255.0f, 36.0f / 255.0f), duskfactor);
	suncolor = m_sky.sun_disc_color(suncolor);

	// the vertical of the sky on the quads of the bodies, which lie in the plane of the view
	glm::vec2 zenith{glm::mat3{modelview} * glm::vec3{0.f, 1.f, 0.f}};
	zenith = (glm::length(zenith) > 1e-3f ? glm::normalize(zenith) : glm::vec2{0.f, 1.f});
	// a disc low over the horizon is flattened by the refraction: its lower edge is lifted more than the upper one (refraction
	// after Bennett, for the apparent elevation). height over width of the disc
	auto const flattening = [](float const Elevation, float const Radius) {
		auto const refraction = [](float const Height) {
			auto const height{std::max(Height, -2.f)};
			return 1.f / std::tan(glm::radians(height + 7.31f / (height + 4.4f))); // arc minutes
		};
		auto const radius{glm::degrees(Radius)};
		return std::clamp(1.f - (refraction(Elevation - radius) - refraction(Elevation + radius)) / (2.f * radius * 60.f), 0.5f, 1.f);
	};
	// the view of the frame gets the sun as bright as it is, which gives the glare of the bloom; the other views (the
	// environment map, mirrors) keep the old dim disc, the reflections of the sun on surfaces come from their shading
	auto const mainview{(m_renderpass.draw_mode == rendermode::color) && (m_current_viewport != nullptr) && m_current_viewport->main};
	std::array<glm::vec4, 3> body{};

	// sun
	{
		// the true size of the disc, 0.533 degrees across, the quad a little larger for a smooth edge
		float const sunradius{glm::radians(0.2666f)};
		float const quadmargin{1.25f};
		// limb darkening per channel (linear law): the edge of the disc is darker and redder than its centre. the centre is
		// this many times as bright as the disc on average
		glm::vec3 const limbdarkening{0.50f, 0.60f, 0.75f};
		float const limbpeak{1.f / (1.f - 0.75f / 3.f)};
		// the brightest the centre of the disc gets: within the range of the half floats of the image, with the sky added
		float const radiancemax{60000.f};

		auto const sunvector = Environment->m_sun.getDirection();
		glm::vec3 radiance{suncolor * overcastfactor};
		if (mainview)
		{
			// radiance of the disc: the illuminance of its sunlight over its solid angle. the shading lights a surface facing
			// the sun with the diffuse colour times SUN_DIFFUSE_SCALE (1.5) and has no 1/pi in its lambert term
			// (light_common.glsl), so its surfaces are pi times as bright as physical ones; the disc is put in the same units.
			// the overcast without the clouds drawn hides the sun, where it only faded the old disc
			auto const solidangle{2.f * glm::pi<float>() * (1.f - std::cos(sunradius))};
			auto const hidden{m_weather.clouds_drawn() ? 0.f : glm::smoothstep(0.5f, 1.f, Global.Overcast)};
			auto const physical{glm::vec3{m_sunlight.diffuse} * (1.5f * glm::pi<float>() / solidangle * Global.gfx_sun_disc)};
			radiance = glm::mix(physical, radiance, hidden);
			auto const peak{std::max({radiance.r, radiance.g, radiance.b}) * limbpeak};
			if (peak > radiancemax)
				radiance *= radiancemax / peak;
		}
		body[0] = glm::vec4{zenith, flattening(Environment->m_sun.getAngle(), sunradius), 1.f / quadmargin};
		body[1] = glm::vec4{0.f};
		body[2] = glm::vec4{limbdarkening, 0.f};
		glProgramUniform4fv(*m_celestial_shader, 0, static_cast<GLsizei>(body.size()), glm::value_ptr(body[0]));
		model_ubs.param[0] = glm::vec4{radiance, 1.f};
		model_ubs.param[1] = glm::vec4(glm::vec3(modelview * glm::vec4(sunvector, 1.0f)), std::tan(sunradius) * quadmargin);
		model_ubs.param[2] = glm::vec4(0.f, 0.f, 0.f, 1.f);
		model_ubo->update(model_ubs);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	}
	// moon
	{
		// the disc of the full moon in the picture of the phases (fx/moon, 3x3): centre and radius in texture units
		glm::vec2 const discposition{0.5f, 0.5f};
		float const discradius{0.1635f};
		float const quadmargin{1.1f};
		// light of the earth on the dark part of the moon, against the light of the sun on the lit part, at the new moon
		float const earthshine{0.005f};
		// extinction of the air per air mass (natural log), red, green and blue
		glm::vec3 const extinction{0.10f, 0.17f, 0.30f};

		Bind_Texture(0, m_moontexture);
    	glm::vec3 mooncolor(1.0f, 0.9490196f, 0.9058824f);
		// fade the moon if it's near the sun in the sky, especially during the day
		float visibility = std::max<float>(0.f, 1.0 - 0.5 * Global.fLuminance - 0.65 * std::max(0.f, glm::dot(Environment->m_sun.getDirection(), Environment->m_moon.getDirection()))) * overcastfactor;
		if (mainview && (false == Global.gfx_skippipeline))
		{
			// in the image of the frame the moon is as bright as its surface in the sunlight: about the diffuse colour of the
			// sun (1.0 .. 1.45) times SUN_DIFFUSE_SCALE (1.5, light_common.glsl), taken down by the albedo of the real moon (0.12)
			// against that of the picture (about 0.19). it is laid over the sky, which hides it by day only as much as the real
			// sky does: a pale moon by day, a bright one at night
			visibility = 1.4f * overcastfactor;
		}

		auto const moonvector = Environment->m_moon.getDirection();
		auto const moonradius{Environment->m_moon.getAngularRadius()};
		auto const moonelevation{Environment->m_moon.getAngle()};
		auto const moonview{glm::vec3(modelview * glm::vec4(moonvector, 1.0f))};

		if (Environment->m_moon.getPhase() == 50)
		{
			// 9th slot of the picture used for easter egg, drawn as it is
			model_ubs.param[0] = glm::vec4(mooncolor, visibility);
			model_ubs.param[1] = glm::vec4(moonview, std::tan(moonradius) * (0.1665f / discradius));
			model_ubs.param[2] = glm::vec4(0.66f, 1.0f - 0.667f, 0.333f, 0.f);
		}
		else
		{
			// the air dims and reddens the moon low over the horizon (air mass after Kasten and Young 1989), against its colour
			// high in the sky
			auto const zenithangle{90.f - std::max(moonelevation, -1.f)};
			auto const airmass{1.f / (std::cos(glm::radians(zenithangle)) + 0.50572f * std::pow(std::max(96.07995f - zenithangle, 0.1f), -1.6364f))};
			auto const transmittance{glm::exp(-extinction * (airmass - 1.f))};
			// the dark part of the moon hides the stars behind it. the sky behind it isn't hidden in the twilight (its light is
			// scattered in front of the moon), so this waits until the sky is dark
			auto const night{1.f - glm::smoothstep(-15.f, -9.f, Environment->m_sun.getAngle())};
			// the moon is lit from the direction of the sun, which shapes its phase; the earth, full at the new moon, lights its
			// dark part a little
			auto const earthphase{0.5f * (1.f - std::cos(glm::radians(Environment->m_moon.getPhaseAngle())))};
			body[0] = glm::vec4{zenith, flattening(moonelevation, moonradius), 1.f / quadmargin};
			body[1] = glm::vec4{glm::normalize(glm::mat3{modelview} * Environment->m_sun.getDirection()), earthshine * earthphase};
			body[2] = glm::vec4{0.f};
			glProgramUniform4fv(*m_celestial_shader, 0, static_cast<GLsizei>(body.size()), glm::value_ptr(body[0]));
			model_ubs.param[0] = glm::vec4(mooncolor * transmittance * (visibility * Global.gfx_moon_disc), night * overcastfactor);
			model_ubs.param[1] = glm::vec4(moonview, std::tan(moonradius) * quadmargin);
			model_ubs.param[2] = glm::vec4(discposition, discradius, 2.f);
		}
		model_ubo->update(model_ubs);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	}

    ::glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  	// clouds of the weather and the fog over the sky: of the main view in its colour pass, of the panorama around the camera in
	// the other views and in the faces of the environment map
	{
		auto const mainview{(m_renderpass.draw_mode == rendermode::color) && (m_current_viewport != nullptr) && m_current_viewport->main};
		m_empty_vao->bind();
		m_weather.draw_sky(OpenGLMatrices.data(GL_PROJECTION), modelview, mainview);
	}

	gl::program::unbind();
	gl::vao::unbind();
    ::glPopMatrix();
	model_ubs.fog_density = fogdensity;
	model_ubo->update(model_ubs);

	m_sunlight.apply_angle();
	m_sunlight.apply_intensity();

    // calculate shadow tone, based on positions of celestial bodies
	m_shadowcolor = glm::mix(glm::vec4{colors::shadow}, glm::vec4{colors::white}, std::clamp(-Environment->m_sun.getAngle(), 0.f, 6.f) / 6.f);
	if ((Environment->m_sun.getAngle() < -18.f) && (Environment->m_moon.getAngle() > 0.f))
	{
		// turn on moon shadows after astronomical twilight, if the moon is actually up. they come in with the moonlight
		// (simulation/simulationenvironment.cpp) and are as deep as it is strong: full from the gibbous moon, faint from a crescent
		auto const moonshadow{std::clamp(4.f * Environment->m_moon.getBrightness() * Global.gfx_moonlight, 0.f, 1.f) * std::clamp((-18.f - Environment->m_sun.getAngle()) / 2.f, 0.f, 1.f) *
		                      std::clamp(Environment->m_moon.getAngle() / 5.f, 0.f, 1.f)};
		m_shadowcolor = glm::mix(glm::vec4{colors::white}, glm::vec4{colors::shadow}, moonshadow);
	}
	// soften shadows depending on sky overcast factor, unless the shadows of the clouds hold back the sun where they fall
	if (false == m_weather.cloud_shadows())
		m_shadowcolor = glm::min(colors::white, m_shadowcolor + ((colors::white - colors::shadow) * Global.Overcast));

    setup_shadow_color( m_shadowcolor);

	return true;
}

// geometry methods
// creates a new geometry bank. returns: handle to the bank or NULL
gfx::geometrybank_handle opengl46_renderer::Create_Bank()
{
        return m_geometry.register_bank(std::make_unique<gfx::opengl46_vaogeometrybank>());
}

// creates a new indexed geometry chunk of specified type from supplied data, in specified bank. returns: handle to the chunk or NULL
gfx::geometry_handle opengl46_renderer::Insert(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type)
{
    // NOTE: we expect indexed geometry to come with calculated tangents
    return m_geometry.create_chunk( Indices, Vertices, Userdata, Geometry, Type );
}

// creates a new geometry chunk of specified type from supplied data, in specified bank. returns: handle to the chunk or NULL
gfx::geometry_handle opengl46_renderer::Insert(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type)
{
	gfx::calculate_tangents(Vertices, gfx::index_array(), Type);

	return m_geometry.create_chunk(Vertices, Userdata, Geometry, Type);
}

// replaces data of specified chunk with the supplied vertex data, starting from specified offset
bool opengl46_renderer::Replace(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type, const std::size_t Offset)
{
	gfx::calculate_tangents(Vertices, gfx::index_array(), Type);

	m_chunkspheres.erase(static_cast<std::uint64_t>(Geometry));
	return m_geometry.replace(Vertices, Userdata, Geometry, Offset);
}

// replaces the index and the vertex data of specified indexed chunk with the supplied data of the same counts
bool opengl46_renderer::Replace_Indexed(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry)
{
	// NOTE: we expect indexed geometry to come with calculated tangents
	m_chunkspheres.erase(static_cast<std::uint64_t>(Geometry));
	return m_geometry.replace(Indices, Vertices, Userdata, Geometry);
}

// adds supplied vertex data at the end of specified chunk
bool opengl46_renderer::Append(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type)
{
	gfx::calculate_tangents(Vertices, gfx::index_array(), Type);

	m_chunkspheres.erase(static_cast<std::uint64_t>(Geometry));
	return m_geometry.append(Vertices, Userdata, Geometry);
}

// provides direct access to index data of specfied chunk
gfx::index_array const & opengl46_renderer::Indices(gfx::geometry_handle const &Geometry) const 
{
    return m_geometry.indices(Geometry);
}

// provides direct access to vertex data of specfied chunk
gfx::vertex_array const &opengl46_renderer::Vertices(gfx::geometry_handle const &Geometry) const
{
	return m_geometry.vertices(Geometry);
}

gfx::userdata_array const &opengl46_renderer::UserData(const gfx::geometry_handle &Geometry) const
{
	return m_geometry.userdata(Geometry);
}

int opengl46_renderer::Primitive(gfx::geometry_handle const &Geometry) const
{
	return static_cast<int>(m_geometry.type(Geometry));
}

// material methods
material_handle opengl46_renderer::Fetch_Material(std::string const &Filename, bool const Loadnow)
{

	return m_materials.create(Filename, Loadnow);
}

material_handle opengl46_renderer::Matte_Material(material_handle const Material)
{

	return m_materials.create_matte(Material);
}

material_handle opengl46_renderer::Tinted_Material(material_handle const Material, glm::vec3 const &Colour)
{

	return m_materials.create_tinted(Material, Colour);
}

material_handle opengl46_renderer::Blend_Material(material_handle const Material, std::array<material_handle, 3> const &Others)
{
	return m_materials.create_blend(Material, Others);
}

bool opengl46_renderer::Blendable_Material(material_handle const Material)
{
	return m_materials.blendable(Material);
}

std::string opengl46_renderer::Shader_Name(std::shared_ptr<gfx::shader_program> const &Program) const
{
	for (auto const &shader : m_shaders)
		if (shader.second == Program)
			return shader.first;
	return {};
}

std::shared_ptr<gfx::shader_program> opengl46_renderer::Fetch_Shader(const std::string &name)
{
	auto it = m_shaders.find(name);
	if (it == m_shaders.end())
	{
		gl::shader fragment("mat_" + name + ".frag");
		gl::program *program = new gl::program({fragment, *m_vertex_shader.get()});
		m_shaders.insert({name, std::shared_ptr<gl::program>(program)});
		// the variant for the multi draws of the instances, made now rather than when the instances first come into view
		if ((Global.gfx_gpucull == 1) && Global.gfx_gpucull_multidraw)
			Multidraw_Program(program);
	}

	return m_shaders[name];
}

void opengl46_renderer::Bind_Material( material_handle const Material, TSubModel const *sm, lighting_data const *lighting )
{
	if (Material != null_handle)
	{
        auto const &material { m_materials.material( Material ) };

        if( false == material.is_good ) {
            // use failure indicator instead
            if( Material != m_invalid_material ) {
                Bind_Material( m_invalid_material );
            }
            return;
        }

		memcpy(&model_ubs.param[0], &material.params[0], sizeof(model_ubs.param));

		for( auto const &entry : material.params_state )
		{
			glm::vec4 src(1.0f);

            // submodel-based parameters
			if (sm)
			{
                switch( entry.defaultparam ) {
                    case gl::shader::defaultparam_e::ambient: {
                        src = sm->f4Ambient;
                        break;
                    }
                    case gl::shader::defaultparam_e::diffuse: {
                        src = sm->f4Diffuse;
                        break;
                    }
                    case gl::shader::defaultparam_e::specular: {
                        src = sm->f4Specular;
                        break;
                    }
                    default: {
                        break;
                    }
                }
			}
            if( lighting )                 
            {
                switch( entry.defaultparam ) {
                    case gl::shader::defaultparam_e::ambient: {
                        src = lighting->ambient;
                        break;
                    }
                    case gl::shader::defaultparam_e::diffuse: {
                        src = lighting->diffuse;
                        break;
                    }
                    case gl::shader::defaultparam_e::specular: {
                        src = lighting->specular;
                        break;
                    }
                    default: {
                        break;
                    }
                }
            }
            // material-based parameters
            switch( entry.defaultparam ) {
                case gl::shader::defaultparam_e::glossiness: {
                    src = glm::vec4 { material.glossiness };
                }
                default: {
                    break;
                }
            }

            if( entry.size == 1 ) {
                // HACK: convert color to luminosity, if it's passed as single value
                src = glm::vec4 { colors::RGBtoHSV( glm::vec3 { src } ).z };
            }
			for (size_t j = 0; j < entry.size; j++)
				model_ubs.param[entry.location][entry.offset + j] = src[j];
		}
/*
		if (m_blendingenabled)
		{
			model_ubs.opacity = -1.0f;
		}
		else
		{
			if (!std::isnan(material.opacity))
				model_ubs.opacity = material.opacity;
			else
				model_ubs.opacity = 0.5f;
		}
*/

        if( material.opacity ) {
            model_ubs.opacity = (
                m_blendingenabled ?
                    -material.opacity.value() :
                     material.opacity.value() );
        }
        else {
            model_ubs.opacity = (
                m_blendingenabled ?
                     0.0f :
                     0.5f );
        }

		if (sm)
			model_ubs.alpha_mult = sm->fVisible;
		else
			model_ubs.alpha_mult = 1.0f;

		if (m_indirectrecording)
		{
			// the walk of Render_Instances_Indirect() only notes what would be bound, it's bound when the draw is made
			m_indirectmaterial = Material;
			m_indirectsubmodel = sm;
			return;
		}
		if (m_scenerecording)
		{
			// the recorded walk of Render_Scene_Depth(): the draws which follow are made with this material, bound later
			m_scenematerial = Material;
			m_scenesubmodel = sm;
			m_sceneseed = TSubModel::iVariantSeed;
			m_scenevariant = (material.texture_variants.empty() ? 0 : TSubModel::VariantId(sm));
			return;
		}

		// materials which declare texture variants provide a different texture set per model instance,
		// picked from the seed of the rendered instance and the variant group of the submodel
		auto const &materialtextures{
			material.texture_variants.empty() ?
		        material.textures :
		        material.GetTextures(TSubModel::VariantId(sm))};

		if (m_scenedepthpass)
		{
			// the depth of the whole scene: the programs of the depth prepass, the same choice as for the instances in
			// Render_Instances_Indirect(). what they can't draw (blended, materials of other programs) is left to the colour pass
			m_scenedepthskip = true;
			if (m_blendingenabled || (model_ubs.opacity < 0.f))
				return;
			auto const prepassclass{Prepass_Class(Material)};
			if (prepassclass == 0)
				return;
			if (prepassclass == 2)
			{
				// a texture which isn't there (yet) would leave the one bound before to the alpha test
				if ((materialtextures[0] == null_handle) || (false == m_textures.mark_as_used(materialtextures[0]).bind(0)))
					return;
				m_prepass_alpha_shader->bind();
			}
			else
			{
				m_prepass_shader->bind();
			}
			m_scenedepthskip = false;
			return;
		}

		GLuint lastdiff = 0;
		size_t i;
		for (i = 0; i < gl::MAX_TEXTURES; i++)
			if (materialtextures[i] != null_handle)
			{
				opengl_texture &tex = m_textures.mark_as_used(materialtextures[i]);
				tex.create();
				if (opengl_texture::units[i] != tex.id)
				{
					opengl_texture::units[i] = tex.id;
					lastdiff = i + 1;
				}
			}
			else
				break;

		if (lastdiff)
			glBindTextures(0, lastdiff, &opengl_texture::units[0]);

		auto *program{static_cast<gl::program *>(material.shader.get())};
		if (m_bindmultidraw)
		{
			// the replay of the instances drawn from the gpu culling with multi draws (Render_Instances_Indirect_Colour())
			if (auto *multidraw{Multidraw_Program(program)})
				program = multidraw;
		}
		program->bind();
	}
	else if (Material != m_invalid_material)
		Bind_Material(m_invalid_material);
}

void opengl46_renderer::Bind_Material_Shadow(material_handle const Material)
{
	if (m_indirectrecording)
	{
		// the walk of Render_Instances_Indirect_Pass() only notes the material, the program and the texture are bound when the
		// draw is made (Render_Instances_Indirect_Shadow())
		m_indirectmaterial = Material;
		m_indirectsubmodel = nullptr;
		return;
	}
	if (Material != null_handle)
	{
		auto &material = m_materials.material(Material);

		if (material.textures[0] != null_handle)
		{
			m_textures.bind(0, material.textures[0]);
			if (m_cubeshadows)
				m_localshadows.program(true).bind();
			else
				m_alpha_shadow_shader->bind();
		}
        else if (m_cubeshadows) {
            m_localshadows.program(false).bind();
        }
        else {
            m_shadow_shader->bind();
        }
	}
    else if (m_cubeshadows) {
        m_localshadows.program(false).bind();
    }
    else {
        m_shadow_shader->bind();
    }
}

IMaterial const *opengl46_renderer::Material(material_handle const Material) const
{
	return &m_materials.material(Material);
}

material_handle opengl46_renderer::Terrain_Material(material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement,
                                                     int const HoleSamples, std::uint8_t const *Holes)
{
	auto const material{m_terrainmaterials.make(Reuse, Layers, Samples, Weights, Placement, HoleSamples, Holes)};
	if (material != null_handle)
		return material;
	// the terrain shader can't be used, the chunk makes do with a plain material
	return gfx_renderer::Terrain_Material(Reuse, Layers, Samples, Weights, Placement, HoleSamples, Holes);
}

void opengl46_renderer::Terrain_Release(material_handle const Material)
{
	m_terrainmaterials.release(Material);
}

void opengl46_renderer::Terrain_Grass(std::vector<gfx::grass_kind> const &Kinds, std::vector<gfx::grass_chunk> const &Chunks)
{
	m_grass.update(Kinds, Chunks);
}

opengl_material &opengl46_renderer::Material(material_handle const Material)
{
	return m_materials.material(Material);
}

opengl_material const & opengl46_renderer::Material( TSubModel const * Submodel ) const {

    return m_materials.material( submodel_material( Submodel ) );
}

texture_handle opengl46_renderer::Fetch_Texture(std::string const &Filename, bool const Loadnow, GLint format_hint)
{
	return m_textures.create(Filename, Loadnow, format_hint);
}

void opengl46_renderer::Bind_Texture( texture_handle const Texture )
{
    return Bind_Texture( 0, Texture );
}

void opengl46_renderer::Bind_Texture(std::size_t const Unit, texture_handle const Texture)
{
	m_textures.bind(Unit, Texture);
}

ITexture &opengl46_renderer::Texture(texture_handle const Texture)
{

	return m_textures.texture(Texture);
}

ITexture const &opengl46_renderer::Texture(texture_handle const Texture) const
{

	return m_textures.texture(Texture);
}

void opengl46_renderer::Update_AnimModel(TAnimModel *model)
{
	model->RaAnimate(m_framestamp);
}


void opengl46_renderer::Draw_Geometry(std::vector<gfx::geometrybank_handle>::iterator begin, std::vector<gfx::geometrybank_handle>::iterator end)
{
	if (begin == end)
	{
		return;
	}
	// the handles are handed over as a whole, so chunks kept in the same bank can be drawn with fewer calls
	auto const *first{&(*begin)};
	m_geometry.draw_batch(first, first + std::distance(begin, end));
}

void opengl46_renderer::Draw_Geometry(const gfx::geometrybank_handle &handle)
{
	m_geometry.draw(handle);
}

void opengl46_renderer::draw(const gfx::geometry_handle &handle)
{
	model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
	if( m_indirectrecording ) {
		// the walk of Render_Instances_Indirect(): the draw is made later, from the gpu culling, with this state
		Record_Indirect_Draw( handle );
		return;
	}
	if( m_scenerecording ) {
		// the recorded walk of Render_Scene_Depth(): the depth pass and the colour pass draw it later, with this state
		Record_Scene_Draw( &handle, 1, false );
		return;
	}
	if( m_scenedepthpass && ( false == Scene_Depth_Draws( handle ) ) ) {
		return;
	}
	model_ubo->update(model_ubs);

	if( m_current_instance_count > 0 ) {
		// inside Render_Instanced(): one GL instanced draw replaces what would
		// otherwise be N regular draws (one per instance) at this submodel
		m_geometry.draw_instanced( handle, m_current_instance_count, m_current_instance_base );
	}
	else {
		m_geometry.draw( handle );
	}
}

void opengl46_renderer::draw(std::vector<gfx::geometrybank_handle>::iterator it, std::vector<gfx::geometrybank_handle>::iterator end)
{
	model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
	if( m_scenerecording ) {
		if( it != end ) {
			Record_Scene_Draw( &( *it ), static_cast<std::size_t>( std::distance( it, end ) ), true );
		}
		return;
	}
	if( m_scenedepthpass && ( ( it == end ) || ( false == Scene_Depth_Draws( *it ) ) ) ) {
		return;
	}
	model_ubo->update(model_ubs);

	Draw_Geometry(it, end);
}



// places supplied modelview matrices in the instance buffer of the frame. returns: base instance of the first of them
std::uint32_t opengl46_renderer::Upload_Instance_Modelviews( glm::mat4 const *Modelviews, std::size_t const Count )
{
	auto const generation { m_instancestream->generation() };
	auto const offset { m_instancestream->push( Modelviews, static_cast<GLsizeiptr>( Count * sizeof( glm::mat4 ) ) ) };
	if( m_instancestream->generation() != generation ) {
		// the draws issued with the replaced buffer keep it until the gpu is done with them. the new one starts with identity
		Bind_Instance_Stream();
	}
	return static_cast<std::uint32_t>( offset / static_cast<GLintptr>( sizeof( glm::mat4 ) ) );
}

void opengl46_renderer::Bind_Instance_Stream() const
{
	GLuint const buffer { m_instancestream->buffer() };
	::glBindBuffersBase( GL_SHADER_STORAGE_BUFFER, gl::INSTANCE_SSBO_BINDING, 1, &buffer );
}


// how the depth prepass treats the material. the colour program of the material decides: the ones which test the alpha of the
// diffuse texture against the opacity (mat_default.frag and the like) get the same test, the ones which never discard a
// fragment get none, and any other (parallax, water, clouds, unknown) is left out and writes its depth in the colour pass
int opengl46_renderer::Prepass_Class( material_handle const Material )
{
	if( Material == null_handle ) { return 0; }
	auto const &material { m_materials.material( Material ) };
	if( ( false == material.is_good ) || ( material.shader == nullptr ) ) { return 0; }
	auto const lookup { m_prepassclasses.find( static_cast<gl::program const *>( material.shader.get() ) ) };
	if( lookup != m_prepassclasses.end() ) { return lookup->second; }

	// names the programs are fetched by: mat_<name>.frag
	static std::unordered_set<std::string> const alphatested {
		"default", "default_1", "default_2", "default_detail", "default_specgloss",
		"detail_normalmap", "detail_normalmap_specgloss", "normalmap", "normalmap_specgloss",
		"reflmap", "reflmap_specgloss", "shadowlessnormalmap", "shadowlessnormalmap_specgloss",
		"sunlessnormalmap", "sunlessnormalmap_specgloss",
		"trackblend", "trackblend_normalmap", "trackblend_specgloss" };
	static std::unordered_set<std::string> const solid { "colored", "default_0", "terrain", "water", "water_specgloss", "invalid" };
	auto prepassclass { 0 };
	for( auto const &shader : m_shaders ) {
		if( shader.second.get() != material.shader.get() ) { continue; }
		prepassclass = ( alphatested.count( shader.first ) > 0 ? 2 : solid.count( shader.first ) > 0 ? 1 : 0 );
		break;
	}
	m_prepassclasses.emplace( static_cast<gl::program const *>( material.shader.get() ), prepassclass );
	return prepassclass;
}

// draws the instances queued in the pass from the result of the gpu culling.
// the submodel trees are walked the way Render_Instanced() walks them, once per lod band of each group, but the walk only
// records the draws (draw() and Bind_Material() note what they would do, see m_indirectrecording): the instances of a band
// are known only on the gpu, so each draw becomes an indirect draw command whose instances the gpu fills in.
// bands no instance of the group can fall in, by the distance of the cells which hold them, are left out
void opengl46_renderer::Render_Instances_Indirect()
{
	auto const cpustart { std::chrono::steady_clock::now() };
	auto *viewport { m_current_viewport };
	// the cpu time of the parts, for the panel
	auto partstart { cpustart };
	auto const part = [ &partstart ]( double &Part ) {
		auto const now { std::chrono::steady_clock::now() };
		Part += std::chrono::duration<double, std::milli>( now - partstart ).count();
		partstart = now; };
	m_cullparts = {};

	if( m_scenerecording ) {
		// the depth of what the recorded walk of the depth pass of the scene went through so far, for the occlusion test
		// and for the depth of the instances to be tested against
		Render_Scene_Prepass();
	}
	part( m_cullparts.scenedepth );
	// occlusion test against the depth drawn so far in the pass: cab, terrain, shapes, tracks, vehicles, other models
	opengl46_instance_cull::occlusion_setup occlusion;
	auto const useocclusion {
		Global.gfx_gpucull_occlusion
		&& ( false == Global.gfx_skippipeline )
		&& ( viewport != nullptr )
		&& ( viewport->msaa_fb != nullptr ) };
	if( useocclusion ) {
		m_hiz.build( *viewport->msaa_fb, viewport->width, viewport->height );
		occlusion.pyramid = m_hiz.texture();
		occlusion.size = m_hiz.size();
		occlusion.levels = m_hiz.levels();
		occlusion.screen = glm::vec2{ m_hiz.source_size() };
		occlusion.projection = OpenGLMatrices.data( GL_PROJECTION );
	}
	part( m_cullparts.pyramid );
	// in two phases the instances seen in the previous frame are drawn first, and the others are tested against a pyramid
	// made again with their depth in it (not in wireframe, which doesn't draw surfaces)
	m_instancecull.dispatch(
		OpenGLMatrices.data( GL_MODELVIEW ),
		( useocclusion ? &occlusion : nullptr ),
		( Global.gfx_gpucull_occlusion_instances && ( false == Global.bWireFrame ) ) );
	part( m_cullparts.culling );

	// the draws, recorded while the gpu culls
	Record_Indirect_Draws( m_instancecull );
	part( m_cullparts.record );

	using command_set = opengl46_instance_cull::command_set;
	auto const twophase { m_instancecull.two_phase() };
	auto const draws { false == m_indirectdraws.empty() };
	// the depth comes first in the depth pass of the whole scene, which has its state set and leaves the colour to the colour
	// pass (m_indirectcolourpending), and with the depth prepass of the instances: the colour pass shades only what stays in
	// view. the depth goes a little farther than the surface, so the colour pass, which tests and writes its depth as usual,
	// is sure to pass where it's the same
	auto const depthfirst { m_scenedepthpass || ( Global.gfx_depthprepass && ( false == Global.bWireFrame ) ) };
	indirect_counts prepasscounts;
	indirect_counts colourcounts;
	auto colourset { command_set::first };
	if( twophase ) {
		if( draws ) {
			// the instances seen in the previous frame, then the pyramid again, with them in it
			m_instancecull.bind_draw();
			if( depthfirst ) {
				prepasscounts += Render_Instances_Indirect_Prepass( command_set::first, m_prepassfirsttimer );
				part( m_cullparts.depth );
			}
			else {
				colourcounts += Render_Instances_Indirect_Colour( command_set::first );
				part( m_cullparts.colour );
			}
			m_hiz.build( *viewport->msaa_fb, viewport->width, viewport->height, true );
			occlusion.pyramid = m_hiz.texture();
			part( m_cullparts.pyramid );
		}
		// the others are tested against it; with no draws the instances are only marked for the next frame
		m_instancecull.dispatch_second( occlusion );
		colourset = ( depthfirst ? command_set::all : command_set::second );
		part( m_cullparts.culling );
	}
	if( draws ) {
		m_instancecull.bind_draw();
		if( depthfirst ) {
			prepasscounts += Render_Instances_Indirect_Prepass( ( twophase ? command_set::second : command_set::first ), m_prepasstimer );
			part( m_cullparts.depth );
		}
		if( false == m_scenedepthpass ) {
			colourcounts += Render_Instances_Indirect_Colour( colourset );
		}
		::glBindBuffer( GL_DRAW_INDIRECT_BUFFER, 0 );
		Bind_Instance_Stream();
		part( m_cullparts.colour );
	}
	if( m_scenedepthpass ) {
		// the colour pass takes the draws from here, and counts them
		m_indirectcolourpending = true;
		m_indirectcolourset = colourset;
		m_indirectcoloursleepers = m_sleeperspass;
		m_indirectcolourmodels = m_modelspass;
		if( m_scenerecording ) {
			// where the colour pass, which draws the record, draws them, in the light the walk had here
			auto &marker { m_scenedraws.emplace_back() };
			marker.instances = true;
			marker.header = static_cast<std::uint32_t>( m_sceneheaders.size() - 1 );
		}
		m_renderpass.draw_stats.drawcalls += static_cast<int>( prepasscounts.calls );
		colourcounts.draws = static_cast<std::uint32_t>( m_indirectdraws.size() );
	}
	else {
		m_renderpass.draw_stats.instanced_drawcalls += static_cast<int>( colourcounts.calls );
		m_renderpass.draw_stats.drawcalls += static_cast<int>( prepasscounts.calls );
		// the count of the frame the gpu reported last
		m_renderpass.draw_stats.instances += static_cast<int>( m_instancecull.last_count() );
		m_renderpass.draw_stats.models += static_cast<int>( m_instancecull.last_count() );
	}
	m_instancecull.draw_result(
		colourcounts.draws,
		prepasscounts.draws,
		colourcounts.calls,
		prepasscounts.calls,
		std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - cpustart ).count() );
}

// draws the instances of a pass other than the main view from the result of its gpu culling (m_passcull): the culling of the
// pass with its own camera, no occlusion test, the draws recorded the same way, then drawn once with the programs the pass
// draws with on the cpu
void opengl46_renderer::Render_Instances_Indirect_Pass()
{
	auto const cpustart { std::chrono::steady_clock::now() };
	auto &cull { Pass_Cull() };
	cull.dispatch( OpenGLMatrices.data( GL_MODELVIEW ) );
	Record_Indirect_Draws( cull );

	using command_set = opengl46_instance_cull::command_set;
	indirect_counts counts;
	if( false == m_indirectdraws.empty() ) {
		cull.bind_draw();
		counts = (
			m_renderpass.draw_mode == rendermode::shadows ?
				Render_Instances_Indirect_Shadow( command_set::first ) :
				Render_Instances_Indirect_Colour( command_set::first ) );
		::glBindBuffer( GL_DRAW_INDIRECT_BUFFER, 0 );
		Bind_Instance_Stream();
	}
	m_renderpass.draw_stats.instanced_drawcalls += static_cast<int>( counts.calls );
	// the count of the frame the gpu reported last
	m_renderpass.draw_stats.instances += static_cast<int>( cull.last_count() );
	m_renderpass.draw_stats.models += static_cast<int>( cull.last_count() );
	cull.draw_result( counts.draws, 0, counts.calls, 0, std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - cpustart ).count() );
}

void opengl46_renderer::Record_Indirect_Draws( opengl46_instance_cull &Cull )
{
	m_indirectcull = &Cull;
	// the cascades of the sun shadows share the walks
	auto const cascade {
		( &Cull >= m_shadowcull.data() )
		&& ( &Cull < m_shadowcull.data() + m_shadowcull.size() ) };
	Walk_Indirect_Draws( Cull, cascade );
	auto const orderstart { std::chrono::steady_clock::now() };

	// with multi draws the draws with the same state follow one another, and their model blocks are in a buffer the programs
	// of the multi draws read them from. the order goes into the commands, so it comes first
	m_indirectmultidraw = ( Global.gfx_gpucull_multidraw && ( m_drawblockstream != nullptr ) );
	if( m_indirectmultidraw && ( false == m_indirectdraws.empty() ) ) {
		Order_Indirect_Draws();
		Upload_Indirect_Blocks();
	}

	Cull.fill_commands( m_indirectcommands );
	m_walkcounts.ordertime += std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - orderstart ).count();
}

opengl46_renderer::indirect_counts opengl46_renderer::Render_Instances_Indirect_Prepass( opengl46_instance_cull::command_set const Set, gl::gpu_timer &Timer )
{
	if( m_scenedepthpass ) {
		return Render_Instances_Indirect_Depth( Set );
	}
	Timer.begin();
	setup_drawing( false );
	::glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE );
	::glEnable( GL_POLYGON_OFFSET_FILL );
	::glPolygonOffset( -1.f, -1.f );
	auto const counts { Render_Instances_Indirect_Depth( Set ) };
	::glDisable( GL_POLYGON_OFFSET_FILL );
	::glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	Timer.end();
	return counts;
}

opengl46_renderer::indirect_counts opengl46_renderer::Render_Instances_Indirect_Depth( opengl46_instance_cull::command_set const Set )
{
	// the draws were recorded with the seed of scenery models and sleepers, 0 (TAnimModel::RaPrepare()); vehicles drawn since
	// leave their own, which would pick other texture variants than those the draws were recorded and ordered with
	TSubModel::iVariantSeed = 0;
	indirect_counts counts;
	auto const count { m_indirectdraws.size() };
	// whether the prepass of the class draws the draw, and with which diffuse texture (class 2)
	auto const prepass_texture = [ this ]( std::size_t const Index, int const Prepassclass, texture_handle &Texture ) -> bool {
		auto const &draw { m_indirectdraws[ Index ] };
		if( ( draw.blending ) || ( draw.ubs.opacity < 0.f ) ) { return false; }
		if( ( draw.type != GL_TRIANGLES ) && ( draw.type != GL_TRIANGLE_STRIP ) && ( draw.type != GL_TRIANGLE_FAN ) ) { return false; }
		if( Prepass_Class( draw.material ) != Prepassclass ) { return false; }
		Texture = null_handle;
		if( Prepassclass == 2 ) {
			// the diffuse texture of the colour pass, the same choice as Bind_Material() makes
			auto const &material { m_materials.material( draw.material ) };
			auto const &textures {
				material.texture_variants.empty() ?
					material.textures :
					material.GetTextures( TSubModel::VariantId( draw.submodel ) ) };
			if( textures[ 0 ] == null_handle ) { return false; }
			Texture = textures[ 0 ];
		}
		return true;
	};
	// the walk goes on with its own model block: what a recorded draw leaves in it (the emission of a lit lamp) would light the
	// tracks and the shapes the walk draws next
	auto const walkubs { model_ubs };
	auto drawn { false };
	// the draws without the alpha test first, then the ones with it, one program each
	for( auto prepassclass = 1; prepassclass <= 2; ++prepassclass ) {
		auto &program { prepassclass == 1 ? *m_prepass_shader : *m_prepass_alpha_shader };
		auto *multidrawprogram {
			m_indirectmultidraw ?
				( prepassclass == 1 ? m_prepass_multidraw_shader.get() : m_prepass_alpha_multidraw_shader.get() ) :
				nullptr };
		auto bound { false };
		for( std::size_t index = 0; index < count; ) {
			texture_handle texture;
			if( false == prepass_texture( index, prepassclass, texture ) ) {
				++index;
				continue;
			}
			if( prepassclass == 2 ) {
				m_textures.bind( 0, texture );
			}
			auto const &draw { m_indirectdraws[ index ] };
			if( multidrawprogram != nullptr ) {
				// the draws which follow in the same bank and of the same primitive, with the same texture for the alpha test.
				// the material doesn't matter to the depth
				auto last { index + 1 };
				texture_handle othertexture;
				while( ( last < count )
					&& ( m_indirectdraws[ last ].group == draw.group )
					&& ( m_indirectdraws[ last ].type == draw.type )
					&& ( m_indirectcommands[ last ].indexed == m_indirectcommands[ index ].indexed )
					&& ( prepass_texture( last, prepassclass, othertexture ) )
					&& ( othertexture == texture ) ) {
					++last;
				}
				if( false == bound ) {
					multidrawprogram->bind();
					bound = true;
				}
				auto const drawcount { static_cast<std::uint32_t>( last - index ) };
				Bind_Indirect_Blocks( index, drawcount );
				m_geometry.draw_indirect_multi( draw.geometry, m_indirectcull->draw_command_offset( Set, index ), drawcount, sizeof( gl::instance_draw_command ) );
				counts.draws += drawcount;
				++counts.calls;
				drawn = true;
				index = last;
				continue;
			}
			if( false == bound ) {
				program.bind();
				bound = true;
			}
			model_ubs = draw.ubs;
			model_ubo->update( model_ubs );
			m_geometry.draw_indirect( draw.geometry, m_indirectcull->draw_command_offset( Set, index ) );
			++counts.draws;
			++counts.calls;
			drawn = true;
			++index;
		}
	}
	model_ubs = walkubs;
	if( drawn ) {
		model_ubo->update( model_ubs );
	}
	return counts;
}

opengl46_renderer::indirect_counts opengl46_renderer::Render_Instances_Indirect_Colour( opengl46_instance_cull::command_set const Set )
{
	// as in Render_Instances_Indirect_Depth(): the seed the draws were recorded with
	TSubModel::iVariantSeed = 0;
	indirect_counts counts;
	auto const count { m_indirectdraws.size() };
	for( std::size_t index = 0; index < count; ) {
		auto const &draw { m_indirectdraws[ index ] };
		auto *multidrawprogram { m_indirectmultidraw ? Multidraw_Program( Indirect_Program( draw.material ) ) : nullptr };
		if( draw.blending ) { setup_drawing( true ); }
		if( multidrawprogram != nullptr ) {
			// the draws which follow with the same state, in one call
			auto last { index + 1 };
			while( ( last < count ) && Indirect_Mergeable( index, last ) ) {
				++last;
			}
			m_bindmultidraw = true;
			Bind_Material( draw.material, draw.submodel );
			m_bindmultidraw = false;
			auto const drawcount { static_cast<std::uint32_t>( last - index ) };
			Bind_Indirect_Blocks( index, drawcount );
			m_geometry.draw_indirect_multi( draw.geometry, m_indirectcull->draw_command_offset( Set, index ), drawcount, sizeof( gl::instance_draw_command ) );
			counts.draws += drawcount;
			index = last;
		}
		else {
			Bind_Material( draw.material, draw.submodel );
			model_ubs = draw.ubs;
			model_ubo->update( model_ubs );
			m_geometry.draw_indirect( draw.geometry, m_indirectcull->draw_command_offset( Set, index ) );
			++counts.draws;
			++index;
		}
		++counts.calls;
		if( draw.blending ) { setup_drawing( false ); }
	}
	if( m_indirectmultidraw && ( count > 0 ) ) {
		// the model block the draws made one by one leave behind
		model_ubs = m_indirectdraws.back().ubs;
		model_ubo->update( model_ubs );
	}
	model_ubs.emission = 0.0f;
	return counts;
}

opengl46_renderer::indirect_counts opengl46_renderer::Render_Instances_Indirect_Shadow( opengl46_instance_cull::command_set const Set )
{
	// the walk goes on with its own model block, as after Render_Instances_Indirect_Depth()
	auto const walkubs { model_ubs };
	indirect_counts counts;
	auto const count { m_indirectdraws.size() };
	for( std::size_t index = 0; index < count; ) {
		auto const &draw { m_indirectdraws[ index ] };
		auto const texture { Indirect_Shadow_Texture( draw.material ) };
		auto *multidrawprogram {
			m_indirectmultidraw ?
				( m_cubeshadows ?
					m_localshadows.multidraw_program( texture != null_handle ) :
					( texture != null_handle ? m_alpha_shadow_multidraw_shader.get() : m_shadow_multidraw_shader.get() ) ) :
				nullptr };
		if( multidrawprogram != nullptr ) {
			// the draws which follow in the same bank and of the same primitive, with the same texture for the alpha test
			auto last { index + 1 };
			while( ( last < count )
				&& ( m_indirectdraws[ last ].group == draw.group )
				&& ( m_indirectdraws[ last ].type == draw.type )
				&& ( m_indirectcommands[ last ].indexed == m_indirectcommands[ index ].indexed )
				&& ( Indirect_Shadow_Texture( m_indirectdraws[ last ].material ) == texture ) ) {
				++last;
			}
			if( texture != null_handle ) {
				m_textures.bind( 0, texture );
			}
			multidrawprogram->bind();
			auto const drawcount { static_cast<std::uint32_t>( last - index ) };
			Bind_Indirect_Blocks( index, drawcount );
			m_geometry.draw_indirect_multi( draw.geometry, m_indirectcull->draw_command_offset( Set, index ), drawcount, sizeof( gl::instance_draw_command ) );
			counts.draws += drawcount;
			index = last;
		}
		else {
			Bind_Material_Shadow( draw.material );
			model_ubs = draw.ubs;
			model_ubo->update( model_ubs );
			m_geometry.draw_indirect( draw.geometry, m_indirectcull->draw_command_offset( Set, index ) );
			++counts.draws;
			++index;
		}
		++counts.calls;
	}
	model_ubs = walkubs;
	if( count > 0 ) {
		model_ubo->update( model_ubs );
	}
	return counts;
}

void opengl46_renderer::Upload_Indirect_Blocks()
{
	m_indirectblocks.clear();
	for( auto const &draw : m_indirectdraws ) {
		m_indirectblocks.push_back( draw.ubs );
	}
	m_indirectblockoffset = m_drawblockstream->push( m_indirectblocks.data(), static_cast<GLsizeiptr>( m_indirectblocks.size() * sizeof( gl::model_ubs ) ) );
	m_indirectblockbuffer = m_drawblockstream->buffer();
	m_indirectblockgeneration = m_drawblockstream->generation();
}

void opengl46_renderer::Bind_Indirect_Blocks( std::size_t const First, std::size_t const Count )
{
	if( m_drawblockstream->generation() != m_indirectblockgeneration ) {
		// the buffer the blocks went to was replaced since (it shouldn't, the draws are recorded once a frame)
		Upload_Indirect_Blocks();
	}
	GLuint const buffer { m_indirectblockbuffer };
	GLintptr const offset { m_indirectblockoffset + static_cast<GLintptr>( First * sizeof( gl::model_ubs ) ) };
	GLsizeiptr const size { static_cast<GLsizeiptr>( Count * sizeof( gl::model_ubs ) ) };
	// the plural form leaves the general GL_SHADER_STORAGE_BUFFER binding alone, as gl::ssbo does
	::glBindBuffersRange( GL_SHADER_STORAGE_BUFFER, gl::MODEL_DRAW_SSBO_BINDING, 1, &buffer, &offset, &size );
}

gl::program const *opengl46_renderer::Indirect_Program( material_handle Material ) const
{
	// the same choice as Bind_Material() makes: what can't be drawn is drawn with the material of failures, if that one can
	if( Material == null_handle ) { Material = m_invalid_material; }
	if( Material == null_handle ) { return nullptr; }
	auto const *material { &m_materials.material( Material ) };
	if( false == material->is_good ) {
		if( Material == m_invalid_material ) { return nullptr; }
		material = &m_materials.material( m_invalid_material );
		if( false == material->is_good ) { return nullptr; }
	}
	return static_cast<gl::program const *>( material->shader.get() );
}

gl::program *opengl46_renderer::Multidraw_Program( gl::program const *Program )
{
	if( ( Program == nullptr ) || ( m_vertex_multidraw_shader == nullptr ) || ( m_drawblockstream == nullptr ) ) { return nullptr; }
	auto const lookup { m_multidrawprograms.find( Program ) };
	if( lookup != m_multidrawprograms.end() ) { return lookup->second.get(); }

	// the fragment shader of the program by its name, mat_<name>.frag (Fetch_Shader())
	std::unique_ptr<gl::program> variant;
	for( auto const &shader : m_shaders ) {
		if( shader.second.get() != Program ) { continue; }
		try {
			gl::shader fragment( "mat_" + shader.first + ".frag", "#define MODEL_DRAWS 1\n" );
			variant = std::make_unique<gl::program>( std::vector<std::reference_wrapper<const gl::shader>>{ fragment, *m_vertex_multidraw_shader } );
		}
		catch( gl::shader_exception const &e ) {
			ErrorLog( "material " + shader.first + " drawn without multi draws: " + std::string( e.what() ) );
		}
		break;
	}
	auto *result { variant.get() };
	m_multidrawprograms.emplace( Program, std::move( variant ) );
	return result;
}

// depth of the whole opaque scene of the colour pass, before its colour. the same walk as the colour pass makes (the cab drawn
// early, then the region), so the same things are seen at the same levels of detail; the programs of the depth prepass draw
// them (Bind_Material(), draw()), with the depth a little farther than the surface like the prepass of the instances, so the
// colour pass, which tests and writes its depth as usual, passes where it's the same surface. the gpu culling of the
// instances runs here, against the same depth as without this pass, and the colour pass draws what it recorded
void opengl46_renderer::Render_Scene_Depth( glm::mat4 const &Future, bool const Contactonly )
{
	auto const cpustart { std::chrono::steady_clock::now() };
	m_scenedepthtimer.begin();
	auto const colourstats { m_renderpass.draw_stats };
	m_renderpass.draw_stats = {};

	m_scenedepthpass = true;
	m_scenedepthskip = true;
	setup_drawing( false );
	::glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE );
	::glEnable( GL_POLYGON_OFFSET_FILL );
	::glPolygonOffset( -1.f, -1.f );

	m_scenereplayed = Global.gfx_depthprepass_replay;
	if( m_scenereplayed ) {
		// the walk of the colour pass, recorded: the same steps as in Render_pass(), the shadow tone of the cab lamp included
		m_scenerecording = true;
		m_scenedraws.clear();
		m_scenechunks.clear();
		m_scenegeometry.clear();
		m_sceneheaders.clear();
		std::memcpy( m_sceneheaders.emplace_back().data(), &light_data, std::tuple_size_v<light_header> );
		m_sceneflushed = 0;
		m_scenematerial = null_handle;
		m_scenesubmodel = nullptr;
		m_sceneseed = TSubModel::iVariantSeed;
		m_scenevariant = 0;
		m_sceneprepassdraws = 0;
		m_sceneprepasscalls = 0;
		m_scenebeyondreach = 0;
		// the contact shadows alone need the depth only within their reach: a ray of a pixel there runs up to their length
		// towards the light, and a surface counts as in its way up to 0.3 m and 4 pixels behind it (see contact_shadow()).
		// past that the colour pass draws with no depth before it, as without the depth pass. the cursor of the editor reads
		// the depth of the ground anywhere
		m_scenedepthreach = ( Contactonly && ( false == EditorModeFlag ) );
		if( m_scenedepthreach ) {
			auto const reach { Global.gfx_contactshadows_range + Global.gfx_contactshadows_length };
			auto const scale { scene_ubs.projection[ 1 ][ 1 ] };
			auto const height { ( m_current_viewport != nullptr ? m_current_viewport->height : Global.gfx_framebuffer_height ) };
			auto const pixel { ( ( scale > 0.f ) && ( height > 0 ) ) ? 2.f * reach / ( scale * static_cast<float>( height ) ) : reach };
			m_scenedepthlimit = reach + 0.3f + 4.f * pixel + 1.f;
		}

		if( ( false == FreeFlyModeFlag ) && ( Global.Overcast <= 1.0f ) && ( Global.render_cab ) ) {
			model_ubs.future = m_cabfuture;
			auto const *vehicle { simulation::Train->Dynamic() };
			if( ( vehicle->InteriorLightLevel > 0.f ) && ( false == m_cablamplit ) ) {
				setup_shadow_color( glm::min( colors::white, m_shadowcolor + glm::vec4( vehicle->InteriorLight * vehicle->InteriorLightLevel, 1.f ) ) );
			}
			Render_cab( vehicle, vehicle->InteriorLightLevel, false );
		}
		setup_shadow_color( m_shadowcolor );
		model_ubs.future = Future;
		Render( simulation::Region );
		// the depth of what's left
		Render_Scene_Prepass();
		m_scenerecording = false;
		m_scenerecorded = true;
		m_scenerecordedparts = static_cast<std::uint32_t>( m_scenechunks.size() );
	}
	else {
		// the conditions of the cab drawn early by the colour pass
		if( ( false == FreeFlyModeFlag ) && ( Global.Overcast <= 1.0f ) && ( Global.render_cab ) ) {
			model_ubs.future = m_cabfuture;
			auto const *vehicle { simulation::Train->Dynamic() };
			Render_cab( vehicle, vehicle->InteriorLightLevel, false );
		}
		model_ubs.future = Future;
		Render( simulation::Region );
	}

	::glDisable( GL_POLYGON_OFFSET_FILL );
	::glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	m_scenedepthpass = false;
	m_scenedepthskip = true;
	m_scenedepthdrawn = true;

	m_scenedepthstats = m_renderpass.draw_stats;
	m_renderpass.draw_stats = colourstats;
	if( m_scenereplayed ) {
		// the walk was the one of the colour pass, which counts it
		m_renderpass.draw_stats += m_scenedepthstats;
	}
	m_scenedepthtimer.end();
	m_scenedepthcputime = std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - cpustart ).count();
}

void opengl46_renderer::Record_Scene_Draw( gfx::geometry_handle const *First, std::size_t const Count, bool const Batch )
{
	auto const index { static_cast<std::uint32_t>( m_scenedraws.size() ) };
	auto &draw { m_scenedraws.emplace_back() };
	draw.ubs = model_ubs;
	draw.material = m_scenematerial;
	draw.submodel = m_scenesubmodel;
	draw.seed = m_sceneseed;
	draw.variant = m_scenevariant;
	draw.chunkfirst = static_cast<std::uint32_t>( m_scenechunks.size() );
	draw.chunkcount = static_cast<std::uint32_t>( Count );
	// a batch is drawn without instances, as Draw_Geometry() draws it
	draw.instancecount = ( Batch ? 0 : static_cast<std::uint32_t>( m_current_instance_count ) );
	draw.instancebase = ( draw.instancecount > 0 ? m_current_instance_base : 0 );
	draw.instancebuffer = ( draw.instancecount > 0 ? m_instancestream->buffer() : 0 );
	draw.header = static_cast<std::uint32_t>( m_sceneheaders.size() - 1 );
	draw.blending = m_blendingenabled;
	draw.batch = Batch;
	// what the depth pass leaves out, as Bind_Material() and draw() do without the record (V41): the blended draws, those of
	// negative opacity, of materials the depth programs can't stand for, and the ones with the alpha test without a texture
	draw.prepassclass = 0;
	draw.prepasstexture = null_handle;
	if( ( false == m_blendingenabled ) && ( model_ubs.opacity >= 0.f ) ) {
		draw.prepassclass = Prepass_Class( m_scenematerial );
		if( draw.prepassclass == 2 ) {
			// the diffuse texture of the colour pass, the same choice as Bind_Material() makes
			auto const &material { m_materials.material( m_scenematerial ) };
			auto const &textures { material.texture_variants.empty() ? material.textures : material.GetTextures( m_scenevariant ) };
			draw.prepasstexture = textures[ 0 ];
			if( draw.prepasstexture == null_handle ) {
				draw.prepassclass = 0;
			}
		}
	}
	// the sphere of a chunk of a draw which isn't instanced, in the view: centre and the radius scaled with the model
	glm::mat3 const rotationscale { draw.ubs.modelview };
	auto const scale { std::sqrt( std::max( { glm::length2( rotationscale[ 0 ] ), glm::length2( rotationscale[ 1 ] ), glm::length2( rotationscale[ 2 ] ) } ) ) };
	for( std::size_t chunk = 0; chunk < Count; ++chunk ) {
		auto const &geometry { First[ chunk ] };
		auto depth { draw.prepassclass != 0 };
		if( depth ) {
			// the program of the prepass doesn't size points, and lines aren't surfaces
			auto const type { m_geometry.type( geometry ) };
			depth = ( ( type == GL_TRIANGLES ) || ( type == GL_TRIANGLE_STRIP ) || ( type == GL_TRIANGLE_FAN ) );
		}
		if( depth && m_scenedepthreach && ( draw.instancecount == 0 ) ) {
			auto const &sphere { Chunk_Sphere( geometry ) };
			if( sphere.w >= 0.f ) {
				auto const centre { glm::vec3{ draw.ubs.modelview * glm::vec4{ glm::vec3{ sphere }, 1.f } } };
				if( glm::length( centre ) - sphere.w * scale > m_scenedepthlimit ) {
					depth = false;
					++m_scenebeyondreach;
				}
			}
		}
		m_scenechunks.push_back( { geometry, index, depth } );
		m_scenegeometry.push_back( geometry );
	}
}

void opengl46_renderer::Render_Scene_Prepass()
{
	auto const end { m_scenechunks.size() };
	if( m_sceneflushed >= end ) { return; }

	// the chunks the depth is drawn for, in runs of the same program, texture of the alpha test, bank, primitive and instance
	// buffer. the depth doesn't depend on the order of the draws; within a bank the chunks keep the order of the walk
	m_sceneprepassitems.clear();
	for( auto chunkindex { m_sceneflushed }; chunkindex < end; ++chunkindex ) {
		auto const &chunk { m_scenechunks[ chunkindex ] };
		if( false == chunk.depth ) { continue; }
		auto const &draw { m_scenedraws[ chunk.draw ] };
		gfx::indirect_source source;
		if( false == m_geometry.indirect_source( chunk.geometry, source ) ) { continue; }
		auto &item { m_sceneprepassitems.emplace_back() };
		item.prepassclass = draw.prepassclass;
		item.texture = ( draw.prepassclass == 2 ? draw.prepasstexture : null_handle );
		// the buffers the geometry lies in, shared by many banks
		item.bank = m_geometry.draw_group( chunk.geometry );
		item.type = source.type;
		item.indexed = ( source.indexed ? 1u : 0u );
		item.instancebuffer = draw.instancebuffer;
		item.chunk = static_cast<std::uint32_t>( chunkindex );
		// count, instances, first, base vertex (the base instance of an array draw), base instance
		auto const instances { std::max<std::uint32_t>( draw.instancecount, 1 ) };
		item.command = {
			source.count,
			instances,
			source.first,
			( source.indexed ? static_cast<std::uint32_t>( source.basevertex ) : draw.instancebase ),
			( source.indexed ? draw.instancebase : 0u ),
			0, item.indexed, 0 };
	}
	m_sceneflushed = end;
	if( m_sceneprepassitems.empty() ) { return; }
	std::sort(
		std::begin( m_sceneprepassitems ), std::end( m_sceneprepassitems ),
		[]( scene_prepass_item const &Left, scene_prepass_item const &Right ) { return Left.tie() < Right.tie(); } );

	auto const multidraw { ( m_drawblockstream != nullptr ) && ( m_scenecommandstream != nullptr ) };
	GLuint const frameinstances { m_instancestream->buffer() };
	GLuint boundinstances { frameinstances };
	auto const bind_instances = [ & ]( GLuint const Buffer ) {
		// a draw which isn't instanced reads the identity at the start of any of the buffers
		if( ( Buffer == 0 ) || ( Buffer == boundinstances ) ) { return; }
		::glBindBuffersBase( GL_SHADER_STORAGE_BUFFER, gl::INSTANCE_SSBO_BINDING, 1, &Buffer );
		boundinstances = Buffer;
	};
	auto const count { m_sceneprepassitems.size() };
	auto const samerun = []( scene_prepass_item const &Left, scene_prepass_item const &Right ) {
		return ( Left.prepassclass == Right.prepassclass )
			&& ( Left.texture == Right.texture )
			&& ( Left.bank == Right.bank )
			&& ( Left.type == Right.type )
			&& ( Left.indexed == Right.indexed )
			&& ( Left.instancebuffer == Right.instancebuffer ); };

	if( multidraw ) {
		// the model blocks and the commands of all of them go to the buffers of the frame at once, a run is one call
		m_sceneprepassblocks.clear();
		m_sceneprepasscommands.clear();
		for( auto const &item : m_sceneprepassitems ) {
			m_sceneprepassblocks.push_back( m_scenedraws[ m_scenechunks[ item.chunk ].draw ].ubs );
			m_sceneprepasscommands.push_back( item.command );
		}
		auto const blockoffset { m_drawblockstream->push( m_sceneprepassblocks.data(), static_cast<GLsizeiptr>( count * sizeof( gl::model_ubs ) ) ) };
		GLuint const blockbuffer { m_drawblockstream->buffer() };
		auto const commandoffset { m_scenecommandstream->push( m_sceneprepasscommands.data(), static_cast<GLsizeiptr>( count * sizeof( gl::instance_draw_command ) ) ) };
		::glBindBuffer( GL_DRAW_INDIRECT_BUFFER, m_scenecommandstream->buffer() );
		auto boundclass { 0 };
		for( std::size_t first = 0; first < count; ) {
			auto const &item { m_sceneprepassitems[ first ] };
			auto last { first + 1 };
			while( ( last < count ) && samerun( item, m_sceneprepassitems[ last ] ) ) {
				++last;
			}
			auto const drawcount { last - first };
			auto const run { first };
			first = last;
			if( ( item.prepassclass == 2 ) && ( false == m_textures.mark_as_used( item.texture ).bind( 0 ) ) ) {
				// a texture which isn't there (yet) would leave the one bound before to the alpha test
				continue;
			}
			if( item.prepassclass != boundclass ) {
				( item.prepassclass == 2 ? m_prepass_alpha_multidraw_shader : m_prepass_multidraw_shader )->bind();
				boundclass = item.prepassclass;
			}
			bind_instances( item.instancebuffer );
			GLintptr const offset { blockoffset + static_cast<GLintptr>( run * sizeof( gl::model_ubs ) ) };
			GLsizeiptr const size { static_cast<GLsizeiptr>( drawcount * sizeof( gl::model_ubs ) ) };
			::glBindBuffersRange( GL_SHADER_STORAGE_BUFFER, gl::MODEL_DRAW_SSBO_BINDING, 1, &blockbuffer, &offset, &size );
			m_geometry.draw_indirect_multi(
				m_scenechunks[ item.chunk ].geometry,
				commandoffset + static_cast<std::intptr_t>( run * sizeof( gl::instance_draw_command ) ),
				static_cast<std::uint32_t>( drawcount ),
				sizeof( gl::instance_draw_command ) );
			m_sceneprepassdraws += static_cast<std::uint32_t>( drawcount );
			++m_sceneprepasscalls;
		}
		::glBindBuffer( GL_DRAW_INDIRECT_BUFFER, 0 );
	}
	else {
		// one by one, with the model block of each draw, as without the record
		auto const walkubs { model_ubs };
		auto boundclass { 0 };
		auto drawn { false };
		for( std::size_t first = 0; first < count; ) {
			auto const &item { m_sceneprepassitems[ first ] };
			auto last { first + 1 };
			while( ( last < count ) && samerun( item, m_sceneprepassitems[ last ] ) ) {
				++last;
			}
			auto const run { first };
			first = last;
			if( ( item.prepassclass == 2 ) && ( false == m_textures.mark_as_used( item.texture ).bind( 0 ) ) ) {
				continue;
			}
			if( item.prepassclass != boundclass ) {
				( item.prepassclass == 2 ? m_prepass_alpha_shader : m_prepass_shader )->bind();
				boundclass = item.prepassclass;
			}
			bind_instances( item.instancebuffer );
			for( auto index { run }; index < last; ++index ) {
				auto const &chunk { m_scenechunks[ m_sceneprepassitems[ index ].chunk ] };
				auto const &draw { m_scenedraws[ chunk.draw ] };
				model_ubs = draw.ubs;
				model_ubo->update( model_ubs );
				if( draw.instancecount > 0 ) {
					m_geometry.draw_instanced( chunk.geometry, draw.instancecount, draw.instancebase );
				}
				else {
					m_geometry.draw( chunk.geometry );
				}
				++m_sceneprepassdraws;
				++m_sceneprepasscalls;
				drawn = true;
			}
		}
		// the walk goes on with its own state
		model_ubs = walkubs;
		if( drawn ) {
			model_ubo->update( model_ubs );
		}
	}
	if( boundinstances != frameinstances ) {
		Bind_Instance_Stream();
	}
}

void opengl46_renderer::Render_Scene_Colour()
{
	// what the walk left behind, which the passes after this one go on with
	auto const walkubs { model_ubs };
	auto const walkseed { TSubModel::iVariantSeed };

	// the light buffer has the header the walk began with (it uploaded none while recording)
	std::uint32_t header { 0 };
	auto blending { false };
	auto bound { false };
	material_handle boundmaterial { null_handle };
	std::uint32_t boundvariant { 0 };
	GLuint const frameinstances { m_instancestream->buffer() };
	GLuint boundinstances { frameinstances };
	for( auto const &draw : m_scenedraws ) {
		if( draw.instances ) {
			// the instances culled in the walk, where the colour pass would draw them
			if( blending ) {
				setup_drawing( false );
				blending = false;
			}
			if( draw.header != header ) {
				light_ssbo->update( m_sceneheaders[ draw.header ].data(), 0, std::tuple_size_v<light_header> );
				header = draw.header;
			}
			if( m_indirectcolourpending ) {
				Render_Instances_Indirect_Pending();
			}
			// they bind their own programs, textures and the instance buffer of the frame
			bound = false;
			boundinstances = m_instancestream->buffer();
			continue;
		}
		if( draw.header != header ) {
			light_ssbo->update( m_sceneheaders[ draw.header ].data(), 0, std::tuple_size_v<light_header> );
			header = draw.header;
		}
		if( draw.blending != blending ) {
			// the transparency hack of submodels faded out
			setup_drawing( draw.blending );
			blending = draw.blending;
		}
		if( ( false == bound ) || ( draw.material != boundmaterial ) || ( draw.variant != boundvariant ) ) {
			// the textures of the variant follow from the seed of the vehicle and the submodel
			TSubModel::iVariantSeed = draw.seed;
			Bind_Material( draw.material, draw.submodel );
			bound = true;
			boundmaterial = draw.material;
			boundvariant = draw.variant;
		}
		model_ubs = draw.ubs;
		model_ubo->update( model_ubs );
		auto const *chunks { m_scenegeometry.data() + draw.chunkfirst };
		if( draw.instancecount > 0 ) {
			if( draw.instancebuffer != boundinstances ) {
				::glBindBuffersBase( GL_SHADER_STORAGE_BUFFER, gl::INSTANCE_SSBO_BINDING, 1, &draw.instancebuffer );
				boundinstances = draw.instancebuffer;
			}
			m_geometry.draw_instanced( chunks[ 0 ], draw.instancecount, draw.instancebase );
		}
		else if( draw.batch ) {
			m_geometry.draw_batch( chunks, chunks + draw.chunkcount );
		}
		else {
			m_geometry.draw( chunks[ 0 ] );
		}
	}
	if( blending ) {
		setup_drawing( false );
	}
	if( header + 1 != m_sceneheaders.size() ) {
		// the header the walk ended with
		light_ssbo->update( m_sceneheaders.back().data(), 0, std::tuple_size_v<light_header> );
	}
	if( boundinstances != m_instancestream->buffer() ) {
		Bind_Instance_Stream();
	}
	model_ubs = walkubs;
	TSubModel::iVariantSeed = walkseed;
}

void opengl46_renderer::Render_Instances_Indirect_Pending()
{
	m_indirectcolourpending = false;
	indirect_counts colourcounts;
	if( false == m_indirectdraws.empty() ) {
		auto const cpustart { std::chrono::steady_clock::now() };
		m_indirectcull = &m_instancecull;
		m_instancecull.bind_draw();
		colourcounts = Render_Instances_Indirect_Colour( m_indirectcolourset );
		::glBindBuffer( GL_DRAW_INDIRECT_BUFFER, 0 );
		Bind_Instance_Stream();
		m_cullparts.colour += std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - cpustart ).count();
		m_instancecull.draw_result(
			colourcounts.draws,
			m_instancecull.prepass_draws(),
			colourcounts.calls,
			m_instancecull.prepass_calls(),
			m_instancecull.cpu_time() + std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - cpustart ).count() );
	}
	m_renderpass.draw_stats.instanced_drawcalls += static_cast<int>( colourcounts.calls );
	m_renderpass.draw_stats.instances += static_cast<int>( m_instancecull.last_count() );
	m_renderpass.draw_stats.models += static_cast<int>( m_instancecull.last_count() );
}

glm::vec4 const &opengl46_renderer::Chunk_Sphere( gfx::geometry_handle const &Geometry )
{
	auto const key { static_cast<std::uint64_t>( Geometry ) };
	if( auto const lookup { m_chunkspheres.find( key ) }; lookup != m_chunkspheres.end() ) {
		return lookup->second;
	}
	glm::vec4 sphere { 0.f, 0.f, 0.f, -1.f };
	auto const &vertices { m_geometry.vertices( Geometry ) };
	if( false == vertices.empty() ) {
		glm::vec3 low { std::numeric_limits<float>::max() };
		glm::vec3 high { std::numeric_limits<float>::lowest() };
		for( auto const &vertex : vertices ) {
			low = glm::min( low, vertex.position );
			high = glm::max( high, vertex.position );
		}
		glm::vec3 const centre { ( low + high ) * 0.5f };
		auto radius { 0.f };
		for( auto const &vertex : vertices ) {
			radius = std::max( radius, glm::length2( vertex.position - centre ) );
		}
		sphere = glm::vec4{ centre, std::sqrt( radius ) };
	}
	return m_chunkspheres.emplace( key, sphere ).first->second;
}

bool opengl46_renderer::Scene_Depth_Draws( gfx::geometry_handle const &Geometry ) const
{
	if( m_scenedepthskip ) { return false; }
	// the program of the prepass doesn't size points, and lines aren't surfaces
	auto const type { m_geometry.type( Geometry ) };
	return ( ( type == GL_TRIANGLES ) || ( type == GL_TRIANGLE_STRIP ) || ( type == GL_TRIANGLE_FAN ) );
}

// the contact shadows read the depth of the scene drawn before the colour, which the main view has, and work out the distance
// of a surface from its depth the way a perspective projection with the far plane at infinity gives it (depth = near / w):
// not in the orthographic view of the editor
bool opengl46_renderer::Contact_Shadows_Wanted( viewport_config const &Viewport ) const
{
	if( ( Global.gfx_contactshadows <= 0 ) || ( false == Viewport.main ) || Global.bWireFrame || Global.gfx_skippipeline || ( Viewport.msaa_fb == nullptr ) ) {
		return false;
	}
	auto const &projection { scene_ubs.projection };
	return ( projection[ 2 ][ 3 ] != 0.f ) && ( projection[ 2 ][ 2 ] == 0.f ) && ( projection[ 3 ][ 2 ] > 0.f );
}

void opengl46_renderer::Setup_Contact_Shadows( viewport_config &Viewport )
{
	if( false == Resolved_Depth_Target( Viewport ) ) { return; }
	// making the target binds it
	Viewport.msaa_fb->bind();

	m_contactshadowstimer.begin();
	// the depth is resolved apart from the buffer the colour pass draws to, which it tests and writes. the scissor test would limit the blit
	auto const scissor { ::glIsEnabled( GL_SCISSOR_TEST ) == GL_TRUE };
	if( scissor ) { ::glDisable( GL_SCISSOR_TEST ); }
	::glBlitNamedFramebuffer(
		*Viewport.msaa_fb, *Viewport.clusters_depth_fb,
		0, 0, Viewport.width, Viewport.height, 0, 0, Viewport.width, Viewport.height,
		GL_DEPTH_BUFFER_BIT, GL_NEAREST );
	if( scissor ) { ::glEnable( GL_SCISSOR_TEST ); }
	::glBindTextureUnit( gl::SCENE_DEPTH_TEX, Viewport.clusters_depth_tex->id );
	m_contactshadowstimer.end();

	// steps of the ray towards the sun, of the rays towards lamps and headlights, and how many of those a pixel traces
	struct contact_level { float sunsteps; float lampsteps; float lamps; };
	static contact_level const levels[] { { 8.f, 0.f, 0.f }, { 12.f, 8.f, 2.f }, { 16.f, 12.f, 4.f } };
	auto const &level { levels[ std::clamp( Global.gfx_contactshadows, 1, 3 ) - 1 ] };
	scene_ubs.contact_shadow_params = glm::vec4{ Global.gfx_contactshadows_length, Global.gfx_contactshadows_range, level.sunsteps, level.lampsteps };
	scene_ubs.contact_shadow_lamps = glm::vec4{ level.lamps, 0.f, 0.f, 0.f };
	scene_ubo->update( scene_ubs );
	m_contactshadows = true;
}

// Renders the per-track sleeper instances (TTrack::m_sleeper_local_transforms) using the
// existing GPU-instanced submodel pipeline. The track owns a vector of pre-baked
// local-space matrices; we compose each with `view * translate(track_origin - camera)`
// to get a camera-space modelview, then issue glDrawElementsInstancedBaseVertexBaseInstance
// calls -- one per LOD band of the sleepers.
//
// Skipped entirely when:
//   - Global.SleeperDistance == 0 (sleeper rendering globally disabled)
//   - the track has no sleepermodel
//   - the track is farther than Global.SleeperDistance meters from the camera
// releases gl textures used only by vehicles farther than gfx.textures.releasedistance times the draw range
void opengl46_renderer::Update_Texture_Release()
{
	if (m_colorpass.draw_range <= 0.f)
	{
		return;
	}
	auto const range{static_cast<double>(Global.gfx_texture_releasedistance) * m_colorpass.draw_range};
	auto const camera{m_colorpass.pass_camera.position()};

	m_textures.begin_release_scan();
	for (auto *vehicle : simulation::Vehicles.sequence())
	{
		if (vehicle == nullptr)
		{
			continue;
		}
		bool const isnear{glm::length(vehicle->vPosition - camera) - vehicle->radius() <= range};
		auto const mark_model = [&](TModel3d *Model) {
			if (Model == nullptr)
			{
				return;
			}
			for (auto const texture : Model_Textures(Model))
			{
				m_textures.mark_as_needed(texture, isnear);
			}
		};
		mark_model(vehicle->mdModel);
		mark_model(vehicle->mdLowPolyInt);
		mark_model(vehicle->mdLoad);
		for (auto *attachment : vehicle->mdAttachments)
		{
			mark_model(attachment);
		}
		for (auto const skin : vehicle->m_materialdata.replacable_skins)
		{
			if (skin <= 0)
			{
				continue;
			}
			auto const &material{m_materials.material(skin)};
			for (auto const texture : material.textures)
			{
				m_textures.mark_as_needed(texture, isnear);
			}
			for (auto const &variant : material.texture_variants)
			{
				for (auto const texture : variant)
				{
					m_textures.mark_as_needed(texture, isnear);
				}
			}
		}
	}
	m_textures.release_unneeded();
}

// with gfx.textures.streaming the data of a texture is read when it's going to be drawn, not with the scenery: the textures of the
// whole scenery don't fit in memory. the cells of the region within the draw range and a margin around the camera are gone through
// a few in a frame, and the textures of what they hold which will be seen from there are read ahead, as are the ones of the vehicles
// nearby. what is drawn without that waits for its data to be read
void opengl46_renderer::Update_Texture_Prefetch()
{
	if (m_colorpass.draw_range <= 0.f)
	{
		return;
	}
	double const margin{500.0}; // m: the way the data has to arrive
	std::size_t const cellsperframe{32};
	auto const camera{m_colorpass.pass_camera.position()};
	auto const range{static_cast<double>(m_colorpass.draw_range) * Global.fDistanceFactor + margin};

	auto const prefetch_material = [this, &camera](material_handle const Material) {
		if (Material <= 0)
		{
			return;
		}
		auto const &material{m_materials.material(Material)};
		for (auto const texture : material.textures)
		{
			m_textures.prefetch(texture, camera);
		}
		for (auto const &variant : material.texture_variants)
		{
			for (auto const texture : variant)
			{
				m_textures.prefetch(texture, camera);
			}
		}
	};
	auto const prefetch_model = [this, &camera](TModel3d *Model) {
		if (Model == nullptr)
		{
			return;
		}
		for (auto const texture : Model_Textures(Model))
		{
			m_textures.prefetch(texture, camera);
		}
	};

	// the sections around the camera, the same way Render(scene::basic_region *) picks them
	int constexpr cellspersection{(scene::EU07_SECTIONSIZE / scene::EU07_CELLSIZE) * (scene::EU07_SECTIONSIZE / scene::EU07_CELLSIZE)};
	auto const camerax{static_cast<int>(std::floor(camera.x / scene::EU07_SECTIONSIZE + scene::EU07_REGIONSIDESECTIONCOUNT / 2))};
	auto const cameraz{static_cast<int>(std::floor(camera.z / scene::EU07_SECTIONSIZE + scene::EU07_REGIONSIDESECTIONCOUNT / 2))};
	int const segmentcount{2 * static_cast<int>(std::ceil(range / scene::EU07_SECTIONSIZE))};
	int const side{segmentcount + 1};
	int const originx{camerax - segmentcount / 2};
	int const originz{cameraz - segmentcount / 2};
	auto const cellcount{static_cast<std::size_t>(side) * side * cellspersection};
	auto *region{simulation::Region};

	// the first time the whole range at once: the textures around the start are read by the workers, not one by one by the first frame
	auto const steps{m_prefetchcursor == 0 ? cellcount : cellsperframe};
	for (std::size_t step = 0; step < steps; ++step, ++m_prefetchcursor)
	{
		auto const index{m_prefetchcursor % cellcount};
		auto const sectionindex{static_cast<int>(index / cellspersection)};
		auto const cellindex{index % cellspersection};
		auto const row{originz + sectionindex / side};
		auto const column{originx + sectionindex % side};
		if ((row < 0) || (row >= scene::EU07_REGIONSIDESECTIONCOUNT) || (column < 0) || (column >= scene::EU07_REGIONSIDESECTIONCOUNT))
		{
			continue;
		}
		auto *section{region->m_sections[row * scene::EU07_REGIONSIDESECTIONCOUNT + column]};
		if (section == nullptr)
		{
			continue;
		}
		if ((cellindex == 0) && (glm::length(section->m_area.center - camera) - section->m_area.radius <= range))
		{
			for (auto const &shape : section->m_shapes)
			{
				prefetch_material(shape.data().material);
			}
		}
		auto const &cell{section->m_cells[cellindex]};
		if ((false == cell.m_active) || (glm::length(cell.m_area.center - camera) - cell.m_area.radius > range))
		{
			continue;
		}
		for (auto const &shape : cell.m_shapesopaque)
		{
			prefetch_material(shape.data().material);
		}
		for (auto const &shape : cell.m_shapestranslucent)
		{
			prefetch_material(shape.data().material);
		}
		for (auto const *track : cell.m_paths)
		{
			prefetch_material(track->m_material1);
			prefetch_material(track->m_material2);
			prefetch_material(track->m_sleeper_skin);
			if (track->SwitchExtension != nullptr)
			{
				prefetch_material(track->SwitchExtension->m_material3);
			}
		}
		// the instances which will be seen from within the margin. the ones asked for once aren't gone through again until they're
		// out of reach, it's when their data could have been dropped
		auto const prefetch_instances = [&](scene::basic_cell::instance_sequence const &Instances) {
			for (auto *instance : Instances)
			{
				auto const distance{glm::length(instance->location() - camera)};
				auto const reach{std::sqrt(instance->m_rangesquaredmax * Global.fDistanceFactor) + margin};
				if ((distance > reach) || (false == instance->visible()))
				{
					if (distance > reach + margin)
					{
						instance->m_texturesprefetched = false;
					}
					continue;
				}
				if (true == instance->m_texturesprefetched)
				{
					continue;
				}
				instance->m_texturesprefetched = true;
				prefetch_model(instance->Model());
				for (auto const skin : instance->Material()->replacable_skins)
				{
					prefetch_material(skin);
				}
			}
		};
		prefetch_instances(cell.m_instancesopaque);
		prefetch_instances(cell.m_instancetranslucent);
	}

	auto const now{std::chrono::steady_clock::now()};
	if (now - m_prefetchvehicletime >= std::chrono::seconds(1))
	{
		m_prefetchvehicletime = now;
		for (auto *vehicle : simulation::Vehicles.sequence())
		{
			if ((vehicle == nullptr) || (glm::length(vehicle->vPosition - camera) - vehicle->radius() > range))
			{
				continue;
			}
			prefetch_model(vehicle->mdModel);
			prefetch_model(vehicle->mdLowPolyInt);
			prefetch_model(vehicle->mdLoad);
			for (auto *attachment : vehicle->mdAttachments)
			{
				prefetch_model(attachment);
			}
			for (auto const skin : vehicle->m_materialdata.replacable_skins)
			{
				prefetch_material(skin);
			}
		}
	}
	// the data read for what wasn't drawn after all goes: when the camera went away from where it was read for (the instances which
	// wanted it are out of reach then), or after a while (the details of models seen only from close, a turn not taken)
	if (now - m_prefetchdroptime >= std::chrono::seconds(2))
	{
		m_prefetchdroptime = now;
		m_textures.drop_read_ahead(std::chrono::minutes(5), camera, 2.0 * range);
	}
}

// textures used by materials of the sub-models of specified model, replaceable skins excluded
std::vector<texture_handle> const &opengl46_renderer::Model_Textures(TModel3d *Model)
{
	auto const lookup{m_modeltextures.find(Model)};
	if (lookup != m_modeltextures.end())
	{
		return lookup->second;
	}
	auto &textures{m_modeltextures[Model]};
	std::vector<TSubModel *> submodels;
	if (Model->GetSMRoot() != nullptr)
	{
		submodels.emplace_back(Model->GetSMRoot());
	}
	while (false == submodels.empty())
	{
		auto *submodel{submodels.back()};
		submodels.pop_back();
		if (submodel->NextGet() != nullptr)
		{
			submodels.emplace_back(submodel->NextGet());
		}
		if (submodel->ChildGet() != nullptr)
		{
			submodels.emplace_back(submodel->ChildGet());
		}
		// negative material is one of replaceable skins of the vehicle, these are handled with the vehicle
		if (submodel->GetMaterial() <= 0)
		{
			continue;
		}
		auto const &material{m_materials.material(submodel->GetMaterial())};
		for (auto const texture : material.textures)
		{
			if (texture > 0)
			{
				textures.emplace_back(texture);
			}
		}
		for (auto const &variant : material.texture_variants)
		{
			for (auto const texture : variant)
			{
				if (texture > 0)
				{
					textures.emplace_back(texture);
				}
			}
		}
	}
	std::sort(std::begin(textures), std::end(textures));
	textures.erase(std::unique(std::begin(textures), std::end(textures)), std::end(textures));
	return textures;
}

// the draws of the shared walk of the scene (gfx::scene_walk), made at once

void opengl46_renderer::walk_material(material_handle const Material, TSubModel const *Submodel, lighting_data const *Lighting)
{
	Bind_Material(Material, Submodel, Lighting);
}

void opengl46_renderer::walk_shadow_material(material_handle const Material)
{
	Bind_Material_Shadow(Material);
}

opengl_material const &opengl46_renderer::walk_material_data(material_handle const Material) const
{
	return m_materials.material(Material);
}

opengl_material const &opengl46_renderer::walk_material_data(TSubModel const *Submodel) const
{
	return Material(Submodel);
}

void opengl46_renderer::walk_program(walk_shader const Program)
{
	switch (Program)
	{
	case walk_shader::pick:
		m_pick_shader->bind();
		break;
	case walk_shader::pick_surface:
		m_pick_surface_shader->bind();
		break;
	case walk_shader::line:
		m_line_shader->bind();
		break;
	case walk_shader::billboard:
		m_billboard_shader->bind();
		break;
	case walk_shader::freespot:
		m_freespot_shader->bind();
		break;
	}
}

void opengl46_renderer::walk_texture(std::size_t const Unit, texture_handle const Texture)
{
	Bind_Texture(Unit, Texture);
}

void opengl46_renderer::walk_blending(bool const Alpha)
{
	setup_drawing(Alpha);
}

void opengl46_renderer::walk_sunlight(float const Factor)
{
	setup_sunlight_intensity(Factor);
}

void opengl46_renderer::walk_environment(TEnvironmentType const Environment)
{
	setup_environment_light(Environment);
}

void opengl46_renderer::walk_ambient_save()
{
	m_cabambient = light_data.ambient;
}

void opengl46_renderer::walk_ambient_add(glm::vec3 const &Addition)
{
	light_data.ambient += packed_vec3( Addition );
	upload_light_header();
}

void opengl46_renderer::walk_ambient_restore()
{
	light_data.ambient = m_cabambient;
}

glm::vec4 opengl46_renderer::walk_sun_ambient() const
{
	return m_sunlight.ambient;
}

void opengl46_renderer::walk_draw(gfx::geometry_handle const &Geometry)
{
	draw(Geometry);
}

void opengl46_renderer::walk_draw(std::vector<gfx::geometrybank_handle>::iterator First, std::vector<gfx::geometrybank_handle>::iterator Last)
{
	draw(First, Last);
}

std::uint32_t opengl46_renderer::walk_instances(glm::mat4 const *Modelviews, std::size_t const Count)
{
	return Upload_Instance_Modelviews(Modelviews, Count);
}

void opengl46_renderer::walk_cull_face(bool const Enabled)
{
	if (Enabled)
		::glEnable(GL_CULL_FACE);
	else
		glDisable(GL_CULL_FACE);
}

void opengl46_renderer::walk_glare(bool const Enabled)
{
	if (Enabled)
	{
		glDisable(GL_DEPTH_TEST);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	}
	else
	{
		glEnable(GL_DEPTH_TEST);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
}

void opengl46_renderer::walk_glare_draw(TSubModel *Submodel, gfx::geometry_handle const &Geometry)
{
	if (Submodel->occlusion_query) {
		glBeginConditionalRender(*Submodel->occlusion_query, GL_QUERY_WAIT);
		draw(Geometry);
		glEndConditionalRender();
	}
	else
		draw(Geometry);
}

void opengl46_renderer::walk_light_query(TSubModel *Submodel, bool const Begin)
{
	if (false == gl::vao::use_vao)
		return;
	if (Begin) {
		if (!Submodel->occlusion_query)
			Submodel->occlusion_query.emplace(gl::query::ANY_SAMPLES_PASSED);
		Submodel->occlusion_query->begin();
	}
	else
		Submodel->occlusion_query->end();
}

void opengl46_renderer::walk_region_begin()
{
	// the gpu culling runs along the first color pass of the main viewport of a frame, and the passes of the cascades of the
	// sun shadows, of the faces of the environment map and of the maps of headlights and lamps drawn for it, each with a culling
	// of its own. its result is drawn (Global.gfx_gpucull 1), or compared with the one of the cpu, which draws (2)
	opengl46_instance_cull *passcull { nullptr };
	if( ( Global.gfx_gpucull != 0 )
	 && ( m_current_viewport != nullptr )
	 && ( m_current_viewport->main ) ) {
		if( ( m_renderpass.draw_mode == rendermode::color )
		 && ( m_instancecullframe != m_framestamp ) ) {
			m_instancecullframe = m_framestamp;
			passcull = &m_instancecull;
		}
		else if( ( m_renderpass.draw_mode == rendermode::shadows )
		      && ( m_lightshadowcull != nullptr ) ) {
			// the map of a set of headlights or the scenery part of the map of a lamp, which picked its culling
			passcull = m_lightshadowcull;
			m_shadowcasters.clear();
		}
		else if( ( m_renderpass.draw_mode == rendermode::shadows )
		      && ( Global.gfx_gpucull_shadows )
		      && ( m_shadowcascade >= 0 )
		      && ( m_shadowcascade < static_cast<int>( m_shadowcull.size() ) )
		      && ( false == m_cubeshadows ) ) {
			// the cascades of the sun
			passcull = &m_shadowcull[ m_shadowcascade ];
			m_shadowcasters.clear();
		}
	}
	Begin_Pass_Cull( passcull, ( passcull == &m_instancecull ) );
}

void opengl46_renderer::walk_editor_cursor()
{
	if (m_current_viewport->main && (false == m_scenedepthdrawn))
	{
		// (with the depth pass of the whole scene it's done there, at the same stage)
		if (m_scenerecording)
		{
			// the depth of what the walk recorded so far
			Render_Scene_Prepass();
		}
		Update_Mouse_Position();
	}
}

bool opengl46_renderer::walk_instances_begin()
{
	if( m_indirectcolourpending ) {
		// the instances culled and recorded by the depth pass of the whole scene, the buckets weren't queued for the cpu
		Render_Instances_Indirect_Pending();
	}
	if( m_instancecullpass ) {
		if( m_passcull->drawing() ) {
			// the buckets of the pass weren't queued for the cpu, which leaves the loop below empty
			if( m_passcull == &m_instancecull ) {
				Render_Instances_Indirect();
			}
			else {
				Render_Instances_Indirect_Pass();
			}
		}
		else {
			// the view of the pass is in place here, the one Render_Instanced() takes
			Pass_Cull().dispatch( OpenGLMatrices.data( GL_MODELVIEW ) );
			// what the cpu drew of the sleepers and the models drawn on their own
			Compare_Pass_Cull();
		}
	}
	return m_instancecullpass;
}

void opengl46_renderer::walk_instances_end()
{
	End_Pass_Cull();
}

gfx::geometrybank_manager &opengl46_renderer::walk_geometry()
{
	return m_geometry;
}

bool opengl46_renderer::walk_blending_enabled() const
{
	return m_blendingenabled;
}



void opengl46_renderer::Render_grass(glm::mat4 const &Future)
{
	if (false == Global.gfx_grass || m_grass.empty())
		return;
	glDebug("render grass");
	// the blades bring their colour, the lights do the rest
	model_ubs.future = Future;
	model_ubs.param[0] = glm::vec4(1.f);
	model_ubs.param[1] = glm::vec4(1.f, 1.f, 0.f, 1.f); // diffuse, specular, reflection, glossiness
	model_ubs.param[2] = glm::vec4(0.f);
	model_ubs.emission = 0.f;
	model_ubs.alpha_mult = 1.f;
	model_ubs.opacity = 0.5f;
	glm::mat4 const view{OpenGLMatrices.data(GL_MODELVIEW)};
	auto const triangles{m_grass.render(m_renderpass.pass_camera, view, [this](glm::mat4 const &Modelview) {
		model_ubs.set_modelview(Modelview);
		model_ubo->update(model_ubs);
	})};
	m_renderpass.draw_stats.triangles += static_cast<int>(triangles);
}

void opengl46_renderer::Render_particles()
{
	model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
	model_ubo->update(model_ubs);

	Bind_Texture(0, m_smoketexture);
	// the smoke of the gpu, when its programs could be made; otherwise the particle manager simulates it on the cpu. a switch
	// takes effect from the next update of the manager
	auto const gpu{Global.gfx_smoke_gpu && m_gpuparticles.smoke_available()};
	simulation::Particles.gpu(gpu);
	if (gpu)
	{
		if (Global.Smoke)
		{
			m_gpuparticles.simulate_smoke(simulation::Particles, m_renderpass.pass_camera.position());
			m_renderpass.draw_stats.particles = static_cast<int>(m_gpuparticles.render_smoke(Particles_View()));
		}
		return;
	}
	m_particlerenderer.update(m_renderpass.pass_camera);
    m_renderpass.draw_stats.particles = m_particlerenderer.render();
}

opengl46_gpu_particles::view opengl46_renderer::Particles_View() const
{
	GLint viewport[4];
	::glGetIntegerv(GL_VIEWPORT, viewport);
	opengl46_gpu_particles::view view;
	view.modelview = OpenGLMatrices.data(GL_MODELVIEW);
	view.projection = OpenGLMatrices.data(GL_PROJECTION);
	view.camera = m_renderpass.pass_camera.position();
	view.height = viewport[3];
	view.soft = m_particlesoft;
	return view;
}

bool opengl46_renderer::Setup_Particles_Depth(viewport_config &Viewport)
{
	// the fading works out the distance from the depth as the contact shadows do (Contact_Shadows_Wanted())
	if (Global.gfx_skippipeline || Global.bWireFrame || (Viewport.msaa_fb == nullptr))
		return false;
	auto const &projection{scene_ubs.projection};
	if ((projection[2][3] == 0.f) || (projection[2][2] != 0.f) || (projection[3][2] <= 0.f))
		return false;
	if (false == Resolved_Depth_Target(Viewport))
		return false;
	// making the target binds it
	Viewport.msaa_fb->bind();
	// the scissor test would limit the blit
	auto const scissor{::glIsEnabled(GL_SCISSOR_TEST) == GL_TRUE};
	if (scissor)
		::glDisable(GL_SCISSOR_TEST);
	::glBlitNamedFramebuffer(*Viewport.msaa_fb, *Viewport.clusters_depth_fb, 0, 0, Viewport.width, Viewport.height, 0, 0, Viewport.width, Viewport.height, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
	if (scissor)
		::glEnable(GL_SCISSOR_TEST);
	::glBindTextureUnit(gl::SCENE_DEPTH_TEX, Viewport.clusters_depth_tex->id);
	return true;
}

void opengl46_renderer::Render_vr_models()
{
    if (!vr)
        return;

    glDebug("Render_vr_models");

    ::glPushMatrix();
    glm::dmat4 tmpmat;
    Global.pCamera.SetMatrix(tmpmat);
    tmpmat = glm::dmat3(tmpmat);
    glMultMatrixd(glm::value_ptr(glm::inverse(tmpmat)));
    std::vector<TModel3d*> list = vr->get_render_models();
    material_data data;
    for (TModel3d *mdl : list)
        Render(mdl, &data, 0.0);
    ::glPopMatrix();
}

void opengl46_renderer::Render_precipitation()
{
	if (Global.Overcast <= 1.f)
	{
		return;
	}

	if (Global.gfx_precipitation_gpu && m_gpuparticles.precipitation_available())
	{
		// the particles of the gpu: the overcast above 1 is how heavy the rain or snow is
		model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
		model_ubo->update(model_ubs);
		m_gpuparticles.render_precipitation(Particles_View(), Global.Weather == "snow:", std::clamp(Global.Overcast - 1.f, 0.f, 1.f), m_framestamp);
		return;
	}

	::glPushMatrix();
	// tilt the precipitation cone against the velocity vector for crude motion blur
	// include current wind vector while at it
	auto const velocity {
		simulation::Environment.m_precipitation.m_cameramove * -1.0
		+ glm::dvec3{ simulation::Environment.wind() } * 0.5 };
	if (glm::length2(velocity) > 0.0)
	{
		auto const forward{glm::normalize(velocity)};
		auto left{glm::cross(forward, {0.0, 1.0, 0.0})};
		auto const rotationangle{std::min(45.0, ((FreeFlyModeFlag || simulation::Train == nullptr) ?
			5 * glm::length(velocity) : simulation::Train->Dynamic()->GetVelocity() * 0.2))};
		::glRotated(rotationangle, left.x, 0.0, left.z);
	}
	if (false == FreeFlyModeFlag)
	{
		// counter potential vehicle roll
		auto const roll{0.5 * glm::degrees(simulation::Train->Dynamic()->Roll())};
		if (roll != 0.0)
		{
			auto const forward{simulation::Train->Dynamic()->VectorFront()};
			auto const vehicledirection = simulation::Train->Dynamic()->DirectionGet();
			::glRotated(roll, forward.x, 0.0, forward.z);
		}
	}
	if (!Global.iPause)
	{
		if (Global.Weather == "rain:")
			// oddly enough random streaks produce more natural looking rain than ones the eye can follow
			m_precipitationrotation = LocalRandom() * 360;
		else
			m_precipitationrotation = 0.0;
	}

	::glRotated(m_precipitationrotation, 0.0, 1.0, 0.0);

	model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
	model_ubs.param[0] = glm::mix(0.5f * (Global.DayLight.diffuse + Global.DayLight.ambient), colors::white, 0.5f * std::clamp((float)Global.fLuminance, 0.f, 1.f));
	model_ubs.param[1].x = simulation::Environment.m_precipitation.get_textureoffset();
	model_ubo->update(model_ubs);

    m_precipitationrenderer.update();
    m_precipitationrenderer.render();

	::glPopMatrix();
}


void opengl46_renderer::Render_editor_overlay()
{
	if (false == EditorModeFlag || EditorOverlay.batches.empty())
		return;
	glDebug("Render_editor_overlay");

	if (m_editor_overlay_revision != EditorOverlay.revision)
	{
		m_editor_overlay_revision = EditorOverlay.revision;
		if (m_editor_overlay_bank == null_handle)
			m_editor_overlay_bank = Create_Bank();
		auto const &batches{EditorOverlay.batches};
		for (std::size_t i = 0; i < batches.size(); ++i)
		{
			gfx::vertex_array vertices;
			vertices.reserve(batches[i].points.size());
			for (auto const &point : batches[i].points)
				vertices.emplace_back(point, glm::vec3(0.f, 1.f, 0.f), glm::vec2());
			gfx::userdata_array userdata;
			if (i >= m_editor_overlay_geometry.size())
				m_editor_overlay_geometry.emplace_back();
			auto &geometry{m_editor_overlay_geometry[i]};
			if (geometry != null_handle)
				m_geometry.replace(vertices, userdata, geometry);
			else if (false == vertices.empty())
				geometry = m_geometry.create_chunk(vertices, userdata, m_editor_overlay_bank, batches[i].type);
		}
	}

	::glPushMatrix();
	auto const origin{EditorOverlay.origin - m_renderpass.pass_camera.position()};
	::glTranslated(origin.x, origin.y, origin.z);
	::glDisable(GL_CULL_FACE);
	::glDepthMask(GL_FALSE);
	m_line_shader->bind();
	auto const &batches{EditorOverlay.batches};
	for (std::size_t i = 0; i < batches.size() && i < m_editor_overlay_geometry.size(); ++i)
	{
		if (batches[i].points.empty() || m_editor_overlay_geometry[i] == null_handle)
			continue;
		model_ubs.param[0] = batches[i].color;
		draw(m_editor_overlay_geometry[i]);
		++m_renderpass.draw_stats.drawcalls;
	}
	::glDepthMask(GL_TRUE);
	::glEnable(GL_CULL_FACE);
	::glPopMatrix();
}


bool opengl46_renderer::Debug_Ui_State(std::optional<bool> param)
{
    if (param) {
        debug_ui_active = *param;
    }

    return debug_ui_active;
}

// utility methods
void opengl46_renderer::Update_Pick_Control()
{
	if (!m_picking_pbo->is_busy())
	{
		unsigned char pickreadout[4];
		if (m_picking_pbo->read_data(1, 1, pickreadout))
		{
			auto const controlindex = pick_index(glm::ivec3{pickreadout[0], pickreadout[1], pickreadout[2]});
			TSubModel const *control{nullptr};
            glm::vec2 position(0.0f);
            if ((controlindex > 0) && (controlindex <= m_pickcontrolsitems.size())) {
				control = m_pickcontrolsitems[controlindex - 1];
            } else if (255 - pickreadout[0] < m_picksurfaceitems.size()) {
                control = m_picksurfaceitems[255 - pickreadout[0]];
                position.x = pickreadout[1] / 255.0f;
                position.y = pickreadout[2] / 255.0f;
            }

			m_pickcontrolitem = control;

			for (auto f : m_control_pick_requests)
                f(m_pickcontrolitem, position);
			m_control_pick_requests.clear();
		}

		if (!m_control_pick_requests.empty())
		{
            glm::vec2 mousepos = Global.cursor_pos;
            mousepos.y = Global.window_size.y - mousepos.y; // cursor coordinates are flipped compared to opengl

			glm::ivec2 pickbufferpos;
            pickbufferpos = glm::ivec2(mousepos * glm::vec2(EU07_PICKBUFFERSIZE) / glm::vec2(Global.window_size));

            if (vr)
                pickbufferpos = glm::ivec2(EU07_PICKBUFFERSIZE / 2 + 1, EU07_PICKBUFFERSIZE / 2 + 1);
			pickbufferpos = glm::clamp(pickbufferpos, glm::ivec2(0, 0), glm::ivec2(EU07_PICKBUFFERSIZE - 1, EU07_PICKBUFFERSIZE - 1));

			Render_pass(*m_viewports.front().get(), rendermode::pickcontrols);
			m_pick_fb->bind();
			m_picking_pbo->request_read(pickbufferpos.x, pickbufferpos.y, 1, 1);
			m_pick_fb->unbind();
		}
	}
}

void opengl46_renderer::Update_Pick_Node()
{
	if (!m_picking_node_pbo->is_busy())
	{
		unsigned char pickreadout[4];
		if (m_picking_node_pbo->read_data(1, 1, pickreadout))
		{
			auto const nodeindex = pick_index(glm::ivec3{pickreadout[0], pickreadout[1], pickreadout[2]});
			scene::basic_node *node{nullptr};
			if ((nodeindex > 0) && (nodeindex <= m_picksceneryitems.size()))
			{
				node = m_picksceneryitems[nodeindex - 1];
			}

			m_picksceneryitem = node;

			for (auto f : m_node_pick_requests)
				f(m_picksceneryitem);
			m_node_pick_requests.clear();
		}

		if (!m_node_pick_requests.empty())
		{
			// determine point to examine
            glm::vec2 mousepos = Global.cursor_pos;
            mousepos.y = Global.window_size.y - mousepos.y; // cursor coordinates are flipped compared to opengl

            glm::ivec2 pickbufferpos;
            pickbufferpos = glm::ivec2(mousepos * glm::vec2(EU07_PICKBUFFERSIZE) / glm::vec2(Global.window_size));
			pickbufferpos = glm::clamp(pickbufferpos, glm::ivec2(0, 0), glm::ivec2(EU07_PICKBUFFERSIZE - 1, EU07_PICKBUFFERSIZE - 1));

			Render_pass(*m_viewports.front().get(), rendermode::pickscenery);
			m_pick_fb->bind();
			m_picking_node_pbo->request_read(pickbufferpos.x, pickbufferpos.y, 1, 1);
			m_pick_fb->unbind();
		}
	}
}

void opengl46_renderer::Pick_Control_Callback(std::function<void(TSubModel const *, const glm::vec2 pos)> callback)
{
  if (!Global.render_cab) {
    callback(nullptr, glm::vec2());
    return;
  }
	m_control_pick_requests.push_back(callback);
}

void opengl46_renderer::Pick_Node_Callback(std::function<void(scene::basic_node *)> callback)
{
	m_node_pick_requests.push_back(callback);
}

glm::dvec3 opengl46_renderer::Update_Mouse_Position()
{
	if (!m_depth_pointer_pbo->is_busy())
	{
		// determine point to examine
        glm::vec2 mousepos = Global.cursor_pos;
        mousepos.y = Global.window_size.y - mousepos.y; // cursor coordinates are flipped compared to opengl

        glm::ivec2 bufferpos;
        bufferpos = glm::ivec2(mousepos * glm::vec2(Global.gfx_framebuffer_width, Global.gfx_framebuffer_height) / glm::vec2(Global.window_size));
		bufferpos = glm::clamp(bufferpos, glm::ivec2(0, 0), glm::ivec2(Global.gfx_framebuffer_width - 1, Global.gfx_framebuffer_height - 1));

		float pointdepth = std::numeric_limits<float>::max();

		m_depth_pointer_pbo->read_data(1, 1, &pointdepth, 4);

		if (!Global.iMultisampling)
		{
			m_depth_pointer_pbo->request_read(bufferpos.x, bufferpos.y, 1, 1, 4, GL_DEPTH_COMPONENT, GL_FLOAT);
		}
		else if (Global.gfx_skippipeline)
		{
			gl::framebuffer::blit(nullptr, m_depth_pointer_fb.get(), bufferpos.x, bufferpos.y, 1, 1, GL_DEPTH_BUFFER_BIT, 0);

			m_depth_pointer_fb->bind();
			m_depth_pointer_pbo->request_read(0, 0, 1, 1, 4, GL_DEPTH_COMPONENT, GL_FLOAT);
			m_depth_pointer_fb->unbind();
		}
		else
		{
			gl::framebuffer::blit(m_viewports.front()->msaa_fb.get(), m_depth_pointer_fb.get(), bufferpos.x, bufferpos.y, 1, 1, GL_DEPTH_BUFFER_BIT, 0);

			m_depth_pointer_fb->bind();
			m_depth_pointer_pbo->request_read(0, 0, 1, 1, 4, GL_DEPTH_COMPONENT, GL_FLOAT);
			m_viewports.front()->msaa_fb->bind();
		}

		if (pointdepth != std::numeric_limits<float>::max())
		{
			m_mousehit = pointdepth > 0.0f;
			if (pointdepth > 0.0f)
				m_worldmousecoordinates = glm::unProjectZO(glm::vec3(bufferpos, pointdepth), glm::mat4(glm::mat3(m_colorpass.pass_camera.modelview())), m_colorpass.pass_camera.projection(),
				                                           glm::vec4(0, 0, Global.gfx_framebuffer_width, Global.gfx_framebuffer_height));
		}
	}

	return m_colorpass.pass_camera.position() + glm::dvec3{m_worldmousecoordinates};
}

void opengl46_renderer::Update(double const Deltatime)
{
	Update_Pick_Control();
	Update_Pick_Node();

	m_updateaccumulator += Deltatime;

	if (m_updateaccumulator < 1.0)
	{
		// too early for any work
		return;
	}

	m_updateaccumulator = 0.0;
    m_framerate = 1000.f / ( Timer::subsystem.mainloop_total.average() );

	// adjust draw ranges etc, based on recent performance

	if (Global.targetfps != 0.0f) {
		float fps_diff = Global.targetfps - m_framerate;
		if (fps_diff > 0.0f)
			Global.fDistanceFactor = std::max(0.5f, Global.fDistanceFactor - 0.05f);
		else
			Global.fDistanceFactor = std::min(Global.gfx_distance_factor_max, Global.fDistanceFactor + 0.05f);
	}
	// legacy framerate parameters
	else if (Global.fFpsAverage != 0.f) {
		auto const fps_diff = m_framerate - Global.fFpsAverage;
		if (fps_diff < -Global.fFpsDeviation) {
			Global.fDistanceFactor = std::min(std::max(1.0f, Global.fDistanceFactor - 0.05f), Global.gfx_distance_factor_max);
		}
		else if (fps_diff > Global.fFpsDeviation) {
			Global.fDistanceFactor = std::min(std::min(3.0f, Global.fDistanceFactor + 0.05f), Global.gfx_distance_factor_max);
		}
	}

    if( Global.UpdateMaterials ) {
        // update resources if there was environmental change
        simulation_state simulationstate {
            Global.Weather,
            Global.Season
        };
        std::swap( m_simulationstate, simulationstate );
        bool changed { false };
        if( ( m_simulationstate.season != simulationstate.season ) && ( false == simulationstate.season.empty() ) ) {
            m_materials.on_season_change();
            changed = true;
        }
        if( ( m_simulationstate.weather != simulationstate.weather ) && ( false == simulationstate.weather.empty() ) ) {
            m_materials.on_weather_change();
            changed = true;
        }
        if( changed ) {
            // the heightmap terrain keeps copies of the textures of its layer materials
            m_terrainmaterials.on_environment_change();
            // the seasonal variants of the models are shown and hidden anew
            m_walkrecords.clear();
        }
    }

	if ((true == Global.ResourceSweep) && (true == simulation::is_ready))
	{
		// garbage collection
		m_geometry.update();
		m_textures.update();
	}

	if ((Global.gfx_texture_releasedistance > 0.f) && (true == simulation::is_ready))
	{
		Update_Texture_Release();
	}

	if ((true == Global.gfx_texture_streaming) && (true == simulation::is_ready) && (simulation::Region != nullptr))
	{
		Update_Texture_Prefetch();
	}

	if ((true == Global.ControlPicking) && (false == FreeFlyModeFlag))
        Pick_Control_Callback([](const TSubModel *, const glm::vec2) {});
	// temporary conditions for testing. eventually will be coupled with editor mode
	if ((true == Global.ControlPicking) && (true == DebugModeFlag) && (true == FreeFlyModeFlag))
        Pick_Node_Callback([](scene::basic_node *) {});

	// dump last opengl error, if any
    int glerror;
    while ((glerror = glGetError()) != GL_NO_ERROR)
	{
		std::string glerrorstring;
		if (glerror == GL_INVALID_ENUM)
			glerrorstring = "GL_INVALID_ENUM";
		else if (glerror == GL_INVALID_VALUE)
			glerrorstring = "GL_INVALID_VALUE";
		else if (glerror == GL_INVALID_OPERATION)
			glerrorstring = "GL_INVALID_OPERATION";
		else if (glerror == GL_OUT_OF_MEMORY)
			glerrorstring = "GL_OUT_OF_MEMORY";
		else if (glerror == GL_INVALID_FRAMEBUFFER_OPERATION)
			glerrorstring = "GL_INVALID_FRAMEBUFFER_OPERATION";
		Global.LastGLError = std::to_string(glerror) + " (" + glerrorstring + ")";
	}
}

// debug performance string
std::string const &opengl46_renderer::info_times() const
{

	return m_debugtimestext;
}

std::vector<gfx_renderer::info_section> const &opengl46_renderer::info_sections() const
{
	return m_debugsections;
}

std::string const &opengl46_renderer::info_stats() const
{

	return m_debugstatstext;
}

void opengl46_renderer::MakeScreenshot()
{
    // rows are tightly packed RGB: the renderer sets GL_PACK_ALIGNMENT to 1
    auto img = new char[Global.fb_size.x * Global.fb_size.y * 3];
    glReadPixels(0, 0, Global.fb_size.x, Global.fb_size.y, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)img);
	//m7t: use pbo
	screenshot_manager::save(img, Global.fb_size.x, Global.fb_size.y);
}

// model previews

namespace
{
// direction from the previewed model to the camera. models are made facing -z (the front view of 3ds max),
// so the camera looks at their front, right side and top
glm::dvec3 const preview_viewdirection{-0.6123724, 0.5, -0.6123724};
// vertical field of view of the preview camera, in degrees
float const preview_fieldofview{30.f};
// brightness of the hdr background, past the white point of the tonemapping, so translucent parts blend with white
float const preview_background{16.f};
} // namespace

void opengl46_renderer::Preview_Bounds(TSubModel const *Submodel, glm::dmat4 const &Transform, glm::dvec3 &Min, glm::dvec3 &Max) const
{
	// same walk as the one of Render(TSubModel *) bar the range checks, which leave out parts only for the bounds to be smaller
	for (auto const *submodel{Submodel}; submodel != nullptr; submodel = submodel->Next)
	{
		if ((submodel->iVisible == 0) || (submodel->fVisible <= 0.f))
		{
			continue;
		}
		auto transform{Transform};
		if ((submodel->iFlags & 0xC000) && (submodel->fMatrix != nullptr))
		{
			transform *= glm::dmat4{glm::make_mat4(submodel->fMatrix->readArray())};
		}
		if ((submodel->eType < TP_ROTATOR) && (submodel->m_geometry.handle != null_handle))
		{
			for (auto const &vertex : m_geometry.vertices(submodel->m_geometry.handle))
			{
				auto const point{glm::dvec3{transform * glm::dvec4{glm::dvec3{vertex.position}, 1.0}}};
				Min = glm::min(Min, point);
				Max = glm::max(Max, point);
			}
		}
		Preview_Bounds(submodel->Child, transform, Min, Max);
	}
}

bool opengl46_renderer::Preview_Targets(int const Size)
{
	if (m_preview.size == Size)
	{
		return true;
	}
	m_preview = preview_targets();

	m_preview.hdr_tex = std::make_unique<opengl_texture>();
	m_preview.hdr_tex->alloc_rendertarget(GL_RGBA16F, GL_RGBA, Size, Size, 1, 1, GL_CLAMP_TO_EDGE);
	m_preview.hdr_rbds = std::make_unique<gl::renderbuffer>();
	m_preview.hdr_rbds->alloc(GL_DEPTH32F_STENCIL8, Size, Size);
	m_preview.hdr_fb = std::make_unique<gl::framebuffer>();
	m_preview.hdr_fb->attach(*m_preview.hdr_tex, GL_COLOR_ATTACHMENT0);
	m_preview.hdr_fb->attach(*m_preview.hdr_rbds, GL_DEPTH_STENCIL_ATTACHMENT);
	m_preview.hdr_fb->setup_drawing(1);
	if (false == m_preview.hdr_fb->is_complete())
	{
		ErrorLog("preview framebuffer setup failed");
		m_preview = preview_targets();
		return false;
	}
	// the tonemapping writes srgb values, either by itself or through the srgb target
	m_preview.ldr_tex = std::make_unique<opengl_texture>();
	m_preview.ldr_tex->alloc_rendertarget((Global.gfx_shadergamma ? GL_RGBA8 : GL_SRGB8_ALPHA8), GL_RGBA, Size, Size, 1, 1, GL_CLAMP_TO_EDGE);
	m_preview.ldr_fb = std::make_unique<gl::framebuffer>();
	m_preview.ldr_fb->attach(*m_preview.ldr_tex, GL_COLOR_ATTACHMENT0);
	m_preview.ldr_fb->setup_drawing(1);
	if (false == m_preview.ldr_fb->is_complete())
	{
		ErrorLog("preview framebuffer setup failed");
		m_preview = preview_targets();
		return false;
	}
	gl::framebuffer::unbind();

	m_preview.size = Size;
	return true;
}

void opengl46_renderer::Preview_Model(TAnimModel *Instance, float const Squaredistance, bool const Alpha)
{
	// the placement of Render(TAnimModel *), without its range and visibility tests
	::glPushMatrix();
	auto const position{Instance->location() - m_renderpass.pass_camera.position()};
	::glTranslated(position.x, position.y, position.z);
	auto const &angle{Instance->vAngle};
	if (angle.y != 0.0)
		::glRotated(angle.y, 0.f, 1.f, 0.f);
	if (angle.x != 0.0)
		::glRotated(angle.x, 1.f, 0.f, 0.f);
	if (angle.z != 0.0)
		::glRotated(angle.z, 0.f, 0.f, 1.f);
	auto const &scale{Instance->Scale()};
	::glScalef(scale.x, scale.y, scale.z);
	if (Alpha)
		Render_Alpha(Instance->pModel, Instance->Material(), Squaredistance);
	else
		Render(Instance->pModel, Instance->Material(), Squaredistance);
	::glPopMatrix();
}

void opengl46_renderer::Preview_Color(TAnimModel *Instance, float const Squaredistance, int const Size, glm::dvec3 const &Eye, glm::mat4 const &View, glm::mat4 const &Projection, bool const Shadows)
{
	m_renderpass.draw_mode = rendermode::color;
	m_renderpass.pass_camera.position() = Eye;
	m_renderpass.pass_camera.modelview() = View;
	m_renderpass.pass_camera.projection() = Projection;
	m_renderpass.viewport_camera.position() = Eye;
	setup_matrices();

	scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
	scene_ubs.inv_view = glm::inverse(glm::mat4{glm::mat3{View}});
	scene_ubs.time = 0.f;
	for (auto idx = 0; idx < m_shadowpass.size(); ++idx)
	{
		// everything is in the first cascade
		scene_ubs.cascade_end[idx] = std::numeric_limits<float>::max();
	}
	scene_ubs.shadow_fit = glm::vec4{0.f};
	scene_ubo->update(scene_ubs);
	scene_ubo->bind_uniform();
	// the sun is the only light, and there's no fog
	auto const &modelview{OpenGLMatrices.data(GL_MODELVIEW)};
	light_data.ambient = m_sunlight.ambient;
	light_data.fog_color = glm::vec3{1.f};
	light_data.lights_total = 1;
	light_data.lights_count = 1;
	light_data.lights_clustered = 0;
	// a uniform ambient light, as the preview always had
	light_data.sky_ambient_factor = 0.f;
	light_data.env_ggx = 0;
	light_data.lights[0].type = gl::light_element::DIR;
	light_data.lights[0].dir = modelview * glm::vec4(m_sunlight.direction, 0.0f);
	light_data.lights[0].color = m_sunlight.diffuse;
	light_data.lights[0].ambient = 0.0f;
	light_data.lights[0].intensity = 1.0f;
	upload_light_header();
	model_ubs.fog_density = 0.f;
	model_ubs.future = glm::mat4();
	model_ubs.emission = 0.f;
	if (Shadows)
	{
		setup_shadow_bind_map();
		setup_shadow_color(colors::shadow);
	}
	else
	{
		setup_shadow_empty_map();
		setup_shadow_color(colors::white);
	}
	setup_env_map(nullptr);

	m_preview.hdr_fb->bind();
	glViewport(0, 0, Size, Size);
	glClearColor(preview_background, preview_background, preview_background, 1.f);
	glClearStencil(0);
	glStencilMask(0xFF);
	m_preview.hdr_fb->clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_DEPTH_TEST);
	// every fragment which made it past the alpha test marks the pixel as covered, translucent ones included
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS, 1, 0xFF);
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

	setup_drawing(false);
	Preview_Model(Instance, Squaredistance, false);
	setup_drawing(true);
	Preview_Model(Instance, Squaredistance, true);
	setup_drawing(false);

	glDisable(GL_STENCIL_TEST);
	setup_shadow_empty_map();
}

bool opengl46_renderer::Render_Preview(TAnimModel *Instance, int const Size, int const Margin, bool const Shadows, std::vector<std::uint8_t> &Image)
{
	if ((Instance == nullptr) || (Instance->pModel == nullptr) || (Size <= 0) || (Margin < 0) || (Margin * 2 >= Size))
	{
		return false;
	}
	// the image is drawn at double size and scaled down, for antialiased edges which blend with the white background
	GLint maxtexturesize{0}, maxrenderbuffersize{0};
	GLint maxviewport[2]{0, 0};
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxtexturesize);
	glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &maxrenderbuffersize);
	glGetIntegerv(GL_MAX_VIEWPORT_DIMS, maxviewport);
	auto const maxsize{std::min({maxtexturesize, maxrenderbuffersize, maxviewport[0], maxviewport[1]})};
	if (Size > maxsize)
	{
		ErrorLog("preview size " + std::to_string(Size) + " exceeds the limit of the graphics driver, " + std::to_string(maxsize));
		return false;
	}
	auto const supersampling{(Size * 2 <= maxsize ? 2 : 1)};
	auto const targetsize{Size * supersampling};
	if (false == Preview_Targets(targetsize))
	{
		return false;
	}

	glfwMakeContextCurrent(m_window);
	opengl_texture::reset_unit_cache();
	gfx::opengl46_vaogeometrybank::reset();
	m_previewing = true;
	// lights and visibility of the submodels, for this instance
	Instance->RaAnimate(m_framestamp);
	Instance->RaPrepare();

	// bounding sphere of the model, which the camera is placed to fit in its view
	glm::dvec3 boundsmin{std::numeric_limits<double>::max()};
	glm::dvec3 boundsmax{std::numeric_limits<double>::lowest()};
	Preview_Bounds(Instance->pModel->Root, glm::translate(glm::dmat4{1.0}, Instance->location()) * glm::dmat4{Instance->rotation_scale()}, boundsmin, boundsmax);
	if (boundsmin.x > boundsmax.x)
	{
		m_previewing = false;
		return false;
	}
	auto const centre{(boundsmin + boundsmax) * 0.5};
	auto const radius{std::max(0.01, glm::length(boundsmax - boundsmin) * 0.5)};
	auto const fieldofview{glm::radians(preview_fieldofview)};
	auto const distance{radius / std::sin(fieldofview * 0.5)};
	auto const eye{centre + glm::normalize(preview_viewdirection) * distance};
	glm::mat4 const view{glm::lookAt(eye, centre, glm::dvec3{0.0, 1.0, 0.0})};
	// the sphere is given some slack for parts which aren't where their vertices are, like billboards turning to the camera
	auto const znear{static_cast<float>(std::max(distance - radius * 1.5, distance * 0.05))};
	auto const zfar{static_cast<float>(distance + radius * 1.5)};
	auto const projection{perspective_projection_raw(fieldofview, 1.f, znear, zfar)};
	// level of detail as seen from the camera, like in Render(TAnimModel *)
	auto const squaredistance{static_cast<float>(glm::length2(Instance->location() - eye) / Global.fDistanceFactor)};

	// key light from above the left shoulder of the viewer, so the visible sides of the model differ in tone
	auto const forward{glm::normalize(centre - eye)};
	auto const right{glm::normalize(glm::cross(forward, glm::dvec3{0.0, 1.0, 0.0}))};
	auto const tosun{glm::normalize(-forward * 0.4 + glm::dvec3{0.0, 1.0, 0.0} - right * 0.9)};
	m_sunlight.direction = glm::vec3{-tosun};
	m_sunlight.diffuse = glm::vec4{1.0f, 0.9686275f, 0.9411765f, 1.f};
	m_sunlight.ambient = glm::vec4{0.65f, 0.66f, 0.68f, 1.f};
	m_sunlight.factor = 1.f;

	auto const shadows{Shadows && Global.gfx_shadowmap_enabled && (m_shadow_fb != nullptr)};
	if (shadows)
	{
		// one map covering the whole model, seen from the sun
		m_renderpass.draw_mode = rendermode::shadows;
		m_renderpass.pass_camera.position() = centre;
		m_renderpass.pass_camera.modelview() = glm::mat4{glm::lookAt(centre, centre + glm::dvec3{m_sunlight.direction}, glm::dvec3{0.0, 1.0, 0.0})};
		auto const extent{static_cast<float>(radius * 1.5)};
		m_renderpass.pass_camera.projection() = ortho_projection(-extent, extent, -extent, extent, -extent, extent);
		m_renderpass.viewport_camera.position() = eye;
		setup_matrices();
		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
		scene_ubo->update(scene_ubs);

		glViewport(0, 0, m_shadowbuffersize, m_shadowbuffersize);
		m_shadow_fb->attach(*m_shadow_tex, GL_DEPTH_ATTACHMENT, 0);
		m_shadow_fb->clear(GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		setup_drawing(false);
		Preview_Model(Instance, squaredistance, false);
		m_shadow_fb->unbind();
		for (auto &pass : m_shadowpass)
		{
			pass = m_renderpass;
		}
	}

	// the view is cropped to the part of the image the model covers: a look from a bit further first, to find it,
	// then a closer one at the full size, as thin parts can show up only there, to refine the crop to a pixel.
	// the crop maps the area to the image less the margins
	std::vector<std::uint8_t> coverage;
	auto const fitsize{targetsize};
	auto const area{1.0 - 2.0 * Margin / Size}; // half of the extent of the area for the model, in normalized device coordinates
	glm::dvec2 cropcentre{0.0};
	auto cropscale{0.75};
	auto tight{false}; // whether the crop was fitted to the model
	auto const cropped = [&]() {
		return glm::mat4{glm::scale(glm::dmat4{1.0}, glm::dvec3{cropscale, cropscale, 1.0}) * glm::translate(glm::dmat4{1.0}, glm::dvec3{-cropcentre, 0.0})} * projection;
	};
	for (auto attempt = 0; attempt < 6; ++attempt)
	{
		auto const passsize{(tight ? fitsize : std::min(fitsize, 512))};
		Preview_Color(Instance, squaredistance, passsize, eye, view, cropped(), shadows);
		coverage.resize(static_cast<std::size_t>(passsize) * passsize);
		m_preview.hdr_fb->bind();
		glReadPixels(0, 0, passsize, passsize, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, coverage.data());

		glm::ivec2 pixelmin{passsize, passsize}, pixelmax{-1, -1};
		for (auto y = 0; y < passsize; ++y)
		{
			auto const *row{coverage.data() + static_cast<std::size_t>(y) * passsize};
			for (auto x = 0; x < passsize; ++x)
			{
				if (row[x] != 0)
				{
					pixelmin = glm::min(pixelmin, glm::ivec2{x, y});
					pixelmax = glm::max(pixelmax, glm::ivec2{x, y});
				}
			}
		}
		if (pixelmax.x < 0)
		{
			if (attempt == 0)
			{
				// nothing to see
				m_previewing = false;
				m_renderpass.draw_mode = rendermode::none;
				gl::framebuffer::unbind();
				return false;
			}
			// the crop missed the model, start over with the plain view
			cropcentre = glm::dvec2{0.0};
			cropscale = 0.5;
			tight = false;
			continue;
		}
		// covered area in normalized device coordinates of the uncropped view
		auto const tondc = [&](glm::ivec2 const &Pixel) { return (glm::dvec2{-1.0} + 2.0 * glm::dvec2{Pixel} / static_cast<double>(passsize)) / cropscale + cropcentre; };
		auto const ndcmin{tondc(pixelmin)};
		auto const ndcmax{tondc(pixelmax + 1)};
		cropcentre = (ndcmin + ndcmax) * 0.5;
		// a model fitted to the area keeps clear of the image edges, unless the margin is too thin to tell
		auto const clipped{(pixelmin.x == 0) || (pixelmin.y == 0) || (pixelmax.x == passsize - 1) || (pixelmax.y == passsize - 1)};
		if (clipped && ((false == tight) || (Margin * passsize / Size >= 2)))
		{
			// part of the model is out of the view, look again from further away
			cropscale *= 0.5;
			tight = false;
			continue;
		}
		cropscale = 2.0 * area / std::max(ndcmax.x - ndcmin.x, ndcmax.y - ndcmin.y);
		if (tight)
		{
			// measured through a fitted crop, which makes it accurate to a pixel of the fitting pass
			break;
		}
		tight = true;
	}

	// the image itself
	Preview_Color(Instance, squaredistance, targetsize, eye, view, cropped(), shadows);
	glViewport(0, 0, targetsize, targetsize);
	if (!Global.gfx_shadergamma)
		glEnable(GL_FRAMEBUFFER_SRGB);
	// no bloom, no automatic exposure
	model_ubs.param[0] = glm::vec4{0.f, 0.f, 1.f, 0.f};
	model_ubo->update(model_ubs);
	m_pfx_tonemapping->apply(*m_preview.hdr_tex, m_preview.ldr_fb.get());
	if (!Global.gfx_shadergamma)
		glDisable(GL_FRAMEBUFFER_SRGB);
	opengl_texture::reset_unit_cache();
	// the post processing binds its own vertex array, which the geometry banks don't know about
	gfx::opengl46_vaogeometrybank::reset();

	auto const targetpixels{static_cast<std::size_t>(targetsize) * targetsize};
	std::vector<std::uint8_t> colors(targetpixels * 4);
	m_preview.ldr_fb->bind();
	glReadPixels(0, 0, targetsize, targetsize, GL_RGBA, GL_UNSIGNED_BYTE, colors.data());
	coverage.resize(targetpixels);
	m_preview.hdr_fb->bind();
	glReadPixels(0, 0, targetsize, targetsize, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, coverage.data());
	gl::framebuffer::unbind();
	glDepthMask(GL_TRUE);

	m_previewing = false;
	m_renderpass.draw_mode = rendermode::none;

	// uncovered pixels are white, the samples are averaged in linear space and the rows flipped to go from the top
	static auto const tolinear{[]() {
		std::array<float, 256> table;
		for (auto idx = 0; idx < 256; ++idx)
		{
			auto const value{idx / 255.f};
			table[idx] = (value <= 0.04045f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f));
		}
		return table;
	}()};
	auto const tosrgb = [](float const Value) {
		auto const value{std::clamp(Value, 0.f, 1.f)};
		auto const encoded{(value <= 0.0031308f ? value * 12.92f : 1.055f * std::pow(value, 1.f / 2.4f) - 0.055f)};
		return static_cast<std::uint8_t>(std::lround(encoded * 255.f));
	};
	auto const samplecount{static_cast<float>(supersampling * supersampling)};
	Image.resize(static_cast<std::size_t>(Size) * Size * 3);
	for (auto y = 0; y < Size; ++y)
	{
		auto *output{Image.data() + static_cast<std::size_t>(y) * Size * 3};
		auto const sourcey{(Size - 1 - y) * supersampling};
		for (auto x = 0; x < Size; ++x)
		{
			glm::vec3 color{0.f};
			for (auto sy = 0; sy < supersampling; ++sy)
			{
				for (auto sx = 0; sx < supersampling; ++sx)
				{
					auto const sample{static_cast<std::size_t>(sourcey + sy) * targetsize + x * supersampling + sx};
					if (coverage[sample] != 0)
					{
						auto const *rgba{colors.data() + sample * 4};
						color += glm::vec3{tolinear[rgba[0]], tolinear[rgba[1]], tolinear[rgba[2]]};
					}
					else
					{
						color += glm::vec3{1.f};
					}
				}
			}
			color /= samplecount;
			output[x * 3 + 0] = tosrgb(color.r);
			output[x * 3 + 1] = tosrgb(color.g);
			output[x * 3 + 2] = tosrgb(color.b);
		}
	}
	return true;
}

void opengl46_renderer::Update_Lights(light_array &Lights, bool const Headlights, bool const Clustered)
{
	glDebug("Update_Lights");

    Bind_Texture( gl::HEADLIGHT_TEX, m_headlightstexture );

	// arrange the light array from closest to farthest from current position of the camera
	auto const camera = m_colorpass.pass_camera.position();
	if (Headlights)
	{
		std::sort(
			std::begin(Lights.data), std::end(Lights.data),
			[&camera](light_array::light_record const &Left, light_array::light_record const &Right) {
				// move lights which are off at the end...
				if (Left.intensity  == 0.f) { return false; }
				if (Right.intensity == 0.f) { return true; }
				// ...otherwise prefer closer and/or brigher light sources
				return (glm::length2(camera - Left.position) / Left.intensity) < (glm::length2(camera - Right.position) / Right.intensity);
			});
	}

    // set up helpers
   	glm::mat4 coordmove;
	// transform 1..-1 NDC xy coordinates to 1..0 (with clip_control z is already 1..0)
	coordmove = glm::mat4( //
		0.5, 0.0, 0.0, 0.0, //
		0.0, 0.5, 0.0, 0.0, //
		0.0, 0.0, 1.0, 0.0, //
		0.5, 0.5, 0.0, 1.0 //
	);
    glm::mat4 mv = OpenGLMatrices.data( GL_MODELVIEW );

    // fill vehicle headlights data, for every light which is on
    // the light is worked out in a scratch object, nothing of it is kept for the next one
    opengl46_light renderlight;
    renderlight.is_directional = false;
    size_t light_i = 1;
    m_lights_dropped = 0;

    // the lights other than the headlights are only taken from the lists of the clusters. the lamp of the occupied cab goes first:
    // it is next to the camera, and it is not to be the one left out of a full cluster
    auto const locallights{Headlights && Clustered};
    m_cablamplit = false;
    if( locallights ) {
        if( auto const *lamp{ m_locallights.cab_lamp() } ) {
            auto &light{ light_data.lights[ light_i++ ] };
            Write_Local_Light( light, *lamp, camera, mv );
            m_cablamplit = true;
            auto const *vehicle{ m_locallights.cab_lamp_vehicle() };
            if( m_localshadows.ready( vehicle ) ) {
                // view space to the space of the map: the space of the cab model, with the lamp at the origin. the vehicle is
                // where it is now, the map stays right as it moves
                auto const toworld{ glm::dmat4{ glm::transpose( glm::mat3{ mv } ) } };
                auto const tomap{
                    glm::translate( glm::dmat4{ 1.0 }, -glm::dvec3{ m_localshadows.lamp() } )
                    * glm::inverse( vehicle->mMatrix )
                    * glm::translate( glm::dmat4{ 1.0 }, camera - vehicle->GetPosition() )
                    * toworld };
                light.headlight_projection = glm::mat4{ tomap };
                light.headlight_weights = m_localshadows.weights();
            }
        }
    }
    auto const headlightsstart{light_i};

    for (auto const &scenelight : Lights.data)
	{
		if (!Headlights || !Global.gfx_lights_headlights || scenelight.intensity == 0.f) {
			// all lights past this one are bound to be off (or the beams of the headlights are turned off altogether)
			break;
		}
		auto const lightoffset = glm::vec3{scenelight.position - camera};
		if (glm::length2(lightoffset) > sq(1000.f)) {
			// we don't care about lights past arbitrary limit of 1 km.
			// but there could still be weaker lights which are closer, so keep looking
			continue;
		}
		if (light_i >= gl::MAX_LIGHTS) {
			// the buffer is full, and the lights come in the order of importance
			++m_lights_dropped;
			continue;
		}
		// if the light passed tests so far, it's good enough
		renderlight.position = lightoffset;
		renderlight.direction = scenelight.direction;

		auto luminance = static_cast<float>(Global.fLuminance);
		// adjust luminance level based on vehicle's location, e.g. tunnels
		auto const environment = scenelight.owner->fShade;
		if (environment > 0.f)
		{
			luminance *= environment;
		}
		renderlight.diffuse = glm::vec4{glm::max(glm::vec3{colors::none}, scenelight.color - glm::vec3{luminance}), renderlight.diffuse[3]};
		renderlight.ambient = glm::vec4{glm::max(glm::vec3{colors::none}, scenelight.color * glm::vec3{scenelight.intensity} - glm::vec3{luminance}), renderlight.ambient[3]};

		renderlight.apply_intensity( ( scenelight.count * 0.5 ) * ( scenelight.owner->DimHeadlights ? 0.5 : 1.0 ) );
		renderlight.apply_angle();

		gl::light_element *light = &light_data.lights[light_i];

		light->pos = mv * glm::vec4(renderlight.position, 1.0f);
		light->dir = mv * glm::vec4(renderlight.direction, 0.0f);
		light->type = gl::light_element::HEADLIGHTS;
		light->in_cutoff = headlight_config.in_cutoff;
		light->out_cutoff = headlight_config.out_cutoff;
		light->color = renderlight.diffuse * renderlight.factor;
		light->linear = headlight_config.falloff_linear / 10.0f;
		light->quadratic = headlight_config.falloff_quadratic / 100.0f;
		light->ambient = headlight_config.ambient;
		light->intensity = headlight_config.intensity;
        // headlights-to-world projection
        // NOTE: we strip transformations from camera projections to remove jitter that occurs
        // with large (and unneded as we only need the offset) transformations back and forth
        auto const worldcam{ glm::mat3{ m_renderpass.pass_camera.modelview() } };
        {
            // headlight projection is 1 km long box aligned with vehicle rotation and aimed slightly downwards
            glm::dvec3 projector;
            glm::mat3 lightcam;
            glm::mat4 depthproj;
            float nearplane, fieldofview;
            Headlight_Projector( scenelight, projector, lightcam, depthproj, nearplane, fieldofview );
            // calculate world->headlight space projection matrix
            light->headlight_projection =
                coordmove
                * depthproj
                * glm::translate(
                    glm::mat4{ lightcam },
                    glm::vec3{ m_renderpass.pass_camera.position() - projector } )
                * glm::mat4{ glm::inverse( worldcam ) };
        }
        // headlights weights
        light->headlight_weights = { scenelight.state, 0.f };
        // the shadow map of the headlights, if they have one: the same projection, from where the projector was when it was drawn
        if( auto const shadowmap{ m_localshadows.headlight_map( scenelight.owner, scenelight.index ) }; shadowmap >= 0 ) {
            auto const &slot{ m_localshadows.headlight( static_cast<std::size_t>( shadowmap ) ) };
            scene_ubs.headlight_shadow[ shadowmap ] =
                coordmove
                * slot.projection
                * glm::translate(
                    glm::mat4{ slot.rotation },
                    glm::vec3{ m_renderpass.pass_camera.position() - slot.projector } )
                * glm::mat4{ glm::inverse( worldcam ) };
            scene_ubs.headlight_shadow_params[ shadowmap ] = m_localshadows.headlight_params( static_cast<std::size_t>( shadowmap ) );
            light->headlight_weights.w = static_cast<float>( shadowmap + 1 );
        }

		++light_i;
	}
    m_lights_lit = static_cast<uint32_t>(light_i - headlightsstart);
    // lamps of models and vehicles, the most important first, those which may be seen lighting something in this view
    if( locallights ) {
        auto const toworld{ glm::dmat4{ glm::transpose( glm::mat3{ mv } ) } };
        uint32_t placed{ 0 };
        uint32_t dropped{ 0 };
        for( auto const &local : m_locallights.lights() ) {
            if( false == m_renderpass.pass_camera.visible( scene::bounding_area{ local.position, local.range } ) ) {
                continue;
            }
            if( light_i >= gl::MAX_LIGHTS ) {
                ++dropped;
                continue;
            }
            auto &light{ light_data.lights[ light_i++ ] };
            Write_Local_Light( light, local, camera, mv );
            std::size_t slot;
            if( auto const *lamp{ m_localshadows.lamp( local, slot ) } ) {
                // view space to the space of the map: world axes with the lamp at the origin
                light.headlight_projection = glm::mat4{ glm::translate( glm::dmat4{ 1.0 }, camera - lamp->position ) * toworld };
                light.headlight_weights = m_localshadows.lamp_weights( slot );
                // the map is in the array of the scenery lamps
                light.linear = 1.0f;
            }
            ++placed;
        }
        m_locallights.placed( placed, dropped );
    }
    // every light is there for the clusters to be culled from, while the shading loop over the lights keeps to the number
    // of headlights it always went through (without the clusters only the headlights are written, from the first slot on)
    light_data.lights_total = static_cast<uint32_t>(light_i);
    light_data.lights_count = 1 + std::min<uint32_t>(m_lights_lit, static_cast<uint32_t>(std::max(Global.DynamicLightCount, 0)));
    light_data.lights_clustered = Clustered ? 1 : 0;
    light_data.env_ggx = ( Global.gfx_envmap_ggx && m_envmap_bound ) ? 1 : 0;
    // fill sunlight data
    light_data.ambient = m_sunlight.ambient * m_sunlight.factor;// *simulation::Environment.light_intensity();
    light_data.sky_ambient_factor = ( Global.gfx_ambient_sh ? m_sunlight.factor : 0.f );
	light_data.lights[0].type = gl::light_element::DIR;
	light_data.lights[0].dir = mv * glm::vec4(m_sunlight.direction, 0.0f);
	light_data.lights[0].color = m_sunlight.diffuse * m_sunlight.factor * simulation::Environment.light_intensity();
	light_data.lights[0].ambient = 0.0f;
	light_data.lights[0].intensity = 1.0f;
    // fill fog data
	light_data.fog_color = m_fogcolor;
	if (Global.fFogEnd > 0)
	{
		// the fog of the weather; the regular density is a reference the shading scales the fog of a draw with
		m_fogrange = m_weather.fog_range();
		model_ubs.fog_density = m_weather.fog_density();
	}
    else
    {
        model_ubs.fog_density = 0.0f;
    }
    // ship config data to the gpu
	model_ubo->update(model_ubs);
	// the header and the lights in use, the rest of the buffer is not read
	light_ssbo->update(&light_data, 0, offsetof(gl::light_data, lights) + sizeof(gl::light_element) * light_i);
}

void opengl46_renderer::Write_Local_Light(gl::light_element &Light, gfx::local_lights::light const &Source, glm::dvec3 const &Camera, glm::mat4 const &View)
{
	// a cone wider than a half of the space is no limit for the culling, but the shading still fades the light out of it
	Light.type = gl::light_element::SPOT;
	Light.pos = View * glm::vec4(glm::vec3{Source.position - Camera}, 1.0f);
	Light.dir = glm::normalize(glm::vec3{View * glm::vec4(Source.direction, 0.0f)});
	Light.in_cutoff = Source.cos_inner;
	Light.out_cutoff = Source.cos_outer;
	if (Source.cos_outer <= -1.f)
	{
		// all around
		Light.type = gl::light_element::POINT;
	}
	Light.color = Source.color;
	Light.intensity = 1.0f;
	Light.ambient = 0.0f;
	// the shading takes the light to end at 1 / sqrt(quadratic)
	Light.linear = 0.0f;
	Light.quadratic = 1.0f / sq(std::max(Source.range, 0.01f));
	Light.headlight_projection = glm::mat4{1.0f};
	// no shadow map
	Light.headlight_weights = glm::vec4{-1.0f, 0.0f, 0.0f, 0.0f};
}

void opengl46_renderer::Render_local_shadows()
{
	// only the light buffers with the lamps read the maps
	auto const *vehicle{Global.gfx_lights_clustered ? m_locallights.cab_lamp_vehicle() : nullptr};
	if (false == m_localshadows.update_wanted(vehicle, (vehicle != nullptr ? vehicle->mdKabina : nullptr), m_locallights.cab_lamp_local(), m_framestamp))
		return;

	glDebug("render lamp shadows");
	// the cab drawn by the shadow pass, with the programs of the cube map. the camera is put at the vehicle, which Render_cab()
	// places the cab relative to, and the modelview takes the cab model to the space of the map
	m_renderpass.draw_mode = rendermode::shadows;
	m_renderpass.pass_camera.position() = vehicle->GetPosition();
	m_renderpass.draw_stats = {};
	OpenGLMatrices.mode(GL_MODELVIEW);
	OpenGLMatrices.load_matrix(glm::mat4{glm::translate(glm::dmat4{1.0}, -glm::dvec3{m_localshadows.lamp()}) * glm::inverse(vehicle->mMatrix)});
	m_localshadows.begin();
	m_cubeshadows = true;
	Render_cab(vehicle, 0.0f, false);
	m_cubeshadows = false;
	m_localshadows.end(static_cast<std::uint32_t>(m_renderpass.draw_stats.drawcalls));
	m_renderpass = m_colorpass;
	glPolygonMode(GL_FRONT_AND_BACK, Global.bWireFrame ? GL_LINE : GL_FILL);
}

void opengl46_renderer::Render_lamp_shadows()
{
	m_lampscenerydrawn = 0;
	auto const &lamps{m_localshadows.lamps()};
	if (lamps.empty() || (false == Global.gfx_lights_clustered))
		return;

	// vehicles near the camera cast the shadows of the lamps, and the occupied cab
	m_localshadows.lamp_vehicles(Global.pCamera.Pos);
	auto const vehiclesrange{static_cast<double>(Global.gfx_lights_shadows_vehicles_range)};
	auto *cab{vehiclesrange > 0.0 ? m_localshadows.shadow_cab() : nullptr};

	auto timing{false};
	for (auto const idx : lamps)
	{
		auto const &slot{m_localshadows.slot(idx)};
		if (m_localshadows.scenery_wanted(idx))
		{
			if (false == timing)
			{
				m_localshadows.lamps_timer_begin();
				timing = true;
			}
			Render_lamp_shadow(idx, false, nullptr);
		}
		if (false == slot.scenery)
			continue;
		// the vehicles by the lamp, within its reach
		auto const &slotvehicles{m_localshadows.slot_vehicles(slot)};
		auto *slotcab{(cab != nullptr) && (glm::length(cab->GetPosition() - slot.position) <= slot.range + cab->radius()) ? cab : nullptr};
		if (slotvehicles.empty() && (slotcab == nullptr))
		{
			m_localshadows.no_vehicles(idx);
			continue;
		}
		if (false == m_localshadows.vehicles_wanted(idx, m_framestamp))
			continue;
		if (false == timing)
		{
			m_localshadows.lamps_timer_begin();
			timing = true;
		}
		Render_lamp_shadow(idx, true, slotcab);
	}
	if (timing)
		m_localshadows.lamps_timer_end();
}

void opengl46_renderer::Render_lamp_shadow(std::size_t const Slot, bool const Vehicles, TDynamicObject *Cab)
{
	auto const &slot{m_localshadows.slot(Slot)};
	glDebug("render lamp shadows");
	// the shadow pass from the lamp, in world axes: the camera of the pass is the lamp, its frustum the box of its reach. the
	// distances which pick the details of the models are measured from the real camera, as in the shadows of the sun
	m_renderpass = m_colorpass;
	m_renderpass.draw_mode = rendermode::shadows;
	m_renderpass.draw_range = slot.range;
	m_renderpass.draw_stats = {};
	auto &camera{m_renderpass.pass_camera};
	camera.position() = slot.position;
	camera.modelview() = glm::mat4{glm::translate(glm::dmat4{1.0}, -slot.position)};
	auto const reach{slot.range};
	camera.projection() = glm::ortho(-reach, reach, -reach, reach, -reach, reach);
	camera.update_frustum();
	OpenGLMatrices.mode(GL_MODELVIEW);
	OpenGLMatrices.load_matrix(glm::mat4{1.f});

	m_localshadows.begin_lamp(Slot, Vehicles, m_instancerecordsversion, m_framestamp);
	m_cubeshadows = true;
	if (Vehicles)
	{
		for (auto *vehicle : m_localshadows.slot_vehicles())
		{
			Render(vehicle);
		}
		if (Cab != nullptr)
		{
			Render_cab(Cab, 0.0f, false);
		}
	}
	else
	{
		m_cubescenery = true;
		m_cubeskipinstance = static_cast<TAnimModel const *>(slot.owner);
		// the instances culled on the gpu, each scenery part drawn in a frame with a culling of its own
		m_lightshadowcull = Light_Shadow_Cull(m_lampcull, m_lampscenerydrawn++, Global.gfx_gpucull_shadows_lamps);
		Render(simulation::Region);
		m_lightshadowcull = nullptr;
		m_cubeskipinstance = nullptr;
		m_cubescenery = false;
	}
	m_cubeshadows = false;
	m_localshadows.end_lamp(static_cast<std::uint32_t>(m_renderpass.draw_stats.drawcalls));
	m_renderpass = m_colorpass;
	glPolygonMode(GL_FRONT_AND_BACK, Global.bWireFrame ? GL_LINE : GL_FILL);
}

void opengl46_renderer::Headlight_Projector(light_array::light_record const &Light, glm::dvec3 &Position, glm::mat3 &Rotation, glm::mat4 &Projection, float &Near, float &Fieldofview)
{
	auto const &ownerdimensions{Light.owner->MoverParameters->Dim};
	auto const up{static_cast<glm::dvec3>(Light.owner->VectorUp())};
	auto const size{static_cast<float>(std::max(ownerdimensions.W, ownerdimensions.H) * 1.0)}; // ensure square ratio
	auto const cone{5.f};
	auto const offset{75.f * (5.f / cone)};
	Position = Light.owner->GetPosition() - glm::dvec3(Light.direction * offset) + up * (size * 0.5);
	Near = static_cast<float>(ownerdimensions.L * 0.5 + offset - 0.25);
	Fieldofview = glm::radians(cone);
/*
	Projection = ortho_projection(
		-size, size,
		-size, size,
		ownerdimensions.L * 0.5 - 0.5, 1000.0f );
*/
	Projection = perspective_projection_raw(Fieldofview, 1.0, Near, 1000.0f);
	glm::dmat4 viewmatrix{1.0};
	viewmatrix *= glm::lookAt(Position, Position - up * 1.0 + glm::dvec3{Light.direction * 1000.f}, glm::dvec3{0.f, 1.f, 0.f});
	Rotation = glm::mat3{glm::mat4{viewmatrix}};
}

void opengl46_renderer::Render_headlight_shadows()
{
	// the headlights which shine, the most important first (in the order of Update_Lights()), whose beam may be seen within the
	// reach of the shadows
	auto const &picked{m_localshadows.pick_headlights(simulation::Lights, m_colorpass.pass_camera)};
	m_localshadows.update_headlights(picked, m_framestamp);
	// the maps may have been made anew, in place of the ones bound for this frame
	m_localshadows.bind();
	auto const &slots{m_localshadows.headlights()};
	if (slots.empty())
		return;

	// the occupied cab, as in Render_lamp_shadows()
	auto *cab{m_localshadows.shadow_cab()};
	auto timing{false};
	for (auto const idx : slots)
	{
		auto const &slot{m_localshadows.headlight(idx)};
		light_array::light_record const *light{nullptr};
		for (auto const *candidate : picked)
		{
			if ((candidate->owner == slot.owner) && (candidate->index == slot.index))
			{
				light = candidate;
				break;
			}
		}
		if (light == nullptr)
			continue;
		glm::dvec3 projector;
		glm::mat3 rotation;
		glm::mat4 projection;
		float nearplane, fieldofview;
		Headlight_Projector(*light, projector, rotation, projection, nearplane, fieldofview);
		// a vehicle on the move needs its map anew, the things around it and the vehicles passing by only now and then
		auto const moved{(glm::length(projector - slot.projector) > 0.01) || (glm::dot(light->direction, slot.direction) < 0.99999f)};
		if (false == m_localshadows.headlight_wanted(idx, moved, m_framestamp))
			continue;
		if (false == timing)
		{
			m_localshadows.headlights_timer_begin();
			timing = true;
		}
		Render_headlight_shadow(idx, *light, cab);
	}
	if (timing)
		m_localshadows.headlights_timer_end();
}

void opengl46_renderer::Render_headlight_shadow(std::size_t const Slot, light_array::light_record const &Light, TDynamicObject *Cab)
{
	glDebug("render headlight shadows");
	glm::dvec3 projector;
	glm::mat3 rotation;
	glm::mat4 projection;
	float nearplane, fieldofview;
	Headlight_Projector(Light, projector, rotation, projection, nearplane, fieldofview);
	auto const reach{nearplane + Global.gfx_lights_shadows_headlights_range};
	// the shadow pass from the projector of the beam, with its projection. the distances which pick the details of the models
	// are measured from the real camera, as in the shadows of the sun
	m_renderpass = m_colorpass;
	m_renderpass.draw_mode = rendermode::shadows;
	m_renderpass.draw_range = reach;
	m_renderpass.draw_stats = {};
	auto &camera{m_renderpass.pass_camera};
	camera.position() = projector;
	camera.modelview() = glm::mat4{glm::dmat4{glm::dmat3{rotation}} * glm::translate(glm::dmat4{1.0}, -projector)};
	camera.projection() = projection;
	// the projection goes on to the infinity, what the shadows are cast by ends at their reach
	camera.update_frustum(glm::perspective(fieldofview, 1.f, nearplane, reach));
	setup_matrices();
	auto const colorprojection{scene_ubs.projection};
	scene_ubs.projection = projection;
	scene_ubo->update(scene_ubs);

	m_localshadows.begin_headlight(Slot, projector, rotation, projection, Light.direction, m_framestamp);
	m_shadowskipvehicle = Light.owner;
	// the instances culled on the gpu, each slot with a culling of its own
	m_lightshadowcull = Light_Shadow_Cull(m_headlightcull, Slot, Global.gfx_gpucull_shadows_headlights);
	Render(simulation::Region);
	m_lightshadowcull = nullptr;
	if ((Cab != nullptr) && (Cab != Light.owner) && (glm::length(Cab->GetPosition() - projector) <= reach + Cab->radius()))
	{
		Render_cab(Cab, 0.0f, false);
	}
	m_shadowskipvehicle = nullptr;
	m_localshadows.end_headlight(static_cast<std::uint32_t>(m_renderpass.draw_stats.drawcalls));
	scene_ubs.projection = colorprojection;
	scene_ubo->update(scene_ubs);
	m_renderpass = m_colorpass;
	glPolygonMode(GL_FRONT_AND_BACK, Global.bWireFrame ? GL_LINE : GL_FILL);
}

opengl46_instance_cull *opengl46_renderer::Light_Shadow_Cull(std::vector<std::unique_ptr<opengl46_instance_cull>> &Culls, std::size_t const Index, bool const Enabled)
{
	// the same conditions as those of the other passes in Render(scene::basic_region *)
	if ((Global.gfx_gpucull == 0) || (false == Enabled) || (m_current_viewport == nullptr) || (false == m_current_viewport->main))
		return nullptr;
	if (Culls.size() <= Index)
		Culls.resize(Index + 1);
	if (!Culls[Index])
	{
		// the buffers of its own, the records and the programs of the main view
		Culls[Index] = std::make_unique<opengl46_instance_cull>();
		Culls[Index]->init(&m_instancecull);
	}
	return Culls[Index].get();
}

std::string opengl46_renderer::Light_Shadow_Cull_Info(bool const Times)
{
	std::string text;
	if (Global.gfx_gpucull == 0)
		return text;
	// the maps are drawn now and then, the numbers are of the last drawing of each
	auto const add = [&](std::vector<std::unique_ptr<opengl46_instance_cull>> &Culls, bool const Enabled, std::string const &Label) {
		if (false == Enabled)
			return;
		for (std::size_t idx = 0; idx < Culls.size(); ++idx)
		{
			if (!Culls[idx] || (false == Culls[idx]->ran()))
				continue;
			if (Times)
			{
				text += Culls[idx]->info_times("instance cull " + Label + " " + std::to_string(idx + 1));
			}
			else
			{
				Culls[idx]->collect();
				text += Culls[idx]->info("gpu cull " + Label + " " + std::to_string(idx + 1), false);
			}
		}
	};
	add(m_headlightcull, Global.gfx_gpucull_shadows_headlights, "headlight shadows");
	add(m_lampcull, Global.gfx_gpucull_shadows_lamps, "lamp shadows, scenery");
	return text;
}

void opengl46_renderer::Update_Clusters(viewport_config &vp, glm::ivec2 const &Targetsize)
{
	glDebug("Update_Clusters");

	auto &grid{*vp.clusters};
	grid.bind();
	// the boxes of the clusters depend only on the projection and the depth range, so they are rebuilt only when those change
	if (grid.update(OpenGLMatrices.data(GL_PROJECTION), m_renderpass.znear, m_renderpass.zfar, glm::vec2{Targetsize}))
	{
		m_clusterstimer.begin();
		grid.build(*m_cluster_build_shader);
		m_clusterstimer.end();
	}
}

void opengl46_renderer::Cull_Lights(viewport_config &vp)
{
	glDebug("Cull_Lights");

	auto &grid{*vp.clusters};
	// without a grid (orthographic view) there is nothing to cull the lights into
	if (!grid.valid())
		return;

	m_cullstimer.begin();
	grid.cull(*m_cluster_cull_shader);
	m_cullstimer.end();
}

// the temporal anti-aliasing of the main view: the scene of the frame (main_tex, with its velocities and depth) blended into the
// history of the last frames (postfx_taa.frag), to the image of the history of this frame
void opengl46_renderer::Render_taa(viewport_config &vp)
{
	auto const target{m_taa.target()};
	// the near plane gives the distance of the pixels from their depth
	model_ubs.param[0] = glm::vec4{m_taa.history_valid() ? 1.f : 0.f, m_colorpass.znear, 0.f, 0.f};
	model_ubs.future = m_taa.reprojection();
	model_ubo->update(model_ubs);
	glViewport(0, 0, vp.width, vp.height);
	// the alpha of the history is the lock of the thin things, written as it is (the blending is on after the translucent parts)
	glDisable(GL_BLEND);
	m_pfx_taa->apply({vp.main_tex.get(), vp.main_texv.get(), vp.main_texd.get(), vp.taa_tex[1 - target].get()}, vp.taa_fb[target].get());
	if (m_blendingenabled)
		glEnable(GL_BLEND);
}

bool opengl46_renderer::Taa_View(viewport_config const &vp) const
{
	return Global.gfx_taa() && vp.main && (vp.taa_tex[0] != nullptr) && (m_pfx_taa != nullptr);
}

// the screen space reflections of the main view trace the depth of the scene drawn before the colour and work out the distance of
// a surface from its depth the way a perspective projection with the far plane at infinity gives it, as the contact shadows do:
// not in the orthographic view of the editor
bool opengl46_renderer::Ssr_View(viewport_config const &vp) const
{
	if ((Global.gfx_ssr <= 0) || (false == vp.main) || Global.bWireFrame || Global.gfx_skippipeline || (vp.msaa_fb == nullptr))
		return false;
	if ((vp.proj_type == viewport_config::vr_left) || (vp.proj_type == viewport_config::vr_right))
		return false;
	auto const &projection{m_renderpass.pass_camera.projection()};
	return (projection[2][3] != 0.f) && (projection[2][2] == 0.f) && (projection[3][2] > 0.f);
}

void opengl46_renderer::Setup_Screen_Reflections(viewport_config &vp, bool const Temporal)
{
	if (false == Ssr_History_Target(vp))
		return;
	if (false == m_contactshadows)
	{
		// the depth resolved apart from the buffer the colour pass draws to, as for the contact shadows
		if (false == Resolved_Depth_Target(vp))
			return;
		vp.msaa_fb->bind();
		auto const scissor{::glIsEnabled(GL_SCISSOR_TEST) == GL_TRUE};
		if (scissor)
			::glDisable(GL_SCISSOR_TEST);
		::glBlitNamedFramebuffer(*vp.msaa_fb, *vp.clusters_depth_fb, 0, 0, vp.width, vp.height, 0, 0, vp.width, vp.height, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
		if (scissor)
			::glEnable(GL_SCISSOR_TEST);
		::glBindTextureUnit(gl::SCENE_DEPTH_TEX, vp.clusters_depth_tex->id);
	}
	::glBindTextureUnit(gl::SSR_HISTORY_TEX, vp.ssr_tex->id);
	m_ssr.setup(scene_ubs, gfx::screen_reflections::steps(Global.gfx_ssr), false, 0.f, Temporal);
	scene_ubo->update(scene_ubs);
	m_ssractive = true;
}

// the finished image of the frame, before the motion blur, copied to the history the reflections of the next frame take their colour
// from, with its mipmaps for the rough surfaces
void opengl46_renderer::Store_Ssr_History(viewport_config &vp, gl::framebuffer &Source, GLenum const Attachment)
{
	if (false == Ssr_History_Target(vp))
		return;
	m_ssrhistorytimer.begin();
	auto const scissor{::glIsEnabled(GL_SCISSOR_TEST) == GL_TRUE};
	if (scissor)
		::glDisable(GL_SCISSOR_TEST);
	Source.blit_to(vp.ssr_fb.get(), vp.width, vp.height, GL_COLOR_BUFFER_BIT, Attachment);
	if (scissor)
		::glEnable(GL_SCISSOR_TEST);
	::glGenerateTextureMipmap(vp.ssr_tex->id);
	m_ssrhistorytimer.end();
	m_ssr.end_frame();
}

bool opengl46_renderer::Ssr_History_Target(viewport_config &vp)
{
	if (vp.ssr_fb)
		return true;
	// a colour of three floats without a sign in 32 bits: half the memory of rgba16f, and the image of the scene has no alpha
	auto texture{std::make_unique<opengl_texture>()};
	texture->alloc_rendertarget(GL_R11F_G11F_B10F, GL_RGB, vp.width, vp.height, 1, 1, GL_CLAMP_TO_EDGE);
	auto const levels{static_cast<int>(std::floor(std::log2(static_cast<float>(std::max({vp.width, vp.height, 1}))))) + 1};
	// the levels below the first are made by glGenerateTextureMipmap() with each copy
	::glTextureParameteri(texture->id, GL_TEXTURE_MAX_LEVEL, levels - 1);
	::glTextureParameteri(texture->id, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	::glGenerateTextureMipmap(texture->id);
	auto framebuffer{std::make_unique<gl::framebuffer>()};
	framebuffer->attach(*texture, GL_COLOR_ATTACHMENT0);
	if (false == framebuffer->is_complete())
	{
		ErrorLog("reflection history framebuffer setup failed, no screen space reflections");
		Global.gfx_ssr = 0;
		return false;
	}
	gl::framebuffer::unbind();
	vp.ssr_tex = std::move(texture);
	vp.ssr_fb = std::move(framebuffer);
	vp.ssr_levels = levels;
	m_ssr.reset();
	WriteLog("screen space reflections: history " + std::to_string(vp.width) + " x " + std::to_string(vp.height) + ", " + std::to_string(levels) + " levels");
	return true;
}

void opengl46_renderer::Render_ambient_occlusion(viewport_config &vp, gl::framebuffer &Target, int const Mode)
{
	glDebug("ambient occlusion");
	// the projection the depth was drawn with
	auto const &projection{m_colorpass.pass_camera.projection()};
	if (Mode == 1)
	{
		// the older ssao, which reads the projection from the scene uniforms
		glViewport(0, 0, vp.ssao_tex->get_width(), vp.ssao_tex->get_height());
		m_pfx_ssao->apply({vp.main_texd.get(), m_ssao_noise_tex.get()}, vp.ssao_fb.get());
		m_pfx_ssao_blur->apply({vp.ssao_tex.get()}, vp.ssao_blur_fb.get());
	}
	else
	{
		m_ao.compute(*vp.main_texd, *vp.ssao_tex, *vp.ssao_blur_tex, projection);
	}
	glViewport(0, 0, vp.width, vp.height);
	m_ao.apply(Mode, *vp.main_texd, *vp.ssao_blur_tex, Target, projection);
}

bool opengl46_renderer::Resolved_Depth_Target(viewport_config &vp)
{
	if (vp.clusters_depth_fb)
		return true;

	auto texture{std::make_unique<opengl_texture>()};
	texture->alloc_rendertarget(Global.gfx_format_depth, GL_DEPTH_COMPONENT, vp.width, vp.height, 1, 1, GL_CLAMP_TO_EDGE);
	// the values themselves are read, texel by texel
	glTextureParameteri(texture->id, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTextureParameteri(texture->id, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	auto framebuffer{std::make_unique<gl::framebuffer>()};
	framebuffer->attach(*texture, GL_DEPTH_ATTACHMENT);
	// depth only
	framebuffer->setup_drawing(0);
	if (!framebuffer->is_complete())
	{
		ErrorLog("resolved depth framebuffer setup failed");
		return false;
	}
	vp.clusters_depth_tex = std::move(texture);
	vp.clusters_depth_fb = std::move(framebuffer);
	return true;
}

glm::vec3 opengl46_renderer::Shadow_Light_Vector() const
{
	return gfx::shadow_fit::light_vector(m_sunlight.direction);
}

glm::mat3 opengl46_renderer::Shadow_Light_Rotation() const
{
	return gfx::shadow_fit::light_rotation(m_sunlight.direction);
}

std::array<float, gl::MAX_CASCADES + 1> opengl46_renderer::Shadow_Fixed_Splits() const
{
	return gfx::shadow_fit::fixed_splits();
}

char const *opengl46_renderer::Shadow_Fit_Unavailable(viewport_config const &Viewport) const
{
	auto const shadowviews{std::count_if(m_viewports.begin(), m_viewports.end(), [](auto const &View) { return View->shadow; })};
	return gfx::shadow_fit::unavailable((false == Global.gfx_skippipeline) && (Viewport.msaa_fb != nullptr), Viewport.shadow, static_cast<std::size_t>(shadowviews));
}

bool opengl46_renderer::Shadow_Fit(viewport_config const &Viewport, std::array<float, gl::MAX_CASCADES + 1> &Splits,
                                   std::array<std::optional<std::array<glm::vec3, 8>>, gl::MAX_CASCADES> &Areas)
{
	auto const *unavailable{Global.gfx_shadow_sdsm ? Shadow_Fit_Unavailable(Viewport) : nullptr};
	if (Global.gfx_shadow_sdsm && (unavailable == nullptr))
	{
		// takes in what came back
		m_shadowfit.latest();
	}
	return m_shadowfit.fit(m_framestamp, m_colorpass.pass_camera, unavailable, Splits, Areas, m_shadowfitstate);
}

void opengl46_renderer::Render_cluster_heatmap(viewport_config &vp, gl::framebuffer *Target, glm::ivec2 const &Targetsize)
{
	glDebug("Render_cluster_heatmap");

	if (!vp.msaa_fb)
		return;

	if (!Resolved_Depth_Target(vp))
	{
		Global.gfx_lights_clusterheatmap = false;
		return;
	}

	// the depth of the scene is still in the msaa buffer, the post processing worked on its colour
	vp.msaa_fb->blit_to(vp.clusters_depth_fb.get(), vp.width, vp.height, GL_DEPTH_BUFFER_BIT, GL_DEPTH_ATTACHMENT);

	if (Target)
		Target->bind();
	else
		gl::framebuffer::unbind();
	glViewport(0, 0, Targetsize.x, Targetsize.y);

	vp.clusters->bind();
	m_cluster_heatmap_shader->bind();
	m_empty_vao->bind();
	vp.clusters_depth_tex->bind(0);

	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	auto const blending{glIsEnabled(GL_BLEND) == GL_TRUE};
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	if (!blending)
		glDisable(GL_BLEND);
	vp.clusters_depth_tex->unbind(0);
}

bool opengl46_renderer::Init_caps()
{
	std::string gl_renderer((char *)glGetString(GL_RENDERER));
	std::string gl_vendor((char *)glGetString(GL_VENDOR));
	std::string gl_version((char *)glGetString(GL_VERSION));

	crashreport_add_info("gl_renderer", gl_renderer);
	crashreport_add_info("gl_vendor", gl_vendor);
	crashreport_add_info("gl_version", gl_version);
	crashreport_add_info("gl_use_vao", gl::vao::use_vao ? "yes" : "no");
	crashreport_add_info("gfx.skippipeline", Global.gfx_skippipeline ? "yes" : "no");

	WriteLog("MaSzyna OpenGL Renderer");
	WriteLog("Renderer: " + gl_renderer);
	WriteLog("Vendor: " + gl_vendor);
	WriteLog("GL version: " + gl_version);

	WriteLog("--------");

	GLint extCount = 0;
	glGetIntegerv(GL_NUM_EXTENSIONS, &extCount);

	WriteLog("Supported extensions:");
	for (int i = 0; i < extCount; i++)
	{
		const char *ext = (const char *)glGetStringi(GL_EXTENSIONS, i);
		WriteLog(ext);
	}
	WriteLog("--------");

	crashreport_add_info("gl_family", "desktop");

	if (!GLAD_GL_VERSION_4_6)
	{
		ErrorLog("requires OpenGL >= 4.6!");
		return false;
	}

	if (!GLAD_GL_EXT_texture_sRGB)
		ErrorLog("EXT_texture_sRGB not supported!");

	if (!GLAD_GL_EXT_texture_compression_s3tc)
		ErrorLog("EXT_texture_compression_s3tc not supported!");

	WriteLog("--------");

	// ograniczenie maksymalnego rozmiaru tekstur - parametr dla skalowania tekstur
	{
		GLint texturesize;
		::glGetIntegerv(GL_MAX_TEXTURE_SIZE, &texturesize);
		Global.iMaxTextureSize = std::min(Global.iMaxTextureSize, texturesize);
        Global.iMaxCabTextureSize = std::min( Global.iMaxCabTextureSize, texturesize );
        WriteLog( "texture sizes capped at " + std::to_string(Global.iMaxTextureSize) + "p (" + std::to_string( Global.iMaxCabTextureSize ) + "p for cab textures)" );
        Global.CurrentMaxTextureSize = Global.iMaxTextureSize;
        m_shadowbuffersize = Global.shadowtune.map_size;
		m_shadowbuffersize = std::min(m_shadowbuffersize, texturesize);
		WriteLog("shadows map size capped at " + std::to_string(m_shadowbuffersize) + "p");
	}

	if (Global.iMultisampling)
	{
		WriteLog("using multisampling x" + std::to_string(1 << Global.iMultisampling));
	}

	// the image of the main window: a part of the window (gfx.buffer.percent), stretched to it
	if ((Global.gfx_framebuffer_width == -1) || (Global.gfx_framebuffer_height == -1))
	{
		auto const size{Global.gfx_buffer_size()};
		Global.gfx_framebuffer_width = size.x;
		Global.gfx_framebuffer_height = size.y;
	}

	WriteLog("main window size: " + std::to_string(Global.fb_size.x) + "x" + std::to_string(Global.fb_size.y) + ", drawn at " + std::to_string(Global.gfx_framebuffer_width) + "x" + std::to_string(Global.gfx_framebuffer_height));

	return true;
}


std::unique_ptr<gfx_renderer> opengl46_renderer::create_func()
{
    return std::unique_ptr<gfx_renderer>(new opengl46_renderer());
}

bool opengl46_renderer::renderer_register = gfx_renderer_factory::get_instance()->register_backend("modern", opengl46_renderer::backend_desc());

bool opengl46_renderer::opengl46_imgui_renderer::Init()
{
	crashreport_add_info("imgui_ver", "gl3");
	return ImGui_ImplOpenGL3_Init("#version 460 core");
}

void opengl46_renderer::opengl46_imgui_renderer::Shutdown()
{
	ImGui_ImplOpenGL3_Shutdown();
}

void opengl46_renderer::opengl46_imgui_renderer::BeginFrame()
{
	ImGui_ImplOpenGL3_NewFrame();
}

void opengl46_renderer::opengl46_imgui_renderer::Render()
{
	gl::buffer::unbind(gl::buffer::ARRAY_BUFFER);
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

std::uint64_t opengl46_renderer::opengl46_imgui_renderer::Create_Image(std::uint8_t const *Rgba, int const Width, int const Height, image_options const &Options)
{
	return Update_Image(0, Rgba, Width, Height, Options);
}

std::uint64_t opengl46_renderer::opengl46_imgui_renderer::Update_Image(std::uint64_t const Image, std::uint8_t const *Rgba, int const Width, int const Height, image_options const &Options)
{
	// the ui colours are made linear in its shaders, an srgb image is too, by sampling it as srgb. the bindings the renderer
	// keeps track of are put back as they were
	GLint texture{0}, unpack{0}, alignment{4};
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture);
	glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &unpack);
	glGetIntegerv(GL_UNPACK_ALIGNMENT, &alignment);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	auto image{static_cast<GLuint>(Image)};
	if (image == 0)
		glGenTextures(1, &image);
	glBindTexture(GL_TEXTURE_2D, image);
	auto const filter{Options.nearest ? GL_NEAREST : GL_LINEAR};
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, Options.mipmaps ? (Options.nearest ? GL_NEAREST_MIPMAP_NEAREST : GL_LINEAR_MIPMAP_LINEAR) : filter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexImage2D(GL_TEXTURE_2D, 0, Options.srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8, Width, Height, 0, GL_RGBA, GL_UNSIGNED_BYTE, Rgba);
	if (Options.mipmaps)
		glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture));
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, static_cast<GLuint>(unpack));
	glPixelStorei(GL_UNPACK_ALIGNMENT, alignment);
	return image;
}

void opengl46_renderer::opengl46_imgui_renderer::Release_Image(std::uint64_t const Image)
{
	auto const image{static_cast<GLuint>(Image)};
	glDeleteTextures(1, &image);
}

