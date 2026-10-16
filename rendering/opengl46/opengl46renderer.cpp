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
int constexpr EU07_REFLECTIONFIDELITYOFFSET { 250 }; // artificial increase of range for reflection pass detail reduction

namespace {

float
editor_track_view_range(float const Range, bool const Apply)
{
	if (false == Apply || false == Global.editor_tracks)
		return Range;
	float extra = 50000.0f;
	if (Global.EditorOrtho)
		extra = Global.EditorOrthoExtent * 3.0f;
	return Range > extra ? Range : extra;
}

bool
editor_far_tracks_only(bool const Apply, scene::basic_cell const *Cell, glm::dvec3 const &Camera, double const SceneryLimit)
{
	if (false == Apply)
		return false;
	double nearest = glm::length(Cell->m_area.center - Camera) - static_cast<double>(Cell->m_area.radius);
	if (nearest < 0.0)
		nearest = 0.0;
	return nearest > SceneryLimit;
}

// returns material assigned to the submodel, resolving replacable skins. null_handle if no skin set is bound
material_handle
submodel_material( TSubModel const *Submodel ) {

    if( Submodel->m_material >= 0 ) {
        return Submodel->m_material;
    }
    return (
        TSubModel::ReplacableSkinId != nullptr ?
            TSubModel::ReplacableSkinId[ -Submodel->m_material ] :
            null_handle );
}

} // namespace

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

	WriteLog("preparing renderer..");

	m_window = Window;

	gl::glsl_common_setup();

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

	scene_ubo = std::make_unique<gl::ubo>(sizeof(gl::scene_ubs), 0);
	model_ubo = std::make_unique<gl::ubo>(sizeof(gl::model_ubs), 1, GL_STREAM_DRAW);
	light_ssbo = std::make_unique<gl::ssbo>(sizeof(gl::light_data), gl::LIGHT_SSBO_BINDING);
	light_ssbo->bind_base();
	for (auto &buffer : m_instancebuffers)
	{
		// grows when a frame needs more
		buffer = std::make_unique<gl::ssbo>(static_cast<GLsizeiptr>(16384 * sizeof(glm::mat4)), gl::INSTANCE_SSBO_BINDING);
	}

	try
	{
		m_sky.init();
		m_instancecull.init();
		m_hiz.init();
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

	// element 0 of the instance buffers is identity. This is the matrix sampled by
	// non-instanced draws, so existing rendering paths
	// continue to multiply effective_modelview = identity * modelview = modelview.
	for (auto &buffer : m_instancebuffers)
	{
		glm::mat4 const identity(1.0f);
		buffer->update(identity);
	}
	m_instancebuffers[m_instancebufferindex]->bind_base();

	int samples = 1 << Global.iMultisampling;
	if (samples > 1)
		glEnable(GL_MULTISAMPLE);

	m_pfx_motionblur = std::make_unique<gl::postfx>("motionblur");
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

		if (Global.gfx_postfx_chromaticaberration_enabled || Global.gfx_postfx_motionblur_enabled || Global.gfx_postfx_ao != 0)
		{
			vp.main_tex = std::make_unique<opengl_texture>();
			vp.main_tex->alloc_rendertarget(Global.gfx_format_color, GL_RGB, vp.width, vp.height, 1, 1, GL_CLAMP_TO_EDGE);

			vp.main_texd = std::make_unique<opengl_texture>();
			vp.main_texd->alloc_rendertarget(Global.gfx_format_depth, GL_DEPTH_COMPONENT, vp.width, vp.height, 1, 1, GL_CLAMP_TO_EDGE);

			vp.main_fb = std::make_unique<gl::framebuffer>();
			vp.main_fb->attach(*vp.main_tex, GL_COLOR_ATTACHMENT0);
			vp.main_fb->attach(*vp.main_texd, GL_DEPTH_ATTACHMENT);

			if (Global.gfx_postfx_motionblur_enabled)
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

		if (Global.gfx_postfx_motionblur_enabled)
		{
			vp.msaa_rbv = std::make_unique<gl::renderbuffer>();
			vp.msaa_rbv->alloc(Global.gfx_postfx_motionblur_format, vp.width, vp.height, samples);
			vp.msaa_fb->attach(*vp.msaa_rbv, GL_COLOR_ATTACHMENT1);

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

			if (Global.gfx_postfx_motionblur_enabled)
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
	Timer::subsystem.gfx_total.start();

	std::optional<int64_t> result = m_timequery->result();
	if (result) {
		m_gllasttime = *result;
		m_timequery->begin();
	}

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
		m_sky.update(m_sunlight, m_fogcolor, simulation::Environment.skydome(), simulation::Environment.sun().getDirection(), simulation::Environment.sun().getAngle());
		// lamps of models and vehicles around the camera and the cab lamp, for the views of the frame to take into their light buffers
		if ((false == Global.gfx_skiprendering) && (simulation::Region != nullptr))
			m_locallights.gather(Global.pCamera.Pos, *simulation::Region, m_geometry, m_framestamp);
	}
	else
	{
		// a scenery is being loaded: the cells, instances and models the lights were taken from may be gone
		m_locallights.reset();
		m_localshadows.reset_lamps();
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
	// scenery lamps which get shadows in this frame. their maps hold the scenery as of the version of the instance records
	if (simulation::is_ready && (false == Global.gfx_skiprendering))
		m_localshadows.update_lamps(m_locallights.lights(), Global.pCamera.Pos, m_instancerecordsversion, m_framestamp);
	// instanced draws of the frame place their matrices in the next of the instance buffers, the one used the longest ago
	m_instancebufferindex = ( m_instancebufferindex + 1 ) % m_instancebuffers.size();
	m_instancebufferfill = 1;
	m_instancebuffers[ m_instancebufferindex ]->bind_base();

	m_renderpass.draw_mode = rendermode::none; // force setup anew
	m_renderpass.draw_stats = debug_stats();
    m_geometry.primitives_count() = 0;

    if (vr)
        vr->begin_frame();

	for (auto &viewport : m_viewports) {
		Render_pass(*viewport, rendermode::color);
	}

	glfwMakeContextCurrent(m_window);

	gl::framebuffer::unbind();

	m_current_viewport = &(*m_viewports.front());
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

    m_debugtimestext.clear();
    m_debugtimestext =
        "cpu frame total: " + to_string( Timer::subsystem.gfx_color.average() + Timer::subsystem.gfx_shadows.average() + ( Global.gfx_envmap_enabled ? Timer::subsystem.gfx_reflections.average() : 0.f ) + Timer::subsystem.gfx_swap.average(), 2 ) + " ms\n"
        + " color: " + to_string( Timer::subsystem.gfx_color.average(), 2 ) + " ms (" + std::to_string( m_cellqueue.size() ) + " sectors)\n";
    if( Global.gfx_shadowmap_enabled ) {
        m_debugtimestext +=
            " shadows: " + to_string( Timer::subsystem.gfx_shadows.average(), 2 ) + " ms\n";
    }
    if( Global.gfx_envmap_enabled ) {
        m_debugtimestext +=
            " reflections: " + to_string( Timer::subsystem.gfx_reflections.average(), 2 ) + " ms\n";
    }
    m_debugtimestext += " swap: " + to_string( Timer::subsystem.gfx_swap.average(), 2 ) + " ms\n";
    if (m_gllasttime)
        m_debugtimestext += "gpu frame total: " + to_string((double)(m_gllasttime / 1000ULL) / 1000.0, 3) + " ms\n";
    if (auto const clusterstime = m_clusterstimer.result())
        m_debugtimestext += " clusters: " + to_string(static_cast<double>(*clusterstime) / 1000000.0, 3) + " ms\n";
    if (auto const culltime = m_cullstimer.result())
        m_debugtimestext += " cull: " + to_string(static_cast<double>(*culltime) / 1000000.0, 3) + " ms\n";
    m_debugtimestext += m_sky.info_times();
    m_debugtimestext += m_exposure.info_times();
    m_debugtimestext += m_ao.info_times();
    m_debugtimestext += m_instancecull.info_times();
    if( Global.gfx_gpucull == 1 ) {
        m_debugtimestext += m_hiz.info_times();
        if( auto const prepasstime = m_prepasstimer.result() )
            m_debugtimestext += " depth prepass: " + to_string( static_cast<double>( *prepasstime ) / 1000000.0, 3 ) + " ms\n";
    }
    m_debugtimestext += m_localshadows.info_times();
    m_debugtimestext += "uilayer: " + to_string( Timer::subsystem.gfx_gui.average(), 2 ) + " ms\n";
    if( DebugModeFlag )
        m_debugtimestext += m_textures.info();

    debug_stats shadowstats;
    for( auto const &shadowpass : m_shadowpass ) {
        shadowstats += shadowpass.draw_stats;
    }

    m_debugstatstext =
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
        + " instance buffer: " + std::to_string( m_instancebufferfill - 1 ) + " matrices, "
        + std::to_string( m_instancebuffers[ m_instancebufferindex ]->size() / 1024 ) + " KiB\n"
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
        + m_instancecull.info();
    if( auto const cullstats = ( m_viewports.empty() || !( Global.gfx_lights_clustered || Global.gfx_lights_clusterheatmap ) ? std::nullopt : m_viewports.front()->clusters->stats() ) ) {
        m_debugstatstext +=
            "\ncluster lists: " + std::to_string( cullstats->index_count )
            + ", overflow " + std::to_string( cullstats->overflow_count )
            + " (peak " + std::to_string( cullstats->overflow_peak ) + ")";
    }
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
    }
    ImGui::End();

    if (ImGui::Begin("Sky and exposure"))
    {
        m_sky.draw_debug_ui();
        ImGui::Separator();
        m_exposure.draw_debug_ui();
        ImGui::Separator();
        m_ao.draw_debug_ui(!m_viewports.empty() && m_viewports.front()->ssao_tex != nullptr);
    }
    ImGui::End();

    if (ImGui::Begin("Instance culling"))
    {
        char const *modes[]{"cpu", "gpu, drawn from its result", "cpu, compared with the gpu"};
        ImGui::Combo("instance culling (gfx.gpucull)", &Global.gfx_gpucull, modes, IM_ARRAYSIZE(modes));
        if (Global.gfx_gpucull == 1)
        {
            ImGui::Checkbox("leave out instances hidden behind the depth drawn before them (gfx.gpucull.occlusion)", &Global.gfx_gpucull_occlusion);
            ImGui::Checkbox("draw the depth of the instances before their colour (gfx.depthprepass)", &Global.gfx_depthprepass);
        }
        auto const info{m_instancecull.info()};
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

        m_colorpass = m_renderpass; // cache pass data

		scene_ubs.time = Timer::GetTime();
		{
			float percipitation_intensity = glm::saturate(Global.Overcast - 1.);
			scene_ubs.rain_params.x = percipitation_intensity; // % amount of droplets
			scene_ubs.rain_params.y = glm::mix(15., 1., percipitation_intensity); // Regeneration time
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
		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
        scene_ubs.inv_view = glm::inverse( glm::mat4{ glm::mat3{ m_colorpass.pass_camera.modelview() } } );
        scene_ubo->update(scene_ubs);
		scene_ubo->bind_uniform();

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
			Render_local_shadows();
			Render_lamp_shadows();
		}

		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

		setup_drawing(false);

		glm::ivec2 target_size(vp.width, vp.height);
        if (vp.main && !vp.custom_backbuffer) // TODO: update window sizes also for extra viewports
            target_size = Global.fb_size;

		if (!Global.gfx_skippipeline)
		{
			vp.msaa_fb->bind();

			if (Global.gfx_postfx_motionblur_enabled)
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
		if (Global.pCamera.m_owner != nullptr)
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

		glDebug("render environment");

		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
		// back to the rotation of the camera after the faces of the environment map, which are drawn with their own
		scene_ubs.inv_view = glm::inverse(glm::mat4{glm::mat3{m_colorpass.pass_camera.modelview()}});
		scene_ubo->update(scene_ubs);
		Render(&simulation::Environment);

        if( Global.gfx_shadowmap_enabled )
            setup_shadow_bind_map();

		// opaque parts...
		setup_drawing(false);

		// without rain/snow we can render the cab early to limit the overdraw
		// precipitation happens when overcast is in 1-2 range
		if (!FreeFlyModeFlag && Global.Overcast <= 1.0f && Global.render_cab)
		{
			glDebug("render opaque cab");
            model_ubs.future = glm::mat4();
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

        Render_vr_models();

		// ...translucent parts
		glDebug("render translucent region");
		setup_drawing(true);
		Render_Alpha(simulation::Region);
		Render_editor_overlay();

		// particles
		Render_particles();

		// precipitation; done at end, only before cab render
		Render_precipitation();

		// cab render
		if (false == FreeFlyModeFlag && Global.render_cab)
		{
			model_ubs.future = glm::mat4();
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
                Render_Alpha( vehicle );
            }
        }

		Timer::subsystem.gfx_color.stop();

        // store draw stats
        m_colorpass.draw_stats = m_renderpass.draw_stats;

        setup_shadow_empty_map();
		setup_env_map(nullptr);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

if (!Global.gfx_skippipeline)
		{
			// the targets of the ambient occlusion are there only when it was on at the start
			auto const aomode{(vp.ssao_tex != nullptr) ? Global.gfx_postfx_ao : 0};
			if (Global.gfx_postfx_motionblur_enabled)
			{
				vp.main_fb->clear(GL_COLOR_BUFFER_BIT);
                vp.main_fb->setup_drawing(2);
				vp.msaa_fb->blit_to(vp.main_fb.get(), vp.width, vp.height, GL_COLOR_BUFFER_BIT, GL_COLOR_ATTACHMENT0);
				vp.msaa_fb->blit_to(vp.main_fb.get(), vp.width, vp.height, GL_COLOR_BUFFER_BIT, GL_COLOR_ATTACHMENT1);
				vp.msaa_fb->blit_to(vp.main_fb.get(), vp.width, vp.height, GL_DEPTH_BUFFER_BIT, GL_DEPTH_ATTACHMENT);

				// the occlusion goes into the image before the blur, which smears both alike
				if (aomode != 0)
					Render_ambient_occlusion(vp, *vp.ao_apply_fb, aomode);

				model_ubs.param[0].x = m_framerate / (1.0 / Global.gfx_postfx_motionblur_shutter);
				model_ubo->update(model_ubs);
				m_pfx_motionblur->apply({vp.main_tex.get(), vp.main_texv.get(), vp.main_texd.get()}, vp.main2_fb.get());
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
			}

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
			model_ubo->update(model_ubs);
			std::vector<opengl_texture *> tonemapinput{vp.main2_tex.get()};
			if (bloomtexture != nullptr)
				tonemapinput.push_back(bloomtexture);

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
		Timer::subsystem.gfx_gui.start();

        if (vp.main) {
            // clear state for ui
            gl::program::unbind();
			draw_debug_ui();
			Application.render_ui();
		}

        Timer::subsystem.gfx_gui.stop();

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
		float csmstageboundaries[] = {0.0f,
		                               Global.shadowtune.range / 32,
		                              Global.shadowtune.range / 8,
		                              Global.shadowtune.range};
        if( Global.shadowtune.map_size > 2048 ) {
            // increase coverage if our shadow map is large enough to produce decent results
			csmstageboundaries[1] *= Global.shadowtune.map_size / 2048;
			csmstageboundaries[2] *= Global.shadowtune.map_size / 2048;
			csmstageboundaries[3] *= Global.shadowtune.map_size / 2048;
        }


        for( auto idx = 0; idx < m_shadowpass.size(); ++idx ) {

            m_shadow_fb->attach( *m_shadow_tex, GL_DEPTH_ATTACHMENT, idx );
            m_shadow_fb->clear( GL_DEPTH_BUFFER_BIT );

            setup_pass( vp, m_renderpass, rendermode::shadows, csmstageboundaries[ idx ], csmstageboundaries[ idx + 1 ] );
            m_shadowpass[ idx ] = m_renderpass; // store pass config for reference in other stages

            setup_matrices();
            scene_ubs.projection = OpenGLMatrices.data( GL_PROJECTION );
            auto const csmstagezfar {
                (csmstageboundaries[ idx + 1 ])
              + ( m_shadowpass.size() - idx ) }; // we can fairly safely add some extra padding in the early stages
            scene_ubs.cascade_end[ idx ] = csmstagezfar * csmstagezfar; // store squared to allow semi-optimized length2 comparisons in the shader
            scene_ubo->update( scene_ubs );

            if( m_shadowcolor != colors::white ) {
                Render( simulation::Region );
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

		setup_matrices();
		// the light buffer holds what the last color pass left in it, in the view space of its camera, while the face is drawn
		// in its own. the lights go in anew; the map is a cache of the surroundings which lasts for seconds, so the headlights
		// of the vehicles around are left out of it and the cost of the pass does not depend on how many of them are on
		Update_Lights(simulation::Lights, false);

		// render
        // environment...
		setup_drawing(true);

        setup_shadow_empty_map();
		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
		// world space directions of the shading of the face (the sky, the directional ambient light)
		scene_ubs.inv_view = glm::inverse(glm::mat4{glm::mat3{m_renderpass.pass_camera.modelview()}});
		scene_ubo->update(scene_ubs);
		Render(&simulation::Environment);
		// opaque parts...
		setup_drawing(false);
		setup_shadow_bind_map();
		scene_ubs.projection = OpenGLMatrices.data(GL_PROJECTION);
		scene_ubo->update(scene_ubs);
		Render(simulation::Region);

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


bool opengl46_renderer::Render_interior( bool const Alpha ) {
    // TODO: early frustum based cull, camera might be pointing elsewhere
    std::vector< std::pair<float, TDynamicObject *> > dynamics;
    auto *dynamic { simulation::Train->Dynamic() };
    // draw interiors of the occupied vehicle, and all vehicles behind it with permanent coupling, in case they're open-ended
    while( dynamic != nullptr ) {

        glm::dvec3 const originoffset { dynamic->vPosition - m_renderpass.pass_camera.position() };
        float const squaredistance{ glm::length2( glm::vec3{ originoffset } / Global.ZoomFactor ) };
        dynamics.emplace_back( squaredistance, dynamic );
        dynamic = dynamic->Next( coupling::permanent );
    }
    // draw also interiors of permanently coupled vehicles in front, if there's any
    dynamic = simulation::Train->Dynamic()->Prev( coupling::permanent );
    while( dynamic != nullptr ) {

        glm::dvec3 const originoffset { dynamic->vPosition - m_renderpass.pass_camera.position() };
        float const squaredistance{ glm::length2( glm::vec3{ originoffset } / Global.ZoomFactor ) };
        dynamics.emplace_back( squaredistance, dynamic );
        dynamic = dynamic->Prev( coupling::permanent );
    }
    if( Alpha ) {
        std::sort(
            std::begin( dynamics ), std::end( dynamics ),
            []( std::pair<float, TDynamicObject *> const &Left, std::pair<float, TDynamicObject *> const &Right ) {
               return ( Left.first ) > ( Right.first ); } );
    }
    for( auto &dynamic : dynamics ) {
        Render_lowpoly( dynamic.second, dynamic.first, true, Alpha );
    }

    return true;
}

bool opengl46_renderer::Render_lowpoly( TDynamicObject *Dynamic, float const Squaredistance, bool const Setup, bool const Alpha ) {

    if( Dynamic->mdLowPolyInt == nullptr ) { return false; }

    // low poly interior
    if( Setup ) {

        TSubModel::iInstance = reinterpret_cast<std::uintptr_t>( Dynamic ); //żeby nie robić cudzych animacji
        TSubModel::iVariantSeed = Dynamic->variant_seed();
        glm::dvec3 const originoffset{ Dynamic->vPosition - m_renderpass.pass_camera.position() };

        Dynamic->ABuLittleUpdate( Squaredistance ); // ustawianie zmiennych submodeli dla wspólnego modelu

        glm::mat4 mv = OpenGLMatrices.data( GL_MODELVIEW );

        ::glPushMatrix();
        ::glTranslated( originoffset.x, originoffset.y, originoffset.z );
		::glMultMatrixd( glm::value_ptr(Dynamic->mMatrix) );
    }
    // HACK: reduce light level for vehicle interior if there's strong global lighting source
    if( false == Alpha ) {
/*
        auto const luminance { static_cast<float>( std::min(
            ( Dynamic->fShade > 0.0 ? Dynamic->fShade * Global.fLuminance : Global.fLuminance ),
            Global.fLuminance - 0.5 * ( std::max( 0.3, Global.fLuminance - Global.Overcast ) ) ) ) };
*/
        auto const luminance{ static_cast<float>(
            0.5
            * ( std::max(
                0.3,
                ( Global.fLuminance - Global.Overcast )
                * ( Dynamic->fShade > 0.0 ?
                    Dynamic->fShade :
                    1.0 ) ) ) ) };
        setup_sunlight_intensity(
            std::clamp( (
                Dynamic->fShade > 0.f ?
                    Dynamic->fShade :
                    1.f )
                - luminance,
                0.f, 1.f ) );
        Render( Dynamic->mdLowPolyInt, Dynamic->Material(), Squaredistance );
        // HACK: if the model has low poly interior, we presume the load is placed inside and also affected by reduced light level
        if( Dynamic->mdLoad ) {
            // renderowanie nieprzezroczystego ładunku
            Render( Dynamic->mdLoad, Dynamic->Material(), Squaredistance, { 0.f, Dynamic->LoadOffset, 0.f }, {} );
        }
        setup_sunlight_intensity( Dynamic->fShade > 0.f ? Dynamic->fShade : 1.f );
    }
    else {
//        Render_Alpha( Dynamic->mdLowPolyInt, Dynamic->Material(), Squaredistance );
        // HACK: some models have windows included as part of the main model instead of lowpoly
        if( Dynamic->mdModel ) {
            // main model
            Render_Alpha( Dynamic->mdModel, Dynamic->Material(), Squaredistance );
        }
    }

    if( Setup ) {

        if( Dynamic->fShade > 0.0f ) {
            // restore regular light level
            setup_sunlight_intensity();
        }

        ::glPopMatrix();

        // TODO: check if this reset is needed. In theory each object should render all parts based on its own instance data anyway?
        if( Dynamic->btnOn ) {
            Dynamic->TurnOff(); // przywrócenie domyślnych pozycji submodeli
        }
    }

    return true;
}

bool opengl46_renderer::Render_coupler_adapter( TDynamicObject *Dynamic, float const Squaredistance, int const End, bool const Alpha ) {

    if( Dynamic->m_coupleradapters[ End ] == nullptr ) { return false; }

    auto const position { glm::dvec3 {
        0.f,
        Dynamic->MoverParameters->Couplers[ End ].adapter_height,
        ( Dynamic->MoverParameters->Couplers[ End ].adapter_length + Dynamic->MoverParameters->Dim.L * 0.5 ) * ( End == end::front ? 1 : -1 ) } };

    auto const angle { glm::vec3{
        0,
        ( End == end::front ? 0 : 180 ),
        0 } };

    if( Alpha ) {
        Render_Alpha( Dynamic->m_coupleradapters[ End ], Dynamic->Material(), Squaredistance, position, angle );
    }
    else {
        Render( Dynamic->m_coupleradapters[ End ], Dynamic->Material(), Squaredistance, position, angle );
    }

    return true;
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
	// an update draws the scene six times, which is too much for a single frame. its cost is spread
	// by drawing one face of the map per frame, all of them as seen from the place the update began at.
	// the first update is done in one go, as there's nothing to show in the meantime
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

		// ...modelview matrix: determine the centre of frustum chunk in world space...
		glm::vec3 frustumchunkmin, frustumchunkmax;
		bounding_box(frustumchunkmin, frustumchunkmax, std::begin(frustumchunkshapepoints), std::end(frustumchunkshapepoints));
		auto const frustumchunkcentre = (frustumchunkmin + frustumchunkmax) * 0.5f;
		// ...cap the vertical angle to keep shadows from getting too long...
		auto const lightvector = glm::normalize(glm::vec3{m_sunlight.direction.x, std::min(m_sunlight.direction.y, Global.gfx_shadow_angle_min), m_sunlight.direction.z});
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
		auto const quantizationstep{std::min(frustumchunkdepth, 50.f)};
		frustumchunkmin = quantizationstep * glm::floor(frustumchunkmin * (1.f / quantizationstep));
		frustumchunkmax = quantizationstep * glm::ceil(frustumchunkmax * (1.f / quantizationstep));
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

    // stars
	if (Environment->m_stars.m_stars != nullptr && Global.fFogEnd > 500.f)
	{
		// setup
		::glPushMatrix();
		::glRotatef(Environment->m_stars.m_latitude, 1.f, 0.f, 0.f); // ustawienie osi OY na północ
		::glRotatef(-std::fmod((float)Global.fTimeAngleDeg, 360.f), 0.f, 1.f, 0.f); // obrót dobowy osi OX

        // render
        Render(Environment->m_stars.m_stars, nullptr, 1.0);

		// post-render cleanup
		::glPopMatrix();
	}

	// celestial bodies
	m_celestial_shader->bind();
	m_empty_vao->bind();

	auto const &modelview = OpenGLMatrices.data(GL_MODELVIEW);

    auto const fogfactor{std::clamp(Global.fFogEnd / 2000.f, 0.f, 1.f)}; // stronger fog reduces opacity of the celestial bodies
	float const duskfactor = 1.0f - std::clamp(std::abs(Environment->m_sun.getAngle()), 0.0f, 12.0f) / 12.0f;
	glm::vec3 suncolor = glm::mix(glm::vec3(1.0f, 242.0f / 255.0f, 231.0f / 255.0f), glm::vec3(235.0f / 255.0f, 140.0f / 255.0f, 36.0f / 255.0f), duskfactor);
	suncolor = m_sky.sun_disc_color(suncolor);

	// sun
	{
		Bind_Texture(0, m_suntexture);
		glm::vec4 color(suncolor.x, suncolor.y, suncolor.z, std::clamp(1.5f - Global.Overcast, 0.f, 1.f) * fogfactor);
		auto const sunvector = Environment->m_sun.getDirection();

        /*float const size = std::lerp( // TODO: expose distance/scale factor from the moon object
            0.0325f,
            0.0275f,
            std::clamp( Environment->m_sun.getAngle(), 0.f, 90.f ) / 90.f );*/
		model_ubs.param[0] = color;
        model_ubs.param[1] = glm::vec4(glm::vec3(modelview * glm::vec4(sunvector, 1.0f)), 0.00463f /* size */);
		model_ubs.param[2] = glm::vec4(0.0f, 1.0f, 1.0f, 0.0f);
		model_ubo->update(model_ubs);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	}
	// moon
	{
		Bind_Texture(0, m_moontexture);
    	glm::vec3 mooncolor(1.0f, 0.9490196f, 0.9058824f);
		glm::vec4 color(mooncolor.r, mooncolor.g, mooncolor.b,
		                // fade the moon if it's near the sun in the sky, especially during the day
		                std::max<float>(0.f, 1.0 - 0.5 * Global.fLuminance - 0.65 * std::max(0.f, glm::dot(Environment->m_sun.getDirection(), Environment->m_moon.getDirection()))) * fogfactor);

		auto const moonvector = Environment->m_moon.getDirection();

		// choose the moon appearance variant, based on current moon phase
		// NOTE: implementation specific, 8 variants are laid out in 3x3 arrangement
		// from new moon onwards, top left to right bottom (last spot is left for future use, if any)
		auto const moonphase = Environment->m_moon.getPhase();
		float moonu, moonv;
		if (moonphase < 1.84566f)
		{
			moonv = 1.0f - 0.0f;
			moonu = 0.0f;
		}
		else if (moonphase < 5.53699f)
		{
			moonv = 1.0f - 0.0f;
			moonu = 0.333f;
		}
		else if (moonphase < 9.22831f)
		{
			moonv = 1.0f - 0.0f;
			moonu = 0.667f;
		}
		else if (moonphase < 12.91963f)
		{
			moonv = 1.0f - 0.333f;
			moonu = 0.0f;
		}
		else if (moonphase < 16.61096f)
		{
			moonv = 1.0f - 0.333f;
			moonu = 0.333f;
		}
		else if (moonphase < 20.30228f)
		{
			moonv = 1.0f - 0.333f;
			moonu = 0.667f;
		}
		else if (moonphase < 23.99361f)
		{
			moonv = 1.0f - 0.667f;
			moonu = 0.0f;
		}
		else if (moonphase < 27.68493f)
		{
			moonv = 1.0f - 0.667f;
			moonu = 0.333f;
		}
		else if (moonphase == 50) //9th slot used for easter egg
		{
			moonv = 1.0f - 0.667f;
			moonu = 0.66f;
		}
		else
		{
			moonv = 1.0f - 0.0f;
			moonu = 0.0f;
		}

        /*
        float const size = std::lerp( // TODO: expose distance/scale factor from the moon object
            0.0160f,
            0.0135f,
            std::clamp( Environment->m_moon.getAngle(), 0.f, 90.f ) / 90.f );*/

		model_ubs.param[0] = color;
        model_ubs.param[1] = glm::vec4(glm::vec3(modelview * glm::vec4(moonvector, 1.0f)), 0.00451f /* size */);
		model_ubs.param[2] = glm::vec4(moonu, moonv, 0.333f, 0.0f);
		model_ubo->update(model_ubs);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	}

    ::glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  	// clouds
	if (Environment->m_clouds.mdCloud)
	{
		// setup
		auto const color {
			glm::clamp(
			    ( glm::vec3 { Global.DayLight.ambient }
                + 0.35f * glm::vec3{ Global.DayLight.diffuse } ) * simulation::Environment.light_intensity()
                * 0.5f,
			    glm::vec3{ 0.f }, glm::vec3{ 1.f } ) };

		// write cloud color into material
		TSubModel *mdl = Environment->m_clouds.mdCloud->Root;
		if (mdl->m_material != null_handle)
			m_materials.material(mdl->m_material).params[0] = glm::vec4(color * 2.5f, 1.0f);

		// render
		Render(Environment->m_clouds.mdCloud, nullptr, 100.0);
		Render_Alpha(Environment->m_clouds.mdCloud, nullptr, 100.0);
		// post-render cleanup
	}

	gl::program::unbind();
	gl::vao::unbind();
    ::glPopMatrix();

	m_sunlight.apply_angle();
	m_sunlight.apply_intensity();

    // calculate shadow tone, based on positions of celestial bodies
	m_shadowcolor = glm::mix(glm::vec4{colors::shadow}, glm::vec4{colors::white}, std::clamp(-Environment->m_sun.getAngle(), 0.f, 6.f) / 6.f);
	if ((Environment->m_sun.getAngle() < -18.f) && (Environment->m_moon.getAngle() > 0.f))
	{
		// turn on moon shadows after nautical twilight, if the moon is actually up
		m_shadowcolor = colors::shadow;
	}
	// soften shadows depending on sky overcast factor
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

	return m_geometry.replace(Vertices, Userdata, Geometry, Offset);
}

// adds supplied vertex data at the end of specified chunk
bool opengl46_renderer::Append(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type)
{
	gfx::calculate_tangents(Vertices, gfx::index_array(), Type);

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

// material methods
material_handle opengl46_renderer::Fetch_Material(std::string const &Filename, bool const Loadnow)
{

	return m_materials.create(Filename, Loadnow);
}

std::shared_ptr<gl::program> opengl46_renderer::Fetch_Shader(const std::string &name)
{
	auto it = m_shaders.find(name);
	if (it == m_shaders.end())
	{
		gl::shader fragment("mat_" + name + ".frag");
		gl::program *program = new gl::program({fragment, *m_vertex_shader.get()});
		m_shaders.insert({name, std::shared_ptr<gl::program>(program)});
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

		// materials which declare texture variants provide a different texture set per model instance,
		// picked from the seed of the rendered instance and the variant group of the submodel
		auto const &materialtextures{
			material.texture_variants.empty() ?
		        material.textures :
		        material.GetTextures(TSubModel::VariantId(sm))};

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

		material.shader->bind();
	}
	else if (Material != m_invalid_material)
		Bind_Material(m_invalid_material);
}

void opengl46_renderer::Bind_Material_Shadow(material_handle const Material)
{
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

material_handle opengl46_renderer::Terrain_Material(material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement)
{
	auto const material{m_terrainmaterials.make(Reuse, Layers, Samples, Weights, Placement)};
	if (material != null_handle)
		return material;
	// the terrain shader can't be used, the chunk makes do with a plain material
	return gfx_renderer::Terrain_Material(Reuse, Layers, Samples, Weights, Placement);
}

void opengl46_renderer::Terrain_Release(material_handle const Material)
{
	m_terrainmaterials.release(Material);
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

void opengl46_renderer::Render(scene::basic_region *Region)
{

	m_sectionqueue.clear();
	m_cellqueue.clear();
	// discard last pass's accumulated instance buckets before this pass starts.
	// the vectors themselves are kept, so their allocations get reused by the following passes
	for( auto &bucket : m_frame_instance_buckets ) {
		bucket.second.clear();
	}
	// the gpu culling runs along the first color pass of the main viewport of a frame. its result is drawn (Global.gfx_gpucull 1),
	// or compared with the one of the cpu, which draws (2)
	m_instancecullpass = false;
	if( ( Global.gfx_gpucull != 0 )
	 && ( m_renderpass.draw_mode == rendermode::color )
	 && ( m_current_viewport != nullptr )
	 && ( m_current_viewport->main )
	 && ( m_instancecullframe != m_framestamp ) ) {
		m_instancecullframe = m_framestamp;
		m_instancecullpass = true;
		// the same choices as in Render_Instanced()
		auto const reflections { m_renderpass.draw_mode == rendermode::reflections };
		opengl46_instance_cull::pass_setup setup;
		setup.frame = m_framestamp;
		setup.camera = m_renderpass.pass_camera.position();
		setup.distancecamera = ( m_renderpass.draw_mode == rendermode::shadows ? m_renderpass.viewport_camera.position() : m_renderpass.pass_camera.position() );
		setup.frustum = &m_renderpass.pass_camera.frustum();
		setup.distancescale = ( reflections ? 1.f : static_cast<float>( 1.0 / ( static_cast<double>( Global.ZoomFactor ) * static_cast<double>( Global.ZoomFactor ) * static_cast<double>( Global.fDistanceFactor ) ) ) );
		setup.distanceoffset = ( reflections ? static_cast<float>( sq( EU07_REFLECTIONFIDELITYOFFSET ) ) : 0.f );
		setup.distancerange = ( reflections ? static_cast<float>( sq( Global.reflectiontune.range_instances ) ) : std::numeric_limits<float>::max() );
		setup.drawrange = static_cast<float>( sq( m_renderpass.draw_range + 250 ) );
		setup.zoom = Global.ZoomFactor;
		setup.drawing = ( Global.gfx_gpucull == 1 );
		m_instancecull.begin_pass( setup );
	}
	// build a list of region sections to render
	glm::vec3 const cameraposition{m_renderpass.pass_camera.position()};
	auto const camerax = static_cast<int>(std::floor(cameraposition.x / scene::EU07_SECTIONSIZE + scene::EU07_REGIONSIDESECTIONCOUNT / 2));
	auto const cameraz = static_cast<int>(std::floor(cameraposition.z / scene::EU07_SECTIONSIZE + scene::EU07_REGIONSIDESECTIONCOUNT / 2));
	float section_range = m_renderpass.draw_range * Global.fDistanceFactor;
	section_range = editor_track_view_range(section_range, EditorModeFlag && (m_renderpass.draw_mode == rendermode::color || m_renderpass.draw_mode == rendermode::pickscenery));
	int const segmentcount = 2 * static_cast<int>(std::ceil(section_range / scene::EU07_SECTIONSIZE));
	int const originx = camerax - segmentcount / 2;
	int const originz = cameraz - segmentcount / 2;

	for (int row = originz; row <= originz + segmentcount; ++row)
	{
		if (row < 0)
		{
			continue;
		}
		if (row >= scene::EU07_REGIONSIDESECTIONCOUNT)
		{
			break;
		}
		for (int column = originx; column <= originx + segmentcount; ++column)
		{
			if (column < 0)
			{
				continue;
			}
			if (column >= scene::EU07_REGIONSIDESECTIONCOUNT)
			{
				break;
			}
			auto *section{Region->m_sections[row * scene::EU07_REGIONSIDESECTIONCOUNT + column]};
			if ((section != nullptr) && (m_renderpass.pass_camera.visible(section->m_area)))
			{
				m_sectionqueue.emplace_back(section);
			}
		}
	}

	switch (m_renderpass.draw_mode)
	{
	case rendermode::color:
	{
		Render(std::begin(m_sectionqueue), std::end(m_sectionqueue));
		// draw queue is filled while rendering sections
		if (EditorModeFlag && m_current_viewport->main)
		{
			// when editor mode is active calculate world position of the cursor
			// at this stage the z-buffer is filled with only ground geometry
            Update_Mouse_Position();
		}
		// draw opaque cells front-to-back: with the depth test enabled this
		// lets the GPU reject hidden fragments early, before the (expensive)
		// lit fragment shader runs on them. Order is irrelevant to the final
		// image for opaque geometry, so this is purely a fill-rate win.
		std::sort( std::begin( m_cellqueue ), std::end( m_cellqueue ),
			[]( distancecell_pair const &Left, distancecell_pair const &Right ) { return Left.first < Right.first; } );
		Render(std::begin(m_cellqueue), std::end(m_cellqueue));
		break;
	}
	case rendermode::shadows:
    case rendermode::pickscenery:
	{
		// these render modes don't bother with lights
		Render(std::begin(m_sectionqueue), std::end(m_sectionqueue));
		// they can also skip queue sorting, as they only deal with opaque geometry
		// NOTE: there's benefit from rendering front-to-back, but is it significant enough? TODO: investigate
		Render(std::begin(m_cellqueue), std::end(m_cellqueue));
		break;
	}
	case rendermode::reflections:
	{
		Render(std::begin(m_sectionqueue), std::end(m_sectionqueue));
        if( Global.reflectiontune.fidelity >= 1 ) {
            Render(std::begin(m_cellqueue), std::end(m_cellqueue));
        }
		break;
	}
	case rendermode::pickcontrols:
	default:
	{
		// no need to render anything ourside of the cab in control picking mode
		break;
	}
	}
}

void opengl46_renderer::Render(section_sequence::iterator First, section_sequence::iterator Last)
{

	switch (m_renderpass.draw_mode)
	{
	case rendermode::color:
	case rendermode::reflections:
    case rendermode::shadows:
		break;
	case rendermode::pickscenery:
	{
		// non-interactive scenery elements get neutral colour
		model_ubs.param[0] = colors::none;
		break;
	}
	default:
		break;
	}

	while (First != Last)
	{

		auto *section = *First;
		section->create_geometry();

		// render shapes held by the section
		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		case rendermode::reflections:
		case rendermode::shadows:
        case rendermode::pickscenery:
		{
			if (false == section->m_shapes.empty())
			{
				// since all shapes of the section share center point we can optimize out a few calls here
				::glPushMatrix();
				auto const originoffset{section->m_area.center - m_renderpass.pass_camera.position()};
				::glTranslated(originoffset.x, originoffset.y, originoffset.z);
				// render
				for (auto const &shape : section->m_shapes)
				{
					Render(shape, true);
				}
				// post-render cleanup
				::glPopMatrix();
			}
			break;
		}
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}

		// add the section's cells to the cell queue
		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		case rendermode::shadows:
        case rendermode::pickscenery:
		{
            // TBD, TODO: refactor into a method to reuse below?
			for (auto &cell : section->m_cells)
			{
				if ((true == cell.m_active) && (m_renderpass.pass_camera.visible(cell.m_area)))
				{
					// store visible cells with content as well as their current distance, for sorting later
					m_cellqueue.emplace_back(glm::length2(m_renderpass.pass_camera.position() - cell.m_area.center), &cell);
				}
			}
			break;
		}
		case rendermode::reflections:
        {
            // we can skip filling the cell queue if reflections pass isn't going to use it
            if( Global.reflectiontune.fidelity == 0 ) { break; }
			for (auto &cell : section->m_cells)
			{
				if ((true == cell.m_active) && (m_renderpass.pass_camera.visible(cell.m_area)))
				{
					// store visible cells with content as well as their current distance, for sorting later
					m_cellqueue.emplace_back(glm::length2(m_renderpass.pass_camera.position() - cell.m_area.center), &cell);
				}
			}
			break;
        }
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}
		// proceed to next section
		++First;
	}

	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
    {
		break;
	}
	default:
	{
		break;
	}
	}
}

void opengl46_renderer::Render(cell_sequence::iterator First, cell_sequence::iterator Last)
{

	// cache initial iterator for the second sweep
	auto first{First};
	auto const scenery_limit{static_cast<double>(Global.BaseDrawRange) * Global.fDistanceFactor + 250.0};
	bool const far_paths{EditorModeFlag && Global.editor_tracks && (m_renderpass.draw_mode == rendermode::color || m_renderpass.draw_mode == rendermode::pickscenery)};
	glm::dvec3 const camera{m_renderpass.pass_camera.position()};
	// first pass draws elements which we know are located in section banks, to reduce vbo switching
	while (First != Last)
	{

		auto *cell = First->second;
		// przeliczenia animacji torów w sektorze
		cell->RaAnimate(m_framestamp);
		bool const tracks_only{editor_far_tracks_only(far_paths, cell, camera, scenery_limit)};

		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		{
			// since all shapes of the section share center point we can optimize out a few calls here
			::glPushMatrix();
			auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
			::glTranslated(originoffset.x, originoffset.y, originoffset.z);

			// render
			// opaque non-instanced shapes
			if (false == tracks_only)
			{
				for (auto const &shape : cell->m_shapesopaque)
				{
					Render(shape, false);
				}
			}
			// tracks
			// TODO: update after path node refactoring
			Render(std::begin(cell->m_paths), std::end(cell->m_paths));
			// post-render cleanup
			::glPopMatrix();

			break;
		}
        case rendermode::reflections:
        {
			// since all shapes of the section share center point we can optimize out a few calls here
			::glPushMatrix();
			auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
			::glTranslated(originoffset.x, originoffset.y, originoffset.z);

			// render
			// opaque non-instanced shapes
			for (auto const &shape : cell->m_shapesopaque)
				Render(shape, false);

			// post-render cleanup
			::glPopMatrix();

			break;
        }
		case rendermode::shadows:
		{
			// since all shapes of the section share center point we can optimize out a few calls here
			::glPushMatrix();
			auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
			::glTranslated(originoffset.x, originoffset.y, originoffset.z);

			// render
			// opaque non-instanced shapes
			for (auto const &shape : cell->m_shapesopaque)
				Render(shape, false);
			// tracks
			Render(std::begin(cell->m_paths), std::end(cell->m_paths));

			// post-render cleanup
			::glPopMatrix();

			break;
		}
		case rendermode::pickscenery:
		{
			// same procedure like with regular render, but editor-enabled nodes receive custom colour used for picking
			// since all shapes of the section share center point we can optimize out a few calls here
			::glPushMatrix();
			auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
			::glTranslated(originoffset.x, originoffset.y, originoffset.z);
			// render
			// opaque non-instanced shapes
			// non-interactive scenery elements get neutral colour
			if (false == tracks_only)
			{
				model_ubs.param[0] = colors::none;
				for (auto const &shape : cell->m_shapesopaque)
					Render(shape, false);
			}
			// tracks
			for (auto *path : cell->m_paths)
			{
				model_ubs.param[0] = glm::vec4(pick_color(m_picksceneryitems.size() + 1), 1.0f);
				Render(path);
			}
			// post-render cleanup
			::glPopMatrix();
			break;
		}
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}

		++First;
	}
	// second pass draws elements with their own vbos
	while (first != Last)
	{

		auto const *cell = first->second;

		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
        case rendermode::shadows:
        {
			if (editor_far_tracks_only(far_paths, cell, camera, scenery_limit))
				break;
            // TBD, TODO: refactor in to a method to reuse in branch below?
			// opaque parts of instanced models -- accumulate this cell's buckets
			// into the frame-level map; the actual Render_Instanced() calls are
			// issued once per unique model after the cell loop (see flush below).
			//
			// Cell-level far-distance pre-cull: when the whole cell lies beyond
			// the instance draw distance, every instance in it would fail the
			// per-instance drawdistancethreshold test inside Render_Instanced(),
			// so there is no point merging its buckets into the frame map. The
			// test reproduces Render_Instanced()'s distance maths exactly -- same
			// ZoomFactor / fDistanceFactor scaling, same +250 margin -- applied to
			// the nearest point of the cell's bounding sphere. basic_cell::enclose_area
			// guarantees m_area.radius >= |m_area.center - instance.location()| for
			// every contained instance, so the nearest-point distance is a true
			// lower bound on every instance's distance: the cull can never drop a
			// cell that still holds a drawable instance. Shadows measure from the
			// real (viewport) camera, matching Render_Instanced()'s shadow branch;
			// every other gated mode measures from the pass camera. Non-instanced
			// scenery and vehicles below keep their own per-node culling and are
			// intentionally left untouched.
			{
				auto const &distancecamera = (
					m_renderpass.draw_mode == rendermode::shadows
						? m_renderpass.viewport_camera
						: m_renderpass.pass_camera );
				auto const cellcenterdistance { glm::length( cell->m_area.center - distancecamera.position() ) };
				auto const cellnearestdistance { std::max( 0.0, cellcenterdistance - cell->m_area.radius ) };
				auto const cellnearestdistancesquared {
					( cellnearestdistance * cellnearestdistance )
					/ ( static_cast<double>( Global.ZoomFactor ) * static_cast<double>( Global.ZoomFactor ) )
					/ static_cast<double>( Global.fDistanceFactor ) };
				if( cellnearestdistancesquared <= sq( static_cast<double>( m_renderpass.draw_range ) + 250.0 ) ) {
					Queue_Instances( cell );
				}
			}
			// remaining (non-instanceable) opaque instance nodes go through the per-node path
			for (auto *instance : cell->m_instancesopaque)
			{
				if( instance->m_instanceable ) { continue; } // already handled via bucket
				// the model of a lamp would shadow everything in the map of the lamp
				if( instance == m_cubeskipinstance ) { continue; }
				Render(instance);
			}
			// opaque parts of vehicles. the kept scenery part of the map of a lamp is drawn without them
			if( false == m_cubescenery ) {
				for (auto *path : cell->m_paths)
				{
					for (auto *dynamic : path->Dynamics)
					{
						Render(dynamic);
					}
				}
			}
			break;
		}
        case rendermode::reflections:
        {
            // a cell lying whole beyond the range the reflections draw model instances at is left out, as each
            // of its instances would fail the range test done for it in Render_Instanced() and Render( TAnimModel * ).
            // like the cull of the regular passes above, the test goes by the nearest point of the bounding sphere
            // of the cell, which holds the locations of all its instances
            auto const cellnearestdistance { std::max( 0.0, glm::length( cell->m_area.center - m_renderpass.pass_camera.position() ) - cell->m_area.radius ) };
            if( ( Global.reflectiontune.fidelity >= 1 )
             && ( cellnearestdistance <= Global.reflectiontune.range_instances ) ) {
                // opaque parts of instanced models -- accumulate into the
                // frame-level map; flushed once per unique model after the loop.
                Queue_Instances( cell );
                for( auto *instance : cell->m_instancesopaque ) {
                    if( instance->m_instanceable ) { continue; }
                    Render( instance );
                }
            }
            if( Global.reflectiontune.fidelity >= 2 ) {
                // opaque parts of vehicles
                for( auto *path : cell->m_paths ) {
                    for( auto *dynamic : path->Dynamics ) {
                        Render( dynamic );
                    }
                }
            }
			break;
        }
        case rendermode::pickscenery:
		{
			if (editor_far_tracks_only(far_paths, cell, camera, scenery_limit))
				break;
			// opaque parts of instanced models
			// same procedure like with regular render, but each node receives custom colour used for picking.
			// picking always uses the per-instance path because each instance needs a unique pick colour;
			// batching would assign the same colour to every instance in the bucket.
			for (auto *instance : cell->m_instancesopaque)
			{
				model_ubs.param[0] = glm::vec4(pick_color(m_picksceneryitems.size() + 1), 1.0f);
				Render(instance);
			}
			// vehicles aren't included in scenery picking for the time being
			break;
		}
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}

		++first;
	}

	// flush accumulated instance buckets: issue one Render_Instanced() per unique
	// (TModel3d*, skins) key across every cell visited in this pass, instead of
	// one call per cell. All per-instance frustum/distance/pixel-area culling
	// still happens inside Render_Instanced(), so correctness is unchanged -- only
	// the number of calls and glBufferSubData round-trips drops sharply.
	// (For pickscenery/pickcontrols modes the map stays empty, so this is a no-op.)
	if( m_instancecullpass ) {
		if( m_instancecull.drawing() ) {
			// the buckets of the pass weren't queued for the cpu, which leaves the loop below empty
			Render_Instances_Indirect();
		}
		else {
			// the view of the pass is in place here, the one Render_Instanced() takes
			m_instancecull.dispatch( OpenGLMatrices.data( GL_MODELVIEW ) );
		}
	}
	for( auto &bucket : m_frame_instance_buckets ) {
		if( bucket.second.empty() ) { continue; }
		Render_Instanced( bucket.first.pModel, bucket.second, ( m_instancecullpass ? &m_instance_bandcounts : nullptr ) );
		if( m_instancecullpass ) {
			m_instancecull.cpu_result( bucket.first, m_instance_bandcounts );
		}
		bucket.second.clear();
	}
	if( m_instancecullpass ) {
		m_instancecull.end_pass();
		m_instancecullpass = false;
	}
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
		gfx::indirect_source source;
		if( m_geometry.indirect_source( handle, source ) ) {
			m_indirectcommands.push_back( { source.count, 0, source.first, static_cast<std::uint32_t>( source.basevertex ), 0, m_indirectentry, ( source.indexed ? 1u : 0u ), 0 } );
			m_indirectdraws.push_back( { model_ubs, m_indirectmaterial, m_indirectsubmodel, handle, source.type, m_blendingenabled } );
		}
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
	model_ubo->update(model_ubs);

	Draw_Geometry(it, end);
}

void opengl46_renderer::Render(scene::shape_node const &Shape, bool const Ignorerange)
{
	auto const &data{Shape.data()};

	if (false == Ignorerange)
	{
		double distancesquared;
		switch (m_renderpass.draw_mode)
		{
		case rendermode::shadows:
        {
			// 'camera' for the light pass is the light source, but we need to draw what the 'real' camera sees
			distancesquared = glm::length2((data.area.center - m_renderpass.viewport_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
			break;
		}
        case rendermode::reflections:
        {
            // reflection mode draws simplified version of the shapes, by artificially increasing view range
            distancesquared =
                // TBD, TODO: bind offset value with setting variable?
                sq(EU07_REFLECTIONFIDELITYOFFSET)
                + glm::length2( ( data.area.center - m_renderpass.pass_camera.position() ) );
/*
                // TBD: take into account distance multipliers?
                / Global.fDistanceFactor;
*/
            break;
        }
		default:
		{
			distancesquared = glm::length2((data.area.center - m_renderpass.pass_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
			break;
		}
		}
		if ((distancesquared < data.rangesquared_min) || (distancesquared >= data.rangesquared_max))
		{
			return;
		}
	}

	// setup
	switch (m_renderpass.draw_mode)
	{
	case rendermode::color:
	case rendermode::reflections:
		Bind_Material(data.material, nullptr, &Shape.data().lighting );
		break;
	case rendermode::shadows:
		// skip if the shadow caster rank is too low for currently set threshold
		if( Material( data.material ).shadow_rank > Global.gfx_shadow_rank_cutoff )
			return;
		Bind_Material_Shadow(data.material);
		break;
	case rendermode::pickscenery:
	case rendermode::pickcontrols:
		m_pick_shader->bind();
		break;
	default:
		break;
	}
	// render

	draw(data.geometry);
	// debug data
	++m_renderpass.draw_stats.shapes;
	++m_renderpass.draw_stats.drawcalls;
}

void opengl46_renderer::Render(TAnimModel *Instance)
{

	if (false == Instance->m_visible) { return; }
    if( false == m_renderpass.pass_camera.visible( Instance->m_area ) ) { return; }

    double distancesquared;
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
    {
        // 'camera' for the light pass is the light source, but we need to draw what the 'real' camera sees
		distancesquared = glm::length2((Instance->location() - m_renderpass.viewport_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
		break;
	}
    case rendermode::reflections:
    {
        // reflection mode draws simplified version of the shapes, by artificially increasing view range
        // it also ignores zoom settings and distance multipliers
        // TBD: take into account distance multipliers?
        distancesquared = glm::length2((Instance->location() - m_renderpass.pass_camera.position())) /* / Global.fDistanceFactor */;
        // NOTE: arbitrary draw range limit
        if( distancesquared > Global.reflectiontune.range_instances * Global.reflectiontune.range_instances ) {
            return;
        }
        // TBD, TODO: bind offset value with setting variable?
        distancesquared += sq(EU07_REFLECTIONFIDELITYOFFSET);
        break;
    }
	default:
	{
		distancesquared = glm::length2((Instance->location() - m_renderpass.pass_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
		break;
	}
	}
	if ((distancesquared < Instance->m_rangesquaredmin) || (distancesquared >= Instance->m_rangesquaredmax))
	{
		return;
	}
     // crude way to reject early items too far to affect the output (mostly relevant for shadow passes)
    auto const drawdistancethreshold{ m_renderpass.draw_range + 250 };
    if( distancesquared > sq(drawdistancethreshold) ) {
        return;
    }
   // second stage visibility cull, reject modelstoo far away to be noticeable
    auto const radiussquared { Instance->radius() * Instance->radius() };
    if( radiussquared * Global.ZoomFactor / distancesquared < 0.003 * 0.003 ) {
        return;
    }

	switch (m_renderpass.draw_mode)
	{
	case rendermode::pickscenery:
	{
		// add the node to the pick list
		m_picksceneryitems.emplace_back(Instance);
		break;
	}
	default:
	{
		break;
	}
	}

	Instance->RaAnimate(m_framestamp); // jednorazowe przeliczenie animacji
	Instance->RaPrepare();
	if (Instance->pModel)
	{
		// per-instance uniform scale: applied between the rotation and the
		// submodel-local transform chain. Pushing it on the matrix stack here
		// means Render(TModel3d, ..., position, angle) doesn't need a new
		// signature and the same scale flows naturally into Render_Alpha and
		// any other recursive callers.
		auto const &scale = Instance->Scale();
		bool const scaled = ( scale.x != 1.0f || scale.y != 1.0f || scale.z != 1.0f );
		if( scaled ) {
			// Per-axis scale applied between rotation and the submodel-local
			// chain: build the per-instance transform here so we can inject
			// glScalef before the submodel walk.
			::glPushMatrix();
			auto const Position = Instance->location() - m_renderpass.pass_camera.position();
			auto const Angle = Instance->vAngle;
			::glTranslated( Position.x, Position.y, Position.z );
			if( Angle.y != 0.0 ) ::glRotated( Angle.y, 0.f, 1.f, 0.f );
			if( Angle.x != 0.0 ) ::glRotated( Angle.x, 1.f, 0.f, 0.f );
			if( Angle.z != 0.0 ) ::glRotated( Angle.z, 0.f, 0.f, 1.f );
			::glScalef( scale.x, scale.y, scale.z );
			Render( Instance->pModel, Instance->Material(), distancesquared );
			::glPopMatrix();
		}
		else {
			// fast path for unscaled (the common case): existing behaviour
			Render(Instance->pModel, Instance->Material(), distancesquared, Instance->location() - m_renderpass.pass_camera.position(), Instance->vAngle);
		}
	    // debug data
	    ++m_renderpass.draw_stats.models;
	}
}

// adds the bucketed instances of a scene cell to the frame-level buckets, for Render_Instanced() to draw.
// Render_Instanced() culls each instance it's given by its record. the tests it makes are repeated here, with some slack
// for the single precision of the records, and the instances which fail are left out of the queue
void opengl46_renderer::Queue_Instances( scene::basic_cell const *Cell )
{
	// the editor moves and resizes the instances in place. the records are made anew for each frame then,
	// but aren't relied on to leave instances out of the queue
	auto const userecords { false == EditorModeFlag };
	auto const reflections { m_renderpass.draw_mode == rendermode::reflections };
	// same choices as in Render_Instanced(): shadows measure the distance from the real camera, reflections
	// ignore zoom and distance factor, test the plain distance against their own range and add the fidelity offset
	auto const &distancecamera { m_renderpass.draw_mode == rendermode::shadows ? m_renderpass.viewport_camera : m_renderpass.pass_camera };
	glm::dvec3 const cameraposition { distancecamera.position() };
	auto const distancescale { reflections ? 1.0 : 1.0 / ( static_cast<double>( Global.ZoomFactor ) * static_cast<double>( Global.ZoomFactor ) * static_cast<double>( Global.fDistanceFactor ) ) };
	auto const distanceoffset { reflections ? sq( static_cast<double>( EU07_REFLECTIONFIDELITYOFFSET ) ) : 0.0 };
	auto const slack { 1.001 };
	auto const distancelimit { slack * ( reflections ? sq( static_cast<double>( Global.reflectiontune.range_instances ) ) : sq( static_cast<double>( m_renderpass.draw_range ) + 250.0 ) ) };
	// up close the rounding of the records is too large a part of the distance for the slack to cover it
	auto const distancetested { sq( 64.0 ) };

	for( auto const &bucket : Cell->m_instancebuckets_opaque ) {
		auto const &instancebucket { bucket.second };
		if( ( instancebucket.recordsversion != m_instancerecordsversion )
		 || ( instancebucket.records.size() != instancebucket.instances.size() ) ) {
			Update_Instance_Records( bucket.first.pModel, instancebucket, Cell->m_area.center );
		}
		if( m_instancecullpass ) {
			// the gpu tests every instance of the bucket, the way Render_Instanced() does
			m_instancecull.add_bucket( bucket.first, instancebucket, Cell->m_area );
			// and when the instances are drawn from its result, the cpu has nothing more to do with them
			if( m_instancecull.drawing() ) { continue; }
		}
		auto const &records { instancebucket.records };
		std::vector<queued_instance> *dest { nullptr }; // looked up when there's something to put in it
		for( std::size_t idx = 0; idx < records.size(); ++idx ) {
			if( userecords ) {
				auto const &record { records[ idx ] };
				auto const center { instancebucket.recordorigin + glm::dvec3{ record.position } };
				auto const distancesquared { glm::length2( center - cameraposition ) };
				if( distancesquared > distancetested ) {
					auto const scaleddistancesquared { distancesquared * distancescale };
					if( scaleddistancesquared > distancelimit ) { continue; }
					if( scaleddistancesquared + distanceoffset >= slack * std::min( instancebucket.lodrangesquared, record.rangesquaredmax ) ) { continue; }
				}
				if( false == m_renderpass.pass_camera.visible( scene::bounding_area{ center, record.radius } ) ) { continue; }
			}
			if( dest == nullptr ) {
				dest = &m_frame_instance_buckets[ bucket.first ];
			}
			dest->push_back( { &instancebucket, idx } );
		}
	}
}

// makes the records of the instances held by a bucket of a scene cell, with locations relative to Origin
void opengl46_renderer::Update_Instance_Records( TModel3d const *Model, scene::basic_cell::instance_bucket const &Bucket, glm::dvec3 const &Origin )
{
	// a submodel is drawn only closer than its fSquareMaxDist, see Render( TSubModel * ). past the largest of these
	// there's nothing of the model left to draw, which makes it another limit of the range of its instances
	auto lodrangesquared { ( Model != nullptr && Model->Root != nullptr ) ? 0.f : std::numeric_limits<float>::max() };
	m_instance_lodpending.clear();
	m_instance_lodpending.push_back( Model != nullptr ? Model->Root : nullptr );
	while( false == m_instance_lodpending.empty() ) {
		auto const *submodel = m_instance_lodpending.back();
		m_instance_lodpending.pop_back();
		if( submodel == nullptr ) { continue; }
		lodrangesquared = std::max( lodrangesquared, submodel->fSquareMaxDist );
		m_instance_lodpending.push_back( submodel->Child );
		m_instance_lodpending.push_back( submodel->Next );
	}
	Bucket.lodrangesquared = lodrangesquared;
	Bucket.recordorigin = Origin;

	auto const tofloat = []( double const Value ) {
		return static_cast<float>( std::min( Value, static_cast<double>( std::numeric_limits<float>::max() ) ) ); };

	Bucket.records.clear();
	Bucket.records.reserve( Bucket.instances.size() );
	for( auto *instance : Bucket.instances ) {
		scene::basic_cell::instance_record record;
		if( instance != nullptr ) {
			record.position = packed_vec3{ instance->location() - Origin };
			record.radius = instance->radius();
			record.rangesquaredmin = tofloat( instance->m_rangesquaredmin );
			record.rangesquaredmax = tofloat( instance->m_rangesquaredmax );
			record.flags =
				( instance->visible() ? scene::basic_cell::instance_visible : 0u )
				| ( instance->m_instanceable ? scene::basic_cell::instance_instanceable : 0u );
			auto const &rotationscale { instance->rotation_scale() };
			for( int column = 0; column < 3; ++column ) {
				record.rotationscale[ column ] = rotationscale[ column ];
			}
		}
		// an empty slot keeps its record without flags, and isn't drawn
		Bucket.records.emplace_back( record );
	}
	Bucket.recordsversion = m_instancerecordsversion;
	Bucket.gpustale = true;
}

// places supplied modelview matrices in the instance buffer of the frame. returns: base instance of the first of them
std::uint32_t opengl46_renderer::Upload_Instance_Modelviews( glm::mat4 const *Modelviews, std::size_t const Count )
{
	auto &buffer { m_instancebuffers[ m_instancebufferindex ] };
	auto const needed { static_cast<GLsizeiptr>( ( m_instancebufferfill + Count ) * sizeof( glm::mat4 ) ) };
	if( needed > buffer->size() ) {
		// the draws issued with the replaced buffer keep it until the gpu is done with them. the new one starts with identity
		buffer->resize( std::max( needed, 2 * buffer->size() ) );
		glm::mat4 const identity( 1.0f );
		buffer->update( identity );
		buffer->bind_base();
		m_instancebufferfill = 1;
	}
	auto const base { static_cast<std::uint32_t>( m_instancebufferfill ) };
	buffer->update( Modelviews, static_cast<GLintptr>( base * sizeof( glm::mat4 ) ), static_cast<GLsizeiptr>( Count * sizeof( glm::mat4 ) ) );
	m_instancebufferfill += Count;
	return base;
}

// True GPU-instanced render path for a group of TAnimModel instances sharing the same
// TModel3d. The submodel tree is walked ONCE per lod band; at every submodel that draws
// geometry we issue a single glDrawElementsInstancedBaseVertexBaseInstance(N) covering all
// visible instances of the band. The vertex shader multiplies effective_modelview =
// instance_modelview[gl_BaseInstance + gl_InstanceID] * model_ubs.modelview, where
// instance_modelview[i] is the camera-space root transform of an instance (precomputed below)
// and model_ubs.modelview is the submodel-local chain (accumulated by the matrix stack starting
// from identity, NOT the camera/instance frame).
void opengl46_renderer::Render_Instanced( TModel3d *Model, std::vector<queued_instance> const &Instances, std::vector<std::uint32_t> *Bandcounts )
{
	if( Bandcounts != nullptr ) { Bandcounts->clear(); }
	if( Model == nullptr ) { return; }
	if( Instances.empty() ) { return; }

	// LOD: a submodel is drawn only while fSquareDist lies inside its [fSquareMinDist, fSquareMaxDist)
	// range, so the distinct range bounds split distance into bands with constant LOD. The bucket
	// collects instances from every visible cell, so a single fSquareDist for all of them would draw
	// far instances at the LOD of the nearest one. Each instance is assigned to its band and every
	// band is drawn with its own fSquareDist, like in Render_Sleepers(). The grouping is a counting
	// sort, linear in the instance count.
	// the same walk also checks whether any piece of the model can get into the shadow map. Models
	// whose materials are all excluded by the shadow rank cutoff (e.g. grass) would otherwise go through
	// the whole per-instance culling and the batched draws in every shadow pass, only for every submodel
	// to be skipped in Render(TSubModel*)
	auto const *frontinstance { Instances.front().bucket->instances[ Instances.front().index ] };
	auto const *skins { frontinstance != nullptr && frontinstance->Material() != nullptr ? frontinstance->Material()->replacable_skins : nullptr };
	bool castsshadow { m_renderpass.draw_mode != rendermode::shadows };
	m_instance_lodbounds.clear();
	m_instance_lodpending.clear();
	m_instance_lodpending.push_back( Model->Root );
	while( false == m_instance_lodpending.empty() ) {
		auto const *submodel = m_instance_lodpending.back();
		m_instance_lodpending.pop_back();
		if( submodel == nullptr ) { continue; }
		if( submodel->fSquareMinDist > 0.f ) { m_instance_lodbounds.emplace_back( submodel->fSquareMinDist ); }
		if( submodel->fSquareMaxDist < std::numeric_limits<float>::max() ) { m_instance_lodbounds.emplace_back( submodel->fSquareMaxDist ); }
		if( ( false == castsshadow ) && ( submodel->eType < TP_ROTATOR ) ) {
			auto const material { submodel->m_material >= 0 ? submodel->m_material : ( skins != nullptr ? skins[ -submodel->m_material ] : null_handle ) };
			castsshadow = ( Material( material ).shadow_rank <= Global.gfx_shadow_rank_cutoff );
		}
		m_instance_lodpending.push_back( submodel->Child );
		m_instance_lodpending.push_back( submodel->Next );
	}
	if( false == castsshadow ) { return; }
	std::sort( m_instance_lodbounds.begin(), m_instance_lodbounds.end() );
	m_instance_lodbounds.erase( std::unique( m_instance_lodbounds.begin(), m_instance_lodbounds.end() ), m_instance_lodbounds.end() );
	auto const bandcount { m_instance_lodbounds.size() + 1 };
	if( Bandcounts != nullptr ) { Bandcounts->assign( bandcount, 0 ); }

	// 1. Visibility / distance cull, by the records of the instances. Collect surviving instances with
	// their precomputed camera-space root modelview matrices and lod bands. m_instance_survivors and the
	// other scratch vectors are persistent members reused across every Render_Instanced() call:
	// clear() drops the contents but keeps the allocated capacity, so after the first few
	// frames this stops calling malloc/free entirely.
	m_instance_survivors.clear();
	m_instance_survivors.reserve( Instances.size() );

	// Pull the current pass camera/view transform once. We use the current GL
	// modelview matrix as the view matrix because at the point Render_Instanced
	// is called, OpenGLMatrices is set to the camera view (scene root) — no
	// per-instance transform has been pushed yet.
	glm::mat4 const view_matrix = OpenGLMatrices.data( GL_MODELVIEW );

	TAnimModel *batch_instance { nullptr }; // the first instance drawn; its RaPrepare() and material serve the whole bucket
	material_data const *batch_material { nullptr };
	float farthest_distancesquared { 0.f };

	for( auto const &queued : Instances ) {
		auto const &record { queued.bucket->records[ queued.index ] };
		// instances which lost their instanceable status after being bucketed (animation or skin
		// assigned by an event) are drawn by the per-node path
		if( ( record.flags & scene::basic_cell::instance_instanceable ) == 0 ) { continue; }
		if( ( record.flags & scene::basic_cell::instance_visible ) == 0 ) { continue; }
		glm::dvec3 const location { queued.bucket->recordorigin + glm::dvec3{ record.position } };
		if( false == m_renderpass.pass_camera.visible( scene::bounding_area{ location, record.radius } ) ) { continue; }

		double distancesquared;
		switch( m_renderpass.draw_mode ) {
		case rendermode::shadows:
			distancesquared = glm::length2( ( location - m_renderpass.viewport_camera.position() ) / (double)Global.ZoomFactor ) / Global.fDistanceFactor;
			break;
		case rendermode::reflections:
			distancesquared = glm::length2( ( location - m_renderpass.pass_camera.position() ) );
			if( distancesquared > Global.reflectiontune.range_instances * Global.reflectiontune.range_instances ) { continue; }
			distancesquared += sq( EU07_REFLECTIONFIDELITYOFFSET );
			break;
		default:
			distancesquared = glm::length2( ( location - m_renderpass.pass_camera.position() ) / (double)Global.ZoomFactor ) / Global.fDistanceFactor;
			break;
		}
		if( ( distancesquared < record.rangesquaredmin )
		 || ( distancesquared >= record.rangesquaredmax ) ) { continue; }

		auto const drawdistancethreshold { m_renderpass.draw_range + 250 };
		if( distancesquared > sq( drawdistancethreshold ) ) { continue; }

		auto const radiussquared { record.radius * record.radius };
		if( radiussquared * Global.ZoomFactor / distancesquared < 0.003 * 0.003 ) { continue; }

		if( batch_instance == nullptr ) {
			batch_instance = queued.bucket->instances[ queued.index ];
			batch_instance->RaPrepare();
			batch_material = batch_instance->Material();
		}

		// Build the camera-relative root modelview for this instance:
		//   mv = view * translate(instance_pos - camera_pos) * rotate(instance_angles) * scale(m_scale)
		// The rotation and scale part comes from the record, the translation goes into its last
		// column. The scale is folded in so the GPU-instanced path produces visually identical output
		// to the regular per-instance path; the shader applies it via effective_modelview = instance_mv * model_local.
		glm::mat4 const root {
			record.rotationscale[ 0 ],
			record.rotationscale[ 1 ],
			record.rotationscale[ 2 ],
			glm::vec4( glm::vec3( location - m_renderpass.pass_camera.position() ), 1.f ) };

		auto const distance { static_cast<float>( distancesquared ) };
		farthest_distancesquared = std::max( farthest_distancesquared, distance );
		m_instance_survivors.push_back( {
			view_matrix * root,
			distance,
			static_cast<std::uint32_t>( std::upper_bound( m_instance_lodbounds.begin(), m_instance_lodbounds.end(), distance ) - m_instance_lodbounds.begin() ) } );
	}

	if( m_instance_survivors.empty() ) { return; }

	// group the modelviews by lod band into a contiguous array, for the upload. every distance in
	// a band selects the same lod; the nearest one represents the band.
	// within a band the instances are additionally ordered roughly near-to-far, in distance slots. dense
	// alpha tested foliage then gets the most out of the early depth test, as the nearest instances fill
	// the depth buffer first and hide the fragments of those behind them before they get shaded.
	// both levels are done with a single counting sort, linear in the instance count
	auto const distanceslots { 16 };
	auto const keycount { bandcount * distanceslots };
	auto const slotscale { distanceslots / std::sqrt( std::max( farthest_distancesquared, 1.f ) ) };
	m_instance_bandstarts.assign( keycount + 1, 0 );
	m_instance_banddistances.assign( bandcount, std::numeric_limits<float>::max() );
	for( auto &survivor : m_instance_survivors ) {
		m_instance_banddistances[ survivor.band ] = std::min( m_instance_banddistances[ survivor.band ], survivor.distancesquared );
		auto const slot { std::min( distanceslots - 1, static_cast<int>( std::sqrt( survivor.distancesquared ) * slotscale ) ) };
		survivor.band = static_cast<std::uint32_t>( survivor.band * distanceslots + slot ); // from now on it's the sort key
		++m_instance_bandstarts[ survivor.band + 1 ];
	}
	for( std::size_t key = 1; key <= keycount; ++key ) {
		m_instance_bandstarts[ key ] += m_instance_bandstarts[ key - 1 ];
	}
	m_instance_bandcursors.assign( m_instance_bandstarts.begin(), m_instance_bandstarts.end() - 1 );
	m_instance_modelviews.resize( m_instance_survivors.size() );
	for( auto const &survivor : m_instance_survivors ) {
		m_instance_modelviews[ m_instance_bandcursors[ survivor.band ]++ ] = survivor.modelview;
	}
	if( Bandcounts != nullptr ) {
		for( std::size_t band = 0; band < bandcount; ++band ) {
			( *Bandcounts )[ band ] = static_cast<std::uint32_t>( m_instance_bandstarts[ ( band + 1 ) * distanceslots ] - m_instance_bandstarts[ band * distanceslots ] );
		}
	}
	// all bands go to the instance buffer at once, each band is drawn from its own base instance
	auto const baseinstance { Upload_Instance_Modelviews( m_instance_modelviews.data(), m_instance_modelviews.size() ) };

	auto alpha = ( batch_material != nullptr ? batch_material->textures_alpha : 0x30300030 );
	alpha ^= 0x0F0F000F;

	// 2. Walk the submodel tree once per lod band. The submodel-local matrix
	// stack starts at identity; the per-instance camera transform comes from
	// instance_modelview[gl_BaseInstance + gl_InstanceID] in the shader.
	for( std::size_t band = 0; band < bandcount; ++band ) {

		std::size_t const band_start = m_instance_bandstarts[ band * distanceslots ];
		std::size_t const band_end = m_instance_bandstarts[ ( band + 1 ) * distanceslots ];
		if( band_start == band_end ) { continue; }

		// 2a. Push identity onto the matrix stack so submodel transforms
		// accumulate from identity, not from camera/instance space.
		::glPushMatrix();
		::glLoadIdentity();

		// 2b. Configure the existing TModel3d/TSubModel render path. Setting
		// m_current_instance_count routes draw(handle) calls to draw_instanced.
		m_current_instance_count = band_end - band_start;
		m_current_instance_base = baseinstance + static_cast<std::uint32_t>( band_start );

		Model->Root->fSquareDist = m_instance_banddistances[ band ]; // shared global, used by submodel LOD
		Model->Root->ReplacableSet( ( batch_material != nullptr ? batch_material->replacable_skins : nullptr ), alpha );
		Model->Root->pRoot = Model;

		Render( Model->Root );

		m_current_instance_count = 0;
		m_current_instance_base = 0;

		::glPopMatrix();

		++m_renderpass.draw_stats.instanced_drawcalls;
	}

	m_renderpass.draw_stats.instances += static_cast<int>( m_instance_survivors.size() );
	m_renderpass.draw_stats.models += static_cast<int>( m_instance_survivors.size() );
}

// how the depth prepass treats the material. the colour program of the material decides: the ones which test the alpha of the
// diffuse texture against the opacity (mat_default.frag and the like) get the same test, the ones which never discard a
// fragment get none, and any other (parallax, water, clouds, unknown) is left out and writes its depth in the colour pass
int opengl46_renderer::Prepass_Class( material_handle const Material )
{
	if( Material == null_handle ) { return 0; }
	auto const &material { m_materials.material( Material ) };
	if( ( false == material.is_good ) || ( material.shader == nullptr ) ) { return 0; }
	auto const lookup { m_prepassclasses.find( material.shader.get() ) };
	if( lookup != m_prepassclasses.end() ) { return lookup->second; }

	// names the programs are fetched by: mat_<name>.frag
	static std::unordered_set<std::string> const alphatested {
		"default", "default_1", "default_2", "default_detail", "default_specgloss",
		"detail_normalmap", "detail_normalmap_specgloss", "normalmap", "normalmap_specgloss",
		"reflmap", "reflmap_specgloss", "shadowlessnormalmap", "shadowlessnormalmap_specgloss",
		"sunlessnormalmap", "sunlessnormalmap_specgloss" };
	static std::unordered_set<std::string> const solid { "colored", "default_0" };
	auto prepassclass { 0 };
	for( auto const &shader : m_shaders ) {
		if( shader.second.get() != material.shader.get() ) { continue; }
		prepassclass = ( alphatested.count( shader.first ) > 0 ? 2 : solid.count( shader.first ) > 0 ? 1 : 0 );
		break;
	}
	m_prepassclasses.emplace( material.shader.get(), prepassclass );
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
	m_instancecull.dispatch( OpenGLMatrices.data( GL_MODELVIEW ), ( useocclusion ? &occlusion : nullptr ) );

	// the draws, recorded while the gpu culls
	m_indirectdraws.clear();
	m_indirectcommands.clear();
	auto const &lodbounds { m_instancecull.lod_bounds() };
	m_indirectrecording = true;
	for( auto const &group : m_instancecull.draw_groups() ) {
		auto *model { group.key->pModel };
		if( ( model == nullptr ) || ( model->Root == nullptr ) || ( group.instance == nullptr ) ) { continue; }
		group.instance->RaPrepare();
		auto const *material { group.instance->Material() };
		auto alpha { material != nullptr ? material->textures_alpha : 0x30300030 };
		alpha ^= 0x0F0F000F;
		for( std::uint32_t band = 0; band <= group.lodcount; ++band ) {
			// a band holds the squared distances from its lower bound up to the next one; any of them selects the same lod
			auto const bandmin { band == 0 ? 0.f : lodbounds[ group.lodfirst + band - 1 ] };
			auto const bandmax { band == group.lodcount ? std::numeric_limits<float>::max() : lodbounds[ group.lodfirst + band ] };
			if( ( bandmin > group.distancemax ) || ( bandmax <= group.distancemin ) ) { continue; }
			m_indirectentry = group.entryfirst + band;
			::glPushMatrix();
			::glLoadIdentity();
			model->Root->fSquareDist = bandmin;
			model->Root->ReplacableSet( ( material != nullptr ? material->replacable_skins : nullptr ), alpha );
			model->Root->pRoot = model;
			Render( model->Root );
			::glPopMatrix();
		}
	}
	m_indirectrecording = false;

	m_instancecull.fill_commands( m_indirectcommands );
	std::uint32_t prepassdraws { 0 };
	if( false == m_indirectdraws.empty() ) {
		m_instancecull.bind_draw();
		auto const commandoffset = []( std::size_t const Index ) {
			return static_cast<std::intptr_t>( Index * sizeof( gl::instance_draw_command ) ); };

		if( Global.gfx_depthprepass && ( false == Global.bWireFrame ) ) {
			// the depth first: the colour pass shades only what stays in view. the depth goes a little farther than the
			// surface, so the colour pass, which tests and writes its depth as usual, is sure to pass where it's the same
			m_prepasstimer.begin();
			setup_drawing( false );
			::glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE );
			::glEnable( GL_POLYGON_OFFSET_FILL );
			::glPolygonOffset( -1.f, -1.f );
			// the draws without the alpha test first, then the ones with it, one program each
			for( auto prepassclass = 1; prepassclass <= 2; ++prepassclass ) {
				auto &program { prepassclass == 1 ? *m_prepass_shader : *m_prepass_alpha_shader };
				auto bound { false };
				for( std::size_t index = 0; index < m_indirectdraws.size(); ++index ) {
					auto const &draw { m_indirectdraws[ index ] };
					if( ( draw.blending ) || ( draw.ubs.opacity < 0.f ) ) { continue; }
					if( ( draw.type != GL_TRIANGLES ) && ( draw.type != GL_TRIANGLE_STRIP ) && ( draw.type != GL_TRIANGLE_FAN ) ) { continue; }
					if( Prepass_Class( draw.material ) != prepassclass ) { continue; }
					if( prepassclass == 2 ) {
						// the diffuse texture of the colour pass, the same choice as Bind_Material() makes
						auto const &material { m_materials.material( draw.material ) };
						auto const &textures {
							material.texture_variants.empty() ?
								material.textures :
								material.GetTextures( TSubModel::VariantId( draw.submodel ) ) };
						if( textures[ 0 ] == null_handle ) { continue; }
						m_textures.bind( 0, textures[ 0 ] );
					}
					if( false == bound ) {
						program.bind();
						bound = true;
					}
					model_ubs = draw.ubs;
					model_ubo->update( model_ubs );
					m_geometry.draw_indirect( draw.geometry, commandoffset( index ) );
					++prepassdraws;
				}
			}
			::glDisable( GL_POLYGON_OFFSET_FILL );
			::glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
			m_prepasstimer.end();
		}

		for( std::size_t index = 0; index < m_indirectdraws.size(); ++index ) {
			auto const &draw { m_indirectdraws[ index ] };
			if( draw.blending ) { setup_drawing( true ); }
			Bind_Material( draw.material, draw.submodel );
			model_ubs = draw.ubs;
			model_ubo->update( model_ubs );
			m_geometry.draw_indirect( draw.geometry, commandoffset( index ) );
			if( draw.blending ) { setup_drawing( false ); }
		}
		model_ubs.emission = 0.0f;

		::glBindBuffer( GL_DRAW_INDIRECT_BUFFER, 0 );
		m_instancebuffers[ m_instancebufferindex ]->bind_base();
	}

	m_renderpass.draw_stats.instanced_drawcalls += static_cast<int>( m_indirectdraws.size() );
	m_renderpass.draw_stats.drawcalls += static_cast<int>( prepassdraws );
	// the count of the frame the gpu reported last
	m_renderpass.draw_stats.instances += static_cast<int>( m_instancecull.last_count() );
	m_renderpass.draw_stats.models += static_cast<int>( m_instancecull.last_count() );
	m_instancecull.draw_result(
		static_cast<std::uint32_t>( m_indirectdraws.size() ),
		prepassdraws,
		std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - cpustart ).count() );
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

void opengl46_renderer::Render_Sleepers( TTrack *Track )
{
	if( Track == nullptr ) { return; }
	if( false == Track->m_sleeper_enabled ) { return; }
	if( Track->m_sleeper_model == nullptr ) { return; }
	if( Track->m_sleeper_local_transforms.empty() ) { return; }
	if( Global.SleeperDistance <= 0.f ) { return; }

	// only the color and reflection passes draw sleepers; shadow/pick skip them on purpose
	// (sleeper shadows would mostly fall back under the trackbed and pick already operates on
	// the track itself).
	switch( m_renderpass.draw_mode ) {
	case rendermode::color:
	case rendermode::reflections:
		break;
	default:
		return;
	}

	// distance gate -- compare against Globals.SleeperDistance squared to avoid the sqrt
	auto const camerapos = m_renderpass.pass_camera.position();
	auto const trackpos  = Track->location();
	auto const distsq    = glm::length2( trackpos - camerapos );
	auto const cutoffsq  = static_cast<double>( Global.SleeperDistance ) * static_cast<double>( Global.SleeperDistance );
	if( distsq > cutoffsq ) { return; }

	// build camera-space modelview matrices.
	// each sleeper's stored matrix is in track-local space (relative to Track->m_origin).
	// Render_Sleepers is called from inside the per-cell origin push -- the cell's center
	// already equals Track->m_origin (see basic_cell::insert), so the current GL_MODELVIEW
	// is already view * translate(m_origin - camera). We just need to compose with each
	// per-sleeper local transform to get the final modelview.
	glm::mat4 const origin_mv = OpenGLMatrices.data( GL_MODELVIEW );

	// per-sleeper frustum cull + LOD selection. The whole-track SleeperDistance
	// gate above keeps or drops the track as a unit; everything from here on
	// operates on individual sleepers.
	//
	// Each m_sleeper_local_transforms entry is
	//   translate(world_pos - m_origin) * rotate(...) * translate(local_offset)
	// so its translation column is the sleeper position relative to m_origin;
	// adding m_origin back yields the sleeper's world-space position. The model's
	// bounding radius (floored to a small minimum, in case it was never measured)
	// serves both as the frustum test sphere and -- via the distance to that
	// world position -- as the per-sleeper LOD distance.
	auto const sleeperradius = std::max( Track->m_sleeper_model->bounding_radius(), 2.0f );

	// Phase 1 -- frustum cull. For every sleeper that survives, store its
	// camera-space modelview paired with the squared distance from the camera
	// to ITS OWN world position (not the track / segment origin). That
	// per-sleeper distance is what drives LOD selection below. A sleeper whose
	// origin sits just off screen while its geometry still reaches into view is
	// kept rather than wrongly culled.
	std::vector<std::pair<float, glm::mat4>> survivors;
	survivors.reserve( Track->m_sleeper_local_transforms.size() );
	for( auto const &local : Track->m_sleeper_local_transforms ) {
		glm::dvec3 const sleeperworldpos {
			Track->m_origin + glm::dvec3( local[ 3 ].x, local[ 3 ].y, local[ 3 ].z ) };
		if( false == m_renderpass.pass_camera.visible( scene::bounding_area{ sleeperworldpos, sleeperradius } ) ) {
			continue;
		}
		auto const sleeperdistancesquared = static_cast<float>( glm::length2( sleeperworldpos - camerapos ) );
		survivors.emplace_back( sleeperdistancesquared, origin_mv * local );
	}

	// every sleeper of this track was frustum-culled -- nothing left to draw
	if( survivors.empty() ) { return; }

	// Phase 2 -- sort survivors near-to-far. A submodel is drawn only while
	// fSquareDist lies inside its [fSquareMinDist, fSquareMaxDist) range, so
	// every distance between two consecutive range bounds selects an identical
	// set of submodels -- i.e. the same LOD. Sorting turns each such LOD band
	// into a contiguous run, letting the draw loop emit one instanced batch per
	// band instead of one track-wide batch at a single distance.
	std::sort( survivors.begin(), survivors.end(),
		[]( std::pair<float, glm::mat4> const &Left, std::pair<float, glm::mat4> const &Right ) {
			return Left.first < Right.first; } );

	// contiguous copy of the sorted modelview matrices, for the upload.
	// Per-sleeper distances stay available as survivors[i].first, in lockstep.
	std::vector<glm::mat4> instance_modelviews;
	instance_modelviews.reserve( survivors.size() );
	for( auto const &survivor : survivors ) {
		instance_modelviews.emplace_back( survivor.second );
	}

	// collect the model's distinct LOD distance bounds. The sorted, de-duplicated
	// set of every submodel's fSquareMinDist / fSquareMaxDist partitions distance
	// into bands within which the selected LOD is constant. A model with no LOD
	// yields a single band, and the draw loop below then behaves exactly like a
	// single plain batched draw.
	std::vector<float> lodbounds;
	{
		std::vector<TSubModel const *> pending;
		if( Track->m_sleeper_model->Root != nullptr ) {
			pending.push_back( Track->m_sleeper_model->Root );
		}
		while( false == pending.empty() ) {
			auto const *submodel = pending.back();
			pending.pop_back();
			lodbounds.emplace_back( submodel->fSquareMinDist );
			lodbounds.emplace_back( submodel->fSquareMaxDist );
			if( submodel->Child != nullptr ) { pending.push_back( submodel->Child ); }
			if( submodel->Next  != nullptr ) { pending.push_back( submodel->Next ); }
		}
	}
	std::sort( lodbounds.begin(), lodbounds.end() );
	lodbounds.erase( std::unique( lodbounds.begin(), lodbounds.end() ), lodbounds.end() );

	// optional replacable skin: build a transient material_data so we can drive ReplacableSet
	// the same way Render_Instanced does. when no skin is set we fall back to the model defaults.
	material_data sleeper_material {};
	bool const has_skin = ( Track->m_sleeper_skin != null_handle );
	if( has_skin ) {
		sleeper_material.replacable_skins[ 1 ] = Track->m_sleeper_skin;
	}

	// Phase 3 -- draw. Walk the sorted survivors one LOD band at a time. Each
	// band is submitted as one instanced draw from its own base instance; every
	// draw uses an in-band fSquareDist, so each sleeper renders at the LOD its own
	// distance selects while instancing stays fully in effect.
	auto *Model = Track->m_sleeper_model;
	std::size_t const total = instance_modelviews.size();
	auto const baseinstance { Upload_Instance_Modelviews( instance_modelviews.data(), total ) };
	std::size_t band_start = 0;
	while( band_start < total ) {
		// the band ends at the first sleeper distance that reaches the next LOD
		// bound above the band's starting distance.
		auto const upperbound = std::upper_bound(
			lodbounds.begin(), lodbounds.end(), survivors[ band_start ].first );
		float const band_limit = ( upperbound == lodbounds.end()
			? std::numeric_limits<float>::max()
			: *upperbound );
		std::size_t band_end = band_start;
		while( ( band_end < total ) && ( survivors[ band_end ].first < band_limit ) ) {
			++band_end;
		}
		// every distance in the band selects the same LOD; the band's nearest
		// sleeper is used as the representative fSquareDist.
		float const band_distancesquared = survivors[ band_start ].first;

		::glPushMatrix();
		::glLoadIdentity();

		m_current_instance_count = band_end - band_start;
		m_current_instance_base = baseinstance + static_cast<std::uint32_t>( band_start );

		Model->Root->fSquareDist = band_distancesquared;
		auto alpha = ( has_skin ? sleeper_material.textures_alpha : 0x30300030 );
		alpha ^= 0x0F0F000F;
		Model->Root->ReplacableSet( ( has_skin ? sleeper_material.replacable_skins : nullptr ), alpha );
		Model->Root->pRoot = Model;

		Render( Model->Root );

		m_current_instance_count = 0;
		m_current_instance_base = 0;

		::glPopMatrix();

		++m_renderpass.draw_stats.instanced_drawcalls;

		band_start = band_end;
	}

	m_renderpass.draw_stats.instances += static_cast<int>( total );
	m_renderpass.draw_stats.models    += static_cast<int>( total );
}

bool opengl46_renderer::Render(TDynamicObject *Dynamic)
{
	glDebug("Render TDynamicObject");

	if (!Global.render_cab && Global.pCamera.m_owner == Dynamic)
		return false;

    Dynamic->renderme = (
        ( Global.pCamera.m_owner == Dynamic && !FreeFlyModeFlag )
     || ( m_renderpass.pass_camera.visible( Dynamic ) ) );

    if (false == Dynamic->renderme) { return false; }

	// setup
	TSubModel::iInstance = reinterpret_cast<std::uintptr_t>(Dynamic); //żeby nie robić cudzych animacji
	TSubModel::iVariantSeed = Dynamic->variant_seed();
	glm::dvec3 const originoffset = Dynamic->vPosition - m_renderpass.pass_camera.position();
	// lod visibility ranges are defined for base (x 1.0) viewing distance. for render we adjust them for actual range multiplier and zoom
	float squaredistance;
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
	{
		squaredistance = glm::length2(glm::vec3{glm::dvec3{Dynamic->vPosition - m_renderpass.viewport_camera.position()}} / Global.ZoomFactor);
        if( false == FreeFlyModeFlag ) {
            // filter out small details if we're in vehicle cab
            squaredistance = std::max( 100.f * 100.f, squaredistance );
        }
		break;
	}
    case rendermode::reflections:
    {
        // reflection mode draws simplified version of the shapes, by artificially increasing view range
        // it also ignores zoom settings and distance multipliers
        squaredistance =
            std::max(
                100.f * 100.f,
                // TBD: take into account distance multipliers?
                glm::length2( glm::vec3{ originoffset } ) /* / Global.fDistanceFactor */ );
        // NOTE: arbitrary draw range limit
        if( squaredistance > Global.reflectiontune.range_vehicles * Global.reflectiontune.range_vehicles ) {
            return false;
        }
        // TBD, TODO: bind offset value with setting variable?
        // NOTE: combined 'squared' distance doesn't equal actual squared (distance + offset) but, eh
        squaredistance += sq(EU07_REFLECTIONFIDELITYOFFSET);
        break;
    }
	default:
	{
		squaredistance = glm::length2(glm::vec3{originoffset} / Global.ZoomFactor);
        // TODO: filter out small details based on fidelity setting
		break;
	}
	}
    // crude way to reject early items too far to affect the output (mostly relevant for shadow and reflection passes)
    auto const drawdistancethreshold{ m_renderpass.draw_range + 250 };
    if( squaredistance > sq(drawdistancethreshold) ) {
        return false;
    }
    // second stage visibility cull, reject vehicles too far away to be noticeable
    auto const squareradius{ Dynamic->radius() * Dynamic->radius() };
    Dynamic->renderme = ( squareradius * Global.ZoomFactor / squaredistance > 0.003 * 0.003 );
    if( false == Dynamic->renderme ) {
        return false;
    }

    // debug data
    ++m_renderpass.draw_stats.dynamics;

	Dynamic->ABuLittleUpdate(squaredistance); // ustawianie zmiennych submodeli dla wspólnego modelu

	glm::mat4 future_stack = model_ubs.future;

	glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
	model_ubs.future *= glm::translate(mv, glm::vec3(Dynamic->get_future_movement())) * glm::inverse(mv);

	::glPushMatrix();
	::glTranslated(originoffset.x, originoffset.y, originoffset.z);
	::glMultMatrixd(glm::value_ptr(Dynamic->mMatrix));

	switch (m_renderpass.draw_mode)
	{

	case rendermode::color:
    case rendermode::reflections:
	{
		if (Dynamic->fShade > 0.0f)
		{
			// change light level based on light level of the occupied track
			setup_sunlight_intensity(Dynamic->fShade);
        }

		// render
        if( Dynamic->mdLowPolyInt ) {
            // low poly interior
            Render_lowpoly( Dynamic, squaredistance, false );
        }
        else {
            // HACK: if the model lacks low poly interior, we presume the load is placed outside
            if( Dynamic->mdLoad ) {
                // renderowanie nieprzezroczystego ładunku
                Render( Dynamic->mdLoad, Dynamic->Material(), squaredistance, { 0.f, Dynamic->LoadOffset, 0.f }, {} );
            }
        }
        if( Dynamic->mdModel ) {
            // main model
            Render( Dynamic->mdModel, Dynamic->Material(), squaredistance );
        }
        // optional attached models
        for( auto *attachment : Dynamic->mdAttachments ) {
            Render( attachment, Dynamic->Material(), squaredistance );
        }
        // optional coupling adapters
        Render_coupler_adapter( Dynamic, squaredistance, end::front );
        Render_coupler_adapter( Dynamic, squaredistance, end::rear );

        // post-render cleanup
		if (Dynamic->fShade > 0.0f)
		{
			// restore regular light level
			setup_sunlight_intensity();
        }
		break;
	}
	case rendermode::shadows:
	{
        if( Dynamic->mdLowPolyInt ) {
            // low poly interior
            Render( Dynamic->mdLowPolyInt, Dynamic->Material(), squaredistance );
        }
        if( Dynamic->mdModel ) {
            // main model
            Render( Dynamic->mdModel, Dynamic->Material(), squaredistance );
        }
        // optional attached models
        for( auto *attachment : Dynamic->mdAttachments ) {
            Render( attachment, Dynamic->Material(), squaredistance );
        }
        // optional coupling adapters
        Render_coupler_adapter( Dynamic, squaredistance, end::front );
        Render_coupler_adapter( Dynamic, squaredistance, end::rear );
        if( Dynamic->mdLoad ) {
            // renderowanie nieprzezroczystego ładunku
            Render( Dynamic->mdLoad, Dynamic->Material(), squaredistance, { 0.f, Dynamic->LoadOffset, 0.f }, {} );
        }
		// post-render cleanup
		break;
	}
	case rendermode::pickcontrols:
	{
		if (Dynamic->mdLowPolyInt) {
			// low poly interior
			Render(Dynamic->mdLowPolyInt, Dynamic->Material(), squaredistance);
		}
		break;
	}
	case rendermode::pickscenery:
	default:
	{
		break;
	}
	}

	::glPopMatrix();

	model_ubs.future = future_stack;

	// TODO: check if this reset is needed. In theory each object should render all parts based on its own instance data anyway?
	if (Dynamic->btnOn)
		Dynamic->TurnOff(); // przywrócenie domyślnych pozycji submodeli

	return true;
}

// rendering kabiny gdy jest oddzielnym modelem i ma byc wyswietlana
bool opengl46_renderer::Render_cab(TDynamicObject const *Dynamic, float const Lightlevel, bool const Alpha)
{

	if (Dynamic == nullptr)
	{

		TSubModel::iInstance = 0;
		TSubModel::iVariantSeed = 0;
		return false;
	}

	TSubModel::iInstance = reinterpret_cast<std::uintptr_t>(Dynamic);
	TSubModel::iVariantSeed = Dynamic->variant_seed();

	if ((true == FreeFlyModeFlag) || (false == Dynamic->bDisplayCab) || (Dynamic->mdKabina == Dynamic->mdModel))
	{
		// ABu: Rendering kabiny jako ostatniej, zeby bylo widac przez szyby, tylko w widoku ze srodka
		return false;
	}

	if (Dynamic->mdKabina)
	{ // bo mogła zniknąć przy przechodzeniu do innego pojazdu
		// setup shared by all render paths
		::glPushMatrix();

		auto const originoffset = Dynamic->GetPosition() - m_renderpass.pass_camera.position();
		::glTranslated(originoffset.x, originoffset.y, originoffset.z);
		::glMultMatrixd(glm::value_ptr(Dynamic->mMatrix));

        switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		{
			// render path specific setup:
			if (Dynamic->fShade > 0.0f)
			{
				// change light level based on light level of the occupied track
				setup_sunlight_intensity(Dynamic->fShade);
            }

            auto const old_ambient { light_data.ambient };
            auto const luminance { Global.fLuminance * ( Dynamic->fShade > 0.0f ? Dynamic->fShade : 1.0f ) };
            if( Lightlevel > 0.f ) {
                // crude way to light the cabin, until we have something more complete in place. with the lamp of the cab among the
                // lights a share of it is kept for the light which bounces around the cab
                auto const share { m_cablamplit ? Global.gfx_lights_cab_bounce : 1.f };
                light_data.ambient += packed_vec3( ( Dynamic->InteriorLight * Lightlevel ) * std::clamp( 1.25f - (float)luminance, 0.f, 1.f ) * share );
                upload_light_header();
            }

			// render
			if (true == Alpha)
			{
				// translucent parts
				Render_Alpha(Dynamic->mdKabina, Dynamic->Material(), 0.0);
			}
			else
			{
				// opaque parts
				Render(Dynamic->mdKabina, Dynamic->Material(), 0.0);
			}
			// post-render restore
            light_data.ambient = old_ambient;
            setup_sunlight_intensity();

			break;
		}
        case rendermode::shadows:
        {
            if( true == Alpha ) {
                // translucent parts
                Render_Alpha( Dynamic->mdKabina, Dynamic->Material(), 0.0 );
            }
            else {
                // opaque parts
                Render( Dynamic->mdKabina, Dynamic->Material(), 0.0 );
            }
            break;
        }
		case rendermode::pickcontrols:
		{
			Render(Dynamic->mdKabina, Dynamic->Material(), 0.0);
			break;
		}
		default:
		{
			break;
		}
		}
		// post-render restore
		::glPopMatrix();
	}

	return true;
}

bool opengl46_renderer::Render(TModel3d *Model, material_data const *Material, float const Squaredistance)
{

	auto alpha = (Material != nullptr ? Material->textures_alpha : 0x30300030);
	alpha ^= 0x0F0F000F; // odwrócenie flag tekstur, aby wyłapać nieprzezroczyste
	if (0 == (alpha & Model->iFlags & 0x1F1F001F))
	{
		// czy w ogóle jest co robić w tym cyklu?
		return false;
	}

	Model->Root->fSquareDist = Squaredistance; // zmienna globalna!

	// setup
	Model->Root->ReplacableSet((Material != nullptr ? Material->replacable_skins : nullptr), alpha);

	Model->Root->pRoot = Model;

	// render
	Render(Model->Root);

	// post-render cleanup

	return true;
}

bool opengl46_renderer::Render(TModel3d *Model, material_data const *Material, float const Squaredistance, glm::dvec3 const &Position, glm::vec3 const &Angle)
{
	::glPushMatrix();
	::glTranslated(Position.x, Position.y, Position.z);
	if (Angle.y != 0.0)
		::glRotated(Angle.y, 0.f, 1.f, 0.f);
	if (Angle.x != 0.0)
		::glRotated(Angle.x, 1.f, 0.f, 0.f);
	if (Angle.z != 0.0)
		::glRotated(Angle.z, 0.f, 0.f, 1.f);

	auto const result = Render(Model, Material, Squaredistance);

	::glPopMatrix();

	return result;
}

void opengl46_renderer::Render(TSubModel *Submodel)
{
	glDebug("Render TSubModel");

	if ((Submodel->iVisible) && (TSubModel::fSquareDist >= Submodel->fSquareMinDist) && (TSubModel::fSquareDist < Submodel->fSquareMaxDist) && (false == m_previewing || Submodel->fVisible > 0.f))
	{
		glm::mat4 future_stack = model_ubs.future;

		if (Submodel->iFlags & 0xC000)
		{
			::glPushMatrix();
			if (Submodel->fMatrix)
				::glMultMatrixf(Submodel->fMatrix->readArray());
			if (Submodel->b_aAnim != TAnimType::at_None)
			{
				Submodel->RaAnimation(Submodel->b_aAnim);

				glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
				model_ubs.future *= (mv * Submodel->future_transform) * glm::inverse(mv);
			}
		}

		if (Submodel->eType < TP_ROTATOR)
		{
			// renderowanie obiektów OpenGL
			if (Submodel->iAlpha & Submodel->iFlags & 0x1F)
			{
				// rysuj gdy element nieprzezroczysty

		        // debug data
		        ++m_renderpass.draw_stats.submodels;
		        ++m_renderpass.draw_stats.drawcalls;

				switch (m_renderpass.draw_mode)
				{
				case rendermode::color:
				case rendermode::reflections:
				{
					// material configuration:
					// transparency hack
					if (Submodel->fVisible < 1.0f)
						setup_drawing(true);

					// textures...
					if (Submodel->m_material < 0)
					{ // zmienialne skóry
						Bind_Material(submodel_material(Submodel), Submodel);
					}
					else
					{
						// również 0
						Bind_Material(Submodel->m_material, Submodel);
					}

					// ...luminance
					auto const isemissive { ( Submodel->f4Emision.a > 0.f ) && ( Global.fLuminance < Submodel->fLight ) };
					if (isemissive)
						model_ubs.emission = Submodel->f4Emision.a;

					// main draw call
					draw(Submodel->m_geometry.handle);

					// post-draw reset
					model_ubs.emission = 0.0f;
					if (Submodel->fVisible < 1.0f)
						setup_drawing(false);

					break;
				}
				case rendermode::shadows:
				{
                    // skip if the shadow caster rank is too low for currently set threshold
                    // or if the submodel is hidden (colour pass only fades it out, e.g. inactive seasonal variant)
                    if( ( Material( Submodel ).shadow_rank > Global.gfx_shadow_rank_cutoff )
                     || ( Submodel->fVisible <= 0.f ) )
                    {
                        --m_renderpass.draw_stats.submodels;
                        --m_renderpass.draw_stats.drawcalls;
                        break;
                    }

                    if (Submodel->m_material < 0)
					{ // zmienialne skóry
						Bind_Material_Shadow(submodel_material(Submodel));
					}
					else
					{
						// również 0
						Bind_Material_Shadow(Submodel->m_material);
					}
                    draw(Submodel->m_geometry.handle);
                    break;
				}
				case rendermode::pickscenery:
				{
					m_pick_shader->bind();
					draw(Submodel->m_geometry.handle);
					break;
				}
				case rendermode::pickcontrols:
				{
				    if (Submodel->screen_touch_list) {
                        // touch screen gradient
                        m_pick_surface_shader->bind();
                        model_ubs.param[0] = glm::vec4(1.0f - m_picksurfaceitems.size() / 255.0f, 0.0f, 0.0f, 1.0f);
                        m_picksurfaceitems.emplace_back(Submodel);
                    }
                    else {
                        // control picking applies individual colour for each submodel
                        m_pick_shader->bind();
                        m_pickcontrolsitems.emplace_back(Submodel);
                        model_ubs.param[0] = glm::vec4(pick_color(m_pickcontrolsitems.size()), 1.0f);
                    }

					draw(Submodel->m_geometry.handle);
					break;
				}
				default:
				{
					break;
				}
				}
			}
		}
		else if (Submodel->eType == TP_STARS)
		{

			switch (m_renderpass.draw_mode)
			{
			// colour points are only rendered in colour mode(s)
			case rendermode::color:
			case rendermode::reflections:
			{
				if (Global.fLuminance < Submodel->fLight)
				{
					Bind_Material(Submodel->m_material, Submodel);

					// main draw call
                    model_ubs.param[1].x = 8.0f;

					draw(Submodel->m_geometry.handle);
				}
				break;
			}
			default:
			{
				break;
			}
			}
		}
		if (Submodel->Child != nullptr)
			if (Submodel->iAlpha & Submodel->iFlags & 0x001F0000)
				Render(Submodel->Child);

		if (Submodel->iFlags & 0xC000)
		{
			model_ubs.future = future_stack;
			::glPopMatrix();
		}
	}
	/*
	    if( Submodel->b_Anim < at_SecondsJump )
	        Submodel->b_Anim = at_None; // wyłączenie animacji dla kolejnego użycia subm
	*/
	if (Submodel->Next)
		if (Submodel->iAlpha & Submodel->iFlags & 0x1F000000)
			Render(Submodel->Next); // dalsze rekurencyjnie
}

void opengl46_renderer::Render(TTrack *Track)
{
	if ((Track->m_material1 == 0) && (Track->m_material2 == 0) && (Track->eType != tt_Switch || Track->SwitchExtension->m_material3 == 0))
	{
		return;
	}
	if (false == Track->m_visible)
	{
		return;
	}

	++m_renderpass.draw_stats.paths;
	++m_renderpass.draw_stats.drawcalls;

	switch (m_renderpass.draw_mode)
	{
	// single path pieces are rendererd in pick scenery mode only
	case rendermode::pickscenery:
	{
        m_pick_shader->bind();
        m_picksceneryitems.emplace_back( Track );

		draw(std::begin(Track->Geometry1), std::end(Track->Geometry1));
		draw(std::begin(Track->Geometry2), std::end(Track->Geometry2));
		if (Track->eType == tt_Switch)
			draw(Track->SwitchExtension->Geometry3);
		break;
	}
	default:
	{
		break;
	}
	}
}

// experimental, does track rendering in two passes, to take advantage of reduced texture switching
void opengl46_renderer::Render(scene::basic_cell::path_sequence::const_iterator First, scene::basic_cell::path_sequence::const_iterator Last)
{

	// setup
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
	{
		// NOTE: roads-based platforms tend to miss parts of shadows if rendered with either back or front culling
		glDisable(GL_CULL_FACE);
		break;
	}
	default:
	{
		break;
	}
	}

	// geometry of neighbouring tracks which share the material is gathered and drawn together.
	// only neighbours are combined, the tracks are still drawn in their order in the cell
	material_handle batchmaterial{null_handle};
	auto const flush_batch = [&]()
	{
		if (false == m_pathbatch.empty())
		{
			draw(std::begin(m_pathbatch), std::end(m_pathbatch));
			m_pathbatch.clear();
		}
		batchmaterial = null_handle;
	};
	// returns: true if the material starts a new batch, false if it continues the current one
	auto const begin_batch = [&](material_handle const Material)
	{
		if (Material == batchmaterial)
		{
			return false;
		}
		flush_batch();
		Bind_Material(Material);
		batchmaterial = Material;
		return true;
	};

	// TODO: render auto generated trackbeds together with regular trackbeds in pass 1, and all rails in pass 2
	// first pass, material 1
	for (auto first{First}; first != Last; ++first)
	{

		auto const track{*first};

		if (track->m_material1 == 0)
		{
			continue;
		}
		if (false == track->m_visible)
		{
			continue;
		}

		++m_renderpass.draw_stats.paths;
		++m_renderpass.draw_stats.drawcalls;

		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		case rendermode::reflections:
		{
			if (track->eEnvironment != e_flat)
			{
				// track with light level of its own is drawn on its own
				flush_batch();
				setup_environment_light(track->eEnvironment);
				Bind_Material(track->m_material1);
				draw(std::begin(track->Geometry1), std::end(track->Geometry1));
				// restore default lighting
				setup_environment_light();
				break;
			}
			if (false == begin_batch(track->m_material1))
			{
				// the track is drawn with the call of its predecessor
				--m_renderpass.draw_stats.drawcalls;
			}
			m_pathbatch.insert(std::end(m_pathbatch), std::begin(track->Geometry1), std::end(track->Geometry1));
			break;
		}
		case rendermode::shadows:
        {
			if ((std::abs(track->fTexHeight1) < 0.35f) || (track->iCategoryFlag != 2))
			{
				// shadows are only calculated for high enough roads, typically meaning track platforms
				--m_renderpass.draw_stats.paths;
				--m_renderpass.draw_stats.drawcalls;
				continue;
			}
            if( Material( track->m_material1 ).shadow_rank > Global.gfx_shadow_rank_cutoff ) {
                // skip if the shadow caster rank is too low for currently set threshold
                --m_renderpass.draw_stats.paths;
                --m_renderpass.draw_stats.drawcalls;
                continue;
            }
			Bind_Material_Shadow(track->m_material1);
			draw(std::begin(track->Geometry1), std::end(track->Geometry1));
			break;
		}
		case rendermode::pickscenery: // pick scenery mode uses piece-by-piece approach
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}
	}
	flush_batch();
	// second pass, material 2
	for (auto first{First}; first != Last; ++first)
	{
		auto const track{*first};

		if (track->m_material2 == 0)
		{
			continue;
		}
		if (false == track->m_visible)
		{
			continue;
		}

		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		case rendermode::reflections:
		{
			if (track->eEnvironment != e_flat)
			{
				flush_batch();
				setup_environment_light(track->eEnvironment);
				Bind_Material(track->m_material2);
				draw(std::begin(track->Geometry2), std::end(track->Geometry2));
				// restore default lighting
				setup_environment_light();
				break;
			}
			begin_batch(track->m_material2);
			m_pathbatch.insert(std::end(m_pathbatch), std::begin(track->Geometry2), std::end(track->Geometry2));
			break;
		}
		case rendermode::shadows:
        {
			if ((std::abs(track->fTexHeight1) < 0.35f) || ((track->iCategoryFlag == 1) && (track->eType != tt_Normal)))
			{
				// shadows are only calculated for high enough trackbeds
				continue;
			}
            if( Material( track->m_material2 ).shadow_rank > Global.gfx_shadow_rank_cutoff ) {
                // skip if the shadow caster rank is too low for currently set threshold
                continue;
            }
			Bind_Material_Shadow(track->m_material2);
			draw(std::begin(track->Geometry2), std::end(track->Geometry2));
			break;
		}
		case rendermode::pickscenery: // pick scenery mode uses piece-by-piece approach
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}
	}
	flush_batch();

	// third pass, material 3
	for (auto first{First}; first != Last; ++first)
	{

		auto const track{*first};

		if (track->eType != tt_Switch)
		{
			continue;
		}
		if (track->SwitchExtension->m_material3 == 0)
		{
			continue;
		}
		if (false == track->m_visible)
		{
			continue;
		}

		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		case rendermode::reflections:
		{
			if (track->eEnvironment != e_flat)
			{
				flush_batch();
				setup_environment_light(track->eEnvironment);
				Bind_Material(track->SwitchExtension->m_material3);
				draw(track->SwitchExtension->Geometry3);
				// restore default lighting
				setup_environment_light();
				break;
			}
			begin_batch(track->SwitchExtension->m_material3);
			m_pathbatch.emplace_back(track->SwitchExtension->Geometry3);
			break;
		}
		case rendermode::shadows:
        {
			if ((std::abs(track->fTexHeight1) < 0.35f) || ((track->iCategoryFlag == 1) && (track->eType != tt_Normal)))
			{
				// shadows are only calculated for high enough trackbeds
				continue;
			}
            if( Material( track->m_material2 ).shadow_rank > Global.gfx_shadow_rank_cutoff ) {
                // skip if the shadow caster rank is too low for currently set threshold
                continue;
            }
			Bind_Material_Shadow(track->SwitchExtension->m_material3);
			draw(track->SwitchExtension->Geometry3);
			break;
		}
		case rendermode::pickscenery: // pick scenery mode uses piece-by-piece approach
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}
	}
	flush_batch();

	// fourth pass: per-track sleeper models (sleepermodel optional directive).
	// drawn after rails/trackbeds so depth pre-pass culling is favourable, and only in passes
	// where Render_Sleepers actually does work (it gates itself on draw mode / distance).
	switch( m_renderpass.draw_mode ) {
	case rendermode::color:
	case rendermode::reflections: {
		for( auto first { First }; first != Last; ++first ) {
			auto *track = *first;
			if( false == track->m_visible )       { continue; }
			if( false == track->m_sleeper_enabled ) { continue; }
			Render_Sleepers( track );
		}
		break;
	}
	default:
		break;
	}

	// post-render reset
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
    {
		// restore standard face cull mode
		::glEnable(GL_CULL_FACE);
		break;
	}
	default:
	{
		break;
	}
	}
}

void opengl46_renderer::Render(TMemCell *Memcell)
{

	::glPushMatrix();
	auto const position = Memcell->location() - m_renderpass.pass_camera.position();
	::glTranslated(position.x, position.y + 0.5, position.z);

	switch (m_renderpass.draw_mode)
	{
	case rendermode::color:
	{
		break;
	}
	case rendermode::shadows:
    case rendermode::pickscenery:
	{
		break;
	}
	case rendermode::reflections:
	case rendermode::pickcontrols:
	{
		break;
	}
	default:
	{
		break;
	}
	}

	::glPopMatrix();
}

void opengl46_renderer::Render_particles()
{
	model_ubs.set_modelview(OpenGLMatrices.data(GL_MODELVIEW));
	model_ubo->update(model_ubs);

	Bind_Texture(0, m_smoketexture);
	m_particlerenderer.update(m_renderpass.pass_camera);
    m_renderpass.draw_stats.particles = m_particlerenderer.render();
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

void opengl46_renderer::Render_Alpha(scene::basic_region *Region)
{

	// sort the nodes based on their distance to viewer
	std::sort(std::begin(m_cellqueue), std::end(m_cellqueue), [](distancecell_pair const &Left, distancecell_pair const &Right) { return (Left.first) < (Right.first); });

	Render_Alpha(std::rbegin(m_cellqueue), std::rend(m_cellqueue));
}

void opengl46_renderer::Render_Alpha(cell_sequence::reverse_iterator First, cell_sequence::reverse_iterator Last)
{

	// NOTE: this method is launched only during color pass therefore we don't bother with mode test here
	// first pass draws elements which we know are located in section banks, to reduce vbo switching
	{
		auto first{First};
		while (first != Last)
		{

			auto const *cell = first->second;

			if (false == cell->m_shapestranslucent.empty())
			{
				// since all shapes of the cell share center point we can optimize out a few calls here
				::glPushMatrix();
				auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
				::glTranslated(originoffset.x, originoffset.y, originoffset.z);
				// render
				// NOTE: we can reuse the method used to draw opaque geometry
				for (auto const &shape : cell->m_shapestranslucent)
				{
					Render(shape, false);
				}
				// post-render cleanup
				::glPopMatrix();
			}

			++first;
		}
	}
	// second pass draws elements with their own vbos
	{
		auto first{First};
		while (first != Last)
		{

			auto const *cell = first->second;

			// translucent parts of instanced models
			for (auto *instance : cell->m_instancetranslucent)
			{
				Render_Alpha(instance);
			}
			// translucent parts of vehicles
			for (auto *path : cell->m_paths)
			{
				for (auto *dynamic : path->Dynamics)
				{
					Render_Alpha(dynamic);
				}
			}

			++first;
		}
	}
	// third pass draws the wires;
	// wires use section vbos, but for the time being we want to draw them at the very end
	{
		auto first{First};
		while (first != Last)
		{

			auto const *cell = first->second;

			if ((false == cell->m_traction.empty() || (false == cell->m_lines.empty())))
			{
				// since all shapes of the cell share center point we can optimize out a few calls here
				::glPushMatrix();
				auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
				::glTranslated(originoffset.x, originoffset.y, originoffset.z);
				Bind_Material(null_handle);
				// render
				for (auto *traction : cell->m_traction)
				{
					Render_Alpha(traction);
				}
				for (auto &lines : cell->m_lines)
				{
					Render_Alpha(lines);
				}
				// post-render cleanup
				::glPopMatrix();
			}

			++first;
		}
	}
}

void opengl46_renderer::Render_Alpha(TAnimModel *Instance)
{

	if (false == Instance->m_visible) { return; }
    if( false == m_renderpass.pass_camera.visible( Instance->m_area ) ) { return; }

    double distancesquared;
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
    {
        // 'camera' for the light pass is the light source, but we need to draw what the 'real' camera sees
		distancesquared = glm::length2((Instance->location() - m_renderpass.viewport_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
		break;
	}
	default:
	{
		distancesquared = glm::length2((Instance->location() - m_renderpass.pass_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
		break;
	}
	}
	if ((distancesquared < Instance->m_rangesquaredmin) || (distancesquared >= Instance->m_rangesquaredmax))
	{
		return;
	}
     // crude way to reject early items too far to affect the output (mostly relevant for shadow passes)
    auto const drawdistancethreshold{ m_renderpass.draw_range + 250 };
    if( distancesquared > sq(drawdistancethreshold) ) {
        return;
    }
   // second stage visibility cull, reject modelstoo far away to be noticeable
    auto const radiussquared { Instance->radius() * Instance->radius() };
    if( radiussquared * Global.ZoomFactor / distancesquared < 0.003 * 0.003 ) {
        return;
    }

	Instance->RaPrepare();
	if (Instance->pModel)
	{
		auto const &scale = Instance->Scale();
		bool const scaled = ( scale.x != 1.0f || scale.y != 1.0f || scale.z != 1.0f );
		if( scaled ) {
			// scaled instance: build the per-instance transform locally and call
			// the distance-only Render_Alpha so we can inject glScalef before
			// the submodel walk. Mirror of the opaque path in Render(TAnimModel*).
			::glPushMatrix();
			auto const Position = Instance->location() - m_renderpass.pass_camera.position();
			auto const Angle = Instance->vAngle;
			::glTranslated( Position.x, Position.y, Position.z );
			if( Angle.y != 0.0 ) ::glRotated( Angle.y, 0.f, 1.f, 0.f );
			if( Angle.x != 0.0 ) ::glRotated( Angle.x, 1.f, 0.f, 0.f );
			if( Angle.z != 0.0 ) ::glRotated( Angle.z, 0.f, 0.f, 1.f );
			::glScalef( scale.x, scale.y, scale.z );
			Render_Alpha( Instance->pModel, Instance->Material(), distancesquared );
			::glPopMatrix();
		}
		else {
			Render_Alpha(Instance->pModel, Instance->Material(), distancesquared, Instance->location() - m_renderpass.pass_camera.position(), Instance->vAngle);
		}
	}
}

void opengl46_renderer::Render_Alpha(TTraction *Traction)
{
	glDebug("Render_Alpha TTraction");

	auto const distancesquared { glm::length2( ( Traction->location() - m_renderpass.pass_camera.position() ) / (double)Global.ZoomFactor ) / Global.fDistanceFactor };
	if ((distancesquared < Traction->m_rangesquaredmin) || (distancesquared >= Traction->m_rangesquaredmax))
	{
		return;
	}

	if (false == Traction->m_visible)
	{
		return;
	}
	// rysuj jesli sa druty i nie zerwana
	if ((Traction->Wires == 0) || (true == TestFlag(Traction->DamageFlag, 128)))
	{
		return;
	}
	// setup
	auto const distance{static_cast<float>(std::sqrt(distancesquared))};
	auto const linealpha = 20.f * Traction->WireThickness / std::max(0.5f * Traction->radius() + 1.f, distance - (0.5f * Traction->radius()));

	// render

	// McZapkie-261102: kolor zalezy od materialu i zasniedzenia
	model_ubs.param[0] = glm::vec4(Traction->wire_color(), glm::min(1.0f, linealpha));

	if (m_renderpass.draw_mode == rendermode::shadows)
		Bind_Material_Shadow(null_handle);
	else
		m_line_shader->bind();

	draw(Traction->m_geometry);

	// debug data
	++m_renderpass.draw_stats.traction;
	++m_renderpass.draw_stats.drawcalls;
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

void opengl46_renderer::Render_Alpha(scene::lines_node const &Lines)
{
	glDebug("Render_Alpha scene::lines_node");

	auto const &data{Lines.data()};

	auto const distancesquared { glm::length2( ( data.area.center - m_renderpass.pass_camera.position() ) / (double)Global.ZoomFactor ) / Global.fDistanceFactor };
	if ((distancesquared < data.rangesquared_min) || (distancesquared >= data.rangesquared_max))
	{
		return;
	}
	// setup
	auto const distance{static_cast<float>(std::sqrt(distancesquared))};
	auto const linealpha =
	    (data.line_width > 0.f ? 10.f * data.line_width / std::max(0.5f * data.area.radius + 1.f, distance - (0.5f * data.area.radius)) : 1.f); // negative width means the lines are always opague

	model_ubs.param[0] = glm::vec4(glm::vec3(data.lighting.diffuse * m_sunlight.ambient), glm::min(1.0f, linealpha));

	if (m_renderpass.draw_mode == rendermode::shadows)
		Bind_Material_Shadow(null_handle);
	else
		m_line_shader->bind();
	draw(data.geometry);

	++m_renderpass.draw_stats.lines;
	++m_renderpass.draw_stats.drawcalls;
}

bool opengl46_renderer::Render_Alpha(TDynamicObject *Dynamic)
{
	if (!Global.render_cab && Global.pCamera.m_owner == Dynamic)
		return false;

	if (false == Dynamic->renderme)
	{
		return false;
	}

	// setup
	TSubModel::iInstance = (size_t)Dynamic; //żeby nie robić cudzych animacji
	TSubModel::iVariantSeed = Dynamic->variant_seed();
	glm::dvec3 const originoffset = Dynamic->vPosition - m_renderpass.pass_camera.position();
	// lod visibility ranges are defined for base (x 1.0) viewing distance. for render we adjust them for actual range multiplier and zoom
	float squaredistance;
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
	default:
	{
		squaredistance = glm::length2(glm::vec3{originoffset} / Global.ZoomFactor);
		break;
	}
	}
	Dynamic->ABuLittleUpdate(squaredistance); // ustawianie zmiennych submodeli dla wspólnego modelu

	glm::mat4 future_stack = model_ubs.future;

	glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
	model_ubs.future *= glm::translate(mv, glm::vec3(Dynamic->get_future_movement())) * glm::inverse(mv);

	::glPushMatrix();

	::glTranslated(originoffset.x, originoffset.y, originoffset.z);
	::glMultMatrixd(glm::value_ptr(Dynamic->mMatrix));

	if (Dynamic->fShade > 0.0f)
	{
		// change light level based on light level of the occupied track
		setup_sunlight_intensity(Dynamic->fShade);
	}

    // render
    if( Dynamic->mdLowPolyInt ) {
        // low poly interior
        Render_Alpha( Dynamic->mdLowPolyInt, Dynamic->Material(), squaredistance );
    }
    if( Dynamic->mdModel ) {
        // main model
        Render_Alpha( Dynamic->mdModel, Dynamic->Material(), squaredistance );
    }
    // optional attached models
    for( auto *attachment : Dynamic->mdAttachments ) {
        Render_Alpha( attachment, Dynamic->Material(), squaredistance );
    }
    // optional coupling adapters
    Render_coupler_adapter( Dynamic, squaredistance, end::front, true );
    Render_coupler_adapter( Dynamic, squaredistance, end::rear, true );
    if( Dynamic->mdLoad ) {
        // renderowanie nieprzezroczystego ładunku
        Render_Alpha( Dynamic->mdLoad, Dynamic->Material(), squaredistance, { 0.f, Dynamic->LoadOffset, 0.f }, {} );
    }

	// post-render cleanup
	if (Dynamic->fShade > 0.0f)
	{
		// restore regular light level
		setup_sunlight_intensity();
	}

	::glPopMatrix();

	model_ubs.future = future_stack;

	if (Dynamic->btnOn)
		Dynamic->TurnOff(); // przywrócenie domyślnych pozycji submodeli

	return true;
}

bool opengl46_renderer::Render_Alpha(TModel3d *Model, material_data const *Material, float const Squaredistance)
{

	auto alpha = (Material != nullptr ? Material->textures_alpha : 0x30300030);

	if (0 == (alpha & Model->iFlags & 0x2F2F002F))
	{
		// nothing to render
		return false;
	}

	Model->Root->fSquareDist = Squaredistance; // zmienna globalna!

	// setup
	Model->Root->ReplacableSet((Material != nullptr ? Material->replacable_skins : nullptr), alpha);

	Model->Root->pRoot = Model;

	// render
	Render_Alpha(Model->Root);

	// post-render cleanup

	return true;
}

bool opengl46_renderer::Render_Alpha(TModel3d *Model, material_data const *Material, float const Squaredistance, glm::dvec3 const &Position, glm::vec3 const &Angle)
{
	::glPushMatrix();
	::glTranslated(Position.x, Position.y, Position.z);
	if (Angle.y != 0.0)
		::glRotated(Angle.y, 0.f, 1.f, 0.f);
	if (Angle.x != 0.0)
		::glRotated(Angle.x, 1.f, 0.f, 0.f);
	if (Angle.z != 0.0)
		::glRotated(Angle.z, 0.f, 0.f, 1.f);

	auto const result = Render_Alpha(Model, Material, Squaredistance); // position is effectively camera offset

	::glPopMatrix();

	return result;
}

// characters of a text display, laid out in a row. the text can be encoded in utf-8 or in windows-1250
void opengl46_renderer::Render_Alpha_text(TSubModel *Submodel)
{
	if (TSubModel::pasText == nullptr)
	{
		return;
	}
	for (auto *p : Submodel->text_letters(*TSubModel::pasText))
	{ // translucent only, for the time being
		Render_Alpha(p);
		if (p->fMatrix)
			::glMultMatrixf(p->fMatrix->readArray()); // move on to the place of the next character
	}
}

void opengl46_renderer::Render_Alpha(TSubModel *Submodel)
{
	// renderowanie przezroczystych przez DL
	if ((Submodel->iVisible) && (TSubModel::fSquareDist >= Submodel->fSquareMinDist) && (TSubModel::fSquareDist < Submodel->fSquareMaxDist) && (false == m_previewing || Submodel->fVisible > 0.f))
	{
		glm::mat4 future_stack = model_ubs.future;

		if (Submodel->iFlags & 0xC000)
		{
			::glPushMatrix();
			if (Submodel->fMatrix)
				::glMultMatrixf(Submodel->fMatrix->readArray());
			if (Submodel->b_aAnim != TAnimType::at_None)
			{
				Submodel->RaAnimation(Submodel->b_aAnim);

				glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
				model_ubs.future *= (mv * Submodel->future_transform) * glm::inverse(mv);
			}
		}

		if (Submodel->eType < TP_ROTATOR)
		{
			// renderowanie obiektów OpenGL
			if (Submodel->iAlpha & Submodel->iFlags & 0x2F)
			{
                // rysuj gdy element przezroczysty

                // debug data
		        ++m_renderpass.draw_stats.submodels;
		        ++m_renderpass.draw_stats.drawcalls;

				switch (m_renderpass.draw_mode)
				{
				case rendermode::color:
				{
					// material configuration:
					// textures...
					if (Submodel->m_material < 0)
					{ // zmienialne skóry
						Bind_Material(submodel_material(Submodel), Submodel);
					}
					else
					{
						Bind_Material(Submodel->m_material, Submodel);
					}
					// ...luminance

					auto const isemissive { ( Submodel->f4Emision.a > 0.f ) && ( Global.fLuminance < Submodel->fLight ) };
					if (isemissive)
						model_ubs.emission = Submodel->f4Emision.a;

					// main draw call
					draw(Submodel->m_geometry.handle);

					model_ubs.emission = 0.0f;
					break;
				}
				case rendermode::shadows:
				{
                    // skip if the shadow caster rank is too low for currently set threshold
                    // or if the submodel is hidden (colour pass only fades it out, e.g. inactive seasonal variant)
                    if( ( Material( Submodel ).shadow_rank > Global.gfx_shadow_rank_cutoff )
                     || ( Submodel->fVisible <= 0.f ) )
                    {
                        --m_renderpass.draw_stats.submodels;
                        --m_renderpass.draw_stats.drawcalls;
                        break;
                    }

					if (Submodel->m_material < 0)
					{ // zmienialne skóry
						Bind_Material_Shadow(submodel_material(Submodel));
					}
					else
					{
						Bind_Material_Shadow(Submodel->m_material);
					}
					draw(Submodel->m_geometry.handle);
					break;
				}
				default:
				{
					break;
				}
				}
			}
		}
		else if (Submodel->eType == TP_FREESPOTLIGHT)
		{
			// NOTE: we're forced here to redo view angle calculations etc, because this data isn't instanced but stored along with the single mesh
			// TODO: separate instance data from reusable geometry
			auto const &modelview = OpenGLMatrices.data(GL_MODELVIEW);
			auto const lightcenter = modelview * glm::mix(glm::vec4(0.f, 0.f, -0.05f, 1.f), glm::vec4(0.f, 0.f, -0.25f, 1.f),
			                                                 static_cast<float>(TSubModel::fSquareDist / Submodel->fSquareMaxDist)); // pozycja punktu świecącego względem kamery
			Submodel->fCosViewAngle = glm::dot(glm::normalize(modelview * glm::vec4(0.f, 0.f, -1.f, 1.f) - lightcenter), glm::normalize(-lightcenter));

			if (Global.fLuminance < Submodel->fLight || Global.Overcast > 1.0f)
			{
				if (Submodel->fCosViewAngle > Submodel->fCosFalloffAngle)
				{
					// only bother if the viewer is inside the visibility cone
					// luminosity at night is at level of ~0.1, so the overall resulting transparency in clear conditions is ~0.5 at full 'brightness'
					auto glarelevel{std::clamp(std::max<float>(0.6f - Global.fLuminance, // reduce the glare in bright daylight
						                                  Global.Overcast - 1.f), // ensure some glare in rainy/foggy conditions
						                  0.f, 1.f)};
					// view angle attenuation
					float const anglefactor{std::clamp((Submodel->fCosViewAngle - Submodel->fCosFalloffAngle) / (Submodel->fCosHotspotAngle - Submodel->fCosFalloffAngle), 0.f, 1.f)};

					glarelevel *= anglefactor;

					if (glarelevel > 0.0f)
					{
						glDisable(GL_DEPTH_TEST);
						glBlendFunc(GL_SRC_ALPHA, GL_ONE);

						::glPushMatrix();
						::glLoadIdentity(); // macierz jedynkowa
						::glTranslatef(lightcenter.x, lightcenter.y, lightcenter.z); // początek układu zostaje bez zmian
						::glRotated(std::atan2(lightcenter.x, lightcenter.z) * 180.0 / M_PI, 0.0, 1.0, 0.0); // jedynie obracamy w pionie o kąt

						auto const lightcolor = glm::vec3(Submodel->DiffuseOverride.r < 0.f ? // -1 indicates no override
						                           Submodel->f4Diffuse :
						                           Submodel->DiffuseOverride);

						m_billboard_shader->bind();
						Bind_Texture(0, m_glaretexture);
						model_ubs.param[0] = glm::vec4(glm::vec3(lightcolor), Submodel->fVisible * glarelevel);

						// main draw call
						if (Submodel->occlusion_query) {
							glBeginConditionalRender(*Submodel->occlusion_query, GL_QUERY_WAIT);
							draw(m_billboardgeometry);
							glEndConditionalRender();
						}
						else
							draw(m_billboardgeometry);

						glEnable(GL_DEPTH_TEST);
						glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

						::glPopMatrix();
					}
				}
			}

			if (Submodel->fCosViewAngle > Submodel->fCosFalloffAngle)
			{
				// kąt większy niż maksymalny stożek swiatła
				float lightlevel = 1.f; // TODO, TBD: parameter to control light strength
				// view angle attenuation
				float const anglefactor = std::clamp((Submodel->fCosViewAngle - Submodel->fCosFalloffAngle) / (Submodel->fCosHotspotAngle - Submodel->fCosFalloffAngle), 0.f, 1.f);
				lightlevel *= anglefactor;

				// distance attenuation. NOTE: since it's fixed pipeline with built-in gamma correction we're using linear attenuation
				// we're capping how much effect the distance attenuation can have, otherwise the lights get too tiny at regular distances
				float const distancefactor{std::max(0.5f, (Submodel->fSquareMaxDist - TSubModel::fSquareDist) / Submodel->fSquareMaxDist)};
                auto const pointsize{std::max(3.f, 5.f * distancefactor * anglefactor)};
				// additionally reduce light strength for farther sources in rain or snow
				if (Global.Overcast > 0.75f)
				{
					float const precipitationfactor{std::lerp(std::lerp(1.f, 0.25f, std::clamp(Global.Overcast * 0.75f - 0.5f, 0.f, 1.f)), 1.f, distancefactor)};
					lightlevel *= precipitationfactor;
				}

				if (lightlevel > 0.f)
				{
					::glPushMatrix();
					::glLoadIdentity();
					::glTranslatef(lightcenter.x, lightcenter.y, lightcenter.z); // początek układu zostaje bez zmian

					// material configuration:
					// limit impact of dense fog on the lights
					auto const lightrange { std::max<float>( 500, m_fogrange * 2 ) }; // arbitrary, visibility at least 500m
					model_ubs.fog_density = 1.0 / lightrange;

					// main draw call
					model_ubs.emission = 1.0f;

					auto lightcolor = glm::vec3(Submodel->DiffuseOverride.r < 0.f ? // -1 indicates no override
                                            Submodel->f4Diffuse :
                                            Submodel->DiffuseOverride);

					m_freespot_shader->bind();

					if (Global.Overcast > 1.0f)
					{
						// fake fog halo
						float const fogfactor{std::lerp(1.5f, 1.f, std::clamp(Global.fFogEnd / 2000, 0.f, 1.f)) * std::max(1.f, Global.Overcast)};
                        model_ubs.param[1].x = pointsize * fogfactor * 4.0f;
						model_ubs.param[0] = glm::vec4(glm::vec3(lightcolor), Submodel->fVisible * std::min(1.f, lightlevel) * 0.5f);

						draw(Submodel->m_geometry.handle);
					}
                    model_ubs.param[1].x = pointsize * 4.0f;
					model_ubs.param[0] = glm::vec4(glm::vec3(lightcolor), Submodel->fVisible * std::min(1.f, lightlevel));

                    if (gl::vao::use_vao) {
                        if (!Submodel->occlusion_query)
                            Submodel->occlusion_query.emplace(gl::query::ANY_SAMPLES_PASSED);
                        Submodel->occlusion_query->begin();
                    }

					draw(Submodel->m_geometry.handle);

                    if (gl::vao::use_vao)
                        Submodel->occlusion_query->end();

					// post-draw reset
					model_ubs.emission = 0.0f;
					model_ubs.fog_density = 1.0f / m_fogrange;

					::glPopMatrix();
				}
			}
		}

		if (Submodel->Child != nullptr)
		{
			if (Submodel->eType == TP_TEXT)
			{ // tekst renderujemy w specjalny sposób, zamiast submodeli z łańcucha Child
				Render_Alpha_text(Submodel);
			}
			else if (Submodel->iAlpha & Submodel->iFlags & 0x002F0000)
				Render_Alpha(Submodel->Child);
		}

		if (Submodel->iFlags & 0xC000)
		{
			model_ubs.future = future_stack;
			::glPopMatrix();
		}
	}
	/*
	    if( Submodel->b_aAnim < at_SecondsJump )
	        Submodel->b_aAnim = at_None; // wyłączenie animacji dla kolejnego użycia submodelu
	*/
	if (Submodel->Next != nullptr)
		if (Submodel->iAlpha & Submodel->iFlags & 0x2F000000)
			Render_Alpha(Submodel->Next);
};

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
        if( ( m_simulationstate.season != simulationstate.season ) && ( false == simulationstate.season.empty() ) ) {
            m_materials.on_season_change();
        }
        if( ( m_simulationstate.weather != simulationstate.weather ) && ( false == simulationstate.weather.empty() ) ) {
            m_materials.on_weather_change();
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

std::string const &opengl46_renderer::info_stats() const
{

	return m_debugstatstext;
}

void opengl46_renderer::MakeScreenshot()
{
	screenshot_manager::make_screenshot();
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
		if (!Headlights || scenelight.intensity == 0.f) {
			// all lights past this one are bound to be off
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
        {
            // headlight projection is 1 km long box aligned with vehicle rotation and aimed slightly downwards
            opengl_camera headlights;
            auto const &ownerdimensions{ scenelight.owner->MoverParameters->Dim };
            auto const up{ static_cast<glm::dvec3>( scenelight.owner->VectorUp() ) };
            auto const size{ static_cast<float>( std::max( ownerdimensions.W, ownerdimensions.H ) * 1.0 ) }; // ensure square ratio
            auto const cone{ 5.f };
            auto const offset{ 75.f * ( 5.f / cone ) };
            headlights.position() =
                scenelight.owner->GetPosition()
                - glm::dvec3(scenelight.direction * offset)
                + up * ( size * 0.5 );
/*
            headlights.projection() = ortho_projection(
                -size, size,
                -size, size,
                ownerdimensions.L * 0.5 - 0.5, 1000.0f );
*/
            headlights.projection() = perspective_projection_raw(
                glm::radians( cone ),
                1.0,
                ownerdimensions.L * 0.5 + offset - 0.25, 1000.0f );
            glm::dmat4 viewmatrix{ 1.0 };
            viewmatrix *=
                glm::lookAt(
                    headlights.position(),
                    headlights.position()
                    - up * 1.0
                    + glm::dvec3{ scenelight.direction * 1000.f },
                    glm::dvec3{ 0.f, 1.f, 0.f } );
            headlights.modelview() = viewmatrix;
            // calculate world->headlight space projection matrix
            glm::mat4 const depthproj{ headlights.projection() };
            // NOTE: we strip transformations from camera projections to remove jitter that occurs
            // with large (and unneded as we only need the offset) transformations back and forth
            auto const lightcam{ glm::mat3{ headlights.modelview() } };
            auto const worldcam{ glm::mat3{ m_renderpass.pass_camera.modelview() } };

            light->headlight_projection =
                coordmove
                * depthproj
                * glm::translate(
                    glm::mat4{ lightcam },
                    glm::vec3{ m_renderpass.pass_camera.position() - headlights.position() } )
                * glm::mat4{ glm::inverse( worldcam ) };
        }
        // headlights weights
        light->headlight_weights = { scenelight.state, 0.f };

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
		m_fogrange = Global.fFogEnd / std::max(1.f, Global.Overcast * 2.f);
		model_ubs.fog_density = 1.0f / m_fogrange;
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

void opengl46_renderer::Write_Local_Light(gl::light_element &Light, opengl46_local_lights::light const &Source, glm::dvec3 const &Camera, glm::mat4 const &View)
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
	auto const &lamps{m_localshadows.lamps()};
	if (lamps.empty() || (false == Global.gfx_lights_clustered))
		return;

	// vehicles near the camera cast the shadows of the lamps, and the occupied cab
	m_lampshadowvehicles.clear();
	auto const vehiclesrange{static_cast<double>(Global.gfx_lights_shadows_vehicles_range)};
	if (vehiclesrange > 0.0)
	{
		for (auto *vehicle : simulation::Vehicles.sequence())
		{
			if (glm::length(vehicle->GetPosition() - Global.pCamera.Pos) <= vehiclesrange + vehicle->radius())
				m_lampshadowvehicles.emplace_back(vehicle);
		}
	}
	TDynamicObject *cab{nullptr};
	if ((vehiclesrange > 0.0) && (false == FreeFlyModeFlag) && Global.render_cab && (simulation::Train != nullptr))
	{
		cab = simulation::Train->Dynamic();
		if ((cab != nullptr) && ((false == cab->bDisplayCab) || (cab->mdKabina == nullptr) || (cab->mdKabina == cab->mdModel)))
			cab = nullptr;
	}

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
		m_lampshadowslot.clear();
		for (auto *vehicle : m_lampshadowvehicles)
		{
			if (glm::length(vehicle->GetPosition() - slot.position) <= slot.range + vehicle->radius())
				m_lampshadowslot.emplace_back(vehicle);
		}
		auto *slotcab{(cab != nullptr) && (glm::length(cab->GetPosition() - slot.position) <= slot.range + cab->radius()) ? cab : nullptr};
		if (m_lampshadowslot.empty() && (slotcab == nullptr))
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
		for (auto *vehicle : m_lampshadowslot)
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
		Render(simulation::Region);
		m_cubeskipinstance = nullptr;
		m_cubescenery = false;
	}
	m_cubeshadows = false;
	m_localshadows.end_lamp(static_cast<std::uint32_t>(m_renderpass.draw_stats.drawcalls));
	m_renderpass = m_colorpass;
	glPolygonMode(GL_FRONT_AND_BACK, Global.bWireFrame ? GL_LINE : GL_FILL);
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

void opengl46_renderer::Render_cluster_heatmap(viewport_config &vp, gl::framebuffer *Target, glm::ivec2 const &Targetsize)
{
	glDebug("Render_cluster_heatmap");

	if (!vp.msaa_fb)
		return;

	if (!vp.clusters_depth_fb)
	{
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
			ErrorLog("cluster heatmap framebuffer setup failed");
			Global.gfx_lights_clusterheatmap = false;
			return;
		}
		vp.clusters_depth_tex = std::move(texture);
		vp.clusters_depth_fb = std::move(framebuffer);
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

	if (Global.gfx_framebuffer_width == -1)
        Global.gfx_framebuffer_width = Global.fb_size.x;
	if (Global.gfx_framebuffer_height == -1)
        Global.gfx_framebuffer_height = Global.fb_size.y;

	WriteLog("main window size: " + std::to_string(Global.gfx_framebuffer_width) + "x" + std::to_string(Global.gfx_framebuffer_height));

	return true;
}

glm::vec3 opengl46_renderer::pick_color(std::size_t const Index)
{
	return glm::vec3{((Index & 0xff0000) >> 16) / 255.0f, ((Index & 0x00ff00) >> 8) / 255.0f, (Index & 0x0000ff) / 255.0f};
}

std::size_t opengl46_renderer::pick_index(glm::ivec3 const &Color)
{
	return Color.b + (Color.g * 256) + (Color.r * 256 * 256);
}

std::unique_ptr<gfx_renderer> opengl46_renderer::create_func()
{
    return std::unique_ptr<gfx_renderer>(new opengl46_renderer());
}

bool opengl46_renderer::renderer_register = gfx_renderer_factory::get_instance()->register_backend("modern", opengl46_renderer::create_func);

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

std::uint64_t opengl46_renderer::opengl46_imgui_renderer::Create_Image(std::uint8_t const *Rgba, int const Width, int const Height)
{
	// the ui colours are made linear in its shaders, the image is too, by sampling it as srgb. the bindings the renderer
	// keeps track of are put back as they were
	GLint texture{0}, unpack{0}, alignment{4};
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture);
	glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &unpack);
	glGetIntegerv(GL_UNPACK_ALIGNMENT, &alignment);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	GLuint image{0};
	glGenTextures(1, &image);
	glBindTexture(GL_TEXTURE_2D, image);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, Width, Height, 0, GL_RGBA, GL_UNSIGNED_BYTE, Rgba);
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

