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

// the top view of the editor shows everything this close to the camera, the corners of the screen included
float
editor_ortho_reach()
{
	auto const aspect{Global.window_size.x / std::max(1.f, static_cast<float>(Global.window_size.y))};
	return Global.EditorOrthoExtent * std::sqrt(1.f + aspect * aspect) + 500.f;
}

float
editor_track_view_range(float const Range, bool const Apply)
{
	if (false == Apply || false == Global.editor_tracks)
		return Range;
	float extra = 50000.0f;
	if (Global.EditorOrtho)
		extra = editor_ortho_reach();
	return Range > extra ? Range : extra;
}

// the top view of the editor may leave the ballast of the railway out, for the ground under the tracks to show
bool
editor_ballast_hidden(TTrack const *Track, bool const Colour)
{
	return Colour && EditorModeFlag && Global.EditorOrtho && false == Global.EditorOrthoBallast && (Track->iCategoryFlag & 15) == 1;
}

// the top view draws the scenery as far as the tracks; its models keep their own ranges
bool
editor_far_tracks_only(bool const Apply, scene::basic_cell const *Cell, glm::dvec3 const &Camera, double const SceneryLimit)
{
	if (false == Apply || Global.EditorOrtho)
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
	model_ubo = std::make_unique<gl::stream_ubo>(sizeof(gl::model_ubs), 1, Global.gfx_bufferstreaming);
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
		m_reflectioncull.init(&m_instancecull);
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
	// the context of the main view is current here, the one of the previous frame's end
	m_gputimeline.begin("frame setup");

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
		m_sleeperrecords.clear();
		m_modelrecords.clear();
		m_cascaderecords.clear();
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
	if( ( m_framestamp % 256 ) == 0 ) {
		// the sleeper records of tracks out of sight for a while. their places in the record buffer of the gpu culling are
		// dropped when it starts over
		for( auto records { m_sleeperrecords.begin() }; records != m_sleeperrecords.end(); ) {
			if( m_framestamp - records->second.frame > 256 ) { records = m_sleeperrecords.erase( records ); }
			else { ++records; }
		}
		// and so are the records of the models drawn on their own
		for( auto records { m_modelrecords.begin() }; records != m_modelrecords.end(); ) {
			if( m_framestamp - records->second.frame > 256 ) { records = m_modelrecords.erase( records ); }
			else { ++records; }
		}
	}
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
    // the whole frame of the main loop and its parts: the update of the simulation, Render() with the ui drawn in it, the swap,
    // and what's left (input, terrain streaming, tasks, network), measured by none of them
    {
        auto const mainloop { Timer::subsystem.mainloop_total.average() };
        auto const simulation { Timer::subsystem.sim_total.average() };
        auto const render { Timer::subsystem.gfx_total.average() };
        auto const swap { Timer::subsystem.gfx_swap.average() };
        m_debugtimestext =
            "mainloop total: " + to_string( mainloop, 2 ) + " ms\n"
            + " simulation: " + to_string( simulation, 2 ) + " ms\n"
            + " render: " + to_string( render, 2 ) + " ms (ui " + to_string( Timer::subsystem.gfx_gui.average(), 2 ) + " ms)\n"
            + " swap: " + to_string( swap, 2 ) + " ms\n"
            + " other: " + to_string( mainloop - simulation - render - swap, 2 ) + " ms\n";
    }
    m_debugtimestext +=
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
    // the parts follow one another and add up to the frame; the steps below are timed each on its own and can hold the tail
    // of the work before them
    if( auto const &gpuparts { m_gputimeline.result() }; false == gpuparts.empty() ) {
        m_debugtimestext += "gpu frame parts: " + to_string( static_cast<double>( m_gputimeline.total() ) / 1000000.0, 3 ) + " ms\n";
        for( auto const &part : gpuparts ) {
            m_debugtimestext += " " + std::string{ part.name } + ": " + to_string( static_cast<double>( part.time ) / 1000000.0, 3 ) + " ms\n";
        }
        m_debugtimestext += "gpu steps:\n";
    }
    if (auto const clusterstime = m_clusterstimer.result())
        m_debugtimestext += " clusters: " + to_string(static_cast<double>(*clusterstime) / 1000000.0, 3) + " ms\n";
    if (auto const culltime = m_cullstimer.result())
        m_debugtimestext += " cull: " + to_string(static_cast<double>(*culltime) / 1000000.0, 3) + " ms\n";
    m_debugtimestext += m_sky.info_times();
    m_debugtimestext += m_weather.info_times();
    m_debugtimestext += m_exposure.info_times();
    m_debugtimestext += m_ao.info_times();
    m_debugtimestext += m_instancecull.info_times();
    // the cullings of the other passes; the environment map is drawn now and then, its numbers are of its last update
    if( Global.gfx_gpucull_shadows ) {
        for( std::size_t cascade = 0; cascade < m_shadowcull.size(); ++cascade ) {
            m_debugtimestext += m_shadowcull[ cascade ].info_times( "instance cull shadows " + std::to_string( cascade + 1 ) );
        }
    }
    if( Global.gfx_gpucull_reflections && m_reflectioncull.ran() ) {
        m_debugtimestext += m_reflectioncull.info_times( "instance cull reflections, last face" );
    }
    m_debugtimestext += Light_Shadow_Cull_Info( true );
    if( Global.gfx_depthprepass_scene || m_contactshadowslast ) {
        if( auto const scenedepthtime = m_scenedepthtimer.result() )
            m_debugtimestext += " scene depth: " + to_string( static_cast<double>( *scenedepthtime ) / 1000000.0, 3 ) + " ms\n";
    }
    if( m_contactshadowslast ) {
        if( auto const contacttime = m_contactshadowstimer.result() )
            m_debugtimestext += " contact shadows depth: " + to_string( static_cast<double>( *contacttime ) / 1000000.0, 3 ) + " ms\n";
    }
    if( Global.gfx_gpucull == 1 ) {
        auto const twophase { m_instancecull.two_phase() };
        m_debugtimestext += m_hiz.info_times( twophase );
        if( auto const prepasstime = m_prepasstimer.result() ) {
            m_debugtimestext += " depth prepass: " + to_string( static_cast<double>( *prepasstime ) / 1000000.0, 3 ) + " ms";
            auto const firsttime { m_prepassfirsttimer.result() };
            if( twophase && firsttime )
                m_debugtimestext += " (+" + to_string( static_cast<double>( *firsttime ) / 1000000.0, 3 ) + " ms of the instances drawn first)";
            m_debugtimestext += "\n";
        }
    }
    m_debugtimestext += m_localshadows.info_times();
    if( Global.gfx_shadow_sdsm )
        m_debugtimestext += m_shadowfit.info_times();
    m_debugtimestext += m_gpuparticles.info_times();
    m_debugtimestext += "uilayer: " + to_string( Timer::subsystem.gfx_gui.average(), 2 ) + " ms\n";
    if( DebugModeFlag )
        m_debugtimestext += m_textures.info();

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
        + m_instancecull.info();
    if( Global.gfx_gpucull_shadows ) {
        for( std::size_t cascade = 0; cascade < m_shadowcull.size(); ++cascade ) {
            if( false == m_shadowcull[ cascade ].ran() ) { continue; }
            m_shadowcull[ cascade ].collect();
            m_debugstatstext += m_shadowcull[ cascade ].info( "gpu cull shadows " + std::to_string( cascade + 1 ), false );
        }
    }
    if( Global.gfx_gpucull_reflections && m_reflectioncull.ran() ) {
        m_reflectioncull.collect();
        m_debugstatstext += m_reflectioncull.info( "gpu cull reflections, last face", false );
    }
    m_debugstatstext += Light_Shadow_Cull_Info( false );
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
            ImGui::Checkbox("the same for the environment map, when it's drawn next (gfx.gpucull.reflections)", &Global.gfx_gpucull_reflections);
            ImGui::Checkbox("the same for the shadow maps of headlights (gfx.gpucull.shadows.headlights)", &Global.gfx_gpucull_shadows_headlights);
            ImGui::Checkbox("the same for the scenery of the shadow maps of lamps, when it's drawn next (gfx.gpucull.shadows.lamps)", &Global.gfx_gpucull_shadows_lamps);
            ImGui::Checkbox("the same for the sleepers of the tracks of the view (gfx.gpucull.sleepers)", &Global.gfx_gpucull_sleepers);
            ImGui::Checkbox("the same for the models drawn on their own, with lights, animations or skins of their own (gfx.gpucull.models)", &Global.gfx_gpucull_models);
            ImGui::Checkbox("leave out the models of a cell lying wholly out of view, beyond their range or too small before the culling (gfx.gpucull.bucketrange)", &Global.gfx_gpucull_bucketrange);
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
        if (Global.gfx_gpucull_reflections && m_reflectioncull.ran())
            info += m_reflectioncull.info("gpu cull reflections, last face", false);
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

        m_colorpass = m_renderpass; // cache pass data
		// parts of the gpu time of the frame; the marks go through the main view only, in the context they were begun in
		auto const gpupart = [this, &vp](char const *Name) {
			if (vp.main)
				m_gputimeline.next(Name);
		};

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

			if (motionblur)
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
		if ((Global.gfx_depthprepass_scene || contactshadows) && vp.main && (false == Global.bWireFrame))
		{
			glDebug("render scene depth");
			gpupart("scene depth");
			Render_Scene_Depth(future, contactshadows && (false == Global.gfx_depthprepass_scene));
			if (contactshadows)
			{
				gpupart("contact shadows depth");
				Setup_Contact_Shadows(vp);
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

		if (m_contactshadows)
		{
			// the depth they read is of this view only
			m_contactshadows = false;
			scene_ubs.contact_shadow_params = glm::vec4{0.f};
			scene_ubs.contact_shadow_lamps = glm::vec4{0.f};
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
			if (motionblur)
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

			// the depth of the finished view, which the cascades of the sun shadows of the next frames are fitted to
			// (gfx.shadow.sdsm). the main framebuffer holds it resolved for the motion blur or the ambient occlusion
			if (vp.main && Global.gfx_shadow_sdsm && (Shadow_Fit_Unavailable(vp) == nullptr))
			{
				auto const resolved{(vp.main_texd != nullptr) && (motionblur || (aomode != 0))};
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

		// ...a cascade fitted to the depth of the view (gfx.shadow.sdsm) covers the area of the pixels it takes, within the
		// slice of the view. the areas are compared across the light, the depth along it stays the one of the slice...
		auto quantizationstep{0.f};
		{
			auto const lightrotation{Shadow_Light_Rotation()};
			glm::vec3 slicemin{std::numeric_limits<float>::max()}, slicemax{std::numeric_limits<float>::lowest()};
			for (auto const &point : frustumchunkshapepoints)
			{
				auto const lightpoint{lightrotation * glm::vec3{point}};
				slicemin = glm::min(slicemin, lightpoint);
				slicemax = glm::max(slicemax, lightpoint);
			}
			m_shadowslicesize = std::max(slicemax.x - slicemin.x, slicemax.y - slicemin.y);
			if (m_shadowfitarea != nullptr)
			{
				glm::vec3 areamin{std::numeric_limits<float>::max()}, areamax{std::numeric_limits<float>::lowest()};
				for (auto const &point : *m_shadowfitarea)
				{
					auto const lightpoint{lightrotation * point};
					areamin = glm::min(areamin, lightpoint);
					areamax = glm::max(areamax, lightpoint);
				}
				auto const fitmin{glm::vec2{glm::max(glm::vec2{slicemin}, glm::vec2{areamin})}};
				auto const fitmax{glm::vec2{glm::min(glm::vec2{slicemax}, glm::vec2{areamax})}};
				if ((fitmin.x < fitmax.x) && (fitmin.y < fitmax.y))
				{
					// the size changes in steps of a power of two of about 1/16 of it, so that the texels of the map keep
					// their size until the area grows or shrinks by a step
					auto const size{std::max(fitmax.x - fitmin.x, fitmax.y - fitmin.y)};
					quantizationstep = std::exp2(std::ceil(std::log2(std::max(size, 1.f) / 16.f)));
					frustumchunkshapepoints.clear();
					auto const inverserotation{glm::transpose(lightrotation)};
					for (int corner = 0; corner < 8; ++corner)
					{
						auto const lightpoint{glm::vec3{(corner & 1) ? fitmax.x : fitmin.x, (corner & 2) ? fitmax.y : fitmin.y, (corner & 4) ? slicemax.z : slicemin.z}};
						frustumchunkshapepoints.emplace_back(inverserotation * lightpoint, 1.f);
					}
				}
			}
		}

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

std::string opengl46_renderer::Shader_Name(std::shared_ptr<gl::program> const &Program) const
{
	for (auto const &shader : m_shaders)
		if (shader.second == Program)
			return shader.first;
	return {};
}

std::shared_ptr<gl::program> opengl46_renderer::Fetch_Shader(const std::string &name)
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

		auto *program{material.shader.get()};
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

void opengl46_renderer::Render(scene::basic_region *Region)
{

	m_sectionqueue.clear();
	m_cellqueue.clear();
	// discard last pass's accumulated instance buckets before this pass starts.
	// the vectors themselves are kept, so their allocations get reused by the following passes
	for( auto &bucket : m_frame_instance_buckets ) {
		bucket.second.clear();
	}
	// the gpu culling runs along the first color pass of the main viewport of a frame, and the passes of the cascades of the
	// sun shadows, of the faces of the environment map and of the maps of headlights and lamps drawn for it, each with a culling
	// of its own. its result is drawn (Global.gfx_gpucull 1), or compared with the one of the cpu, which draws (2)
	m_instancecullpass = false;
	m_passcull = nullptr;
	if( ( Global.gfx_gpucull != 0 )
	 && ( m_current_viewport != nullptr )
	 && ( m_current_viewport->main ) ) {
		if( ( m_renderpass.draw_mode == rendermode::color )
		 && ( m_instancecullframe != m_framestamp ) ) {
			m_instancecullframe = m_framestamp;
			m_passcull = &m_instancecull;
		}
		else if( ( m_renderpass.draw_mode == rendermode::shadows )
		      && ( m_lightshadowcull != nullptr ) ) {
			// the map of a set of headlights or the scenery part of the map of a lamp, which picked its culling
			m_passcull = m_lightshadowcull;
			m_shadowcasters.clear();
		}
		else if( ( m_renderpass.draw_mode == rendermode::shadows )
		      && ( Global.gfx_gpucull_shadows )
		      && ( m_shadowcascade >= 0 )
		      && ( m_shadowcascade < static_cast<int>( m_shadowcull.size() ) )
		      && ( false == m_cubeshadows ) ) {
			// the cascades of the sun
			m_passcull = &m_shadowcull[ m_shadowcascade ];
			m_shadowcasters.clear();
		}
		else if( ( m_renderpass.draw_mode == rendermode::reflections )
		      && ( Global.gfx_gpucull_reflections )
		      && ( Global.reflectiontune.fidelity >= 1 ) ) {
			m_passcull = &m_reflectioncull;
		}
	}
	if( m_passcull != nullptr ) {
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
		m_passcull->begin_pass( setup );
	}
	// the sleepers of tracks go with the instances of the main view
	m_sleeperspass = ( ( m_passcull == &m_instancecull ) && Global.gfx_gpucull_sleepers );
	if( m_sleeperspass && ( false == m_passcull->drawing() ) ) {
		for( auto &counts : m_sleeperbandcounts ) {
			counts.second.clear();
		}
	}
	// the models drawn on their own go with the instances of every pass culled on the gpu
	m_modelspass = ( m_instancecullpass && Global.gfx_gpucull_models );
	m_modelcpudistances.clear();
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
		if (EditorModeFlag && m_current_viewport->main && (false == m_scenedepthdrawn))
		{
			// when editor mode is active calculate world position of the cursor
			// at this stage the z-buffer is filled with only ground geometry
			// (with the depth pass of the whole scene it's done there, at the same stage)
			if (m_scenerecording)
			{
				// the depth of what the walk recorded so far
				Render_Scene_Prepass();
			}
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
			// scenery drawn by the cpu and vehicles below keep their own per-node
			// culling and are intentionally left untouched; the non-instanced models
			// culled on the gpu (Queue_Model()) are left out with the cell the same way.
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
				auto const cellinrange { cellnearestdistancesquared <= sq( static_cast<double>( m_renderpass.draw_range ) + 250.0 ) };
				if( cellinrange ) {
					Queue_Instances( cell );
				}
				// remaining (non-instanceable) opaque instance nodes go through the per-node path, or with the instances
				// to the gpu culling, each on its own
				for (auto *instance : cell->m_instancesopaque)
				{
					if( instance->m_instanceable ) { continue; } // already handled via bucket
					// the model of a lamp would shadow everything in the map of the lamp
					if( instance == m_cubeskipinstance ) { continue; }
					if( Queue_Model( instance, cellinrange ) ) { continue; }
					Render(instance);
					m_modelcomparekey = nullptr;
				}
			}
			// opaque parts of vehicles. the kept scenery part of the map of a lamp is drawn without them
			if( false == m_cubescenery ) {
				for (auto *path : cell->m_paths)
				{
					for (auto *dynamic : path->Dynamics)
					{
						// the vehicle of the headlights a map is drawn for would cover all of it
						if (dynamic == m_shadowskipvehicle)
							continue;
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
                    if( Queue_Model( instance, true ) ) { continue; }
                    Render( instance );
                    m_modelcomparekey = nullptr;
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
			m_passcull->dispatch( OpenGLMatrices.data( GL_MODELVIEW ) );
			// the sleepers the cpu drew along the tracks, before the culling ran
			if( m_sleeperspass ) {
				for( auto const &counts : m_sleeperbandcounts ) {
					if( false == counts.second.empty() ) {
						m_passcull->cpu_result( counts.first, counts.second, true );
					}
				}
			}
			// and the models drawn on their own
			for( auto const &model : m_modelcpudistances ) {
				m_passcull->cpu_instance( model.first, model.second );
			}
		}
	}
	for( auto &bucket : m_frame_instance_buckets ) {
		if( bucket.second.empty() ) { continue; }
		Render_Instanced( bucket.first.pModel, bucket.second, ( m_instancecullpass ? &m_instance_bandcounts : nullptr ) );
		if( m_instancecullpass ) {
			m_passcull->cpu_result( bucket.first, m_instance_bandcounts );
		}
		bucket.second.clear();
	}
	if( m_instancecullpass ) {
		m_passcull->end_pass();
		m_instancecullpass = false;
		m_passcull = nullptr;
	}
	m_sleeperspass = false;
	m_modelspass = false;
	m_modelcpudistances.clear();
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

void opengl46_renderer::Render(scene::shape_node const &Shape, bool const Ignorerange)
{
	auto const &data{Shape.data()};

	// a shape lying wholly outside the view of the pass draws nothing. the section and the cell holding it were tested as a
	// whole, while a section holds the chunks of the heightmap terrain (250 m each, 16 of them) and the large shapes of the
	// scenery, drawn in every pass the section is seen in: in a cascade of the sun shadows or a face of the environment map
	// most of them are out of view. a shape merged from others has its radius invalidated (-1) and keeps being drawn
	if ((data.area.radius > 0.f) && (false == m_renderpass.pass_camera.visible(data.area)))
	{
		return;
	}

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
    if( m_modelcomparekey != nullptr ) {
        // queued in the gpu culling as well, to compare with (Queue_Model())
        m_modelcpudistances.emplace_back( *m_modelcomparekey, static_cast<float>( distancesquared ) );
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
	// a bucket whose visible instances all fail the tests of the culling by the bounds of their records (Bucket_Culled()) is left
	// out of the pass before the culling. the bounds are made with the records, which the editor doesn't let go by
	auto const buckettests { Global.gfx_gpucull_bucketrange && userecords };

	for( auto const &bucket : Cell->m_instancebuckets_opaque ) {
		auto const &instancebucket { bucket.second };
		if( ( instancebucket.recordsversion != m_instancerecordsversion )
		 || ( instancebucket.records.size() != instancebucket.instances.size() ) ) {
			Update_Instance_Records( bucket.first.pModel, instancebucket, Cell->m_area.center );
		}
		if( buckettests && Bucket_Culled( instancebucket ) ) {
			if( m_instancecullpass ) {
				m_passcull->skip_bucket( instancebucket );
			}
			continue;
		}
		if( m_instancecullpass ) {
			// the gpu tests every instance of the bucket, the way Render_Instanced() does. models with nothing to draw in the
			// sun shadows are left out of them, as Render_Instanced() leaves them. the lod bands drawn go by the distances
			// the locations of the instances can be at: of their own bounds, or of the cell, which holds them too
			if( ( m_renderpass.draw_mode != rendermode::shadows ) || Instances_Cast_Shadow( bucket.first, instancebucket ) ) {
				m_passcull->add_bucket( bucket.first, instancebucket, ( buckettests ? Bucket_Area( instancebucket ) : Cell->m_area ) );
			}
			// and when the instances are drawn from its result, the cpu has nothing more to do with them
			if( m_passcull->drawing() ) { continue; }
		}
		// the depth pass of the whole scene culled them already, the colour pass draws what it recorded
		if( m_indirectcolourpending ) { continue; }
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

// queues a model instance drawn on its own in the gpu culling of the pass, as a bucket and a group of its own. the culling
// makes the tests of Render( TAnimModel * ): visibility, frustum, the range of the instance, the draw range, the size on the
// screen, and gives it the lod band of the bounds of its submodels (mindistance/maxdistance) its distance falls in, then
// Record_Indirect_Draws() animates, prepares and walks its submodel tree from its origin, like it does for a group of
// instanced ones
bool opengl46_renderer::Queue_Model( TAnimModel *Instance, bool const Inrange )
{
	// a billboard is turned to the camera from the modelview of the instance, which a walk from its origin doesn't have
	if( ( false == Instance->m_cameraindependent ) || ( Instance->Model() == nullptr ) ) { return false; }
	if( false == m_modelspass ) {
		// the depth pass of the whole scene culled and recorded it, the colour pass draws what it recorded
		return ( m_indirectcolourpending && m_indirectcolourmodels );
	}
	auto const drawing { m_passcull->drawing() };
	// a model hidden by an event, or in a cell wholly out of the range of the pass, would be left out by the culling
	if( ( false == Instance->visible() ) || ( false == Inrange ) ) { return drawing; }

	auto &records { m_modelrecords[ Instance ] };
	auto &bucket { records.bucket };
	if( ( bucket.recordsversion != m_instancerecordsversion ) || ( bucket.records.size() != 1 ) ) {
		// the key, the instance and the record anew: the editor may have given the instance another model or skins, or
		// another instance may have taken the place of a deleted one
		auto &key { records.key };
		key = {};
		key.pModel = Instance->Model();
		if( auto const *material { Instance->Material() } ) {
			for( int skin = 0; skin < 5; ++skin ) { key.skins[ skin ] = material->replacable_skins[ skin ]; }
		}
		key.instance = Instance;
		bucket.instances.assign( 1, Instance );
		// the location of the instance as the origin of the record, which keeps the precision
		Update_Instance_Records( key.pModel, bucket, Instance->location() );
		// drawn by its bucket, though it isn't instanceable
		bucket.records.front().flags |= scene::basic_cell::instance_instanceable;
		records.area = scene::bounding_area{ Instance->location(), 0.f };
	}
	records.frame = m_framestamp;
	if( m_renderpass.draw_mode == rendermode::shadows ) {
		// a model with nothing to draw in the shadows is left out of them, like the instances. the answer depends on the
		// model and the skins only, and is shared by the instances with them
		auto modelkey { records.key };
		modelkey.instance = nullptr;
		if( false == Instances_Cast_Shadow( modelkey, bucket ) ) { return drawing; }
	}
	// out of the frustum of the pass, beyond its range or too small on the screen by its record: the culling would leave it out,
	// and the walk of Record_Indirect_Draws() isn't made for it
	if( Global.gfx_gpucull_bucketrange && ( false == EditorModeFlag ) && Bucket_Culled( bucket ) ) {
		m_passcull->skip_bucket( bucket );
		return drawing;
	}
	m_passcull->add_bucket( records.key, bucket, records.area );
	if( false == drawing ) {
		// the cpu draws it too, and tells what it drew for the comparison
		m_modelcomparekey = &records.key;
	}
	return drawing;
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
	// the range of the bucket: past it none of its instances is drawn, by its own range or by the submodels of the model
	auto rangesquared { 0.f };
	for( auto *instance : Bucket.instances ) {
		scene::basic_cell::instance_record record;
		if( instance != nullptr ) {
			record.position = packed_vec3{ instance->location() - Origin };
			record.radius = instance->radius();
			record.rangesquaredmin = tofloat( instance->m_rangesquaredmin );
			record.rangesquaredmax = tofloat( instance->m_rangesquaredmax );
			rangesquared = std::max( rangesquared, record.rangesquaredmax );
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
	Bucket.rangesquared = std::min( rangesquared, lodrangesquared );

	// bounds of the visible instances, for Bucket_Culled(): the centre of the box around their locations, and the spheres
	// around it which hold the locations and the instances whole. the gpu leaves out the ones which aren't visible
	glm::dvec3 low { std::numeric_limits<double>::max() };
	glm::dvec3 high { std::numeric_limits<double>::lowest() };
	auto radiusmax { 0.f };
	for( auto const &record : Bucket.records ) {
		if( ( record.flags & scene::basic_cell::instance_visible ) == 0 ) { continue; }
		glm::dvec3 const position { record.position };
		low = glm::min( low, position );
		high = glm::max( high, position );
		radiusmax = std::max( radiusmax, record.radius );
	}
	Bucket.recordlocationradius = 0.f;
	Bucket.recordradius = -1.f;
	Bucket.recordradiusmax = radiusmax;
	if( low.x <= high.x ) {
		auto const center { 0.5 * ( low + high ) };
		auto locationradius { 0.0 };
		auto radius { 0.0 };
		for( auto const &record : Bucket.records ) {
			if( ( record.flags & scene::basic_cell::instance_visible ) == 0 ) { continue; }
			auto const distance { glm::length( glm::dvec3{ record.position } - center ) };
			locationradius = std::max( locationradius, distance );
			radius = std::max( radius, distance + record.radius );
		}
		Bucket.recordcenter = Origin + center;
		Bucket.recordlocationradius = static_cast<float>( locationradius );
		Bucket.recordradius = static_cast<float>( radius );
	}
	else {
		Bucket.recordcenter = Origin;
	}
	Bucket.recordsversion = m_instancerecordsversion;
	Bucket.gpustale = true;
}

// whether none of the instances of a bucket can pass the tests of the culling of the pass (Render_Instanced(), on the gpu
// instance_cull.comp): by the bounds of their records the visible ones lie wholly out of the frustum of the pass, beyond their
// range or the draw range, or are too small on the screen. the tests are those of the culling with the nearest location and
// the largest instance of the bucket, with a margin for the single precision of the records and of the gpu, so they leave out
// only buckets which the culling would leave out whole
bool opengl46_renderer::Bucket_Culled( scene::basic_cell::instance_bucket const &Bucket ) const
{
	if( Bucket.recordradius < 0.f ) { return true; }

	auto const slack { 1.001 };
	auto const margin { 0.5 }; // metres
	// the planes of the frustum in world coordinates, the ones the gpu culling moves to the camera of the pass
	auto const &planes { m_renderpass.pass_camera.frustum().planes() };
	auto const &center { Bucket.recordcenter };
	auto const radius { slack * Bucket.recordradius + margin };
	for( auto const *plane : planes ) {
		if( plane[ 0 ] * center.x + plane[ 1 ] * center.y + plane[ 2 ] * center.z + plane[ 3 ] <= -radius ) { return true; }
	}

	// the same choices as in Render_Instanced(): shadows measure the distance from the real camera, reflections ignore zoom and
	// distance factor, test the plain distance against their own range and add the fidelity offset
	auto const reflections { m_renderpass.draw_mode == rendermode::reflections };
	auto const &distancecamera { m_renderpass.draw_mode == rendermode::shadows ? m_renderpass.viewport_camera : m_renderpass.pass_camera };
	auto const nearest { std::max( 0.0, glm::length( center - distancecamera.position() ) - ( slack * Bucket.recordlocationradius + margin ) ) };
	auto const zoom { static_cast<double>( Global.ZoomFactor ) };
	auto const scaledsquared { nearest * nearest * ( reflections ? 1.0 : 1.0 / ( zoom * zoom * static_cast<double>( Global.fDistanceFactor ) ) ) };
	if( reflections && ( scaledsquared > slack * sq( static_cast<double>( Global.reflectiontune.range_instances ) ) ) ) { return true; }
	auto const distancesquared { scaledsquared + ( reflections ? sq( static_cast<double>( EU07_REFLECTIONFIDELITYOFFSET ) ) : 0.0 ) };
	if( distancesquared >= slack * Bucket.rangesquared ) { return true; }
	if( distancesquared > slack * sq( static_cast<double>( m_renderpass.draw_range ) + 250.0 ) ) { return true; }
	// the largest instance at the nearest location takes the largest part of the view
	auto const radiusmax { static_cast<double>( Bucket.recordradiusmax ) };
	if( radiusmax * radiusmax * zoom * slack < 0.003 * 0.003 * distancesquared ) { return true; }

	return false;
}

// the sphere around the locations of the visible instances of a bucket, with the margin of Bucket_Culled(): their distances
// from it set the lod bands the gpu culling can put them in
scene::bounding_area opengl46_renderer::Bucket_Area( scene::basic_cell::instance_bucket const &Bucket )
{
	return scene::bounding_area{ Bucket.recordcenter, static_cast<float>( 1.001 * Bucket.recordlocationradius + 0.5 ) };
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
		"sunlessnormalmap", "sunlessnormalmap_specgloss",
		"trackblend", "trackblend_normalmap", "trackblend_specgloss" };
	static std::unordered_set<std::string> const solid { "colored", "default_0", "terrain", "water", "water_specgloss", "invalid" };
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

	if( m_scenerecording ) {
		// the depth of what the recorded walk of the depth pass of the scene went through so far, for the occlusion test
		// and for the depth of the instances to be tested against
		Render_Scene_Prepass();
	}
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
	// in two phases the instances seen in the previous frame are drawn first, and the others are tested against a pyramid
	// made again with their depth in it (not in wireframe, which doesn't draw surfaces)
	m_instancecull.dispatch(
		OpenGLMatrices.data( GL_MODELVIEW ),
		( useocclusion ? &occlusion : nullptr ),
		( Global.gfx_gpucull_occlusion_instances && ( false == Global.bWireFrame ) ) );

	// the draws, recorded while the gpu culls
	Record_Indirect_Draws( m_instancecull );

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
			}
			else {
				colourcounts += Render_Instances_Indirect_Colour( command_set::first );
			}
			m_hiz.build( *viewport->msaa_fb, viewport->width, viewport->height, true );
			occlusion.pyramid = m_hiz.texture();
		}
		// the others are tested against it; with no draws the instances are only marked for the next frame
		m_instancecull.dispatch_second( occlusion );
		colourset = ( depthfirst ? command_set::all : command_set::second );
	}
	if( draws ) {
		m_instancecull.bind_draw();
		if( depthfirst ) {
			prepasscounts += Render_Instances_Indirect_Prepass( ( twophase ? command_set::second : command_set::first ), m_prepasstimer );
		}
		if( false == m_scenedepthpass ) {
			colourcounts += Render_Instances_Indirect_Colour( colourset );
		}
		::glBindBuffer( GL_DRAW_INDIRECT_BUFFER, 0 );
		Bind_Instance_Stream();
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
	auto &cull { *m_passcull };
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
	m_indirectdraws.clear();
	m_indirectcommands.clear();
	auto const &lodbounds { Cull.lod_bounds() };
	// the cascades of the sun shadows make the same walks: the mode, the lod band and the state of the models are the same in all
	// of them within a frame, only the instances differ, and those the gpu fills in. the walk of a group in a band is made by the
	// first cascade which draws it, the others copy its draws and commands with entries of their own
	auto const cascade {
		( m_renderpass.draw_mode == rendermode::shadows )
		&& ( &Cull >= m_shadowcull.data() )
		&& ( &Cull < m_shadowcull.data() + m_shadowcull.size() ) };
	auto constexpr nowalk { std::numeric_limits<std::uint32_t>::max() };
	if( cascade && ( m_cascaderecordframe != m_framestamp ) ) {
		m_cascaderecordframe = m_framestamp;
		m_cascadedraws.clear();
		m_cascadecommands.clear();
		if( ( m_framestamp % 256 ) == 0 ) {
			// the groups not drawn for a while
			for( auto records { m_cascaderecords.begin() }; records != m_cascaderecords.end(); ) {
				if( m_framestamp - records->second.frame > 256 ) { records = m_cascaderecords.erase( records ); }
				else { ++records; }
			}
		}
	}
	m_indirectrecording = true;
	for( auto const &group : Cull.draw_groups() ) {
		auto *model { group.key->pModel };
		if( ( model == nullptr ) || ( model->Root == nullptr ) || ( ( group.instance == nullptr ) && ( false == group.sleepers ) ) ) { continue; }
		material_handle const *skins { nullptr };
		auto alpha { 0x30300030 };
		auto prepared { false };
		auto const prepare = [ & ]() {
			prepared = true;
			if( group.sleepers ) {
				// as Render_Sleepers() sets them up: the skin of the track, if any. like scenery models they pick no texture
				// variants of their own
				if( group.key->skins[ 1 ] != null_handle ) {
					skins = group.key->skins.data();
				}
				TSubModel::iVariantSeed = 0;
			}
			else {
				if( group.key->instance != nullptr ) {
					// a model drawn on its own: its lights and animations, the way Render( TAnimModel * ) sets them up
					group.instance->RaAnimate( m_framestamp );
				}
				group.instance->RaPrepare();
				auto const *material { group.instance->Material() };
				if( material != nullptr ) {
					skins = material->replacable_skins;
					alpha = material->textures_alpha;
				}
			}
			alpha ^= 0x0F0F000F;
		};
		// the walks of the group the cascades made in this frame. a group whose bands were all walked already isn't prepared
		cascade_record *walks { nullptr };
		if( cascade ) {
			walks = &m_cascaderecords[ *group.key ];
			if( ( walks->frame != m_framestamp ) || ( walks->bands.size() != group.lodcount + 1 ) ) {
				walks->frame = m_framestamp;
				walks->bands.assign( group.lodcount + 1, cascade_band { nowalk, 0 } );
			}
		}
		else {
			prepare();
		}
		for( std::uint32_t band = 0; band <= group.lodcount; ++band ) {
			// a band holds the squared distances from its lower bound up to the next one; any of them selects the same lod
			auto const bandmin { band == 0 ? 0.f : lodbounds[ group.lodfirst + band - 1 ] };
			auto const bandmax { band == group.lodcount ? std::numeric_limits<float>::max() : lodbounds[ group.lodfirst + band ] };
			if( ( bandmin > group.distancemax ) || ( bandmax <= group.distancemin ) ) { continue; }
			m_indirectentry = group.entryfirst + band;
			if( ( walks != nullptr ) && ( walks->bands[ band ].first != nowalk ) ) {
				// walked by another cascade: its draws, for the instances of this entry
				auto const &walk { walks->bands[ band ] };
				m_indirectdraws.insert( m_indirectdraws.end(), m_cascadedraws.begin() + walk.first, m_cascadedraws.begin() + walk.first + walk.count );
				for( std::uint32_t draw = 0; draw < walk.count; ++draw ) {
					auto &command { m_indirectcommands.emplace_back( m_cascadecommands[ walk.first + draw ] ) };
					command.entry = m_indirectentry;
				}
				// as the walk would count them
				m_renderpass.draw_stats.submodels += static_cast<int>( walk.count );
				m_renderpass.draw_stats.drawcalls += static_cast<int>( walk.count );
				continue;
			}
			if( false == prepared ) {
				prepare();
			}
			auto const walkfirst { m_indirectdraws.size() };
			::glPushMatrix();
			::glLoadIdentity();
			model->Root->fSquareDist = bandmin;
			model->Root->ReplacableSet( skins, alpha );
			model->Root->pRoot = model;
			Render( model->Root );
			::glPopMatrix();
			if( walks != nullptr ) {
				walks->bands[ band ] = cascade_band {
					static_cast<std::uint32_t>( m_cascadedraws.size() ),
					static_cast<std::uint32_t>( m_indirectdraws.size() - walkfirst ) };
				m_cascadedraws.insert( m_cascadedraws.end(), m_indirectdraws.begin() + walkfirst, m_indirectdraws.end() );
				m_cascadecommands.insert( m_cascadecommands.end(), m_indirectcommands.begin() + walkfirst, m_indirectcommands.end() );
			}
		}
	}
	m_indirectrecording = false;

	// with multi draws the draws with the same state follow one another, and their model blocks are in a buffer the programs
	// of the multi draws read them from. the order goes into the commands, so it comes first
	m_indirectmultidraw = ( Global.gfx_gpucull_multidraw && ( m_drawblockstream != nullptr ) );
	if( m_indirectmultidraw && ( false == m_indirectdraws.empty() ) ) {
		Order_Indirect_Draws();
		Upload_Indirect_Blocks();
	}

	Cull.fill_commands( m_indirectcommands );
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
	auto lastdrawn { count };
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
					&& ( m_indirectdraws[ last ].geometry.bank == draw.geometry.bank )
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
				lastdrawn = last - 1;
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
			++index;
		}
	}
	if( lastdrawn < count ) {
		// the model block the draws made one by one leave behind
		model_ubs = m_indirectdraws[ lastdrawn ].ubs;
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
				&& ( m_indirectdraws[ last ].geometry.bank == draw.geometry.bank )
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
	if( m_indirectmultidraw && ( count > 0 ) ) {
		// the model block the draws made one by one leave behind
		model_ubs = m_indirectdraws.back().ubs;
		model_ubo->update( model_ubs );
	}
	return counts;
}

texture_handle opengl46_renderer::Indirect_Shadow_Texture( material_handle const Material )
{
	if( Material == null_handle ) { return null_handle; }
	return m_materials.material( Material ).textures[ 0 ];
}

bool opengl46_renderer::Instances_Cast_Shadow( scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket )
{
	auto const lookup { m_shadowcasters.find( Key ) };
	if( lookup != m_shadowcasters.end() ) { return lookup->second; }

	// the skins of the instance Render_Instanced() would take them from
	TAnimModel const *instance { nullptr };
	for( auto const *candidate : Bucket.instances ) {
		if( candidate != nullptr ) {
			instance = candidate;
			break;
		}
	}
	auto const *skins { instance != nullptr && instance->Material() != nullptr ? instance->Material()->replacable_skins : nullptr };
	auto castsshadow { false };
	m_instance_lodpending.clear();
	m_instance_lodpending.push_back( Key.pModel != nullptr ? Key.pModel->Root : nullptr );
	while( ( false == castsshadow ) && ( false == m_instance_lodpending.empty() ) ) {
		auto const *submodel = m_instance_lodpending.back();
		m_instance_lodpending.pop_back();
		if( submodel == nullptr ) { continue; }
		if( submodel->eType < TP_ROTATOR ) {
			auto const material { submodel->m_material >= 0 ? submodel->m_material : ( skins != nullptr ? skins[ -submodel->m_material ] : null_handle ) };
			castsshadow = ( Material( material ).shadow_rank <= Global.gfx_shadow_rank_cutoff );
		}
		m_instance_lodpending.push_back( submodel->Child );
		m_instance_lodpending.push_back( submodel->Next );
	}
	m_shadowcasters.emplace( Key, castsshadow );
	return castsshadow;
}

// the order of the recorded draws for multi draws. draws go in one multi draw when they follow one another with the same
// state (material and its texture variant, geometry bank, primitive, blending; Indirect_Mergeable()). the draws of one entry,
// the walk of one submodel tree in one lod band, keep their order: submodels of a model may lie on one another and rely on
// the later one being drawn over the earlier one. the draws of different entries are drawn for different instances, which
// the gpu draws in no particular order anyway, so they're free to move. the entries are queues of draws; the draws at the
// heads of the queues which have the state taken next all go, each queue as far as its draws keep that state, then the
// queues wait for the state of their next draw. the states go in the order in which the queues first reach them
void opengl46_renderer::Order_Indirect_Draws()
{
	auto const count { m_indirectdraws.size() };
	if( count < 2 ) { return; }

	// a number for each state the draws have, by sorting the states (the buffers are kept between frames)
	m_indirectstatekeys.clear();
	// the sun shadows draw with the program and the texture of the alpha test, whatever the material
	auto const shadows { m_renderpass.draw_mode == rendermode::shadows };
	for( std::size_t index = 0; index < count; ++index ) {
		auto const &draw { m_indirectdraws[ index ] };
		if( shadows ) {
			m_indirectstatekeys.push_back( { Indirect_Shadow_Texture( draw.material ), 0u, draw.geometry.bank, draw.type, m_indirectcommands[ index ].indexed, draw.blending, static_cast<std::uint32_t>( index ) } );
			continue;
		}
		auto variant { 0u };
		if( draw.material != null_handle ) {
			auto const &material { m_materials.material( draw.material ) };
			if( false == material.texture_variants.empty() ) {
				variant = TSubModel::VariantId( draw.submodel ) % static_cast<std::uint32_t>( material.texture_variants.size() );
			}
		}
		m_indirectstatekeys.push_back( { draw.material, variant, draw.geometry.bank, draw.type, m_indirectcommands[ index ].indexed, draw.blending, static_cast<std::uint32_t>( index ) } );
	}
	std::sort(
		std::begin( m_indirectstatekeys ), std::end( m_indirectstatekeys ),
		[]( indirect_state_key const &Left, indirect_state_key const &Right ) {
			return Left.tie() < Right.tie(); } );
	m_indirectdrawstates.resize( count );
	std::uint32_t statecount { 0 };
	for( std::size_t index = 0; index < count; ++index ) {
		if( ( index > 0 ) && ( m_indirectstatekeys[ index ].tie() != m_indirectstatekeys[ index - 1 ].tie() ) ) {
			++statecount;
		}
		m_indirectdrawstates[ m_indirectstatekeys[ index ].draw ] = statecount;
	}
	++statecount;
	if( statecount == count ) {
		// no two draws share a state, nothing would go together
		return;
	}

	auto constexpr none { std::numeric_limits<std::uint32_t>::max() };
	m_indirectstates.assign( statecount, indirect_state { none, none } );
	m_indirectstateorder.clear();
	m_indirectqueues.clear();
	// the queue waits for the state of its next draw, behind the queues waiting for it already
	auto const enqueue = [ this, none ]( std::uint32_t const Queue ) {
		auto &queue { m_indirectqueues[ Queue ] };
		auto &state { m_indirectstates[ m_indirectdrawstates[ queue.next ] ] };
		queue.link = none;
		if( state.head == none ) {
			state.head = Queue;
			m_indirectstateorder.push_back( m_indirectdrawstates[ queue.next ] );
		}
		else {
			m_indirectqueues[ state.tail ].link = Queue;
		}
		state.tail = Queue;
	};
	for( std::size_t index = 0; index < count; ) {
		auto end { index + 1 };
		while( ( end < count ) && ( m_indirectcommands[ end ].entry == m_indirectcommands[ index ].entry ) ) { ++end; }
		m_indirectqueues.push_back( { static_cast<std::uint32_t>( index ), static_cast<std::uint32_t>( end ), none } );
		index = end;
	}
	for( std::uint32_t queue = 0; queue < m_indirectqueues.size(); ++queue ) {
		enqueue( queue );
	}
	m_indirectorder.clear();
	for( std::size_t taken = 0; taken < m_indirectstateorder.size(); ++taken ) {
		auto const stateindex { m_indirectstateorder[ taken ] };
		auto queueindex { m_indirectstates[ stateindex ].head };
		m_indirectstates[ stateindex ] = { none, none };
		while( queueindex != none ) {
			auto &queue { m_indirectqueues[ queueindex ] };
			auto const nextqueue { queue.link };
			while( ( queue.next < queue.end ) && ( m_indirectdrawstates[ queue.next ] == stateindex ) ) {
				m_indirectorder.push_back( queue.next );
				++queue.next;
			}
			if( queue.next < queue.end ) {
				enqueue( queueindex );
			}
			queueindex = nextqueue;
		}
	}

	m_indirectdrawsordered.clear();
	m_indirectcommandsordered.clear();
	for( auto const index : m_indirectorder ) {
		m_indirectdrawsordered.push_back( m_indirectdraws[ index ] );
		m_indirectcommandsordered.push_back( m_indirectcommands[ index ] );
	}
	m_indirectdraws.swap( m_indirectdrawsordered );
	m_indirectcommands.swap( m_indirectcommandsordered );
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

bool opengl46_renderer::Indirect_Mergeable( std::size_t const First, std::size_t const Other ) const
{
	auto const &first { m_indirectdraws[ First ] };
	auto const &other { m_indirectdraws[ Other ] };
	if( ( first.material != other.material )
	 || ( first.blending != other.blending )
	 || ( first.type != other.type )
	 || ( first.geometry.bank != other.geometry.bank )
	 || ( m_indirectcommands[ First ].indexed != m_indirectcommands[ Other ].indexed ) ) {
		return false;
	}
	if( first.material == null_handle ) { return true; }
	// the texture set Bind_Material() binds
	auto const &material { m_materials.material( first.material ) };
	if( material.texture_variants.empty() ) { return true; }
	return ( &material.GetTextures( TSubModel::VariantId( first.submodel ) ) == &material.GetTextures( TSubModel::VariantId( other.submodel ) ) );
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
	return material->shader.get();
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
			model_ubs.future = glm::mat4();
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
			model_ubs.future = glm::mat4();
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
		item.bank = chunk.geometry.bank;
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

namespace {

// the group of the sleepers of a track in the gpu culling: their model, and the skin in the slot Render_Sleepers() sets it in
scene::basic_cell::instance_bucket_key sleeper_key( TTrack const &Track ) {
	scene::basic_cell::instance_bucket_key key;
	key.pModel = Track.m_sleeper_model;
	key.skins[ 1 ] = Track.m_sleeper_skin;
	return key;
}

} // namespace

void opengl46_renderer::Queue_Sleepers( TTrack *Track )
{
	// the gates of Render_Sleepers(); the range is tested for the whole track
	if( ( Track->m_sleeper_model == nullptr ) || Track->m_sleeper_local_transforms.empty() || ( Global.SleeperDistance <= 0.f ) ) { return; }
	auto const distsq { glm::length2( Track->location() - m_renderpass.pass_camera.position() ) };
	auto const cutoffsq { static_cast<double>( Global.SleeperDistance ) * static_cast<double>( Global.SleeperDistance ) };
	if( distsq > cutoffsq ) { return; }

	auto &records { m_sleeperrecords[ Track ] };
	auto const &transforms { Track->m_sleeper_local_transforms };
	if( ( records.bucket.recordsversion != m_instancerecordsversion )
	 || ( records.model != Track->m_sleeper_model )
	 || ( records.skin != Track->m_sleeper_skin )
	 || ( records.transforms != transforms.data() )
	 || ( records.count != transforms.size() ) ) {
		// each sleeper is a record placed and turned by its transform, relative to the origin of the track (the centre of its
		// cell), with the sphere Render_Sleepers() tests against the frustum
		auto const radius { std::max( Track->m_sleeper_model->bounding_radius(), 2.0f ) };
		auto &bucket { records.bucket };
		bucket.records.clear();
		bucket.records.reserve( transforms.size() );
		glm::vec3 low { std::numeric_limits<float>::max() };
		glm::vec3 high { std::numeric_limits<float>::lowest() };
		for( auto const &local : transforms ) {
			scene::basic_cell::instance_record record;
			glm::vec3 const position { local[ 3 ] };
			record.position = packed_vec3{ position };
			record.radius = radius;
			record.rangesquaredmin = 0.f;
			record.rangesquaredmax = std::numeric_limits<float>::max();
			record.flags = scene::basic_cell::instance_visible | scene::basic_cell::instance_instanceable;
			for( int column = 0; column < 3; ++column ) {
				record.rotationscale[ column ] = local[ column ];
			}
			bucket.records.emplace_back( record );
			low = glm::min( low, position );
			high = glm::max( high, position );
		}
		bucket.recordorigin = Track->m_origin;
		bucket.lodrangesquared = std::numeric_limits<float>::max();
		bucket.recordsversion = m_instancerecordsversion;
		bucket.gpustale = true;
		records.key = sleeper_key( *Track );
		records.area = scene::bounding_area{ Track->m_origin + glm::dvec3{ ( low + high ) * 0.5f }, 0.5f * glm::length( high - low ) };
		records.model = Track->m_sleeper_model;
		records.skin = Track->m_sleeper_skin;
		records.transforms = transforms.data();
		records.count = transforms.size();
	}
	records.frame = m_framestamp;
	m_passcull->add_sleepers( records.key, records.bucket, records.area );
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

	if( m_sleeperspass && m_instancecullpass && ( false == m_passcull->drawing() ) ) {
		// the comparison with the gpu culling: the sleepers drawn by lod band, the bands as the culling makes them, split by
		// the bounds of the model past 0 and short of the largest float
		auto const counted = []( float const Bound ) {
			return ( Bound > 0.f ) && ( Bound < std::numeric_limits<float>::max() ); };
		auto &counts { m_sleeperbandcounts[ sleeper_key( *Track ) ] };
		auto const bands { static_cast<std::size_t>( std::count_if( lodbounds.begin(), lodbounds.end(), counted ) ) + 1 };
		if( counts.size() < bands ) {
			counts.resize( bands, 0 );
		}
		for( auto const &survivor : survivors ) {
			std::size_t band { 0 };
			for( auto const bound : lodbounds ) {
				if( counted( bound ) && ( bound <= survivor.first ) ) { ++band; }
			}
			++counts[ band ];
		}
	}
}

// Renders the sleepers and switch timbers of a run of tracks of a cell. Each beam is made of instances of the parts of a model or
// of a whole model, kept by its track relative to the origin of the track. The runs of beams in range and in sight are gathered
// for the whole run of tracks by the model and its part or its level of detail, and each is drawn in one instanced batch.
// In the shadow pass only those near the camera are drawn, their shadows falling where they can be seen.
void opengl46_renderer::Render_Timbers( scene::basic_cell::path_sequence::const_iterator First, scene::basic_cell::path_sequence::const_iterator Last )
{
	if( Global.SleeperDistance <= 0.f ) { return; }

	auto const shadows = ( m_renderpass.draw_mode == rendermode::shadows );
	// the distance is taken from the camera of the view, the shadow pass culling against its own frustum
	auto const camera = ( shadows ? m_renderpass.viewport_camera.position() : m_renderpass.pass_camera.position() );
	auto const range = static_cast<double>( shadows ? std::min( Global.SleeperDistance, 50.f ) : Global.SleeperDistance );
	if( m_renderpass.draw_mode == rendermode::color && m_timbersweepframe != m_framestamp ) {
		// once a frame the beams of the tracks well past where they're laid and worked out ahead are let go
		m_timbersweepframe = m_framestamp;
		track_sleepers::release_far( camera, range + 600.0 );
	}
	glm::mat4 const originmodelview = OpenGLMatrices.data( GL_MODELVIEW );
	for( auto &bucket : m_timberbuckets ) {
		bucket.instances.clear();
	}
	auto const bucket_for = [&]( track_sleepers::beam_model const *Model, int const Part, float const Band ) -> std::vector<glm::mat4> & {
		for( auto &bucket : m_timberbuckets ) {
			if( bucket.model == Model && bucket.part == Part && bucket.band == Band ) {
				return bucket.instances;
			}
		}
		m_timberbuckets.push_back( { Model, Part, Band, {} } );
		return m_timberbuckets.back().instances; };
	// the level of detail of the whole model a beam is seen at, from its distance
	auto const band_of = []( track_sleepers::beam_model const &Model, double const Distance ) {
		auto const squaredistance { static_cast<float>( Distance * Distance ) };
		auto band { Model.bands.empty() ? 0.f : Model.bands.front() };
		for( auto const start : Model.bands ) {
			if( squaredistance >= start ) {
				band = start;
			}
		}
		return band; };
	// the beams of a run made of a whole model, each at the level of detail of its own distance; the run is culled as one
	auto const put_by_band = [&]( track_sleepers::beam_model const &Model, std::vector<glm::mat4> const &Transforms, std::uint32_t const From, std::uint32_t const To, glm::dvec3 const &Origin ) {
		std::vector<glm::mat4> *instances { nullptr };
		auto current { 0.f };
		for( auto index = From; index < To; ++index ) {
			auto const &local = Transforms[ index ];
			auto const band = band_of( Model, glm::length( Origin + glm::dvec3( glm::vec3( local[ 3 ] ) ) - camera ) );
			if( instances == nullptr || band != current ) {
				// looked up again only as the band changes; a bucket added on the way moves the others, so the pointer goes with it
				instances = &bucket_for( &Model, -1, band );
				current = band;
			}
			instances->emplace_back( originmodelview * local );
		}
	};
	for( auto first { First }; first != Last; ++first ) {
		auto *track = *first;
		if( false == track->m_visible || false == track->m_sleepers3d.active ) { continue; }
		if( glm::length( track->location() - camera ) > range + track->radius() ) {
			// worked out ahead, in the background, before it comes in range
			if( false == shadows && track->m_sleepers3d.dirty && glm::length( track->location() - camera ) < range + track->radius() + 300.0 ) {
				track_sleepers::prefetch( *track );
			}
			continue;
		}
		if( track->m_sleepers3d.dirty ) {
			track_sleepers::lay( *track );
		}
		auto const &laid = track->m_sleepers3d;
		if( laid.timbers == nullptr ) { continue; }
		// the transforms of the beams are relative to the origin of the track, which the modelview is translated to already
		auto const &trackmodelview = originmodelview;
		for( auto const &chunk : laid.chunks ) {
			auto const centre = track->m_origin + glm::dvec3( chunk.centre );
			auto const distance = glm::length( centre - camera );
			if( distance > range + chunk.radius ) { continue; }
			if( false == m_renderpass.pass_camera.visible( scene::bounding_area{ centre, chunk.radius } ) ) { continue; }
			if( laid.timbers->stretchable ) {
				for( int part = 0; part < track_sleepers::part_count; ++part ) {
					auto &instances = bucket_for( laid.timbers, part, 0.f );
					for( auto index = chunk.first[ part ]; index < chunk.last[ part ]; ++index ) {
						instances.emplace_back( trackmodelview * laid.instances[ part ][ index ] );
					}
				}
			}
			if( chunk.wholelast > chunk.wholefirst ) {
				put_by_band( *laid.timbers, laid.whole, chunk.wholefirst, chunk.wholelast, track->m_origin );
			}
			if( laid.plains != nullptr && chunk.plainlast > chunk.plainfirst ) {
				put_by_band( *laid.plains, laid.plain, chunk.plainfirst, chunk.plainlast, track->m_origin );
			}
		}
	}

	auto const alpha = ( 0x30300030 ^ 0x0F0F000F );
	std::size_t total { 0 };
	for( auto &bucket : m_timberbuckets ) {
		auto const &instances = bucket.instances;
		if( instances.empty() ) { continue; }
		total += instances.size();
		auto const &beammodel = *bucket.model;
		auto *model = beammodel.model;
		if( bucket.part >= 0 ) {
			// only the part of the model is visible, its siblings are skipped as the model is walked
			for( int part = 0; part < track_sleepers::part_count; ++part ) {
				beammodel.parts[ part ]->iVisible = ( part == bucket.part ? 1 : 0 );
			}
		}
		material_data skins {};
		if( beammodel.skin != null_handle ) {
			skins.replacable_skins[ 1 ] = beammodel.skin;
		}
		// the instances go to the instance stream of the frame, as in Render_Sleepers(), which the recorded walk of the depth
		// pass keeps for its replay as well
		auto const baseinstance { Upload_Instance_Modelviews( instances.data(), instances.size() ) };
		::glPushMatrix();
		::glLoadIdentity();
		m_current_instance_count = instances.size();
		m_current_instance_base = baseinstance;
		// inside the range of the level of detail the beams are at
		model->Root->fSquareDist = ( bucket.part >= 0 ? 0.f : bucket.band + 1.f );
		model->Root->ReplacableSet( ( beammodel.skin != null_handle ? skins.replacable_skins : nullptr ), alpha );
		model->Root->pRoot = model;
		Render( model->Root );
		m_current_instance_count = 0;
		m_current_instance_base = 0;
		::glPopMatrix();
		++m_renderpass.draw_stats.instanced_drawcalls;
		if( bucket.part >= 0 ) {
			for( int part = 0; part < track_sleepers::part_count; ++part ) {
				beammodel.parts[ part ]->iVisible = 1;
			}
		}
	}
	m_renderpass.draw_stats.instances += static_cast<int>( total );
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
	// the velocity it leaves in the colour pass keeps the motion blur on (Render_pass())
	if ((m_renderpass.draw_mode == rendermode::color) && (Dynamic->get_future_movement() != glm::dvec3{0.0}))
		m_vehiclesmoving = true;

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
		for (auto &tread : Track->GeometryTreads)
			draw(std::begin(tread.geometry), std::end(tread.geometry));
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
				Bind_Material(track->m_blendmaterial1 != null_handle ? track->m_blendmaterial1 : track->m_material1);
				draw(std::begin(track->Geometry1), std::end(track->Geometry1));
				// restore default lighting
				setup_environment_light();
				break;
			}
			if (false == begin_batch(track->m_blendmaterial1 != null_handle ? track->m_blendmaterial1 : track->m_material1))
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
		if (track->eType == tt_Normal && editor_ballast_hidden(track, m_renderpass.draw_mode == rendermode::color))
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
				Bind_Material(track->m_blendmaterial2 != null_handle ? track->m_blendmaterial2 : track->m_material2);
				draw(std::begin(track->Geometry2), std::end(track->Geometry2));
				// restore default lighting
				setup_environment_light();
				break;
			}
			begin_batch(track->m_blendmaterial2 != null_handle ? track->m_blendmaterial2 : track->m_material2);
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
		if (editor_ballast_hidden(track, m_renderpass.draw_mode == rendermode::color))
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
				Bind_Material(track->m_blendmaterial3 != null_handle ? track->m_blendmaterial3 : track->SwitchExtension->m_material3);
				draw(track->SwitchExtension->Geometry3);
				// restore default lighting
				setup_environment_light();
				break;
			}
			begin_batch(track->m_blendmaterial3 != null_handle ? track->m_blendmaterial3 : track->SwitchExtension->m_material3);
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

	// cut faces of rails, in matte variants of the rail materials
	if ((m_renderpass.draw_mode == rendermode::color) || (m_renderpass.draw_mode == rendermode::reflections))
	{
		for (auto first{First}; first != Last; ++first)
		{
			auto const track{*first};

			if (false == track->m_visible)
			{
				continue;
			}
			std::vector<std::pair<material_handle, TTrack::geometryhandle_sequence *>> parts{{track->m_materialcut1, &track->GeometryCut1}, {track->m_materialcut2, &track->GeometryCut2}};
			for (auto &tread : track->GeometryTreads)
			{
				parts.emplace_back(tread.material, &tread.geometry);
				parts.emplace_back(tread.matte, &tread.cuts);
			}
			for (auto const &cuts : parts)
			{
				if ((cuts.first == null_handle) || cuts.second->empty())
				{
					continue;
				}
				if (track->eEnvironment != e_flat)
				{
					flush_batch();
					setup_environment_light(track->eEnvironment);
					Bind_Material(cuts.first);
					draw(std::begin(*cuts.second), std::end(*cuts.second));
					setup_environment_light();
					continue;
				}
				begin_batch(cuts.first);
				m_pathbatch.insert(std::end(m_pathbatch), std::begin(*cuts.second), std::end(*cuts.second));
			}
		}
		flush_batch();
	}

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
			// a track with the sleepers and timbers of eu07.ini lays its sleeper model there
			if( track->m_sleepers3d.active )      { continue; }
			if( m_sleeperspass ) {
				// culled with the instances of the pass and drawn from the result, or drawn by the cpu as well to compare
				Queue_Sleepers( track );
				if( m_passcull->drawing() ) { continue; }
			}
			else if( m_indirectcolourpending && m_indirectcoloursleepers ) {
				// the depth pass of the whole scene culled and recorded them, the colour pass draws what it recorded
				continue;
			}
			Render_Sleepers( track );
		}
		if( m_renderpass.draw_mode == rendermode::color ) {
			Render_Timbers( First, Last );
		}
		break;
	}
	case rendermode::shadows: {
		Render_Timbers( First, Last );
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
	// the velocity it leaves in the colour pass keeps the motion blur on (Render_pass())
	if ((m_renderpass.draw_mode == rendermode::color) && (Dynamic->get_future_movement() != glm::dvec3{0.0}))
		m_vehiclesmoving = true;

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
	m_lampscenerydrawn = 0;
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
	m_headlightshadowpicked.clear();
	auto const count{(Global.gfx_lights_headlights && (std::clamp(Global.gfx_lights_shadows, 0, 3) > 0)) ? std::clamp(Global.gfx_lights_shadows_headlights, 0, static_cast<int>(gl::MAX_HEADLIGHT_SHADOWS)) : 0};
	if (count > 0)
	{
		m_headlightshadowcandidates.clear();
		auto const camera{m_colorpass.pass_camera.position()};
		auto const range{static_cast<double>(Global.gfx_lights_shadows_headlights_range)};
		for (auto const &light : simulation::Lights.data)
		{
			if (light.intensity == 0.f)
				continue;
			// as in Update_Lights(): by day the light around takes the beam down to nothing
			auto luminance{static_cast<float>(Global.fLuminance)};
			if (light.owner->fShade > 0.f)
				luminance *= light.owner->fShade;
			auto const beam{light.color - glm::vec3{luminance}};
			if (std::max(beam.r, std::max(beam.g, beam.b)) <= 0.f)
				continue;
			// the part of the beam the shadows are cast in: from the end of the vehicle to the reach ahead of it
			auto const &dimensions{light.owner->MoverParameters->Dim};
			auto const radius{range * 0.5 + std::max(dimensions.W, dimensions.H) + 2.0};
			auto const centre{light.position + glm::dvec3{light.direction} * (range * 0.5 + 2.0)};
			if (glm::length(centre - camera) - radius > range)
				continue;
			if (false == m_colorpass.pass_camera.visible(scene::bounding_area{centre, static_cast<float>(radius)}))
				continue;
			m_headlightshadowcandidates.emplace_back(static_cast<float>(glm::length2(camera - light.position)) / light.intensity, &light);
		}
		auto const picked{std::min(m_headlightshadowcandidates.size(), static_cast<std::size_t>(count))};
		std::partial_sort(std::begin(m_headlightshadowcandidates), std::begin(m_headlightshadowcandidates) + picked, std::end(m_headlightshadowcandidates),
		                  [](auto const &Left, auto const &Right) { return Left.first < Right.first; });
		for (std::size_t idx = 0; idx < picked; ++idx)
		{
			m_headlightshadowpicked.emplace_back(m_headlightshadowcandidates[idx].second);
		}
	}
	m_localshadows.update_headlights(m_headlightshadowpicked, m_framestamp);
	// the maps may have been made anew, in place of the ones bound for this frame
	m_localshadows.bind();
	auto const &slots{m_localshadows.headlights()};
	if (slots.empty())
		return;

	// the occupied cab, as in Render_lamp_shadows()
	TDynamicObject *cab{nullptr};
	if ((false == FreeFlyModeFlag) && Global.render_cab && (simulation::Train != nullptr))
	{
		cab = simulation::Train->Dynamic();
		if ((cab != nullptr) && ((false == cab->bDisplayCab) || (cab->mdKabina == nullptr) || (cab->mdKabina == cab->mdModel)))
			cab = nullptr;
	}
	auto timing{false};
	for (auto const idx : slots)
	{
		auto const &slot{m_localshadows.headlight(idx)};
		light_array::light_record const *light{nullptr};
		for (auto const *picked : m_headlightshadowpicked)
		{
			if ((picked->owner == slot.owner) && (picked->index == slot.index))
			{
				light = picked;
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
	// cap the vertical angle to keep shadows from getting too long
	return glm::normalize(glm::vec3{m_sunlight.direction.x, std::min(m_sunlight.direction.y, Global.gfx_shadow_angle_min), m_sunlight.direction.z});
}

glm::mat3 opengl46_renderer::Shadow_Light_Rotation() const
{
	// the rotation of the view of the shadow pass (setup_pass())
	return glm::mat3{glm::lookAt(glm::vec3{0.f}, Shadow_Light_Vector(), glm::vec3{0.f, 1.f, 0.f})};
}

std::array<float, gl::MAX_CASCADES + 1> opengl46_renderer::Shadow_Fixed_Splits() const
{
	static_assert(gl::MAX_CASCADES == 3, "the fixed cascades are made for 3 of them");
	std::array<float, gl::MAX_CASCADES + 1> splits{0.0f, Global.shadowtune.range / 32, Global.shadowtune.range / 8, Global.shadowtune.range};
	if (Global.shadowtune.map_size > 2048)
	{
		// increase coverage if our shadow map is large enough to produce decent results
		for (std::size_t idx = 1; idx < splits.size(); ++idx)
			splits[idx] *= Global.shadowtune.map_size / 2048;
	}
	return splits;
}

char const *opengl46_renderer::Shadow_Fit_Unavailable(viewport_config const &Viewport) const
{
	if (false == Global.gfx_shadowmap_enabled)
		return "no shadow maps";
	if (Global.bWireFrame)
		return "wireframe";
	// the shadows follow the camera, the view shows another one
	if (DebugCameraFlag)
		return "debug camera";
	if (Global.gfx_skippipeline || (Viewport.msaa_fb == nullptr))
		return "no post processing";
	if (EditorModeFlag && Global.EditorOrtho)
		return "orthographic view";
	auto const shadowviews{std::count_if(m_viewports.begin(), m_viewports.end(), [](auto const &View) { return View->shadow; })};
	if ((false == Viewport.shadow) || (shadowviews != 1))
		return "the shadows cover more than one view";
	return nullptr;
}

bool opengl46_renderer::Shadow_Fit(viewport_config const &Viewport, std::array<float, gl::MAX_CASCADES + 1> &Splits,
                                   std::array<std::optional<std::array<glm::vec3, 8>>, gl::MAX_CASCADES> &Areas)
{
	// a measurement older than this many frames, made with the camera this many metres away or the view turned this many
	// degrees from the one now, isn't used
	constexpr unsigned int AGE_MAX{6};
	constexpr float SHIFT_MAX{25.f};
	constexpr float TURN_MAX{20.f};
	// metres an area grows by on each side: the vehicles move in the frames since the measurement too
	constexpr float MARGIN{0.25f};

	auto &state{m_shadowfitstate};
	state = shadow_fit_state{};
	if (false == Global.gfx_shadow_sdsm)
		return false;
	if ((state.reason = Shadow_Fit_Unavailable(Viewport)) != nullptr)
		return false;
	auto const *latest{m_shadowfit.latest()};
	if (latest == nullptr)
	{
		state.reason = "no measurement yet";
		return false;
	}
	auto const &data{latest->data};
	auto const &source{latest->source};
	state.age = m_framestamp - latest->frame;
	state.depthmin = data.depthmin;
	state.depthmax = data.depthmax;
	if (state.age > AGE_MAX)
	{
		state.reason = "the measurement is too old";
		return false;
	}
	if (data.samples == 0)
	{
		state.reason = "only the sky in the view";
		return false;
	}
	if (source.range != Shadow_Fixed_Splits().back())
	{
		state.reason = "the range of the shadows changed";
		return false;
	}
	// the shadows of this frame are made for its colour pass
	auto const &camera{m_colorpass.pass_camera};
	auto const shift{glm::vec3{source.camera - camera.position()}};
	if (glm::length(shift) > SHIFT_MAX)
	{
		state.reason = "the camera jumped";
		return false;
	}
	auto const turnmatrix{glm::mat3{camera.modelview()} * glm::transpose(source.rotation)};
	auto const turn{std::acos(std::clamp((turnmatrix[0][0] + turnmatrix[1][1] + turnmatrix[2][2] - 1.f) * 0.5f, -1.f, 1.f))};
	state.turn = glm::degrees(turn);
	if (state.turn > TURN_MAX)
	{
		state.reason = "the view turned too fast";
		return false;
	}
	auto const &projection{camera.projection()};
	if ((std::abs(projection[0][0] - source.projection[0][0]) > 0.02f * std::abs(projection[0][0]))
	 || (std::abs(projection[1][1] - source.projection[1][1]) > 0.02f * std::abs(projection[1][1])))
	{
		state.reason = "the field of view changed";
		return false;
	}

	// the first cascade from the camera, like the fixed ones. setup_pass() takes distances up to 1 m as shares of the far
	// end, so the others start past that
	Splits[0] = 0.f;
	for (std::size_t split = 1; split < Splits.size(); ++split)
		Splits[split] = std::max(data.splits[split], Splits[split - 1] + 1.5f);
	if (false == std::isfinite(Splits.back()))
	{
		state.reason = "invalid measurement";
		return false;
	}

	auto const inverselight{glm::transpose(source.light)};
	// the scenery stays where it was while the camera moved, the cab and the train the camera rides with moved along. an area
	// covers both
	auto const lightshift{source.light * shift};
	auto const chord{2.f * std::sin(turn * 0.5f)};
	for (std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade)
	{
		// with nothing of the view in a cascade, its map covers the whole slice
		if (data.counts[cascade] == 0)
			continue;
		auto const *bounds{&data.bounds[cascade * 8]};
		glm::vec3 low{opengl46_shadow_fit::unorder(bounds[0]), opengl46_shadow_fit::unorder(bounds[1]), opengl46_shadow_fit::unorder(bounds[2])};
		glm::vec3 high{opengl46_shadow_fit::unorder(bounds[4]), opengl46_shadow_fit::unorder(bounds[5]), opengl46_shadow_fit::unorder(bounds[6])};
		if ((false == std::isfinite(low.x + low.y + high.x + high.y)) || (low.x > high.x) || (low.y > high.y))
			continue;
		low = glm::min(low, low + lightshift);
		high = glm::max(high, high + lightshift);
		// and what came into the view as it turned, out to the reach of the cascade
		auto const reach{Splits[cascade + 1] + static_cast<float>(gl::MAX_CASCADES - cascade)};
		auto const margin{reach * chord + MARGIN + 0.01f * std::max(high.x - low.x, high.y - low.y)};
		low -= glm::vec3{margin};
		high += glm::vec3{margin};
		auto &area{Areas[cascade].emplace()};
		for (int corner = 0; corner < 8; ++corner)
			area[corner] = inverselight * glm::vec3{(corner & 1) ? high.x : low.x, (corner & 2) ? high.y : low.y, (corner & 4) ? high.z : low.z};
	}
	state.fitted = true;
	return true;
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

