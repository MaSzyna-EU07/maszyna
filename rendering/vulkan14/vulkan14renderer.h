/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the vulkan 1.4 renderer, picked with gfxrenderer vulkan. so far the frame of the renderer only: the gpu, the windows with
// their swapchains, the frames in flight; each window is cleared to a colour. the scenery, the materials and the textures
// come in the later stages of the plan (claude/vulkan14-renderer-plan.md); until then those methods keep the data the
// simulation reads back and draw nothing.
//
// the vulkan headers stay in the files of rendering/vulkan14: the objects of the api live in vulkan14_renderer::context

#pragma once

#include "rendering/renderer.h"
#include "rendering/geometrybank.h"

class vulkan14_renderer : public gfx_renderer
{
  public:
	vulkan14_renderer();
	~vulkan14_renderer() override;

	// lifecycle
	bool Init(GLFWwindow *Window) override;
	bool AddViewport(global_settings::extraviewport_config const &conf) override;
	void Shutdown() override;
	bool Render() override;
	void SwapBuffers() override;
	float Framerate() override;
	// geometry
	gfx::geometrybank_handle Create_Bank() override;
	gfx::geometry_handle Insert(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type) override;
	gfx::geometry_handle Insert(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type) override;
	bool Replace(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type, std::size_t const Offset = 0) override;
	bool Append(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type) override;
	gfx::index_array const &Indices(gfx::geometry_handle const &Geometry) const override;
	gfx::vertex_array const &Vertices(gfx::geometry_handle const &Geometry) const override;
	gfx::userdata_array const &UserData(gfx::geometry_handle const &Geometry) const override;
	// materials
	material_handle Fetch_Material(std::string const &Filename, bool const Loadnow = true) override;
	void Bind_Material(material_handle const Material, TSubModel const *sm = nullptr, lighting_data const *lighting = nullptr) override;
	IMaterial const *Material(material_handle const Material) const override;
	// shaders
	std::shared_ptr<gl::program> Fetch_Shader(std::string const &name) override;
	// textures
	texture_handle Fetch_Texture(std::string const &Filename, bool const Loadnow = true, GLint format_hint = GL_SRGB_ALPHA) override;
	void Bind_Texture(texture_handle const Texture) override;
	void Bind_Texture(std::size_t const Unit, texture_handle const Texture) override;
	ITexture &Texture(texture_handle const Texture) override;
	ITexture const &Texture(texture_handle const Texture) const override;
	// picking
	void Pick_Control_Callback(std::function<void(TSubModel const *, glm::vec2 const)> Callback) override;
	void Pick_Node_Callback(std::function<void(scene::basic_node *)> Callback) override;
	TSubModel const *Pick_Control() const override;
	scene::basic_node const *Pick_Node() const override;
	glm::dvec3 Mouse_Position() const override;
	// maintenance
	void Update(double const Deltatime) override;
	void Update_Pick_Control() override;
	void Update_Pick_Node() override;
	glm::dvec3 Update_Mouse_Position() override;
	bool Debug_Ui_State(std::optional<bool> State) override;
	// debug
	std::string const &info_times() const override;
	std::string const &info_stats() const override;
	// ui
	imgui_renderer *GetImguiRenderer() override;
	void MakeScreenshot() override;

	static std::unique_ptr<gfx_renderer> create_func();
	// how the application makes the windows of the renderer, and whether it can run on the machine
	static gfx_backend_desc backend_desc();

  private:
	// the objects of the vulkan api (vulkan14renderer.cpp)
	struct context;

	// imgui without drawing: the frames of the ui go on (its input works), nothing is drawn until the stage R5 of the plan
	class imgui_stub : public imgui_renderer
	{
	  public:
		bool Init() override;
		void Shutdown() override;
		void BeginFrame() override;
		void Render() override;
	};

	void update_info();

	std::unique_ptr<context> m_context;
	// the data of the geometry, which the simulation reads back; nothing on the gpu yet
	gfx::geometrybank_manager m_geometry;
	imgui_stub m_imgui;
	float m_framerate{0.f};
	double m_updateaccumulator{0.0};
	bool m_debug_ui_active{false};
	std::string m_infotimes;
	std::string m_infostats;

	static bool renderer_register;
};
