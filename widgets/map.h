#pragma once

#include "application/uilayer.h"
#include "widgets/map_objects.h"
#include "widgets/popup.h"
#include "rendering/frustum.h"

namespace ui
{

class disambiguation_popup : public popup
{
	map::sorted_object_list m_list;

  public:
	disambiguation_popup(ui_panel &panel, map::sorted_object_list &&list);

	virtual void render_content() override;
};

class semaphore_window : public popup
{
	std::shared_ptr<map::semaphore> m_sem;
	command_relay m_relay;

  public:
	semaphore_window(ui_panel &panel, std::shared_ptr<map::semaphore> &&sem);

	virtual void render_content() override;
};

class launcher_window : public popup
{
	std::shared_ptr<map::launcher> m_switch;
	command_relay m_relay;

  public:
	launcher_window(ui_panel &panel, std::shared_ptr<map::launcher> &&sw);

	virtual void render_content() override;
};

class track_switch_window : public popup
{
    std::shared_ptr<map::track_switch> m_switch;
    command_relay m_relay;

  public:
    track_switch_window(ui_panel &panel, std::shared_ptr<map::track_switch> &&sw);

	virtual bool render() override;
    virtual void render_content() override;
};

class obstacle_insert_window : public popup
{
	glm::dvec3 m_position;
	std::vector<std::pair<std::string, std::string>> m_obstacles;
	command_relay m_relay;

  public:
	obstacle_insert_window(ui_panel &panel, glm::dvec3 const &pos);

	virtual void render_content() override;
};

class obstacle_remove_window : public popup
{
	std::shared_ptr<map::obstacle> m_obstacle;
	command_relay m_relay;

  public:
	obstacle_remove_window(ui_panel &panel, std::shared_ptr<map::obstacle> &&obstacle);

	virtual void render_content() override;
};

class vehicle_click_window : public popup
{
	std::shared_ptr<map::vehicle> m_vehicle;
	command_relay m_relay;

	std::array<char, 128> command_buf = { 0 };
	int command_param1 = 0;
	int command_param2 = 0;

  public:
	vehicle_click_window(ui_panel &panel, std::shared_ptr<map::vehicle> &&obstacle);

	virtual void render_content() override;
};

class map_panel : public ui_panel
{
    friend class track_switch_window;

	std::vector<gfx::geometrybank_handle> m_section_handles;
	map_colored_paths m_colored_paths;

    const int fb_size = 1024;

	glm::vec2 translate;
	float zoom = 1.0f / 1000.0f;
	float m_manual_rotation { 0.0f };
	enum { MODE_MANUAL = 0, MODE_CAMERA, MODE_VEHICLE } mode = MODE_MANUAL;
	bool m_camera_click_pending { false };
	double m_camera_click_deadline { 0.0 };
	glm::dvec3 m_camera_click_position{};
	std::shared_ptr<map::map_object> m_hovered_map_object;

	float get_vehicle_rotation();
	void collect_map_geometry(glm::mat4 transform);
	void render_grid(ImDrawList *draw_list, ImVec2 origin, glm::vec2 surface_size, glm::mat4 transform);
	void render_map_lines(ImDrawList *draw_list, ImVec2 origin, glm::vec2 surface_size, glm::mat4 transform);
	void render_labels(glm::mat4 transform, ImVec2 origin, glm::vec2 surface_size);

	bool init_done = false;

    std::vector<std::pair<TTrack*, int>> highlighted_switches;

  public:
	map_panel();
	void render_contents() override;
};

void handle_map_object_click(ui_panel &parent, std::shared_ptr<map::map_object> &obj);
void handle_map_object_hover(std::shared_ptr<map::map_object> &obj);
} // namespace ui
