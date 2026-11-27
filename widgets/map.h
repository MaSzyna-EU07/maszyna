#pragma once

#include <cstdint>
#include <chrono>
#include <cstddef>
#include <unordered_map>

#include "application/uilayer.h"
#include "widgets/map_objects.h"
#include "widgets/popup.h"

class TAnimModel;

namespace ui
{

class disambiguation_popup : public popup
{
	map::sorted_object_list m_list;

  public:
	disambiguation_popup(ui_panel &panel, map::sorted_object_list &&list);

	void render_content() override;
};

class semaphore_popup : public popup
{
	std::shared_ptr<map::semaphore> m_sem;
	command_relay m_relay;

  public:
	semaphore_popup(ui_panel &panel, std::shared_ptr<map::semaphore> &&sem);

	virtual void render_content() override;
};

class event_launcher_popup : public popup
{
	std::shared_ptr<map::launcher> m_switch;
	command_relay m_relay;

  public:
	event_launcher_popup(ui_panel &panel, std::shared_ptr<map::launcher> &&sw);

	virtual void render_content() override;
};

class track_switch_popup : public popup
{
    std::shared_ptr<map::track_switch> m_switch;
    command_relay m_relay;

  public:
    track_switch_popup(ui_panel &panel, std::shared_ptr<map::track_switch> &&sw);

	virtual bool render() override;
    virtual void render_content() override;
};

class obstacle_insert_popup : public popup
{
	glm::dvec3 m_position;
	std::vector<std::pair<std::string, std::string>> m_obstacles;
	command_relay m_relay;

  public:
	obstacle_insert_popup(ui_panel &panel, glm::dvec3 const &pos);

	virtual void render_content() override;
};

class obstacle_remove_popup : public popup
{
	std::shared_ptr<map::obstacle> m_obstacle;
	command_relay m_relay;

  public:
	obstacle_remove_popup(ui_panel &panel, std::shared_ptr<map::obstacle> &&obstacle);

	virtual void render_content() override;
};

class vehicle_popup : public popup
{
	std::shared_ptr<map::vehicle> m_vehicle;
	command_relay m_relay;

	std::array<char, 128> command_buf = { 0 };
	int command_param1 = 0;
	int command_param2 = 0;

  public:
	vehicle_popup(ui_panel &panel, std::shared_ptr<map::vehicle> &&obstacle);

	virtual void render_content() override;
};

class map_panel : public ui_panel
{
    friend class track_switch_popup;

	struct signal_track_entry
	{
		TTrack *track { nullptr };
		glm::dvec2 minimum {};
		glm::dvec2 maximum {};
		std::size_t sequence_order { 0 };
	};

	struct signal_track_node
	{
		glm::dvec2 minimum {};
		glm::dvec2 maximum {};
		std::size_t first { 0 };
		std::size_t last { 0 };
		int left { -1 };
		int right { -1 };
	};

	struct cached_signal_light
	{
		TAnimModel *model { nullptr };
		std::uint64_t revision { 0 };
		std::chrono::steady_clock::time_point refreshed_at {};
		glm::dvec2 direction {};
		std::vector<std::uint32_t> colors;
		bool valid { false };
		bool direction_valid { false };
	};

	struct cached_semaphore
	{
		std::vector<cached_signal_light> lights;
		cached_signal_light fallback;
	};

	struct cached_world_lines
	{
		std::vector<glm::vec3> vertices;
		glm::dvec2 minimum;
		glm::dvec2 maximum;
		ImU32 color { 0 };
		std::vector<ImU32> line_colors;
		std::vector<float> line_thicknesses;
	};

	struct cached_water_outline
	{
		std::vector<glm::dvec2> points;
		double level;
		glm::dvec2 minimum;
		glm::dvec2 maximum;
	};

	struct cached_station_label
	{
		std::string name;
		glm::dvec3 position;
	};

	std::vector<gfx::geometrybank_handle> m_section_handles;
	std::vector<glm::vec4> m_cached_section_lines;
	std::vector<glm::vec4> m_cached_road_lines;
	map_colored_paths m_colored_paths;
	scene::basic_region *m_cached_region { nullptr };
	glm::mat4 m_cached_view_transform { 0.0f };
	bool m_static_view_cache_valid { false };
	std::vector<TTrack *> m_visible_switch_tracks;
	scene::basic_region *m_static_scene_region { nullptr };
	std::vector<TTrack *> m_switch_tracks;
	std::vector<signal_track_entry> m_signal_tracks;
	std::vector<signal_track_node> m_signal_track_tree;
	std::unordered_map<map::semaphore const *, cached_semaphore> m_signal_caches;
	std::vector<cached_water_outline> m_water_outlines;
	std::vector<cached_world_lines> m_river_lines;
	std::vector<cached_world_lines> m_environment_lines;
	std::vector<cached_world_lines> m_platform_lines;
	std::vector<cached_station_label> m_station_labels;
	scene::basic_region *m_dynamic_tracks_region { nullptr };
	std::uint64_t m_cached_dynamic_revision { 0 };
	bool m_dynamic_tracks_cache_valid { false };
	std::vector<TTrack *> m_occupied_tracks;

    const int fb_size = 1024;

	glm::vec2 translate;
	float zoom = 1.0f / 1000.0f;
	float m_manual_rotation { 0.0f };
	enum { MODE_MANUAL = 0, MODE_CAMERA, MODE_VEHICLE } mode = MODE_MANUAL;
	std::shared_ptr<map::map_object> m_hovered_map_object;

	float get_vehicle_rotation();
	void prepare_static_scene_cache();
	int build_signal_track_tree(std::size_t first, std::size_t last);
	void find_nearest_signal_track(
		int node_index, glm::dvec3 const &position, TTrack *&nearest_track,
		double &nearest_distance2, std::size_t &nearest_order) const;
	glm::dvec2 map_signal_direction(glm::dvec3 const &position, float facing_angle) const;
	void cache_signal_directions();
	void cache_static_water();
	void cache_static_track_lines(TTrack *track);
	void cache_static_station_labels();
	void cache_static_platforms();
	static void append_cached_line(
		cached_world_lines &lines, glm::vec3 const &first, glm::vec3 const &second,
		ImU32 color, float thickness);
	void prepare_static_view_cache(glm::mat4 const &transform);
	void collect_map_geometry(glm::mat4 transform);
	void render_grid(ImDrawList *draw_list, ImVec2 origin, glm::vec2 surface_size, glm::mat4 transform);
	void render_map_lines(ImDrawList *draw_list, ImVec2 origin, glm::vec2 surface_size, glm::mat4 transform, double scale_length);
	void render_labels(glm::mat4 transform, ImVec2 origin, glm::vec2 surface_size);
	void render_camera_frustum(ImDrawList *draw_list, ImVec2 screen_origin, glm::vec2 surface_size, glm::mat4 transform);

	bool init_done = false;

    std::vector<std::pair<TTrack*, int>> highlighted_switches;

  public:
	map_panel();
	void render_contents() override;
};

void handle_map_object_click(ui_panel &parent, std::shared_ptr<map::map_object> &obj);
void handle_map_object_hover(std::shared_ptr<map::map_object> &obj);
} // namespace ui
