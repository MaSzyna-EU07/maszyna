#pragma once

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "gl/shader.h"
#include "model/Texture.h"
#include "application/uilayer.h"
#include "widgets/map_objects.h"
#include "widgets/popup.h"
#include "gl/framebuffer.h"
#include "rendering/frustum.h"
#ifdef WITH_OPENGL_MODERN
#include "rendering/opengl46/opengl46renderer.h"
#endif

namespace ui
{

// what can be done to the objects of the map: the same for the windows of imgui and the map of RmlUi
namespace map_actions
{

// what the object is: Signal, Switch, Level crossing, Obstacle, Vehicle or Object, to be translated
std::string kind(map::map_object const &object);

// a lamp of a head of a signal
struct lamp
{
	int index; // of the light of the model
	glm::vec3 colour; // as it shines now
	glm::vec3 lit; // as it shines when on
	float level; // 0: off, 1: on, 2: blinking
};
// the lamps a head of a signal has, which it can light
std::vector<lamp> lamps(TAnimModel *model);
// the lamp set to its next level: off, on, blinking
void cycle_lamp(TAnimModel *model, lamp const &Lamp);

// a choice of an object: its name as shown, the event which makes it
struct choice
{
	std::string name;
	basic_event *event;
};
// the aspects a signal is set to by its events
std::vector<choice> aspects(map::semaphore const &semaphore);
// the two positions of a switch or of a level crossing, translated
std::vector<choice> positions(map::launcher const &launcher);
// a position of a switch of more tracks, and the tracks it sets to show it
struct switch_choice
{
	choice position;
	std::vector<std::pair<TTrack *, int>> preview;
};
std::vector<switch_choice> positions(map::track_switch const &track_switch);
// the event run
void run(basic_event const &event);

// the obstacles which can be put on a track: their names and what they are (obstaclebank.txt)
std::vector<std::pair<std::string, std::string>> obstacle_bank();
void insert_obstacle(glm::dvec3 const &position, std::pair<std::string, std::string> const &obstacle);
void remove_obstacle(std::shared_ptr<map::obstacle> const &obstacle);
// the light signal a vehicle shows: none (0), Pc6 or A1
void set_vehicle_signal(std::string const &vehicle, int signal);

} // namespace map_actions

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

	std::unique_ptr<gl::program> m_track_shader;
	std::unique_ptr<gl::program> m_poi_shader;
	std::unique_ptr<gl::framebuffer> m_msaa_fb;
	std::unique_ptr<gl::renderbuffer> m_msaa_rb;
	texture_handle m_icon_atlas;

	std::unique_ptr<gl::framebuffer> m_fb;
	std::unique_ptr<opengl_texture> m_tex;

	std::unique_ptr<gl::ubo> scene_ubo;
	gl::scene_ubs scene_ubs;

	std::vector<gfx::geometrybank_handle> m_section_handles;
	map_colored_paths m_colored_paths;

    // the side of the square the map is drawn into; it grows to the size of the screen for the map over the whole of it
    int fb_size = 1024;

	glm::vec2 translate;
	float zoom = 1.0f / 1000.0f;
	enum { MODE_MANUAL = 0, MODE_CAMERA, MODE_VEHICLE } mode = MODE_MANUAL;

  public:
	// a label of a train, at Position in pixels of the surface the map is drawn on
	struct label
	{
		std::string text;
		glm::vec2 position;
	};
	// the image of the map drawn for a surface, and the corners of it to show
	struct image
	{
		ImTextureID texture;
		ImVec2 uv0;
		ImVec2 uv1;
	};
	// how the map is drawn; the defaults are the look of the window of the map
	struct style
	{
		std::optional<glm::vec3> background; // the image cleared to it, else to what the renderer clears with
		glm::vec3 track{0.5f, 0.5f, 0.5f};
		glm::vec3 future{0.7f, 0.7f, 0.0f}; // the route of the train ahead
		glm::vec3 switches{0.0f, 1.0f, 0.0f}; // the tracks the switches are set to
		glm::vec3 occupied{1.0f, 0.0f, 0.0f};
		glm::vec3 highlighted{0.3f, 0.3f, 1.0f}; // the tracks of the switch chosen
		int thickness{0}; // pixels the lines are drawn wider by on each side
		bool icons{true}; // the signals, the switches and the other objects
	};
	// a train: the vehicle its driver is in
	struct train
	{
		TDynamicObject const *vehicle;
		std::string name; // of the train, or of the vehicle when the train has none
		glm::vec2 position;
		bool shown; // on the surface
		float heading; // degrees on the surface, clockwise from up
	};
	struct segment
	{
		glm::vec2 from;
		glm::vec2 to;
	};
	// a head of a signal: its name and the colours its lamps show
	struct signal_head
	{
		std::string name;
		std::vector<glm::vec3> lamps;
	};
	struct object_info
	{
		std::string kind; // Signal, Switch, Level crossing, Obstacle, Vehicle or Object, to be translated
		std::string name;
		std::vector<signal_head> heads;
	};

  private:
	// how the world is seen on the surface: the whole transform, and the part of it which keeps the aspect of the surface
	struct view
	{
		glm::mat4 transform;
		glm::mat4 aspect;
	};

	float get_vehicle_rotation();
	float get_vehicle_rotation(TDynamicObject const *vehicle);
	// the vehicle the map follows: the one chosen on the map of RmlUi, else the one driven
	TDynamicObject const *followed_vehicle() const;
	// the paths drawn in the colour, wider by Thickness pixels on each side
	void draw_paths(std::vector<gfx::geometrybank_handle> &paths, glm::vec3 const &colour, glm::mat4 const &transform, glm::vec2 surface_size, int thickness);
	// where a point of the world is on the surface of the last draw, in pixels
	glm::vec2 to_surface(glm::dvec3 const &world) const;
	// the image the map is drawn into, Size pixels square. false: the framebuffers couldn't be made
	bool make_targets(int Size);
	// the image as big as the surface needs. false: it couldn't be made, the map can't be drawn
	bool fit_targets(glm::vec2 surface_size);
	// the view of the surface; following the camera or the vehicle moves the map to them
	view view_of(glm::vec2 surface_size);
	static glm::vec3 world_at(view const &View, glm::vec2 ndc_pos);
	// the window of what was clicked on the map at the point: the object, a choice of the objects, or an obstacle on the track
	void open_objects(glm::vec3 const &world_pos, map::sorted_object_list &&objects);
	void render_map_texture(glm::mat4 transform, glm::vec2 surface_size);
	void render_map_texture(glm::mat4 transform, glm::vec2 surface_size, style const &Style);
	std::vector<label> collect_labels(glm::mat4 const &transform, glm::vec2 surface_size) const;
	void render_labels(glm::mat4 transform, ImVec2 origin, glm::vec2 surface_size);

	bool init_done = false;
	// the view of the map shown by the hud of RmlUi, of its last draw
	view m_view;
	glm::vec2 m_surface{0.0f, 0.0f};
	// the name of the vehicle followed, chosen on the map of RmlUi; empty: the one driven
	std::string m_followed;

    std::vector<std::pair<TTrack*, int>> highlighted_switches;

  public:
	map_panel();
	void render_contents() override;

	// the map shown by the hud of RmlUi, on a surface of its own rather than in a window of imgui. the positions are in
	// pixels of the surface, from its top left corner
	bool available() const { return init_done; }
	// the map drawn for a surface of the size; none when it can't be drawn (not the opengl renderer)
	std::optional<image> draw(glm::vec2 surface_size, style const &Style);
	std::vector<label> labels() const;
	// the trains, where they are on the surface of the last draw
	std::vector<train> trains() const;
	// the scale of the last draw
	float pixels_per_metre() const;
	// the lines of a grid of the world of Step metres seen on the surface
	std::vector<segment> grid(float step) const;
	// the object under the point, none if there's none
	std::optional<object_info> object_at(glm::vec2 surface_pos) const;
	// the objects at the point, the nearest first
	map::sorted_object_list objects_at(glm::vec2 surface_pos) const;
	// the signal of the memory cell at the place (where the event of the signal reads it from), none if there's none
	static std::shared_ptr<map::semaphore> semaphore_at(glm::dvec3 const &location);
	// the point of the track nearest to the point, if it's close enough to put an obstacle on
	std::optional<glm::dvec3> track_point_at(glm::vec2 surface_pos) const;
	// the tracks a switch would set, shown in their colour; empty: none
	void preview_switch(std::vector<std::pair<TTrack *, int>> const &tracks) { highlighted_switches = tracks; }
	// a vehicle drawn to scale: its middle on the surface, the angle of its axis (degrees, clockwise from the right), its length
	// and width in pixels
	struct vehicle_mark
	{
		glm::vec2 position;
		float angle;
		float length;
		float width;
		bool powered; // with engines of its own
		bool own; // of the consist driven
	};
	// the vehicles on the surface; none while they'd be shorter than Shortest pixels
	std::vector<vehicle_mark> vehicles(float shortest) const;
	// the map follows the vehicle of the name (empty: the one driven)
	void follow(std::string const &vehicle);
	std::string const &followed() const { return m_followed; }
	// Steps: powers of 2, above 0 closer; the point under the cursor stays
	void zoom_by(float steps, glm::vec2 surface_pos);
	void drag_by(glm::vec2 delta);
	// Button: 1 the object (or an obstacle) at the point, 2 the free camera there
	void click(int button, glm::vec2 surface_pos);
	// 0: panned by hand, 1: following the camera, 2: following the vehicle (the one driven, unless another is followed)
	int view_mode() const { return mode; }
	void set_view_mode(int Mode);
	// the windows of the objects clicked on the map shown by the hud, in a frame of imgui
	void render_popups();
};

void handle_map_object_click(ui_panel &parent, std::shared_ptr<map::map_object> &obj);
void handle_map_object_hover(std::shared_ptr<map::map_object> &obj);
} // namespace ui
