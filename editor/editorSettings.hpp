#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

class editorSettings
{
public:
	// camera movement key scheme in the editor
	enum class movement_scheme
	{
		wsad,   // new default: W/S/A/D + E/Q
		legacy  // old scheme: arrows + Page Up/Down
	};

	editorSettings() = default;

	bool load();
	bool save();

	movement_scheme movement() const { return m_movement; }
	void movement(movement_scheme scheme) { m_movement = scheme; }

	// geoportal orthophoto layer (editor_orthophoto), how it's shown and whether it's on; the origin is kept per scenery, and the
	// scenery keeps it too
	struct orthophoto_settings
	{
		int radius{2};
		float height{0.0f};
		float opacity{0.6f};
		int year{0};
		bool hires{false};
		bool drape{true};
		float lift{0.05f};
		bool in_scene{false};
		// turned on or off by the user, for every scenery; none chosen yet: as the scenery keeps it, or the terrain model turns it on
		std::optional<bool> on;
	};
	orthophoto_settings &orthophoto() { return m_orthophoto; }
	// PUWG 1992 northing/easting of the scenery's (0,0,0) point; false when none was stored
	bool orthophoto_origin(std::string const &Scenery, double &North, double &East) const;
	void orthophoto_origin(std::string const &Scenery, double North, double East);
	// the WMS layers the user added under the scenery, over the orthophoto, the top one first: the address of the service, the
	// layers asked of it, its title and terms as its capabilities gave them. Kept with the editor, not with the scenery
	struct wms_layer
	{
		std::string url;
		std::string layers;
		std::string title;
		std::string terms;
		float opacity{0.8f};
		bool visible{true};
	};
	std::vector<wms_layer> &wms_layers() { return m_wms_layers; }

	double platform_edge() const { return m_platform_edge; }
	void platform_edge(double Distance) { m_platform_edge = Distance; }
	double signal_offset() const { return m_signal_offset; }
	void signal_offset(double Distance) { m_signal_offset = Distance; }

	// node bank: cards with the previews of the models or a list, and the size of the cards in pixels
	bool nodebank_previews() const { return m_nodebank_previews; }
	void nodebank_previews(bool Previews) { m_nodebank_previews = Previews; }
	int nodebank_card() const { return m_nodebank_card; }
	void nodebank_card(int Size) { m_nodebank_card = Size; }

	// colours of the editor: 0 green, as the rest of the simulator; 1 amber
	int palette() const { return m_palette; }
	void palette(int Palette) { m_palette = Palette; }

	bool first_steps() const { return m_first_steps; }
	void first_steps(bool Show) { m_first_steps = Show; }
	// a scenery begun with no terrain is offered the terrain model as the first track is laid
	bool terrain_offer() const { return m_terrain_offer; }
	void terrain_offer(bool Offer) { m_terrain_offer = Offer; }
	// a track laid and hidden in the ground is told of, with the way to fit the ground to it
	bool terrain_buried_ask() const { return m_terrain_buried_ask; }
	void terrain_buried_ask(bool Ask) { m_terrain_buried_ask = Ask; }
	// the terrain of the terrain model made under the tracks as they're laid, around the cursor from the first point placed
	bool terrain_while_laying() const { return m_terrain_while_laying; }
	void terrain_while_laying(bool const Make) { m_terrain_while_laying = Make; }
	// the terrain model in the terrain panel: the band along the tracks it's loaded in, m; the chunks along a side of the square
	// made around the camera; the formation and the slopes made along the tracks; the terrain following the tracks edited
	struct terrain_model_settings
	{
		double band{300.0};
		int around{4};
		bool shape{true};
		bool follow{true};
	};
	terrain_model_settings &terrain_model() { return m_terrain_model; }
	// the height offset of the terrain model to the tracks of the scenery, and whether it was fitted to them; false when none was stored
	bool terrain_model_offset(std::string const &Scenery, double &Offset, bool &Fitted) const;
	void terrain_model_offset(std::string const &Scenery, double Offset, bool Fitted);
	bool track_tools_more() const { return m_track_tools_more; }
	void track_tools_more(bool Show) { m_track_tools_more = Show; }
	// the ballast of the railway drawn in the top view
	int top_tracks() const { return m_top_tracks; }
	void top_tracks(int const Tracks) { m_top_tracks = Tracks; }
	std::uint32_t top_rails() const { return m_top_rails; }
	void top_rails(std::uint32_t const Colour) { m_top_rails = Colour; }
	// the radii the laying from point to point eases the bends of the straights with, at the peak of the two transition curves
	double corner_low() const { return m_corner_low; }
	double corner_high() const { return m_corner_high; }
	void corners(double const Low, double const High)
	{
		m_corner_low = Low;
		m_corner_high = High;
	}
	// the heights of the tracks laid on the ground: fitted to it as the tracks go onto the terrain, the points clicked on it with
	// even grades between them, level
	enum class lay_heights
	{
		fitted,
		points,
		level
	};
	lay_heights heights() const { return m_heights; }
	void heights(lay_heights Heights) { m_heights = Heights; }

private:
	movement_scheme m_movement{movement_scheme::wsad};
	orthophoto_settings m_orthophoto;
	std::map<std::string, std::pair<double, double>> m_orthophoto_origins;
	std::vector<wms_layer> m_wms_layers;
	double m_platform_edge{1.725};
	double m_signal_offset{2.25};
	bool m_nodebank_previews{true};
	int m_nodebank_card{96};
	int m_palette{0};
	bool m_first_steps{true};
	bool m_terrain_offer{true};
	bool m_terrain_buried_ask{true};
	bool m_track_tools_more{false};
	bool m_terrain_while_laying{true};
	terrain_model_settings m_terrain_model;
	std::map<std::string, std::pair<double, bool>> m_terrain_model_offsets;
	int m_top_tracks{0}; // the railway in the top view: whole, without the ballast, the rails alone
	std::uint32_t m_top_rails{0xffff6e28}; // colour of the rails alone, as ImGui keeps it: ABGR
	double m_corner_low{3000.0};
	double m_corner_high{6000.0};
	lay_heights m_heights{lay_heights::points};
};

// global editor settings instance
extern editorSettings EditorSettings;
