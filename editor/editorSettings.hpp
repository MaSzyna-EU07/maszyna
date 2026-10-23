#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>

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

	// geoportal orthophoto layer (editor_orthophoto); the origin is kept per scenery
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
	};
	orthophoto_settings &orthophoto() { return m_orthophoto; }
	// PUWG 1992 northing/easting of the scenery's (0,0,0) point; false when none was stored
	bool orthophoto_origin(std::string const &Scenery, double &North, double &East) const;
	void orthophoto_origin(std::string const &Scenery, double North, double East);

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
	bool track_tools_more() const { return m_track_tools_more; }
	void track_tools_more(bool Show) { m_track_tools_more = Show; }
	// the ballast of the railway drawn in the top view
	int top_tracks() const { return m_top_tracks; }
	void top_tracks(int const Tracks) { m_top_tracks = Tracks; }
	std::uint32_t top_rails() const { return m_top_rails; }
	void top_rails(std::uint32_t const Colour) { m_top_rails = Colour; }
	// how the new tracks are laid: asked as the laying starts, or the choice kept
	enum class track_norms
	{
		ask,
		free_laying,
		plk
	};
	track_norms norms() const { return m_norms; }
	void norms(track_norms Norms) { m_norms = Norms; }

private:
	movement_scheme m_movement{movement_scheme::wsad};
	orthophoto_settings m_orthophoto;
	std::map<std::string, std::pair<double, double>> m_orthophoto_origins;
	double m_platform_edge{1.725};
	double m_signal_offset{2.25};
	bool m_nodebank_previews{true};
	int m_nodebank_card{96};
	int m_palette{0};
	bool m_first_steps{true};
	bool m_terrain_offer{true};
	bool m_track_tools_more{false};
	int m_top_tracks{0}; // the railway in the top view: whole, without the ballast, the rails alone
	std::uint32_t m_top_rails{0xffff6e28}; // colour of the rails alone, as ImGui keeps it: ABGR
	track_norms m_norms{track_norms::ask};
};

// global editor settings instance
extern editorSettings EditorSettings;
