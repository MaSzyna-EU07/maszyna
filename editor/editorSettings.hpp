#pragma once

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
	};
	orthophoto_settings &orthophoto() { return m_orthophoto; }
	// PUWG 1992 northing/easting of the scenery's (0,0,0) point; false when none was stored
	bool orthophoto_origin(std::string const &Scenery, double &North, double &East) const;
	void orthophoto_origin(std::string const &Scenery, double North, double East);

private:
	movement_scheme m_movement{movement_scheme::wsad};
	orthophoto_settings m_orthophoto;
	std::map<std::string, std::pair<double, double>> m_orthophoto_origins;
};

// global editor settings instance
extern editorSettings EditorSettings;
