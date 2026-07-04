#pragma once

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
	void movement(const movement_scheme scheme) { m_movement = scheme; }

private:
	movement_scheme m_movement{movement_scheme::wsad};
};

// global editor settings instance
extern editorSettings EditorSettings;
