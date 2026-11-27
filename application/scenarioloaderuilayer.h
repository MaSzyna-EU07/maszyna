/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/uilayer.h"
#include "launcher/deferred_image.h"

#include <string>
#include <string_view>

class scenarioloader_ui : public ui_layer {
	int m_gradient_overlay_width;
	int m_gradient_overlay_height;
	std::uint64_t m_gradient_overlay_tex; // imgui_renderer image
	deferred_image m_loading_wheel_frames[60];
    std::vector<std::string> m_trivia;
	std::string m_notice; // shown at the top of the screen, clear of the trivia and the texts of the backgrounds at the bottom
public:
	scenarioloader_ui();
	void render_() override;
	// a notice for the player at the top of the screen, such as that the graphics are being prepared; empty hides it
	void set_notice( std::string_view const Text ) { m_notice = Text; }
private:
	void render_notice( ImDrawList *Drawlist, float Scale, float Fontscale ) const;

	/// <summary>
	/// Loads random trivia from lang/trivia.json
	/// and returns it as a vector of strings. Also performs background selection.
	/// </summary>
	std::vector<std::string> get_random_trivia();
	void generate_gradient_tex();
	void load_wheel_frames();

	/// <summary>
	/// Scenery name for eligable trivias
	/// </summary>
	std::string sceneryName;
};
