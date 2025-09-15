/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "uilayer.h"
#include "launcher/deferred_image.h"

class scenarioloader_ui : public ui_layer {
	int m_gradient_overlay_width;
	int m_gradient_overlay_height;
	GLuint m_gradient_overlay_tex;
	deferred_image m_loading_wheel_frames[60];
    std::vector<std::string> m_trivia;
public:
	scenarioloader_ui();
	void render_() override;
private:

	/// <summary>
	/// Loads random trivia from lang/trivia.json
	/// and returns it as a vector of strings. Also performs background selection.
	/// </summary>
	std::vector<std::string> get_random_trivia();
	void generate_gradient_tex();
	void load_wheel_frames();
};
