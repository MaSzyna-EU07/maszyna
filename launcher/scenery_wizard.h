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
#include "editor/editorGeoMap.hpp"

#include <cstdint>
#include <future>
#include <string>
#include <vector>

namespace ui
{

class scenerywizard_panel : public ui_panel
{
  public:
	scenerywizard_panel();
	~scenerywizard_panel() override;

	void render_contents() override;

  private:
	void render_form();
	void pick(glm::dvec2 const &Point);
	bool create_scenery();

	editor_map m_map;
	bool m_realistic{true};
	char m_filename[64]{};
	char m_title[128]{};
	char m_latlon[64]{};
	std::string m_status;
};

} // namespace ui
