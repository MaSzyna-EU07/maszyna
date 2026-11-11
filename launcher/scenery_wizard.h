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
	// how the range of the aerial photos is given: a square around the centre, or an area of the user's clicked on the map
	enum class range_shape
	{
		square,
		drawn
	};

	void render_form();
	void render_place();
	void render_range();
	void pick(glm::dvec2 const &Point);
	// the range as it's written to the scenery, PUWG (east, north); empty while there's none
	std::vector<glm::dvec2> range() const;
	// why the scenery can't be created yet, as the user reads it; empty when it can
	std::string missing(bool const Named, bool const Exists) const;
	void show_range();
	bool create_scenery();

	editor_map m_map;
	bool m_realistic{true};
	char m_filename[64]{};
	char m_title[128]{};
	char m_place[64]{};
	std::string m_status;
	range_shape m_shape{range_shape::square};
	int m_side{2}; // km, the side of the square
	std::vector<glm::dvec2> m_drawn; // the area of the user's
	bool m_drawing{false};           // its corners are being clicked
	std::vector<glm::dvec2> m_before; // the area there was before the drawing began, back with Cancel
};

} // namespace ui
