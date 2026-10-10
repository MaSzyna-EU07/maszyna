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
	struct map_result
	{
		glm::dvec2 low{0.0};
		glm::dvec2 high{0.0};
		int width{0};
		int height{0};
		std::vector<std::uint8_t> pixels;
		std::string error;
	};

	void render_map(glm::vec2 const &Size);
	void render_form();
	void request_map(glm::vec2 const &Size);
	void upload_map();
	void pick(glm::dvec2 const &Point);
	bool create_scenery();

	glm::dvec2 m_centre{515000.0, 460000.0};
	double m_scale{1000.0};
	bool m_fitted{false};
	double m_changed{0.0};
	bool m_dirty{true};
	glm::vec2 m_pressed{0.f};
	bool m_dragging{false};

	std::future<map_result> m_request;
	unsigned int m_texture{0};
	glm::dvec2 m_image_low{0.0};
	glm::dvec2 m_image_high{0.0};
	std::string m_map_error;

	bool m_picked{false};
	glm::dvec2 m_point{0.0};
	char m_filename[64]{};
	char m_title[128]{};
	char m_latlon[64]{};
	std::string m_status;
};

} // namespace ui
