module;
#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include "imgui/imgui.h"
#include <string>

module eu07.application.uilayer;

ui::popup::popup(ui_panel &panel) : m_parent(panel) {}

ui::popup::~popup() {}

bool ui::popup::render()
{
	if (!m_id.size())
	{
		m_id = "popup:" + std::to_string(id++);
		ImGui::OpenPopup(m_id.c_str());
	}

	if (!ImGui::BeginPopup(m_id.c_str()))
		return true;

	render_content();

	ImGui::EndPopup();

	return false;
}

int ui::popup::id = 0;
