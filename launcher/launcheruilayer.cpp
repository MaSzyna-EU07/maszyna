#include "stdafx.h"
#include "launcher/launcheruilayer.h"
#include "application/application.h"
#include "utilities/translation.h"

launcher_ui::launcher_ui()
	: m_scenery_scanner(m_vehicles_bank),
	  m_scenerylist_panel(m_scenery_scanner)
{
	m_vehicles_bank.scan_textures();
	m_scenery_scanner.scan();

	add_external_panel(&m_scenerylist_panel);
	add_external_panel(&m_keymapper_panel);
	add_external_panel(&m_vehiclepicker_panel);

	open_panel(&m_scenerylist_panel);
	m_suppress_menu = true;
}

bool launcher_ui::on_key(const int Key, const int Action)
{
	if (m_scenerylist_panel.on_key(Key, Action))
		return true;

	if (m_keymapper_panel.key(Key))
		return true;

	return ui_layer::on_key(Key, Action);
}

void launcher_ui::on_window_resize(int w, int h)
{
	open_panel(m_current_panel);
}

void launcher_ui::render_()
{
	const float scale = Global.ui_scale;

	/*
	 * 2026 Launcher UI
	 *
	 * Functionality intentionally remains unchanged:
	 * - Scenario list
	 * - Vehicle list
	 * - Keymapper
	 * - Quit
	 *
	 * Only presentation and navigation styling are modernized.
	 */

	const float topbar_height = 68.0f * scale;
	const float horizontal_padding = 18.0f * scale;
	const float button_height = 42.0f * scale;
	const float button_width = 154.0f * scale;
	const float button_spacing = 8.0f * scale;

	ImGuiWindowFlags flags =
		ImGuiWindowFlags_NoTitleBar |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoScrollbar |
		ImGuiWindowFlags_NoCollapse |
		ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoSavedSettings;

	/*
	 * ---------------------------------------------------------
	 * Launcher header
	 * ---------------------------------------------------------
	 */

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(
		ImVec2(
			Global.window_size.x,
			topbar_height
		)
	);

	/*
	 * Header styling
	 */
	ImGui::PushStyleVar(
		ImGuiStyleVar_WindowPadding,
		ImVec2(
			horizontal_padding,
			13.0f * scale
		)
	);

	ImGui::PushStyleVar(
		ImGuiStyleVar_ItemSpacing,
		ImVec2(
			button_spacing,
			0.0f
		)
	);

	ImGui::PushStyleVar(
		ImGuiStyleVar_FrameRounding,
		8.0f * scale
	);

	ImGui::PushStyleVar(
		ImGuiStyleVar_FramePadding,
		ImVec2(
			16.0f * scale,
			8.0f * scale
		)
	);

	if (ImGui::Begin(STR_C("Main menu"), nullptr, flags))
	{
		/*
		 * -----------------------------------------------------
		 * Application title
		 * -----------------------------------------------------
		 */

		ImGui::AlignTextToFramePadding();

		ImGui::TextUnformatted(
			STR_C("Launcher")
		);

		ImGui::SameLine();

		ImGui::TextDisabled(
			STR_C("Choose what you want to do")
		);

		ImGui::SameLine();

		/*
		 * Push navigation buttons to the right of the title.
		 */
		const float available_width =
			ImGui::GetContentRegionAvail().x;

		const float navigation_width =
			(button_width * 4.0f) +
			(button_spacing * 3.0f);

		if (available_width > navigation_width)
		{
			ImGui::SameLine(
				ImGui::GetCursorPosX() +
				available_width -
				navigation_width
			);
		}
		else
		{
			ImGui::SameLine();
		}

		/*
		 * -----------------------------------------------------
		 * Scenario list
		 * -----------------------------------------------------
		 */

		const bool scenario_active =
			(m_current_panel == &m_scenerylist_panel);

		if (scenario_active)
			ImGui::PushStyleColor(
				ImGuiCol_Button,
				ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered]
			);

		if (ImGui::Button(
			STR_C("Scenarios"),
			ImVec2(button_width, button_height)
		))
		{
			open_panel(&m_scenerylist_panel);
		}

		if (scenario_active)
			ImGui::PopStyleColor();

		ImGui::SameLine();

		/*
		 * -----------------------------------------------------
		 * Vehicle list
		 * -----------------------------------------------------
		 */

		const bool vehicle_active =
			(m_current_panel == &m_vehiclepicker_panel);

		if (vehicle_active)
			ImGui::PushStyleColor(
				ImGuiCol_Button,
				ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered]
			);

		if (ImGui::Button(
			STR_C("Vehicles"),
			ImVec2(button_width, button_height)
		))
		{
			open_panel(&m_vehiclepicker_panel);
		}

		if (vehicle_active)
			ImGui::PopStyleColor();

		ImGui::SameLine();

		/*
		 * -----------------------------------------------------
		 * Keymapper
		 * -----------------------------------------------------
		 */

		const bool keymapper_active =
			(m_current_panel == &m_keymapper_panel);

		if (keymapper_active)
			ImGui::PushStyleColor(
				ImGuiCol_Button,
				ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered]
			);

		if (ImGui::Button(
			STR_C("Controls"),
			ImVec2(button_width, button_height)
		))
		{
			open_panel(&m_keymapper_panel);
		}

		if (keymapper_active)
			ImGui::PopStyleColor();

		ImGui::SameLine();

		/*
		 * -----------------------------------------------------
		 * Quit
		 * -----------------------------------------------------
		 */

		if (ImGui::Button(
			STR_C("Quit"),
			ImVec2(button_width, button_height)
		))
		{
			Application.queue_quit(false);
		}
	}

	ImGui::End();

	ImGui::PopStyleVar(4);
}

void launcher_ui::close_panels()
{
	m_scenerylist_panel.is_open = false;
	m_vehiclepicker_panel.is_open = false;
	m_keymapper_panel.is_open = false;
}

void launcher_ui::open_panel(ui_panel* panel)
{
	if (panel == nullptr)
		return;

	close_panels();

	const float topbar_height =
		68.0f * Global.ui_scale;

	panel->pos = glm::vec2(
		0.0f,
		topbar_height
	);

	panel->size = glm::vec2(
		Global.window_size.x,
		Global.window_size.y - topbar_height
	);

	panel->no_title_bar = true;
	panel->is_open = true;

	m_current_panel = panel;
}
