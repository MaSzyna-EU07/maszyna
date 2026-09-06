/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "widgets/multiplayer_lobby.h"

#include "application/application.h"
#include "network/entities.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "utilities/translation.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"

ui::multiplayer_lobby_panel::multiplayer_lobby_panel()
    : ui_panel(STR_C("Multiplayer lobby"), false)
{
	size_min = {480, 260};
}

void ui::multiplayer_lobby_panel::enter_vehicle(std::string const &Name)
{
	TDynamicObject *vehicle = simulation::Vehicles.find(Name);
	if (vehicle == nullptr)
		return;

	std::string payload{Name};
	// the request travels as an ordinary simulation command, so every peer builds the cab
	m_relay.post(user_command::entervehicle, 0.0, simulation::Train ? simulation::Train->id() : 0, GLFW_PRESS, 0, vehicle->GetPosition(), &payload);
	// ...and the driver mode moves our own camera in once the cab shows up locally
	Global.network_pending_vehicle = Name;
	is_open = false;
}

void ui::multiplayer_lobby_panel::render_contents()
{
	bool const host{Application.is_server()};
	bool const client{Application.is_client()};

	if (!host && !client)
	{
		ImGui::TextUnformatted(STR_C("This is not a multiplayer session."));
		return;
	}

	ImGui::Text("%s: %s, peer %u", STR_C("Session"), host ? (client ? "host + client" : "host") : "client", Global.network_peer_id);
	ImGui::TextUnformatted(Global.SceneryFile.c_str());
	ImGui::Separator();

	auto const &entries = network::Entities.entries();
	if (entries.empty())
	{
		ImGui::TextUnformatted(STR_C("Waiting for the vehicle list from the server..."));
		return;
	}

	std::string const currentvehicle{simulation::Train != nullptr && simulation::Train->Dynamic() != nullptr ? simulation::Train->Dynamic()->name() : std::string()};

	if (!FreeFlyModeFlag)
	{
		ImGui::TextUnformatted(STR_C("Leave the cab to change vehicles."));
		ImGui::Separator();
	}

	ImGui::Columns(4, "mp_vehicles", false);
	ImGui::SetColumnWidth(1, 80.0f * Global.ui_scale);
	ImGui::SetColumnWidth(2, 90.0f * Global.ui_scale);
	ImGui::SetColumnWidth(3, 130.0f * Global.ui_scale);

	ImGui::TextDisabled("%s", STR_C("Vehicle"));
	ImGui::NextColumn();
	ImGui::TextDisabled("%s", STR_C("Crew"));
	ImGui::NextColumn();
	ImGui::TextDisabled("%s", STR_C("Driver"));
	ImGui::NextColumn();
	ImGui::NextColumn();
	ImGui::Separator();

	for (auto const &entry : entries)
	{
		ImGui::PushID(static_cast<int>(entry.id));

		ImGui::TextUnformatted(entry.name.c_str());
		ImGui::NextColumn();

		ImGui::Text("%u/%u", (unsigned)entry.crew_count, (unsigned)entry.crew_capacity);
		ImGui::NextColumn();

		ImGui::TextUnformatted(entry.crew_count > 0 ? STR_C("player") : entry.ai_active ? STR_C("AI") : STR_C("free"));
		ImGui::NextColumn();

		if (entry.name == currentvehicle)
		{
			ImGui::TextUnformatted(STR_C("you"));
		}
		else if (entry.claimable && FreeFlyModeFlag && network::Entities.resolve(entry.id) != nullptr)
		{
			if (ImGui::Button(STR_C("Enter"), ImVec2(-1, 0)))
				enter_vehicle(entry.name);
		}
		else
		{
			ImGui::TextDisabled("%s", entry.claimable ? "-" : STR_C("occupied"));
		}
		ImGui::NextColumn();

		ImGui::PopID();
	}

	ImGui::Columns(1);
}
