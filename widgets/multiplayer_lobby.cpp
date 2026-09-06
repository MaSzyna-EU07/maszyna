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
#include "network/session.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"

ui::multiplayer_lobby_panel::multiplayer_lobby_panel()
    : ui_panel(STR_C("Multiplayer lobby"), false)
{
	size_min = {520, 280};
}

std::string ui::multiplayer_lobby_panel::describe_crew(network::NetworkEntityId const Entity) const
{
	auto const crew = network::Crews.crew_of(Entity);
	if (crew.empty())
		return std::string();

	std::string description;
	for (network::PeerId const peer : crew)
	{
		if (!description.empty())
			description += ", ";

		if (peer == Global.network_peer_id)
			description += STR("you");
		else if (peer == network::PEER_HOST)
			description += STR("host");
		else
			description += STR("player") + " " + std::to_string(peer);
	}

	return description;
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

	if (!Global.network_lobby_message.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), "%s", Global.network_lobby_message.c_str());

	ImGui::Separator();

	auto const &entries = network::Entities.entries();
	if (entries.empty())
	{
		ImGui::TextUnformatted(STR_C("Waiting for the vehicle list from the server..."));
		return;
	}

	network::NetworkEntityId const ownvehicle{network::Crews.vehicle_of(Global.network_peer_id)};

	ImGui::Columns(4, "mp_vehicles", false);
	ImGui::SetColumnWidth(1, 70.0f * Global.ui_scale);
	ImGui::SetColumnWidth(2, 200.0f * Global.ui_scale);
	ImGui::SetColumnWidth(3, 130.0f * Global.ui_scale);

	ImGui::TextDisabled("%s", STR_C("Vehicle"));
	ImGui::NextColumn();
	ImGui::TextDisabled("%s", STR_C("Crew"));
	ImGui::NextColumn();
	ImGui::TextDisabled("%s", STR_C("On board"));
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

		std::string const crew{describe_crew(entry.id)};
		if (!crew.empty())
			ImGui::TextUnformatted(crew.c_str());
		else
			ImGui::TextDisabled("%s", entry.ai_active ? STR_C("AI") : STR_C("free"));
		ImGui::NextColumn();

		if (entry.id == ownvehicle)
		{
			if (ImGui::Button(STR_C("Leave"), ImVec2(-1, 0)))
				Application.request_vehicle_leave(entry.id);
		}
		else if (entry.claimable)
		{
			// a vehicle somebody else is already working on can still be joined, that is
			// what the crew capacity is for
			if (ImGui::Button(entry.crew_count > 0 ? STR_C("Join crew") : STR_C("Enter"), ImVec2(-1, 0)))
				claim(entry.id, ownvehicle);
		}
		else
		{
			ImGui::TextDisabled("%s", STR_C("full"));
		}
		ImGui::NextColumn();

		ImGui::PopID();
	}

	ImGui::Columns(1);
}

void ui::multiplayer_lobby_panel::claim(network::NetworkEntityId const Entity, network::NetworkEntityId const Current)
{
	if (Current != network::ENTITY_NONE)
	{
		// one person cannot work two vehicles at once
		Application.request_vehicle_leave(Current);
	}

	Application.request_vehicle_claim(Entity);
}
