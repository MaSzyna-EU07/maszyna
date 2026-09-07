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
	// both bounds have to be set: ui_panel feeds them straight to imgui, and a maximum
	// left at its -1 default is smaller than the minimum, which collapses the window
	size_min = {560, 320};
	size_max = {1920, 1080};
}

std::string ui::multiplayer_lobby_panel::describe_crew(network::NetworkEntityId const Entity) const
{
	// the people on the whole train, wherever in it they happen to be sitting
	std::vector<network::PeerId> crew;
	for (auto const member : network::Entities.consist_members(network::Entities.consist_of(Entity)))
	{
		for (auto const peer : network::Crews.crew_of(member))
			crew.emplace_back(peer);
	}

	if (crew.empty())
		return std::string();

	std::string description;
	for (network::PeerId const peer : crew)
	{
		if (!description.empty())
			description += ", ";

		if (peer == Global.network_peer_id)
			description += STR("you");
		else
			description += network::Peers.name_of(peer);
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

	ImGui::Text("%s: %s, %s (peer %u)", STR_C("Session"), host ? (client ? "host + client" : "host") : "client",
	            network::Peers.name_of(Global.network_peer_id).c_str(), Global.network_peer_id);

	auto const roster = network::Peers.roster();
	if (roster.size() > 1)
	{
		std::string others;
		for (auto const &entry : roster)
		{
			if (entry.id == Global.network_peer_id)
				continue;
			if (!others.empty())
				others += ", ";
			others += entry.name;
		}
		if (!others.empty())
			ImGui::TextDisabled("%s: %s", STR_C("Also here"), others.c_str());
	}
	if (!Global.SceneryFile.empty())
		ImGui::TextDisabled("%s", Global.SceneryFile.c_str());

	if (!Global.network_lobby_message.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), "%s", Global.network_lobby_message.c_str());

	ImGui::Separator();

	auto const &entries = network::Entities.entries();
	if (entries.empty())
	{
		ImGui::TextUnformatted(client ? STR_C("Waiting for the vehicle list from the server...") : STR_C("This scenario has no vehicles anybody could drive."));
		return;
	}

	network::NetworkEntityId const ownvehicle{network::Crews.vehicle_of(Global.network_peer_id)};

	float const scale = std::max(1.0f, Global.ui_scale);
	float const crewwidth = 60.0f * scale;
	float const statewidth = 150.0f * scale;
	float const actionwidth = 110.0f * scale;

	ImGui::Columns(4, "mp_vehicles", false);
	ImGui::SetColumnWidth(0, std::max(120.0f * scale, ImGui::GetWindowWidth() - crewwidth - statewidth - actionwidth - 30.0f * scale));
	ImGui::SetColumnWidth(1, crewwidth);
	ImGui::SetColumnWidth(2, statewidth);
	ImGui::SetColumnWidth(3, actionwidth);

	ImGui::TextDisabled("%s", STR_C("Train"));
	ImGui::NextColumn();
	ImGui::TextDisabled("%s", STR_C("Crew"));
	ImGui::NextColumn();
	ImGui::TextDisabled("%s", STR_C("On board"));
	ImGui::NextColumn();
	ImGui::NextColumn();
	ImGui::Separator();

	network::NetworkEntityId const owntrain{network::Entities.consist_of(ownvehicle)};

	for (auto const &entry : entries)
	{
		// one row per train. a player takes the whole set, not a single car, and moves
		// between its cars freely once they are on it
		if (!entry.consist_lead)
			continue;

		ImGui::PushID(static_cast<int>(entry.id));

		if (entry.consist_size > 1)
			ImGui::Text("%s +%u", entry.name.c_str(), (unsigned)(entry.consist_size - 1));
		else
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

		if (owntrain != network::ENTITY_NONE && entry.consist_id == owntrain)
		{
			if (ImGui::Button(STR_C("Leave")))
				Application.request_vehicle_leave(ownvehicle);
		}
		else if (entry.claimable)
		{
			// a vehicle somebody else is already working on can still be joined, that is
			// what the crew capacity is for
			if (ImGui::Button(entry.crew_count > 0 ? STR_C("Join crew") : STR_C("Enter")))
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

	ImGui::Separator();
	ImGui::TextDisabled("%s", STR_C("You can also walk up to a train and use the enter-vehicle key."));
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
