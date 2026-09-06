/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "network/session.h"

#include "input/command.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"

namespace network
{

crew_registry Crews;

std::string describe(claim_result Result)
{
	switch (Result)
	{
	case claim_result::granted:
		return "granted";
	case claim_result::already_member:
		return "already a crew member";
	case claim_result::unknown_vehicle:
		return "no such vehicle in this session";
	case claim_result::vehicle_missing:
		return "the vehicle is not present in the scenario";
	case claim_result::crew_full:
		return "the crew of this vehicle is full";
	}
	return "unknown";
}

std::string describe(command_verdict Verdict)
{
	switch (Verdict)
	{
	case command_verdict::accepted:
		return "accepted";
	case command_verdict::not_in_crew:
		return "not a member of the crew of that vehicle";
	case command_verdict::privileged:
		return "reserved for the session host";
	case command_verdict::unsupported_target:
		return "command target not allowed over the network";
	}
	return "unknown";
}

command_verdict validate_command(PeerId Peer, user_command Command, uint32_t Recipient)
{
	if (Peer == PEER_HOST)
	{
		// the host runs the session and holds every right in it
		return command_verdict::accepted;
	}

	auto const target = (command_target)(Recipient & ~0xffff);

	if (target == command_target::vehicle)
	{
		// a player works the controls of the vehicle they are on, and of no other
		return Crews.is_member(Peer, (NetworkEntityId)(Recipient & 0xffff)) ? command_verdict::accepted : command_verdict::not_in_crew;
	}

	if (target == command_target::simulation)
	{
		// setweather, setdatetime, spawntrainset, destroytrainset, queueevent, the pause
		// and the debug commands all reach the whole world, so they stay with the authority.
		// TODO: open a subset of them once dispatcher and admin roles exist
		(void)Command;
		return command_verdict::privileged;
	}

	return command_verdict::unsupported_target;
}

claim_result claim_by_name(PeerId Peer, std::string const &Vehicle, NetworkEntityId &Entity)
{
	Entity = Entities.id_of(Vehicle);
	if (Entity == ENTITY_NONE)
		return claim_result::unknown_vehicle;

	auto const previous = Crews.vehicle_of(Peer);
	auto const result = Crews.claim(Peer, Entity);

	if ((result == claim_result::granted) && (previous != ENTITY_NONE) && (previous != Entity))
	{
		// nobody works two vehicles at once
		Crews.leave(Peer, previous);
	}

	return result;
}

void filter_commands(PeerId Peer, command_queue::commands_map &Commands)
{
	for (auto it = Commands.begin(); it != Commands.end();)
	{
		auto &sequence = it->second;

		for (auto command = sequence.begin(); command != sequence.end();)
		{
			if (command->source == PEER_NONE)
			{
				// posted by the authority itself rather than by a person; it is already
				// the decision, not a request for one
				++command;
				continue;
			}

			if (command->command == user_command::entervehicle)
			{
				// the local player walked into a cab. that is a crew request; the cab
				// itself is built by the authority once the seat is granted
				NetworkEntityId entity{ENTITY_NONE};
				auto const result = claim_by_name(Peer, command->payload, entity);
				if (result != claim_result::granted && result != claim_result::already_member)
					WriteLog("net: cannot take " + command->payload + ": " + describe(result), logtype::net);

				command = sequence.erase(command);
				continue;
			}

			if (validate_command(Peer, command->command, it->first) != command_verdict::accepted)
			{
				command = sequence.erase(command);
				continue;
			}

			command->source = Peer;
			++command;
		}

		if (sequence.empty())
			it = Commands.erase(it);
		else
			++it;
	}
}

vehicle_crew &crew_registry::entry(NetworkEntityId Id)
{
	auto lookup = m_vehicles.find(Id);
	if (lookup == m_vehicles.end())
	{
		vehicle_crew fresh;
		fresh.entity_id = Id;
		lookup = m_vehicles.emplace(Id, fresh).first;
	}
	return lookup->second;
}

claim_result crew_registry::claim(PeerId Peer, NetworkEntityId Id)
{
	vehicle_entry const *published = Entities.find(Id);
	if (published == nullptr)
		return claim_result::unknown_vehicle;

	if (Entities.resolve(Id) == nullptr)
		return claim_result::vehicle_missing;

	vehicle_crew &vehicle = entry(Id);

	if (std::find(vehicle.crew.begin(), vehicle.crew.end(), Peer) != vehicle.crew.end())
		return claim_result::already_member;

	if (vehicle.crew.size() >= published->crew_capacity)
		return claim_result::crew_full;

	if (vehicle.crew.empty())
	{
		// remember whether this vehicle was actually being driven by the AI, so that it is
		// handed back only to a driver that was really there before
		TDynamicObject const *dynamic = Entities.resolve(Id);
		vehicle.ai_before_claim = (dynamic != nullptr && dynamic->Mechanik != nullptr && dynamic->Mechanik->AIControllFlag);
		vehicle.ai_restore_pending = false;
	}

	vehicle.crew.emplace_back(Peer);
	m_pending.emplace_back(Id);

	WriteLog("net: peer " + std::to_string(Peer) + " joined the crew of " + published->name + " (" + std::to_string(vehicle.crew.size()) + "/" +
	             std::to_string((int)published->crew_capacity) + ")",
	         logtype::net);

	return claim_result::granted;
}

bool crew_registry::leave(PeerId Peer, NetworkEntityId Id)
{
	auto lookup = m_vehicles.find(Id);
	if (lookup == m_vehicles.end())
		return false;

	auto &crew = lookup->second.crew;
	auto member = std::find(crew.begin(), crew.end(), Peer);
	if (member == crew.end())
		return false;

	crew.erase(member);
	m_pending.emplace_back(Id);

	WriteLog("net: peer " + std::to_string(Peer) + " left the crew of " + Entities.name_of(Id) + " (" + std::to_string(crew.size()) + " left)", logtype::net);

	if (crew.empty() && lookup->second.ai_before_claim)
	{
		// the vehicle came with an AI driver, so it gets one back
		lookup->second.ai_restore_pending = true;
	}

	return true;
}

int64_t pack_session_config()
{
	int64_t config{0};

	if (Global.FullPhysics)
		config |= CONFIG_FULLPHYSICS;
	if (Global.RealisticControlMode)
		config |= CONFIG_REALISTICCONTROL;

	return config;
}

void apply_session_config(int64_t Config)
{
	bool const fullphysics{(Config & CONFIG_FULLPHYSICS) != 0};
	bool const realisticcontrol{(Config & CONFIG_REALISTICCONTROL) != 0};

	if (Global.FullPhysics != fullphysics)
		WriteLog(std::string("net: the session runs with full physics ") + (fullphysics ? "on" : "off"), logtype::net);
	if (Global.RealisticControlMode != realisticcontrol)
		WriteLog(std::string("net: the session runs with realistic control ") + (realisticcontrol ? "on" : "off"), logtype::net);

	Global.FullPhysics = fullphysics;
	Global.RealisticControlMode = realisticcontrol;
}

void enforce_session_settings()
{
	if (Global.trainThreads != 0)
	{
		// the order vehicles are stepped in must not depend on how threads happen to be
		// scheduled; a session cannot afford that kind of drift
		WriteLog("net: multiplayer session, running the vehicle physics on one thread", logtype::net);
		Global.trainThreads = 0;
	}
}

PeerId resolve_peer_identity(uint64_t &Token)
{
	static std::unordered_map<uint64_t, PeerId> known;

	if (Token != 0)
	{
		auto const lookup = known.find(Token);
		if (lookup != known.end())
		{
			WriteLog("net: peer " + std::to_string(lookup->second) + " recognised, reconnecting", logtype::net);
			Crews.resume_peer(lookup->second);
			return lookup->second;
		}
	}

	auto const peer = allocate_peer_id();

	// only has to be hard to collide with, not hard to guess: it decides who is who
	// across a dropped connection, nothing more
	Token = ((uint64_t)Global.local_random_engine() << 32) ^ (uint64_t)Global.local_random_engine() ^ ((uint64_t)peer << 8);
	if (Token == 0)
		Token = peer;

	known.emplace(Token, peer);

	return peer;
}

void crew_registry::suspend_peer(PeerId Peer)
{
	if (vehicle_of(Peer) == ENTITY_NONE)
		return;

	m_absent[Peer] = Timer::GetTime() + RECONNECT_GRACE_SECONDS;

	WriteLog("net: peer " + std::to_string(Peer) + " lost its connection, holding its seat for " + std::to_string((int)RECONNECT_GRACE_SECONDS) + " s", logtype::net);
}

void crew_registry::resume_peer(PeerId Peer)
{
	m_absent.erase(Peer);
}

bool crew_registry::is_absent(PeerId Peer) const
{
	return m_absent.find(Peer) != m_absent.end();
}

void crew_registry::expire_absences()
{
	auto const now = Timer::GetTime();

	for (auto it = m_absent.begin(); it != m_absent.end();)
	{
		if (now < it->second)
		{
			++it;
			continue;
		}

		WriteLog("net: peer " + std::to_string(it->first) + " did not come back, giving up its seat", logtype::net);
		auto const peer = it->first;
		it = m_absent.erase(it);
		drop_peer(peer);
	}
}

void crew_registry::drop_peer(PeerId Peer)
{
	for (auto &pair : m_vehicles)
	{
		if (std::find(pair.second.crew.begin(), pair.second.crew.end(), Peer) != pair.second.crew.end())
			leave(Peer, pair.first);
	}
}

std::vector<PeerId> crew_registry::crew_of(NetworkEntityId Id) const
{
	auto const lookup = m_vehicles.find(Id);
	return lookup != m_vehicles.end() ? lookup->second.crew : std::vector<PeerId>();
}

bool crew_registry::is_member(PeerId Peer, NetworkEntityId Id) const
{
	auto const lookup = m_vehicles.find(Id);
	if (lookup == m_vehicles.end())
		return false;
	return std::find(lookup->second.crew.begin(), lookup->second.crew.end(), Peer) != lookup->second.crew.end();
}

uint8_t crew_registry::count(NetworkEntityId Id) const
{
	auto const lookup = m_vehicles.find(Id);
	return lookup != m_vehicles.end() ? (uint8_t)lookup->second.crew.size() : (uint8_t)0;
}

NetworkEntityId crew_registry::vehicle_of(PeerId Peer) const
{
	for (auto const &pair : m_vehicles)
	{
		if (std::find(pair.second.crew.begin(), pair.second.crew.end(), Peer) != pair.second.crew.end())
			return pair.first;
	}
	return ENTITY_NONE;
}

void crew_registry::mirror(NetworkEntityId Id, std::vector<PeerId> const &Crew)
{
	entry(Id).crew = Crew;
}

std::vector<NetworkEntityId> crew_registry::take_pending_updates()
{
	std::vector<NetworkEntityId> pending;
	pending.swap(m_pending);

	// the same vehicle can change more than once between two publications
	std::sort(pending.begin(), pending.end());
	pending.erase(std::unique(pending.begin(), pending.end()), pending.end());

	return pending;
}

void crew_registry::mark_all_pending()
{
	for (auto const &pair : m_vehicles)
	{
		if (!pair.second.crew.empty())
			m_pending.emplace_back(pair.first);
	}
}

void crew_registry::clear()
{
	m_vehicles.clear();
	m_pending.clear();
}

namespace
{

// posts a command the way an input source would, but without going through command_relay:
// the authority acts on behalf of the session, not on behalf of the local camera, so it
// must not be filtered by the local free fly state
void post_authority_command(user_command Command, uint32_t Recipient, glm::vec3 const &Location, std::string const &Payload)
{
	command_data data;
	data.command = Command;
	data.action = GLFW_PRESS;
	data.param1 = 0.0;
	data.param2 = 0.0;
	data.time_delta = Timer::GetDeltaTime();
	data.freefly = true;
	data.location = Location;
	data.payload = Payload;

	simulation::Commands.push(data, Recipient);
}

} // namespace

// frames to wait before repeating an authority request that has not taken effect yet
static int const ACTION_COOLDOWN_FRAMES = 30;

void crew_registry::reconcile()
{
	for (auto &pair : m_vehicles)
	{
		vehicle_crew &vehicle = pair.second;

		TDynamicObject *dynamic = Entities.resolve(vehicle.entity_id);
		if (dynamic == nullptr)
			continue;

		if (vehicle.action_cooldown > 0)
		{
			--vehicle.action_cooldown;
			continue;
		}

		TTrain *train = simulation::Trains.find(dynamic->name());

		if (!vehicle.crew.empty())
		{
			if (train == nullptr)
			{
				// the cab has to exist on every peer, so the authority asks for it with the
				// ordinary replicated command instead of building it locally
				post_authority_command(user_command::entervehicle, (uint32_t)command_target::simulation, dynamic->GetPosition(), dynamic->name());
				vehicle.action_cooldown = ACTION_COOLDOWN_FRAMES;
				continue;
			}

			if (dynamic->Mechanik != nullptr && dynamic->Mechanik->AIControllFlag)
			{
				// first human on board takes over from the AI
				WriteLog("net: " + dynamic->name() + " taken over from the AI driver", logtype::net);
				post_authority_command(user_command::aidriverdisable, (uint32_t)command_target::vehicle | train->id(), dynamic->GetPosition(), std::string());
				vehicle.action_cooldown = ACTION_COOLDOWN_FRAMES;
			}
			continue;
		}

		if (vehicle.ai_restore_pending && train != nullptr)
		{
			// aidriverenable resets the driver, so it must happen exactly once
			vehicle.ai_restore_pending = false;
			vehicle.ai_before_claim = false;
			WriteLog("net: handing " + dynamic->name() + " back to the AI driver", logtype::net);
			post_authority_command(user_command::aidriverenable, (uint32_t)command_target::vehicle | train->id(), dynamic->GetPosition(), std::string());
			vehicle.action_cooldown = ACTION_COOLDOWN_FRAMES;
		}
	}
}

} // namespace network
