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

namespace
{

// counts up for as long as the session lasts; the number a crew member is stamped with is
// the whole of their claim to running the train
uint64_t next_join_stamp()
{
	static uint64_t stamp{0};
	return ++stamp;
}

} // namespace

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
		return may_control(Peer, (NetworkEntityId)(Recipient & 0xffff)) ? command_verdict::accepted : command_verdict::not_in_crew;
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
	if (previous == Entity)
		return claim_result::already_member;

	if ((previous != ENTITY_NONE) && (Entities.consist_of(previous) == Entities.consist_of(Entity)))
	{
		// walking to another car of the train you are already working is not a new claim,
		// so it asks nothing of the capacity. the seat still moves, because which car a
		// player sits in decides which half they keep when the train comes apart
		Crews.leave(Peer, previous);
		Crews.take_seat(Peer, Entity);
		return claim_result::granted;
	}

	auto const result = Crews.claim(Peer, Entity);

	if ((result == claim_result::granted) && (previous != ENTITY_NONE))
	{
		// nobody works two trains at once
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

			if ((command->command == user_command::entervehicle) && (Entities.id_of(command->payload) != ENTITY_NONE))
			{
				// the local player walked into a cab. that is a crew request; the cab
				// itself is built by the authority once the seat is granted.
				// a vehicle the session does not list falls through to the ordinary path,
				// so a scenario that starts its host in something unusual still works
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

bool may_control(PeerId Peer, NetworkEntityId Entity)
{
	auto const seat = Crews.vehicle_of(Peer);
	if (seat == ENTITY_NONE)
		return false;

	auto const train = Entities.consist_of(Entity);
	if (train == ENTITY_NONE)
		return false;

	// the whole set the player is sitting in, so that the desk of a driving trailer works
	// on the motor cars behind it and the cab switches of a multiple unit go through
	return Entities.consist_of(seat) == train;
}

PeerId simulation_owner(NetworkEntityId Entity)
{
	auto const train = Entities.consist_of(Entity);
	if (train == ENTITY_NONE)
		return PEER_HOST;

	// whoever boarded this train first runs it. not the lowest numbered car, not the
	// newest arrival: the physics has to stay with one person for as long as they are
	// aboard, because every handover is a stutter in somebody's ride - and the person
	// who has been driving is the one who would notice it
	PeerId owner{PEER_NONE};
	uint64_t earliest{0};

	for (auto const id : Entities.consist_members(train))
	{
		auto const crew = Crews.crew_of(id);
		auto const since = Crews.crew_since_of(id);

		for (size_t i = 0; i < crew.size(); ++i)
		{
			// a stamp of zero is a crew we were told about by an older peer, or one that
			// somehow lost its order; it sorts behind anything stamped properly
			uint64_t const stamp = (i < since.size() && since[i] != 0 ? since[i] : ~0ull - crew[i]);

			if (owner == PEER_NONE || stamp < earliest)
			{
				earliest = stamp;
				owner = crew[i];
			}
		}
	}

	return owner != PEER_NONE ? owner : PEER_HOST;
}

bool is_locally_simulated(NetworkEntityId Entity)
{
	auto const self = (Global.network_peer_id != PEER_NONE ? Global.network_peer_id : PEER_HOST);
	return simulation_owner(Entity) == self;
}

bool is_predictable(uint32_t Recipient)
{
	if ((command_target)(Recipient & ~0xffff) != command_target::vehicle)
		return false;

	auto const entity = (NetworkEntityId)(Recipient & 0xffff);
	return is_locally_simulated(entity) && may_control(Global.network_peer_id, entity);
}

void collect_predictable(command_queue::commands_map const &Commands, command_queue::commands_map &Predicted)
{
	for (auto const &kv : Commands)
	{
		if (!is_predictable(kv.first))
			continue;

		auto lookup = Predicted.emplace(kv.first, command_queue::commanddata_sequence());
		for (auto const &data : kv.second)
			lookup.first->second.emplace_back(data);
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

	// capacity counts the people on the whole train, not on this one car
	if (published->crew_count >= published->crew_capacity)
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
	vehicle.crew_since.emplace_back(next_join_stamp());
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

	auto const index = (size_t)std::distance(crew.begin(), member);
	crew.erase(member);
	if (index < lookup->second.crew_since.size())
		lookup->second.crew_since.erase(lookup->second.crew_since.begin() + index);
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

TTrain *ensure_local_cab(TDynamicObject *Vehicle)
{
	if (Vehicle == nullptr)
		return nullptr;

	TTrain *train = simulation::Trains.find(Vehicle->name());
	if (train != nullptr)
		return train;

	train = new TTrain();
	if (false == train->Init(Vehicle))
	{
		delete train;
		ErrorLog("net: could not build a cab for " + Vehicle->name(), logtype::net);
		return nullptr;
	}

	simulation::Trains.insert(train);

	return train;
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

void crew_registry::mirror(NetworkEntityId Id, std::vector<PeerId> const &Crew, std::vector<uint64_t> const &Since)
{
	auto &vehicle = entry(Id);
	vehicle.crew = Crew;
	vehicle.crew_since = Since;
	vehicle.crew_since.resize(Crew.size(), 0);
}

std::vector<uint64_t> crew_registry::crew_since_of(NetworkEntityId Id) const
{
	auto const lookup = m_vehicles.find(Id);
	return lookup != m_vehicles.end() ? lookup->second.crew_since : std::vector<uint64_t>();
}

void crew_registry::take_seat(PeerId Peer, NetworkEntityId Id)
{
	auto &vehicle = entry(Id);
	if (std::find(vehicle.crew.begin(), vehicle.crew.end(), Peer) != vehicle.crew.end())
		return;

	if (vehicle.crew.empty())
	{
		TDynamicObject const *dynamic = Entities.resolve(Id);
		vehicle.ai_before_claim = (dynamic != nullptr && dynamic->Mechanik != nullptr && dynamic->Mechanik->AIControllFlag);
		vehicle.ai_restore_pending = false;
	}

	vehicle.crew.emplace_back(Peer);
	vehicle.crew_since.emplace_back(next_join_stamp());
	m_pending.emplace_back(Id);
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
// how many times to ask a vehicle's AI driver to step aside before giving up on it
static int const AI_HANDOVER_ATTEMPTS = 5;

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
				// the authority needs a cab of its own to address the AI commands to. it
				// is built here and not replicated: a cab belongs to the peer it is on,
				// and every peer that is actually aboard builds its own
				train = ensure_local_cab(dynamic);
				if (train == nullptr)
				{
					vehicle.action_cooldown = ACTION_COOLDOWN_FRAMES;
					continue;
				}
			}

			if (dynamic->Mechanik != nullptr && dynamic->Mechanik->AIControllFlag)
			{
				if (vehicle.ai_attempts < AI_HANDOVER_ATTEMPTS)
				{
					// first human on board takes over from the AI
					if (vehicle.ai_attempts == 0)
						WriteLog("net: " + dynamic->name() + " taken over from the AI driver", logtype::net);

					++vehicle.ai_attempts;
					post_authority_command(user_command::aidriverdisable, (uint32_t)command_target::vehicle | train->id(), dynamic->GetPosition(), std::string());
					vehicle.action_cooldown = ACTION_COOLDOWN_FRAMES;
				}
				else if (vehicle.ai_attempts == AI_HANDOVER_ATTEMPTS)
				{
					// asking again would only fill the log; say so once and leave it
					++vehicle.ai_attempts;
					ErrorLog("net: " + dynamic->name() + " will not let go of its AI driver", logtype::net);
				}
			}
			else
			{
				vehicle.ai_attempts = 0;
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
