/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "network/entities.h"
#include "network/session.h"

#include "simulation/simulation.h"
#include "vehicle/DynObj.h"
#include "vehicle/Driver.h"
#include "vehicle/Train.h"
#include "utilities/Logs.h"
#include "utilities/Globals.h"

namespace network
{

entity_registry Entities;
peer_registry Peers;

namespace
{

// long enough for a name worth having, short enough not to wreck a lobby column
constexpr size_t NICKNAME_LIMIT = 24;

std::string tidy_name(std::string const &Raw)
{
	std::string name;
	name.reserve(Raw.size());

	bool space{false};
	for (char const character : Raw)
	{
		// control characters and tabs have no business in a name that goes on somebody
		// else's screen, and a run of spaces is one space
		unsigned char const value = (unsigned char)character;
		if (value < 0x20 || value == 0x7f)
			continue;

		if (character == ' ')
		{
			space = !name.empty();
			continue;
		}

		if (space)
		{
			name += ' ';
			space = false;
		}

		name += character;
		if (name.size() >= NICKNAME_LIMIT)
			break;
	}

	return name;
}

} // namespace

std::string local_nickname()
{
	auto name = tidy_name(Global.multiplayer_nickname);
	if (!name.empty())
		return name;

	// nothing configured: whoever is logged in will do, and failing that nothing at all -
	// the registry has a stand-in for that case
	for (char const *variable : {"EU07_NICKNAME", "USERNAME", "USER", "LOGNAME"})
	{
		char const *value = std::getenv(variable);
		if (value == nullptr)
			continue;
		name = tidy_name(value);
		if (!name.empty())
			return name;
	}

	return std::string();
}

bool peer_registry::taken(std::string const &Name, PeerId const Except) const
{
	for (auto const &entry : m_names)
	{
		if (entry.first == Except)
			continue;
		if (entry.second == Name)
			return true;
	}
	return false;
}

std::string peer_registry::assign(PeerId const Peer, std::string const &Requested)
{
	std::string name = tidy_name(Requested);

	if (name.empty())
		name = (Peer == PEER_HOST ? std::string("host") : "player " + std::to_string(Peer));

	// two people called Marcin are two people, and the lobby has to say which is which
	if (taken(name, Peer))
	{
		std::string const base = name;
		for (int suffix = 2; suffix < 100; ++suffix)
		{
			name = base + " (" + std::to_string(suffix) + ")";
			if (!taken(name, Peer))
				break;
		}
	}

	m_names[Peer] = name;

	return name;
}

void peer_registry::forget(PeerId const Peer)
{
	m_names.erase(Peer);
}

void peer_registry::adopt(std::vector<peer_entry> const &Roster)
{
	m_names.clear();
	for (auto const &entry : Roster)
		m_names[entry.id] = entry.name;
}

std::string peer_registry::name_of(PeerId const Peer) const
{
	auto const lookup = m_names.find(Peer);
	if (lookup != m_names.end())
		return lookup->second;

	if (Peer == PEER_HOST)
		return "host";
	if (Peer == PEER_NONE)
		return "nobody";

	return "player " + std::to_string(Peer);
}

std::vector<peer_entry> peer_registry::roster() const
{
	std::vector<peer_entry> entries;
	entries.reserve(m_names.size());
	for (auto const &entry : m_names)
		entries.emplace_back(peer_entry{entry.first, entry.second});
	return entries;
}

void peer_registry::clear()
{
	m_names.clear();
}

PeerId allocate_peer_id()
{
	static PeerId next = PEER_FIRST_REMOTE;
	return next++;
}

bool is_multiplayer()
{
	return Global.network_client.has_value() || !Global.network_servers.empty();
}

bool is_authority()
{
	return !Global.network_client.has_value();
}

bool is_drivable(TDynamicObject const *Vehicle)
{
	if (Vehicle == nullptr || Vehicle->MoverParameters == nullptr)
		return false;

	auto const &mover = *Vehicle->MoverParameters;

	// a driving position is what counts, not an engine: the driving trailer of a multiple
	// unit has a full desk and no traction of its own, and the scenario puts nobody in it
	return mover.EngineType != TEngineType::None || Vehicle->Mechanik != nullptr || mover.MainCtrlPosNo > 0 || mover.BrakeCtrlPosNo > 0;
}

void entity_registry::clear()
{
	m_entries.clear();
	m_byname.clear();
	m_byid.clear();
}

void entity_registry::build()
{
	clear();

	// every vehicle gets an id, not only the ones worth driving. commands are addressed by
	// it, and a player working a train may walk into any of its cars
	std::vector<std::string> names;
	for (TDynamicObject *vehicle : simulation::Vehicles.sequence())
	{
		if (vehicle == nullptr)
			continue;
		names.emplace_back(vehicle->name());
	}

	// name order keeps the numbering reproducible regardless of scenario spawn order
	std::sort(names.begin(), names.end());

	for (auto const &name : names)
	{
		if (m_byname.find(name) != m_byname.end())
		{
			// a malformed scenario can hold duplicate names; only the first one is addressable
			ErrorLog("net: duplicate vehicle name \"" + name + "\", not assigning a second network id", logtype::net);
			continue;
		}

		vehicle_entry entry;
		entry.id = static_cast<NetworkEntityId>(m_entries.size() + 1);
		entry.name = name;
		m_entries.emplace_back(entry);
		m_byname.emplace(name, entry.id);
	}

	if (m_entries.size() > 0xffff)
	{
		// the command channel packs the recipient into sixteen bits
		ErrorLog("net: scenario holds more vehicles than the command channel can address", logtype::net);
	}

	reindex();
	refresh();

	uint32_t drivable{0};
	for (auto const &entry : m_entries)
		if (entry.drivable)
			++drivable;

	WriteLog("net: assigned network ids to " + std::to_string(m_entries.size()) + " vehicles, " + std::to_string(drivable) + " of them with a driving position",
	         logtype::net);
}

void entity_registry::adopt(std::vector<vehicle_entry> const &Entries)
{
	m_entries = Entries;
	m_byname.clear();
	for (auto const &entry : m_entries)
		m_byname.emplace(entry.name, entry.id);
	reindex();
}

void entity_registry::reindex()
{
	m_byid.clear();
	for (size_t i = 0; i < m_entries.size(); ++i)
		m_byid.emplace(m_entries[i].id, i);
}

void entity_registry::refresh()
{
	// only there so that a malformed consist cannot spin us forever
	int const CONSIST_LIMIT{256};

	for (auto &entry : m_entries)
	{
		TDynamicObject const *vehicle = resolve(entry.id);

		entry.consist_id = entry.id;
		entry.consist_size = 1;
		entry.consist_lead = false;
		entry.drivable = is_drivable(vehicle);
		entry.ai_active = (vehicle != nullptr && vehicle->Mechanik != nullptr && vehicle->Mechanik->AIControllFlag);
		entry.crew_capacity = CREW_CAPACITY;
	}

	// group the vehicles into the trains they are currently coupled into. this is redone
	// every time, so a train that comes apart turns into two trains here, and the crews
	// follow whichever half each player happens to be sitting in
	std::unordered_set<TDynamicObject const *> visited;

	for (auto const &entry : m_entries)
	{
		TDynamicObject *vehicle = resolve(entry.id);
		if (vehicle == nullptr || visited.count(vehicle) > 0)
			continue;

		TDynamicObject *front = vehicle;
		for (int step = 0; step < CONSIST_LIMIT; ++step)
		{
			TDynamicObject *previous = front->Prev();
			if (previous == nullptr || previous == vehicle || visited.count(previous) > 0)
				break;
			front = previous;
		}

		std::vector<NetworkEntityId> members;
		TDynamicObject *member = front;
		for (int step = 0; step < CONSIST_LIMIT && member != nullptr; ++step, member = member->Next())
		{
			if (visited.count(member) > 0)
				break;
			visited.emplace(member);

			auto const id = id_of(member->name());
			if (id != ENTITY_NONE)
				members.emplace_back(id);
		}

		if (members.empty())
			continue;

		// the lowest id in the set names the train, and the lowest one that can actually
		// be driven stands for it in the lobby
		NetworkEntityId consist{members.front()};
		NetworkEntityId lead{ENTITY_NONE};
		uint8_t crew{0};

		for (auto const id : members)
		{
			consist = std::min(consist, id);
			crew = (uint8_t)std::min<int>(255, crew + Crews.count(id));

			auto const *candidate = find(id);
			if (candidate != nullptr && candidate->drivable && (lead == ENTITY_NONE || id < lead))
				lead = id;
		}
		if (lead == ENTITY_NONE)
			lead = consist;

		for (auto const id : members)
		{
			auto const lookup = m_byid.find(id);
			if (lookup == m_byid.end())
				continue;

			auto &target = m_entries[lookup->second];
			target.consist_id = consist;
			target.consist_size = (uint8_t)std::min<size_t>(255, members.size());
			target.consist_lead = (id == lead);
			target.crew_count = crew;
			target.claimable = (crew < target.crew_capacity);
		}
	}
}

NetworkEntityId entity_registry::consist_of(NetworkEntityId Id) const
{
	auto const *entry = find(Id);
	return entry != nullptr ? entry->consist_id : ENTITY_NONE;
}

std::vector<NetworkEntityId> entity_registry::consist_members(NetworkEntityId Consist) const
{
	std::vector<NetworkEntityId> members;
	if (Consist == ENTITY_NONE)
		return members;

	for (auto const &entry : m_entries)
	{
		if (entry.consist_id == Consist)
			members.emplace_back(entry.id);
	}

	return members;
}

NetworkEntityId entity_registry::id_of(std::string const &Name) const
{
	auto const lookup = m_byname.find(Name);
	return lookup != m_byname.end() ? lookup->second : ENTITY_NONE;
}

NetworkEntityId entity_registry::id_of(TDynamicObject const *Vehicle) const
{
	return Vehicle != nullptr ? id_of(Vehicle->name()) : ENTITY_NONE;
}

std::string entity_registry::name_of(NetworkEntityId Id) const
{
	auto const *entry = find(Id);
	return entry != nullptr ? entry->name : std::string();
}

vehicle_entry const *entity_registry::find(NetworkEntityId Id) const
{
	auto const lookup = m_byid.find(Id);
	return lookup != m_byid.end() ? &m_entries[lookup->second] : nullptr;
}

TDynamicObject *entity_registry::resolve(NetworkEntityId Id) const
{
	auto const *entry = find(Id);
	if (entry == nullptr)
		return nullptr;

	return simulation::Vehicles.find(entry->name);
}

} // namespace network
