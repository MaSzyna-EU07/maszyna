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

	// anything with its own drive, plus anything the scenario already put a driver in
	// (this covers driving trailers of multiple units, which carry no engine of their own)
	// TODO: a cab count exposed by TMoverParameters would be a more precise test
	return Vehicle->MoverParameters->EngineType != TEngineType::None || Vehicle->Mechanik != nullptr;
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

	std::vector<std::string> names;
	for (TDynamicObject *vehicle : simulation::Vehicles.sequence())
	{
		if (!is_drivable(vehicle))
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

	reindex();
	refresh();

	WriteLog("net: assigned network ids to " + std::to_string(m_entries.size()) + " drivable vehicles", logtype::net);
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
	for (auto &entry : m_entries)
	{
		TDynamicObject const *vehicle = resolve(entry.id);

		entry.ai_active = (vehicle != nullptr && vehicle->Mechanik != nullptr && vehicle->Mechanik->AIControllFlag);
		entry.crew_count = Crews.count(entry.id);
		entry.crew_capacity = CREW_CAPACITY;
		entry.claimable = (vehicle != nullptr && entry.crew_count < entry.crew_capacity);
	}
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
