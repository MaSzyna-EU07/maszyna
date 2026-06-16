//
// Created by Daniu
//

#include "ECWorld.h"
#include "entitysystem/components/BasicComponents.h"

entt::entity ECWorld::CreateEntity()
{
	return m_registry.create();
}

void ECWorld::DestroyEntity(entt::entity entity)
{
	if (m_registry.valid(entity)) {
		if (auto *id = m_registry.try_get<ECSComponent::Identification>(entity))
			m_nameIndex.erase(id->Name.ToString());
		m_registry.destroy(entity);
	}
}

void ECWorld::Clear()
{
	m_nameIndex.clear();
	m_registry.clear();
}

void ECWorld::UpdateNameIndex(entt::entity entity, const std::string &name)
{
	m_nameIndex[name] = entity;
}

bool ECWorld::IsAlive(entt::entity entity) const
{
	return m_registry.valid(entity);
}


entt::entity ECWorld::FindEntityByName(const std::string& name) const
{
	auto it = m_nameIndex.find(name);
	return (it != m_nameIndex.end()) ? it->second : entt::null;
}

entt::registry& ECWorld::Registry()
{
	return m_registry;
}

const entt::registry& ECWorld::Registry() const
{
	return m_registry;
}

ECWorld& GetComponentSystem()
{
	static ECWorld instance;
	return instance;
}
