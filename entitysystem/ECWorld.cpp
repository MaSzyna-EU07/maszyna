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
	if (m_registry.valid(entity))
		m_registry.destroy(entity);
}

void ECWorld::Clear()
{
	m_registry.clear();
}

bool ECWorld::IsAlive(entt::entity entity) const
{
	return m_registry.valid(entity);
}


entt::entity ECWorld::FindEntityByName(const std::string& name) const
{
	auto view = m_registry.view<ECSComponent::Identification>();

	for (auto entity : view)
	{
		const auto& id = view.get<ECSComponent::Identification>(entity);
		if (id.Name.ToString() == name)
		{
			return entity;
		}
	}

	return entt::null;
}

entt::registry& ECWorld::Registry()
{
	return m_registry;
}

const entt::registry& ECWorld::Registry() const
{
	return m_registry;
}
