//
// Created by Hirek on 3/14/2026.
//

#include "ecs.h"

void ECS::ClearWorld()
{
	world_.clear();
}

entt::entity ECS::CreateObject()
{
	const auto e = world_.create();
	world_.emplace<ECSComponent::Transform>(e);
	// add UID
	auto id = world_.emplace<ECSComponent::Identification>(e);
	id.Id = nextId_++;
	return e;
}

void ECS::DestroyObject(entt::entity Entity)
{
	if (Entity == entt::null) // check if Entity is not null
		return;
	if (!world_.valid(Entity)) // check if exist
		return;
	world_.destroy(Entity);
}

bool ECS::ValidObject(entt::entity entity) const
{
	return entity != entt::null && world_.valid(entity);
}

entt::entity ECS::FindById(std::uint64_t id)
{
	auto view = world_.view<const ECSComponent::Identification>();
	for (auto e : view)
	{
		const auto &ident = view.get<ECSComponent::Identification>(e);
		if (ident.Id == id)
			return e;
	}
	return entt::null;
}

std::vector<entt::entity> ECS::FindByName(std::string_view name)
{
	std::vector<entt::entity> result;

	auto view = world_.view<const ECSComponent::Identification>();
	for (auto e : view)
	{
		const auto &ident = view.get<ECSComponent::Identification>(e);
		if (ident.Name == name)
		{
			result.push_back(e);
		}
	}

	return result;
}

ECS &GetComponentSystem()
{
	static ECS _ecs;
	return _ecs;
}