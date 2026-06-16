//
// Created by Daniu
//

#ifndef EU07_ECWORLD_H
#define EU07_ECWORLD_H



#pragma once

#include <entt/entt.hpp>
#include <utility>
#include <type_traits>
#include <unordered_map>
#include <string>

// Forward declaration so the name-index branch in AddComponent<T> compiles in
// translation units that include ECWorld.h without BasicComponents.h.
// if constexpr discards the branch for non-Identification T, but the name must
// still resolve at parse time.
namespace ECSComponent { struct Identification; }

class ECWorld
{
public:
	ECWorld() = default;
	~ECWorld() = default;

	entt::entity CreateEntity();
	void DestroyEntity(entt::entity entity);
	void Clear();
	// Updates the name→entity index after an Identification component's Name is changed at runtime.
	void UpdateNameIndex(entt::entity entity, const std::string &name);

	bool IsAlive(entt::entity entity) const;

	entt::registry& Registry();
	const entt::registry& Registry() const;

	template<typename T, typename... Args>
	T& AddComponent(entt::entity entity, Args&&... args);

	template<typename T>
	bool HasComponent(entt::entity entity) const;

	template<typename T>
	T* GetComponent(entt::entity entity);

	template<typename T>
	const T* GetComponent(entt::entity entity) const;

	template<typename T>
	void RemoveComponent(entt::entity entity);
 
	entt::entity FindEntityByName(const std::string& name) const;

	template<typename... Components>
	auto View()
	{
		return m_registry.view<Components...>();
	}
	
	template<typename... Components, typename Func>
	void Each(Func&& func)
	{
		auto view = m_registry.view<Components...>();

		for (auto entity : view)
		{
			auto components = std::forward_as_tuple(view.get<Components>(entity)...);

			std::apply([&](Components&... comps)
			{
				func(entity, comps...);
			}, components);
		}
	}

	// Same as Each but skips entities that have any of the Excluded components.
	// Usage: world.Each<A, B>(entt::exclude<Disabled>, lambda)
	template<typename... Components, typename... Excluded, typename Func>
	void Each(entt::exclude_t<Excluded...>, Func&& func)
	{
		auto view = m_registry.view<Components...>(entt::exclude<Excluded...>);

		for (auto entity : view)
		{
			auto components = std::forward_as_tuple(view.get<Components>(entity)...);

			std::apply([&](Components&... comps)
			{
				func(entity, comps...);
			}, components);
		}
	}
  
	std::vector<entt::entity> GetEntities() const 
	{
		std::vector<entt::entity> entities;
		auto *storage = m_registry.storage<entt::entity>();

		for (auto entity : *storage)
		{
		
			entities.push_back(entity);
		}
		return entities;
	}

	std::size_t GetEntityCount() const
	{
		return m_registry.storage<entt::entity>()->size();
	}

private:
	entt::registry m_registry;
	std::unordered_map<std::string, entt::entity> m_nameIndex;
};

template<typename T, typename... Args>
T& ECWorld::AddComponent(entt::entity entity, Args&&... args)
{
	static_assert(std::is_constructible_v<T, Args...>,
		"Component must be constructible with provided arguments");

	if (m_registry.all_of<T>(entity))
		m_registry.replace<T>(entity, std::forward<Args>(args)...);
	else
		m_registry.emplace<T>(entity, std::forward<Args>(args)...);

	if constexpr (std::is_empty_v<T>) {
		static T dummy{};
		return dummy;
	} else {
		auto &comp = m_registry.get<T>(entity);
		if constexpr (std::is_same_v<T, ECSComponent::Identification>)
			m_nameIndex[comp.Name.ToString()] = entity;
		return comp;
	}
}

template<typename T>
bool ECWorld::HasComponent(entt::entity entity) const
{
	return m_registry.all_of<T>(entity);
}

template<typename T>
T* ECWorld::GetComponent(entt::entity entity)
{
	return m_registry.try_get<T>(entity);
}

template<typename T>
const T* ECWorld::GetComponent(entt::entity entity) const
{
	return m_registry.try_get<T>(entity);
}

template<typename T>
void ECWorld::RemoveComponent(entt::entity entity)
{
	if (m_registry.all_of<T>(entity))
		m_registry.remove<T>(entity);
}



extern ECWorld& GetComponentSystem();
#define ECS GetComponentSystem()
#endif //EU07_ECWORLD_H
