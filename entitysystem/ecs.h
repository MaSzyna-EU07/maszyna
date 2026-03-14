//
// Created by Hirek on 3/14/2026.
//

#ifndef EU07_ECS_H
#define EU07_ECS_H
#include "components/BasicComponents.h"
#include "entt/entity/registry.hpp"
#include <cstdint>
#include <vector>
#include <string_view>
#include <utility>
class ECS final
{
  private:
	entt::registry world_;

  public:
	//
	// World lifecycle
	//

	/// <summary>
	/// Clears the world, removing all entities and components
	/// </summary>
	void ClearWorld();

	//
	// Objects
	//

	/// <summary>
	/// Creates new object with basic components (Transform, Identification) and returns its entity handle.
	/// </summary>
	/// <returns>Entity handle of created object</returns>
	entt::entity CreateObject();

	/// <summary>
	/// Destroys object with it's all components
	/// </summary>
	/// <param name="Entity">Entity to be removed</param>
	void DestroyObject(entt::entity Entity);

	/// <summary>
	/// Checks if object with given entity handle exists in the registry
	/// </summary>
	/// <param name="entity">Entity handle to check</param>
	/// <returns>true if object exists, false otherwise</returns>
	bool ValidObject(entt::entity entity) const;

	//
	// Identification lookups
	//

	/// <summary>
	/// Finds object with given UID. Returns null entity if not found.
	/// </summary>
	/// <param name="id">UID of object</param>
	entt::entity FindById(std::uint64_t id);

	/// <summary>
	/// Finds objects with given name. Returns empty vector if not found.
	/// </summary>
	/// <param name="name">Name of object</param>
	/// <remarks>May return more than 1 as many objects can have the same name</remarks>
	std::vector<entt::entity> FindByName(std::string_view name);

	//
	// Components
	//

	/// <summary>
	/// Adds component of type T to entity, forwarding provided arguments to component constructor. If component already exists, it will be replaced.
	/// </summary>
	/// <typeparam name="T">Component type</typeparam>
	/// <param name="entity">Entity to which component will be added</param>
	/// <param name="args">Arguments forwarded to component constructor</param>
	/// <returns>Reference to added component</returns>
	template <class T, class... Args> T &Add(entt::entity entity, Args &&...args)
	{
		return world_.emplace<T>(entity, std::forward<Args>(args)...);
	}

	/// <summary>
	/// Returns entity's component
	/// </summary>
	/// <typeparam name="T"> type</typeparam>
	/// <param name="entity">Entity to which component will be added</param>
	/// <returns>Reference to added component</returns>
	template <class T> T &Get(entt::entity entity)
	{
		return world_.get<T>(entity);
	}

	/// <summary>
	/// Returns entity's component
	/// </summary>
	/// <typeparam name="T"> type</typeparam>
	/// <param name="entity">Entity to which component will be added</param>
	/// <returns>Reference to added component</returns>
	template <class T> const T &Get(entt::entity entity) const
	{
		return world_.get<T>(entity);
	}

	/// <summary>
	/// Tries to get component of type T from entity. Returns nullptr if component does not exist.
	/// </summary>
	/// <typeparam name="T">Component type</typeparam>
	/// <param name="entity">Entity from which component will be retrieved</param>
	/// <returns>Pointer to component if exists, nullptr otherwise</returns>
	template <class T> T *TryGet(entt::entity entity)
	{
		return world_.try_get<T>(entity);
	}

	/// <summary>
	/// Tries to get component of type T from entity. Returns nullptr if component does not exist.
	/// </summary>
	/// <typeparam name="T">Component type</typeparam>
	/// <param name="entity">Entity from which component will be retrieved</param>
	/// <returns>Pointer to component if exists, nullptr otherwise</returns>
	template <class T> const T *TryGet(entt::entity entity) const
	{
		return world_.try_get<T>(entity);
	}

	/// <summary>
	/// Checks if entity has component of type T.
	/// </summary>
	/// <typeparam name="T">Component type</typeparam>
	/// <param name="entity">Entity to check</param>
	/// <returns>true if entity has component, false otherwise</returns>
	template <class... T> bool Has(entt::entity entity) const
	{
		return world_.all_of<T...>(entity);
	}

	/// <summary>
	/// Removes component of type T from entity. Does nothing if component does not exist.
	/// </summary> <typeparam name="T">Component type</typeparam>
	/// <param name="entity">Entity from which component will be removed</param>
	template <class... T> void Remove(entt::entity entity)
	{
		world_.remove<T...>(entity);
	}
};
#endif // EU07_ECS_H
