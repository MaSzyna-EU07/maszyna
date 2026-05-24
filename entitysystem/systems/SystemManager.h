//
// Created by Daniu
//

#ifndef EU07_SYSTEMMANAGER_H
#define EU07_SYSTEMMANAGER_H



#pragma once

#include "BaseSystem.h"

#include <vector>
#include <memory>

class ECWorld;

class SystemManager
{
public:
	SystemManager() = default;
	~SystemManager();

	template<typename T, typename... Args>
	T& AddSystem(Args&&... args)
	{
		static_assert(std::is_base_of_v<BaseSystem, T>);
		auto system = std::make_unique<T>(std::forward<Args>(args)...);
		T& ref = *system;
		m_systems.emplace_back(std::move(system));
		return ref;
	}

	void Create(ECWorld& world);
	void Destroy(ECWorld& world);

	void Update(ECWorld& world, float dt);
	void Render(ECWorld& world);

private:
	std::vector<std::unique_ptr<BaseSystem>> m_systems;
};



#endif //EU07_SYSTEMMANAGER_H
