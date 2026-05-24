//
// Created by Daniu
//

#include "SystemManager.h"
#include "entitysystem/systems/BaseSystem.h"
#include "entitysystem/ECWorld.h"
#include "utilities/Logs.h"

SystemManager::~SystemManager() = default;

void SystemManager::Create(ECWorld& world)
{
	WriteLog("[SystemManager] Creating systems");
	for (auto& system : m_systems){
		system->OnCreate(world);
	}
		
}

void SystemManager::Destroy(ECWorld& world)
{
	WriteLog("[SystemManager] Destroying systems");
	for (auto& system : m_systems)
		system->OnDestroy(world);
}

void SystemManager::Update(ECWorld& world, float dt)
{
	for (auto& system : m_systems)
		system->Update(world, dt);
}

void SystemManager::Render(ECWorld& world)
{
	for (auto& system : m_systems)
		system->Render(world);
}
