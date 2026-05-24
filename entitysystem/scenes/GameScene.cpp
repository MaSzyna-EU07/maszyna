//
// Created by Daniu
//

#include "GameScene.h"

#include "entitysystem/components/BasicComponents.h"
#include "entitysystem/components/RenderComponents.h"
#include "entitysystem/systems/MovementSystem.h"
#include "entitysystem/systems/ParticlesSystem.h"
#include "entitysystem/systems/MeshRenderSystem.h"
#include "entitysystem/systems/LODSystem.h"
#include "entitysystem/systems/SoundSystem.h"
#include "entitysystem/systems/LightSystem.h"
#include "entitysystem/systems/HierarchySystem.h"
#include "entitysystem/systems/LineSystem.h"
#include "entitysystem/systems/VehicleSyncSystem.h"
#include "entitysystem/systems/BillboardSystem.h"
#include "utilities/Logs.h"

GameScene::GameScene() = default;
GameScene::~GameScene() = default;

void GameScene::OnLoad()
{
	m_systems.AddSystem<VehicleSyncSystem>(); // must be first — updates Transform from live vehicle
	m_systems.AddSystem<MovementSystem>();
	m_systems.AddSystem<HierarchySystem>();
	m_systems.AddSystem<LightSystem>();
	m_systems.AddSystem<ParticlesSystem>();
	m_systems.AddSystem<SoundSystem>();
	m_systems.AddSystem<LODSystem>();
	m_systems.AddSystem<MeshRenderSystem>();
	m_systems.AddSystem<LineSystem>();
	m_systems.AddSystem<BillboardSystem>();

#ifdef _DEBUG
	CreateTestEntities();
#endif
}

void GameScene::OnUpdate(float dt)
{
}

void GameScene::OnUnload()
{
	World().Clear();
}

void GameScene::CreateTestEntities()
{
	auto& world = World();

	// smoke emitter at world origin for visual testing of ParticlesSystem
	{
		auto entity = world.CreateEntity();

		auto& id = world.AddComponent<ECSComponent::Identification>(entity);
		id.Name = "TestSmokeEmitter";

		auto& transform = world.AddComponent<ECSComponent::Transform>(entity);
		transform.Position = glm::dvec3(0.0, 2.0, 0.0);

		auto& emitter = world.AddComponent<ECSComponent::ParticleEmitter>(entity);
		emitter.emitterLocation  = glm::vec3(0.0f, 2.0f, 0.0f);
		emitter.isActive         = true;
		emitter.spawnRate        = 15.0f;
		emitter.particleLifetime = 3.0f;
		emitter.gravity          = glm::vec3(0.0f, 0.3f, 0.0f);
		emitter.airResistance    = 0.2f;
		emitter.colorFade        = glm::vec4(0.0f, 0.0f, 0.0f, -0.4f);

		WriteLog("[ECS] Created test entity: TestSmokeEmitter");
	}

	// moving entity to test MovementSystem + LOD culling
	{
		auto entity = world.CreateEntity();

		auto& id = world.AddComponent<ECSComponent::Identification>(entity);
		id.Name = "TestMovingBox";

		auto& transform = world.AddComponent<ECSComponent::Transform>(entity);
		transform.Position = glm::dvec3(10.0, 0.5, 0.0);

		auto& velocity = world.AddComponent<ECSComponent::Velocity>(entity);
		velocity.Value = glm::vec3(-1.0f, 0.0f, 0.0f);

		auto& lod = world.AddComponent<ECSComponent::LODController>(entity);
		lod.RangeMin = 0.0;
		lod.RangeMax = 200.0;

		auto& mesh = world.AddComponent<ECSComponent::MeshRenderer>(entity);
		mesh.visible = true;

		WriteLog("[ECS] Created test entity: TestMovingBox");
	}
}