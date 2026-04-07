//
// Created by Daniu
//

#include "GameScene.h"


#include "entitysystem/components/BasicComponents.h"
#include "entitysystem/components/RenderComponents.h"
#include "entitysystem/systems/MovementSystem.h"
#include "utilities/Logs.h"

GameScene::GameScene() = default;
GameScene::~GameScene()  = default ;

void GameScene::OnLoad()
{
	m_systems.AddSystem<MovementSystem>();
 
}

void GameScene::OnUpdate(float dt)
{

	//auto& world = World();

	//auto view = world.View<ECSComponent::Transform>();
 
	
} 

void GameScene::OnUnload()
{
	World().Clear();
}