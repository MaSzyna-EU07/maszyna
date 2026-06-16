//
// Created by Daniu
//

#include "ECScene.h"
#include "utilities/Logs.h"
#include "simulation/simulation.h"

ECScene::ECScene()
	: m_loaded(false)
{
}

ECScene::~ECScene()
{
	Unload();
}

void ECScene::Load()
{
	if (m_loaded)
		return;

	m_systems.Create(m_world);
	OnLoad();
	m_loaded = true;
}
void ECScene::Unload()
{
	if (!m_loaded)
		return;

	OnUnload();
	m_world.Clear();
	m_loaded = false;
}

void ECScene::Update(float dt)
{
	if (!m_loaded)
		return;

	if (!simulation::is_ready)
		return;

	m_systems.Update(m_world, dt);
	OnUpdate(dt);
}

void ECScene::Render()
{
	if (!m_loaded)
		return;

	m_systems.Render(m_world);
}

 
ECWorld& ECScene::World()
{
    return m_world;
}

const ECWorld& ECScene::World() const
{
	return m_world;
}

void ECScene::OnLoad()
{
}

void ECScene::OnUnload()
{;
}

void ECScene::OnUpdate(float dt)
{
	(void)dt;
}