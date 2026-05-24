//
// Created by Daniu
//

#include "SceneManager.h"
#include "ECScene.h"

SceneManager::SceneManager() = default;

SceneManager::~SceneManager()
{
	if (m_currentScene)
		m_currentScene->Unload();
}

void SceneManager::SetScene(std::unique_ptr<ECScene> scene)
{
	m_pendingScene = std::move(scene);
}

void SceneManager::Update(float dt)
{
	if (m_pendingScene)
		SwitchScene();

	if (m_currentScene){
		m_currentScene->Update(dt);
	}
		
		 
}

void SceneManager::SwitchScene()
{
	if (m_currentScene)
		m_currentScene->Unload();

	m_currentScene = std::move(m_pendingScene);

	if (m_currentScene)
		m_currentScene->Load();
}

ECScene* SceneManager::CurrentScene()
{
	return m_currentScene.get();
}

const ECScene* SceneManager::CurrentScene() const
{
	return m_currentScene.get();
}

bool SceneManager::HasScene() const
{
	return m_currentScene != nullptr;
}