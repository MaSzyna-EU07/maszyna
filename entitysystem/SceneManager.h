//
// Created by Daniu
//

#ifndef EU07_SCENEMANAGER_H
#define EU07_SCENEMANAGER_H


#pragma once

#include <memory>
#include <string>

class ECScene;

class SceneManager
{
public:
	SceneManager();
	~SceneManager();

	void SetScene(std::unique_ptr<ECScene> scene);
	void Update(float dt);

	ECScene* CurrentScene();
	const ECScene* CurrentScene() const;

	bool HasScene() const;

private:
	void SwitchScene();

private:
	std::unique_ptr<ECScene> m_currentScene;
	std::unique_ptr<ECScene> m_pendingScene;
};



#endif //EU07_SCENEMANAGER_H
