//
// Created by Daniu
//

#ifndef EU07_ECSCENE_H
#define EU07_ECSCENE_H



#pragma once

#include "ECWorld.h"
#include "systems/SystemManager.h"

class ECScene
{
public:
	ECScene();
	virtual ~ECScene();

	void Load();
	void Unload();
	void Update(float dt);

	ECWorld& World();
	const ECWorld& World() const;

protected:
	virtual void OnLoad();
	virtual void OnUnload();
	virtual void OnUpdate(float dt);

protected:
	ECWorld m_world;
	SystemManager m_systems;

private:
	bool m_loaded;
};


#endif //EU07_ECSCENE_H
