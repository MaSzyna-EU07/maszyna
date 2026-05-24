//
// Created by Daniu
//

#ifndef EU07_BASICSYSTEM_H
#define EU07_BASICSYSTEM_H


#pragma once

class ECWorld;

class BaseSystem
{
public:
	virtual ~BaseSystem() = default;

	virtual void OnCreate(ECWorld& world) {}
	virtual void OnDestroy(ECWorld& world) {}

	virtual void Update(ECWorld& world, float dt) {}
	virtual void Render(ECWorld& world) {}
};


#endif //EU07_BASICSYSTEM_H
