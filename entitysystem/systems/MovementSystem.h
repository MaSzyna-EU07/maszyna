//
// Created by Daniu
//

#ifndef EU07_MOVEMENTSYSTEM_H
#define EU07_MOVEMENTSYSTEM_H


#pragma once
#include "BaseSystem.h"

namespace ECSComponent
{
struct Transform;
struct Velocity;
}

class MovementSystem : public BaseSystem
{
public:
	void Update(ECWorld& world, float dt) override;
};


#endif //EU07_MOVEMENTSYSTEM_H
