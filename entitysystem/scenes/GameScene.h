//
// Created by Daniu
//

#ifndef EU07_GAMESCENE_H
#define EU07_GAMESCENE_H



#pragma once

#include "entitysystem/ECScene.h"

class GameScene final : public ECScene
{
public:
	GameScene(); 
	~GameScene() override;

private:
    void OnLoad() override;
    void OnUnload() override;
    void OnUpdate(float dt) override;

private:
	void CreateTestEntities();
};


#endif //EU07_GAMESCENE_H
