#pragma once

#include "BaseSystem.h"

// Drives SoundComponent: starts/stops sound_source playback and keeps
// position, volume and pitch in sync with the entity's Transform.
class SoundSystem : public BaseSystem
{
public:
    void Update(ECWorld& world, float dt) override;
};
