#include "stdafx.h"
#include "SoundSystem.h"

#include "entitysystem/ECWorld.h"
#include "entitysystem/components/BasicComponents.h"

void SoundSystem::Update(ECWorld& world, float dt)
{
    world.Each<ECSComponent::SoundComponent, ECSComponent::Transform>(
        entt::exclude<ECSComponent::Disabled>,
        [](entt::entity,
           ECSComponent::SoundComponent& sc,
           ECSComponent::Transform& transform)
        {
            sc.sound.offset( glm::vec3( transform.Position ) );
            sc.sound.gain( sc.volume );
            sc.sound.pitch( sc.pitch );

            if (sc.isPlaying && !sc.sound.is_playing()) {
                int flags = sc.loop ? sound_flags::looping : 0;
                sc.sound.play( flags );
            } else if (!sc.isPlaying && sc.sound.is_playing()) {
                sc.sound.stop();
            }
        });
}
