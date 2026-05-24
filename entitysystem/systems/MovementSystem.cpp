//
// Created by Daniu  
//

#include "MovementSystem.h"
#include "entitysystem/ECWorld.h"
#include "entitysystem/components/BasicComponents.h"
#include "utilities/Logs.h"


void MovementSystem::Update(ECWorld& world, float dt)
{
	constexpr double damping = 0.90; // 0.0 = instant stop, 1.0 = without damping 

    world.Each<ECSComponent::Transform, ECSComponent::Velocity>(
        entt::exclude<ECSComponent::Disabled>,
        [dt](auto entity, auto& transform, auto& velocity)
        {
             
            transform.Position += velocity.Value * dt;
 
            velocity.Value *= damping;
 
            if (glm::length(velocity.Value) < 0.001)
            {
                velocity.Value = glm::vec3(0.0f);
            }
        }
    );
}