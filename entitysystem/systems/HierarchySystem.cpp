#include "stdafx.h"
#include "HierarchySystem.h"

#include "entitysystem/ECWorld.h"
#include "entitysystem/components/BasicComponents.h"

void HierarchySystem::Update(ECWorld& world, float dt)
{
    world.Each<ECSComponent::Parent, ECSComponent::Transform>(
        entt::exclude<ECSComponent::Disabled>,
        [&](entt::entity, ECSComponent::Parent& parent, ECSComponent::Transform& childTransform)
        {
            if (parent.value == entt::null || !world.IsAlive(parent.value))
                return;

            const auto* parentTransform = world.GetComponent<ECSComponent::Transform>(parent.value);
            if (!parentTransform)
                return;

            // Rotate local offset by parent rotation then add parent world position.
            glm::dvec3 rotatedOffset = glm::dvec3(parentTransform->Rotation * glm::vec3(parent.localOffset));
            childTransform.Position = parentTransform->Position + rotatedOffset;

            // Compose rotations: child world = parent world * child local.
            childTransform.Rotation = parentTransform->Rotation * parent.localRotation;
        });
}
