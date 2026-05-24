#include "stdafx.h"
#include "VehicleSyncSystem.h"

#include "entitysystem/ECWorld.h"
#include "entitysystem/components/BasicComponents.h"
#include "vehicle/DynObj.h"

void VehicleSyncSystem::Update(ECWorld& world, float dt)
{
    world.Each<ECSComponent::VehicleRef, ECSComponent::Transform>(
        entt::exclude<ECSComponent::Disabled>,
        [](entt::entity,
           ECSComponent::VehicleRef& ref,
           ECSComponent::Transform& transform)
        {
            if (!ref.vehicle) return;

            auto const &pos = ref.vehicle->GetPosition();
            transform.Position = glm::dvec3(pos.x, pos.y, pos.z);

            // Extract rotation from the vehicle's render matrix (column vectors = basis)
            auto const *m = ref.vehicle->mMatrix.getArray();
            // mMatrix is column-major 4x4; extract upper-left 3x3 into glm
            glm::mat3 rot(
                m[0], m[1], m[2],
                m[4], m[5], m[6],
                m[8], m[9], m[10]);
            transform.Rotation = glm::quat_cast(rot);
        });
}
