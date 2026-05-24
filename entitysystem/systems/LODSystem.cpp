#include "stdafx.h"
#include "LODSystem.h"

#include "entitysystem/ECWorld.h"
#include "entitysystem/components/BasicComponents.h"
#include "entitysystem/components/RenderComponents.h"
#include "utilities/Globals.h"
#include "model/AnimModel.h"

void LODSystem::Update(ECWorld& world, float dt)
{
    const glm::dvec3 camPos{
        Global.pCamera.Pos.x,
        Global.pCamera.Pos.y,
        Global.pCamera.Pos.z
    };

    world.Each<ECSComponent::LODController, ECSComponent::Transform>(
        entt::exclude<ECSComponent::Disabled>,
        [&](entt::entity entity,
            ECSComponent::LODController& lod,
            ECSComponent::Transform& transform)
        {
            double dist = glm::length(camPos - transform.Position);
            bool inRange = (dist >= lod.RangeMin) && (dist <= lod.RangeMax);

            if (auto* mesh = world.GetComponent<ECSComponent::MeshRenderer>(entity)) {
                mesh->visible = inRange;
                if (mesh->modelInstance)
                    mesh->modelInstance->visible(inRange);
            }
        });
}
