#include "stdafx.h"
#include "LightSystem.h"

#include "entitysystem/ECWorld.h"
#include "entitysystem/components/BasicComponents.h"
#include "entitysystem/components/RenderComponents.h"
#include "utilities/Globals.h"
#include "rendering/lightarray.h"
#include "simulation/simulation.h"

void LightSystem::Update(ECWorld& world, float dt)
{
    // Sync SunLight intensity with global overcast value.
    // Overcast 0 = clear sky (full sun), 2 = heavy overcast (dim).
    const float sunIntensity = glm::clamp( 1.0f - Global.Overcast * 0.4f, 0.1f, 1.0f );
    world.Each<ECSComponent::SunLight>(
        entt::exclude<ECSComponent::Disabled>,
        [sunIntensity](entt::entity, ECSComponent::SunLight& sun)
        {
            if (sun.Enabled)
                sun.Intensity = sunIntensity;
        });

    // SpotLights: cull distant lights and rebuild the free_lights list for the renderer.
    const glm::dvec3 camPos{
        Global.pCamera.Pos.x,
        Global.pCamera.Pos.y,
        Global.pCamera.Pos.z
    };
    simulation::Lights.free_lights.clear();
    world.Each<ECSComponent::SpotLight, ECSComponent::Transform>(
        entt::exclude<ECSComponent::Disabled>,
        [&](entt::entity, ECSComponent::SpotLight& spot, ECSComponent::Transform& transform)
        {
            double dist = glm::length(camPos - transform.Position);
            if (dist > static_cast<double>(spot.Range) * 4.0) {
                spot.Enabled = false;
                return;
            }
            if (spot.Range > 0.0f)
                spot.Enabled = true;

            if (!spot.Enabled) return;

            light_array::free_light_record rec;
            rec.position     = transform.Position;
            rec.direction    = glm::vec3(0.f, -1.f, 0.f); // default downward; rotation support can be added later
            rec.color        = spot.Color;
            rec.intensity    = spot.Intensity;
            rec.range        = spot.Range;
            rec.inner_cutoff = glm::cos(glm::radians(spot.InnerAngle));
            rec.outer_cutoff = glm::cos(glm::radians(spot.OuterAngle));
            simulation::Lights.free_lights.push_back(rec);
        });
}

void LightSystem::Render(ECWorld& world)
{
    // Rendering of ECS lights is handled by the main renderer pipeline,
    // which queries SpotLight/AreaLight/SunLight components via World views.
    // Nothing to do here for now.
}
