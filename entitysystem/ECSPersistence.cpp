#include "stdafx.h"
#include "ECSPersistence.h"

#include "entitysystem/ECWorld.h"
#include "entitysystem/components/BasicComponents.h"
#include "entitysystem/components/RenderComponents.h"
#include "utilities/Logs.h"
#include "nlohmann/json.hpp"
#include <fstream>

namespace ecs_persistence
{

int save(ECWorld &world, const std::string &path)
{
    nlohmann::json root = nlohmann::json::array();

    for (auto entity : world.GetEntities()) {
        auto *idComp = world.GetComponent<ECSComponent::Identification>(entity);
        if (!idComp) continue;

        nlohmann::json ent;
        ent["name"] = idComp->Name.ToString();

        if (auto *t = world.GetComponent<ECSComponent::Transform>(entity)) {
            ent["Transform"] = {
                {"px", t->Position.x}, {"py", t->Position.y}, {"pz", t->Position.z},
                {"rx", t->Rotation.x}, {"ry", t->Rotation.y}, {"rz", t->Rotation.z}, {"rw", t->Rotation.w},
                {"sx", t->Scale.x},    {"sy", t->Scale.y},    {"sz", t->Scale.z}
            };
        }
        if (auto *s = world.GetComponent<ECSComponent::SpotLight>(entity)) {
            ent["SpotLight"] = {
                {"r", s->Color.x}, {"g", s->Color.y}, {"b", s->Color.z},
                {"intensity", s->Intensity}, {"range", s->Range},
                {"innerAngle", s->InnerAngle}, {"outerAngle", s->OuterAngle},
                {"enabled", s->Enabled}, {"castShadows", s->CastShadows}
            };
        }
        if (auto *p = world.GetComponent<ECSComponent::ParticleEmitter>(entity)) {
            ent["ParticleEmitter"] = {
                {"active", p->isActive}, {"spawnRate", p->spawnRate},
                {"lifetime", p->particleLifetime}, {"airResistance", p->airResistance},
                {"gx", p->gravity.x}, {"gy", p->gravity.y}, {"gz", p->gravity.z},
                {"fadex", p->colorFade.x}, {"fadey", p->colorFade.y},
                {"fadez", p->colorFade.z}, {"fadew", p->colorFade.w}
            };
        }
        if (auto *b = world.GetComponent<ECSComponent::Billboard>(entity)) {
            ent["Billboard"] = {
                {"texture", b->texturePath.ToString()},
                {"size", b->size}, {"active", b->active}
            };
        }
        if (auto *l = world.GetComponent<ECSComponent::Line>(entity)) {
            ent["Line"] = {
                {"sx", l->start.x}, {"sy", l->start.y}, {"sz", l->start.z},
                {"ex", l->end.x},   {"ey", l->end.y},   {"ez", l->end.z},
                {"r", l->color.x},  {"g", l->color.y},  {"b", l->color.z},
                {"thickness", l->thickness}, {"active", l->active}
            };
        }
        if (auto *lod = world.GetComponent<ECSComponent::LODController>(entity)) {
            ent["LODController"] = {{"min", lod->RangeMin}, {"max", lod->RangeMax}};
        }
        if (world.HasComponent<ECSComponent::Disabled>(entity))
            ent["disabled"] = true;

        // skip entities that are purely runtime-managed (VehicleRef, MeshRenderer linked to legacy)
        bool runtimeOnly =
            world.HasComponent<ECSComponent::VehicleRef>(entity) ||
            (world.HasComponent<ECSComponent::MeshRenderer>(entity) &&
             world.GetComponent<ECSComponent::MeshRenderer>(entity)->modelInstance != nullptr);
        if (runtimeOnly) continue;

        root.push_back(ent);
    }

    std::ofstream f(path);
    if (!f) {
        WriteLog("[ECS] ECSPersistence: cannot write to " + path);
        return -1;
    }
    f << root.dump(2);
    WriteLog("[ECS] ECSPersistence: saved " + std::to_string(root.size()) + " entities to " + path);
    return static_cast<int>(root.size());
}

int load(ECWorld &world, const std::string &path)
{
    std::ifstream f(path);
    if (!f) return -1; // file doesn't exist — silently skip

    nlohmann::json root;
    try { root = nlohmann::json::parse(f); }
    catch (const std::exception &e) {
        WriteLog("[ECS] ECSPersistence: JSON parse error in " + path + ": " + e.what());
        return -1;
    }

    int created = 0;
    for (auto const &ent : root) {
        if (!ent.contains("name")) continue;
        std::string name = ent["name"].get<std::string>();
        if (world.FindEntityByName(name) != entt::null) continue;

        auto entity = world.CreateEntity();
        auto &id = world.AddComponent<ECSComponent::Identification>(entity);
        id.Name = name;

        if (ent.contains("Transform")) {
            auto &j = ent["Transform"];
            auto &t = world.AddComponent<ECSComponent::Transform>(entity);
            t.Position = glm::dvec3(double(j["px"]), double(j["py"]), double(j["pz"]));
            t.Rotation = glm::quat(float(j["rw"]), float(j["rx"]), float(j["ry"]), float(j["rz"]));
            t.Scale    = glm::vec3(float(j["sx"]), float(j["sy"]), float(j["sz"]));
        }
        if (ent.contains("SpotLight")) {
            auto &j = ent["SpotLight"];
            auto &s = world.AddComponent<ECSComponent::SpotLight>(entity);
            s.Color      = glm::vec3(float(j["r"]), float(j["g"]), float(j["b"]));
            s.Intensity  = j["intensity"];
            s.Range      = j["range"];
            s.InnerAngle = j["innerAngle"];
            s.OuterAngle = j["outerAngle"];
            s.Enabled    = j["enabled"];
            s.CastShadows = j["castShadows"];
        }
        if (ent.contains("ParticleEmitter")) {
            auto &j = ent["ParticleEmitter"];
            auto &p = world.AddComponent<ECSComponent::ParticleEmitter>(entity);
            p.isActive         = j["active"];
            p.spawnRate        = j["spawnRate"];
            p.particleLifetime = j["lifetime"];
            p.airResistance    = j["airResistance"];
            p.gravity          = glm::vec3(float(j["gx"]), float(j["gy"]), float(j["gz"]));
            p.colorFade        = glm::vec4(float(j["fadex"]), float(j["fadey"]),
                                           float(j["fadez"]), float(j["fadew"]));
        }
        if (ent.contains("Billboard")) {
            auto &j = ent["Billboard"];
            auto &b = world.AddComponent<ECSComponent::Billboard>(entity);
            b.texturePath = FName(j["texture"].get<std::string>());
            b.size        = j["size"];
            b.active      = j["active"];
        }
        if (ent.contains("Line")) {
            auto &j = ent["Line"];
            auto &l = world.AddComponent<ECSComponent::Line>(entity);
            l.start     = glm::vec3(float(j["sx"]), float(j["sy"]), float(j["sz"]));
            l.end       = glm::vec3(float(j["ex"]), float(j["ey"]), float(j["ez"]));
            l.color     = glm::vec3(float(j["r"]),  float(j["g"]),  float(j["b"]));
            l.thickness = j["thickness"];
            l.active    = j["active"];
        }
        if (ent.contains("LODController")) {
            auto &j = ent["LODController"];
            auto &lod = world.AddComponent<ECSComponent::LODController>(entity);
            lod.RangeMin = j["min"];
            lod.RangeMax = j["max"];
        }
        if (ent.value("disabled", false))
            world.AddComponent<ECSComponent::Disabled>(entity);

        ++created;
    }

    WriteLog("[ECS] ECSPersistence: loaded " + std::to_string(created) + " entities from " + path);
    return created;
}

} // namespace ecs_persistence
