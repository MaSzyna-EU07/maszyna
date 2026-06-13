#pragma once

// Payload per typ `node` — odpowiedniki klas w maszyna-fresh.

#include <eu07/scene/runtime/basic_node.hpp>
#include <eu07/scene/runtime/types.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace eu07::scene::runtime {

// --- model → TAnimModel (model/AnimModel.h) ---

struct RuntimeModelInstance {
    BasicNode node;
    Vec3 location;
    Vec3 angles;
    Vec3 scale{1.0, 1.0, 1.0};
    std::string modelFile;
    std::string textureFile;
    std::vector<float> lightStates;
    std::vector<std::uint32_t> lightColors;
    Vec3 inlineScale{1.0, 1.0, 1.0};
    bool transition = true;
    bool isTerrain = false;
};

// --- triangles / triangle_strip / triangle_fan → shape_node ---

struct RuntimeShapeNode {
    BasicNode node;
    bool translucent = false;
    std::string materialPath;
    LightingData lighting;
    Vec3 origin;
    std::vector<WorldVertex> vertices;
};

// --- lines / line_strip / line_loop → lines_node ---

struct RuntimeLinesNode {
    BasicNode node;
    LightingData lighting;
    float lineWidth = 1.f;
    Vec3 origin;
    std::vector<WorldVertex> vertices;
};

// --- track → TTrack (world/Track.h) ---

enum class TrackType : std::uint8_t {
    Unknown,
    Normal,
    Switch,
    Table,
    Cross,
    Tributary,
};

enum class TrackCategory : std::uint8_t {
    Rail = 1,
    Road = 2,
    Water = 4,
};

enum class TrackEnvironment : std::int8_t {
    Unknown = -1,
    Flat = 0,
    Mountains,
    Canyon,
    Tunnel,
    Bridge,
    Bank,
};

struct TrackVisibility {
    std::string material1;
    float texLength = 4.f;
    std::string material2;
    float texHeight1 = 0.f;
    float texWidth = 0.f;
    float texSlope = 0.f;
};

struct RuntimeTrack {
    BasicNode node;
    TrackType trackType = TrackType::Unknown;
    TrackCategory category = TrackCategory::Rail;
    float length = 0.f;
    float trackWidth = 0.f;
    float friction = 0.f;
    float soundDistance = 0.f;
    int qualityFlag = 0;
    int damageFlag = 0;
    TrackEnvironment environment = TrackEnvironment::Unknown;
    std::optional<TrackVisibility> visibility;
    std::vector<SegmentPath> paths;
    // tail TTrack::Load (event0, fouling, velocity, …) — rozszerzalne pozniej
    std::vector<std::pair<std::string, std::string>> tailKeywords;
};

// --- traction → TTraction (world/Traction.h) ---

enum class TractionWireMaterial : std::uint8_t {
    None = 0,
    Copper = 1,
    Aluminium = 2,
};

struct RuntimeTraction {
    BasicNode node;
    std::string powerSupplyName;
    float nominalVoltage = 0.f;
    float maxCurrent = 0.f;
    float resistivityOhmPerM = 0.f;
    double resistivityLegacy = 0.0;
    std::string materialRaw = "cu";
    TractionWireMaterial material = TractionWireMaterial::Copper;
    float wireThickness = 0.f;
    int damageFlag = 0;
    Vec3 wireP1;
    Vec3 wireP2;
    Vec3 wireP3;
    Vec3 wireP4;
    double minHeight = 0.0;
    double segmentLength = 0.0;
    int wireCount = 0;
    float wireOffset = 0.f;
    std::optional<std::string> parallelName;
};

// --- tractionpowersource → TTractionPowerSource (world/TractionPower.cpp) ---

enum class PowerSourceModifier : std::uint8_t {
    None,
    Recuperation,
    Section,
};

struct RuntimeTractionPowerSource {
    BasicNode node;
    Vec3 position{};
    float nominalVoltage = 0.f;
    float voltageFrequency = 0.f;
    double internalResistanceLegacy = 0.2;
    float internalResistance = 0.2f;
    float maxOutputCurrent = 0.f;
    float fastFuseTimeout = 0.f;
    float fastFuseRepetition = 0.f;
    float slowFuseTimeout = 0.f;
    PowerSourceModifier modifier = PowerSourceModifier::None;
};

// --- memcell → TMemCell (world/MemCell.h) ---

struct RuntimeMemCell {
    BasicNode node;
    std::string text;
    double value1 = 0.0;
    double value2 = 0.0;
    std::optional<std::string> trackName;
};

// --- eventlauncher → TEventLauncher (world/EvLaunch.h) ---

struct EventLauncherCondition {
    std::string memcellName;
    std::string compareText;
    double compareValue1 = 0.0;
    double compareValue2 = 0.0;
    int checkMask = 0;
};

struct RuntimeEventLauncher {
    BasicNode node;
    Vec3 location;
    double radiusSquared = 0.0;
    std::string activationKeyRaw;
    int activationKey = 0;
    double deltaTime = -1.0;
    std::string event1Name;
    std::string event2Name;
    std::optional<EventLauncherCondition> condition;
    bool trainTriggered = false;
    int launchHour = -1;
    int launchMinute = -1;
};

// --- dynamic → TDynamicObject (vehicle/DynObj) ---

struct RuntimeDynamicObject {
    BasicNode node;
    std::string dataFolder;
    std::string skinFile;
    std::string mmdFile;
    std::string trackName;
    double offset = -1.0;
    std::string driverType;
    int coupling = 3;
    std::string couplingRaw = "3";
    std::string couplingParams;
    float velocity = 0.f;
    int loadCount = 0;
    std::string loadType;
    std::optional<std::string> destination;
    std::optional<std::size_t> trainsetIndex;
};

// --- sound → sound_source (audio/sound.h) ---

struct RuntimeSoundSource {
    BasicNode node;
    Vec3 location;
    std::string wavFile;
};

} // namespace eu07::scene::runtime
