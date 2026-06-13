#pragma once

// EU7B v2 — chunki TRAC, PWRS, MEMC, LAUN, DYNM, SOND (include wewnątrz binary::detail).

#include <eu07/scene/binary/io.hpp>
#include <eu07/scene/binary/runtime_codec.hpp>
#include <eu07/scene/binary/string_table.hpp>
#include <eu07/scene/bake/module.hpp>

#include <istream>
#include <ostream>
#include <sstream>
#include <vector>

inline void writeRuntimeTraction(
    std::ostream& out,
    StringTable& table,
    const runtime::RuntimeTraction& traction) {
    codec::writeSlimNode(out, table, traction.node, "traction");
    codec::writeStringId(out, table, traction.powerSupplyName);
    io::writeU8(out, static_cast<std::uint8_t>(traction.material));
    io::writeF32(out, traction.nominalVoltage);
    io::writeF32(out, traction.maxCurrent);
    io::writeF32(out, traction.resistivityOhmPerM);
    io::writeF64(out, traction.resistivityLegacy);
    codec::writeStringId(out, table, traction.materialRaw);
    io::writeF32(out, traction.wireThickness);
    io::writeI32(out, traction.damageFlag);
    io::writeVec3(out, traction.wireP1);
    io::writeVec3(out, traction.wireP2);
    io::writeVec3(out, traction.wireP3);
    io::writeVec3(out, traction.wireP4);
    io::writeF64(out, traction.minHeight);
    io::writeF64(out, traction.segmentLength);
    io::writeI32(out, traction.wireCount);
    io::writeF32(out, traction.wireOffset);
    io::writeU8(out, traction.parallelName.has_value() ? 1 : 0);
    if (traction.parallelName) {
        codec::writeStringId(out, table, *traction.parallelName);
    }
}

inline void writeRuntimeMemcell(
    std::ostream& out,
    StringTable& table,
    const runtime::RuntimeMemCell& cell) {
    codec::writeSlimNode(out, table, cell.node, "memcell");
    codec::writeStringId(out, table, cell.text);
    io::writeF64(out, cell.value1);
    io::writeF64(out, cell.value2);
    codec::writeStringId(out, table, cell.trackName.value_or(""));
}

inline void writeRuntimeLauncher(
    std::ostream& out,
    StringTable& table,
    const runtime::RuntimeEventLauncher& launcher) {
    codec::writeSlimNode(out, table, launcher.node, "eventlauncher");
    io::writeVec3(out, launcher.location);
    io::writeF64(out, launcher.radiusSquared);
    codec::writeStringId(out, table, launcher.activationKeyRaw);
    io::writeI32(out, launcher.activationKey);
    io::writeF64(out, launcher.deltaTime);
    codec::writeStringId(out, table, launcher.event1Name);
    codec::writeStringId(out, table, launcher.event2Name);
    io::writeI32(out, launcher.launchHour);
    io::writeI32(out, launcher.launchMinute);
    io::writeU8(out, launcher.condition.has_value() ? 1 : 0);
    if (launcher.condition) {
        codec::writeStringId(out, table, launcher.condition->memcellName);
        codec::writeStringId(out, table, launcher.condition->compareText);
        io::writeF64(out, launcher.condition->compareValue1);
        io::writeF64(out, launcher.condition->compareValue2);
        io::writeI32(out, launcher.condition->checkMask);
    }
    io::writeU8(out, launcher.trainTriggered ? 1 : 0);
}

inline void writeRuntimeDynamic(
    std::ostream& out,
    StringTable& table,
    const runtime::RuntimeDynamicObject& vehicle) {
    codec::writeSlimNode(out, table, vehicle.node, "dynamic");
    codec::writeStringId(out, table, vehicle.dataFolder);
    codec::writeStringId(out, table, vehicle.skinFile);
    codec::writeStringId(out, table, vehicle.mmdFile);
    codec::writeStringId(out, table, vehicle.trackName);
    codec::writeStringId(out, table, vehicle.driverType);
    codec::writeStringId(out, table, vehicle.loadType);
    codec::writeStringId(out, table, vehicle.couplingParams);
    codec::writeStringId(out, table, vehicle.couplingRaw);
    io::writeF64(out, vehicle.offset);
    io::writeI32(out, vehicle.coupling);
    io::writeI32(out, vehicle.loadCount);
    io::writeF32(out, vehicle.velocity);
    io::writeU8(out, vehicle.destination.has_value() ? 1 : 0);
    if (vehicle.destination) {
        codec::writeStringId(out, table, *vehicle.destination);
    }
    io::writeU8(out, vehicle.trainsetIndex.has_value() ? 1 : 0);
    if (vehicle.trainsetIndex) {
        io::writeU32(out, static_cast<std::uint32_t>(*vehicle.trainsetIndex));
    }
}

inline void writeRuntimeSound(
    std::ostream& out,
    StringTable& table,
    const runtime::RuntimeSoundSource& sound) {
    codec::writeSlimNode(out, table, sound.node, "sound");
    io::writeVec3(out, sound.location);
    codec::writeStringId(out, table, sound.wavFile);
}

inline void collectTractionStrings(StringTable& table, const runtime::RuntimeTraction& traction) {
    table.intern(traction.node.name);
    table.intern(traction.powerSupplyName);
    table.intern(traction.materialRaw);
    if (traction.parallelName) {
        table.intern(*traction.parallelName);
    }
}

inline void collectMemcellStrings(StringTable& table, const runtime::RuntimeMemCell& cell) {
    table.intern(cell.node.name);
    table.intern(cell.text);
    if (cell.trackName) {
        table.intern(*cell.trackName);
    }
}

inline void collectLauncherStrings(StringTable& table, const runtime::RuntimeEventLauncher& launcher) {
    table.intern(launcher.node.name);
    table.intern(launcher.activationKeyRaw);
    table.intern(launcher.event1Name);
    table.intern(launcher.event2Name);
    if (launcher.condition) {
        table.intern(launcher.condition->memcellName);
        table.intern(launcher.condition->compareText);
    }
}

inline void collectDynamicStrings(StringTable& table, const runtime::RuntimeDynamicObject& vehicle) {
    table.intern(vehicle.node.name);
    table.intern(vehicle.dataFolder);
    table.intern(vehicle.skinFile);
    table.intern(vehicle.mmdFile);
    table.intern(vehicle.trackName);
    table.intern(vehicle.driverType);
    table.intern(vehicle.loadType);
    table.intern(vehicle.couplingParams);
    table.intern(vehicle.couplingRaw);
    if (vehicle.destination) {
        table.intern(*vehicle.destination);
    }
}

inline void collectSoundStrings(StringTable& table, const runtime::RuntimeSoundSource& sound) {
    table.intern(sound.node.name);
    table.intern(sound.wavFile);
}

[[nodiscard]] inline std::vector<char> buildTracPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeTraction>& traction) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(traction.size()));
    for (const runtime::RuntimeTraction& item : traction) {
        writeRuntimeTraction(out, table, item);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildMemcPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeMemCell>& cells) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(cells.size()));
    for (const runtime::RuntimeMemCell& cell : cells) {
        writeRuntimeMemcell(out, table, cell);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildLaunPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeEventLauncher>& launchers) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(launchers.size()));
    for (const runtime::RuntimeEventLauncher& launcher : launchers) {
        writeRuntimeLauncher(out, table, launcher);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildDynmPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeDynamicObject>& dynamics) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(dynamics.size()));
    for (const runtime::RuntimeDynamicObject& vehicle : dynamics) {
        writeRuntimeDynamic(out, table, vehicle);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildSondPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeSoundSource>& sounds) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(sounds.size()));
    for (const runtime::RuntimeSoundSource& sound : sounds) {
        writeRuntimeSound(out, table, sound);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

inline runtime::RuntimeTraction readRuntimeTraction(std::istream& in, const StringTable& table) {
    runtime::RuntimeTraction traction;
    traction.node = codec::readSlimNode(in, table, "traction");
    traction.powerSupplyName = table.resolve(io::readU32(in));
    traction.material = static_cast<runtime::TractionWireMaterial>(io::readU8(in));
    traction.nominalVoltage = io::readF32(in);
    traction.maxCurrent = io::readF32(in);
    traction.resistivityOhmPerM = io::readF32(in);
    traction.resistivityLegacy = io::readF64(in);
    traction.materialRaw = table.resolve(io::readU32(in));
    traction.wireThickness = io::readF32(in);
    traction.damageFlag = io::readI32(in);
    traction.wireP1 = io::readVec3(in);
    traction.wireP2 = io::readVec3(in);
    traction.wireP3 = io::readVec3(in);
    traction.wireP4 = io::readVec3(in);
    traction.minHeight = io::readF64(in);
    traction.segmentLength = io::readF64(in);
    traction.wireCount = io::readI32(in);
    traction.wireOffset = io::readF32(in);
    if (io::readU8(in) != 0) {
        const std::string parallel = table.resolve(io::readU32(in));
        if (!parallel.empty()) {
            traction.parallelName = parallel;
        }
    }
    return traction;
}

inline runtime::RuntimeMemCell readRuntimeMemcell(std::istream& in, const StringTable& table) {
    runtime::RuntimeMemCell cell;
    cell.node = codec::readSlimNode(in, table, "memcell");
    cell.text = table.resolve(io::readU32(in));
    cell.value1 = io::readF64(in);
    cell.value2 = io::readF64(in);
    const std::string track = table.resolve(io::readU32(in));
    if (!track.empty()) {
        cell.trackName = track;
    }
    return cell;
}

inline runtime::RuntimeEventLauncher readRuntimeLauncher(std::istream& in, const StringTable& table) {
    runtime::RuntimeEventLauncher launcher;
    launcher.node = codec::readSlimNode(in, table, "eventlauncher");
    launcher.location = io::readVec3(in);
    launcher.radiusSquared = io::readF64(in);
    launcher.activationKeyRaw = table.resolve(io::readU32(in));
    launcher.activationKey = io::readI32(in);
    launcher.deltaTime = io::readF64(in);
    launcher.event1Name = table.resolve(io::readU32(in));
    launcher.event2Name = table.resolve(io::readU32(in));
    launcher.launchHour = io::readI32(in);
    launcher.launchMinute = io::readI32(in);
    if (io::readU8(in) != 0) {
        runtime::EventLauncherCondition cond;
        cond.memcellName = table.resolve(io::readU32(in));
        cond.compareText = table.resolve(io::readU32(in));
        cond.compareValue1 = io::readF64(in);
        cond.compareValue2 = io::readF64(in);
        cond.checkMask = io::readI32(in);
        launcher.condition = cond;
    }
    launcher.trainTriggered = io::readU8(in) != 0;
    return launcher;
}

inline runtime::RuntimeDynamicObject readRuntimeDynamic(std::istream& in, const StringTable& table) {
    runtime::RuntimeDynamicObject vehicle;
    vehicle.node = codec::readSlimNode(in, table, "dynamic");
    vehicle.dataFolder = table.resolve(io::readU32(in));
    vehicle.skinFile = table.resolve(io::readU32(in));
    vehicle.mmdFile = table.resolve(io::readU32(in));
    vehicle.trackName = table.resolve(io::readU32(in));
    vehicle.driverType = table.resolve(io::readU32(in));
    vehicle.loadType = table.resolve(io::readU32(in));
    vehicle.couplingParams = table.resolve(io::readU32(in));
    vehicle.couplingRaw = table.resolve(io::readU32(in));
    vehicle.offset = io::readF64(in);
    vehicle.coupling = io::readI32(in);
    if (vehicle.couplingRaw.empty()) {
        if (!vehicle.couplingParams.empty()) {
            vehicle.couplingRaw = std::to_string(vehicle.coupling) + "." + vehicle.couplingParams;
        } else {
            vehicle.couplingRaw = std::to_string(vehicle.coupling);
        }
    }
    vehicle.loadCount = io::readI32(in);
    vehicle.velocity = io::readF32(in);
    if (io::readU8(in) != 0) {
        vehicle.destination = table.resolve(io::readU32(in));
    }
    if (io::readU8(in) != 0) {
        vehicle.trainsetIndex = io::readU32(in);
    }
    return vehicle;
}

inline runtime::RuntimeSoundSource readRuntimeSound(std::istream& in, const StringTable& table) {
    runtime::RuntimeSoundSource sound;
    sound.node = codec::readSlimNode(in, table, "sound");
    sound.location = io::readVec3(in);
    sound.wavFile = table.resolve(io::readU32(in));
    return sound;
}

inline void readTracChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.traction.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.traction.push_back(readRuntimeTraction(in, table));
    }
}

inline void readMemcChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.memcells.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.memcells.push_back(readRuntimeMemcell(in, table));
    }
}

inline void readLaunChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.eventLaunchers.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.eventLaunchers.push_back(readRuntimeLauncher(in, table));
    }
}

inline void readDynmChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.dynamics.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.dynamics.push_back(readRuntimeDynamic(in, table));
    }
}

inline void readSondChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.sounds.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.sounds.push_back(readRuntimeSound(in, table));
    }
}

inline void writeRuntimePowerSource(
    std::ostream& out,
    StringTable& table,
    const runtime::RuntimeTractionPowerSource& source) {
    codec::writeSlimNode(out, table, source.node, "tractionpowersource");
    io::writeVec3(out, source.position);
    io::writeF32(out, source.nominalVoltage);
    io::writeF32(out, source.voltageFrequency);
    io::writeF64(out, source.internalResistanceLegacy);
    io::writeF32(out, source.internalResistance);
    io::writeF32(out, source.maxOutputCurrent);
    io::writeF32(out, source.fastFuseTimeout);
    io::writeF32(out, source.fastFuseRepetition);
    io::writeF32(out, source.slowFuseTimeout);
    io::writeU8(out, static_cast<std::uint8_t>(source.modifier));
}

inline void collectPowerSourceStrings(
    StringTable& table,
    const runtime::RuntimeTractionPowerSource& source) {
    table.intern(source.node.name);
}

[[nodiscard]] inline std::vector<char> buildPwrsPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeTractionPowerSource>& sources) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(sources.size()));
    for (const runtime::RuntimeTractionPowerSource& source : sources) {
        writeRuntimePowerSource(out, table, source);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

inline runtime::RuntimeTractionPowerSource readRuntimePowerSource(
    std::istream& in,
    const StringTable& table) {
    runtime::RuntimeTractionPowerSource source;
    source.node = codec::readSlimNode(in, table, "tractionpowersource");
    source.position = io::readVec3(in);
    source.node.area.center = source.position;
    source.nominalVoltage = io::readF32(in);
    source.voltageFrequency = io::readF32(in);
    source.internalResistanceLegacy = io::readF64(in);
    source.internalResistance = io::readF32(in);
    source.maxOutputCurrent = io::readF32(in);
    source.fastFuseTimeout = io::readF32(in);
    source.fastFuseRepetition = io::readF32(in);
    source.slowFuseTimeout = io::readF32(in);
    source.modifier = static_cast<runtime::PowerSourceModifier>(io::readU8(in));
    return source;
}

inline void readPwrsChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.powerSources.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.powerSources.push_back(readRuntimePowerSource(in, table));
    }
}
