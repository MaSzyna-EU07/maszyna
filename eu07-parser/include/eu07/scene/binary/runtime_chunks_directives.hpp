#pragma once

// EU7B v2 — chunki TRSET, EVNT (include wewnątrz binary::detail).

#include <eu07/scene/binary/io.hpp>
#include <eu07/scene/binary/runtime_codec.hpp>
#include <eu07/scene/binary/string_table.hpp>
#include <eu07/scene/bake/module.hpp>
#include <eu07/scene/runtime/directives.hpp>

#include <istream>
#include <ostream>
#include <sstream>
#include <vector>

inline void writeRuntimeTrainset(
    std::ostream& out,
    StringTable& table,
    const runtime::RuntimeTrainset& trainset) {
    codec::writeStringId(out, table, trainset.name);
    codec::writeStringId(out, table, trainset.track);
    io::writeF32(out, trainset.offset);
    io::writeF32(out, trainset.velocity);
    io::writeU32(out, static_cast<std::uint32_t>(trainset.assignment.size()));
    for (const auto& [key, value] : trainset.assignment) {
        codec::writeStringId(out, table, key);
        codec::writeStringId(out, table, value);
    }
    io::writeU32(out, static_cast<std::uint32_t>(trainset.vehicleIndices.size()));
    for (const std::size_t index : trainset.vehicleIndices) {
        io::writeU32(out, static_cast<std::uint32_t>(index));
    }
    io::writeU32(out, static_cast<std::uint32_t>(trainset.couplings.size()));
    for (const int coupling : trainset.couplings) {
        io::writeI32(out, coupling);
    }
    io::writeU32(out, static_cast<std::uint32_t>(trainset.driverIndex));
}

inline void writeRuntimeEvent(std::ostream& out, StringTable& table, const runtime::RuntimeEvent& event) {
    codec::writeStringId(out, table, event.name);
    io::writeU8(out, static_cast<std::uint8_t>(event.type));
    io::writeF64(out, event.delay);
    io::writeU32(out, static_cast<std::uint32_t>(event.targets.size()));
    for (const std::string& target : event.targets) {
        codec::writeStringId(out, table, target);
    }
    io::writeF64(out, event.delayRandom);
    io::writeF64(out, event.delayDeparture);
    io::writeU8(out, event.ignored ? 1 : 0);
    io::writeU8(out, event.passive ? 1 : 0);
    io::writeU32(out, static_cast<std::uint32_t>(event.payload.size()));
    for (const auto& [key, value] : event.payload) {
        codec::writeStringId(out, table, key);
        codec::writeStringId(out, table, value);
    }
}

inline void collectTrainsetStrings(StringTable& table, const runtime::RuntimeTrainset& trainset) {
    table.intern(trainset.name);
    table.intern(trainset.track);
    for (const auto& [key, value] : trainset.assignment) {
        table.intern(key);
        table.intern(value);
    }
}

inline void collectEventStrings(StringTable& table, const runtime::RuntimeEvent& event) {
    table.intern(event.name);
    for (const std::string& target : event.targets) {
        table.intern(target);
    }
    for (const auto& [key, value] : event.payload) {
        table.intern(key);
        table.intern(value);
    }
}

[[nodiscard]] inline std::vector<char> buildTrsetPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeTrainset>& trainsets) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(trainsets.size()));
    for (const runtime::RuntimeTrainset& trainset : trainsets) {
        writeRuntimeTrainset(out, table, trainset);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildEvntPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeEvent>& events) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(events.size()));
    for (const runtime::RuntimeEvent& event : events) {
        writeRuntimeEvent(out, table, event);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

inline runtime::RuntimeTrainset readRuntimeTrainset(std::istream& in, const StringTable& table) {
    runtime::RuntimeTrainset trainset;
    trainset.name = table.resolve(io::readU32(in));
    trainset.track = table.resolve(io::readU32(in));
    trainset.offset = io::readF32(in);
    trainset.velocity = io::readF32(in);
    const std::uint32_t assignmentCount = io::readU32(in);
    for (std::uint32_t i = 0; i < assignmentCount; ++i) {
        const std::string key = table.resolve(io::readU32(in));
        const std::string value = table.resolve(io::readU32(in));
        trainset.assignment.emplace(std::move(key), std::move(value));
    }
    const std::uint32_t vehicleCount = io::readU32(in);
    trainset.vehicleIndices.reserve(vehicleCount);
    for (std::uint32_t i = 0; i < vehicleCount; ++i) {
        trainset.vehicleIndices.push_back(io::readU32(in));
    }
    const std::uint32_t couplingCount = io::readU32(in);
    trainset.couplings.reserve(couplingCount);
    for (std::uint32_t i = 0; i < couplingCount; ++i) {
        trainset.couplings.push_back(io::readI32(in));
    }
    trainset.driverIndex = io::readU32(in);
    return trainset;
}

inline runtime::RuntimeEvent readRuntimeEvent(std::istream& in, const StringTable& table) {
    runtime::RuntimeEvent event;
    event.name = table.resolve(io::readU32(in));
    event.type = static_cast<runtime::EventType>(io::readU8(in));
    event.delay = io::readF64(in);
    const std::uint32_t targetCount = io::readU32(in);
    event.targets.reserve(targetCount);
    for (std::uint32_t i = 0; i < targetCount; ++i) {
        event.targets.push_back(table.resolve(io::readU32(in)));
    }
    event.delayRandom = io::readF64(in);
    event.delayDeparture = io::readF64(in);
    event.ignored = io::readU8(in) != 0;
    event.passive = io::readU8(in) != 0;
    const std::uint32_t payloadCount = io::readU32(in);
    event.payload.reserve(payloadCount);
    for (std::uint32_t i = 0; i < payloadCount; ++i) {
        event.payload.emplace_back(table.resolve(io::readU32(in)), table.resolve(io::readU32(in)));
    }
    return event;
}

inline void readTrsetChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.trainsets.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.trainsets.push_back(readRuntimeTrainset(in, table));
    }
}

inline void readEvntChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.events.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.events.push_back(readRuntimeEvent(in, table));
    }
}
