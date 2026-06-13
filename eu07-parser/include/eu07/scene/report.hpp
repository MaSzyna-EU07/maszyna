#pragma once

#include <eu07/scene/document.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/types.hpp>

#include <filesystem>
#include <fstream>
#include <ostream>
#include <stdexcept>
#include <string_view>

namespace eu07::scene {

namespace detail {

inline void writeHeader(std::ostream& out, const NodeHeader& header) {
    out << "L" << (header.line + 1) << " range_max=" << header.rangeMax
        << " range_min=" << header.rangeMin << " name=" << header.name;
}

inline void writeVec3(std::ostream& out, const Vec3& v) {
    out << v.x << ' ' << v.y << ' ' << v.z;
}

template <typename Node>
inline void writeNodeList(std::ostream& out, const std::string_view section, const std::vector<Node>& nodes) {
    out << "\n[" << section << "] count=" << nodes.size() << '\n';
}

} // namespace detail

inline void writeSceneReport(const std::filesystem::path& outPath, const SceneDocument& doc) {
    std::ofstream out(outPath);
    if (!out) {
        throw std::runtime_error("Nie mozna zapisac: " + outPath.string());
    }

    out << "# raport scenariusza — sparsowane struktury node + tokeny pozostalych dyrektyw\n\n";
    out << "[podsumowanie] node=" << countNodeInstances(doc) << " razem=" << countDirectiveInstances(doc)
        << " nieznane=" << doc.unknown.size() << " include=" << doc.include.size() << '\n';

    detail::writeNodeList(out, "include", doc.include);
    for (const ParsedInclude& inc : doc.include) {
        out << "L" << (inc.line + 1) << " file=" << inc.file;
        if (!inc.parameters.empty()) {
            out << " params=" << inc.parameters.size();
        }
        out << " expanded=" << (inc.expanded ? "yes" : "no");
        if (inc.error) {
            out << " error=" << *inc.error;
        }
        out << '\n';
    }

    detail::writeNodeList(out, "node_dynamic", doc.nodeDynamic);
    for (const ParsedNodeDynamic& node : doc.nodeDynamic) {
        detail::writeHeader(out, node.header);
        out << " folder=" << node.datafolder << " skin=" << node.skinfile << " mmd=" << node.mmdfile
            << " track=" << node.pathname << " driver=" << node.driverType
            << " coupling=" << node.coupling.raw;
        if (node.inTrainset) {
            out << " trainset";
        }
        if (node.destination) {
            out << " dest=" << *node.destination;
        }
        out << '\n';
    }

    detail::writeNodeList(out, "node_model", doc.nodeModel);
    for (const ParsedNodeModel& node : doc.nodeModel) {
        detail::writeHeader(out, node.header);
        out << " pos=";
        detail::writeVec3(out, node.location);
        out << " rotY=" << node.rotationY << " model=" << node.modelPath << '\n';
    }

    detail::writeNodeList(out, "node_track_normal", doc.nodeTrackNormal);
    for (const ParsedTrackNormal& node : doc.nodeTrackNormal) {
        detail::writeHeader(out, node.header);
        out << " kind=" << node.trajectoryKind << " gauge=" << node.gauge << " env=" << node.environment
            << " beziers=" << node.beziers.size() << '\n';
    }

    detail::writeNodeList(out, "node_track_switch", doc.nodeTrackSwitch);
    for (const ParsedTrackSwitch& node : doc.nodeTrackSwitch) {
        detail::writeHeader(out, node.header);
        out << " gauge=" << node.gauge << " env=" << node.environment
            << " beziers=" << node.beziers.size() << '\n';
    }

    detail::writeNodeList(out, "node_track_road", doc.nodeTrackRoad);
    for (const ParsedTrackRoad& node : doc.nodeTrackRoad) {
        detail::writeHeader(out, node.header);
        out << " width=" << node.roadWidth << " env=" << node.environment
            << " beziers=" << node.beziers.size() << '\n';
    }

    detail::writeNodeList(out, "node_track_cross", doc.nodeTrackCross);
    for (const ParsedTrackCross& node : doc.nodeTrackCross) {
        detail::writeHeader(out, node.header);
        out << " width=" << node.roadWidth << " env=" << node.environment
            << " beziers=" << node.beziers.size() << '\n';
    }

    detail::writeNodeList(out, "node_track_other", doc.nodeTrackOther);
    for (const ParsedTrackOther& node : doc.nodeTrackOther) {
        detail::writeHeader(out, node.header);
        out << " kind=" << node.trajectoryKind << " width=" << node.width << " env=" << node.environment
            << " beziers=" << node.beziers.size() << '\n';
    }

    detail::writeNodeList(out, "node_traction", doc.nodeTraction);
    for (const ParsedNodeTraction& node : doc.nodeTraction) {
        detail::writeHeader(out, node.header);
        out << " supply=" << node.powerSourceName << " U=" << node.nominalVoltage << '\n';
    }

    detail::writeNodeList(out, "node_traction_power", doc.nodeTractionPower);
    for (const ParsedNodeTractionPowerSource& node : doc.nodeTractionPower) {
        detail::writeHeader(out, node.header);
        out << " U=" << node.nominalVoltage << " Imax=" << node.maxOutputCurrent;
        if (node.recuperation) {
            out << " recuperation";
        }
        if (node.section) {
            out << " section";
        }
        out << '\n';
    }

    detail::writeNodeList(out, "node_triangles", doc.nodeTriangles);
    for (const ParsedNodeTriangles& node : doc.nodeTriangles) {
        detail::writeHeader(out, node.header);
        out << " tex=" << node.texture << " vertices=" << node.vertices.size() << '\n';
    }

    detail::writeNodeList(out, "node_triangle_strip", doc.nodeTriangleStrip);
    for (const ParsedNodeTriangleStrip& node : doc.nodeTriangleStrip) {
        detail::writeHeader(out, node.header);
        out << " tex=" << node.texture << " vertices=" << node.vertices.size() << '\n';
    }

    detail::writeNodeList(out, "node_triangle_fan", doc.nodeTriangleFan);
    for (const ParsedNodeTriangleFan& node : doc.nodeTriangleFan) {
        detail::writeHeader(out, node.header);
        out << " tex=" << node.texture << " vertices=" << node.vertices.size() << '\n';
    }

    detail::writeNodeList(out, "node_lines", doc.nodeLines);
    for (const ParsedNodeLines& node : doc.nodeLines) {
        detail::writeHeader(out, node.header);
        out << " segments=" << node.segments.size() << '\n';
    }

    detail::writeNodeList(out, "node_line_strip", doc.nodeLineStrip);
    for (const ParsedNodeLineStrip& node : doc.nodeLineStrip) {
        detail::writeHeader(out, node.header);
        out << " points=" << node.points.size() << '\n';
    }

    detail::writeNodeList(out, "node_line_loop", doc.nodeLineLoop);
    for (const ParsedNodeLineLoop& node : doc.nodeLineLoop) {
        detail::writeHeader(out, node.header);
        out << " points=" << node.points.size() << '\n';
    }

    detail::writeNodeList(out, "node_memcell", doc.nodeMemcell);
    for (const ParsedNodeMemcell& node : doc.nodeMemcell) {
        detail::writeHeader(out, node.header);
        out << " cmd=" << node.command;
        if (node.trackName) {
            out << " track=" << *node.trackName;
        }
        out << '\n';
    }

    detail::writeNodeList(out, "node_eventlauncher", doc.nodeEventlauncher);
    for (const ParsedNodeEventlauncher& node : doc.nodeEventlauncher) {
        detail::writeHeader(out, node.header);
        out << " key=" << node.key << " event1=" << node.event1;
        if (node.event2) {
            out << " event2=" << *node.event2;
        }
        if (node.condition) {
            out << " condition";
        }
        if (node.trainTriggered) {
            out << " traintriggered";
        }
        out << '\n';
    }

    detail::writeNodeList(out, "node_sound", doc.nodeSound);
    for (const ParsedNodeSound& node : doc.nodeSound) {
        detail::writeHeader(out, node.header);
        out << " wav=" << node.wavFile << '\n';
    }
}

} // namespace eu07::scene
