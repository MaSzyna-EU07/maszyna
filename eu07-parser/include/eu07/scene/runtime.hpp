#pragma once

// Kontrakt runtime MaSzyny — structy docelowe po state_serializer.
//
// Zrodla w maszyna-fresh:
//   simulation/simulationstateserializer.cpp  (deserialize_*, transform)
//   scene/scenenode.h / scenenode.cpp         (shape_node, lines_node, basic_node)
//   model/AnimModel.cpp                       (TAnimModel::Load)
//   world/Track.cpp, Segment.cpp              (TTrack::Load, segment_data)
//   world/Traction.cpp, TractionPower.cpp
//   world/MemCell.cpp, EvLaunch.cpp
//   world/Event.cpp                           (make_event, basic_event)
//
// Warstwa Parsed* (document.hpp) = AST z tekstu.
// Warstwa runtime::* = pola, ktore symulator trzyma po imporcie.

#include <eu07/scene/runtime/basic_node.hpp>
#include <eu07/scene/runtime/directives.hpp>
#include <eu07/scene/runtime/nodes.hpp>
#include <eu07/scene/runtime/scene.hpp>
#include <eu07/scene/runtime/scratch.hpp>
#include <eu07/scene/runtime/transform.hpp>
#include <eu07/scene/runtime/types.hpp>
