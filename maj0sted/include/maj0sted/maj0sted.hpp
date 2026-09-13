#pragma once

// maj0sted — the model behind a railway track-layout editor ("edytor
// torowiska"). One header pulls in the whole public surface.
//
// The shape of it: a Document is authored (construction lines, catalogue
// turnout types and their placements, tracks made of elements), `solve` turns it
// into a Solution, and nothing ever travels back the other way.

#include "maj0sted/version.hpp"

// The authored model, and what solving it produces.
#include "maj0sted/editor/model.hpp"
#include "maj0sted/editor/solution.hpp"

// Solving: one track's chain, and a whole document in dependency order.
#include "maj0sted/editor/chain.hpp"
#include "maj0sted/editor/layout.hpp"

// Geometry primitives shared by the solver and the renderer.
#include "maj0sted/domain/geometry/segment_layout.hpp"
#include "maj0sted/domain/geometry/turnout.hpp"

// Presentation and IO.
#include "maj0sted/editor/render.hpp"
#include "maj0sted/io/document_io.hpp"
#include "maj0sted/io/scn_export.hpp"
