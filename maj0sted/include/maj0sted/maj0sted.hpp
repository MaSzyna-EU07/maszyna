#pragma once

// maj0sted — a Domain-Driven Design library for a railway track-layout editor
// ("edytor torowiska"). Include this single header to pull in the whole public
// domain model.

#include "maj0sted/version.hpp"

// Value Objects
#include "maj0sted/domain/value_objects/azimuth.hpp"
#include "maj0sted/domain/value_objects/cartesian_position.hpp"
#include "maj0sted/domain/value_objects/crs.hpp"
#include "maj0sted/domain/value_objects/length.hpp"
#include "maj0sted/domain/value_objects/niweleta_id.hpp"
#include "maj0sted/domain/value_objects/radius.hpp"
#include "maj0sted/domain/value_objects/straight_id.hpp"
#include "maj0sted/domain/value_objects/track_offset.hpp"

// Geometry — krzywe w planie i w profilu
#include "maj0sted/domain/geometry/horizontal_alignment.hpp"
#include "maj0sted/domain/geometry/plan_element.hpp"
#include "maj0sted/domain/geometry/profile_element.hpp"
#include "maj0sted/domain/geometry/vertical_alignment.hpp"

// Aggregates
#include "maj0sted/domain/map_project.hpp"
#include "maj0sted/domain/niweleta.hpp"

// Parallelism — równoległość torów
#include "maj0sted/domain/parallelism/parallelism_service.hpp"
#include "maj0sted/domain/parallelism/track_parallelism.hpp"

// Fitting — dopasowanie krzywej między prostymi
#include "maj0sted/domain/fitting/fit_parameters.hpp"
#include "maj0sted/domain/fitting/fit_result.hpp"
#include "maj0sted/domain/fitting/fitting_service.hpp"

// IO — serializacja projektu
#include "maj0sted/io/project_serializer.hpp"
