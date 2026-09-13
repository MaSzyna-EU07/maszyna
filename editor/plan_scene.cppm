/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
export module eu07.editor.plan_scene;

export {

namespace editor::plan::render {

/// A point in the project's CRS (metres for EPSG:2180).
struct Point {
    double x;
    double y;
};

/// Which kind of plan element a polyline came from (lets a GUI colour it).
enum class ElementKind { Straight, Arc, Transition };

}  // namespace editor::plan::render

}  // export
