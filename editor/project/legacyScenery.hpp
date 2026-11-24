/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutAnchors.hpp"
#include "editor/layout/layoutLegacy.hpp"
#include "editor/project/sceneryText.hpp"

#include <filesystem>
#include <string>
#include <vector>

// The old tracks and the lineside objects of a scenery read from the text of its files (stage E6, editor-tracks-domain.md §10), with no
// scene, as the simulator places them: the tracks and the wires moved by the origin in effect, the models, the memory cells and the event
// launchers placed by the origin, the rotation and the scale. For the takeover of the old tracks and the anchors of the objects; the files
// aren't written.
namespace project
{

struct legacy_scenery
{
	std::vector<layout::legacy_track> tracks; // the tracks of the railway: normal and switch
	std::vector<layout::lineside_object> objects; // the models, the wires, the memory cells, the event launchers
	std::vector<std::string> problems; // entries which can't be read
};

// of the statements of the scenery: Map only, those of the files of the map
legacy_scenery read_legacy(scenery_text::scenery const &Scenery, bool const Map = false);
// the scenery of the file Root, relative to the folder, with its includes
legacy_scenery read_legacy(std::filesystem::path const &Folder, std::string const &Root);

} // namespace project
