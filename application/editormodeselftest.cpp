/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "editor/editorSettings.hpp"
#include "editor/editorGeometry.hpp"
#include "application/application.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "application/editorgestures.h"
#include "imgui/imgui.h"

#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "scene/scenelayers.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Track.h"
#include "world/TrackRails.h"
#include "world/MemCell.h"
#include "world/Event.h"
#include "vehicle/DynObj.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#include "world/Sweep.h"
#include "input/keyboardinput.h"
#include "launcher/textures_scanner.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "scene/sceneterrain.h"
#include "editor/editorSpeed.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

#include "application/editormodeselftest.h"

editor_mode::selftest_result editor_mode::selftest_dispatch(std::string const &command, std::istringstream &words)
{
	for (auto const group : {&editor_mode::selftest_scene, &editor_mode::selftest_sleepers, &editor_mode::selftest_sweep, &editor_mode::selftest_track, &editor_mode::selftest_switch, &editor_mode::selftest_rails, &editor_mode::selftest_join, &editor_mode::selftest_signal, &editor_mode::selftest_terrain, &editor_mode::selftest_props, &editor_mode::selftest_monkey})
		if (auto const result{(this->*group)(command, words)}; result != selftest_result::unhandled)
			return result;
	for (auto *tool : m_tools)
		if (auto const result{tool->selftest(command, words)}; result != selftest_result::unhandled)
			return result;
	return selftest_result::unhandled;
}

// the script is a text file, one action a line:
//   wait <frames>
//   camera <x> <y> <z> <x> <y> <z>        from the first point, looking at the second
//   select <track name>
//   selectat <x> <z>                     the path nearest the point, within 3 m, as a click on it
//   trackends <name>                     logs the ends of the first path of the track against the ground under them
//   kbreak <x> <z>                       K on the track nearest the point: split there, a break of the grade at the height of the ground
//   blade [name]                         logs the blades of each switch cut from the rails: tip, 50 mm head, full head, heel, drop of the head, open flangeway, the gauge lines apart against x^2/2R, vertices in both positions
//   frog [name]                          logs where the inner rails of each switch cross: distance on both paths, 1:n, throat, nose, check rails
//   guards [name]                        logs each guard rail and what each of its ends and breaks is cut along; a fault when a cut is not along a rail or a glued end not on the guard rail of the other track
//   railtime                             lays out the rails of every track, as when its geometry is made, and logs how long it took
//   throw <0|1>                          sets every switch to the path and moves its blades there at once
//   crossings [name]                     logs where rails of tracks cross rails of other tracks and of what kind (acute, obtuse, middle acute or obtuse, wide), checks that each acute crossing has one partner and one nose
//   switchtimbers <model|none>           the model of the timbers set on all the switches and crossings, as the buttons of the switch tool do it
//   frogkey name [on|off]                sets or clears the frogs key of a switch (one undo step); logs the key, whether it is written, the frog and the crossings
//   geomhash [name]                      logs the number of vertices and a hash of the drawn geometry of each rail track (rails, trackbed, cut faces), to compare builds; exact: the bits of every value, userdata and tread materials too
//   sweep <key>=<value>...               model, params (separated by commas), lateral, height, side (origin, near, far,
//                                        centre), top (origin, bottom, top, edge), mode (bend, repeat), step,
//                                        length (original, all, or metres), from, to, widen (0, 1), face (0, 1), tilt (0, 1)
//   selectmodel <x> <z>                  the model instance nearest the point
//   bend <key>=<value>...                the selected model bent along a track: track (index on the list), axis (auto, z, x),
//                                        turned (0, 1), along (start, origin, middle, end, point), side (origin, near, far,
//                                        middle, point), height (origin, bottom, top, edge, point), point=<x>,<y>,<z>,
//                                        pick=<x>,<y>,<z> (a point of the scene picked as the reference point), go (0, 1)
//   selectinclude <x> <z>                the include placed nearest the point
//   addmodel <x> <z>                     the model instance nearest the point added to the selection, or taken out (Shift+LMB)
//   selection                            logs the selected models, with the models of their groups
//   moveselection <x> <y> <z>            the selection moved by the offset, as the gizmo moves it
//   turnselection <degrees>              the selection turned around the model with the gizmo
//   copy, paste <x> <y> <z>, group, ungroup, drop, delete   as the keys of the selection do them
//   bendset <key>=<value>...             the bent model being edited: length, lateral, height, along, side, height
//   bendprobe                            logs the bent model being edited
//   undo, redo
//   key <S|W|CTRL|RMB> <press|release> [ctrl]  a key or right button event, as the window gives it
//   lineside <vehicle|hekto|fouling|parallel>  the tab of the objects along the track in the track window
//   event <name>                         queues the event
//   memcell <name>                       logs the memory cell
//   bendpanel                            unfolds the bend section of the toolset
//   openscenerypopup                     the window of File / Open scenery, its list searched for zz_
//   openscenery <scenery file>           the editor starts again with the scenery
//   checkspans                           logs whether the text the scenery files hold at each node's place starts and ends it
//   spread <track> <stop 0..2> <reach>   logs the paths the spread from the track takes
//   newscenery                           the editor starts again with the wizard of a new scenery
//   mapstate                             logs the orthophoto, the top view and the //$o line of the scenery
//   trainset <track> <vehicle path> <skin> <facing> [km/h]  the trainset tool puts the vehicle in the middle of the track
//   vehicles                             logs each vehicle: its head, facing, and what its couplers hold
//   terrainmode                          the terrain field of work, with its texture browser
//   terrainsculpt <x> <z> <radius> <metres>, terrainpaint <x> <z> <radius> <palette entry>,
//   terrainlevel <x> <z> <radius> <target height> [mode]   as a stroke of the brush
//   terrainadd <count> <spacing>          chunks made around the camera, count x count, as the button of the chunks panel does it
//   terrainprobe <x> <z>                 logs the height of the heightmap terrain and the materials of the chunk
//   terrainconvert                       converts every material of the terrain files of the scenery
//   terrainmodifier <name> <x0> <z0> <x1> <z1> <formation> <half width>   a straight modifier, as the vertical profile makes them
//   terraintouchup <0|1>                 the brushes touch up over the modifiers, or shape the ground under them
//   terrainrestore <x> <z> <radius>      a stroke of the restore brush
//   terrainbake <name>                   the modifier baked into the ground
//   terrainselect <cx> <cz> [add]        the chunk selected with the spacing tool (add: Ctrl)
//   terrainresolution <spacing> [paint 0|1]   the selected chunks converted, as the button of the resolution panel does it
//   terrainfill <palette entry> <blur> <rotation> <x> <z> <x> <z> <x> <z>...   the area outlined filled, as the fill tool does it
//   terrainturn <palette entry> <degrees>  the textures of the palette entry turned
//   terrainwater <index> <material> [size]    material and repeat of a body of water; terrainwaterdefault <material>
//   orthophoto <north> <east> <radius> <fit 0|1> [in scene 0|1]   the imagery layer on, placed and fitted as given
//   join <x> <z> <x> <z> [radius] [variant] [transitions 0|1]   the free ends at the points joined, as a drag from one to the other
//   layto <x> <z> <x> <z> <layout 0..3> <radius>   from the free end at the first point towards the second, as a drag in the layout
//   layclicks <layout 0..3> <radius> <x> <z>...   clicks of the lay tool at the points, a free end within 3 m joined
//   layspeed <km/h> <own 0|1>            the speed set for the laying, and whether it goes also on from a track with its own
//   laymode <straights|curve>, layradius <R from> <R to>, laytrace <radii 0..3> <straights between 0|1>   as the panel of the laying sets them
//   endcurves                            logs every free end of the rails: where, the way out and the curvature its track goes on with
//   curvejoint <x> <z>                   logs the paths meeting at the end nearest the point, the curvature of each and its jump from the ones before to the ones after
//   laywalk <from|into> <x> <z> <spacing> <noise> <k0> <k1> <L>...   a line of known curves on from the free end nearest the point,
//                                        the points along it clicked in the mode of a curve, from the end or towards it (see the command)
//   laybends <x> <z> <distance> <degrees> ... <distance>   the bends of straights clicked from the free end nearest the point
//   layp <x> <z>...                      the points clicked in the mode set, on the ground, a free end within 3 m reached
//   layreport <label>                    logs the line the points clicked make, as Enter would lay it: its bends and curves, warnings, traces
//   layenter, layback, laycancel         as Enter, Backspace and Esc while laying
//   laysample <x> <z> <heading> <spacing> <noise> <deflection> <R> [<from the first vertex to the second, m> <deflection> <R>]
//                                        a line of known curves, the points along them clicked as a curve traced, the line they make logged
//   laychain <layout> <R> <x> <z> | b | u | r | e | t | p...   the points clicked, Backspace, undo, redo, Enter, Tab, the line logged
//   joints <x> <z>...                    the joints check over the scenery, the issues around the points logged, every issue with free ends without points
//   route <x> <z> <single 0|1>          the route design of the path nearest the point, a curve or the whole line
//   routevertex <i> <R> <L in> <L out> <cant>, routearc <i> <R> <L> <share>, routeshare <i> <share>, routekm <km>
//   routeapply, routetake                as the buttons of the route design do it
//   alignments                           logs the alignments kept in the scenery and the element under each path
//   digitizeall                          every line not digitized yet kept in the scenery, as the button of the route design does it
//   splitkind <kind>, nudgekind <kind> <dx> <dz>   the first path over an element of the kind split in two, or moved
//   switchplace <template> <x> <z> <direction> <side>   the catalogue template whose label starts so put into the straight
//                                        nearest the point, or into the curve (switchcurve); slipplace <template> <x> <z>
//   switchpieces <R> <n> [name]          logs the diverging path made of pieces of each switch: length, the arc R of 1:n against the axis
//   switchbreak [name]                   the first piece of the diverging path moved off its neighbour, the switch committed
//   switches                             logs the switches kept in the scenery and the speed of each diverging track
//   deletetrack <name>, deletekind <kind>   the path deleted as Del does it, or the first path over an element of the kind
//   tracksel <name>...                   the tracks selected together, as Shift+clicks select them; none: nothing selected
//   trackprop <key> [value]              the value given to the selected tracks, as typed into the inspector; trackprops logs their values
//   trackstyle copy [name] | groups <n>... | paste | panel   the properties of a track copied, the groups chosen, pasted on the selected tracks
//   tracksimilar <key>                   the tracks of the scenery with the value of the property of the selected ones selected
//   btfcheck <terrain file>              the .txtf converted to .btf, the editor lines the binary file keeps logged
//   mouse <x> <y>, mouseat <x> <y> <z>   the cursor moved to the point of the window, or to where the point of the scene is seen, as the mouse moves it
//   glide <x> <y> <z> <steps> <frames> [shot]   the cursor led to where the point is seen in steps, frames apart, a screenshot after each step with shot
//   button <left|right|middle> <press|release>, wheel <offset>   the buttons and the wheel of the mouse, with the keys held
//   press <key>, release <key>, tap <key>   a key held, let go, or both (ctrl, shift, alt, esc, enter, backspace, del, space, tab, a..z, 0..9, f1..f12)
//   softcursor <0|1>                     the cursor drawn into the picture, for screenshots
//   type <text>                          the characters typed into the text field with the focus
//   screenshot
//   save
//   log <text>
//   perf <label>                         the frames drawn since the last perf and the time per frame logged, the longest frame and those over 33 ms,
//                                        with the timings and counts of the renderer
//   fittime <metres>...                  the grade line of a straight over rolling ground of each length fitted, as at a click, and the time the
//                                        frame waits for it while the track is laid, the line fitted on the thread of the fitter
//   nmtauto [unfitted]                   the tracks put onto the terrain model, all or the ones whose grade line isn't fitted yet
//   fittedstate [reload]                 logs how many tracks have their grade line fitted, and which; reload: read again from the //$f lines
//   trackterrain <pull|restore> <track>...   the terrain drawn to the tracks, or their formations taken out of it, as the panel of the tracks
//   terrainhash                          a hash of the heights of the terrain chunks in memory, to compare the ground two builds made
//   rebuildall                           the geometry of every path in the sections made already laid anew, the time it took logged,
//                                        and the 3d sleepers of all the tracks laid anew, timed apart
//   sleepercheck                         the 3d sleepers of all the tracks checked for beams over each other and rails without enough under them
//   beams <track> <z from> <z to>        the beams the track lays with their middles between the two z
//   sharedzones <track>                  where the timbers shared with other tracks reach along the paths of the track and past its ends
//   quit
void editor_mode::selftest_step()
{
	auto &test{m_selftest};
	if (false == test.loaded)
	{
		test.loaded = true;
		auto const *file{std::getenv("EU07_EDITOR_SELFTEST")};
		if (file == nullptr)
			return;
		std::ifstream stream(file);
		std::string line;
		while (std::getline(stream, line))
			if (false == line.empty() && line.front() != '#')
				test.lines.push_back(line);
		WriteLog("SELFTEST: " + std::to_string(test.lines.size()) + " steps from " + file);
	}
	++editor_selftest::frames();
	// how long each frame took, for the longest one perf tells
	{
		static auto last{std::chrono::steady_clock::now()};
		auto const now{std::chrono::steady_clock::now()};
		auto const frame{std::chrono::duration<double, std::milli>(now - last).count()};
		last = now;
		editor_selftest::frame_longest() = std::max(editor_selftest::frame_longest(), frame);
		if (frame > 33.0)
			++editor_selftest::frames_slow();
	}
	// the gesture of the clip being recorded, its bottom left corner where the clip has it
	if (false == test.badge.empty())
	{
		auto const words{gesture_picture(test.badge)};
		auto const height{std::round(ImGui::GetTextLineHeight() * 1.6f)};
		auto const pad{std::round(height * 0.25f)};
		ImVec2 const at{test.badge_at.x + pad, test.badge_at.y - pad - height};
		auto &list{*ImGui::GetForegroundDrawList(ImGui::GetMainViewport())};
		list.AddRectFilled(ImVec2(at.x - pad, at.y - pad), ImVec2(at.x + gesture_width(height, words) + pad, at.y + height + pad), IM_COL32(16, 20, 26, 225), pad);
		gesture_draw(list, at, height, words, IM_COL32(235, 235, 235, 255));
	}
	if (test.wait > 0)
	{
		--test.wait;
		return;
	}
	while (test.next < test.lines.size())
	{
		std::istringstream words(test.lines[test.next++]);
		std::string command;
		words >> command;
		if (command == "wait")
		{
			words >> test.wait;
			return;
		}
		if (auto const result{selftest_dispatch(command, words)}; result == selftest_result::pause || result == selftest_result::again)
		{
			if (result == selftest_result::again)
				--test.next;
			return;
		}
		if (command == "quit")
		{
			if (false == test.quitting)
			{
				WriteLog("SELFTEST done");
				test.quitting = true;
				test.wait = 30;
				--test.next;
				return;
			}
			Application.queue_quit(true);
			return;
		}
	}
}
