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
#include <format>
#include <fstream>
#include <map>
#include <span>
#include <sstream>
#include <string_view>
#include <utility>
#include "application/editormodeselftest.h"

namespace
{

// the text put into the buffer of a field, cut to its size
void copy_text(std::span<char> const Buffer, std::string_view const Text)
{
	*std::format_to_n(Buffer.data(), static_cast<std::ptrdiff_t>(Buffer.size()) - 1, "{}", Text).out = '\0';
}

// the track nearest the point in the plan, and how far it is in the plan; Plain: of the plain tracks only, no switches nor turntables
std::pair<TTrack *, double> nearest_track(glm::dvec3 const &Point, bool const Plain)
{
	TTrack *nearest{nullptr};
	double distance{1e9};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty() || (Plain && track->eType != tt_Normal))
			continue;
		if (auto const d{geometry::plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, Point)), Point)}; d < distance)
		{
			distance = d;
			nearest = track;
		}
	}
	return {nearest, distance};
}

void log_rebuild_all()
{
	// the geometry of every path made anew, as the editor does it after a change, timed
	auto const start{std::chrono::steady_clock::now()};
	auto count{0};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		auto const &section{simulation::Region->section(track->location())};
		if (false == section.m_geometrycreated)
			continue;
		track->rebuild_geometry(section.m_geometrybank);
		++count;
	}
	WriteLog(format("SELFTEST rebuildall: %d paths in %.1f ms", count, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()));
	// the sleepers laid again on all the tracks, which the renderer otherwise does as they come in sight, timed by section
	std::map<scene::basic_section const *, double> sectiontimes;
	auto laidcount{0};
	std::size_t parts{0};
	auto laytime{0.0};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || false == track->m_sleepers3d.active)
			continue;
		auto const laystart{std::chrono::steady_clock::now()};
		track_sleepers::lay(*track);
		auto const took{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - laystart).count()};
		laytime += took;
		sectiontimes[&simulation::Region->section(track->location())] += took;
		++laidcount;
		for (auto const &instances : track->m_sleepers3d.instances)
			parts += instances.size();
	}
	auto slowest{0.0};
	for (auto const &[section, sectiontime] : sectiontimes)
		slowest = std::max(slowest, sectiontime);
	WriteLog(format("SELFTEST rebuildall sleepers3d: %d tracks laid in %.1f ms, %zu parts; %zu sections, %.2f ms per section, %.2f ms the slowest", laidcount, laytime, parts, sectiontimes.size(),
	    (sectiontimes.empty() ? 0.0 : laytime / static_cast<double>(sectiontimes.size())), slowest));
}

// the frames of the sweep named at its stations from, to, by the step
void log_frames(std::istringstream &Words)
{
	std::string name;
	double from{0.0};
	double to{0.0};
	double step{0.1};
	Words >> name >> from >> to >> step;
	auto const *sweep{simulation::Sweeps.find(name)};
	if (sweep == nullptr)
		return;
	auto const side{sweep->definition().lateral >= 0.0 ? 1 : 0};
	if (step <= 0.0)
		return;
	for (int i = 0; from + i * step <= to + 1e-9; ++i)
	{
		auto const station{from + i * step};
		auto const at{sweep->frame_at(station)};
		WriteLog(format("SELFTEST frame %s %.2f: %.3f %.3f %.3f widening %.4f cant %.4f setback %.4f", name.c_str(), station, at.position.x, at.position.y, at.position.z, at.widening[side], at.cant[side], sweep->setback(at)));
	}
}

// where points of the outline of a sweep come on the screen
void log_projection(std::vector<glm::dvec3> const &Outline)
{
	screen_projection const projection;
	for (std::size_t i = 0; i < Outline.size() && i < 45; i += 4)
	{
		ImVec2 screen;
		auto const shown{projection.project(Outline[i], screen)};
		auto const clip{projection.clip(Outline[i])};
		WriteLog(format("SELFTEST proj %zu: %.1f %.1f %.1f -> %d %.0f %.0f w %.2f", i, Outline[i].x, Outline[i].y, Outline[i].z, shown ? 1 : 0, screen.x, screen.y, clip.w));
	}
}

// what the end of the vehicle is coupled to: the vehicle and the flags of the coupling, a dash for nothing
std::string coupled_to(TDynamicObject const &Vehicle, int const End)
{
	auto const &hitch{Vehicle.MoverParameters->Couplers[End]};
	return hitch.Connected != nullptr ? std::format("{}/{}", hitch.Connected->Name, hitch.CouplingFlag) : std::string{"-"};
}

void log_vehicles()
{
	for (auto const *vehicle : simulation::Vehicles.sequence())
	{
		if (vehicle == nullptr)
			continue;
		auto const head{vehicle->HeadPosition()};
		auto const facing{vehicle->VectorFront()};
		WriteLog(format("SELFTEST vehicle %s head %.2f %.2f front %.2f %.2f dir %d coupler0 %s coupler1 %s", vehicle->name().c_str(), head.x, head.z, facing.x, facing.z, vehicle->iDirection, coupled_to(*vehicle, 0).c_str(), coupled_to(*vehicle, 1).c_str()));
	}
}

// the mouse led to the point in steps eased in and out: the lines of the script for them put next, with a wait after each
// and a screenshot when asked for
void glide_to(std::istringstream &Words, scripted_input const &Input, std::vector<std::string> &Lines, std::size_t const Next)
{
	glm::dvec3 at{0.0};
	int steps{8};
	int every{2};
	std::string shots;
	Words >> at.x >> at.y >> at.z >> steps >> every >> shots;
	ImVec2 projected;
	if (false == screen_projection{}.project(at, projected))
		return;
	glm::dvec2 const from{Input.mouse().value_or(glm::dvec2{projected.x, projected.y})};
	glm::dvec2 const to{projected.x, projected.y};
	std::vector<std::string> expanded;
	for (int step = 1; step <= std::max(1, steps); ++step)
	{
		auto const t{static_cast<double>(step) / std::max(1, steps)};
		auto const eased{t * t * (3.0 - 2.0 * t)};
		auto const point{glm::mix(from, to, eased)};
		expanded.push_back(format("mouse %.1f %.1f", point.x, point.y));
		expanded.push_back(std::format("wait {}", std::max(1, every)));
		if (shots == "shot")
			expanded.emplace_back("screenshot");
	}
	Lines.insert(Lines.begin() + static_cast<std::ptrdiff_t>(Next), expanded.begin(), expanded.end());
}

// the layer of the file named, none for no such layer
scene::layer_handle layer_named(std::string_view const Name)
{
	for (std::size_t idx = 1; idx <= scene::Layers.size(); ++idx)
		if (scene::Layers.layer(static_cast<scene::layer_handle>(idx)).name == Name)
			return static_cast<scene::layer_handle>(idx);
	return null_handle;
}

void log_layer_merge(std::istringstream &Words)
{
	// the layer of the first file merged into the layer of the second, as the Merge button of the layers panel does
	std::string source;
	std::string target;
	Words >> source >> target;
	auto const from{layer_named(source)};
	auto const into{layer_named(target)};
	std::string reason;
	auto const merged{from != null_handle && into != null_handle && scene::Layers.can_merge(from, into, reason) && scene::Layers.merge(from, into)};
	WriteLog("SELFTEST layermerge " + source + " " + target + ": " + (merged ? std::string{"merged"} : "not merged " + reason));
}

// what the top view shows of the tracks, by its name: the rails alone, the tracks without their ballast, all of them
int ortho_tracks_of(std::string_view const Ballast)
{
	if (Ballast == "rails")
		return 2;
	if (Ballast == "noballast")
		return 1;
	return 0;
}

// the code of the key named: CTRL, F10 or a capital letter; 0 for another
int key_code(std::string const &Key)
{
	if (Key == "CTRL")
		return GLFW_KEY_LEFT_CONTROL;
	if (Key == "F10")
		return GLFW_KEY_F10;
	if (Key.size() == 1 && Key[0] >= 'A' && Key[0] <= 'Z')
		return GLFW_KEY_A + (Key[0] - 'A');
	return 0;
}

// a submodel without triangles drawn up close, which the box of the model leaves out
bool left_out_of_box(TSubModel const &Submodel)
{
	return Submodel.eType != GL_TRIANGLES || Submodel.fSquareMinDist > 0.f || (Submodel.m_geometry.handle.bank == 0 && Submodel.m_geometry.handle.chunk == 0);
}

// the transform of the submodel in its model: the matrices of it and of its parents which have one
glm::dmat4 local_of(TSubModel *Submodel)
{
	glm::dmat4 local{1.0};
	for (auto *parent = Submodel; parent != nullptr; parent = parent->Parent)
		if ((parent->iFlags & 0xC000) != 0 && parent->GetMatrix() != nullptr)
			local = glm::dmat4(glm::make_mat4(parent->GetMatrix()->readArray())) * local;
	return local;
}

// the box grown by the vertices of the submodel, placed by the transform
void grow_box_by(TSubModel const &Submodel, glm::dmat4 const &Transform, glm::dvec3 &Low, glm::dvec3 &High)
{
	for (auto const &vertex : GfxRenderer->Vertices(Submodel.m_geometry.handle))
	{
		auto const world{glm::dvec3(Transform * glm::dvec4(glm::dvec3(vertex.position), 1.0))};
		Low = glm::min(Low, world);
		High = glm::max(High, world);
	}
}

// the box grown by the vertices of the submodels from the root down, placed by the transform
void grow_box(TSubModel *Root, glm::dmat4 const &Transform, glm::dvec3 &Low, glm::dvec3 &High)
{
	std::vector pending{Root};
	while (false == pending.empty())
	{
		auto *submodel{pending.back()};
		pending.pop_back();
		while (submodel != nullptr)
		{
			if (submodel->Child != nullptr)
				pending.push_back(submodel->Child);
			if (false == left_out_of_box(*submodel))
				grow_box_by(*submodel, Transform * local_of(submodel), Low, High);
			submodel = submodel->Next;
		}
	}
}

// the model nearest the point in the plan, within 50 m
TAnimModel *nearest_model(glm::dvec3 const &Point)
{
	TAnimModel *nearest{nullptr};
	auto best{50.0};
	for (auto *model : simulation::Instances.sequence())
	{
		if (model == nullptr)
			continue;
		auto const distance{glm::length(glm::dvec2{model->location().x - Point.x, model->location().z - Point.z})};
		if (distance < best)
		{
			best = distance;
			nearest = model;
		}
	}
	return nearest;
}

// the name of the model and where it stands, none for no model
std::string model_place(TAnimModel const *Model)
{
	return Model != nullptr ? format("%s at %.2f %.2f %.2f", Model->name().c_str(), Model->location().x, Model->location().y, Model->location().z) : std::string{"none"};
}

// undo <name>: the record of that name as it is after it, a slip laid again back of its template; nothing without a name
std::string record_after_undo(std::string const &Name)
{
	if (Name.empty())
		return {};
	auto const &records{switch_store::records()};
	auto const known{std::ranges::find_if(records, [&Name](switch_store::record const &Entry) { return Entry.name == Name; })};
	return known != records.end() ? format(": %s %s, %zu parts", known->kind.c_str(), switch_store::label(*known).c_str(), known->parts.size()) : ": no record " + Name;
}

} // namespace

editor_mode::selftest_result editor_mode::selftest_select_at(std::istringstream &words)
{
	glm::dvec3 point{0.0};
	words >> point.x >> point.z;
	auto const [nearest, distance]{nearest_track(point, false)};
	m_node = distance < 3.0 ? nearest : nullptr;
	m_track_point = {};
	ui()->set_node(m_node);
	WriteLog(format("SELFTEST selectat %.1f %.1f: %s", point.x, point.z, m_node != nullptr ? m_node->name().c_str() : "nothing"));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_track_ends(std::istringstream &words) const
{
	// the ends of the first path of the track, against the ground under them
	std::string name;
	words >> name;
	auto const *track{simulation::Paths.find(name)};
	if (track == nullptr || track->m_paths.empty())
	{
		WriteLog("SELFTEST trackends " + name + ": no such track");
		return selftest_result::handled;
	}
	for (auto const end : {segment_data::point::start, segment_data::point::end})
	{
		auto const &point{track->m_paths.front().points[end]};
		std::vector<char> found;
		auto const ground{ground_heights({glm::dvec3{point.x, point.y + 50.0, point.z}}, true, &found).front()};
		WriteLog(format("SELFTEST trackends %s %s: %.2f %.3f %.2f, ground %.3f (found %d)", name.c_str(), end == segment_data::point::start ? "start" : "end", point.x, point.y, point.z, ground, found[0]));
	}
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_kbreak(std::istringstream &words)
{
	// K on the track nearest the point: split there, the point between at the height of the ground and the ballast
	glm::dvec3 point{0.0};
	words >> point.x >> point.z;
	auto const [nearest, distance]{nearest_track(point, true)};
	if (nearest == nullptr || distance > 3.0)
	{
		WriteLog(format("SELFTEST kbreak %.1f %.1f: no track", point.x, point.z));
		return selftest_result::handled;
	}
	auto const tracks{simulation::Paths.sequence().size()};
	split_track_at(*nearest, point);
	auto const &path{nearest->m_paths.front()};
	auto const joint{path.points[segment_data::point::end]};
	std::vector<char> found;
	auto const ground{ground_heights({glm::dvec3{joint.x, joint.y + 50.0, joint.z}}, true, &found).front()};
	geometry::bezier const curve{path};
	auto const rise = [](glm::dvec3 const &Tangent) { return 1000.0 * Tangent.y / std::max(1e-9, glm::length(geometry::plan_of(Tangent))); };
	WriteLog(format("SELFTEST kbreak %s: %zu tracks more, joint %.2f %.3f %.2f, ground %.3f (found %d), grade %.2f -> %.2f permille", nearest->name().c_str(), simulation::Paths.sequence().size() - tracks, joint.x, joint.y, joint.z, ground, found[0], rise(curve.first(0.0)),
	                rise(curve.first(1.0))));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_scene_view(std::string_view const command, std::istringstream &words)
{
	if (command == "camera")
	{
		glm::dvec3 from;
		glm::dvec3 to;
		words >> from.x >> from.y >> from.z >> to.x >> to.y >> to.z;
		auto const look{glm::normalize(to - from)};
		Camera.Pos = from;
		Camera.Angle = glm::vec3(static_cast<float>(std::asin(std::clamp(look.y, -1.0, 1.0))), static_cast<float>(std::atan2(-look.x, -look.z)), 0.0f);
		Global.pCamera = Camera;
		m_focus_active = false;
		return selftest_result::handled;
	}
	if (command == "zoom")
	{
		float factor{1.0f};
		words >> factor;
		Global.ZoomFactor = std::max(factor, 0.01f);
		return selftest_result::handled;
	}
	if (command == "selectat")
		return selftest_select_at(words);
	if (command == "trackends")
		return selftest_track_ends(words);
	if (command == "kbreak")
		return selftest_kbreak(words);
	if (command == "select")
	{
		std::string name;
		words >> name;
		m_node = simulation::Paths.find(name);
		m_track_point = {};
		ui()->set_node(m_node);
		WriteLog("SELFTEST select " + name + (m_node != nullptr ? "" : ": no such track"));
		return selftest_result::handled;
	}
	if (command == "perf")
	{
		// the frames drawn and the time gone since the last perf, with the timings and the counts the renderer keeps
		std::string label;
		words >> label;
		static auto last{std::chrono::steady_clock::now()};
		static std::size_t lastframe{0};
		auto const now{std::chrono::steady_clock::now()};
		auto const frames{editor_selftest::frames() - lastframe};
		auto const elapsed{std::chrono::duration<double, std::milli>(now - last).count()};
		WriteLog(format("SELFTEST perf %s: %zu frames, %.3f ms per frame, the longest %.1f ms, %zu over 33 ms | %s | %s", label.c_str(), frames, frames > 0 ? elapsed / static_cast<double>(frames) : 0.0, editor_selftest::frame_longest(), editor_selftest::frames_slow(), editor_selftest::one_line(GfxRenderer->info_times()).c_str(), editor_selftest::one_line(GfxRenderer->info_stats()).c_str()));
		WriteLog("SELFTEST perf " + label + " " + m_streamer.stats());
		last = now;
		lastframe = editor_selftest::frames();
		editor_selftest::frame_longest() = 0.0;
		editor_selftest::frames_slow() = 0;
		return selftest_result::handled;
	}
	if (command == "rebuildall")
	{
		log_rebuild_all();
		return selftest_result::handled;
	}
	if (command == "frames")
	{
		log_frames(words);
		return selftest_result::handled;
	}
	if (command == "weather")
	{
		auto const &time{simulation::Time.data()};
		WriteLog(format("SELFTEST weather: %02d:%02d, fog end %.0f m, overcast %.2f", time.wHour, time.wMinute, Global.fFogEnd, Global.Overcast));
		return selftest_result::handled;
	}
	if (command == "projtest")
	{
		log_projection(m_sweep.outline);
		return selftest_result::handled;
	}
	if (command == "newscenery")
	{
		WriteLog(restart_for_new_scenery() ? "SELFTEST newscenery: started" : "SELFTEST newscenery: failed");
		return selftest_result::pause;
	}
	if (command == "vehicles")
	{
		log_vehicles();
		return selftest_result::handled;
	}
	if (command == "mouseat")
	{
		glm::dvec3 at{0.0};
		words >> at.x >> at.y >> at.z;
		ImVec2 projected;
		if (false == screen_projection{}.project(at, projected))
		{
			WriteLog("SELFTEST mouseat: behind the camera");
			return selftest_result::handled;
		}
		m_selftest.input.mouse_to(glm::dvec2{projected.x, projected.y});
		WriteLog(format("SELFTEST mouse %.0f %.0f", projected.x, projected.y));
		return selftest_result::handled;
	}
	if (command == "glide")
	{
		glide_to(words, m_selftest.input, m_selftest.lines, m_selftest.next);
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_scene_files(std::string_view const command, std::istringstream &words)
{
	if (command == "screenshot")
	{
		Application.queue_screenshot();
		m_selftest.wait = 3;
		return selftest_result::pause;
	}
	if (command == "save")
	{
		save();
		return selftest_result::handled;
	}
	if (command == "layermerge")
	{
		log_layer_merge(words);
		return selftest_result::handled;
	}
	if (command == "ortho")
		return selftest_ortho(words);
	if (command == "mapstate")
		return selftest_map_state();
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_ortho(std::istringstream &words)
{
	float extent{0.0f};
	std::string ballast;
	words >> extent >> ballast;
	choose_work_area(work_area::tracks);
	if (Global.EditorOrtho == extent < 0.0f)
		toggle_ortho();
	if (extent > 0.0f)
		Global.EditorOrthoExtent = std::clamp(extent, 5.0f, 50000.0f);
	if (false == ballast.empty())
		Global.EditorOrthoTracks = ortho_tracks_of(ballast);
	WriteLog(format("SELFTEST ortho extent %.0f tracks %d", Global.EditorOrthoExtent, Global.EditorOrthoTracks));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_map_state() const
{
	auto const marked{scene::Layers.empty() ? std::vector<std::string>{} : scene::Layers.marked(scene::layer_handle{1}, "//$o")};
	WriteLog(format("SELFTEST mapstate orthophoto %d top %d extent %.0f camera %.1f %.1f %.1f", m_orthophoto.enabled() ? 1 : 0, Global.EditorOrtho ? 1 : 0, Global.EditorOrthoExtent, Camera.Pos.x, Camera.Pos.y, Camera.Pos.z));
	WriteLog("SELFTEST mapstate mark " + (marked.empty() ? std::string{"-"} : marked.front()));
	auto const [grounded, guessed]{m_orthophoto.grounded()};
	WriteLog(format("SELFTEST mapstate tiles on the ground %zu, at guessed heights %zu", grounded, guessed));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_scene_nodes(std::string_view const command, std::istringstream &words)
{
	if (command == "selectmodel")
	{
		glm::dvec3 point{0.0};
		words >> point.x >> point.z;
		auto *nearest{nearest_model(point)};
		m_node = nearest;
		select_include(0);
		ui()->set_node(m_node);
		WriteLog("SELFTEST selectmodel: " + model_place(nearest));
		return selftest_result::handled;
	}
	if (command == "addmodel")
	{
		glm::dvec3 point{0.0};
		words >> point.x >> point.z;
		auto *nearest{nearest_model(point)};
		select_node(nearest, true);
		WriteLog("SELFTEST addmodel: " + model_place(nearest));
		return selftest_result::handled;
	}
	if (command == "selection")
	{
		std::string text;
		for (auto *node : selection_with_groups())
			text += format(" [%s %.2f %.2f %.2f group %d]", node->name().c_str(), node->location().x, node->location().y, node->location().z, static_cast<int>(node->group()));
		WriteLog("SELFTEST selection:" + text);
		return selftest_result::handled;
	}
	if (command == "moveselection" && m_node != nullptr)
	{
		glm::dvec3 offset{0.0};
		words >> offset.x >> offset.y >> offset.z;
		selection_snapshots(EditorSnapshot::Action::Move);
		auto const before{m_node->location()};
		m_editor.translate(m_node, before + offset, true);
		selection_follow(m_node->location() - before, glm::vec3(0.0f), m_node->location());
		return selftest_result::handled;
	}
	if (command == "turnselection" && m_node != nullptr)
	{
		float degrees{0.0f};
		words >> degrees;
		selection_snapshots(EditorSnapshot::Action::Rotate);
		glm::vec3 const turned{0.0f, degrees, 0.0f};
		m_editor.rotate(m_node, turned, 0.0f);
		selection_follow(glm::dvec3(0.0), turned, m_node->location());
		return selftest_result::handled;
	}
	if (command == "copy")
	{
		copy_selection();
		return selftest_result::handled;
	}
	if (command == "paste")
	{
		glm::dvec3 point{0.0};
		words >> point.x >> point.y >> point.z;
		paste_clipboard(point);
		return selftest_result::handled;
	}
	if (command == "group")
	{
		group_selection();
		return selftest_result::handled;
	}
	if (command == "ungroup")
	{
		ungroup_selection();
		return selftest_result::handled;
	}
	if (command == "drop")
	{
		drop_selection_to_ground();
		return selftest_result::handled;
	}
	if (command == "delete")
	{
		delete_selected();
		return selftest_result::handled;
	}
	if (command == "selectinclude")
		return selftest_select_include(words);
	if (command == "modelbox")
		return selftest_model_box();
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_select_include(std::istringstream &words)
{
	glm::dvec3 point{0.0};
	words >> point.x >> point.z;
	scene::instance_handle nearest{0};
	auto best{50.0};
	m_node = nullptr;
	for (scene::instance_handle instance = 1; instance <= scene::Layers.instance_count(); ++instance)
	{
		if (false == scene::Layers.removable(instance))
			continue;
		select_include(instance);
		bend_source source;
		if (false == bend_source_of_selection(source))
			continue;
		auto const at{glm::dvec3(source.transform[3])};
		auto const distance{glm::length(glm::dvec2{at.x - point.x, at.z - point.z})};
		if (distance < best)
		{
			best = distance;
			nearest = instance;
		}
	}
	select_include(nearest);
	WriteLog("SELFTEST selectinclude: " + (nearest != 0 ? *scene::Layers.instance(nearest).file : std::string{"none"}));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_model_box() const
{
	auto const *model{dynamic_cast<TAnimModel const *>(m_node)};
	bend_source source;
	if (model == nullptr || false == bend_source_of_selection(source))
	{
		WriteLog("SELFTEST modelbox: no model selected");
		return selftest_result::handled;
	}
	scene::node_data const data{};
	sweep_node probe{data};
	sweep_node::state state;
	state.model = source.file;
	state.skin = source.skin;
	state.scale = source.scale;
	probe.define(state);
	glm::dvec3 boxlow{std::numeric_limits<double>::max()};
	glm::dvec3 boxhigh{-std::numeric_limits<double>::max()};
	for (auto const &item : probe.items())
		grow_box(item.model->GetSMRoot(), source.transform * glm::scale(glm::dmat4(1.0), source.scale) * item.transform, boxlow, boxhigh);
	WriteLog(format("SELFTEST modelbox: %.2f %.2f %.2f - %.2f %.2f %.2f", boxlow.x, boxlow.y, boxlow.z, boxhigh.x, boxhigh.y, boxhigh.z));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_key(std::istringstream &words)
{
	std::string key;
	std::string action;
	std::string mods;
	words >> key >> action >> mods;
	if (key == "RMB")
		m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT, action == "press" ? GLFW_PRESS : GLFW_RELEASE);
	else
		on_key(key_code(key), 0, action == "press" ? GLFW_PRESS : GLFW_RELEASE, mods == "ctrl" ? GLFW_MOD_CONTROL : 0);
	WriteLog(format("SELFTEST key %s %s %s: camera %.2f %.2f %.2f, key S %d, right button %d", key.c_str(), action.c_str(), mods.c_str(), Camera.Pos.x, Camera.Pos.y, Camera.Pos.z, static_cast<int>(input::keys[GLFW_KEY_S]), m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT)));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_scene_other(std::string_view const command, std::istringstream &words)
{
	if (command == "key")
		return selftest_key(words);
	if (command == "event")
	{
		std::string name;
		words >> name;
		auto *found{simulation::Events.FindEvent(name)};
		if (found != nullptr)
			simulation::Events.AddToQuery(found, nullptr);
		WriteLog("SELFTEST event " + name + (found != nullptr ? " queued" : ": no such event"));
		return selftest_result::handled;
	}
	if (command == "memcell")
	{
		std::string name;
		words >> name;
		auto const *cell{simulation::Memory.find(name)};
		WriteLog("SELFTEST memcell " + name + (cell != nullptr ? format(": %s %.1f %.1f", cell->Text().c_str(), cell->Value1(), cell->Value2()) : std::string{": none"}));
		return selftest_result::handled;
	}
	if (command == "newscenerypopup")
	{
		m_newscenery_asked = true;
		return selftest_result::handled;
	}
	if (command == "openscenerypopup")
	{
		m_openscenery_asked = true;
		copy_text(m_openscenery_search, "zz_");
		return selftest_result::handled;
	}
	if (command == "openscenery")
	{
		std::string name;
		words >> name;
		WriteLog(restart_editor(name) ? "SELFTEST openscenery: started " + name : "SELFTEST openscenery: failed");
		return selftest_result::handled;
	}
	if (command == "undo")
	{
		undo_last();
		std::string name;
		words >> name;
		WriteLog("SELFTEST undo" + record_after_undo(name));
		return selftest_result::handled;
	}
	if (command == "redo")
	{
		redo_last();
		WriteLog("SELFTEST redo");
		return selftest_result::handled;
	}
	if (command == "log")
	{
		std::string text;
		std::getline(words, text);
		WriteLog("SELFTEST" + text);
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_scene(std::string const &command, std::istringstream &words)
{
	// the commands of the view and the tracks go ahead of those of the scripted input, the rest after them
	if (auto const viewed{selftest_scene_view(command, words)}; viewed != selftest_result::unhandled)
		return viewed;
	if (m_selftest.input.dispatch(command, words))
		return selftest_result::handled;
	if (auto const filed{selftest_scene_files(command, words)}; filed != selftest_result::unhandled)
		return filed;
	if (auto const noded{selftest_scene_nodes(command, words)}; noded != selftest_result::unhandled)
		return noded;
	return selftest_scene_other(command, words);
}
