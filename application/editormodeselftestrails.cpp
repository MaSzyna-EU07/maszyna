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
#include <fstream>
#include <map>
#include <sstream>
#include "application/editormodeselftest.h"

editor_mode::selftest_result editor_mode::selftest_rails(std::string const &command, std::istringstream &words)
{
	if (command == "blade")
	{
		std::string name;
		words >> name;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->SwitchExtension == nullptr)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			auto const vertices = [&]() {
				return std::make_pair(track->Geometry1.empty() ? std::size_t{0} : GfxRenderer->Vertices(track->Geometry1[0]).size(), track->Geometry2.empty() ? std::size_t{0} : GfxRenderer->Vertices(track->Geometry2[0]).size());
			};
			auto const set = [&](int const Path) {
				track->Switch(Path);
				track->SwitchExtension->fOffset = track->SwitchExtension->fDesiredOffset;
				track->RaAnimate();
			};
			set(0);
			auto const straight{vertices()};
			set(1);
			auto const diverging{vertices()};
			set(0);
			auto const frog{track->frog()};
			auto const radius{std::abs(track->m_paths.size() > 1 ? track->m_paths[1].radius : 0.f)};
			for (int path = 0; path < 2; ++path)
			{
				auto const &shape{track->blade(path)};
				auto const &segment{*track->SwitchExtension->Segments[path]};
				auto const old{segment.GetLength() * track_rails::blade_steps(*track, path) / segment.RaSegCount()};
				auto const throat{frog.valid ? frog.at[path] - track_rails::flangeway / std::tan(frog.angle) : 0.0};
				auto const at = [&](double const Width) {
					for (std::size_t i = 1; i < shape.gauge.size(); ++i)
						if (shape.gauge[i].y >= Width)
							return glm::mix(shape.gauge[i - 1].x, shape.gauge[i].x, (Width - shape.gauge[i - 1].y) / std::max(shape.gauge[i].y - shape.gauge[i - 1].y, 1e-9));
					return -1.0;
				};
				auto const gauge = [&](double const Distance) {
					for (std::size_t i = 1; i < shape.gauge.size(); ++i)
						if (shape.gauge[i].x >= Distance)
							return glm::mix(shape.gauge[i - 1].y, shape.gauge[i].y, (Distance - shape.gauge[i - 1].x) / std::max(shape.gauge[i].x - shape.gauge[i - 1].x, 1e-9));
					return shape.gauge.empty() ? 0.0 : shape.gauge.back().y;
				};
				std::string samples;
				auto worst{0.0};
				for (auto const distance : {1.0, 2.0, 4.0, 6.0, 8.0, 10.0})
				{
					auto const analytic{radius > 0.f ? distance * distance / (2.0 * radius) : 0.0};
					samples += format(" %.0f:%.4f/%.4f", distance, gauge(distance), analytic);
					if (shape.valid && distance <= shape.heel)
						worst = std::max(worst, std::abs(gauge(distance) - analytic));
				}
				std::string faults;
				if (track_rails::cut_rails(*track) && false == shape.valid)
					faults += " FAULT blade not cut from the rails";
				if (shape.valid && shape.opening < track_rails::blade_clearance - 1e-4)
					faults += " FAULT flangeway";
				if (shape.valid && frog.valid && shape.heel >= throat)
					faults += " FAULT heel past the throat";
				if (straight != diverging)
					faults += " FAULT vertex count changes";
				WriteLog(format("SELFTEST blade %s path %d: %s, R %.0f, tip %.3f, 50mm %.3f, full head %.3f, heel %.3f (old %.3f), drop %.4f, opening %.4f at %.3f, throat %.3f, d(x)/x^2/2R%s, worst %.4f, vertices %zu/%zu %zu/%zu%s", track->name().c_str(), path, shape.valid ? (shape.rigid ? "geometric rigid" : "geometric") : "old", radius, shape.tip, at(track_rails::blade_lowered + track_rails::blade_gap), shape.full, shape.heel, old, shape.drop, shape.opening, shape.openingat,
				                throat, samples.c_str(), worst, straight.first, diverging.first, straight.second, diverging.second, faults.c_str()));
			}
		}
		return selftest_result::handled;
	}
	if (command == "frog")
	{
		std::string name;
		words >> name;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->SwitchExtension == nullptr)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			auto const frog{track->frog()};
			if (false == frog.valid)
			{
				WriteLog(format("SELFTEST frog %s: no frog, paths %.3f and %.3f m, blades %.3f m", track->name().c_str(), track->SwitchExtension->Segments[0]->GetLength(), track->SwitchExtension->Segments[1]->GetLength(), frog.blades[0]));
				continue;
			}
			auto const n{1.0 / std::tan(frog.angle)};
			glm::dvec3 point, across;
			track->SwitchExtension->Segments[0]->plan_frame(frog.at[0], point, across);
			auto const gauge{point + across * (frog.side[0] * 0.5 * std::abs(track->fTrackWidth))};
			auto const slope{std::tan(frog.angle)};
			auto const check{0.5 * track_rails::check_length(slope)};
			WriteLog(format("SELFTEST frog %s: crossing at %.3f / %.3f m (%.3f %.3f %.3f), 1:%.2f, side %d, blades %.3f / %.3f m, throat %.3f, nose %.3f, wing end %.3f, check rails %.3f-%.3f / %.3f-%.3f m", track->name().c_str(),
			    frog.at[0], frog.at[1], gauge.x, gauge.y, gauge.z, n, frog.side[0], frog.blades[0], frog.blades[1], frog.at[0] - track_rails::flangeway / slope, frog.at[0] + track_rails::nose_tip / slope, frog.at[0] + track_rails::wing_reach(slope),
			    frog.at[0] - 0.25 - check, frog.at[0] - 0.25 + check, frog.at[1] - 0.25 - check, frog.at[1] - 0.25 + check));
		}
		return selftest_result::handled;
	}
	if (command == "beddump")
	{
		// the trackbed triangles and the axes of the paths of the tracks near a point, in plan, for a look at where the trackbed has holes
		glm::dvec3 point{0.0};
		double radius{30.0};
		std::string file;
		words >> point.x >> point.z >> radius >> file;
		std::ofstream output{file, std::ios_base::trunc};
		auto triangles{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1 || track->m_paths.empty())
				continue;
			if (geometry::plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, point)), point) > radius)
				continue;
			auto const paths{track->eType == tt_Switch ? 2 : 1};
			for (int path = 0; path < paths; ++path)
			{
				auto const &segment{track->eType == tt_Switch ? track->SwitchExtension->Segments[path] : track->Segment};
				if (segment == nullptr)
					continue;
				output << "axis " << track->name() << ' ' << path;
				for (auto at{0.0}; at <= segment->GetLength() + 1e-6; at += 0.25)
				{
					glm::dvec3 spot, across;
					segment->plan_frame(std::min(at, segment->GetLength()), spot, across);
					output << format(" %.4f %.4f", spot.x, spot.z);
				}
				output << "\n";
			}
			std::vector<gfx::geometry_handle> beds;
			if (track->eType == tt_Switch && track->SwitchExtension != nullptr)
				beds.push_back(track->SwitchExtension->Geometry3);
			else
				beds.insert(beds.end(), track->Geometry2.begin(), track->Geometry2.end());
			for (auto const &bed : beds)
			{
				if (bed == null_handle)
					continue;
				auto const &vertices{GfxRenderer->Vertices(bed)};
				for (std::size_t i = 2; i < vertices.size(); ++i)
				{
					glm::dvec3 const a{glm::dvec3{vertices[i - 2].position} + track->m_origin};
					glm::dvec3 const b{glm::dvec3{vertices[i - 1].position} + track->m_origin};
					glm::dvec3 const c{glm::dvec3{vertices[i].position} + track->m_origin};
					output << format("tri %s %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", track->name().c_str(), a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
					++triangles;
				}
			}
		}
		WriteLog(format("SELFTEST beddump: %d triangles into %s", triangles, file.c_str()));
		return selftest_result::handled;
	}
	if (command == "raildump")
	{
		// the rail geometry of the tracks near a point as drawn: s strips of the lofts, t triangles of the cut faces and the rest
		glm::dvec3 point{0.0};
		double radius{30.0};
		std::string file;
		words >> point.x >> point.z >> radius >> file;
		std::ofstream output{file, std::ios_base::trunc};
		auto chunks{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1 || track->m_paths.empty())
				continue;
			if (geometry::plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, point)), point) > radius)
				continue;
			output << format("track %s %.4f\n", track->name().c_str(), track->m_paths.front().points[segment_data::point::start].y);
			auto const dump = [&](TTrack::geometryhandle_sequence const &Sequence, char const *List, char const Kind) {
				for (std::size_t index = 0; index < Sequence.size(); ++index)
				{
					auto const &vertices{GfxRenderer->Vertices(Sequence[index])};
					if (vertices.empty())
						continue;
					auto const primitive{GfxRenderer->Primitive(Sequence[index])};
					auto const kind{primitive == GL_TRIANGLES ? 't' : primitive == GL_TRIANGLE_STRIP ? 's' : Kind};
					output << format("chunk %s %s %zu %c %zu\n", track->name().c_str(), List, index, kind, vertices.size());
					for (auto const &vertex : vertices)
					{
						glm::dvec3 const at{glm::dvec3{vertex.position} + track->m_origin};
						output << format("%.4f %.4f %.4f\n", at.x, at.y, at.z);
					}
					++chunks;
				}
			};
			dump(track->Geometry1, "rails1", 's');
			dump(track->Geometry2, track->eType == tt_Switch ? "rails2" : "bed", 's');
			dump(track->GeometryCut1, "cut1", 't');
			dump(track->GeometryCut2, "cut2", 't');
			for (std::size_t i = 0; i < track->GeometryTreads.size(); ++i)
			{
				dump(track->GeometryTreads[i].geometry, "tread", 's');
				dump(track->GeometryTreads[i].cuts, "treadcut", 't');
			}
		}
		WriteLog(format("SELFTEST raildump: %d chunks into %s", chunks, file.c_str()));
		return selftest_result::handled;
	}
	if (command == "geomhash")
	{
		std::string name;
		words >> name;
		auto const mix = [](std::uint64_t &Hash, float const Value) {
			Hash = (Hash ^ static_cast<std::uint64_t>(static_cast<std::int64_t>(std::llround(Value * 1e4)))) * 1099511628211ull;
		};
		auto const exact = [](std::uint64_t &Hash, float const Value) {
			std::uint32_t bits;
			std::memcpy(&bits, &Value, sizeof(bits));
			Hash = (Hash ^ bits) * 1099511628211ull;
		};
		auto const named = [](std::uint64_t &Hash, material_handle const Material) {
			for (auto const letter : (Material != null_handle ? GfxRenderer->Material(Material)->GetName() : std::string{}) + '|')
				Hash = (Hash ^ static_cast<unsigned char>(letter)) * 1099511628211ull;
		};
		std::uint64_t total{1469598103934665603ull};
		std::uint64_t exacttotal{1469598103934665603ull};
		std::size_t vertices{0};
		int tracks{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			std::uint64_t hash{1469598103934665603ull};
			std::uint64_t exacthash{1469598103934665603ull};
			std::size_t count{0};
			int chunks{0};
			std::vector<TTrack::geometryhandle_sequence const *> sequences{&track->Geometry1, &track->Geometry2, &track->GeometryCut1, &track->GeometryCut2};
			for (auto const &tread : track->GeometryTreads)
			{
				sequences.push_back(&tread.geometry);
				sequences.push_back(&tread.cuts);
				named(exacthash, tread.material);
				named(exacthash, tread.matte);
			}
			for (auto const *sequence : sequences)
			{
				for (auto const &geometry : *sequence)
				{
					// an empty piece of a rail gets no chunk, as the renderer skips it
					if (geometry == null_handle)
						continue;
					++chunks;
					for (auto const &vertex : GfxRenderer->Vertices(geometry))
					{
						for (int i = 0; i < 3; ++i)
						{
							mix(hash, vertex.position[i]);
							mix(hash, vertex.normal[i]);
							exact(exacthash, vertex.position[i]);
							exact(exacthash, vertex.normal[i]);
						}
						mix(hash, vertex.texture.x);
						mix(hash, vertex.texture.y);
						exact(exacthash, vertex.texture.x);
						exact(exacthash, vertex.texture.y);
						++count;
					}
					for (auto const &userdata : GfxRenderer->UserData(geometry))
					{
						for (int i = 0; i < 4; ++i)
							exact(exacthash, userdata.data[i]);
					}
					exact(exacthash, -2.f);
				}
				mix(hash, -1.f);
				exact(exacthash, -1.f);
			}
			WriteLog(format("SELFTEST geom %s: chunks %d, vertices %zu, hash %016llx, exact %016llx", track->name().c_str(), chunks, count, static_cast<unsigned long long>(hash), static_cast<unsigned long long>(exacthash)));
			total = (total ^ hash) * 1099511628211ull;
			exacttotal = (exacttotal ^ exacthash) * 1099511628211ull;
			vertices += count;
			++tracks;
		}
		WriteLog(format("SELFTEST geom total: tracks %d, vertices %zu, hash %016llx, exact %016llx", tracks, vertices, static_cast<unsigned long long>(total), static_cast<unsigned long long>(exacttotal)));
		return selftest_result::handled;
	}
	if (command == "switchtimbers")
	{
		std::string model;
		words >> model;
		if (model == "none")
			model.clear();
		WriteLog(format("SELFTEST switchtimbers %s: %zu tracks", model.c_str(), set_switch_timbers(model)));
		return selftest_result::handled;
	}
	if (command == "frogkey")
	{
		std::string name, value;
		words >> name >> value;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr || track->eType != tt_Switch || track->SwitchExtension == nullptr)
			WriteLog(format("SELFTEST frogkey %s: no such switch", name.c_str()));
		else
		{
		if (value == "on" || value == "off")
		{
			auto const before{editor_track::capture(*track)};
			track->SwitchExtension->m_frogs = (value == "on");
			push_track_snapshot({{track, before}});
			editor_track::commit_parameters(*track);
		}
		std::ostringstream text;
		track->export_as_text(text);
		WriteLog(format("SELFTEST frogkey %s: key %d, written %d, frog %d, crossings %zu, profile '%s'", name.c_str(), track->SwitchExtension->m_frogs ? 1 : 0, text.str().find(" frogs ") != std::string::npos ? 1 : 0,
		    track->frog().valid ? 1 : 0, track->rail_crossings().size(), track->m_profile1.first.c_str()));
		}
		return selftest_result::handled;
	}
	if (command == "headgap")
	{
		// the least gap between the heads of the rails of two tracks running side by side, by the gauge lines and the head of the profile
		std::string first, second;
		words >> first >> second;
		auto *a{simulation::Paths.find(first)};
		auto *b{simulation::Paths.find(second)};
		if (a == nullptr || b == nullptr)
		{
			WriteLog("SELFTEST headgap: no such tracks");
			return selftest_result::handled;
		}
		auto const head_a{a->rail_dimensions().head};
		auto const head_b{b->rail_dimensions().head};
		for (int pa = 0; pa < (a->eType == tt_Switch ? 2 : 1); ++pa)
			for (auto const sa : {-1, 1})
				for (int pb = 0; pb < (b->eType == tt_Switch ? 2 : 1); ++pb)
					for (auto const sb : {-1, 1})
					{
						auto const length{a->eType == tt_Switch ? a->SwitchExtension->Segments[pa]->GetLength() : a->Segment->GetLength()};
						auto const lengthb{b->eType == tt_Switch ? b->SwitchExtension->Segments[pb]->GetLength() : b->Segment->GetLength()};
						auto least{1e9};
						auto where{0.0};
						for (auto at{0.0}; at <= length; at += 0.02)
						{
							double d, l;
							track_rails::rail_offset(*b, pb, sb, track_rails::rail_point(*a, pa, sa, at, head_a), d, l);
							if (d < 0.0 || d > lengthb || l < -0.3 || l > 0.3)
								continue;
							if (l - head_b < least)
							{
								least = l - head_b;
								where = at;
							}
						}
						if (least < 1e8)
							WriteLog(format("SELFTEST headgap %s/%d%+d to %s/%d%+d: %.1f mm at %.2f m (heads %.1f %.1f mm, width %.4f)", first.c_str(), pa, sa, second.c_str(), pb, sb, least * 1000.0, where, head_a * 1000.0,
							                head_b * 1000.0, a->fTrackWidth));
					}
		return selftest_result::handled;
	}
	if (command == "checks")
	{
		// check rails of a track as they're drawn, and the rails of the tracks they run into
		std::string name;
		words >> name;
		int count{0};
		int faults{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1 || track->name() != name)
				continue;
			auto const layout{track->plan_rails(true)};
			auto const size{track->rail_dimensions()};
			std::vector<TTrack *> others;
			for (auto *other : simulation::Paths.sequence())
				if (other != nullptr && false == other->m_editorremoved && (other->iCategoryFlag & 15) == 1 && glm::length(other->location() - track->location()) < other->radius() + track->radius() + 1.0)
					others.push_back(other);
			for (int path = 0; path < (track->eType == tt_Switch ? 2 : 1); ++path)
				for (int right = 0; right < 2; ++right)
				{
					auto const side{right ? 1 : -1};
					for (auto const &check : layout.checks[path][right])
					{
						if (track_rails::replaced_by_wing(check, layout.wingends[path][right]))
							continue;
						auto const ownlength{track->eType == tt_Switch ? track->SwitchExtension->Segments[path]->GetLength() : track->Segment->GetLength()};
						auto const from{std::max(check.from, 0.0)};
						auto const to{std::min(check.to, ownlength)};
						if (to - from <= 0.01)
							continue;
						++count;
						std::string hits;
						for (auto at{from}; at <= to; at += 0.05)
							// the head of the check rail as drawn, 40 mm at the head of the rail of 73.5
							for (auto const lateral : {-track_rails::check_flangeway, -track_rails::check_flangeway - 0.020 * size.head / 0.0735, -track_rails::check_flangeway - 0.040 * size.head / 0.0735})
							{
								auto const point{track_rails::rail_point(*track, path, side, at, lateral)};
								for (auto *other : others)
									for (int otherpath = 0; otherpath < (other->eType == tt_Switch ? 2 : 1); ++otherpath)
										for (auto const otherside : {-1, 1})
										{
											if (other == track && otherpath == path && otherside == side)
												continue;
											auto const length{other->eType == tt_Switch ? other->SwitchExtension->Segments[otherpath]->GetLength() : other->Segment->GetLength()};
											double d, l;
											track_rails::rail_offset(*other, otherpath, otherside, point, d, l);
											if (d < 0.0 || d > length || l < 0.0 || l > other->rail_dimensions().head)
												continue;
											auto const hit{format(" %s/%d%+d", other->name().c_str(), otherpath, otherside)};
											if (hits.find(hit) == std::string::npos)
												hits += hit + format(" at %.2f m", at);
										}
							}
						if (false == hits.empty())
							++faults;
						WriteLog(format("SELFTEST check %s path %d side %+d: %.3f-%.3f m drawn of %.3f-%.3f m%s", name.c_str(), path, side, from, to, check.from, check.to, hits.empty() ? ", clear" : (", over" + hits + " FAULT").c_str()));
					}
				}
		}
		WriteLog(format("SELFTEST checks: %d check rails, %d faults", count, faults));
		return selftest_result::handled;
	}
	if (command == "cuts")
	{
		// the cuts of the rails of a track, with the ends of the parts in the world
		std::string name, detail;
		words >> name >> detail;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr)
			return selftest_result::handled;
		auto const layout{track->plan_rails(true)};
		auto const head{track->rail_dimensions().head};
		// with knots: the lines of the parts too
		auto const knots_of = [](knots::line const &Line) {
			std::string result;
			for (auto const &knot : Line)
				result += format(" %.3f:%.4f", knot.x, knot.y);
			return result;
		};
		for (int path = 0; path < (track->eType == tt_Switch ? 2 : 1); ++path)
			for (int right = 0; right < 2; ++right)
				for (auto const &cut : layout.plans[path][right].cuts)
				{
					auto const side{right ? 1 : -1};
					auto const where = [&](double const At, knots::line const &Offsets) {
						auto const point{track_rails::rail_point(*track, path, side, At, knots::sample(Offsets, At, 0.0) + 0.5 * head)};
						return format("%.3f m (%.3f %.3f) offset %.3f", At, point.x, point.z, knots::sample(Offsets, At, 0.0));
					};
					WriteLog(format("SELFTEST cut %s path %d side %+d at %.3f: end %s%s, start %s%s, of %.3f m", name.c_str(), path, side, cut.at, where(cut.end, cut.endoffsets).c_str(), cut.endextents.empty() ? "" : " extents", where(cut.start, cut.startoffsets).c_str(), cut.startextents.empty() ? "" : " extents", track_rails::path_segment(*track, path).GetLength()));
					if (detail == "knots")
						for (auto const &[label, line] : std::initializer_list<std::pair<char const *, knots::line const *>>{
								 {"endoffsets", &cut.endoffsets}, {"endextents", &cut.endextents}, {"endinner", &cut.endinner}, {"endrolled", &cut.endrolled}, {"endworn", &cut.endworn},
								 {"startoffsets", &cut.startoffsets}, {"startextents", &cut.startextents}, {"startinner", &cut.startinner}, {"startrolled", &cut.startrolled}, {"startworn", &cut.startworn}})
							if (false == line->empty())
								WriteLog(format("SELFTEST   %s%s", label, knots_of(*line).c_str()));
				}
		return selftest_result::handled;
	}
	if (command == "wings")
	{
		// the heads of the rails bent out at the cuts, the wing rails mostly, against the heads of the other rails as they're laid out
		std::string name;
		words >> name;
		std::map<TTrack const *, track_rails::rail_layout> layouts;
		auto const layout_of = [&](TTrack *Track) -> track_rails::rail_layout const & {
			auto found{layouts.find(Track)};
			if (found == layouts.end())
				found = layouts.emplace(Track, Track->plan_rails(true)).first;
			return found->second;
		};
		auto const length_of = [](TTrack const &Track, int const Path) { return Track.eType == tt_Switch ? Track.SwitchExtension->Segments[Path]->GetLength() : Track.Segment->GetLength(); };
		// whether the head of a rail lies at a distance along it and out from its gauge line, the part of the nearest cut there
		auto const occupied = [](track_rails::rail_plan const &Plan, double const Head, double const At, double const Lateral) {
			if (Plan.cuts.empty())
				return Lateral >= 0.0 && Lateral <= Head;
			auto const *cut{&Plan.cuts.front()};
			for (auto const &each : Plan.cuts)
				if (std::abs(each.at - At) < std::abs(cut->at - At))
					cut = &each;
			auto const part = [&](knots::line const &Offsets, knots::line const &Extents, knots::line const &Inner) {
				auto const offset{knots::sample(Offsets, At, 0.0)};
				return Lateral >= offset + std::max(0.0, knots::sample(Inner, At, 0.0)) && Lateral <= offset + std::min(Head, knots::sample(Extents, At, Head));
			};
			return (At <= cut->end && part(cut->endoffsets, cut->endextents, cut->endinner)) || (At >= cut->start && part(cut->startoffsets, cut->startextents, cut->startinner));
		};
		int count{0};
		int faults{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1 || track->m_paths.empty())
				continue;
			if (name != "all" && track->name() != name)
				continue;
			auto const &layout{layout_of(track)};
			auto const head{track->rail_dimensions().head};
			std::vector<TTrack *> others;
			for (auto *other : simulation::Paths.sequence())
				if (other != nullptr && false == other->m_editorremoved && (other->iCategoryFlag & 15) == 1 && false == other->m_paths.empty() && glm::length(other->location() - track->location()) < other->radius() + track->radius() + 1.0)
					others.push_back(other);
			for (int path = 0; path < (track->eType == tt_Switch ? 2 : 1); ++path)
				for (int right = 0; right < 2; ++right)
				{
					auto const side{right ? 1 : -1};
					auto const ownlength{length_of(*track, path)};
					for (auto const &cut : layout.plans[path][right].cuts)
						for (auto const coming : {true, false})
						{
							auto const &offsets{coming ? cut.endoffsets : cut.startoffsets};
							if (offsets.size() < 2 || offsets.back().x - offsets.front().x < 0.05)
								continue;
							auto const from{std::max(0.0, coming ? offsets.front().x : std::max(offsets.front().x, cut.start))};
							auto const to{std::min(ownlength, coming ? std::min(offsets.back().x, cut.end) : offsets.back().x)};
							if (to - from < 0.05)
								continue;
							++count;
							std::string hits;
							for (auto at{from}; at <= to; at += 0.02)
							{
								auto const offset{knots::sample(offsets, at, 0.0)};
								if (offset <= 0.002)
									continue;
								// only what is left of the head where the wing rail is planed
								auto const lo{offset + std::max(0.0, knots::sample(coming ? cut.endinner : cut.startinner, at, 0.0))};
								auto const hi{offset + std::min(head, knots::sample(coming ? cut.endextents : cut.startextents, at, head))};
								if (hi - lo < 0.012)
									continue;
								for (auto const lateral : {lo + 0.005, 0.5 * (lo + hi), hi - 0.005})
								{
									auto const point{track_rails::rail_point(*track, path, side, at, lateral)};
									for (auto *other : others)
										for (int otherpath = 0; otherpath < (other->eType == tt_Switch ? 2 : 1); ++otherpath)
											for (auto const otherside : {-1, 1})
											{
												if (other == track && otherpath == path && otherside == side)
													continue;
												double d, l;
												track_rails::rail_offset(*other, otherpath, otherside, point, d, l);
												if (d < 0.0 || d > length_of(*other, otherpath) || l < -0.3 || l > 0.3)
													continue;
												// a wing rail planed flush against another head from the outside covers its rounded shoulder
												auto const theirs{other->rail_dimensions()};
												if (false == occupied(layout_of(other).plans[otherpath][otherside > 0], theirs.head - theirs.shoulder, d, l))
													continue;
												auto const hit{format(" %s/%d%+d", other->name().c_str(), otherpath, otherside)};
												if (hits.find(hit) == std::string::npos)
													hits += hit + format(" at %.2f m (theirs %.2f m, %.0f mm in)", at, d, l * 1000.0);
											}
								}
							}
							if (false == hits.empty())
								++faults;
							if (false == hits.empty() || name != "all")
								WriteLog(format("SELFTEST wing %s path %d side %+d cut at %.3f, %s part %.3f-%.3f m, out %.0f-%.0f mm%s", track->name().c_str(), path, side, cut.at, coming ? "coming" : "going", from, to, offsets.front().y * 1000.0,
								                offsets.back().y * 1000.0, hits.empty() ? ", clear" : (", over" + hits + " FAULT").c_str()));
						}
				}
		}
		WriteLog(format("SELFTEST wings: %d bent parts, %d faults", count, faults));
		return selftest_result::handled;
	}
	if (command == "space")
	{
		// the rails as laid out, every 2 cm along their parts: a head in the flangeway of another rail where its wheels run (F), over the head of
		// another rail past its shoulder (O), a line of a part planing on past its last knot (C), the head jumping in width (J), nothing left of the
		// head inside a part (V)
		std::string name;
		double tolerance{0.5};
		words >> name >> tolerance;
		if (name.empty())
			name = "all";
		tolerance *= 0.001;
		struct piece
		{
			double from{-1e9};
			double to{1e9};
			track_rails::rail_cut const *before{nullptr};
			track_rails::rail_cut const *after{nullptr};
		};
		struct rail
		{
			TTrack *track{nullptr};
			int path{0};
			int side{0};
			double length{0.0};
			double laidfrom{0.0};
			double heel{0.0};
			track_rails::rail_size size;
			track_rails::gauge_line const *line{nullptr};
			std::vector<piece> pieces;
		};
		std::map<TTrack const *, track_rails::rail_layout> layouts;
		std::map<TTrack const *, std::vector<rail>> rails;
		auto const rails_of = [&](TTrack *Track) -> std::vector<rail> & {
			auto found{rails.find(Track)};
			if (found != rails.end())
				return found->second;
			auto &result{rails[Track]};
			auto const &layout{layouts.emplace(Track, Track->plan_rails(true)).first->second};
			auto const heel{Track->eType == tt_Switch ? std::max(track_rails::blade_length(*Track, 0), track_rails::blade_length(*Track, 1)) : 0.0};
			for (int path = 0; path < track_rails::path_count(*Track); ++path)
				for (int right = 0; right < 2; ++right)
				{
					rail each;
					each.track = Track;
					each.path = path;
					each.side = (right ? 1 : -1);
					each.length = track_rails::path_segment(*Track, path).GetLength();
					each.laidfrom = (Track->eType == tt_Switch && each.side == (Track->SwitchExtension->RightSwitch == (path == 0) ? 1 : -1) ? track_rails::blade_length(*Track, path) : 0.0);
					each.heel = heel;
					each.size = Track->rail_dimensions();
					each.line = &track_rails::rail_line(*Track, path, right, false);
					std::vector<track_rails::rail_cut const *> cuts;
					for (auto const &cut : layout.plans[path][right].cuts)
						cuts.push_back(&cut);
					std::stable_sort(cuts.begin(), cuts.end(), [](track_rails::rail_cut const *A, track_rails::rail_cut const *B) { return A->at < B->at; });
					for (std::size_t j = 0; j <= cuts.size(); ++j)
					{
						piece part;
						part.before = (j > 0 ? cuts[j - 1] : nullptr);
						part.after = (j < cuts.size() ? cuts[j] : nullptr);
						if (part.before != nullptr)
							part.from = part.before->start;
						if (part.after != nullptr)
							part.to = part.after->end;
						each.pieces.push_back(part);
					}
					result.push_back(each);
				}
			return result;
		};
		knots::line const none;
		struct lines
		{
			knots::line const *offsets;
			knots::line const *extents;
			knots::line const *inner;
		};
		// the lines of the part from the nearer of its cuts
		auto const lines_at = [&](piece const &Part, double const S) -> lines {
			auto const before{Part.before != nullptr && (Part.after == nullptr || std::abs(S - Part.before->at) <= std::abs(S - Part.after->at))};
			if (before)
				return {&Part.before->startoffsets, &Part.before->startextents, &Part.before->startinner};
			if (Part.after != nullptr)
				return {&Part.after->endoffsets, &Part.after->endextents, &Part.after->endinner};
			return {&none, &none, &none};
		};
		auto const head_of = [&](rail const &Rail, piece const &Part, double const S, double &Offset, double &Low, double &High) {
			auto const l{lines_at(Part, S)};
			Offset = knots::sample(*l.offsets, S, 0.0);
			Low = Offset + std::max(0.0, knots::sample(*l.inner, S, 0.0));
			High = Offset + std::min(Rail.size.head, knots::sample(*l.extents, S, Rail.size.head));
		};
		// whether the head of a rail covers a point out from its gauge line, past its rounded shoulder and Tolerance
		auto const covers = [&](rail const &Rail, double const D, double const L) {
			for (auto const &part : Rail.pieces)
			{
				if (D < part.from || D > part.to)
					continue;
				double offset, low, high;
				head_of(Rail, part, D, offset, low, high);
				if (L > low + tolerance && L < std::min(high, offset + Rail.size.head - Rail.size.shoulder) - tolerance)
					return true;
			}
			return false;
		};
		// the parts near an obtuse crossing of either rail are designed to meet
		auto const obtuse_near = [](piece const &Part, double const S) {
			for (auto const *cut : {Part.before, Part.after})
				if (cut != nullptr && cut->obtuse && std::abs(S - cut->at) < 0.3)
					return true;
			return false;
		};
		auto const throwable = [](piece const &Part) { return Part.before != nullptr && Part.before->throwable; };
		int counts[5]{};
		int parts{0};
		char const kinds[]{"FOCJV"};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1 || track->m_paths.empty() || false == track_rails::cut_rails(*track) || track->rail_dimensions().grooved)
				continue;
			if (name != "all" && track->name() != name)
				continue;
			std::vector<TTrack *> others;
			for (auto *other : simulation::Paths.sequence())
				if (other != nullptr && false == other->m_editorremoved && (other->iCategoryFlag & 15) == 1 && false == other->m_paths.empty() && track_rails::cut_rails(*other) && false == other->rail_dimensions().grooved &&
				    glm::length(other->location() - track->location()) < other->radius() + track->radius() + 1.0)
					others.push_back(other);
			for (auto const &ours : rails_of(track))
				for (auto const &part : ours.pieces)
				{
					auto const from{std::max({part.from, ours.laidfrom, 0.0})};
					auto const to{std::min(part.to, ours.length)};
					if (to - from < 0.04)
						continue;
					++parts;
					bool found[5]{};
					std::string where[5];
					auto const fault = [&](int const Kind, std::string const &What) {
						if (false == found[Kind])
						{
							found[Kind] = true;
							where[Kind] = What;
						} };
					// past the last knots of its lines, the lines planing on as they end
					for (auto s{from}; s <= to + 1e-9; s += 0.02)
					{
						auto const l{lines_at(part, s)};
						for (auto const *line : {l.extents, l.inner})
						{
							if (line->empty())
								continue;
							auto const value{knots::sample(*line, s, 0.0)};
							auto const planing{line == l.extents ? value < ours.size.head - 1e-4 : value > 1e-4};
							// the lines have knots every 2 cm at most
							if (planing && (s > line->back().x + 0.025 || s < line->front().x - 0.025))
								fault(2, format("%s at %.2f m, past its knots %.2f-%.2f m", line == l.extents ? "extents" : "inner", s, line->front().x, line->back().x));
						}
					}
					// along the part: nothing of the head left, the head jumping in width
					auto start{from + 0.05};
					auto end{to - 0.05};
					if (part.before != nullptr && part.before->startkind == track_rails::rail_end::sharp)
						for (auto s{from}; s < to; s += 0.02)
						{
							double offset, low, high;
							head_of(ours, part, s, offset, low, high);
							if (high - low >= 0.001)
							{
								start = std::max(start, s + 0.04);
								break;
							}
						}
					std::vector<double> stations, widths;
					for (auto s{std::ceil(start / 0.02) * 0.02}; s <= end; s += 0.02)
					{
						double offset, low, high;
						head_of(ours, part, s, offset, low, high);
						auto const width{std::max(0.0, high - low)};
						if (width < 0.001 && false == obtuse_near(part, s))
							fault(4, format("at %.2f m, %.1f mm left", s, width * 1000.0));
						stations.push_back(s);
						widths.push_back(width);
					}
					// a step in width well past those around it: a head tapering off steeply at a wide angle aside
					for (std::size_t i = 1; i < widths.size(); ++i)
					{
						auto const step{std::abs(widths[i] - widths[i - 1])};
						auto const around{std::max(i > 1 ? std::abs(widths[i - 1] - widths[i - 2]) : 0.0, i + 1 < widths.size() ? std::abs(widths[i + 1] - widths[i]) : 0.0)};
						if (step > 0.003 && step > 2.0 * around + 0.001 && false == obtuse_near(part, stations[i]))
							fault(3, format("at %.2f m, %.1f to %.1f mm", stations[i], widths[i - 1] * 1000.0, widths[i] * 1000.0));
					}
					// against the rails around
					for (auto *other : others)
						for (auto const &theirs : rails_of(other))
						{
							if (theirs.track == ours.track && theirs.path == ours.path)
								continue;
							auto const &low{theirs.line->low};
							auto const &high{theirs.line->high};
							auto distance{-1e9};
							for (auto s{std::ceil(from / 0.02) * 0.02}; s <= to; s += 0.02)
							{
								double offset, lo, hi;
								head_of(ours, part, s, offset, lo, hi);
								if (hi - lo < 2.0 * tolerance)
									continue;
								auto const centre{track_rails::rail_point(*ours.track, ours.path, ours.side, s, offset + 0.5 * ours.size.head)};
								if (centre.x < low.x - 0.5 || centre.x > high.x + 0.5 || centre.z < low.y - 0.5 || centre.z > high.y + 0.5)
									continue;
								if (distance < -1e8)
									distance = track_rails::nearest_distance(*theirs.line, centre);
								auto const across{track_rails::lateral(*theirs.line, theirs.side, centre, distance)};
								if (distance < 0.0 || distance > theirs.length || std::abs(across) > 0.3)
									continue;
								if (theirs.track == ours.track && (theirs.side == ours.side && distance < theirs.heel + 0.1 || throwable(part)))
									continue;
								// the same rail going on past a joint
								{
									auto d{distance};
									if (std::abs(track_rails::lateral(*theirs.line, theirs.side, track_rails::rail_point(*ours.track, ours.path, ours.side, s, 0.0), d)) < 0.002)
										continue;
								}
								// the head of ours keeps out of the flangeways, the top of it out from under other heads: their rounded shoulders lie under
								// it, ours under theirs
								auto const outer{hi - tolerance};
								auto const crown{std::min(hi, offset + ours.size.head - ours.size.shoulder) - tolerance};
								for (auto const lateral : {lo + tolerance, 0.5 * (lo + crown), crown, outer})
								{
									auto const top{lateral <= crown + 1e-9};
									auto const point{track_rails::rail_point(*ours.track, ours.path, ours.side, s, lateral)};
									auto d{distance};
									auto const l{track_rails::lateral(*theirs.line, theirs.side, point, d)};
									// rails meeting end on at joints aside
									if (d < 0.05 || d > theirs.length - 0.05 || s < 0.05 || s > ours.length - 0.05)
										continue;
									if (l > -track_rails::flangeway + tolerance && l < -tolerance)
										fault(0, format("at %.2f m, %.1f mm into the flangeway of %s/%d%+d at %.2f m", s, (-l) * 1000.0, theirs.track->name().c_str(), theirs.path, theirs.side, d));
									if (top && false == obtuse_near(part, s) && covers(theirs, d, l))
									{
										auto inobtuse{false};
										for (auto const &their : theirs.pieces)
											if (d >= their.from && d <= their.to && obtuse_near(their, d))
												inobtuse = true;
										if (false == inobtuse)
											fault(1, format("at %.2f m, over the head of %s/%d%+d at %.2f m, %.1f mm in", s, theirs.track->name().c_str(), theirs.path, theirs.side, d, l * 1000.0));
									}
								}
							}
						}
					for (int kind = 0; kind < 5; ++kind)
						if (found[kind])
						{
							++counts[kind];
							WriteLog(format("SELFTEST space %c %s/%d%+d part %.3f-%.3f m: %s", kinds[kind], ours.track->name().c_str(), ours.path, ours.side, from, to, where[kind].c_str()));
						}
				}
		}
		WriteLog(format("SELFTEST space: %d parts, F %d, O %d, C %d, J %d, V %d", parts, counts[0], counts[1], counts[2], counts[3], counts[4]));
		return selftest_result::handled;
	}
	if (command == "guards")
	{
		std::string name;
		words >> name;
		int count{0};
		int faults{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			auto const layout{track->plan_rails(true)};
			auto const size{track->rail_dimensions()};
			auto const guardoffset{-(track_rails::flangeway + size.head)};
			double const laterals[2]{size.low + guardoffset, track_rails::flangeway + size.head + size.low - 0.002 + guardoffset};
			for (int path = 0; path < 2; ++path)
			{
				for (int right = 0; right < 2; ++right)
				{
					auto const side{right ? 1 : -1};
					auto const length{track->eType == tt_Switch ? track->SwitchExtension->Segments[path]->GetLength() : track->Segment->GetLength()};
					auto const drawn = [&](glm::dvec2 const &Miter, double const Square) {
						auto const at{Miter.y == 0.0 ? Square : Miter.x + Miter.y * 0.5 * (laterals[0] + laterals[1])};
						return at > 0.0 && at < length;
					};
					for (auto const &guard : layout.guards[path][right])
					{
						if (false == drawn(guard.frommiter, guard.from) && false == drawn(guard.tomiter, guard.to) && (guard.to <= 0.0 || guard.from >= length))
							continue;
						++count;
						std::string text;
						auto const end = [&](glm::dvec2 const &Miter, bool const Flared, bool const Glued) {
							if (Miter.y == 0.0)
								return std::string{Flared ? "planed" : "square"};
							if (false == drawn(Miter, 0.0))
								return std::string{"past the end of the track"};
							glm::dvec3 points[2];
							for (int i = 0; i < 2; ++i)
								points[i] = track_rails::rail_point(*track, path, side, Miter.x + Miter.y * laterals[i], laterals[i]);
							auto best{1e9};
							double lateral{0.0}, along{0.0};
							TTrack *against{nullptr};
							int againstpath{0}, againstside{0};
							for (auto *other : layout.crossed)
								for (int otherpath = 0; otherpath < (other->eType == tt_Switch ? 2 : 1); ++otherpath)
									for (auto const otherside : {-1, 1})
									{
										double d[2], l[2];
										for (int i = 0; i < 2; ++i)
											track_rails::rail_offset(*other, otherpath, otherside, points[i], d[i], l[i]);
										if (std::abs(l[0]) > 0.5 || std::abs(l[1]) > 0.5 || std::abs(l[0] - l[1]) >= best)
											continue;
										best = std::abs(l[0] - l[1]);
										lateral = 0.5 * (l[0] + l[1]);
										along = 0.5 * (d[0] + d[1]);
										against = other;
										againstpath = otherpath;
										againstside = otherside;
									}
							if (against == nullptr)
							{
								++faults;
								return std::string{"cut along nothing FAULT"};
							}
							auto result{format("along %s/%d%+d at %.3f m, %.1f mm off its gauge line, skew %.1f mm", against->name().c_str(), againstpath, againstside, along, lateral * 1000.0, best * 1000.0)};
							auto fault{best > 0.002};
							if (Glued)
							{
								auto const theirsize{against->rail_dimensions()};
								auto const theiroffset{-(track_rails::flangeway + theirsize.head)};
								double const theirlaterals[2]{theirsize.low + theiroffset, track_rails::flangeway + theirsize.head + theirsize.low - 0.002 + theiroffset};
								auto const reach = [&](glm::dvec2 const &Miter, double const Square, bool const Latest) {
									if (Miter.y == 0.0)
										return Square;
									auto const a{Miter.x + Miter.y * theirlaterals[0]};
									auto const b{Miter.x + Miter.y * theirlaterals[1]};
									return Latest ? std::max(a, b) : std::min(a, b);
								};
								auto covered{false};
								for (auto const &theirs : against->plan_rails(true).guards[againstpath][againstside > 0])
									covered |= (along > reach(theirs.frommiter, theirs.from, false) - 0.002 && along < reach(theirs.tomiter, theirs.to, true) + 0.002);
								result += covered ? ", on its guard rail" : ", no guard rail there";
								fault |= (false == covered || std::abs(lateral + track_rails::flangeway) > 0.002);
							}
							if (fault)
							{
								++faults;
								result += " FAULT";
							}
							return result;
						};
						text += "from " + end(guard.frommiter, guard.flarefrom, false == guard.flarefrom) + "; to " + end(guard.tomiter, guard.flareto, false == guard.flareto);
						for (auto const &cut : guard.plan.cuts)
							if (cut.end > 0.0 && cut.start < length)
								text += format("; break %.3f-%.3f: ", cut.end, cut.start) + end(cut.endmiter, false, false) + " / " + end(cut.startmiter, false, false);
						WriteLog(format("SELFTEST guard %s path %d side %+d %.3f-%.3f m: %s", track->name().c_str(), path, side, guard.from, guard.to, text.c_str()));
					}
				}
			}
		}
		WriteLog(format("SELFTEST guards: %d guard rails, %d faults", count, faults));
		return selftest_result::handled;
	}
	if (command == "throw")
	{
		// throw <path> [part] [name]: the switches set over to the path, by a part of the way as in the middle of the animation
		int path{0};
		double part{1.0};
		std::string name;
		words >> path;
		if (false == static_cast<bool>(words >> part))
			part = 1.0;
		words >> name;
		int count{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->SwitchExtension == nullptr)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			auto const before{track->SwitchExtension->fOffset};
			track->Switch(path);
			track->SwitchExtension->fOffset = before + static_cast<float>(part) * (track->SwitchExtension->fDesiredOffset - before);
			track->RaAnimate();
			++count;
		}
		WriteLog(format("SELFTEST switches: %d switches set to path %d", count, path));
		return selftest_result::handled;
	}
	if (command == "railtime")
	{
		track_rails::forget();
		auto const started{std::chrono::steady_clock::now()};
		int tracks{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1)
				continue;
			track->rail_crossings(false);
			++tracks;
		}
		auto const crossed{std::chrono::steady_clock::now()};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1)
				continue;
			track->plan_rails(true);
		}
		auto const finished{std::chrono::steady_clock::now()};
		auto const crossings{std::chrono::duration<double, std::milli>(crossed - started).count()};
		auto const elapsed{std::chrono::duration<double, std::milli>(finished - started).count()};
		WriteLog(format("SELFTEST railtime: the rails of %d tracks laid out in %.1f ms, of which their crossings %.1f ms", tracks, elapsed, crossings));
		return selftest_result::handled;
	}
	if (command == "crossings")
	{
		std::string name;
		words >> name;
		auto const started{std::chrono::steady_clock::now()};
		int tracks{0};
		int found{0};
		int acutes{0};
		int middles{0};
		int mismatched{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			++tracks;
			auto const gauge = [](TTrack *Track, track_rails::rail_crossing const &Crossing) {
				auto const &segment{Track->eType == tt_Switch ? *Track->SwitchExtension->Segments[Crossing.path] : *Track->Segment};
				glm::dvec3 point, across;
				segment.plan_frame(Crossing.at, point, across);
				return point + across * (Crossing.side * 0.5 * std::abs(Track->fTrackWidth));
			};
			for (auto const &crossing : track->rail_crossings())
			{
				++found;
				auto const middle{crossing.angle > track_rails::middle_angle && crossing.angle <= track_rails::wide_angle};
				middles += middle ? 1 : 0;
				auto const kind{crossing.angle > track_rails::wide_angle ? "wide" : crossing.facing < 0.0 ? (middle ? "middle acute" : "acute") : (middle ? "middle obtuse" : "obtuse")};
				auto const where{gauge(track, crossing)};
				WriteLog(format("SELFTEST crossing %s path %d side %d at %.3f m: %s with %s, 1:%.2f, into %.3f, facing %.3f, nose %d, plan %.2f %.2f", track->name().c_str(), crossing.path, crossing.side, crossing.at, kind,
				    crossing.track->name().c_str(), 1.0 / std::tan(crossing.angle), crossing.into, crossing.facing, crossing.nose ? 1 : 0, where.x, where.z));
			}
			for (auto const &crossing : track->rail_crossings(false))
			{
				if (crossing.angle > track_rails::wide_angle || crossing.facing >= 0.0)
					continue;
				++acutes;
				auto const here{gauge(track, crossing)};
				auto partners{0};
				auto noses{crossing.nose ? 1 : 0};
				for (auto const &other : crossing.track->rail_crossings(false))
				{
					if (other.track != track || other.angle > track_rails::wide_angle || other.facing >= 0.0)
						continue;
					auto const there{gauge(crossing.track, other)};
					if (glm::length(glm::dvec2{there.x - here.x, there.z - here.z}) > 0.05)
						continue;
					++partners;
					noses += other.nose ? 1 : 0;
				}
				if (partners != 1 || noses != 1)
					++mismatched;
				WriteLog(format("SELFTEST acute %s %s path %d side %d at %.3f m with %s (%.2f %.2f %.2f): partners %d, noses %d", partners != 1 || noses != 1 ? "mismatch" : "ok", track->name().c_str(), crossing.path, crossing.side, crossing.at, crossing.track->name().c_str(), here.x, here.y, here.z, partners, noses));
			}
		}
		auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count()};
		WriteLog(format("SELFTEST crossings: %d rail crossings on %d tracks, %d of them middle, %d acute, %d mismatched, %.1f ms", found, tracks, middles, acutes, mismatched, elapsed));
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}
