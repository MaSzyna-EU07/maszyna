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

#include "world/TrackSleepers.h"
#include "world/SleeperFastenings.h"

editor_mode::selftest_result editor_mode::selftest_sleepers(std::string const &command, std::istringstream &words)
{
	if (command == "beams")
	{
		// the beams the track lays, between two distances along it: their middles, lengths and the way they lie square to
		std::string name;
		double from{-1e9}, to{1e9};
		words >> name >> from >> to;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr || false == track->m_sleepers3d.active)
		{
			WriteLog("SELFTEST beams: no track with beams " + name);
			return selftest_result::handled;
		}
		std::string text;
		for (auto const &beam : track_sleepers::place(*track))
		{
			auto const middle{0.5 * (beam.from + beam.to)};
			if (middle.z < from || middle.z > to)
				continue;
			text += format(" [%.2f %.2f L%.2f a%.2f%s]", middle.x, middle.z, glm::length(beam.to - beam.from), glm::degrees(std::atan2(beam.along.x, beam.along.z)), beam.plain ? " p" : "");
		}
		WriteLog("SELFTEST beams " + name + ":" + text);
		return selftest_result::handled;
	}
	if (command == "sharedzones")
	{
		// how far the timbers shared with other tracks reach along the paths of the track and of the tracks joined to its ends
		std::string name;
		words >> name;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr)
		{
			WriteLog("SELFTEST sharedzones: no track " + name);
			return selftest_result::handled;
		}
		auto const zones_text = [](TTrack const &Track, int const Path) {
			std::string text;
			for (auto const &zone : track_sleepers::shared_zones(Track, Path))
				text += format(" %.2f..%.2f", zone.first, zone.second);
			return text.empty() ? std::string{" none"} : text;
		};
		auto const paths{track->eType == tt_Switch ? 2 : 1};
		for (int path = 0; path < paths; ++path)
		{
			auto const length{track->eType == tt_Switch ? track->SwitchExtension->Segments[path]->GetLength() : track->Segment->GetLength()};
			WriteLog(format("SELFTEST sharedzones %s path %d (%.2f m):%s", name.c_str(), path, length, zones_text(*track, path).c_str()));
			// the track going on from the end of the path, and how far past the end the shared timbers reach along it
			auto *next{track->eType == tt_Switch ? track->SwitchExtension->pNexts[path] : track->trNext};
			if (next == nullptr || next->eType != tt_Normal || next->Segment == nullptr)
				continue;
			glm::dvec3 end, start, unused;
			glm::vec3 angles;
			(track->eType == tt_Switch ? track->SwitchExtension->Segments[path] : track->Segment)->RaPositionGet(length, end, angles);
			next->Segment->RaPositionGet(0.0, start, angles);
			auto const fromstart{glm::length(start - end) < 0.1};
			auto past{0.0};
			for (auto const &zone : track_sleepers::shared_zones(*next, 0))
			{
				if (fromstart && zone.first < 0.05)
					past = zone.second;
				else if (false == fromstart && zone.second > next->Segment->GetLength() - 0.05)
					past = next->Segment->GetLength() - zone.first;
			}
			WriteLog(format("SELFTEST sharedzones %s path %d goes on into %s:%s; shared timbers reach %.2f m past the end", name.c_str(), path, next->name().c_str(), zones_text(*next, 0).c_str(), past));
		}
		return selftest_result::handled;
	}
	if (command == "sleepersbg")
	{
		// the sleepers worked out in the background as in the simulation, to see they come out the same, or back to as they're needed
		int on{1};
		words >> on;
		track_sleepers::lay_in_background(on != 0);
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr)
				track->m_sleepers3d.dirty = true;
		track_sleepers::unsettle_around(*simulation::Paths.sequence().front());
		WriteLog(format("SELFTEST sleepersbg %d", on));
		return selftest_result::handled;
	}
	if (command == "beamsnear")
	{
		double x{0.0}, z{0.0}, radius{3.0};
		words >> x >> z >> radius;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || false == track->m_sleepers3d.active)
				continue;
			for (auto const &beam : track_sleepers::place(*track))
			{
				auto const middle{0.5 * (beam.from + beam.to)};
				if (glm::length(glm::dvec2{middle.x - x, middle.z - z}) > radius)
					continue;
				WriteLog(format("SELFTEST beam %s: %.2f %.2f -> %.2f %.2f, %.2f m, along %.1f deg%s", track->name().c_str(), beam.from.x, beam.from.z, beam.to.x, beam.to.z, glm::length(beam.to - beam.from), glm::degrees(std::atan2(beam.along.x, beam.along.z)), beam.plain ? " plain" : ""));
			}
		}
		return selftest_result::handled;
	}
	if (command == "fastenings")
	{
		// the fastenings on the timbers of the track: along the track, the part and where across it, which way its +x goes
		std::string name, detail;
		words >> name >> detail;
		auto *track{simulation::Paths.find(name)};
		auto const beams{track != nullptr ? track_sleepers::place(*track) : std::vector<track_sleepers::beam>{}};
		auto const *model{track != nullptr ? track_sleepers::timber_model(*track, beams) : nullptr};
		if (model == nullptr || false == model->fastened())
		{
			WriteLog("SELFTEST fastenings: no track " + name + " with the fastenings in the model of its timbers");
			return selftest_result::handled;
		}
		track_sleepers::fastened_rails const rails{*track};
		std::array<int, track_sleepers::part_count> counts{};
		for (auto const &beam : beams)
		{
			if (beam.plain)
				continue;
			auto const middle{0.5 * (beam.from + beam.to)};
			std::string text{format("SELFTEST fastenings %s at %.2f %.2f:", track->name().c_str(), middle.x, middle.z)};
			if (detail == "rails")
			{
				auto const span{beam.to - beam.from};
				auto const length{glm::length(glm::dvec2{span.x, span.z})};
				text += " rails";
				for (auto const &rail : rails.lying(beam))
				{
					auto const at{beam.from + span * (rail.at / length)};
					text += format(" %.3f %.3f k%d r%.3f;", at.x, at.z, static_cast<int>(rail.kind), rail.rise);
				}
				text += " parts";
			}
			for (auto const &fastening : rails.on(beam, *model))
			{
				text += format(" %s%s %.3f %.3f %+.0f", fastening.pad ? "pad:" : "", track_sleepers::part_names[fastening.part], fastening.point.x, fastening.point.z, glm::degrees(std::atan2(fastening.across.y, fastening.across.x)));
				if (fastening.lift != 0.f)
					text += format(" ^%.3f", fastening.lift);
				if (detail == "rails" && (fastening.scale != 1.f || fastening.shift != 0.f))
					text += format(" s%.3f/%.3f/%.3f", fastening.scale, fastening.pivot, fastening.shift);
				if (false == fastening.pad)
					++counts[fastening.part];
			}
			WriteLog(text);
		}
		WriteLog(format("SELFTEST fastenings %s: %d plates, %d chairs, %d checks", track->name().c_str(), counts[track_sleepers::part_plate], counts[track_sleepers::part_chair], counts[track_sleepers::part_check]));
		return selftest_result::handled;
	}
	if (command == "fastencheck")
	{
		// the fastenings of all the tracks laying timbers with them: of each kind, and those over one another on a timber
		std::string mode;
		words >> mode;
		auto const verbose{mode == "all"};
		auto const start{std::chrono::steady_clock::now()};
		auto const rows{track_sleepers::check_fastenings()};
		std::size_t beams{0}, overlaps{0}, tracks{0}, bare{0};
		std::array<std::size_t, track_sleepers::part_count> parts{};
		for (auto const &row : rows)
		{
			beams += row.beams;
			overlaps += row.overlaps;
			bare += row.bares.size();
			tracks += (row.overlaps > 0 ? 1 : 0);
			for (int part = 0; part < track_sleepers::part_count; ++part)
				parts[part] += row.parts[part];
			WriteLog(format("SELFTEST fastencheck %s: %zu timbers, %zu plates, %zu chairs, %zu checks, %zu bare rails, %zu overlaps%s",
			    row.track.c_str(), row.beams, row.parts[track_sleepers::part_plate], row.parts[track_sleepers::part_chair], row.parts[track_sleepers::part_check], row.bares.size(), row.overlaps,
			    row.overlaps > 0 ? format(", worst %.3f m %s at %.2f %.2f", row.worst, row.worstparts.c_str(), row.worstat.x, row.worstat.z).c_str() : ""));
			if (verbose)
				for (auto const &at : row.bares)
					WriteLog(format("SELFTEST fastencheck bare %s at %.3f %.3f", row.track.c_str(), at.x, at.y));
			if (verbose)
				for (auto const &overlap : row.cases)
					WriteLog(format("SELFTEST fastencheck overlap %s %d %d apart %.3f over %.3f at %.2f %.2f", row.track.c_str(), overlap.parts[0], overlap.parts[1], overlap.apart, overlap.over, overlap.at.x, overlap.at.z));
		}
		WriteLog(format("SELFTEST fastencheck: %zu tracks, %zu timbers, %zu plates, %zu chairs, %zu checks, %zu bare rails, %zu overlaps on %zu tracks; in %.1f ms",
		    rows.size(), beams, parts[track_sleepers::part_plate], parts[track_sleepers::part_chair], parts[track_sleepers::part_check], bare, overlaps, tracks,
		    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()));
		return selftest_result::handled;
	}
	if (command == "checkseats")
	{
		// the check rails with timbers under them and no fastening seating them, along how much of them
		auto const gaps{track_sleepers::check_rail_seats()};
		double bare{0.0};
		std::size_t unsupported{0};
		for (auto const &gap : gaps)
		{
			bare += gap.bare;
			unsupported += gap.unsupported;
			WriteLog(format("SELFTEST checkseats %s path %d side %+d %.2f-%.2f: %.0f of %zu timbers bare, %zu with nothing under, first at %.2f %.2f", gap.track.c_str(), gap.path, gap.side, gap.from, gap.to, gap.bare, gap.timbers, gap.unsupported, gap.at.x, gap.at.y));
		}
		WriteLog(format("SELFTEST checkseats: %zu check rails with %.0f timbers bare, %zu with nothing under", gaps.size(), bare, unsupported));
		return selftest_result::handled;
	}
	if (command == "sleepercheck")
	{
		auto const start{std::chrono::steady_clock::now()};
		auto const result{track_sleepers::check()};
		WriteLog(format("SELFTEST sleepercheck: %zu beams, %zu overlaps, %zu gaps, %zu bare rails, %zu too long (longest %.2f m), widest spacing %.2f m at %s; %zu shared off one plane, %zu switch panels off one plane, %zu ends clipped, %zu beams dropped, slowest cluster %.1f ms of %zu tracks; digest %016llx; in %.1f ms",
		    result.beams, result.overlaps, result.gaps, result.bare, result.toolong, result.longest, result.widest, result.widestat.c_str(), result.offplane, result.panels, result.clipped, result.dropped, result.slowest, result.biggest, static_cast<unsigned long long>(result.digest),
		    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()));
		for (auto const &example : result.examples)
			WriteLog("SELFTEST sleepercheck " + example);
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}
