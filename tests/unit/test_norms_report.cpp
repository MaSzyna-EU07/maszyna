// the report of the norms on the tracks laid (editor/editorNormsReport.cpp): tracks read from the text of a scenery with no scene, the
// way test_track_text.cpp reads them, chained into a line, sampled, described as elements and checked. The line of data/norms/line.scn
// (written by make_line.py there) against each profile gives the report kept in data/norms/line_report.txt; with
// EU07_UNIT_TESTS_UPDATE_GOLDEN=1 the report is written there instead, to be looked through as the change it is

#include "stdafx.h"
#include "doctest.h"
#include "track_text.h"
#include "editor/editorAlignment.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorNormsReport.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

using test::read_track;
using test::track_reading;

std::string const kData{EU07_UNIT_TESTS_DATA "/norms/"};

std::vector<std::unique_ptr<TTrack>> read_tracks(std::string const &File) {
	std::ifstream in{File};
	REQUIRE(in.good());
	std::vector<std::unique_ptr<TTrack>> result;
	std::string line;
	while (std::getline(in, line)) {
		// a checkout may end the lines with CR LF
		std::erase(line, '\r');
		if (line.rfind("node", 0) == 0) {
			result.push_back(read_track(line));
		}
	}
	return result;
}

// the tracks one after another, from the first one: each next the one an end of which is where the line has come to
std::vector<norms_report::span> chained(std::vector<std::unique_ptr<TTrack>> const &Tracks) {
	std::vector<norms_report::span> result;
	if (Tracks.empty()) {
		return result;
	}
	std::vector<bool> taken(Tracks.size(), false);
	result.push_back({Tracks.front().get(), 0, true});
	taken.front() = true;
	auto reached{Tracks.front()->m_paths[0].points[segment_data::point::end]};
	for (bool found{true}; found;) {
		found = false;
		for (std::size_t i = 0; i < Tracks.size() && false == found; ++i) {
			if (taken[i]) {
				continue;
			}
			auto const &path{Tracks[i]->m_paths[0]};
			auto const at_start{geometry::plan_distance(path.points[segment_data::point::start], reached) < geometry::same_point};
			auto const at_end{geometry::plan_distance(path.points[segment_data::point::end], reached) < geometry::same_point};
			if (false == at_start && false == at_end) {
				continue;
			}
			result.push_back({Tracks[i].get(), 0, at_start});
			reached = path.points[at_start ? segment_data::point::end : segment_data::point::start];
			taken[i] = true;
			found = true;
		}
	}
	return result;
}

std::string normalized(std::string Text) {
	std::erase(Text, '\r');
	return Text;
}

std::string file_text(std::string const &File) {
	std::ifstream in{File, std::ios::binary};
	std::ostringstream text;
	text << in.rdbuf();
	return normalized(text.str());
}

// the track text of a path laid by the editor, for it to be read as the scenery loader reads it
std::string entry(std::string const &Name, segment_data const &Path) {
	std::ostringstream text;
	text.precision(10);
	auto const length{geometry::bezier{Path}.plan_length()};
	text << "node -1 0 " << Name << " track normal " << length << " 1.435 0.15 25 20 0 flat vis rail_screw_used1 4 tpbps-new2 0.2 0.5 1.1";
	auto const &points{Path.points};
	for (auto const index : {segment_data::point::start, segment_data::point::control1, segment_data::point::control2, segment_data::point::end}) {
		text << ' ' << points[index].x << ' ' << points[index].y << ' ' << points[index].z;
		if (index == segment_data::point::start) {
			text << ' ' << Path.rolls[0];
		} else if (index == segment_data::point::end) {
			text << ' ' << Path.rolls[1];
		}
	}
	text << " 0 endtrack";
	return text.str();
}

} // namespace

TEST_CASE("norms report: the cant of a curve laid by the editor leans into it, both ways and run both ways, as the samples sign it") {
	track_reading const reading;
	for (auto const side : {1.0, -1.0}) {
		CAPTURE(side);
		// a corner of R 400 m with transitions of 60 m and a cant of 80 mm, as in test_cant.cpp
		alignment::design design;
		design.start = {0.0, 0.0, 0.0};
		design.end = {1000.0 * side, 0.0, 1000.0};
		design.start_direction = {0.0, 1.0};
		design.end_direction = {side, 0.0};
		alignment::vertex vertex;
		vertex.position = {0.0, 1000.0};
		vertex.radius = 400.0;
		vertex.transition_in = 60.0;
		vertex.transition_out = 60.0;
		vertex.cant = 80.0;
		design.vertices.push_back(vertex);
		auto const line{alignment::compute(design)};
		REQUIRE(line.valid);
		std::vector<std::unique_ptr<TTrack>> tracks;
		int count{0};
		for (auto const &path : alignment::pieces(line, design, alignment::minimum_pieces(line, design))) {
			tracks.push_back(read_track(entry("corner" + std::to_string(++count), path)));
		}
		auto spans{chained(tracks)};
		REQUIRE(spans.size() == tracks.size());
		for (auto const backward : {false, true}) {
			CAPTURE(backward);
			if (backward) {
				std::ranges::reverse(spans);
				for (auto &span : spans) {
					span.forward = !span.forward;
				}
			}
			auto const sampled{norms_report::sample(spans, 1.0, false)};
			int curved{0};
			for (auto const &sample : sampled.samples) {
				if (std::abs(sample.curvature) > 1.0 / 450.0) {
					++curved;
					CHECK(sample.curvature * sample.cant > 0.0);
					CHECK(std::abs(sample.cant) > 70.0);
					CHECK(std::abs(sample.cant) < 80.5);
				}
			}
			CHECK(curved > 50);
			// so at 90 km/h the arc falls short by 11.8 V^2 / R - 80 mm, not by more: 159 mm, past the 110 mm of a finding (at 80 km/h
			// the 108.8 mm would make none, and the check would never run)
			norms_report::options options;
			options.line.vmax = 90.0;
			options.track_speeds = false;
			auto const checked{norms_report::check(spans, options)};
			auto const deficient{std::ranges::find_if(checked.entries, [](auto const &Found) { return Found.finding.what == norms::parameter::cant_deficiency && Found.kind == norms::element_kind::arc; })};
			REQUIRE(deficient != checked.entries.end());
			CHECK(deficient->finding.value == doctest::Approx(11.8 * 90 * 90 / 400.0 - 80.0).epsilon(0.02));
		}
	}
}

TEST_CASE("norms report: the golden report of the line of data/norms/line.scn against each profile") {
	track_reading const reading;
	auto const tracks{read_tracks(kData + "line.scn")};
	REQUIRE(tracks.size() == 14);
	auto const spans{chained(tracks)};
	REQUIRE(spans.size() == tracks.size());
	std::string report;
	for (auto const profile : {norms::profile_id::plk, norms::profile_id::tsi, norms::profile_id::en13803, norms::profile_id::ch}) {
		norms_report::options options;
		options.profile = profile;
		options.line.vmax = 100.0;
		options.line.kind = norms::traffic::mixed;
		options.line.track = norms::track_kind::main;
		options.line.load = 5.0;
		auto const checked{norms_report::check(spans, options)};
		report += norms_report::header(options) + norms_report::text(checked, profile == norms::profile_id::plk) + "\n";
		// every finding lies on a track of the line, at a point of it
		for (auto const &found : checked.entries) {
			CHECK(found.track != nullptr);
		}
	}
	auto const golden{kData + "line_report.txt"};
	if (auto const *update{std::getenv("EU07_UNIT_TESTS_UPDATE_GOLDEN")}; update != nullptr && std::string{update} == "1") {
		std::ofstream{golden, std::ios::binary} << report;
		MESSAGE("written " << golden);
		return;
	}
	auto const expected{file_text(golden)};
	// line by line, for the first difference to be shown by itself
	std::istringstream got{report};
	std::istringstream kept{expected};
	std::string got_line;
	std::string kept_line;
	int number{0};
	while (true) {
		auto const more_got{static_cast<bool>(std::getline(got, got_line))};
		auto const more_kept{static_cast<bool>(std::getline(kept, kept_line))};
		++number;
		if (false == more_got && false == more_kept) {
			break;
		}
		CAPTURE(number);
		REQUIRE(got_line == kept_line);
	}
}
