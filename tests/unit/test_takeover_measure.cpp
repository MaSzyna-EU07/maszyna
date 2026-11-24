// The recognition of the old tracks measured on the sceneries of the game (editor-tracks-tests.md §4 "E6"): not a test of the build, skipped
// unless asked for, read only. Run as
//   EU07_TAKEOVER_MEASURE="<the scenery folder>;<a file of tracks>[;<another>...]" eu07_unit_tests --no-skip --test-case="takeover measure*"
// it reads the tracks of each file with its includes, chains the plain ones and recognizes each chain as a line, and prints how far the
// lines keep off the old tracks, how many would be taken over with no question and what the domain lays otherwise.

#include "stdafx.h"
#include "doctest.h"
#include "editor/layout/layoutLegacy.hpp"
#include "editor/project/legacyScenery.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <map>
#include <string>
#include <vector>

namespace {

std::vector<std::string> parts_of(std::string const &Text) {
	std::vector<std::string> result;
	std::size_t at{0};
	while (at <= Text.size()) {
		auto end{Text.find(';', at)};
		end = end == std::string::npos ? Text.size() : end;
		if (end > at) {
			result.push_back(Text.substr(at, end - at));
		}
		at = end + 1;
	}
	return result;
}

// how many of the values are up to each bound, the last one past all of them
struct histogram {
	std::vector<double> bounds;
	std::vector<std::size_t> counts;
	explicit histogram(std::vector<double> Bounds) : bounds{std::move(Bounds)}, counts(bounds.size() + 1, 0) {}
	void add(double const Value) {
		std::size_t i{0};
		while (i < bounds.size() && Value > bounds[i]) {
			++i;
		}
		++counts[i];
	}
	std::string text(char const *Unit, double const Scale) const {
		std::string result;
		for (std::size_t i = 0; i < counts.size(); ++i) {
			result += i < bounds.size() ? std::format(" <={}{} {}", bounds[i] * Scale, Unit, counts[i]) : std::format(" >{}{} {}", bounds.back() * Scale, Unit, counts[i]);
		}
		return result;
	}
};

std::string kind_of(std::string const &Finding) {
	if (Finding.find("at a joint") != std::string::npos) {
		return "kink at a joint";
	}
	if (Finding.find("compound") != std::string::npos) {
		return "compound curve";
	}
	if (Finding.find("no transition curve") != std::string::npos) {
		return "cant with no transition";
	}
	if (Finding.find("grade line") != std::string::npos) {
		return "grade line not laid";
	}
	return Finding;
}

} // namespace

TEST_CASE("takeover measure: the old tracks of sceneries of the game, read only" * doctest::skip()) {
	auto const *setting{std::getenv("EU07_TAKEOVER_MEASURE")};
	if (setting == nullptr) {
		MESSAGE("EU07_TAKEOVER_MEASURE not set");
		return;
	}
	auto const parts{parts_of(setting)};
	REQUIRE(parts.size() >= 2);
	std::filesystem::path const folder{parts.front()};
	layout::takeover_tolerance const tolerance;
	for (std::size_t f = 1; f < parts.size(); ++f) {
		auto const started{std::chrono::steady_clock::now()};
		auto const scenery{project::read_legacy(folder, parts[f])};
		std::vector<std::size_t> take;
		std::size_t switches{0};
		for (std::size_t i = 0; i < scenery.tracks.size(); ++i) {
			take.push_back(i);
			switches += scenery.tracks[i].is_switch ? 1 : 0;
		}
		auto const plan{layout::plan_takeover(scenery.tracks, take)};
		auto const seconds{std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()};
		histogram plan_off{{0.005, 0.01, 0.02, 0.05, 0.1, 0.5}};
		histogram height_off{{0.005, 0.01, 0.02, 0.05, 0.1, 0.5}};
		histogram cant_off{{2.0, 5.0, 10.0, 20.0, 50.0}};
		std::size_t silent{0};
		std::size_t tracks{0};
		double length{0.0};
		double silent_length{0.0};
		std::array<std::size_t, 4> beyond{}; // plan, height, cant, more than one
		std::map<std::string, std::size_t> findings;
		for (auto const &line : plan.lines) {
			auto const &m{line.fit.measured};
			plan_off.add(m.plan);
			height_off.add(m.height);
			cant_off.add(m.cant);
			tracks += line.chain.links.size();
			length += line.fit.solved.length;
			if (line.silent) {
				++silent;
				silent_length += line.fit.solved.length;
			}
			else {
				auto const over{static_cast<int>(m.plan > tolerance.plan) + static_cast<int>(m.height > tolerance.height) + static_cast<int>(m.cant > tolerance.cant)};
				if (over > 1) {
					++beyond[3];
				}
				else if (m.plan > tolerance.plan) {
					++beyond[0];
				}
				else if (m.height > tolerance.height) {
					++beyond[1];
				}
				else {
					++beyond[2];
				}
			}
			for (auto const &finding : line.fit.findings) {
				++findings[kind_of(finding)];
			}
		}
		std::size_t failed{0};
		for (auto const &note : plan.notes) {
			failed += note.find("a switch stays") == std::string::npos ? 1 : 0;
		}
		std::printf("%s: %zu tracks (%zu switches), %zu lines of %zu tracks, %.1f km, %zu chains not laid, %.1f s\n", parts[f].c_str(), scenery.tracks.size(), switches, plan.lines.size(), tracks, length / 1000.0,
		            failed, seconds);
		std::printf("  silent (within %.0f mm, %.0f mm, %.0f mm): %zu lines, %.1f km; the question: %zu (the plan %zu, the height %zu, the cant %zu, more than one %zu)\n", tolerance.plan * 1000.0,
		            tolerance.height * 1000.0, tolerance.cant, silent, silent_length / 1000.0, plan.lines.size() - silent, beyond[0], beyond[1], beyond[2], beyond[3]);
		std::printf("  plan:  %s\n  height:%s\n  cant:  %s\n", plan_off.text("mm", 1000.0).c_str(), height_off.text("mm", 1000.0).c_str(), cant_off.text("mm", 1.0).c_str());
		for (auto const &[kind, count] : findings) {
			std::printf("  %zu x %s\n", count, kind.c_str());
		}
		// the lines keeping farthest off their old tracks
		std::vector<layout::taken_line const *> worst;
		for (auto const &line : plan.lines) {
			worst.push_back(&line);
		}
		std::ranges::sort(worst, [&](auto const *A, auto const *B) { return A->fit.measured.share(tolerance) > B->fit.measured.share(tolerance); });
		for (std::size_t i = 0; i < worst.size() && i < 8; ++i) {
			auto const &fit{worst[i]->fit};
			auto const &first{scenery.tracks[worst[i]->chain.links.front().track]};
			std::printf("  worst: %s... %zu tracks, %.0f m, %zu curves: plan %.3f m (rms %.3f) at %.1f %.1f, height %.3f m, cant %.1f mm\n", first.name.c_str(), worst[i]->chain.links.size(), fit.solved.length,
			            fit.shape.vertices.size(), fit.measured.plan, fit.measured.plan_rms, fit.measured.plan_at.x, fit.measured.plan_at.z, fit.measured.height, fit.measured.cant);
		}
		// the line of a track named, in detail: EU07_TAKEOVER_TRACK=<name>
		if (auto const *named{std::getenv("EU07_TAKEOVER_TRACK")}) {
			for (auto const &line : plan.lines) {
				if (std::ranges::none_of(line.chain.links, [&](layout::legacy_link const &Link) { return scenery.tracks[Link.track].name == named; })) {
					continue;
				}
				auto const samples{layout::sample_chain(layout::paths_of(scenery.tracks, line.chain), 1.0)};
				for (auto const &curve : layout::find_curves(samples)) {
					std::printf("    curve %.1f..%.1f turning %.5f, R %.1f, in %.1f, out %.1f%s%s\n", curve.from, curve.to, curve.deflection, curve.radius, curve.transition_in, curve.transition_out,
					            curve.kink ? ", a kink" : "", curve.compound ? ", compound" : "");
				}
				for (auto const &vertex : line.fit.shape.vertices) {
					std::printf("    vertex %.2f %.2f R %.1f in %.1f out %.1f D %.0f\n", vertex.point.x, vertex.point.y, vertex.radius, vertex.transition_in, vertex.transition_out, vertex.cant);
				}
				for (auto const &link : line.chain.links) {
					auto const &path{scenery.tracks[link.track].paths[static_cast<std::size_t>(link.path)]};
					auto const &a{path.points[segment_data::point::start]};
					auto const &b{path.points[segment_data::point::end]};
					std::printf("    %s %s: %.2f %.2f -> %.2f %.2f\n", scenery.tracks[link.track].name.c_str(), link.forward ? "on" : "back", a.x, a.z, b.x, b.z);
				}
			}
		}
		for (std::size_t i = 0, shown = 0; i < plan.notes.size() && shown < 5; ++i) {
			if (plan.notes[i].find("a switch stays") == std::string::npos) {
				std::printf("  not laid: %s\n", plan.notes[i].substr(0, 200).c_str());
				++shown;
			}
		}
		std::fflush(stdout);
	}
}
