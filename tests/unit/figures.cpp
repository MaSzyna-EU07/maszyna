#include "stdafx.h"
#include "figures.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>


namespace test {

namespace {

// the units of the figures, with the decimals they're written with
struct unit {
	char const *name;
	int decimals;
};
unit const seconds{"s", 2};
unit const kmh{"km/h", 1};
unit const metres{"m", 1};
unit const bar{"bar", 3};
unit const amperes{"A", 0};
unit const kilonewtons{"kN", 1};
unit const units[]{seconds, kmh, metres, bar, amperes, kilonewtons};

std::string line(char const *Name, double Value, unit const &Unit) {
	char text[128];
	std::snprintf(text, sizeof(text), "%s = %.*f %s", Name, Unit.decimals, Value, Unit.name);
	return text;
}

// the name, the value and the unit of a line of a figure, or false for the header of a part
bool parse(std::string const &Line, std::string &Name, double &Value, std::string &Unit) {
	auto const equals{Line.find(" = ")};
	if (equals == std::string::npos) {
		return false;
	}
	Name = Line.substr(0, equals);
	auto const value{Line.substr(equals + 3)};
	auto const space{value.find(' ')};
	Value = std::strtod(value.substr(0, space).c_str(), nullptr);
	Unit = space == std::string::npos ? std::string{} : value.substr(space + 1);
	return true;
}

// a difference of 0.5%, or of the last decimal written
bool close(double Recorded, double Now, std::string const &Unit) {
	auto step{0.0};
	for (auto const &unit : units) {
		if (Unit == unit.name) {
			step = std::pow(10.0, -unit.decimals);
		}
	}
	return std::abs(Recorded - Now) <= 0.005 * std::max(std::abs(Recorded), std::abs(Now)) + step + 1e-9;
}

// the drop of the brake pipe which the time to get back counts from, and the pressure it counts to
constexpr double pipe_drop{4.8};
constexpr double pipe_back{4.9};

} // namespace

void figures::sample(double Time, TMoverParameters const &Vehicle) {
	auto &part{m_parts.back()};
	auto const speed{Vehicle.V * 3.6};
	auto const pipe{Vehicle.PipePress};
	if (false == part.sampled) {
		part.sampled = true;
		// from the last step of the part before, so the parts add up to the run
		part.start = m_last_time < 0.0 ? Time : m_last_time;
		part.speed_start = m_last_time < 0.0 ? speed : m_last_speed;
		part.distance_start = m_last_time < 0.0 ? Vehicle.DistCounter : m_last_distance;
		part.pipe_lowest = pipe;
	}
	part.end = Time;
	part.speed_end = speed;
	part.speed_peak = std::max(part.speed_peak, std::abs(speed));
	part.distance_end = Vehicle.DistCounter;
	if (part.stopped < 0.0 && std::abs(part.speed_start) > 1.0 && std::abs(speed) < 0.05) {
		part.stopped = Time - part.start;
	}
	if (pipe < part.pipe_lowest) {
		part.pipe_lowest = pipe;
		part.pipe_back = -1.0;
	}
	part.pipe_end = pipe;
	if (part.pipe_back < 0.0 && part.pipe_lowest < pipe_drop && pipe >= pipe_back) {
		part.pipe_back = Time - part.start;
	}
	part.cylinders_peak = std::max(part.cylinders_peak, Vehicle.BrakePress);
	part.current_peak = std::max(part.current_peak, std::abs(Vehicle.Im));
	part.force_peak = std::max(part.force_peak, std::abs(Vehicle.Ft) / 1000.0);

	m_last_time = Time;
	m_last_speed = speed;
	m_last_distance = Vehicle.DistCounter;
	// which figures the vehicle has, over the whole run, so each part has the same ones
	m_moved |= std::abs(speed) >= 0.05;
	m_piped |= pipe >= 0.1;
	m_braked |= Vehicle.BrakePress >= 0.005;
	m_current |= std::abs(Vehicle.Im) >= 0.5;
	m_force |= std::abs(Vehicle.Ft) >= 50.0;
}

void figures::mark(std::string const &Event) {
	auto &part{m_parts.back()};
	if (false == part.sampled) {
		// nothing ran since the last event: a part of both
		part.event = part.event == "start" ? Event : part.event + ", " + Event;
		return;
	}
	m_parts.push_back(figures::part{Event});
}

std::vector<std::string> figures::lines() const {
	std::vector<std::string> result;
	for (auto const &part : m_parts) {
		if (false == part.sampled) {
			continue;
		}
		result.push_back("[" + part.event + "]");
		result.push_back(line("duration", part.end - part.start, seconds));
		if (m_moved) {
			result.push_back(line("speed at the start", part.speed_start, kmh));
			result.push_back(line("speed at the end", part.speed_end, kmh));
			result.push_back(line("speed peak", part.speed_peak, kmh));
			result.push_back(line("distance", (part.distance_end - part.distance_start) * 1000.0, metres));
			if (part.stopped >= 0.0) {
				result.push_back(line("stopped after", part.stopped, seconds));
			}
		}
		if (m_piped) {
			result.push_back(line("brake pipe lowest", part.pipe_lowest, bar));
			result.push_back(line("brake pipe at the end", part.pipe_end, bar));
			if (part.pipe_back >= 0.0) {
				result.push_back(line("brake pipe back at 4.9 bar after", part.pipe_back, seconds));
			}
		}
		if (m_braked) {
			result.push_back(line("brake cylinders peak", part.cylinders_peak, bar));
		}
		if (m_current) {
			result.push_back(line("motor current peak", part.current_peak, amperes));
		}
		if (m_force) {
			result.push_back(line("tractive force peak", part.force_peak, kilonewtons));
		}
	}
	return result;
}

namespace {

struct figure {
	std::string name;
	double value;
	std::string unit;
	std::string text; // the value and the unit as written
};

struct recorded_part {
	std::string header;
	std::vector<figure> figures;
};

std::vector<recorded_part> parts_of(std::vector<std::string> const &Lines) {
	std::vector<recorded_part> parts;
	for (auto const &line : Lines) {
		figure figure;
		if (false == parse(line, figure.name, figure.value, figure.unit)) {
			parts.push_back({line, {}});
			continue;
		}
		if (parts.empty()) {
			parts.push_back({});
		}
		figure.text = line.substr(line.find(" = ") + 3);
		parts.back().figures.push_back(figure);
	}
	return parts;
}

figure const *find(std::vector<figure> const &Figures, std::string const &Name) {
	for (auto const &figure : Figures) {
		if (figure.name == Name) {
			return &figure;
		}
	}
	return nullptr;
}

} // namespace

std::vector<std::string> figures::compare(std::vector<std::string> const &Recorded, std::vector<std::string> const &Now) {
	std::vector<std::string> differences;
	auto const recorded{parts_of(Recorded)};
	auto const now{parts_of(Now)};
	for (std::size_t i = 0; i < std::max(recorded.size(), now.size()); ++i) {
		if (i >= recorded.size() || i >= now.size() || recorded[i].header != now[i].header) {
			// the parts can't be paired up from here on
			differences.push_back("the parts differ from part " + std::to_string(i + 1) + ": " + (i < now.size() ? now[i].header : "none") + " now, " +
			                      (i < recorded.size() ? recorded[i].header : "none") + " recorded");
			break;
		}
		auto const &header{now[i].header};
		for (auto const &figure : recorded[i].figures) {
			auto const *current{find(now[i].figures, figure.name)};
			if (current == nullptr) {
				differences.push_back(header + " " + figure.name + ": none now, " + figure.text + " recorded");
			}
			else if (current->unit != figure.unit || false == close(figure.value, current->value, figure.unit)) {
				differences.push_back(header + " " + figure.name + ": " + current->text + " now, " + figure.text + " recorded");
			}
		}
		for (auto const &figure : now[i].figures) {
			if (find(recorded[i].figures, figure.name) == nullptr) {
				differences.push_back(header + " " + figure.name + ": " + figure.text + " now, none recorded");
			}
		}
	}
	return differences;
}

} // namespace test
