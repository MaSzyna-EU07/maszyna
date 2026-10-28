#include "stdafx.h"
#include "fiz.h"

#include <algorithm>
#include <array>
#include <format>

#include "utilities/parser.h"

namespace test::fiz {

namespace {

// the lists of LoadFIZ_Parse(), as bits of the set of the ones reading a row
enum list : unsigned {
	brake_table = 1 << 0, // startBPT
	motor_table = 1 << 1, // startMPT
	motor_table0 = 1 << 2, // startMPT0
	resistors = 1 << 3, // startRLIST
	universal = 1 << 4, // startUCLIST
	diesel = 1 << 5, // startDLIST
	diesel_momentum = 1 << 6, // startDIZELMOMENTUMLIST
	diesel_v2n = 1 << 7, // startDIZELV2NMAXLIST
	hydro = 1 << 8, // startHYDROTCLIST
	diesel_pmax = 1 << 9, // startPMAXLIST
	field = 1 << 10, // startFFLIST
	field_brake = 1 << 11, // startFFEDLIST
	generator = 1 << 12, // startWWLIST
	wipers = 1 << 13, // startWiperList
	dimmer = 1 << 14, // startDimmerList
	lights = 1 << 15, // startLIGHTSLIST
	compressors = 1 << 16, // startCOMPRESSORLIST
};

// the issection() calls of LoadFIZ_Parse()
std::array const sections{
    section{"Param", '.'},
    section{"Load", ':'},
    section{"Dimensions", ':'},
    section{"Wheels", ':'},
    section{"Brake", ':'},
    section{"Doors", ':'},
    section{"BuffCoupl", '.'},
    section{"BuffCoupl1", '.'},
    section{"BuffCoupl2", '.'},
    section{"TurboPos", ':'},
    section{"Cntrl", '.', brake_table, ""},
    section{"Headlights", ':'},
    section{"Blending", ':'},
    section{"DCEMUED", ':'},
    section{"SpringBrake", ':'},
    section{"Light", ':'},
    section{"Security", ':'},
    section{"Clima", ':'},
    section{"Power", ':'},
    section{"SpeedControl", ':'},
    section{"Engine", ':'},
    section{"Switches", ':'},
    section{"MotorParamTable", ':', motor_table, "END-MPT"},
    section{"MotorParamTable0", ':', motor_table0, "END-MPT"},
    section{"Circuit", ':'},
    section{"AI", ':'},
    section{"RList", ':', resistors, "END-RL"},
    section{"UCList", ':', universal, "END-UCL"},
    section{"DList", ':', diesel, "END-DL"},
    section{"DMList", ':', diesel_momentum, "END-DML"},
    section{"HTCList", ':', hydro, "END-HTCL"},
    section{"PmaxList", ':', diesel_pmax, "END-PML"},
    section{"V2NList", ':', diesel_v2n, "END-V2NL"},
    section{"ffList", ':', field, "endff"},
    section{"ffBrakeList", ':', field_brake, "endff"},
    section{"WWList", ':', generator, "END-WWL"},
    section{"WiperList", ':', wipers, "endwl"},
    section{"DimmerList", ':', dimmer, "endDimmerList"},
    section{"LightsList", ':', lights, "endL"},
    section{"CompressorList", ':', compressors, "endCL"},
};

// the lines closing lists, checked by the loader before the sections in this order, each closing the brake table too
struct closing {
	std::string_view line;
	unsigned lists;
};
std::array const closings{
    closing{"END-MPT", motor_table | motor_table0},
    closing{"END-RL", resistors},
    closing{"END-UCL", universal},
    closing{"END-DL", diesel},
    closing{"END-DML", diesel_momentum},
    closing{"END-V2NL", diesel_v2n},
    closing{"END-HTCL", hydro},
    closing{"END-PML", diesel_pmax},
    closing{"endff", field | field_brake},
    closing{"endwl", wipers},
    closing{"endDimmerList", dimmer},
    closing{"END-WWL", generator},
    closing{"endL", lights},
    closing{"endCL", compressors},
};

} // namespace

std::string value(std::string const &Key, std::string const &Line) {
	std::string const input{" " + Line};
	auto const lookup{input.find(" " + Key + "=")};
	if (lookup == std::string::npos) {
		return {};
	}
	auto const start{input.find_first_not_of(' ', lookup + Key.size() + 2)};
	if (start == std::string::npos) {
		return {};
	}
	auto const end{input.find(' ', start)};
	return input.substr(start, end == std::string::npos ? std::string::npos : end - start);
}

std::vector<std::string> words(std::string const &Line) {
	std::vector<std::string> result;
	std::size_t start{0};
	while ((start = Line.find_first_not_of(" \t;", start)) != std::string::npos) {
		auto const end{Line.find_first_of(" \t;", start)};
		result.push_back(Line.substr(start, end == std::string::npos ? std::string::npos : end - start));
		start = end;
	}
	return result;
}

section const *named(std::string_view Name) {
	auto const lookup{std::ranges::find(sections, Name, &section::name)};
	return lookup == sections.end() ? nullptr : &*lookup;
}

section const *opened_by(std::string_view Line) {
	auto const lookup{std::ranges::find_if(sections, [&](section const &Section) { return Line.starts_with(Section.name) && Line.size() > Section.name.size() && Line[Section.name.size()] == Section.separator; })};
	return lookup == sections.end() ? nullptr : &*lookup;
}

std::vector<std::string> lines(cParser &Parser) {
	std::vector<std::string> result;
	while (Parser.ok()) {
		auto line{Parser.getToken<std::string>(false, "\n\r")};
		if (line.starts_with("//") || line.find('#') != std::string::npos) {
			continue;
		}
		line.erase(0, line.find_first_not_of(' '));
		if (false == line.empty() && line.back() == '\r') {
			line.pop_back();
		}
		result.push_back(line);
	}
	return result;
}

std::vector<std::string> keys(std::string const &Line) {
	std::vector<std::string> result;
	std::string const input{" " + Line};
	for (auto equals{input.find('=')}; equals != std::string::npos; equals = input.find('=', equals + 1)) {
		auto const start{input.rfind(' ', equals) + 1};
		auto key{input.substr(start, equals - start)};
		// a=b=c gives a, not a=b
		if (false == key.empty() && key.find('=') == std::string::npos && std::ranges::find(result, key) == result.end()) {
			result.push_back(std::move(key));
		}
	}
	return result;
}

std::vector<std::string> reading(std::vector<std::string> const &Lines) {
	std::vector<std::string> result;
	unsigned active{0};
	for (auto const &line : Lines) {
		if (line.empty()) {
			active &= ~brake_table;
			continue;
		}
		if (auto const close{std::ranges::find_if(closings, [&](closing const &Closing) { return line.starts_with(Closing.line); })}; close != closings.end()) {
			active &= ~(close->lists | brake_table);
			continue;
		}
		if (auto const opened{opened_by(line)}; opened != nullptr) {
			active = (active & ~brake_table) | opened->list;
			// the values the section gives, as the loader asks for them by their keys
			auto keys{fiz::keys(line)};
			std::ranges::sort(keys);
			std::string values{opened->name};
			for (auto const &key : keys) {
				if (auto const value{fiz::value(key, line)}; false == value.empty()) {
					values += " " + key + "=" + value;
				}
			}
			result.push_back(std::move(values));
			continue;
		}
		// the first open list reads the row, in the order of the loader's checks, which carries on past the dimmer list
		unsigned readers{0};
		for (unsigned list = brake_table; list <= compressors; list <<= 1) {
			if ((active & list) != 0) {
				readers |= list;
				if (list != dimmer) {
					break;
				}
			}
		}
		if (readers != 0) {
			std::string row{std::format("row of {:x}:", readers)};
			for (auto const &word : words(line)) {
				row += " " + word;
			}
			result.push_back(std::move(row));
		}
	}
	return result;
}

} // namespace test::fiz
