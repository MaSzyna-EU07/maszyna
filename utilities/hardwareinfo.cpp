/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/hardwareinfo.h"

#include <algorithm>
#include <array>
#include <format>
#include <set>
#include <utility>

namespace hardware {

namespace {

std::string lowercase(std::string_view Text)
{
	std::string result{Text};
	for (auto &c : result)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return result;
}

bool contains(std::string_view Text, std::string_view What)
{
	return Text.find(What) != std::string_view::npos;
}

void erase_all(std::string &Text, std::string_view What)
{
	auto at = Text.find(What);
	while (at != std::string::npos)
	{
		Text.erase(at, What.size());
		at = Text.find(What, at);
	}
}

// the words of a GPU name, lowercase: without the (TM) and (R) marks, and without what follows them in parentheses
// (Mesa adds the driver and the chip: "AMD Radeon Graphics (radeonsi, renoir, LLVM 15.0.7)")
std::vector<std::string> name_words(std::string_view Name)
{
	auto name{lowercase(Name)};
	erase_all(name, "(tm)");
	erase_all(name, "(r)");
	if (auto const parenthesis = name.find('('); parenthesis != std::string::npos)
		name.erase(parenthesis);
	std::vector<std::string> words;
	std::string word;
	for (auto const c : name)
	{
		if (std::isalnum(static_cast<unsigned char>(c)))
		{
			word += c;
		}
		else if (!word.empty())
		{
			words.push_back(std::move(word));
			word.clear();
		}
	}
	if (!word.empty())
		words.push_back(std::move(word));
	return words;
}

// whether the words hold First directly followed by Second
bool has_pair(std::vector<std::string> const &Words, std::string_view First, std::string_view Second)
{
	for (std::size_t i = 0; i + 1 < Words.size(); ++i)
		if (Words[i] == First && Words[i + 1] == Second)
			return true;
	return false;
}

bool has_word(std::vector<std::string> const &Words, std::string_view What)
{
	return std::ranges::find(Words, What) != Words.end();
}

bool is_software_name(std::string_view Name)
{
	constexpr std::array<std::string_view, 6> markers{"llvmpipe", "softpipe", "swiftshader", "gdi generic", "software rasterizer", "basic render driver"};
	auto const name{lowercase(Name)};
	return std::ranges::any_of(markers, [&name](std::string_view const Marker) { return contains(name, Marker); });
}

gpu_kind classify_intel(std::vector<std::string> const &Words)
{
	using enum gpu_kind;
	// Iris Xe MAX (DG1) is a laptop card
	for (std::size_t i = 0; i + 2 < Words.size(); ++i)
		if (Words[i] == "iris" && Words[i + 1] == "xe" && Words[i + 2] == "max")
			return discrete;
	// the Arc cards have a letter and a number after "Arc" (A770, B580) or are the Pro ones; the integrated Arc are
	// "Arc Graphics" (Meteor Lake) and "Arc 140V" (Lunar Lake)
	for (std::size_t i = 0; i + 1 < Words.size(); ++i)
	{
		if (Words[i] != "arc")
			continue;
		auto const &model{Words[i + 1]};
		if (model == "pro")
			return discrete;
		if (model.size() >= 2 && std::isalpha(static_cast<unsigned char>(model[0])) && std::isdigit(static_cast<unsigned char>(model[1])))
			return discrete;
	}
	return integrated;
}

gpu_kind classify_amd(std::vector<std::string> const &Words)
{
	using enum gpu_kind;
	if (has_word(Words, "rx") || has_word(Words, "firepro") || has_word(Words, "instinct") || has_pair(Words, "radeon", "pro") || has_pair(Words, "radeon", "vii"))
		return discrete;
	// the APUs are "Radeon Graphics", "Radeon Vega 8 Graphics", "Radeon R7 Graphics", "Radeon 780M Graphics"; the
	// cards are named by their model ("Radeon RX 580 Series", "Radeon R7 370")
	if (!Words.empty() && Words.back() == "graphics")
		return integrated;
	return unknown;
}

std::string_view yes_no(int const Value)
{
	if (Value > 0)
		return "yes";
	if (Value == 0)
		return "no";
	return "unknown";
}

// "Key<spaces>: value" of /proc/cpuinfo and /proc/meminfo
std::pair<std::string_view, std::string_view> split_entry(std::string_view Line)
{
	auto const colon = Line.find(':');
	if (colon == std::string_view::npos)
		return {};
	auto key{Line.substr(0, colon)};
	auto value{Line.substr(colon + 1)};
	while (!key.empty() && std::isspace(static_cast<unsigned char>(key.back())))
		key.remove_suffix(1);
	while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())))
		value.remove_prefix(1);
	while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
		value.remove_suffix(1);
	return {key, value};
}

template <typename Function> void for_each_line(std::string_view Text, Function &&Each)
{
	while (!Text.empty())
	{
		auto const end = Text.find('\n');
		Each(Text.substr(0, end));
		if (end == std::string_view::npos)
			break;
		Text.remove_prefix(end + 1);
	}
}

std::uint64_t to_number(std::string_view Text)
{
	std::uint64_t result{0};
	for (auto const c : Text)
	{
		if (!std::isdigit(static_cast<unsigned char>(c)))
			break;
		result = result * 10 + static_cast<std::uint64_t>(c - '0');
	}
	return result;
}

// the state of reading /proc/cpuinfo, line by line
struct cpuinfo_reader
{
	cpu_info cpu;
	std::set<std::pair<std::string, std::string>> cores; // (physical id, core id)
	std::set<std::string, std::less<>> packages;
	int cores_per_package{0};
	std::string package;

	void read(std::string_view Line);
};

void cpuinfo_reader::read(std::string_view const Line)
{
	auto const [key, value] = split_entry(Line);
	if (key == "processor")
	{
		++cpu.logical_cores;
		package.clear();
	}
	else if ((key == "model name" || key == "Model") && cpu.model.empty())
	{
		cpu.model = tidy_name(value);
	}
	else if (key == "physical id")
	{
		package = value;
		packages.emplace(value);
	}
	else if (key == "core id")
	{
		cores.emplace(package, value);
	}
	else if (key == "cpu cores" && cores_per_package == 0)
	{
		cores_per_package = static_cast<int>(to_number(value));
	}
	else if (key == "flags" && cpu.avx2 < 0)
	{
		auto const flags{name_words(value)};
		cpu.avx2 = has_word(flags, "avx2") ? 1 : 0;
	}
}

} // namespace

std::string_view to_string(gpu_kind const Kind)
{
	using enum gpu_kind;
	switch (Kind)
	{
	case integrated:
		return "integrated";
	case discrete:
		return "discrete";
	case software:
		return "software";
	default:
		return "unknown";
	}
}

gpu_kind classify_gpu(std::uint32_t const VendorId, std::string_view const Name)
{
	using enum gpu_kind;
	if (is_software_name(Name))
		return software;
	auto const words{name_words(Name)};
	switch (VendorId)
	{
	case vendor_nvidia:
		return discrete;
	case vendor_intel:
		return classify_intel(words);
	case vendor_amd:
	case vendor_amd_alt:
		return classify_amd(words);
	default:
		return unknown;
	}
}

std::uint32_t vendor_from_gl(std::string_view const GlVendor, std::string_view const GlRenderer)
{
	// the vendor first; Mesa ("Mesa", "X.Org", "Collabora Ltd") and the D3D12 layer ("Microsoft Corporation") name
	// the maker of the chip in the renderer
	for (auto const &text : {lowercase(GlVendor), lowercase(GlRenderer)})
	{
		if (contains(text, "nvidia") || contains(text, "geforce") || contains(text, "quadro"))
			return vendor_nvidia;
		if (contains(text, "ati technologies") || contains(text, "advanced micro devices") || contains(text, "radeon") || has_word(name_words(text), "amd"))
			return vendor_amd;
		if (contains(text, "intel"))
			return vendor_intel;
	}
	return 0;
}

bool is_software_gl(std::string_view const GlVendor, std::string_view const GlRenderer)
{
	return is_software_name(GlRenderer) || is_software_name(GlVendor);
}

gpu_kind classify_gl(std::string_view const GlVendor, std::string_view const GlRenderer)
{
	if (is_software_gl(GlVendor, GlRenderer))
		return gpu_kind::software;
	return classify_gpu(vendor_from_gl(GlVendor, GlRenderer), GlRenderer);
}

std::string format_size(std::uint64_t const Bytes)
{
	constexpr std::uint64_t mebibyte{1024 * 1024};
	constexpr std::uint64_t gibibyte{1024 * mebibyte};
	if (Bytes < gibibyte)
		return std::format("{} MiB", (Bytes + mebibyte / 2) / mebibyte);
	return std::format("{:.1f} GiB", static_cast<double>(Bytes) / static_cast<double>(gibibyte));
}

std::string tidy_name(std::string_view const Name)
{
	std::string result;
	for (auto const c : Name)
	{
		if (c == '\0')
			break;
		if (std::isspace(static_cast<unsigned char>(c)))
		{
			if (!result.empty() && result.back() != ' ')
				result += ' ';
			continue;
		}
		result += c;
	}
	if (!result.empty() && result.back() == ' ')
		result.pop_back();
	return result;
}

cpu_info parse_cpuinfo(std::string_view const Text)
{
	cpuinfo_reader reader;
	for_each_line(Text, [&reader](std::string_view const Line) { reader.read(Line); });
	auto cpu{std::move(reader.cpu)};
	if (!reader.cores.empty())
		cpu.physical_cores = static_cast<int>(reader.cores.size());
	else if (reader.cores_per_package > 0)
		cpu.physical_cores = reader.cores_per_package * static_cast<int>(std::max<std::size_t>(reader.packages.size(), 1));
	return cpu;
}

memory_info parse_meminfo(std::string_view const Text)
{
	memory_info memory;
	for_each_line(Text, [&memory](std::string_view const Line) {
		auto const [key, value] = split_entry(Line);
		// the values are in kB (kibibytes, whatever the unit says)
		if (key == "MemTotal")
			memory.total = to_number(value) * 1024;
		else if (key == "MemAvailable")
			memory.available = to_number(value) * 1024;
	});
	return memory;
}

std::string format_cpu(cpu_info const &Cpu)
{
	auto line{std::format("hardware: cpu: {}", Cpu.model.empty() ? std::string{"unknown"} : Cpu.model)};
	if (Cpu.physical_cores > 0)
		line += std::format(", {} cores", Cpu.physical_cores);
	if (Cpu.logical_cores > 0)
		line += std::format(", {} threads", Cpu.logical_cores);
	line += std::format(", avx2 {}", yes_no(Cpu.avx2));
	return line;
}

std::string format_memory(memory_info const &Memory)
{
	auto line{std::format("hardware: ram: {} total", Memory.total > 0 ? format_size(Memory.total) : std::string{"unknown"})};
	if (Memory.available > 0)
		line += std::format(", {} available", format_size(Memory.available));
	return line;
}

std::string format_adapter(int const Index, gpu_adapter const &Adapter)
{
	return std::format("hardware: gpu {}: {} (vendor 0x{:04x}, device 0x{:04x}), {} dedicated, {} shared, {} outputs, {}", Index, Adapter.description, Adapter.vendor_id, Adapter.device_id,
	                   format_size(Adapter.dedicated_video_memory), format_size(Adapter.shared_system_memory), Adapter.outputs,
	                   Adapter.software ? to_string(gpu_kind::software) : to_string(classify_gpu(Adapter.vendor_id, Adapter.description)));
}

std::string format_gl(std::string_view const GlVendor, std::string_view const GlRenderer)
{
	return std::format("hardware: opengl gpu: {} ({}), {}", GlRenderer, GlVendor, to_string(classify_gl(GlVendor, GlRenderer)));
}

std::string format_gl_memory(gl_memory_info const &Memory)
{
	std::string line{"hardware: opengl gpu memory:"};
	if (Memory.dedicated > 0)
		line += std::format(" {} dedicated", format_size(Memory.dedicated));
	if (Memory.available > 0)
		line += std::format("{} {} free", Memory.dedicated > 0 ? "," : "", format_size(Memory.available));
	if (Memory.dedicated == 0 && Memory.available == 0)
		line += " unknown";
	return line;
}

std::vector<std::string> gl_warnings(std::vector<gpu_adapter> const &Adapters, std::string_view const GlVendor, std::string_view const GlRenderer)
{
	std::vector<std::string> warnings;
	auto const kind{classify_gl(GlVendor, GlRenderer)};
	if (kind == gpu_kind::software)
	{
		warnings.push_back(std::format("hardware: warning: OpenGL draws on the cpu ({}), the graphics driver is missing or not used", GlRenderer));
		return warnings;
	}
	if (kind != gpu_kind::integrated)
		return warnings;
	if (auto const discrete = std::ranges::find_if(Adapters, [](gpu_adapter const &Adapter) {
		    return !Adapter.software && classify_gpu(Adapter.vendor_id, Adapter.description) == gpu_kind::discrete;
	    });
	    discrete != Adapters.end())
	{
		warnings.push_back(std::format("hardware: warning: OpenGL runs on the integrated GPU ({}) though there is a dedicated one ({}); "
		                               "choose High performance for the game in the Windows graphics settings or the GPU control panel, "
		                               "and connect the monitor to the dedicated GPU",
		                               GlRenderer, discrete->description));
	}
	return warnings;
}

std::vector<std::string> cpu_warnings(cpu_info const &Cpu, bool const BuildNeedsAvx2)
{
	std::vector<std::string> warnings;
	if (BuildNeedsAvx2 && Cpu.avx2 == 0)
		warnings.emplace_back("hardware: warning: the cpu has no AVX2, which this build of the game needs");
	return warnings;
}

} // namespace hardware
