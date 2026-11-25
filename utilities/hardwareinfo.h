/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

// what the log says about the machine at startup: the cpu, the memory, the graphics adapters and the one OpenGL runs
// on. a bug report then shows whether the game ran on the integrated GPU of a laptop instead of the dedicated one.
// this part holds the rules and the text, which need nothing of the machine; the probing is in hardwareprobe.cpp
namespace hardware {

// the PCI vendor ids of the graphics adapters
constexpr std::uint32_t vendor_nvidia{0x10de};
constexpr std::uint32_t vendor_amd{0x1002};
constexpr std::uint32_t vendor_amd_alt{0x1022}; // AMD's cpu id, shown by some APUs
constexpr std::uint32_t vendor_intel{0x8086};
constexpr std::uint32_t vendor_microsoft{0x1414}; // Microsoft Basic Render Driver, the software adapter

enum class gpu_kind {
	unknown, // can't tell from the name; never taken for either of the other two
	integrated, // part of the cpu, shares the system memory
	discrete, // a card (or a chip) with its own memory
	software, // drawn by the cpu (Microsoft Basic Render Driver, GDI Generic, llvmpipe)
};

std::string_view to_string(gpu_kind Kind);

// whether a GPU is integrated or discrete, from its vendor and its name (the DXGI description or GL_RENDERER).
// conservative, by the names the vendors give their chips:
// - NVIDIA: discrete (PC NVIDIA graphics have been cards or laptop chips with their own memory since the nForce days)
// - Intel: discrete only for the Arc cards (Arc A380/A770/B580, Arc Pro A60) and Iris Xe MAX; the rest (HD, UHD,
//   Iris, Arc Graphics, Arc 140V and Arc B370/B390 of Meteor/Lunar/Panther Lake) is integrated
// - AMD: discrete for Radeon RX/Pro/VII, FirePro and Instinct; integrated for the APU names, which have no model
//   number before "Graphics" (Radeon(TM) Graphics, Radeon Vega 8 Graphics, Radeon 780M Graphics, Radeon 8060S
//   Graphics), and Radeon RX Vega 11 Graphics of Raven Ridge; anything else (an old Radeon HD, R7 ...) is unknown, as
//   both cards and APUs were named so
// - software renderers are software; any other vendor (virtual machines, Qualcomm) is unknown
gpu_kind classify_gpu(std::uint32_t VendorId, std::string_view Name);

// the PCI vendor id behind the GL_VENDOR and GL_RENDERER strings (Mesa names the vendor in the renderer); 0 if none
std::uint32_t vendor_from_gl(std::string_view GlVendor, std::string_view GlRenderer);

// whether GL_VENDOR and GL_RENDERER name a renderer that draws on the cpu
bool is_software_gl(std::string_view GlVendor, std::string_view GlRenderer);

// the kind of the GPU an OpenGL context runs on
gpu_kind classify_gl(std::string_view GlVendor, std::string_view GlRenderer);

// a graphics adapter of the system (on Windows, as DXGI lists it)
struct gpu_adapter {
	std::string description;
	std::uint32_t vendor_id{0};
	std::uint32_t device_id{0};
	std::uint64_t dedicated_video_memory{0}; // bytes
	std::uint64_t shared_system_memory{0}; // bytes
	int outputs{0}; // the monitors connected to it
	bool software{false};
};

struct cpu_info {
	std::string model;
	int logical_cores{0}; // 0: unknown
	int physical_cores{0}; // 0: unknown
	int avx2{-1}; // 1 yes, 0 no, -1 unknown
};

struct memory_info {
	std::uint64_t total{0}; // bytes, 0: unknown
	std::uint64_t available{0}; // bytes, 0: unknown
};

// what the context's driver says of its memory (GL_NVX_gpu_memory_info, GL_ATI_meminfo), in bytes, 0: not told
struct gl_memory_info {
	std::uint64_t dedicated{0};
	std::uint64_t available{0};
};

// a size for a person to read: "512 MiB", "7.9 GiB"
std::string format_size(std::uint64_t Bytes);

// trims the padding cpuid and the registry leave in the cpu brand string, squeezes the runs of spaces
std::string tidy_name(std::string_view Name);

// what /proc/cpuinfo says of the cpu: the model name, the processors (logical cores) and the distinct cores
cpu_info parse_cpuinfo(std::string_view Text);

// what /proc/meminfo says of the memory: MemTotal and MemAvailable
memory_info parse_meminfo(std::string_view Text);

// the lines of the log, every one starting with "hardware: " so a single grep finds them all
std::string format_cpu(cpu_info const &Cpu);
std::string format_memory(memory_info const &Memory);
std::string format_adapter(int Index, gpu_adapter const &Adapter);
std::string format_gl(std::string_view GlVendor, std::string_view GlRenderer);
std::string format_gl_memory(gl_memory_info const &Memory);

// the warnings of the log about the GPU OpenGL runs on: on the cpu, or on an integrated GPU while the system has a
// discrete one (Adapters: as the system lists them; the software ones are passed over). nothing if all is well
std::vector<std::string> gl_warnings(std::vector<gpu_adapter> const &Adapters, std::string_view GlVendor, std::string_view GlRenderer);

// the warning of the log about a cpu without AVX2 when the build needs it. nothing if all is well
std::vector<std::string> cpu_warnings(cpu_info const &Cpu, bool BuildNeedsAvx2);

} // namespace hardware
