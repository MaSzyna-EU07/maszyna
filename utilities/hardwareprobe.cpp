/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/hardwareprobe.h"

#include "utilities/hardwareinfo.h"
#include "utilities/Logs.h"

#include <cstring>

#if defined(_M_X64) || defined(_M_IX86) || defined(__x86_64__) || defined(__i386__)
#define EU07_HARDWARE_X86 1
#ifdef _MSC_VER
#include <intrin.h>
#else
#include <cpuid.h>
#endif
#endif

#ifdef _WIN32
#include <dxgi.h>
#endif

namespace hardware {

namespace {

// the adapters log_system() found, for the warning of log_gl_context()
std::vector<gpu_adapter> system_adapters;

#ifdef EU07_HARDWARE_X86
void cpuid(unsigned int const Leaf, unsigned int const Subleaf, unsigned int (&Registers)[4])
{
#ifdef _MSC_VER
	int registers[4]{};
	__cpuidex(registers, static_cast<int>(Leaf), static_cast<int>(Subleaf));
	for (int i = 0; i < 4; ++i)
		Registers[i] = static_cast<unsigned int>(registers[i]);
#else
	__cpuid_count(Leaf, Subleaf, Registers[0], Registers[1], Registers[2], Registers[3]);
#endif
}

// the brand string of the cpu ("AMD Ryzen 5 5600X 6-Core Processor"); empty if the cpu has none
std::string cpu_brand()
{
	unsigned int registers[4]{};
	cpuid(0x80000000u, 0, registers);
	if (registers[0] < 0x80000004u)
		return {};
	char brand[49]{};
	for (unsigned int leaf = 0; leaf < 3; ++leaf)
	{
		cpuid(0x80000002u + leaf, 0, registers);
		std::memcpy(brand + leaf * 16, registers, 16);
	}
	return tidy_name(brand);
}

// whether the cpu has AVX2 and the system saves the AVX registers
int cpu_avx2()
{
#ifdef _MSC_VER
	unsigned int registers[4]{};
	cpuid(0, 0, registers);
	if (registers[0] < 7)
		return 0;
	cpuid(1, 0, registers);
	auto const osxsave{(registers[2] & (1u << 27)) != 0};
	auto const avx{(registers[2] & (1u << 28)) != 0};
	if (!osxsave || !avx)
		return 0;
	if ((_xgetbv(0) & 0x6) != 0x6) // the system saves xmm and ymm
		return 0;
	cpuid(7, 0, registers);
	return (registers[1] & (1u << 5)) != 0 ? 1 : 0;
#else
	// checks the system's support as well
	__builtin_cpu_init();
	return __builtin_cpu_supports("avx2") ? 1 : 0;
#endif
}
#endif

#ifdef _WIN32
std::string narrow(wchar_t const *Text)
{
	auto const length{WideCharToMultiByte(CP_UTF8, 0, Text, -1, nullptr, 0, nullptr, nullptr)};
	if (length <= 1)
		return {};
	std::string result(static_cast<std::size_t>(length - 1), '\0');
	WideCharToMultiByte(CP_UTF8, 0, Text, -1, result.data(), length, nullptr, nullptr);
	return result;
}

std::string registry_cpu_name()
{
	char name[256]{};
	DWORD size{sizeof(name)};
	if (RegGetValueA(HKEY_LOCAL_MACHINE, "HARDWARE\\DESCRIPTION\\System\\CentralProcessor\\0", "ProcessorNameString", RRF_RT_REG_SZ, nullptr, name, &size) != ERROR_SUCCESS)
		return {};
	return tidy_name(name);
}

int physical_cores()
{
	DWORD length{0};
	GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &length);
	if (length == 0)
		return 0;
	std::vector<std::uint8_t> buffer(length);
	auto *records{reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *>(buffer.data())};
	if (!GetLogicalProcessorInformationEx(RelationProcessorCore, records, &length))
		return 0;
	int cores{0};
	for (DWORD offset = 0; offset < length;)
	{
		auto const *record{reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX const *>(buffer.data() + offset)};
		if (record->Size == 0)
			break;
		++cores;
		offset += record->Size;
	}
	return cores;
}

cpu_info probe_cpu()
{
	cpu_info cpu;
#ifdef EU07_HARDWARE_X86
	cpu.model = cpu_brand();
	cpu.avx2 = cpu_avx2();
#endif
	if (cpu.model.empty())
		cpu.model = registry_cpu_name();
	cpu.logical_cores = static_cast<int>(GetActiveProcessorCount(ALL_PROCESSOR_GROUPS));
	cpu.physical_cores = physical_cores();
	return cpu;
}

memory_info probe_memory()
{
	memory_info memory;
	MEMORYSTATUSEX status{};
	status.dwLength = sizeof(status);
	if (GlobalMemoryStatusEx(&status))
	{
		memory.total = status.ullTotalPhys;
		memory.available = status.ullAvailPhys;
	}
	return memory;
}

// the adapters DXGI lists, the software one left out. dxgi.dll is loaded only for this, so the game doesn't link it
std::vector<gpu_adapter> probe_adapters()
{
	std::vector<gpu_adapter> adapters;
	auto *library{LoadLibraryW(L"dxgi.dll")};
	if (library == nullptr)
	{
		WriteLog("hardware: gpus: dxgi.dll not found");
		return adapters;
	}
	using create_factory = HRESULT(WINAPI *)(REFIID, void **);
	auto const create{reinterpret_cast<create_factory>(reinterpret_cast<void *>(GetProcAddress(library, "CreateDXGIFactory1")))};
	IDXGIFactory1 *factory{nullptr};
	if (create == nullptr || FAILED(create(__uuidof(IDXGIFactory1), reinterpret_cast<void **>(&factory))) || factory == nullptr)
	{
		WriteLog("hardware: gpus: can't create a DXGI factory");
		FreeLibrary(library);
		return adapters;
	}
	IDXGIAdapter1 *adapter{nullptr};
	for (UINT index = 0; SUCCEEDED(factory->EnumAdapters1(index, &adapter)); ++index)
	{
		DXGI_ADAPTER_DESC1 description{};
		if (SUCCEEDED(adapter->GetDesc1(&description)))
		{
			gpu_adapter entry;
			entry.description = tidy_name(narrow(description.Description));
			entry.vendor_id = description.VendorId;
			entry.device_id = description.DeviceId;
			entry.dedicated_video_memory = description.DedicatedVideoMemory;
			entry.shared_system_memory = description.SharedSystemMemory;
			entry.software = (description.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) != 0 || description.VendorId == vendor_microsoft;
			IDXGIOutput *output{nullptr};
			while (SUCCEEDED(adapter->EnumOutputs(static_cast<UINT>(entry.outputs), &output)))
			{
				output->Release();
				++entry.outputs;
			}
			if (!entry.software)
				adapters.push_back(std::move(entry));
		}
		adapter->Release();
	}
	factory->Release();
	FreeLibrary(library);
	return adapters;
}
#else
std::string read_file(char const *Path)
{
	std::ifstream file{Path};
	if (!file)
		return {};
	std::stringstream text;
	text << file.rdbuf();
	return text.str();
}

cpu_info probe_cpu()
{
	auto cpu{parse_cpuinfo(read_file("/proc/cpuinfo"))};
#ifdef EU07_HARDWARE_X86
	if (cpu.model.empty())
		cpu.model = cpu_brand();
	cpu.avx2 = cpu_avx2();
#endif
	if (cpu.logical_cores == 0)
		cpu.logical_cores = static_cast<int>(std::thread::hardware_concurrency());
	return cpu;
}

memory_info probe_memory()
{
	return parse_meminfo(read_file("/proc/meminfo"));
}
#endif

std::string gl_string(GLenum const Name)
{
	auto const *text{reinterpret_cast<char const *>(glGetString(Name))};
	return text != nullptr ? std::string{text} : std::string{"unknown"};
}

bool has_gl_extension(std::string_view const Name)
{
	if (glGetStringi != nullptr)
	{
		GLint count{0};
		glGetIntegerv(GL_NUM_EXTENSIONS, &count);
		for (GLint i = 0; i < count; ++i)
		{
			auto const *extension{reinterpret_cast<char const *>(glGetStringi(GL_EXTENSIONS, static_cast<GLuint>(i)))};
			if (extension != nullptr && Name == extension)
				return true;
		}
		return false;
	}
	// OpenGL ES 2: one string of them all
	auto const *extensions{reinterpret_cast<char const *>(glGetString(GL_EXTENSIONS))};
	return extensions != nullptr && std::string_view{extensions}.find(Name) != std::string_view::npos;
}

gl_memory_info probe_gl_memory()
{
	// GL_NVX_gpu_memory_info and GL_ATI_meminfo tokens, in kilobytes
	constexpr GLenum gpu_memory_info_dedicated_vidmem_nvx{0x9047};
	constexpr GLenum gpu_memory_info_current_available_vidmem_nvx{0x9049};
	constexpr GLenum texture_free_memory_ati{0x87fc};

	gl_memory_info memory;
	if (has_gl_extension("GL_NVX_gpu_memory_info"))
	{
		GLint kilobytes{0};
		glGetIntegerv(gpu_memory_info_dedicated_vidmem_nvx, &kilobytes);
		memory.dedicated = static_cast<std::uint64_t>(std::max(kilobytes, 0)) * 1024;
		kilobytes = 0;
		glGetIntegerv(gpu_memory_info_current_available_vidmem_nvx, &kilobytes);
		memory.available = static_cast<std::uint64_t>(std::max(kilobytes, 0)) * 1024;
	}
	else if (has_gl_extension("GL_ATI_meminfo"))
	{
		// the free memory of the texture pool, the largest free block, and the same of the auxiliary memory
		GLint kilobytes[4]{};
		glGetIntegerv(texture_free_memory_ati, kilobytes);
		memory.available = static_cast<std::uint64_t>(std::max(kilobytes[0], 0)) * 1024;
	}
	// a driver that takes the queries amiss leaves an error the renderer would report as its own
	while (glGetError() != GL_NO_ERROR)
	{
	}
	return memory;
}

} // namespace

void log_system()
{
	auto const cpu{probe_cpu()};
	WriteLog(format_cpu(cpu));
#ifdef __AVX2__
	constexpr bool build_needs_avx2{true};
#else
	constexpr bool build_needs_avx2{false};
#endif
	for (auto const &warning : cpu_warnings(cpu, build_needs_avx2))
		WriteLog(warning);
	WriteLog(format_memory(probe_memory()));
#ifdef _WIN32
	system_adapters = probe_adapters();
	for (std::size_t index = 0; index < system_adapters.size(); ++index)
		WriteLog(format_adapter(static_cast<int>(index), system_adapters[index]));
#endif
}

void log_gl_context()
{
	auto const vendor{gl_string(GL_VENDOR)};
	auto const renderer{gl_string(GL_RENDERER)};
	WriteLog(format_gl(vendor, renderer));
	auto const memory{probe_gl_memory()};
	if (memory.dedicated > 0 || memory.available > 0)
		WriteLog(format_gl_memory(memory));
	for (auto const &warning : gl_warnings(system_adapters, vendor, renderer))
		WriteLog(warning);
}

} // namespace hardware
