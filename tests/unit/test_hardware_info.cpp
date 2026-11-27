#include "stdafx.h"
#include "doctest.h"
#include "utilities/hardwareinfo.h"

// what the log says of the machine at startup (utilities/hardwareinfo.h): the names of real GPUs, as DXGI and the
// OpenGL drivers give them, and the text of /proc on Linux
using namespace hardware;

TEST_CASE("hardware info: NVIDIA GPUs are discrete")
{
	CHECK(classify_gpu(vendor_nvidia, "NVIDIA GeForce RTX 3060") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_nvidia, "NVIDIA GeForce MX150") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_nvidia, "NVIDIA RTX A2000 Laptop GPU") == gpu_kind::discrete);
}

TEST_CASE("hardware info: Intel GPUs are integrated, except the Arc cards and Iris Xe MAX")
{
	CHECK(classify_gpu(vendor_intel, "Intel(R) UHD Graphics 630") == gpu_kind::integrated);
	CHECK(classify_gpu(vendor_intel, "Intel(R) HD Graphics 4600") == gpu_kind::integrated);
	CHECK(classify_gpu(vendor_intel, "Intel(R) Iris(R) Xe Graphics") == gpu_kind::integrated);
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) Graphics") == gpu_kind::integrated); // Meteor Lake
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) 140V GPU (16GB)") == gpu_kind::integrated); // Lunar Lake
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) B390 GPU") == gpu_kind::integrated); // Panther Lake
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) B370 GPU") == gpu_kind::integrated);
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) A310 Graphics") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) A380 Graphics") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) A370M Graphics") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) B570 Graphics") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) A770 Graphics") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) B580 Graphics") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_intel, "Intel(R) Arc(TM) Pro A60 Graphics") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_intel, "Intel(R) Iris(R) Xe MAX Graphics") == gpu_kind::discrete);
}

TEST_CASE("hardware info: AMD cards are discrete, the APUs integrated, the rest unknown")
{
	CHECK(classify_gpu(vendor_amd, "AMD Radeon RX 6600") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_amd, "Radeon RX 580 Series") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_amd, "AMD Radeon Pro W6600") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_amd, "AMD Radeon VII") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_amd, "AMD FirePro W5100") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_amd, "AMD Radeon(TM) Graphics") == gpu_kind::integrated);
	CHECK(classify_gpu(vendor_amd, "AMD Radeon(TM) Vega 8 Graphics") == gpu_kind::integrated);
	CHECK(classify_gpu(vendor_amd, "AMD Radeon 780M Graphics") == gpu_kind::integrated);
	CHECK(classify_gpu(vendor_amd, "AMD Radeon(TM) R7 Graphics") == gpu_kind::integrated);
	CHECK(classify_gpu(vendor_amd, "AMD Radeon(TM) RX Vega 11 Graphics") == gpu_kind::integrated); // Raven Ridge
	CHECK(classify_gpu(vendor_amd, "AMD Radeon(TM) RX Vega 10 Graphics") == gpu_kind::integrated);
	CHECK(classify_gpu(vendor_amd, "Radeon RX Vega") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_amd, "AMD Radeon RX Vega 56") == gpu_kind::discrete);
	CHECK(classify_gpu(vendor_amd, "Radeon RX Vega M GH Graphics") == gpu_kind::discrete); // Kaby Lake G, own memory
	CHECK(classify_gpu(vendor_amd, "AMD Radeon R7 370") == gpu_kind::unknown);
	CHECK(classify_gpu(vendor_amd, "AMD Radeon HD 7850") == gpu_kind::unknown);
	CHECK(classify_gpu(vendor_amd_alt, "AMD Radeon(TM) Graphics") == gpu_kind::integrated);
}

TEST_CASE("hardware info: other vendors are unknown, software renderers software")
{
	CHECK(classify_gpu(0x15ad, "VMware SVGA 3D") == gpu_kind::unknown);
	CHECK(classify_gpu(0x5143, "Qualcomm(R) Adreno(TM) X1-85 GPU") == gpu_kind::unknown);
	CHECK(classify_gpu(vendor_microsoft, "Microsoft Basic Render Driver") == gpu_kind::software);
}

TEST_CASE("hardware info: the OpenGL vendor and renderer strings")
{
	// Windows drivers
	CHECK(vendor_from_gl("NVIDIA Corporation", "NVIDIA GeForce RTX 3060/PCIe/SSE2") == vendor_nvidia);
	CHECK(vendor_from_gl("ATI Technologies Inc.", "AMD Radeon RX 6600") == vendor_amd);
	CHECK(vendor_from_gl("Intel", "Intel(R) UHD Graphics 630") == vendor_intel);
	// Mesa names the maker in the renderer
	CHECK(vendor_from_gl("Mesa", "Mesa Intel(R) UHD Graphics 620 (KBL GT2)") == vendor_intel);
	CHECK(vendor_from_gl("X.Org", "AMD Radeon Graphics (radeonsi, renoir, LLVM 15.0.7, DRM 3.49)") == vendor_amd);
	CHECK(vendor_from_gl("Mesa", "llvmpipe (LLVM 15.0.7, 256 bits)") == 0);

	CHECK(classify_gl("NVIDIA Corporation", "NVIDIA GeForce RTX 3060/PCIe/SSE2") == gpu_kind::discrete);
	CHECK(classify_gl("Intel", "Intel(R) UHD Graphics 630") == gpu_kind::integrated);
	CHECK(classify_gl("ATI Technologies Inc.", "AMD Radeon(TM) Graphics") == gpu_kind::integrated);
	CHECK(classify_gl("X.Org", "AMD Radeon Graphics (radeonsi, renoir, LLVM 15.0.7, DRM 3.49)") == gpu_kind::integrated);
	CHECK(classify_gl("AMD", "AMD Radeon RX 6600 (radeonsi, navi23, LLVM 17.0.6, DRM 3.57)") == gpu_kind::discrete);
	CHECK(classify_gl("Intel", "Mesa Intel(R) Arc(tm) A770 Graphics (DG2)") == gpu_kind::discrete);
	CHECK(classify_gl("Mesa", "llvmpipe (LLVM 15.0.7, 256 bits)") == gpu_kind::software);
	CHECK(classify_gl("Microsoft Corporation", "GDI Generic") == gpu_kind::software);
	CHECK(classify_gl("nouveau", "NV138") == gpu_kind::unknown);
}

namespace {
gpu_adapter adapter(std::string Description, std::uint32_t const VendorId, bool const Software = false)
{
	gpu_adapter result;
	result.description = std::move(Description);
	result.vendor_id = VendorId;
	result.software = Software;
	return result;
}
} // namespace

TEST_CASE("hardware info: a warning when OpenGL runs on the integrated GPU beside a dedicated one")
{
	std::vector const laptop{adapter("Intel(R) UHD Graphics 630", vendor_intel), adapter("NVIDIA GeForce GTX 1650", vendor_nvidia)};

	auto const warnings{gl_warnings(laptop, "Intel", "Intel(R) UHD Graphics 630")};
	REQUIRE(warnings.size() == 1);
	CHECK(warnings.front().starts_with("hardware: warning: "));
	CHECK(warnings.front().find("Intel(R) UHD Graphics 630") != std::string::npos);
	CHECK(warnings.front().find("NVIDIA GeForce GTX 1650") != std::string::npos);

	// on the dedicated one all is well
	CHECK(gl_warnings(laptop, "NVIDIA Corporation", "NVIDIA GeForce GTX 1650/PCIe/SSE2").empty());
	// a desktop with the integrated GPU only, or a GPU that can't be told, says nothing
	CHECK(gl_warnings({adapter("Intel(R) UHD Graphics 630", vendor_intel)}, "Intel", "Intel(R) UHD Graphics 630").empty());
	CHECK(gl_warnings({adapter("AMD Radeon HD 7850", vendor_amd), adapter("Intel(R) HD Graphics 4600", vendor_intel)}, "Intel", "Intel(R) HD Graphics 4600").empty());
	// Linux lists no adapters
	CHECK(gl_warnings({}, "Intel", "Mesa Intel(R) UHD Graphics 620 (KBL GT2)").empty());
	// an AMD APU beside an AMD card
	CHECK(gl_warnings({adapter("AMD Radeon(TM) Graphics", vendor_amd), adapter("AMD Radeon RX 6600M", vendor_amd)}, "ATI Technologies Inc.", "AMD Radeon(TM) Graphics").size() == 1);
	// a Ryzen 2400G with the monitor on the motherboard beside a GeForce, a Panther Lake laptop running GL on the B390
	CHECK(gl_warnings({adapter("AMD Radeon(TM) RX Vega 11 Graphics", vendor_amd), adapter("NVIDIA GeForce GTX 1650", vendor_nvidia)}, "ATI Technologies Inc.", "AMD Radeon(TM) RX Vega 11 Graphics").size() == 1);
	CHECK(gl_warnings({adapter("Intel(R) Arc(TM) B390 GPU", vendor_intel), adapter("NVIDIA GeForce RTX 5060 Laptop GPU", vendor_nvidia)}, "Intel", "Intel(R) Arc(TM) B390 GPU").size() == 1);
}

TEST_CASE("hardware info: a warning when OpenGL draws on the cpu")
{
	auto const warnings{gl_warnings({adapter("NVIDIA GeForce GTX 1650", vendor_nvidia)}, "Microsoft Corporation", "GDI Generic")};
	REQUIRE(warnings.size() == 1);
	CHECK(warnings.front().find("GDI Generic") != std::string::npos);
}

TEST_CASE("hardware info: a warning for a cpu without AVX2 only when the build needs it")
{
	cpu_info cpu;
	cpu.avx2 = 0;
	CHECK(cpu_warnings(cpu, true).size() == 1);
	CHECK(cpu_warnings(cpu, false).empty());
	cpu.avx2 = 1;
	CHECK(cpu_warnings(cpu, true).empty());
	cpu.avx2 = -1; // can't tell: no warning
	CHECK(cpu_warnings(cpu, true).empty());
}

TEST_CASE("hardware info: sizes")
{
	CHECK(format_size(0) == "0 MiB");
	CHECK(format_size(128ULL * 1024 * 1024) == "128 MiB");
	CHECK(format_size(1023ULL * 1024 * 1024) == "1023 MiB");
	CHECK(format_size(1024ULL * 1024 * 1024) == "1.0 GiB");
	CHECK(format_size(12ULL * 1024 * 1024 * 1024) == "12.0 GiB");
	CHECK(format_size(17'058'000'000ULL) == "15.9 GiB");
}

TEST_CASE("hardware info: names")
{
	CHECK(tidy_name("       Intel(R) Core(TM) i5-4430 CPU @ 3.00GHz") == "Intel(R) Core(TM) i5-4430 CPU @ 3.00GHz");
	CHECK(tidy_name("AMD Ryzen 5 5600X 6-Core Processor             ") == "AMD Ryzen 5 5600X 6-Core Processor");
	CHECK(tidy_name("") == "");
}

TEST_CASE("hardware info: /proc/cpuinfo")
{
	// two cores with two threads each, one package; the flags of the first processor count
	auto const cpu{parse_cpuinfo("processor\t: 0\n"
	                             "model name\t: Intel(R) Core(TM) i3-7100U CPU @ 2.40GHz\n"
	                             "physical id\t: 0\n"
	                             "core id\t\t: 0\n"
	                             "cpu cores\t: 2\n"
	                             "flags\t\t: fpu vme sse sse2 avx avx2 bmi2\n"
	                             "\n"
	                             "processor\t: 1\n"
	                             "model name\t: Intel(R) Core(TM) i3-7100U CPU @ 2.40GHz\n"
	                             "physical id\t: 0\n"
	                             "core id\t\t: 1\n"
	                             "\n"
	                             "processor\t: 2\n"
	                             "physical id\t: 0\n"
	                             "core id\t\t: 0\n"
	                             "\n"
	                             "processor\t: 3\n"
	                             "physical id\t: 0\n"
	                             "core id\t\t: 1\n")};
	CHECK(cpu.model == "Intel(R) Core(TM) i3-7100U CPU @ 2.40GHz");
	CHECK(cpu.logical_cores == 4);
	CHECK(cpu.physical_cores == 2);
	CHECK(cpu.avx2 == 1);

	// no core ids (some virtual machines): the cores of the package; no avx2 in the flags
	auto const vm{parse_cpuinfo("processor : 0\nmodel name : Some CPU\ncpu cores : 4\nflags : sse sse2 avx\n")};
	CHECK(vm.physical_cores == 4);
	CHECK(vm.avx2 == 0);

	// nothing known
	auto const empty{parse_cpuinfo("")};
	CHECK(empty.model.empty());
	CHECK(empty.logical_cores == 0);
	CHECK(empty.physical_cores == 0);
	CHECK(empty.avx2 == -1);
}

TEST_CASE("hardware info: /proc/meminfo")
{
	auto const memory{parse_meminfo("MemTotal:       16303064 kB\nMemFree:         1234567 kB\nMemAvailable:    8151532 kB\n")};
	CHECK(memory.total == 16303064ULL * 1024);
	CHECK(memory.available == 8151532ULL * 1024);
	CHECK(parse_meminfo("").total == 0);
}

TEST_CASE("hardware info: the lines of the log")
{
	cpu_info cpu;
	cpu.model = "AMD Ryzen 5 5600X 6-Core Processor";
	cpu.physical_cores = 6;
	cpu.logical_cores = 12;
	cpu.avx2 = 1;
	CHECK(format_cpu(cpu) == "hardware: cpu: AMD Ryzen 5 5600X 6-Core Processor, 6 cores, 12 threads, avx2 yes");
	CHECK(format_cpu(cpu_info{}) == "hardware: cpu: unknown, avx2 unknown");

	memory_info memory;
	memory.total = 16ULL * 1024 * 1024 * 1024;
	memory.available = 7ULL * 1024 * 1024 * 1024;
	CHECK(format_memory(memory) == "hardware: ram: 16.0 GiB total, 7.0 GiB available");
	CHECK(format_memory(memory_info{}) == "hardware: ram: unknown total");

	gpu_adapter gpu{adapter("NVIDIA GeForce GTX 1650", vendor_nvidia)};
	gpu.device_id = 0x1f82;
	gpu.dedicated_video_memory = 4ULL * 1024 * 1024 * 1024;
	gpu.shared_system_memory = 8ULL * 1024 * 1024 * 1024;
	gpu.outputs = 1;
	CHECK(format_adapter(1, gpu) == "hardware: gpu 1: NVIDIA GeForce GTX 1650 (vendor 0x10de, device 0x1f82), 4.0 GiB dedicated, 8.0 GiB shared, 1 outputs, discrete");

	CHECK(format_gl("Intel", "Intel(R) UHD Graphics 630") == "hardware: opengl gpu: Intel(R) UHD Graphics 630 (Intel), integrated");

	gl_memory_info gl;
	gl.dedicated = 4ULL * 1024 * 1024 * 1024;
	gl.available = 3ULL * 1024 * 1024 * 1024;
	CHECK(format_gl_memory(gl) == "hardware: opengl gpu memory: 4.0 GiB dedicated, 3.0 GiB free");
	gl.dedicated = 0;
	CHECK(format_gl_memory(gl) == "hardware: opengl gpu memory: 3.0 GiB free");
	CHECK(format_gl_memory(gl_memory_info{}) == "hardware: opengl gpu memory: unknown");
}
