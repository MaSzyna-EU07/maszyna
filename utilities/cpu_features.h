#pragma once

// the processor features the simulator is built to use (ENABLE_AVX2), checked before any code built with them runs.
// header only and free of the C runtime: the simulator checks them from its entry point, before the runtime starts

#if defined(_M_X64) || defined(__x86_64__)
#define EU07_CPU_FEATURES_X64 1
#ifdef _MSC_VER
#include <intrin.h>
#else
#include <cpuid.h>
#endif
#endif

namespace cpu_features
{

enum feature : unsigned int
{
	avx2 = 1 << 0,
	fma3 = 1 << 1,
	bmi1 = 1 << 2,
	bmi2 = 1 << 3,
	lzcnt = 1 << 4,
	// the processor has AVX, but the system does not save its registers, so it is turned off (old systems, some virtual machines)
	os_avx = 1 << 5,
};

// the exit code of the simulator stopped by the check of the processor, after it told the user (eu07_starter stays silent then)
constexpr unsigned int exit_unsupported{0xE0450010};

// msvc builds with /arch:AVX2 use the bit manipulation instructions of the same processor generation as well,
// gcc and clang with -mavx2 only AVX2 itself. every processor with AVX2 (Intel Haswell, AMD Excavator and newer) has them all
#ifdef _MSC_VER
constexpr unsigned int required_for_avx2{avx2 | fma3 | bmi1 | bmi2 | lzcnt | os_avx};
#else
constexpr unsigned int required_for_avx2{avx2 | os_avx};
#endif

#ifdef EU07_CPU_FEATURES_X64

inline void cpuid(int const Leaf, int const Subleaf, int Registers[4])
{
#ifdef _MSC_VER
	__cpuidex(Registers, Leaf, Subleaf);
#else
	unsigned int a, b, c, d;
	__cpuid_count(Leaf, Subleaf, a, b, c, d);
	Registers[0] = static_cast<int>(a);
	Registers[1] = static_cast<int>(b);
	Registers[2] = static_cast<int>(c);
	Registers[3] = static_cast<int>(d);
#endif
}

// the register state the system saves on a thread switch; xgetbv may be used only when cpuid reports OSXSAVE
inline unsigned long long xcr0()
{
#ifdef _MSC_VER
	return _xgetbv(0);
#else
	unsigned int low, high;
	__asm__ volatile("xgetbv" : "=a"(low), "=d"(high) : "c"(0));
	return (static_cast<unsigned long long>(high) << 32) | low;
#endif
}

// the features of the processor, a mask of the feature values
inline unsigned int present()
{
	int registers[4];
	cpuid(0, 0, registers);
	int const maxleaf{registers[0]};
	cpuid(0x80000000, 0, registers);
	unsigned int const maxextendedleaf{static_cast<unsigned int>(registers[0])};

	unsigned int result{0};
	cpuid(1, 0, registers);
	bool const hasavx{(registers[2] & (1 << 28)) != 0};
	bool const hasosxsave{(registers[2] & (1 << 27)) != 0};
	if ((registers[2] & (1 << 12)) != 0)
	{
		result |= fma3;
	}
	// bits 1 and 2: the SSE and AVX registers
	if (hasavx && hasosxsave && ((xcr0() & 0x6) == 0x6))
	{
		result |= os_avx;
	}
	if (maxleaf >= 7)
	{
		cpuid(7, 0, registers);
		if ((registers[1] & (1 << 5)) != 0)
		{
			result |= avx2;
		}
		if ((registers[1] & (1 << 3)) != 0)
		{
			result |= bmi1;
		}
		if ((registers[1] & (1 << 8)) != 0)
		{
			result |= bmi2;
		}
	}
	if (maxextendedleaf >= 0x80000001)
	{
		cpuid(0x80000001, 0, registers);
		if ((registers[2] & (1 << 5)) != 0)
		{
			result |= lzcnt;
		}
	}
	return result;
}

// the name of the processor, up to 48 characters
inline void brand(char Output[49])
{
	int registers[4];
	cpuid(0x80000000, 0, registers);
	if (static_cast<unsigned int>(registers[0]) < 0x80000004)
	{
		Output[0] = '\0';
		return;
	}
	for (int leaf = 0; leaf < 3; ++leaf)
	{
		cpuid(0x80000002 + leaf, 0, registers);
		for (int index = 0; index < 16; ++index)
		{
			Output[leaf * 16 + index] = reinterpret_cast<char const *>(registers)[index];
		}
	}
	Output[48] = '\0';
}

#else

inline unsigned int present()
{
	return ~0u;
}

inline void brand(char Output[49])
{
	Output[0] = '\0';
}

#endif

// the required features the processor does not have
inline unsigned int missing(unsigned int const Required)
{
	return Required & ~present();
}

// the name of a single feature
inline char const *name(unsigned int const Feature)
{
	switch (Feature)
	{
	case avx2:
		return "AVX2";
	case fma3:
		return "FMA3";
	case bmi1:
		return "BMI1";
	case bmi2:
		return "BMI2";
	case lzcnt:
		return "LZCNT";
	case os_avx:
		return "AVX support of the operating system";
	default:
		return "?";
	}
}

} // namespace cpu_features
