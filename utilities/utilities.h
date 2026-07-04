/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "stdafx.h"
#include "utilities/parser.h"

/*rozne takie duperele do operacji na stringach w paszczalu, pewnie w delfi sa lepsze*/
/*konwersja zmiennych na stringi, funkcje matematyczne, logiczne, lancuchowe, I/O etc*/

template <typename T> T sign(T x)
{
	return x < static_cast<T>(0) ? static_cast<T>(-1) : x > static_cast<T>(0) ? static_cast<T>(1) : static_cast<T>(0);
}

template <typename T> constexpr T sq(T v)
{
	return v * v;
}

// returns the per-user config path for `filename` (platform-specific dir),
// or an empty path if the home/appdata env var is unavailable
std::filesystem::path user_config_path(const std::string &filename);

// TODO: Shouldn't this be in globals?
namespace paths
{
inline constexpr auto scenery = "scenery/";
inline constexpr auto textures = "textures/";
inline constexpr auto models = "models/";
inline constexpr auto dynamic = "dynamic/";
inline constexpr auto sounds = "sounds/";
inline constexpr auto data = "data/";
}

#define MAKE_ID4(a, b, c, d) (((std::uint32_t)(d) << 24) | ((std::uint32_t)(c) << 16) | ((std::uint32_t)(b) << 8) | (std::uint32_t)(a))

extern bool DebugModeFlag;
extern bool FreeFlyModeFlag;
extern bool EditorModeFlag;
extern bool DebugCameraFlag;
extern bool DebugTractionFlag;

/*funkcje matematyczne*/
inline double Sign(const double x)
{
	return x >= 0 ? 1.0 : -1.0;
}

inline long Round(double const f)
{
	return (long)(f + 0.5);
	// return lround(f);
}

double Random(double a, double b);
int Random(int min, int max);
std::string generate_uuid_v4();
double LocalRandom(double a, double b);

inline double Random()
{
	return Random(0.0, 1.0);
}

inline double Random(const double b)
{
	return Random(0.0, b);
}

inline double LocalRandom()
{
	return LocalRandom(0.0, 1.0);
}

inline double LocalRandom(const double b)
{
	return LocalRandom(0.0, b);
}

inline double BorlandTime()
{
	const auto timesinceepoch = std::time(nullptr);
	return timesinceepoch / (24.0 * 60 * 60);
	/*
	    // std alternative
	    auto timesinceepoch = std::chrono::system_clock::now().time_since_epoch();
	    return std::chrono::duration_cast<std::chrono::seconds>( timesinceepoch ).count() / (24.0 * 60 * 60);
	*/
}

std::string Now();

double CompareTime(double t1h, double t1m, double t2h, double t2m);

/*funkcje logiczne*/
inline bool TestFlag(int const Flag, int const Value)
{
	return (Flag & Value) == Value;
}
inline bool TestFlagAny(int const Flag, int const Value)
{
	return (Flag & Value) != 0;
}
bool SetFlag(int &Flag, int Value);
bool ClearFlag(int &Flag, int Value);

bool FuzzyLogic(double Test, double Threshold, double Probability);
/*jesli Test>Threshold to losowanie*/
bool FuzzyLogicAI(double Test, double Threshold, double Probability);
/*to samo ale zawsze niezaleznie od DebugFlag*/

/*operacje na stringach*/
std::vector<std::string> Split(std::string_view s, char delim);
std::pair<std::string, int> split_string_and_number(std::string const &Key);

std::string to_string(int Value, int width);
std::string to_string(double Value, int precision);
std::string to_string(double Value, int precision, int width);
std::string to_hex_str(int Value, int width = 4);
std::string to_minutes_str(float Minutes, bool Leadingzero, int Width);

template <std::same_as<bool> T> // Without this line this function can be used with other types implicit casted to boolean which may create hard to debug bugs.
std::string to_string(T Value)
{
	return Value == true ? "true" : "false";
}

template <typename Type_, glm::precision Precision_ = glm::defaultp> std::string to_string(glm::tvec3<Type_, Precision_> const &Value)
{
	return to_string(Value.x, 2) + ", " + to_string(Value.y, 2) + ", " + to_string(Value.z, 2);
}

template <typename Type_, glm::precision Precision_ = glm::defaultp> std::string to_string(glm::tvec4<Type_, Precision_> const &Value, int const Width = 2)
{
	return to_string(Value.x, Width) + ", " + to_string(Value.y, Width) + ", " + to_string(Value.z, Width) + ", " + to_string(Value.w, Width);
}

int stol_def(const std::string &str, const int &DefaultValue);

std::string ToLower(std::string const &text);
std::string ToUpper(std::string const &text);

// replaces polish letters with basic ascii
void win1250_to_ascii(std::string &Input);
// TODO: unify with win1250_to_ascii()
std::string Bezogonkow(std::string Input, bool Underscorestospaces = false);

std::string win1250_to_utf8(const std::string &input);

inline std::string extract_value(std::string const &Key, std::string const &Input)
{
	// NOTE, HACK: the leading space allows to uniformly look for " variable=" substring
	std::string const input{" " + Input};
	std::string value;
	auto lookup = input.find(" " + Key + "=");
	if (lookup != std::string::npos)
	{
		value = input.substr(input.find_first_not_of(' ', lookup + Key.size() + 2));
		lookup = value.find(' ');
		if (lookup != std::string::npos)
		{
			// trim everything past the value
			value.erase(lookup);
		}
	}
	return value;
}

template <typename Type_> bool extract_value(Type_ &Variable, std::string const &Key, std::string const &Input, std::string const &Default)
{

	const auto value = extract_value(Key, Input);
	if (false == value.empty())
	{
		// set the specified variable to retrieved value
		std::stringstream converter;
		converter << value;
		converter >> Variable;
		return true; // located the variable
	}
	else
	{
		// set the variable to provided default value
		if (false == Default.empty())
		{
			std::stringstream converter;
			converter << Default;
			converter >> Variable;
		}
		return false; // couldn't locate the variable in provided input
	}
}

template <> bool extract_value(bool &Variable, std::string const &Key, std::string const &Input, std::string const &Default);

bool FileExists(std::string const &Filename);

std::pair<std::string, std::string> FileExists(std::vector<std::string> const &Names, std::vector<std::string> const &Extensions);

// returns time of last modification for specified file
std::time_t last_modified(std::string const &Filename);

// potentially erases file extension from provided file name. returns: true if extension was removed, false otherwise
bool erase_extension(std::string &Filename);

// potentially erase leading slashes from provided file path
void erase_leading_slashes(std::string &Filename);

// potentially replaces backward slashes in provided file path with unix-compatible forward slashes
void replace_slashes(std::string &Filename);

// returns potential path part from provided file name
std::string substr_path(std::string const &Filename);

// returns common prefix of two provided strings
std::ptrdiff_t len_common_prefix(std::string_view a, std::string_view b);

// returns true if provided string contains another provided string
bool contains(std::string_view String, std::string_view Substring);
bool contains(std::string_view String, char Character);

// TODO: Ideally unique_ptr should be used instead of this (not safe) inline functions
template <typename T> void SafeDelete(T* &Pointer)
{
	delete Pointer;
	Pointer = nullptr;
}

template <typename T> void SafeDeleteArray(T *&Pointer)
{
	delete[] Pointer;
	Pointer = nullptr;
}

template <typename T> bool is_equal(T const &Left, T const &Right, T const Epsilon = T(1e-5))
{
	if (Epsilon != T(0))
		return glm::epsilonEqual(Left, Right, Epsilon);

	return Left == Right;
}

// Tolerant clamp. Unlike std::clamp, this does NOT invoke undefined behaviour when
// the bounds are inverted (Lo > Hi) - it normalizes them instead. This restores the
// pre-refactor behaviour of the old custom clamp() for code paths where the bounds are
// computed at runtime and can legitimately cross (e.g. AI acceleration/braking limits,
// physics jolt calculations), where std::clamp's UB produced wrong results.
template <typename Type_>
constexpr Type_ safe_clamp( Type_ const Value, Type_ const Lo, Type_ const Hi )
{
	return Hi < Lo
		? std::clamp( Value, Hi, Lo )
		: std::clamp( Value, Lo, Hi );
}

// keeps the provided value in specified range 0-Range, as if the range was circular buffer
template <typename T> T clamp_circular(T Value, T const Range = T(360))
{
	if constexpr (std::is_floating_point_v<T>)
	{
		Value = std::fmod(Value, Range);
	}
	else
	{
		Value %= Range;
	}

	if (Value < T(0))
		Value += Range;
	return Value;
}

// rounds down provided value to nearest power of two
template <typename T> T clamp_power_of_two(T Value, T const Min = T(1), T const Max = T(16384))
{
	if (Value < Min)
		return Min;

	T p2 = std::bit_floor(Value);

	if (p2 > Max)
		return Max;

	return p2;
}

template <typename Type_> Type_ quantize(Type_ const Value, Type_ const Step)
{

	return Step * std::round(Value / Step);
}

template <typename T> T min_speed(T const Left, T const Right)
{
	constexpr T none = T(-1);
	if (Left == none)
		return Right;
	if (Right == none)
		return Left;

	return std::min(Left, Right);
}

template <typename T> T smoothInterpolate(T const &First, T const &Second, double Factor)
{
	// Apply smoothing (ease-in-out quadratic)
	Factor = Factor * Factor * (3 - 2 * Factor);

	return First + (Second - First) * Factor;
}

// tests whether provided points form a degenerate triangle
template <typename VecType_> bool degenerate(VecType_ const &Vertex1, VecType_ const &Vertex2, VecType_ const &Vertex3)
{

	//  degenerate( A, B, C, minarea ) = ( ( B - A ).cross( C - A ) ).lengthSquared() < ( 4.0f * minarea * minarea );
	return glm::length2(glm::cross(Vertex2 - Vertex1, Vertex3 - Vertex1)) == 0.0;
}

// calculates bounding box for provided set of points
template <class Iterator_, class VecType_> void bounding_box(VecType_ &Mincorner, VecType_ &Maxcorner, Iterator_ First, Iterator_ Last)
{

	Mincorner = VecType_(std::numeric_limits<typename VecType_::value_type>::max());
	Maxcorner = VecType_(std::numeric_limits<typename VecType_::value_type>::lowest());

	std::for_each(First, Last,
	              [&](typename Iterator_::value_type &point)
	              {
		              Mincorner = glm::min(Mincorner, VecType_{point});
		              Maxcorner = glm::max(Maxcorner, VecType_{point});
	              });
}

// finds point on specified segment closest to specified point in 3d space. returns: point on segment as value in range 0-1 where 0 = start and 1 = end of the segment
template <typename VecType_> typename VecType_::value_type nearest_segment_point(VecType_ const &Segmentstart, VecType_ const &Segmentend, VecType_ const &Point)
{

	auto const v = Segmentend - Segmentstart;
	auto const w = Point - Segmentstart;

	auto const c1 = glm::dot(w, v);
	if (c1 <= 0.0)
	{
		return 0.0;
	}
	auto const c2 = glm::dot(v, v);
	if (c2 <= c1)
	{
		return 1.0;
	}
	return c1 / c2;
}

glm::dvec3 LoadPoint(class cParser &Input);

// extracts a group of tokens from provided data stream
std::string deserialize_random_set(cParser &Input, char const *Break = "\n\r\t ;");

// extracts a group of <key, value> pairs from provided data stream
// NOTE: expects no more than single pair per line
template <typename MapType_> void deserialize_map(MapType_ &Map, cParser &Input)
{

	while (Input.ok() && !Input.eof())
	{
		auto const key{Input.getToken<typename MapType_::key_type>(false)};
		auto const value{Input.getToken<typename MapType_::mapped_type>(false, "\n")};
		Map.emplace(key, value);
	}
}

namespace threading
{

// simple POD pairing of a data item and a mutex
// NOTE: doesn't do any locking itself, it's merely for cleaner argument arrangement and passing
template <typename Type_> struct lockable
{

	Type_ data;
	std::mutex mutex;
};

// basic wrapper simplifying use of std::condition_variable for most typical cases.
// has its own mutex and secondary variable to ignore spurious wakeups
class condition_variable
{

  public:
	// methods
	void wait()
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_condition.wait(lock, [this]() { return m_spurious == false; });
	}
	template <class Rep_, class Period_> void wait_for(const std::chrono::duration<Rep_, Period_> &Time)
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_condition.wait_for(lock, Time, [this]() { return m_spurious == false; });
	}
	void notify_one()
	{
		spurious(false);
		m_condition.notify_one();
	}
	void notify_all()
	{
		spurious(false);
		m_condition.notify_all();
	}
	void spurious(bool const Spurious)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_spurious = Spurious;
	}

  private:
	// members
	mutable std::mutex m_mutex;
	std::condition_variable m_condition;
	bool m_spurious{true};
};

} // namespace threading

//---------------------------------------------------------------------------
