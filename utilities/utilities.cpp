/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
MaSzyna EU07 - SPKS
Brakes.
Copyright (C) 2007-2014 Maciej Cierniak
*/
//#include "stdafx.h"
//
//#include <sys/types.h>
//#include <sys/stat.h>
#include <ranges>
//#ifndef WIN32
//#include <unistd.h>
//#endif
//
//#ifdef WIN32
//#define stat _stat
//#endif

#include "utilities/utilities.h"
#include "utilities/Globals.h"
#include "utilities/parser.h"

//#include "utilities/Logs.h"

// TODO: This shouldn't be in Globals?
bool DebugModeFlag = false;
bool FreeFlyModeFlag = false;
bool EditorModeFlag = false;
bool DebugCameraFlag = false;
bool DebugTractionFlag = false;

std::string Now()
{
	auto now = std::chrono::system_clock::now();
	auto local = std::chrono::current_zone()->to_local(now);
	return std::format("{:%c}", local);
}

// zwraca różnicę czasu
// jeśli pierwsza jest aktualna, a druga rozkładowa, to ujemna oznacza opóżnienie
// na dłuższą metę trzeba uwzględnić datę, jakby opóżnienia miały przekraczać 12h (towarowych)
double CompareTime(double t1h, double t1m, double t2h, double t2m)
{

	if ((t2h < 0))
		return 0;
	else
	{
		auto t = (t2h - t1h) * 60 + t2m - t1m; // jeśli t2=00:05, a t1=23:50, to różnica wyjdzie ujemna
		if ((t < -720)) // jeśli różnica przekracza 12h na minus
			t = t + 1440; // to dodanie doby minut;else
		if ((t > 720)) // jeśli przekracza 12h na plus
			t = t - 1440; // to odjęcie doby minut
		return t;
	}
}

bool SetFlag(int &Flag, int const Value)
{

	if (Value > 0)
	{
		if (false == TestFlag(Flag, Value))
		{
			Flag |= Value;
			return true; // true, gdy było wcześniej 0 i zostało ustawione
		}
	}
	else if (Value < 0)
	{
		// Value jest ujemne, czyli zerowanie flagi
		return ClearFlag(Flag, -Value);
	}
	return false;
}

bool ClearFlag(int &Flag, int const Value)
{

	if (true == TestFlag(Flag, Value))
	{
		Flag &= ~Value;
		return true;
	}
	else
	{
		return false;
	}
}

double Random(double a, double b)
{
	std::uniform_real_distribution<double> dist(a, b);
	return dist(Global.random_engine);
}

int Random(int min, int max)
{
	std::uniform_int_distribution<int> dist(min, max);
	return dist(Global.random_engine);
}

std::string generate_uuid_v4()
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> dist(0, 255);

	std::array<uint8_t, 16> bytes;
	for (auto &b : bytes)
		b = static_cast<uint8_t>(dist(gen));

	// UUID v4 (RFC 4122)
	bytes[6] = (bytes[6] & 0x0F) | 0x40;
	bytes[8] = (bytes[8] & 0x3F) | 0x80;

	char buf[37]; // 36 znaków + \0
	std::snprintf(buf, sizeof(buf),
	              "%02x%02x%02x%02x-"
	              "%02x%02x-"
	              "%02x%02x-"
	              "%02x%02x-"
	              "%02x%02x%02x%02x%02x%02x",
	              bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8], bytes[9], bytes[10], bytes[11], bytes[12], bytes[13], bytes[14], bytes[15]);

	return std::string(buf);
}

double LocalRandom(double a, double b)
{
	std::uniform_real_distribution<double> dist(a, b);
	return dist(Global.local_random_engine);
}

bool FuzzyLogic(double Test, double Threshold, double Probability)
{
	if ((Test > Threshold) && (!DebugModeFlag))
		return (Random() < Probability * Threshold * 1.0 / Test) /*im wiekszy Test tym wieksza szansa*/;
	else
		return false;
}

bool FuzzyLogicAI(double Test, double Threshold, double Probability)
{
	if ((Test > Threshold))
		return (Random() < Probability * Threshold * 1.0 / Test) /*im wiekszy Test tym wieksza szansa*/;
	else
		return false;
}

std::vector<std::string> Split(std::string_view s, char delim)
{ // dzieli tekst na wektor tekstow
	std::vector<std::string> out;
	for (const auto& part : s | std::ranges::views::split(delim))
		out.emplace_back(part.begin(), part.end());
	return out;
}

std::pair<std::string, int> split_string_and_number(std::string const &Key)
{

	auto const indexstart{Key.find_first_of("-1234567890")};
	auto const indexend{Key.find_first_not_of("-1234567890", indexstart)};
	if (indexstart != std::string::npos)
	{
		return {Key.substr(0, indexstart), std::stoi(Key.substr(indexstart, indexend - indexstart))};
	}
	return {Key, 0};
}

std::string to_string(int Value, int width)
{
	std::ostringstream o;
	o << std::setw(width) << Value;
	return std::move(o).str();
};

std::string to_string(double Value, int precision)
{
	std::ostringstream o;
	o << std::fixed << std::setprecision(precision) << Value;
	return std::move(o).str();
};

std::string to_string(double const Value, int const Precision, int const Width)
{
	std::ostringstream o;
	o << std::setw(Width) << std::fixed << std::setprecision(Precision) << Value;
	return std::move(o).str();
};

std::string to_hex_str(int const Value, int const Width)
{
	std::ostringstream o;
	o << "0x" << std::uppercase << std::setfill('0') << std::setw(Width) << std::hex << Value;
	return o.str();
};

std::string const fractionlabels[] = {" ", "¹", "²", "³", "⁴", "⁵", "⁶", "⁷", "⁸", "⁹"};

std::string to_minutes_str(float const Minutes, bool const Leadingzero, int const Width)
{

	float minutesintegral;
	auto const minutesfractional{std::modf(Minutes, &minutesintegral)};
	auto const width{Width - 1};
	auto minutes = (std::string(width - 1, ' ') + (Leadingzero ? std::to_string(100 + minutesintegral).substr(1, 2) : to_string(minutesintegral, 0)));
	return (minutes.substr(minutes.size() - width, width) + fractionlabels[static_cast<int>(std::floor(minutesfractional * 10 + 0.1))]);
}

int stol_def(const std::string &str, const int &DefaultValue)
{

	int result{DefaultValue};
	std::stringstream converter;
	converter << str;
	converter >> result;
	return result;
}

std::string ToLower(std::string const &text)
{

	auto lowercase{text};
	std::transform(std::begin(text), std::end(text), std::begin(lowercase), [](unsigned char c) { return std::tolower(c); });
	return lowercase;
}

std::string ToUpper(std::string const &text)
{

	auto uppercase{text};
	std::transform(std::begin(text), std::end(text), std::begin(uppercase), [](unsigned char c) { return std::toupper(c); });
	return uppercase;
}

// replaces polish letters with basic ascii
void win1250_to_ascii(std::string &Input)
{

	std::unordered_map<char, char> const charmap{{165, 'A'}, {198, 'C'}, {202, 'E'}, {163, 'L'}, {209, 'N'}, {211, 'O'}, {140, 'S'}, {143, 'Z'}, {175, 'Z'},
	                                             {185, 'a'}, {230, 'c'}, {234, 'e'}, {179, 'l'}, {241, 'n'}, {243, 'o'}, {156, 's'}, {159, 'z'}, {191, 'z'}};
	std::unordered_map<char, char>::const_iterator lookup;
	for (auto &input : Input)
	{
		if ((lookup = charmap.find(input)) != charmap.end())
			input = lookup->second;
	}
}

std::string win1250_to_utf8(const std::string &Input)
{
	std::unordered_map<char, std::string> const charmap{{165, "Ą"}, {198, "Ć"}, {202, "Ę"}, {163, "Ł"}, {209, "Ń"}, {211, "Ó"}, {140, "Ś"}, {143, "Ź"}, {175, "Ż"},
	                                                    {185, "ą"}, {230, "ć"}, {234, "ę"}, {179, "ł"}, {241, "ń"}, {243, "ó"}, {156, "ś"}, {159, "ź"}, {191, "ż"}};
	std::string output;
	std::unordered_map<char, std::string>::const_iterator lookup;
	for (auto &input : Input)
	{
		if ((lookup = charmap.find(input)) != charmap.end())
			output += lookup->second;
		else
			output += input;
	}
	return output;
}

// Ra: tymczasowe rozwiązanie kwestii zagranicznych (czeskich) napisów
char charsetconversiontable[] = "E?,?\"_++?%S<STZZ?`'\"\".--??s>stzz"
                                " ^^L$A|S^CS<--RZo±,l'uP.,as>L\"lz"
                                "RAAAALCCCEEEEIIDDNNOOOOxRUUUUYTB"
                                "raaaalccceeeeiiddnnoooo-ruuuuyt?";

// wycięcie liter z ogonkami
std::string Bezogonkow(std::string Input, bool const Underscorestospaces)
{

	char const extendedcharsetbit{static_cast<char>(0x80)};
	char const space{' '};
	char const underscore{'_'};

	for (auto &input : Input)
	{
		if (input & extendedcharsetbit)
		{
			input = charsetconversiontable[input ^ extendedcharsetbit];
		}
		else if (input < space)
		{
			input = space;
		}
		else if (Underscorestospaces && (input == underscore))
		{
			input = space;
		}
	}

	return Input;
}

template <> bool extract_value(bool &Variable, std::string const &Key, std::string const &Input, std::string const &Default)
{

	auto value = extract_value(Key, Input);
	if (false == value.empty())
	{
		// set the specified variable to retrieved value
		Variable = (ToLower(value) == "yes");
		return true; // located the variable
	}
	else
	{
		// set the variable to provided default value
		if (false == Default.empty())
		{
			Variable = (ToLower(Default) == "yes");
		}
		return false; // couldn't locate the variable in provided input
	}
}

bool FileExists(std::string const &Filename)
{
	return std::filesystem::exists(Filename);
}

std::pair<std::string, std::string> FileExists(std::vector<std::string> const &Names, std::vector<std::string> const &Extensions)
{

	for (auto const &name : Names)
	{
		for (auto const &extension : Extensions)
		{
			if (FileExists(name + extension))
			{
				return {name, extension};
			}
		}
	}
	// nothing found
	return {{}, {}};
}

// returns time of last modification for specified file
std::time_t last_modified(std::string const &Filename)
{
	std::string fn = Filename;
	struct stat filestat;
	if (::stat(fn.c_str(), &filestat) == 0)
		return filestat.st_mtime;
	else
		return 0;
}

// potentially erases file extension from provided file name. returns: true if extension was removed, false otherwise
bool erase_extension(std::string &Filename)
{

	auto const extensionpos{Filename.rfind('.')};

	if (extensionpos == std::string::npos)
	{
		return false;
	}

	if (extensionpos != Filename.rfind("..") + 1)
	{
		// we can get extension for .mat or, in legacy files, some image format. just trim it and set it to material file extension
		Filename.erase(extensionpos);
		return true;
	}
	return false;
}

void erase_leading_slashes(std::string &Filename)
{
	auto pos = Filename.find_first_not_of('/');
	Filename.erase(0, pos);
}

// potentially replaces backward slashes in provided file path with unix-compatible forward slashes
void replace_slashes(std::string &Filename)
{
	std::ranges::replace(Filename, '\\', '/');
}

// returns potential path part from provided file name
std::string_view substr_path(std::string const &Filename)
{
	if (auto pos = Filename.rfind('/'); pos != std::string::npos)
		return Filename.substr(0, pos + 1);
	return {};
}

// returns length of common prefix between two provided strings
std::ptrdiff_t len_common_prefix(std::string_view a, std::string_view b)
{
	auto [it1, it2] = std::ranges::mismatch(a, b);
	return std::distance(a.begin(), it1);
}

// returns true if provided string contains another provided string
bool contains(std::string_view const String, std::string_view Substring)
{
	// To be replaced with string::contains in C++ 23
	return (String.find(Substring) != std::string::npos);
}

bool contains(std::string_view const String, char Character)
{
	// To be replaced with string::contains in C++ 23
	return (String.find(Character) != std::string::npos);
}

// helper, restores content of a 3d vector from provided input stream
// TODO: review and clean up the helper routines, there's likely some redundant ones

glm::dvec3 LoadPoint(cParser &Input)
{
	// pobranie współrzędnych punktu
	glm::dvec3 point;
	Input.getTokens(3);
	Input >> point.x >> point.y >> point.z;
	return point;
}

// extracts a group of tokens from provided data stream, returns one of them picked randomly
std::string deserialize_random_set(cParser &Input, char const *Break)
{

	auto token{Input.getToken<std::string>(true, Break)};
	std::replace(token.begin(), token.end(), '\\', '/');
	if (token != "[")
	{
		// simple case, single token
		return token;
	}
	// if instead of a single token we've encountered '[' this marks a beginning of a random set
	// we retrieve all entries, then return a random one
	std::vector<std::string> tokens;
	while (((token = deserialize_random_set(Input, Break)) != "") && (token != "]"))
	{
		tokens.emplace_back(token);
	}
	if (false == tokens.empty())
	{
		std::shuffle(std::begin(tokens), std::end(tokens), Global.random_engine);
		return tokens.front();
	}
	else
	{
		// shouldn't ever get here but, eh
		return "";
	}
}