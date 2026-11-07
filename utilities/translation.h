/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

class locale {
public:
	void init();
	std::string label_cab_control(const std::string &Label);
	const std::string &coupling_name(int c);

	const char* lookup_c(const char *msg, bool constant = false);
	const std::string& lookup_s(const std::string &msg, bool constant = false);
	const char* lookup_n(const char *singular, const char *plural, long long n);

private:
	bool parse_translation(std::istream &stream);
	std::string parse_c_literal(const std::string &str);
	std::size_t plural_form(long long n) const;

	// the hash of a key text, taking a string_view as well as a string
	struct text_hash {
		using is_transparent = void;
		std::size_t operator()(std::string_view const Text) const noexcept { return std::hash<std::string_view>{}(Text); }
	};

	std::unordered_map<std::string, std::string> lang_mapping;
	std::unordered_map<const void*, const void*> pointer_cache;
	std::unordered_map<std::string, std::vector<std::string>, text_hash, std::equal_to<>> plural_mapping;
	std::unordered_map<const void*, const std::vector<std::string>*> plural_cache;
};

extern locale Translations;

#define STR(x) Translations.lookup_s(x, false)
#define STR_C(x) Translations.lookup_c(x, true)
#define STRN(x) x
#define STR_P(singular, plural, n) Translations.lookup_n(singular, plural, n)

//find . -name "*.cpp" | xargs xgettext --from-code="UTF-8" -k --keyword=STR --keyword=STR_C --keyword=STRN --keyword=STR_P:1,2
