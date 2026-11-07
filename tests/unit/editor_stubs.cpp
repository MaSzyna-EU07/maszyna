// stand-ins for the parts of the engine which the code of the editor under test reaches for: the translations give the English
// messages back as they are

#include "stdafx.h"
#include "utilities/translation.h"

locale Translations; // NOSONAR: the object translation.h declares extern, and its lookups aren't const

const char *locale::lookup_c(const char *msg, bool) { return msg; } // NOSONAR: as translation.h declares it, the engine's lookup fills a cache

const std::string &locale::lookup_s(const std::string &msg, bool) { return msg; } // NOSONAR: as translation.h declares it, the engine's lookup fills a cache

const char *locale::lookup_n(const char *singular, const char *plural, long long n) { return n == 1 ? singular : plural; } // NOSONAR: as translation.h declares it, the engine's lookup fills a cache
