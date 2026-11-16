/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/loadingscreen.h"

#ifdef WITH_RMLUI

#include <algorithm>
#include <cmath>
#include <format>

#include <RmlUi/Core.h>

#include "application/rmluidocument.h"
#include "application/rmluilayer.h"

namespace
{

// staticFiles/ui/loading.rml of the sources, written out by CMake as the bytes of loading_rml
#include "loading_rml.inc"

// the values of the data model of the document, see staticFiles/ui/loading.rml
struct loading_values
{
	int percent{0};
	Rml::String done; // the width of the bar, n%
	Rml::String subtask;
	Rml::String text;
	Rml::String title;
	Rml::String trivia;
};

Rml::String share(float const Share)
{
	return std::format("{:.1f}%", std::clamp(Share, 0.f, 1.f) * 100.f);
}

} // namespace

struct loading_screen::state
{
	rmlui::bound_values<loading_values> values;
	rmlui::document document;
};

loading_screen::loading_screen() = default;

loading_screen::~loading_screen() = default;

bool loading_screen::open()
{
	auto opened{std::make_unique<state>()};
	auto const bound = [values = &opened->values](Rml::DataModelConstructor &Constructor) {
		values->bind(Constructor, "percent", &loading_values::percent);
		values->bind(Constructor, "done", &loading_values::done);
		values->bind(Constructor, "subtask", &loading_values::subtask);
		values->bind(Constructor, "text", &loading_values::text);
		values->bind(Constructor, "title", &loading_values::title);
		values->bind(Constructor, "trivia", &loading_values::trivia);
	};
	if (false == opened->document.open("loading_screen", "ui/loading.rml", loading_rml, bound))
		return false;
	m_state = std::move(opened);
	return true;
}

bool loading_screen::update(progress const &Progress)
{
	if (m_failed || false == rmlui::active())
		return false;
	if (m_state == nullptr && false == open())
	{
		m_failed = true;
		return false;
	}
	loading_values values;
	values.percent = static_cast<int>(std::lround(std::clamp(Progress.done, 0.f, 1.f) * 100.f));
	values.done = share(Progress.done);
	values.subtask = share(Progress.subtask);
	values.text = Progress.text;
	values.title = Progress.title;
	values.trivia = Progress.trivia;
	m_state->values.update(values, m_state->document);
	m_state->document.show_in_frame();
	return true;
}

#else

struct loading_screen::state
{
};

loading_screen::loading_screen() = default;

loading_screen::~loading_screen() = default;

bool loading_screen::open()
{
	return false;
}

bool loading_screen::update(progress const &)
{
	return false;
}

#endif
