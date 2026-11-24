/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorClickHeights.hpp"

#include <algorithm>
#include <cmath>

namespace click_heights
{

std::size_t settle(std::vector<glm::dvec3> &Points, std::vector<source> &Sources, std::function<double(std::size_t)> const &Ground)
{
	std::size_t changed{0};
	auto const count{std::min(Points.size(), Sources.size())};
	for (std::size_t i = 0; i < count; ++i)
	{
		auto &point{Points[i]};
		switch (Sources[i])
		{
		case source::none:
		{
			auto const height{Ground(i)};
			if (false == std::isfinite(height))
				break;
			if (point.y != height)
			{
				point.y = height;
				++changed;
			}
			Sources[i] = source::ground;
			break;
		}
		case source::level:
			if (i > 0 && point.y != Points[i - 1].y)
			{
				point.y = Points[i - 1].y;
				++changed;
			}
			break;
		default:
			break;
		}
	}
	return changed;
}

bool guessed(std::vector<source> const &Sources)
{
	return std::find(Sources.begin(), Sources.end(), source::none) != Sources.end();
}

std::size_t guessed_count(std::vector<source> const &Sources)
{
	return static_cast<std::size_t>(std::count(Sources.begin(), Sources.end(), source::none));
}

bool waiting(std::vector<source> const &Sources, std::function<bool(std::size_t)> const &Coming)
{
	for (std::size_t i = 0; i < Sources.size(); ++i)
		if (Sources[i] == source::none && Coming(i))
			return true;
	return false;
}

bool wait_on(std::vector<source> const &Sources, std::function<bool(std::size_t)> const &Coming, double const Waited)
{
	return Waited < wait_longest && waiting(Sources, Coming);
}

editorSettings::lay_heights heights_for(editorSettings::lay_heights const Setting, bool const Traced, bool const Chosen)
{
	return Traced && false == Chosen ? editorSettings::lay_heights::fitted : Setting;
}

} // namespace click_heights
