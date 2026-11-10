/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scripting/scriptscreenkernels.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <utility>
#include <vector>

namespace script_screens
{

namespace
{

constexpr int glow_radius{6};
constexpr double glow_factor{0.08};

using light = std::array<std::int64_t, 3>;

// led_util's get_px_str: the light gathered from the diodes around, weighted 1 / (0.08 + distance squared), the sums
// taken in the same order as there to get the very same numbers
light glow_at(std::span<std::uint8_t const> const Source, int const Width, int const Height, int const X, int const Y)
{
	std::array<double, 3> sum{};
	auto const left{std::max(X - glow_radius, 0)};
	auto const right{std::min(X + glow_radius + 1, Width)};
	auto const top{std::max(Y - glow_radius, 0)};
	auto const bottom{std::min(Y + glow_radius + 1, Height)};
	for (auto i = left; i < right; ++i)
	{
		auto const m{glow_factor + static_cast<double>((i - X) * (i - X))};
		if (m > 20.0)
		{
			continue;
		}
		for (auto j = top; j < bottom; ++j)
		{
			auto const s{m + static_cast<double>((j - Y) * (j - Y))};
			if (s > 20.0)
			{
				continue;
			}
			auto const *pixel{&Source[(static_cast<std::size_t>(j) * Width + i) * 4]};
			for (std::size_t channel = 0; channel < 3; ++channel)
			{
				sum[channel] += static_cast<double>(pixel[channel]) / s;
			}
		}
	}
	return {static_cast<std::int64_t>(sum[0] * glow_factor), static_cast<std::int64_t>(sum[1] * glow_factor), static_cast<std::int64_t>(sum[2] * glow_factor)};
}

double sinc(double X)
{
	if (X == 0.0)
	{
		return 1.0;
	}
	X *= std::numbers::pi;
	return std::sin(X) / X;
}

double lanczos(double const X)
{
	if (X >= -3.0 && X < 3.0)
	{
		return sinc(X) * sinc(X / 3.0);
	}
	return 0.0;
}

constexpr int precision_bits{32 - 8 - 2};

// the fixed-point weights of one pass, as Pillow's Resample.c (precompute_coeffs, normalize_coeffs_8bpc) makes them
struct coefficients
{
	int size{0};
	std::vector<std::pair<int, int>> bounds; // first input sample and the number of samples, per output sample
	std::vector<std::int64_t> weights;

	coefficients(int const Input, int const Output)
	{
		auto const scale{static_cast<double>(Input) / static_cast<double>(Output)};
		auto const filterscale{std::max(scale, 1.0)};
		auto const support{3.0 * filterscale};
		size = static_cast<int>(std::ceil(support)) * 2 + 1;
		bounds.reserve(Output);
		weights.assign(static_cast<std::size_t>(Output) * size, 0);
		std::vector<double> prepared(size);
		for (auto xx = 0; xx < Output; ++xx)
		{
			auto const center{(xx + 0.5) * scale};
			auto const ss{1.0 / filterscale};
			auto const xmin{std::max(static_cast<int>(center - support + 0.5), 0)};
			auto const xmax{std::min(static_cast<int>(center + support + 0.5), Input) - xmin};
			auto total{0.0};
			for (auto x = 0; x < xmax; ++x)
			{
				prepared[x] = lanczos((x + xmin - center + 0.5) * ss);
				total += prepared[x];
			}
			for (auto x = 0; x < xmax; ++x)
			{
				if (total != 0.0)
				{
					prepared[x] /= total;
				}
				auto const fixed{prepared[x] * static_cast<double>(1 << precision_bits)};
				weights[static_cast<std::size_t>(xx) * size + x] = static_cast<std::int64_t>(prepared[x] < 0.0 ? -0.5 + fixed : 0.5 + fixed);
			}
			bounds.emplace_back(xmin, xmax);
		}
	}

	static std::uint8_t clip(std::int64_t const Sum)
	{
		if (Sum >= (std::int64_t{1} << (precision_bits + 8)))
		{
			return 255;
		}
		if (Sum <= 0)
		{
			return 0;
		}
		return static_cast<std::uint8_t>(Sum >> precision_bits);
	}
};

constexpr std::int64_t rounding{std::int64_t{1} << (precision_bits - 1)};

// one pass of the resampling, along rows (Step 4: neighbouring pixels) or along columns (Step: a row)
struct pass
{
	coefficients const &weights;
	std::size_t step; // bytes between the samples the pass mixes
};

std::uint8_t mix(std::uint8_t const *First, pass const &Pass, std::size_t const Index)
{
	auto const count{Pass.weights.bounds[Index].second};
	auto const offset{Index * static_cast<std::size_t>(Pass.weights.size)};
	auto sum{rounding};
	for (auto sample = 0; sample < count; ++sample)
	{
		sum += static_cast<std::int64_t>(First[sample * Pass.step]) * Pass.weights.weights[offset + sample];
	}
	return coefficients::clip(sum);
}

} // namespace

bool led_post(std::span<std::uint8_t const> const Source, int const Width, int const Height, std::span<std::uint8_t> const Target, bool const Bloom)
{
	auto const targetwidth{Width * 2 + 2};
	auto const targetheight{Height * 2 + 2};
	if (Width <= 0 || Height <= 0 || Source.size() < image_size(Width, Height) || Target.size() < image_size(targetwidth, targetheight))
	{
		return false;
	}
	// the light of every diode; above 255 it is clamped only in the final pixels, as led_util (Pillow) does
	std::vector<light> diodes(static_cast<std::size_t>(Width) * Height);
	for (auto y = 0; y < Height; ++y)
	{
		for (auto x = 0; x < Width; ++x)
		{
			auto const at{(static_cast<std::size_t>(y) * Width + x) * 4};
			diodes[static_cast<std::size_t>(y) * Width + x] = Bloom ? glow_at(Source, Width, Height, x, y) : light{Source[at], Source[at + 1], Source[at + 2]};
		}
	}
	// a black frame, then the diodes
	for (std::size_t idx = 0; idx < image_size(targetwidth, targetheight); ++idx)
	{
		Target[idx] = idx % 4 == 3 ? 255 : 0;
	}
	for (auto i = 0; i < Width * 2; ++i)
	{
		for (auto j = 0; j < Height * 2; ++j)
		{
			auto const &diode{diodes[static_cast<std::size_t>(j / 2) * Width + i / 2]};
			auto const shade{1.0 - static_cast<double>(i % 2 + j % 2) * 0.15};
			auto *pixel{&Target[(static_cast<std::size_t>(j + 1) * targetwidth + (i + 1)) * 4]};
			for (std::size_t channel = 0; channel < 3; ++channel)
			{
				pixel[channel] = static_cast<std::uint8_t>(std::clamp<std::int64_t>(static_cast<std::int64_t>(static_cast<double>(diode[channel]) * shade), 0, 255));
			}
		}
	}
	return true;
}

bool resize_lanczos(std::span<std::uint8_t const> const Source, int const Width, int const Height, std::span<std::uint8_t> const Target, int const TargetWidth,
                    int const TargetHeight)
{
	if (Width <= 0 || Height <= 0 || TargetWidth <= 0 || TargetHeight <= 0 || Source.size() < image_size(Width, Height) ||
	    Target.size() < image_size(TargetWidth, TargetHeight))
	{
		return false;
	}
	coefficients const horizontal{Width, TargetWidth};
	coefficients const vertical{Height, TargetHeight};

	std::vector<std::uint8_t> temporary(image_size(TargetWidth, Height));
	pass const along_row{horizontal, 4};
	for (auto y = 0; y < Height; ++y)
	{
		for (auto x = 0; x < TargetWidth; ++x)
		{
			auto const *first{&Source[(static_cast<std::size_t>(y) * Width + horizontal.bounds[x].first) * 4]};
			for (std::size_t channel = 0; channel < 3; ++channel)
			{
				temporary[(static_cast<std::size_t>(y) * TargetWidth + x) * 4 + channel] = mix(first + channel, along_row, x);
			}
		}
	}
	pass const along_column{vertical, static_cast<std::size_t>(TargetWidth) * 4};
	for (auto y = 0; y < TargetHeight; ++y)
	{
		for (auto x = 0; x < TargetWidth; ++x)
		{
			auto const *first{&temporary[(static_cast<std::size_t>(vertical.bounds[y].first) * TargetWidth + x) * 4]};
			auto *pixel{&Target[(static_cast<std::size_t>(y) * TargetWidth + x) * 4]};
			for (std::size_t channel = 0; channel < 3; ++channel)
			{
				pixel[channel] = mix(first + channel, along_column, y);
			}
			pixel[3] = 255;
		}
	}
	return true;
}

int width_after(post_step const &Step, int const Width)
{
	return Step.type == post_step::kind::led ? Width * 2 + 2 : Step.width;
}

int height_after(post_step const &Step, int const Height)
{
	return Step.type == post_step::kind::led ? Height * 2 + 2 : Step.height;
}

bool apply_post(post_image &Image, std::span<post_step const> const Steps)
{
	if (Steps.empty())
	{
		return true;
	}
	auto result{Image};
	for (auto const &step : Steps)
	{
		auto const width{width_after(step, result.width)};
		auto const height{height_after(step, result.height)};
		if (width <= 0 || height <= 0)
		{
			return false;
		}
		std::vector<std::uint8_t> pixels(image_size(width, height));
		auto const done{step.type == post_step::kind::led ? led_post(result.pixels, result.width, result.height, pixels, step.bloom)
		                                                  : resize_lanczos(result.pixels, result.width, result.height, pixels, width, height)};
		if (false == done)
		{
			return false;
		}
		result = post_image{std::move(pixels), width, height};
	}
	Image = std::move(result);
	return true;
}

} // namespace script_screens
