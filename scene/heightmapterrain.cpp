/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/heightmapterrain.h"

#include "scene/sceneterrain.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>

#include <yaml-cpp/yaml.h>
#include "zstd.h"

namespace heightmap
{

namespace
{

std::uint32_t constexpr fourcc(char const A, char const B, char const C, char const D)
{
	return static_cast<std::uint32_t>(static_cast<std::uint8_t>(A)) | (static_cast<std::uint32_t>(static_cast<std::uint8_t>(B)) << 8) |
	       (static_cast<std::uint32_t>(static_cast<std::uint8_t>(C)) << 16) | (static_cast<std::uint32_t>(static_cast<std::uint8_t>(D)) << 24);
}

std::uint32_t constexpr tag_chunk{fourcc('C', 'H', 'N', 'K')};
std::uint32_t constexpr tag_heights{fourcc('H', 'G', 'H', 'T')};
std::uint32_t constexpr tag_base{fourcc('H', 'B', 'A', 'S')};
std::uint32_t constexpr tag_paint{fourcc('S', 'P', 'L', 'T')};
std::uint32_t constexpr tag_adjust{fourcc('H', 'A', 'D', 'J')};

std::uint32_t constexpr pack_magic{fourcc('T', 'C', 'H', '1')};
std::uint32_t constexpr pack_version{1};
std::size_t constexpr header_size{64};
std::size_t constexpr index_entry_size{16};
std::size_t constexpr slice_entry_size{32};
std::uint32_t constexpr codec_stored{0};
std::uint32_t constexpr codec_zstd{1};
int constexpr zstd_level{3}; // light and fast, the data is simple

// the slices are small, a slice larger than this is taken for a damaged file
std::uint32_t constexpr max_slice_size{256u * 1024u * 1024u};

template <typename Type_> void put(std::vector<std::uint8_t> &Out, Type_ const Value)
{
	auto const offset{Out.size()};
	Out.resize(offset + sizeof(Type_));
	std::memcpy(Out.data() + offset, &Value, sizeof(Type_));
}

template <typename Type_> void put_at(std::vector<std::uint8_t> &Out, std::size_t const Offset, Type_ const Value)
{
	std::memcpy(Out.data() + Offset, &Value, sizeof(Type_));
}

// reads values from a block of data, without going past its end
class reader
{
  public:
	reader(std::uint8_t const *Data, std::size_t const Size) : m_data(Data), m_size(Size) {}
	template <typename Type_> bool get(Type_ &Value)
	{
		if (m_offset + sizeof(Type_) > m_size)
			return false;
		std::memcpy(&Value, m_data + m_offset, sizeof(Type_));
		m_offset += sizeof(Type_);
		return true;
	}
	bool get(void *Out, std::size_t const Size)
	{
		if (m_offset + Size > m_size)
			return false;
		std::memcpy(Out, m_data + m_offset, Size);
		m_offset += Size;
		return true;
	}
	std::size_t left() const { return m_size - m_offset; }

  private:
	std::uint8_t const *m_data;
	std::size_t m_size;
	std::size_t m_offset{0};
};

void encode_heights(std::vector<float> const &Heights, std::vector<std::uint8_t> &Out)
{
	if (Heights.empty())
		return;
	auto const [low, high]{std::minmax_element(Heights.begin(), Heights.end())};
	auto const base{static_cast<std::int64_t>(std::floor(*low / height_step))};
	auto const top{static_cast<std::int64_t>(std::ceil(*high / height_step))};
	if (top - base > 0xffff || base < std::numeric_limits<std::int32_t>::min() || top > std::numeric_limits<std::int32_t>::max())
	{
		// too high a range for the steps: kept as they are
		put<std::uint8_t>(Out, 1);
		for (auto const height : Heights)
			put<float>(Out, height);
		return;
	}
	put<std::uint8_t>(Out, 0);
	put<std::int32_t>(Out, static_cast<std::int32_t>(base));
	// differences of the neighbouring points are small and repeat a lot, which compresses well
	std::uint16_t previous{0};
	for (auto const height : Heights)
	{
		auto const steps{static_cast<std::uint16_t>(std::clamp<std::int64_t>(std::llround(height / height_step) - base, 0, 0xffff))};
		put<std::uint16_t>(Out, static_cast<std::uint16_t>(steps - previous));
		previous = steps;
	}
}

bool decode_heights(std::vector<std::uint8_t> const &Data, std::size_t const Count, std::vector<float> &Out)
{
	reader input{Data.data(), Data.size()};
	std::uint8_t encoding{0};
	if (false == input.get(encoding))
		return false;
	Out.resize(Count);
	if (encoding == 1)
	{
		if (input.left() != Count * sizeof(float))
			return false;
		return input.get(Out.data(), Count * sizeof(float));
	}
	if (encoding != 0)
		return false;
	std::int32_t base{0};
	if (false == input.get(base) || input.left() != Count * sizeof(std::uint16_t))
		return false;
	std::uint16_t value{0};
	for (std::size_t i = 0; i < Count; ++i)
	{
		std::uint16_t delta{0};
		input.get(delta);
		value = static_cast<std::uint16_t>(value + delta);
		Out[i] = static_cast<float>((static_cast<std::int64_t>(base) + value) * height_step);
	}
	return true;
}

std::uint32_t checksum(std::vector<std::uint8_t> const &Data)
{
	return scene::terrain_file::checksum(reinterpret_cast<char const *>(Data.data()), Data.size());
}

int floor_div(int const Value, int const Divisor)
{
	return (Value >= 0 ? Value / Divisor : -((-Value + Divisor - 1) / Divisor));
}

// the larger of two values, the corner between them rounded off over K
double soft_max(double const A, double const B, double const K)
{
	if (K <= 1e-6)
		return std::max(A, B);
	auto const h{std::max(K - std::abs(A - B), 0.0) / K};
	return std::max(A, B) + h * h * K * 0.25;
}

} // namespace

int cells_for(float const Spacing)
{
	return static_cast<int>(std::lround(chunk_size / valid_spacing(Spacing)));
}

float valid_spacing(float const Spacing)
{
	float best{default_spacing};
	float distance{std::numeric_limits<float>::max()};
	for (auto const spacing : spacings)
		if (std::abs(spacing - Spacing) < distance)
		{
			distance = std::abs(spacing - Spacing);
			best = spacing;
		}
	return best;
}

chunk_key chunk_at(double const X, double const Z)
{
	return {static_cast<int>(std::floor(X / chunk_size)), static_cast<int>(std::floor(Z / chunk_size))};
}

glm::dvec2 chunk_corner(chunk_key const &Key)
{
	return {Key.first * chunk_size, Key.second * chunk_size};
}

pack_key pack_of(chunk_key const &Key)
{
	return {floor_div(Key.first, pack_side), floor_div(Key.second, pack_side)};
}

glm::dvec2 pack_corner(pack_key const &Key)
{
	return {Key.first * pack_side * chunk_size, Key.second * pack_side * chunk_size};
}

std::string pack_file_name(pack_key const &Key)
{
	return std::to_string(Key.first) + "_" + std::to_string(Key.second) + ".tch";
}

// chunk_data

std::size_t chunk_data::layer_count() const
{
	std::size_t count{0};
	while (count < layers.size() && layers[count] != no_layer)
		++count;
	return count;
}

bool chunk_data::contains(double const X, double const Z) const
{
	auto const low{corner()};
	return X >= low.x && X <= low.x + chunk_size && Z >= low.y && Z <= low.y + chunk_size;
}

double chunk_data::height_at(double const X, double const Z) const
{
	return height_in(heights, X, Z);
}

double chunk_data::height_in(std::vector<float> const &Heights, double const X, double const Z) const
{
	if (cells <= 0 || Heights.size() != static_cast<std::size_t>(side()) * side())
		return 0.0;
	auto const low{corner()};
	auto const fx{(X - low.x) / spacing()};
	auto const fz{(Z - low.y) / spacing()};
	auto const ix{std::clamp(static_cast<int>(std::floor(fx)), 0, cells - 1)};
	auto const iz{std::clamp(static_cast<int>(std::floor(fz)), 0, cells - 1)};
	auto const tx{std::clamp(fx - ix, 0.0, 1.0)};
	auto const tz{std::clamp(fz - iz, 0.0, 1.0)};
	auto const at = [&](int const X_, int const Z_) { return static_cast<double>(Heights[static_cast<std::size_t>(Z_) * side() + X_]); };
	auto const h00{at(ix, iz)}, h10{at(ix + 1, iz)}, h01{at(ix, iz + 1)}, h11{at(ix + 1, iz + 1)};
	// the quads are cut along the diagonal from (x0, z1) to (x1, z0), as the mesh is made
	if (tx + tz <= 1.0)
		return h00 + tx * (h10 - h00) + tz * (h01 - h00);
	return h11 + (1.0 - tx) * (h01 - h11) + (1.0 - tz) * (h10 - h11);
}

std::shared_ptr<chunk_data> chunk_data::make_flat(chunk_key const &Key, float const Spacing, float const Height)
{
	auto chunk{std::make_shared<chunk_data>()};
	chunk->key = Key;
	chunk->cells = cells_for(Spacing);
	chunk->heights.assign(static_cast<std::size_t>(chunk->side()) * chunk->side(), Height);
	chunk->layers[0] = 0;
	return chunk;
}

std::shared_ptr<chunk_data> chunk_data::resampled(float const Spacing, int const Paint) const
{
	auto chunk{std::make_shared<chunk_data>(*this)};
	chunk->cells = cells_for(Spacing);
	if (chunk->cells != cells)
	{
		auto const low{corner()};
		auto const step{chunk->spacing()};
		// a coarser grid would otherwise pick single points of the finer one: each point inside the chunk takes the average of
		// the ground around it (a tent of the size of its cell). the points of the edges are the ground itself, as they're
		// shared with the neighbours, which keep theirs
		auto const reach{chunk->cells < cells ? step * 0.5 : 0.0};
		auto const taps{reach > 0.0 ? std::max(1, static_cast<int>(std::ceil(reach / spacing()))) : 0};
		auto const resample = [&](std::vector<float> const &Source, std::vector<float> &Target) {
			Target.resize(static_cast<std::size_t>(chunk->side()) * chunk->side());
			for (int iz = 0; iz <= chunk->cells; ++iz)
				for (int ix = 0; ix <= chunk->cells; ++ix)
				{
					auto const x{low.x + ix * step}, z{low.y + iz * step};
					auto value{height_in(Source, x, z)};
					if (taps > 0 && ix > 0 && iz > 0 && ix < chunk->cells && iz < chunk->cells)
					{
						double sum{0.0}, weights{0.0};
						for (int dz = -taps; dz <= taps; ++dz)
							for (int dx = -taps; dx <= taps; ++dx)
							{
								auto const weight{static_cast<double>((taps + 1 - std::abs(dx)) * (taps + 1 - std::abs(dz)))};
								sum += weight * height_in(Source, x + dx * reach / taps, z + dz * reach / taps);
								weights += weight;
							}
						value = sum / weights;
					}
					Target[static_cast<std::size_t>(iz) * chunk->side() + ix] = static_cast<float>(value);
				}
		};
		resample(heights, chunk->heights);
		if (false == base.empty())
			resample(base, chunk->base);
		if (false == adjust.empty())
			resample(adjust, chunk->adjust);
	}
	if (Paint > 0 && paint > 0 && Paint != paint)
	{
		// the weights of the samples between the old ones blended from them, and brought back to 255 a sample
		auto const count{layer_count()};
		auto const oldside{static_cast<std::size_t>(paint + 1)};
		auto const newside{static_cast<std::size_t>(Paint + 1)};
		chunk->paint = Paint;
		chunk->weights.assign(newside * newside * count, 0);
		std::vector<double> values(count);
		for (std::size_t iz = 0; iz < newside; ++iz)
			for (std::size_t ix = 0; ix < newside; ++ix)
			{
				auto const fx{static_cast<double>(ix) * paint / Paint}, fz{static_cast<double>(iz) * paint / Paint};
				auto const x0{std::min(static_cast<std::size_t>(fx), oldside - 2)}, z0{std::min(static_cast<std::size_t>(fz), oldside - 2)};
				auto const tx{fx - x0}, tz{fz - z0};
				double total{0.0};
				for (std::size_t slot = 0; slot < count; ++slot)
				{
					auto const at = [&](std::size_t const X, std::size_t const Z) { return static_cast<double>(weights[slot * oldside * oldside + Z * oldside + X]); };
					values[slot] = (at(x0, z0) * (1.0 - tx) + at(x0 + 1, z0) * tx) * (1.0 - tz) + (at(x0, z0 + 1) * (1.0 - tx) + at(x0 + 1, z0 + 1) * tx) * tz;
					total += values[slot];
				}
				// rounded down, what's left of 255 goes to the heaviest
				int sum{0};
				std::size_t heaviest{0};
				for (std::size_t slot = 0; slot < count; ++slot)
				{
					auto const weight{total > 0.0 ? static_cast<int>(values[slot] * 255.0 / total) : (slot == 0 ? 255 : 0)};
					chunk->weights[slot * newside * newside + iz * newside + ix] = static_cast<std::uint8_t>(weight);
					sum += weight;
					if (values[slot] > values[heaviest])
						heaviest = slot;
				}
				chunk->weights[heaviest * newside * newside + iz * newside + ix] += static_cast<std::uint8_t>(255 - sum);
			}
	}
	return chunk;
}

std::uint8_t chunk_data::weight(std::size_t const Slot, int const Ix, int const Iz) const
{
	if (paint <= 0)
		return Slot == 0 ? 255 : 0;
	auto const samples{static_cast<std::size_t>(paint + 1) * (paint + 1)};
	auto const index{Slot * samples + static_cast<std::size_t>(Iz) * (paint + 1) + Ix};
	return index < weights.size() ? weights[index] : 0;
}

int chunk_data::slot_of(std::uint16_t const Layer) const
{
	for (std::size_t slot = 0; slot < layers.size(); ++slot)
		if (layers[slot] == Layer)
			return static_cast<int>(slot);
	return -1;
}

std::size_t chunk_data::dominant_slot() const
{
	auto const count{layer_count()};
	if (paint <= 0 || count < 2)
		return 0;
	auto const samples{static_cast<std::size_t>(paint + 1) * (paint + 1)};
	std::size_t best{0};
	std::uint64_t bestsum{0};
	for (std::size_t slot = 0; slot < count; ++slot)
	{
		std::uint64_t sum{0};
		for (std::size_t i = 0; i < samples && slot * samples + i < weights.size(); ++i)
			sum += weights[slot * samples + i];
		if (sum > bestsum)
		{
			bestsum = sum;
			best = slot;
		}
	}
	return best;
}

void chunk_data::ensure_paint(int const Samples)
{
	if (paint > 0)
		return;
	if (layers[0] == no_layer)
		layers[0] = 0;
	paint = std::max(1, Samples);
	auto const samples{static_cast<std::size_t>(paint + 1) * (paint + 1)};
	weights.assign(samples * layer_count(), 0);
	std::fill(weights.begin(), weights.begin() + samples, 255);
}

void chunk_data::compact_layers()
{
	if (paint <= 0)
		return;
	auto const samples{static_cast<std::size_t>(paint + 1) * (paint + 1)};
	auto const count{layer_count()};
	std::vector<std::uint8_t> kept;
	std::array<std::uint16_t, max_chunk_layers> keptlayers;
	keptlayers.fill(no_layer);
	std::size_t used{0};
	for (std::size_t slot = 0; slot < count; ++slot)
	{
		auto const begin{weights.begin() + slot * samples};
		if (std::all_of(begin, begin + samples, [](std::uint8_t const Weight) { return Weight == 0; }))
			continue;
		kept.insert(kept.end(), begin, begin + samples);
		keptlayers[used++] = layers[slot];
	}
	if (used == 0)
	{
		// nothing painted at all; shouldn't happen, but the chunk can't be left without a material
		keptlayers[0] = layers[0] == no_layer ? 0 : layers[0];
		used = 1;
		kept.clear();
	}
	layers = keptlayers;
	if (used == 1)
	{
		paint = 0;
		weights.clear();
		return;
	}
	weights = std::move(kept);
}

// modifiers

std::pair<glm::dvec2, glm::dvec2> modifier::bounds() const
{
	glm::dvec2 low{std::numeric_limits<double>::max()}, high{-std::numeric_limits<double>::max()};
	double widest{0.0};
	for (auto const &point : points)
	{
		low = glm::min(low, point.position);
		high = glm::max(high, point.position);
		widest = std::max(widest, point.half_width);
	}
	if (points.empty())
		return {glm::dvec2{0.0}, glm::dvec2{0.0}};
	auto const margin{widest + reach};
	return {low - glm::dvec2{margin}, high + glm::dvec2{margin}};
}

namespace
{
std::uint64_t grid_key(int const X, int const Z)
{
	return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(X)) << 32) | static_cast<std::uint32_t>(Z);
}
} // namespace

modifier_shape::modifier_shape(modifier const &Modifier) : m_modifier(Modifier)
{
	for (auto const &point : Modifier.points)
		m_widest = std::max(m_widest, point.half_width);
	auto const bounds{Modifier.bounds()};
	m_min = bounds.first;
	m_max = bounds.second;
	auto const &points{Modifier.points};
	for (std::size_t i = 0; i + 1 < points.size(); ++i)
	{
		auto const &a{points[i].position};
		auto const &b{points[i + 1].position};
		auto const x0{static_cast<int>(std::floor(std::min(a.x, b.x) / m_cell))}, x1{static_cast<int>(std::floor(std::max(a.x, b.x) / m_cell))};
		auto const z0{static_cast<int>(std::floor(std::min(a.y, b.y) / m_cell))}, z1{static_cast<int>(std::floor(std::max(a.y, b.y) / m_cell))};
		for (auto x = x0; x <= x1; ++x)
			for (auto z = z0; z <= z1; ++z)
				m_grid.emplace_back(grid_key(x, z), static_cast<std::uint32_t>(i));
	}
	std::sort(m_grid.begin(), m_grid.end());
}

bool modifier_shape::reaches(glm::dvec2 const &Min, glm::dvec2 const &Max) const
{
	return false == m_grid.empty() && Max.x >= m_min.x && Min.x <= m_max.x && Max.y >= m_min.y && Min.y <= m_max.y;
}

bool modifier_shape::apply(double const X, double const Z, float &Height) const
{
	if (m_grid.empty() || X < m_min.x || X > m_max.x || Z < m_min.y || Z > m_max.y)
		return false;
	auto const &points{m_modifier.points};
	auto const reach{m_widest + m_modifier.reach};
	auto distance{reach};
	double formation{0.0}, halfwidth{0.0};
	bool found{false};
	auto const x0{static_cast<int>(std::floor((X - reach) / m_cell))}, x1{static_cast<int>(std::floor((X + reach) / m_cell))};
	auto const z0{static_cast<int>(std::floor((Z - reach) / m_cell))}, z1{static_cast<int>(std::floor((Z + reach) / m_cell))};
	glm::dvec2 const point{X, Z};
	for (auto x = x0; x <= x1; ++x)
		for (auto z = z0; z <= z1; ++z)
		{
			auto const key{grid_key(x, z)};
			auto it{std::lower_bound(m_grid.begin(), m_grid.end(), std::make_pair(key, std::uint32_t{0}))};
			for (; it != m_grid.end() && it->first == key; ++it)
			{
				auto const i{it->second};
				auto const &a{points[i].position};
				auto const &b{points[i + 1].position};
				auto const run{b - a};
				auto const length{glm::dot(run, run)};
				auto const t{length > 1e-12 ? std::clamp(glm::dot(point - a, run) / length, 0.0, 1.0) : 0.0};
				auto const offset{glm::length(point - (a + run * t))};
				if (offset < distance)
				{
					distance = offset;
					formation = points[i].formation + (points[i + 1].formation - points[i].formation) * t;
					halfwidth = points[i].half_width + (points[i + 1].half_width - points[i].half_width) * t;
					found = true;
				}
			}
		}
	if (false == found)
		return false;
	// the formation under the axis, slopes from its edges down or up to the ground, the bends rounded off
	auto const slope{std::max(0.1, m_modifier.slope)};
	auto const rounding{std::max(0.0, m_modifier.rounding)};
	auto const ground{static_cast<double>(Height)};
	auto const beyond{soft_max(0.0, distance - halfwidth, rounding)};
	auto const bend{rounding / slope};
	auto const target{ground > formation ? -soft_max(-ground, -(formation + beyond / slope), bend) : soft_max(ground, formation - beyond / slope, bend)};
	if (std::abs(target - ground) < 0.001)
		return false;
	Height = static_cast<float>(target);
	return true;
}

// manifest

bool load_manifest(std::string const &Path, manifest &Manifest, std::string *Error)
{
	try
	{
		auto const document{YAML::LoadFile(Path)};
		manifest result;
		result.layers.clear();
		if (auto const node{document["version"]})
			result.version = node.as<int>();
		if (auto const node{document["compress"]})
			result.compress = node.as<bool>();
		if (auto const node{document["spacing"]})
			result.spacing = valid_spacing(node.as<float>());
		if (auto const node{document["water_material"]})
			result.water_material = node.as<std::string>();
		if (auto const node{document["layers"]}; node && node.IsSequence())
			for (auto const &layer : node)
			{
				// a material name, or a map with the material and the size its textures repeat at
				if (layer.IsMap())
				{
					layer_def entry;
					if (auto const material{layer["material"]})
						entry.material = material.as<std::string>();
					if (auto const size{layer["size"]})
						entry.size = std::max(0.f, size.as<float>());
					if (false == entry.material.empty())
						result.layers.push_back(entry);
				}
				else
					result.layers.push_back({layer.as<std::string>(), 0.f});
			}
		if (result.layers.empty())
			result.layers.push_back({"grass", 0.f});
		if (auto const node{document["water"]}; node && node.IsSequence())
			for (auto const &entry : node)
			{
				water_body water;
				if (auto const name{entry["name"]})
					water.name = name.as<std::string>();
				if (auto const level{entry["level"]})
					water.level = level.as<double>();
				if (auto const material{entry["material"]})
					water.material = material.as<std::string>();
				if (auto const size{entry["size"]})
					water.size = std::max(0.f, size.as<float>());
				if (auto const outline{entry["outline"]}; outline && outline.IsSequence())
					for (auto const &point : outline)
						if (point.IsSequence() && point.size() >= 2)
							water.outline.emplace_back(point[0].as<double>(), point[1].as<double>());
				if (water.outline.size() >= 3)
					result.water.push_back(std::move(water));
			}
		if (auto const node{document["modifiers"]}; node && node.IsSequence())
			for (auto const &entry : node)
			{
				modifier item;
				if (auto const name{entry["name"]})
					item.name = name.as<std::string>();
				if (auto const slope{entry["slope"]})
					item.slope = slope.as<double>();
				if (auto const reach{entry["reach"]})
					item.reach = reach.as<double>();
				if (auto const rounding{entry["rounding"]})
					item.rounding = rounding.as<double>();
				if (auto const points{entry["points"]}; points && points.IsSequence())
					for (auto const &point : points)
						if (point.IsSequence() && point.size() >= 4)
							item.points.push_back({{point[0].as<double>(), point[1].as<double>()}, point[2].as<double>(), point[3].as<double>()});
				if (item.points.size() >= 2)
					result.modifiers.push_back(std::move(item));
			}
		Manifest = std::move(result);
		return true;
	}
	catch (YAML::Exception const &exception)
	{
		if (Error != nullptr)
			*Error = exception.what();
		return false;
	}
}

bool save_manifest(std::string const &Path, manifest const &Manifest, std::string *Error)
{
	YAML::Emitter out;
	out.SetDoublePrecision(12);
	out.SetFloatPrecision(7);
	out << YAML::Comment("MaSzyna heightmap terrain; the chunks are in the .tch files next to this one");
	out << YAML::BeginMap;
	out << YAML::Key << "version" << YAML::Value << Manifest.version;
	out << YAML::Key << "compress" << YAML::Value << Manifest.compress;
	out << YAML::Key << "spacing" << YAML::Value << Manifest.spacing;
	out << YAML::Key << "water_material" << YAML::Value << Manifest.water_material;
	out << YAML::Key << "layers" << YAML::Value << YAML::BeginSeq;
	for (auto const &layer : Manifest.layers)
	{
		if (layer.size > 0.f)
			out << YAML::Flow << YAML::BeginMap << YAML::Key << "material" << YAML::Value << layer.material << YAML::Key << "size" << YAML::Value << layer.size << YAML::EndMap;
		else
			out << layer.material;
	}
	out << YAML::EndSeq;
	if (false == Manifest.water.empty())
	{
		out << YAML::Key << "water" << YAML::Value << YAML::BeginSeq;
		for (auto const &water : Manifest.water)
		{
			out << YAML::BeginMap;
			out << YAML::Key << "name" << YAML::Value << water.name;
			out << YAML::Key << "level" << YAML::Value << water.level;
			if (false == water.material.empty())
				out << YAML::Key << "material" << YAML::Value << water.material;
			if (water.size > 0.f)
				out << YAML::Key << "size" << YAML::Value << water.size;
			out << YAML::Key << "outline" << YAML::Value << YAML::BeginSeq;
			for (auto const &point : water.outline)
				out << YAML::Flow << YAML::BeginSeq << point.x << point.y << YAML::EndSeq;
			out << YAML::EndSeq;
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;
	}
	if (false == Manifest.modifiers.empty())
	{
		out << YAML::Key << "modifiers" << YAML::Value << YAML::BeginSeq;
		for (auto const &item : Manifest.modifiers)
		{
			out << YAML::BeginMap;
			out << YAML::Key << "name" << YAML::Value << item.name;
			out << YAML::Key << "slope" << YAML::Value << item.slope;
			out << YAML::Key << "reach" << YAML::Value << item.reach;
			out << YAML::Key << "rounding" << YAML::Value << item.rounding;
			out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;
			for (auto const &point : item.points)
				out << YAML::Flow << YAML::BeginSeq << point.position.x << point.position.y << point.formation << point.half_width << YAML::EndSeq;
			out << YAML::EndSeq;
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;
	}
	out << YAML::EndMap;
	if (false == out.good())
	{
		if (Error != nullptr)
			*Error = out.GetLastError();
		return false;
	}
	std::error_code ec;
	std::filesystem::create_directories(std::filesystem::path(Path).parent_path(), ec);
	std::ofstream file{Path, std::ios::binary | std::ios::trunc};
	file << out.c_str() << '\n';
	if (false == file.good())
	{
		if (Error != nullptr)
			*Error = "can't write " + Path;
		return false;
	}
	return true;
}

// pack files

bool pack_file::open(std::string const &Path, std::string *Error)
{
	auto const fail = [&](std::string const &Message) {
		if (Error != nullptr)
			*Error = Path + ": " + Message;
		return false;
	};
	m_path = Path;
	for (auto &slices : m_slices)
		slices.clear();
	std::ifstream file{Path, std::ios::binary};
	if (false == file.good())
		return fail("can't open the file");
	std::vector<std::uint8_t> header(header_size);
	file.read(reinterpret_cast<char *>(header.data()), header.size());
	if (file.gcount() != static_cast<std::streamsize>(header.size()))
		return fail("the file is too short");
	reader input{header.data(), header.size()};
	std::uint32_t magic{0}, version{0}, side{0};
	std::int32_t px{0}, pz{0};
	float size{0.f};
	std::uint64_t indexoffset{0};
	input.get(magic);
	input.get(version);
	input.get(px);
	input.get(pz);
	input.get(side);
	input.get(size);
	input.get(indexoffset);
	if (magic != pack_magic)
		return fail("not a terrain chunks file");
	if (version != pack_version)
		return fail("unsupported version " + std::to_string(version));
	if (side != pack_side || std::abs(size - chunk_size) > 1e-3)
		return fail("unsupported chunk layout");
	m_key = {px, pz};

	std::vector<std::uint8_t> index(pack_side * pack_side * index_entry_size);
	file.seekg(static_cast<std::streamoff>(indexoffset));
	file.read(reinterpret_cast<char *>(index.data()), index.size());
	if (file.gcount() != static_cast<std::streamsize>(index.size()))
		return fail("damaged chunk index");
	reader entries{index.data(), index.size()};
	for (auto &slices : m_slices)
	{
		std::uint64_t tableoffset{0};
		std::uint32_t count{0}, reserved{0};
		entries.get(tableoffset);
		entries.get(count);
		entries.get(reserved);
		if (tableoffset == 0 || count == 0)
			continue;
		if (count > 64)
			return fail("damaged chunk index");
		std::vector<std::uint8_t> table(count * slice_entry_size);
		file.seekg(static_cast<std::streamoff>(tableoffset));
		file.read(reinterpret_cast<char *>(table.data()), table.size());
		if (file.gcount() != static_cast<std::streamsize>(table.size()))
			return fail("damaged slice table");
		reader tableinput{table.data(), table.size()};
		slices.resize(count);
		for (auto &slice : slices)
		{
			std::uint32_t reserved2{0};
			tableinput.get(slice.tag);
			tableinput.get(slice.codec);
			tableinput.get(slice.offset);
			tableinput.get(slice.stored);
			tableinput.get(slice.size);
			tableinput.get(slice.crc);
			tableinput.get(reserved2);
			if (slice.stored > max_slice_size || slice.size > max_slice_size)
				return fail("damaged slice table");
		}
	}
	return true;
}

bool pack_file::has(chunk_key const &Key) const
{
	if (pack_of(Key) != m_key)
		return false;
	auto const x{Key.first - m_key.first * pack_side};
	auto const z{Key.second - m_key.second * pack_side};
	return false == m_slices[static_cast<std::size_t>(z) * pack_side + x].empty();
}

std::vector<chunk_key> pack_file::chunks() const
{
	std::vector<chunk_key> result;
	for (int z = 0; z < pack_side; ++z)
		for (int x = 0; x < pack_side; ++x)
			if (false == m_slices[static_cast<std::size_t>(z) * pack_side + x].empty())
				result.emplace_back(m_key.first * pack_side + x, m_key.second * pack_side + z);
	return result;
}

std::shared_ptr<chunk_data> pack_file::read(chunk_key const &Key, std::string *Error) const
{
	auto const fail = [&](std::string const &Message) -> std::shared_ptr<chunk_data> {
		if (Error != nullptr)
			*Error = m_path + ": chunk " + std::to_string(Key.first) + "," + std::to_string(Key.second) + ": " + Message;
		return nullptr;
	};
	if (false == has(Key))
		return fail("not in the file");
	auto const &slices{m_slices[static_cast<std::size_t>(Key.second - m_key.second * pack_side) * pack_side + (Key.first - m_key.first * pack_side)]};
	std::ifstream file{m_path, std::ios::binary};
	if (false == file.good())
		return fail("can't open the file");

	auto const load = [&](slice const &Slice, std::vector<std::uint8_t> &Out) {
		std::vector<std::uint8_t> stored(Slice.stored);
		file.seekg(static_cast<std::streamoff>(Slice.offset));
		file.read(reinterpret_cast<char *>(stored.data()), stored.size());
		if (file.gcount() != static_cast<std::streamsize>(stored.size()))
			return false;
		if (Slice.codec == codec_stored)
			Out = std::move(stored);
		else if (Slice.codec == codec_zstd)
		{
			Out.resize(Slice.size);
			auto const result{ZSTD_decompress(Out.data(), Out.size(), stored.data(), stored.size())};
			if (ZSTD_isError(result) || result != Slice.size)
				return false;
		}
		else
			return false;
		return Out.size() == Slice.size && checksum(Out) == Slice.crc;
	};

	auto chunk{std::make_shared<chunk_data>()};
	chunk->key = Key;
	slice const *heights{nullptr}, *base{nullptr}, *adjust{nullptr}, *paint{nullptr};
	bool header{false};
	for (auto const &slice : slices)
	{
		if (slice.tag == tag_chunk)
		{
			std::vector<std::uint8_t> data;
			if (false == load(slice, data))
				return fail("damaged chunk header");
			reader input{data.data(), data.size()};
			std::uint16_t cells{0}, samples{0};
			input.get(cells);
			input.get(samples);
			for (auto &layer : chunk->layers)
				input.get(layer);
			chunk->cells = cells;
			chunk->paint = samples;
			header = true;
		}
		else if (slice.tag == tag_heights)
			heights = &slice;
		else if (slice.tag == tag_base)
			base = &slice;
		else if (slice.tag == tag_adjust)
			adjust = &slice;
		else if (slice.tag == tag_paint)
			paint = &slice;
		// slices of unknown kinds are left for newer versions of the simulator
	}
	if (false == header || heights == nullptr || chunk->cells <= 0 || chunk->cells > 4096)
		return fail("incomplete chunk");
	if (chunk->layers[0] == no_layer)
		chunk->layers[0] = 0;
	auto const count{static_cast<std::size_t>(chunk->side()) * chunk->side()};
	std::vector<std::uint8_t> data;
	if (false == load(*heights, data) || false == decode_heights(data, count, chunk->heights))
		return fail("damaged heights");
	if (base != nullptr && (false == load(*base, data) || false == decode_heights(data, count, chunk->base)))
		return fail("damaged base heights");
	// touch-ups go with the base heights only
	if (adjust != nullptr && false == chunk->base.empty() && (false == load(*adjust, data) || false == decode_heights(data, count, chunk->adjust)))
		return fail("damaged touch-ups");
	if (chunk->paint > 0)
	{
		auto const expected{static_cast<std::size_t>(chunk->paint + 1) * (chunk->paint + 1) * chunk->layer_count()};
		if (paint == nullptr || false == load(*paint, chunk->weights) || chunk->weights.size() != expected)
			return fail("damaged paint");
	}
	return chunk;
}

bool write_pack(std::string const &Path, pack_key const &Key, std::vector<chunk_ptr> const &Chunks, bool const Compress, std::string *Error)
{
	struct slice_record
	{
		std::uint32_t tag;
		std::uint32_t codec;
		std::uint64_t offset;
		std::uint32_t stored;
		std::uint32_t size;
		std::uint32_t crc;
	};
	std::array<std::vector<slice_record>, pack_side * pack_side> tables;
	std::vector<std::uint8_t> payload; // slice data, placed after the header

	auto const add = [&](std::vector<slice_record> &Table, std::uint32_t const Tag, std::vector<std::uint8_t> const &Data) {
		slice_record record{Tag, codec_stored, header_size + payload.size(), static_cast<std::uint32_t>(Data.size()), static_cast<std::uint32_t>(Data.size()), checksum(Data)};
		if (Compress && Data.size() > 64)
		{
			std::vector<std::uint8_t> packed(ZSTD_compressBound(Data.size()));
			auto const result{ZSTD_compress(packed.data(), packed.size(), Data.data(), Data.size(), zstd_level)};
			if (false == ZSTD_isError(result) && result < Data.size())
			{
				packed.resize(result);
				record.codec = codec_zstd;
				record.stored = static_cast<std::uint32_t>(packed.size());
				payload.insert(payload.end(), packed.begin(), packed.end());
				Table.push_back(record);
				return;
			}
		}
		payload.insert(payload.end(), Data.begin(), Data.end());
		Table.push_back(record);
	};

	for (auto const &chunk : Chunks)
	{
		if (chunk == nullptr || false == chunk->valid() || pack_of(chunk->key) != Key)
			continue;
		auto &table{tables[static_cast<std::size_t>(chunk->key.second - Key.second * pack_side) * pack_side + (chunk->key.first - Key.first * pack_side)]};
		table.clear();
		std::vector<std::uint8_t> data;
		put<std::uint16_t>(data, static_cast<std::uint16_t>(chunk->cells));
		put<std::uint16_t>(data, static_cast<std::uint16_t>(chunk->layer_count() > 1 ? chunk->paint : 0));
		for (auto const layer : chunk->layers)
			put<std::uint16_t>(data, layer);
		add(table, tag_chunk, data);
		data.clear();
		encode_heights(chunk->heights, data);
		add(table, tag_heights, data);
		if (chunk->base.size() == chunk->heights.size())
		{
			data.clear();
			encode_heights(chunk->base, data);
			add(table, tag_base, data);
			if (chunk->adjust.size() == chunk->heights.size())
			{
				data.clear();
				encode_heights(chunk->adjust, data);
				add(table, tag_adjust, data);
			}
		}
		if (chunk->paint > 0 && chunk->layer_count() > 1)
			add(table, tag_paint, chunk->weights);
	}

	std::vector<std::uint8_t> file;
	file.reserve(header_size + payload.size() + 4096);
	put<std::uint32_t>(file, pack_magic);
	put<std::uint32_t>(file, pack_version);
	put<std::int32_t>(file, Key.first);
	put<std::int32_t>(file, Key.second);
	put<std::uint32_t>(file, pack_side);
	put<float>(file, static_cast<float>(chunk_size));
	auto const indexfield{file.size()};
	put<std::uint64_t>(file, 0);
	file.resize(header_size, 0);
	file.insert(file.end(), payload.begin(), payload.end());
	// slice tables
	std::array<std::uint64_t, pack_side * pack_side> tableoffsets{};
	for (std::size_t i = 0; i < tables.size(); ++i)
	{
		if (tables[i].empty())
			continue;
		tableoffsets[i] = file.size();
		for (auto const &record : tables[i])
		{
			put<std::uint32_t>(file, record.tag);
			put<std::uint32_t>(file, record.codec);
			put<std::uint64_t>(file, record.offset);
			put<std::uint32_t>(file, record.stored);
			put<std::uint32_t>(file, record.size);
			put<std::uint32_t>(file, record.crc);
			put<std::uint32_t>(file, 0);
		}
	}
	// chunk index
	put_at<std::uint64_t>(file, indexfield, file.size());
	for (std::size_t i = 0; i < tables.size(); ++i)
	{
		put<std::uint64_t>(file, tableoffsets[i]);
		put<std::uint32_t>(file, static_cast<std::uint32_t>(tables[i].size()));
		put<std::uint32_t>(file, 0);
	}

	// written next to the old file first, so a failed write doesn't take the old content with it
	std::error_code ec;
	std::filesystem::create_directories(std::filesystem::path(Path).parent_path(), ec);
	auto const temporary{Path + ".tmp"};
	{
		std::ofstream out{temporary, std::ios::binary | std::ios::trunc};
		out.write(reinterpret_cast<char const *>(file.data()), file.size());
		if (false == out.good())
		{
			if (Error != nullptr)
				*Error = "can't write " + temporary;
			return false;
		}
	}
	std::filesystem::rename(temporary, Path, ec);
	if (ec)
	{
		// some systems won't replace an existing file with rename
		std::filesystem::remove(Path, ec);
		std::filesystem::rename(temporary, Path, ec);
		if (ec)
		{
			if (Error != nullptr)
				*Error = "can't replace " + Path + ": " + ec.message();
			return false;
		}
	}
	return true;
}

std::vector<std::uint32_t> triangulate(std::vector<glm::dvec2> const &Outline)
{
	std::vector<std::uint32_t> result;
	auto const count{Outline.size()};
	if (count < 3)
		return result;
	// ear clipping, the outline turned counterclockwise first
	double area{0.0};
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const &a{Outline[i]};
		auto const &b{Outline[(i + 1) % count]};
		area += a.x * b.y - b.x * a.y;
	}
	std::vector<std::uint32_t> ring(count);
	for (std::size_t i = 0; i < count; ++i)
		ring[i] = static_cast<std::uint32_t>(area >= 0.0 ? i : count - 1 - i);
	auto const cross = [](glm::dvec2 const &O, glm::dvec2 const &A, glm::dvec2 const &B) { return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x); };
	auto const inside = [&](glm::dvec2 const &P, glm::dvec2 const &A, glm::dvec2 const &B, glm::dvec2 const &C) {
		return cross(A, B, P) >= 0.0 && cross(B, C, P) >= 0.0 && cross(C, A, P) >= 0.0;
	};
	std::size_t guard{0};
	while (ring.size() > 3 && guard < count * count)
	{
		++guard;
		bool clipped{false};
		for (std::size_t i = 0; i < ring.size(); ++i)
		{
			auto const prev{ring[(i + ring.size() - 1) % ring.size()]};
			auto const curr{ring[i]};
			auto const next{ring[(i + 1) % ring.size()]};
			auto const &a{Outline[prev]};
			auto const &b{Outline[curr]};
			auto const &c{Outline[next]};
			if (cross(a, b, c) <= 1e-12)
				continue; // reflex or degenerate corner
			bool ear{true};
			for (auto const other : ring)
			{
				if (other == prev || other == curr || other == next)
					continue;
				if (inside(Outline[other], a, b, c))
				{
					ear = false;
					break;
				}
			}
			if (false == ear)
				continue;
			result.insert(result.end(), {prev, curr, next});
			ring.erase(ring.begin() + i);
			clipped = true;
			break;
		}
		if (false == clipped)
			break; // a self-crossing outline; what's left goes as a fan
	}
	for (std::size_t i = 1; i + 1 < ring.size(); ++i)
		result.insert(result.end(), {ring[0], ring[i], ring[i + 1]});
	return result;
}

// meshes

int max_stride(int const Cells)
{
	int stride{1};
	while (stride * 4 <= Cells)
		stride *= 2;
	return stride;
}

namespace
{
int segments(int const Cells, int const Stride)
{
	return (Cells + Stride - 1) / Stride;
}
} // namespace

std::size_t mesh_vertex_count(int const Cells, int const Stride)
{
	auto const n{static_cast<std::size_t>(segments(Cells, Stride))};
	return (n + 1) * (n + 1) + 4 * (n + 1);
}

void build_indices(int const Cells, int const Stride, gfx::index_array &Out)
{
	auto const n{static_cast<gfx::basic_index>(segments(Cells, Stride))};
	auto const row{n + 1};
	auto const grid{row * row};
	Out.clear();
	Out.reserve(static_cast<std::size_t>(n) * n * 6 + static_cast<std::size_t>(n) * 24);
	auto const at = [&](gfx::basic_index const I, gfx::basic_index const J) { return J * row + I; };
	for (gfx::basic_index j = 0; j < n; ++j)
		for (gfx::basic_index i = 0; i < n; ++i)
		{
			auto const v00{at(i, j)}, v10{at(i + 1, j)}, v01{at(i, j + 1)}, v11{at(i + 1, j + 1)};
			Out.insert(Out.end(), {v00, v01, v10, v11, v10, v01});
		}
	// skirts: south (low z) and east face the way the edge is walked turns them, north and west are turned around
	auto const skirt = [&](gfx::basic_index const Skirt, auto const &Edge, bool const Flip) {
		for (gfx::basic_index k = 0; k < n; ++k)
		{
			auto const a{Edge(k)}, b{Edge(k + 1)}, la{Skirt + k}, lb{Skirt + k + 1};
			if (Flip)
				Out.insert(Out.end(), {la, b, a, la, lb, b});
			else
				Out.insert(Out.end(), {a, b, la, b, lb, la});
		}
	};
	skirt(grid, [&](gfx::basic_index const K) { return at(K, 0); }, false);
	skirt(grid + row, [&](gfx::basic_index const K) { return at(K, n); }, true);
	skirt(grid + 2 * row, [&](gfx::basic_index const K) { return at(0, K); }, true);
	skirt(grid + 3 * row, [&](gfx::basic_index const K) { return at(n, K); }, false);
}

void build_mesh(chunk_data const &Chunk, int const Stride, glm::dvec3 const &Origin, height_source const &Outside, mesh &Out)
{
	auto const cells{Chunk.cells};
	auto const stride{std::clamp(Stride, 1, max_stride(cells))};
	auto const n{segments(cells, stride)};
	auto const spacing{Chunk.spacing()};
	auto const low{Chunk.corner()};
	auto const texturebase{pack_corner(pack_of(Chunk.key))};
	auto const coordinate = [&](int const I) { return std::min(I * stride, cells); };

	// heights of the grid points, the neighbouring chunks asked about the points past the edges
	auto const sample = [&](int const Ix, int const Iz) -> double {
		if (Ix >= 0 && Ix <= cells && Iz >= 0 && Iz <= cells)
			return Chunk.height(Ix, Iz);
		double height;
		if (Outside && Outside(low.x + Ix * spacing, low.y + Iz * spacing, height))
			return height;
		return Chunk.height(std::clamp(Ix, 0, cells), std::clamp(Iz, 0, cells));
	};
	// normals from the differences over the stride, so the distant, coarse grid is lit as it's drawn
	auto const normal = [&](int const Ix, int const Iz) {
		auto const dx{(sample(Ix + stride, Iz) - sample(Ix - stride, Iz)) / (2.0 * stride * spacing)};
		auto const dz{(sample(Ix, Iz + stride) - sample(Ix, Iz - stride)) / (2.0 * stride * spacing)};
		return glm::normalize(glm::vec3(static_cast<float>(-dx), 1.f, static_cast<float>(-dz)));
	};

	// the skirt has to reach below the edge of a neighbour drawn with a coarser grid
	double deviation{0.0};
	{
		auto const coarse{std::min(stride * 4, cells)};
		auto const edge = [&](auto const &Height) {
			for (int start = 0; start < cells; start += coarse)
			{
				auto const end{std::min(start + coarse, cells)};
				auto const a{Height(start)}, b{Height(end)};
				for (int k = start + 1; k < end; ++k)
					deviation = std::max(deviation, std::abs(Height(k) - (a + (b - a) * (k - start) / double(end - start))));
			}
		};
		edge([&](int const K) { return static_cast<double>(Chunk.height(K, 0)); });
		edge([&](int const K) { return static_cast<double>(Chunk.height(K, cells)); });
		edge([&](int const K) { return static_cast<double>(Chunk.height(0, K)); });
		edge([&](int const K) { return static_cast<double>(Chunk.height(cells, K)); });
	}
	auto const skirtdepth{static_cast<float>(0.5 + 2.0 * deviation + 0.05 * stride * spacing)};

	Out.vertices.clear();
	Out.vertices.reserve(mesh_vertex_count(cells, stride));
	glm::dvec3 lowest{std::numeric_limits<double>::max()}, highest{-std::numeric_limits<double>::max()};
	auto const vertex = [&](int const Ix, int const Iz, float const Drop) {
		gfx::basic_vertex v;
		glm::dvec3 const world{low.x + Ix * spacing, Chunk.height(Ix, Iz), low.y + Iz * spacing};
		lowest = glm::min(lowest, world);
		highest = glm::max(highest, world);
		v.position = glm::vec3(world - Origin);
		v.position.y -= Drop;
		v.normal = normal(Ix, Iz);
		// texture coordinates in metres from the corner of the pack: small enough for floats, and seamless for any
		// texture repeating a whole number of times over the pack. v runs along -z, the way the tangent space expects it
		v.texture = glm::vec2(static_cast<float>(world.x - texturebase.x), static_cast<float>(texturebase.y - world.z));
		auto const tangent{glm::vec3(1.f, 0.f, 0.f) - v.normal * v.normal.x};
		v.tangent = glm::vec4(glm::length(tangent) > 1e-6f ? glm::normalize(tangent) : glm::vec3(1.f, 0.f, 0.f), 1.f);
		Out.vertices.push_back(v);
	};
	for (int j = 0; j <= n; ++j)
		for (int i = 0; i <= n; ++i)
			vertex(coordinate(i), coordinate(j), 0.f);
	for (int k = 0; k <= n; ++k)
		vertex(coordinate(k), 0, skirtdepth);
	for (int k = 0; k <= n; ++k)
		vertex(coordinate(k), cells, skirtdepth);
	for (int k = 0; k <= n; ++k)
		vertex(0, coordinate(k), skirtdepth);
	for (int k = 0; k <= n; ++k)
		vertex(cells, coordinate(k), skirtdepth);

	build_indices(cells, stride, Out.indices);
	Out.centre = (lowest + highest) * 0.5;
	Out.radius = static_cast<float>(glm::length(highest - lowest) * 0.5 + skirtdepth);
	Out.low = lowest - glm::dvec3{0.0, skirtdepth, 0.0};
	Out.high = highest;
}

} // namespace heightmap
