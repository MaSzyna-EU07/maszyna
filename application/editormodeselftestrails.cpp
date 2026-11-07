/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "editor/editorSettings.hpp"
#include "editor/editorGeometry.hpp"
#include "application/application.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"

#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "scene/scenelayers.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Track.h"
#include "world/TrackRails.h"
#include "world/MemCell.h"
#include "world/Event.h"
#include "vehicle/DynObj.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#include "world/Sweep.h"
#include "input/keyboardinput.h"
#include "launcher/textures_scanner.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <cstring>
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "scene/sceneterrain.h"
#include "editor/editorSpeed.hpp"

#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>
#include "application/editormodeselftest.h"

namespace
{

// the hashes of the geometry: FNV-1a over its values
std::uint64_t constexpr hash_basis{1469598103934665603ULL};
std::uint64_t constexpr hash_prime{1099511628211ULL};

// a switch of the scenery, not removed in the editor
bool live_switch(TTrack const *Track)
{
	return Track != nullptr && false == Track->m_editorremoved && Track->eType == tt_Switch && Track->SwitchExtension != nullptr;
}

// a track of the scenery for trains, not removed in the editor
bool live_railway(TTrack const *Track)
{
	return Track != nullptr && false == Track->m_editorremoved && (Track->iCategoryFlag & 15) == 1;
}

// the track asked for by its name, or any with no name given
bool named(std::string_view const Name, TTrack const &Track)
{
	return Name.empty() || Track.name() == Name;
}

// the length of a path of the track
double path_length(TTrack const &Track, int const Path)
{
	return Track.eType == tt_Switch ? Track.SwitchExtension->Segments[Path]->GetLength() : Track.Segment->GetLength();
}

// the tracks for trains whose bounds come within a metre of those of the track, the track too, of those Also lets through
std::vector<TTrack *> tracks_around(TTrack &Track, bool (*Also)(TTrack const &))
{
	std::vector<TTrack *> result;
	for (auto *other : simulation::Paths.sequence())
		if (other != nullptr && false == other->m_editorremoved && (other->iCategoryFlag & 15) == 1 && Also(*other) && glm::length(other->location() - Track.location()) < other->radius() + Track.radius() + 1.0)
			result.push_back(other);
	return result;
}

bool any_track(TTrack const &)
{
	return true;
}

bool with_paths(TTrack const &Track)
{
	return false == Track.m_paths.empty();
}

bool with_cut_rails(TTrack const &Track)
{
	return false == Track.m_paths.empty() && track_rails::cut_rails(Track) && false == Track.rail_dimensions().grooved;
}

// how many of a thing were looked at, and how many of them with a fault
struct fault_count
{
	int count{0};
	int faults{0};
};

// the vertices of the first chunks of the rails of the switch, of either texture
std::pair<std::size_t, std::size_t> blade_vertices(TTrack const &Track)
{
	return std::make_pair(Track.Geometry1.empty() ? std::size_t{0} : GfxRenderer->Vertices(Track.Geometry1[0]).size(), Track.Geometry2.empty() ? std::size_t{0} : GfxRenderer->Vertices(Track.Geometry2[0]).size());
}

// the switch set over to a path at once, past its animation
void set_switch(TTrack &Track, int const Path)
{
	Track.Switch(Path);
	Track.SwitchExtension->fOffset = Track.SwitchExtension->fDesiredOffset;
	Track.RaAnimate();
}

// how far along the blade its gauge line gets Width out, -1 where it doesn't
double blade_reach(track_rails::switch_blade const &Shape, double const Width)
{
	for (std::size_t i = 1; i < Shape.gauge.size(); ++i)
		if (Shape.gauge[i].y >= Width)
			return glm::mix(Shape.gauge[i - 1].x, Shape.gauge[i].x, (Width - Shape.gauge[i - 1].y) / std::max(Shape.gauge[i].y - Shape.gauge[i - 1].y, 1e-9));
	return -1.0;
}

// how far out the gauge line of the blade lies at a distance along it
double blade_offset(track_rails::switch_blade const &Shape, double const Distance)
{
	for (std::size_t i = 1; i < Shape.gauge.size(); ++i)
		if (Shape.gauge[i].x >= Distance)
			return glm::mix(Shape.gauge[i - 1].y, Shape.gauge[i].y, (Distance - Shape.gauge[i - 1].x) / std::max(Shape.gauge[i].x - Shape.gauge[i - 1].x, 1e-9));
	return Shape.gauge.empty() ? 0.0 : Shape.gauge.back().y;
}

// how the blade is made
char const *blade_kind(track_rails::switch_blade const &Shape)
{
	if (false == Shape.valid)
		return "old";
	return Shape.rigid ? "geometric rigid" : "geometric";
}

// a switch thrown over for the blade test: its frog, the radius of its diverging path, the vertices with either path set
struct blade_switch
{
	TTrack const *track{nullptr};
	track_rails::switch_frog frog;
	float radius{0.f};
	std::pair<std::size_t, std::size_t> straight;
	std::pair<std::size_t, std::size_t> diverging;
};

// what is wrong with the blade of a path: not cut from the rails, the flangeway too narrow, the heel past the throat of the frog, the vertices
// changing as the switch is thrown
std::string blade_faults(blade_switch const &Switch, track_rails::switch_blade const &Shape, double const Throat)
{
	std::string faults;
	if (track_rails::cut_rails(*Switch.track) && false == Shape.valid)
		faults += " FAULT blade not cut from the rails";
	if (Shape.valid && Shape.opening < track_rails::blade_clearance - 1e-4)
		faults += " FAULT flangeway";
	if (Shape.valid && Switch.frog.valid && Shape.heel >= Throat)
		faults += " FAULT heel past the throat";
	if (Switch.straight != Switch.diverging)
		faults += " FAULT vertex count changes";
	return faults;
}

// the blade of a path of the switch against the curve of the path
void log_blade(blade_switch const &Switch, int const Path)
{
	auto const &track{*Switch.track};
	auto const &shape{track.blade(Path)};
	auto const &segment{*track.SwitchExtension->Segments[Path]};
	auto const old{segment.GetLength() * track_rails::blade_steps(track, Path) / segment.RaSegCount()};
	auto const throat{Switch.frog.valid ? Switch.frog.at[Path] - track_rails::flangeway / std::tan(Switch.frog.angle) : 0.0};
	std::string samples;
	auto worst{0.0};
	for (auto const distance : {1.0, 2.0, 4.0, 6.0, 8.0, 10.0})
	{
		auto const analytic{Switch.radius > 0.f ? distance * distance / (2.0 * Switch.radius) : 0.0};
		samples += format(" %.0f:%.4f/%.4f", distance, blade_offset(shape, distance), analytic);
		if (shape.valid && distance <= shape.heel)
			worst = std::max(worst, std::abs(blade_offset(shape, distance) - analytic));
	}
	auto const faults{blade_faults(Switch, shape, throat)};
	WriteLog(format("SELFTEST blade %s path %d: %s, R %.0f, tip %.3f, 50mm %.3f, full head %.3f, heel %.3f (old %.3f), drop %.4f, opening %.4f at %.3f, throat %.3f, d(x)/x^2/2R%s, worst %.4f, vertices %zu/%zu %zu/%zu%s", track.name().c_str(), Path, blade_kind(shape), Switch.radius, shape.tip, blade_reach(shape, track_rails::blade_lowered + track_rails::blade_gap), shape.full, shape.heel, old, shape.drop, shape.opening, shape.openingat,
	                throat, samples.c_str(), worst, Switch.straight.first, Switch.diverging.first, Switch.straight.second, Switch.diverging.second, faults.c_str()));
}

// blade [switch]: the blades of the switches against the curves of their paths, and the vertices of the rails with either path set
void selftest_blade(std::istringstream &Words)
{
	std::string name;
	Words >> name;
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == live_switch(track) || false == named(name, *track))
			continue;
		set_switch(*track, 0);
		auto const straight{blade_vertices(*track)};
		set_switch(*track, 1);
		auto const diverging{blade_vertices(*track)};
		set_switch(*track, 0);
		blade_switch const thrown{track, track->frog(), std::abs(track->m_paths.size() > 1 ? track->m_paths[1].radius : 0.f), straight, diverging};
		for (int path = 0; path < 2; ++path)
			log_blade(thrown, path);
	}
}

// the frog of a switch made from the rails: where the rails cross, the parts around
void log_frog(TTrack const &Track)
{
	auto const frog{Track.frog()};
	if (false == frog.valid)
	{
		WriteLog(format("SELFTEST frog %s: no frog, paths %.3f and %.3f m, blades %.3f m", Track.name().c_str(), Track.SwitchExtension->Segments[0]->GetLength(), Track.SwitchExtension->Segments[1]->GetLength(), frog.blades[0]));
		return;
	}
	auto const n{1.0 / std::tan(frog.angle)};
	glm::dvec3 point;
	glm::dvec3 across;
	Track.SwitchExtension->Segments[0]->plan_frame(frog.at[0], point, across);
	auto const gauge{point + across * (frog.side[0] * 0.5 * std::abs(Track.fTrackWidth))};
	auto const slope{std::tan(frog.angle)};
	auto const check{0.5 * track_rails::check_length(slope)};
	WriteLog(format("SELFTEST frog %s: crossing at %.3f / %.3f m (%.3f %.3f %.3f), 1:%.2f, side %d, blades %.3f / %.3f m, throat %.3f, nose %.3f, wing end %.3f, check rails %.3f-%.3f / %.3f-%.3f m", Track.name().c_str(),
	    frog.at[0], frog.at[1], gauge.x, gauge.y, gauge.z, n, frog.side[0], frog.blades[0], frog.blades[1], frog.at[0] - track_rails::flangeway / slope, frog.at[0] + track_rails::nose_tip / slope, frog.at[0] + track_rails::wing_reach(slope),
	    frog.at[0] - 0.25 - check, frog.at[0] - 0.25 + check, frog.at[1] - 0.25 - check, frog.at[1] - 0.25 + check));
}

// frog [switch]: the frogs of the switches
void selftest_frog(std::istringstream &Words)
{
	std::string name;
	Words >> name;
	for (auto const *track : simulation::Paths.sequence())
		if (live_switch(track) && named(name, *track))
			log_frog(*track);
}

// whether the track lies farther than Radius from the point in plan, by its nearest point
bool track_beyond(TTrack const &Track, glm::dvec3 const &Point, double const Radius)
{
	return geometry::plan_distance(editor_track::point_at(Track, editor_track::nearest_parameter(Track, Point)), Point) > Radius;
}

// the axes of the paths of a track in plan, a point every 25 cm
void dump_axes(std::ostream &Output, TTrack const &Track)
{
	auto const paths{Track.eType == tt_Switch ? 2 : 1};
	for (int path = 0; path < paths; ++path)
	{
		auto const &segment{Track.eType == tt_Switch ? Track.SwitchExtension->Segments[path] : Track.Segment};
		if (segment == nullptr)
			continue;
		Output << "axis " << Track.name() << ' ' << path;
		for (int k = 0; k * 0.25 <= segment->GetLength() + 1e-6; ++k)
		{
			auto const at{k * 0.25};
			glm::dvec3 spot;
			glm::dvec3 across;
			segment->plan_frame(std::min(at, segment->GetLength()), spot, across);
			Output << format(" %.4f %.4f", spot.x, spot.z);
		}
		Output << "\n";
	}
}

// the triangles of the trackbed of a track, how many of them
int dump_bed(std::ostream &Output, TTrack const &Track)
{
	std::vector<gfx::geometry_handle> beds;
	if (Track.eType == tt_Switch && Track.SwitchExtension != nullptr)
		beds.push_back(Track.SwitchExtension->Geometry3);
	else
		beds.insert(beds.end(), Track.Geometry2.begin(), Track.Geometry2.end());
	auto triangles{0};
	for (auto const &bed : beds)
	{
		if (bed == null_handle)
			continue;
		auto const &vertices{GfxRenderer->Vertices(bed)};
		for (std::size_t i = 2; i < vertices.size(); ++i)
		{
			glm::dvec3 const a{glm::dvec3{vertices[i - 2].position} + Track.m_origin};
			glm::dvec3 const b{glm::dvec3{vertices[i - 1].position} + Track.m_origin};
			glm::dvec3 const c{glm::dvec3{vertices[i].position} + Track.m_origin};
			Output << format("tri %s %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", Track.name().c_str(), a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
			++triangles;
		}
	}
	return triangles;
}

// beddump <x> <z> <radius> <file>: the trackbed triangles and the axes of the paths of the tracks near a point, in plan, for a look at where the
// trackbed has holes
void selftest_beddump(std::istringstream &Words)
{
	glm::dvec3 point{0.0};
	double radius{30.0};
	std::string file;
	Words >> point.x >> point.z >> radius >> file;
	std::ofstream output{file, std::ios_base::trunc};
	auto triangles{0};
	for (auto const *track : simulation::Paths.sequence())
	{
		if (false == live_railway(track) || track->m_paths.empty())
			continue;
		if (track_beyond(*track, point, radius))
			continue;
		dump_axes(output, *track);
		triangles += dump_bed(output, *track);
	}
	WriteLog(format("SELFTEST beddump: %d triangles into %s", triangles, file.c_str()));
}

// what a chunk of the rails is drawn as: t triangles, s a strip, Kind where the renderer doesn't tell
char chunk_kind(int const Primitive, char const Kind)
{
	if (Primitive == GL_TRIANGLES)
		return 't';
	if (Primitive == GL_TRIANGLE_STRIP)
		return 's';
	return Kind;
}

// the chunks of a list of the geometry of a track with their vertices in the world, how many of them
int dump_chunks(std::ostream &Output, TTrack const &Track, TTrack::geometryhandle_sequence const &Sequence, char const *List, char const Kind)
{
	auto chunks{0};
	for (std::size_t index = 0; index < Sequence.size(); ++index)
	{
		auto const &vertices{GfxRenderer->Vertices(Sequence[index])};
		if (vertices.empty())
			continue;
		auto const kind{chunk_kind(GfxRenderer->Primitive(Sequence[index]), Kind)};
		Output << format("chunk %s %s %zu %c %zu\n", Track.name().c_str(), List, index, kind, vertices.size());
		for (auto const &vertex : vertices)
		{
			glm::dvec3 const at{glm::dvec3{vertex.position} + Track.m_origin};
			Output << format("%.4f %.4f %.4f\n", at.x, at.y, at.z);
		}
		++chunks;
	}
	return chunks;
}

// raildump <x> <z> <radius> <file>: the rail geometry of the tracks near a point as drawn: s strips of the lofts, t triangles of the cut faces and
// the rest
void selftest_raildump(std::istringstream &Words)
{
	glm::dvec3 point{0.0};
	double radius{30.0};
	std::string file;
	Words >> point.x >> point.z >> radius >> file;
	std::ofstream output{file, std::ios_base::trunc};
	auto chunks{0};
	for (auto const *track : simulation::Paths.sequence())
	{
		if (false == live_railway(track) || track->m_paths.empty())
			continue;
		if (track_beyond(*track, point, radius))
			continue;
		output << format("track %s %.4f\n", track->name().c_str(), track->m_paths.front().points[segment_data::point::start].y);
		chunks += dump_chunks(output, *track, track->Geometry1, "rails1", 's');
		chunks += dump_chunks(output, *track, track->Geometry2, track->eType == tt_Switch ? "rails2" : "bed", 's');
		chunks += dump_chunks(output, *track, track->GeometryCut1, "cut1", 't');
		chunks += dump_chunks(output, *track, track->GeometryCut2, "cut2", 't');
		for (auto const &tread : track->GeometryTreads)
		{
			chunks += dump_chunks(output, *track, tread.geometry, "tread", 's');
			chunks += dump_chunks(output, *track, tread.cuts, "treadcut", 't');
		}
	}
	WriteLog(format("SELFTEST raildump: %d chunks into %s", chunks, file.c_str()));
}

// a value into the hash, rounded to a tenth of a millimetre
void mix_rounded(std::uint64_t &Hash, float const Value)
{
	Hash = (Hash ^ static_cast<std::uint64_t>(static_cast<std::int64_t>(std::llround(Value * 1e4)))) * hash_prime;
}

// a value into the hash by its bits
void mix_exact(std::uint64_t &Hash, float const Value)
{
	Hash = (Hash ^ std::bit_cast<std::uint32_t>(Value)) * hash_prime;
}

// the name of a material into the hash, with a mark after it
void mix_name(std::uint64_t &Hash, material_handle const Material)
{
	for (auto const letter : (Material != null_handle ? GfxRenderer->Material(Material)->GetName() : std::string{}) + '|')
		Hash = (Hash ^ static_cast<unsigned char>(letter)) * hash_prime;
}

// the hashes of the geometry of a track: rounded, and by the bits of the values with the materials of the treads; its chunks and vertices
struct geometry_hash
{
	std::uint64_t rounded{hash_basis};
	std::uint64_t exact{hash_basis};
	std::size_t vertices{0};
	int chunks{0};
};

// a chunk of the geometry into the hashes, its vertices and their user data
void hash_chunk(geometry_hash &Hash, gfx::geometry_handle const &Geometry)
{
	++Hash.chunks;
	for (auto const &vertex : GfxRenderer->Vertices(Geometry))
	{
		for (int i = 0; i < 3; ++i)
		{
			mix_rounded(Hash.rounded, vertex.position[i]);
			mix_rounded(Hash.rounded, vertex.normal[i]);
			mix_exact(Hash.exact, vertex.position[i]);
			mix_exact(Hash.exact, vertex.normal[i]);
		}
		mix_rounded(Hash.rounded, vertex.texture.x);
		mix_rounded(Hash.rounded, vertex.texture.y);
		mix_exact(Hash.exact, vertex.texture.x);
		mix_exact(Hash.exact, vertex.texture.y);
		++Hash.vertices;
	}
	for (auto const &userdata : GfxRenderer->UserData(Geometry))
	{
		for (int i = 0; i < 4; ++i)
			mix_exact(Hash.exact, userdata.data[i]);
	}
	mix_exact(Hash.exact, -2.f);
}

// the geometry of a track hashed
geometry_hash hash_track(TTrack const &Track)
{
	geometry_hash result;
	std::vector<TTrack::geometryhandle_sequence const *> sequences{&Track.Geometry1, &Track.Geometry2, &Track.GeometryCut1, &Track.GeometryCut2};
	for (auto const &tread : Track.GeometryTreads)
	{
		sequences.push_back(&tread.geometry);
		sequences.push_back(&tread.cuts);
		mix_name(result.exact, tread.material);
		mix_name(result.exact, tread.matte);
	}
	for (auto const *sequence : sequences)
	{
		for (auto const &geometry : *sequence)
		{
			// an empty piece of a rail gets no chunk, as the renderer skips it
			if (geometry == null_handle)
				continue;
			hash_chunk(result, geometry);
		}
		mix_rounded(result.rounded, -1.f);
		mix_exact(result.exact, -1.f);
	}
	return result;
}

// geomhash [track]: the hashes of the geometry of the tracks, each and all together
void selftest_geomhash(std::istringstream &Words)
{
	std::string name;
	Words >> name;
	std::uint64_t total{hash_basis};
	std::uint64_t exacttotal{hash_basis};
	std::size_t vertices{0};
	int tracks{0};
	for (auto const *track : simulation::Paths.sequence())
	{
		if (false == live_railway(track) || false == named(name, *track))
			continue;
		auto const hash{hash_track(*track)};
		WriteLog(format("SELFTEST geom %s: chunks %d, vertices %zu, hash %016llx, exact %016llx", track->name().c_str(), hash.chunks, hash.vertices, static_cast<unsigned long long>(hash.rounded), static_cast<unsigned long long>(hash.exact)));
		total = (total ^ hash.rounded) * hash_prime;
		exacttotal = (exacttotal ^ hash.exact) * hash_prime;
		vertices += hash.vertices;
		++tracks;
	}
	WriteLog(format("SELFTEST geom total: tracks %d, vertices %zu, hash %016llx, exact %016llx", tracks, vertices, static_cast<unsigned long long>(total), static_cast<unsigned long long>(exacttotal)));
}

// frogkey <switch> [on|off]: the key of the frog made from the rails of the switch set, Snapshot keeping the switch as it was for an undo, and
// how the key is written and what it lays
void selftest_frogkey(std::istringstream &Words, auto const &Snapshot)
{
	std::string name;
	std::string value;
	Words >> name >> value;
	auto *track{simulation::Paths.find(name)};
	if (track == nullptr || track->eType != tt_Switch || track->SwitchExtension == nullptr)
	{
		WriteLog(format("SELFTEST frogkey %s: no such switch", name.c_str()));
		return;
	}
	if (value == "on" || value == "off")
	{
		auto const before{editor_track::capture(*track)};
		track->SwitchExtension->m_frogs = (value == "on");
		Snapshot(track, before);
		editor_track::commit_parameters(*track);
	}
	std::ostringstream text;
	track->export_as_text(text);
	WriteLog(format("SELFTEST frogkey %s: key %d, written %d, frog %d, crossings %zu, profile '%s'", name.c_str(), track->SwitchExtension->m_frogs ? 1 : 0, text.str().find(" frogs ") != std::string::npos ? 1 : 0,
	    track->frog().valid ? 1 : 0, track->rail_crossings().size(), track->m_profile1.first.c_str()));
}

// two tracks the heads of whose rails are measured against each other: their names, the heads of their profiles
struct head_gap
{
	TTrack const *a{nullptr};
	TTrack const *b{nullptr};
	char const *first{nullptr};
	char const *second{nullptr};
	double head_a{0.0};
	double head_b{0.0};
};

// the least gap between the head of a rail of the first track and the head of a rail of the second one along it, where they run side by side
void log_head_gap(head_gap const &Gap, int const Pa, int const Sa, int const Pb, int const Sb)
{
	auto const length{path_length(*Gap.a, Pa)};
	auto const lengthb{path_length(*Gap.b, Pb)};
	auto least{1e9};
	auto where{0.0};
	for (int k = 0; k * 0.02 <= length; ++k)
	{
		auto const at{k * 0.02};
		double d{0.0};
		double l{0.0};
		track_rails::rail_offset(*Gap.b, Pb, Sb, track_rails::rail_point(*Gap.a, Pa, Sa, at, Gap.head_a), d, l);
		if (d < 0.0 || d > lengthb || l < -0.3 || l > 0.3)
			continue;
		if (l - Gap.head_b < least)
		{
			least = l - Gap.head_b;
			where = at;
		}
	}
	if (least < 1e8)
		WriteLog(format("SELFTEST headgap %s/%d%+d to %s/%d%+d: %.1f mm at %.2f m (heads %.1f %.1f mm, width %.4f)", Gap.first, Pa, Sa, Gap.second, Pb, Sb, least * 1000.0, where, Gap.head_a * 1000.0,
		                Gap.head_b * 1000.0, Gap.a->fTrackWidth));
}

// the gaps between the head of a rail of the first track and the heads of the rails of the second one
void log_head_gaps(head_gap const &Gap, int const Pa, int const Sa)
{
	for (int pb = 0; pb < (Gap.b->eType == tt_Switch ? 2 : 1); ++pb)
		for (auto const sb : {-1, 1})
			log_head_gap(Gap, Pa, Sa, pb, sb);
}

// headgap <track> <track>: the least gap between the heads of the rails of two tracks running side by side, by the gauge lines and the head of
// the profile
void selftest_headgap(std::istringstream &Words)
{
	std::string first;
	std::string second;
	Words >> first >> second;
	auto const *a{simulation::Paths.find(first)};
	auto const *b{simulation::Paths.find(second)};
	if (a == nullptr || b == nullptr)
	{
		WriteLog("SELFTEST headgap: no such tracks");
		return;
	}
	head_gap const gap{a, b, first.c_str(), second.c_str(), a->rail_dimensions().head, b->rail_dimensions().head};
	for (int pa = 0; pa < (a->eType == tt_Switch ? 2 : 1); ++pa)
		for (auto const sa : {-1, 1})
			log_head_gaps(gap, pa, sa);
}

// a rail of a track looked at against the rails of the tracks around it
struct rail_probe
{
	TTrack const *track{nullptr};
	int path{0};
	int side{0};
	std::vector<TTrack *> const *others{nullptr};
};

// the rails of another track the head of a check rail runs over at a point of it, each put into Hits once
void check_hits_on(std::string &Hits, rail_probe const &Probe, TTrack const &Other, glm::dvec3 const &Point, double const At)
{
	for (int otherpath = 0; otherpath < (Other.eType == tt_Switch ? 2 : 1); ++otherpath)
		for (auto const otherside : {-1, 1})
		{
			if (&Other == Probe.track && otherpath == Probe.path && otherside == Probe.side)
				continue;
			auto const length{path_length(Other, otherpath)};
			double d{0.0};
			double l{0.0};
			track_rails::rail_offset(Other, otherpath, otherside, Point, d, l);
			if (d < 0.0 || d > length || l < 0.0 || l > Other.rail_dimensions().head)
				continue;
			auto const hit{format(" %s/%d%+d", Other.name().c_str(), otherpath, otherside)};
			if (Hits.find(hit) == std::string::npos)
				Hits += hit + format(" at %.2f m", At);
		}
}

// the rails of the tracks around the head of a check rail runs over along it, of the head of the rail Head wide
std::string check_hits(rail_probe const &Probe, double const From, double const To, double const Head)
{
	std::string hits;
	for (int k = 0; From + k * 0.05 <= To; ++k)
		// the head of the check rail as drawn, 40 mm at the head of the rail of 73.5
		for (auto const lateral : {-track_rails::check_flangeway, -track_rails::check_flangeway - 0.020 * Head / 0.0735, -track_rails::check_flangeway - 0.040 * Head / 0.0735})
		{
			auto const at{From + k * 0.05};
			auto const point{track_rails::rail_point(*Probe.track, Probe.path, Probe.side, at, lateral)};
			for (auto const *other : *Probe.others)
				check_hits_on(hits, Probe, *other, point, at);
		}
	return hits;
}

// the check rails of a rail of the track as they're drawn, each logged with the rails it runs over
void log_checks(rail_probe const &Probe, std::vector<track_rails::check_rail> const &Checks, knots::line const &Wingends, double const Head, std::string const &Name, fault_count &Tally)
{
	for (auto const &check : Checks)
	{
		if (track_rails::replaced_by_wing(check, Wingends))
			continue;
		auto const ownlength{path_length(*Probe.track, Probe.path)};
		auto const from{std::max(check.from, 0.0)};
		auto const to{std::min(check.to, ownlength)};
		if (to - from <= 0.01)
			continue;
		++Tally.count;
		auto const hits{check_hits(Probe, from, to, Head)};
		if (false == hits.empty())
			++Tally.faults;
		WriteLog(format("SELFTEST check %s path %d side %+d: %.3f-%.3f m drawn of %.3f-%.3f m%s", Name.c_str(), Probe.path, Probe.side, from, to, check.from, check.to, hits.empty() ? ", clear" : (", over" + hits + " FAULT").c_str()));
	}
}

// checks <track>: the check rails of a track as they're drawn, and the rails of the tracks they run into
void selftest_checks(std::istringstream &Words)
{
	std::string name;
	Words >> name;
	fault_count checks;
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == live_railway(track) || track->name() != name)
			continue;
		auto const layout{track->plan_rails(true)};
		auto const size{track->rail_dimensions()};
		auto const others{tracks_around(*track, any_track)};
		for (int path = 0; path < (track->eType == tt_Switch ? 2 : 1); ++path)
			for (int onright = 0; onright < 2; ++onright)
				log_checks({track, path, onright ? 1 : -1, &others}, layout.checks[path][onright], layout.wingends[path][onright], size.head, name, checks);
	}
	WriteLog(format("SELFTEST checks: %d check rails, %d faults", checks.count, checks.faults));
}

// a rail of a track at its cuts, with the head of its profile
struct cut_rail
{
	TTrack const *track{nullptr};
	int path{0};
	int side{0};
	double head{0.0};
};

// the knots of a line
std::string knots_text(knots::line const &Line)
{
	std::string result;
	for (auto const &knot : Line)
		result += format(" %.3f:%.4f", knot.x, knot.y);
	return result;
}

// an end of a part of the rail at a cut, in the world
std::string cut_end_text(cut_rail const &Rail, double const At, knots::line const &Offsets)
{
	auto const point{track_rails::rail_point(*Rail.track, Rail.path, Rail.side, At, knots::sample(Offsets, At, 0.0) + 0.5 * Rail.head)};
	return format("%.3f m (%.3f %.3f) offset %.3f", At, point.x, point.z, knots::sample(Offsets, At, 0.0));
}

// a cut of the rail with the ends of its parts in the world, with Knots the lines of the parts too
void log_cut(cut_rail const &Rail, std::string const &Name, track_rails::rail_cut const &Cut, bool const Knots)
{
	WriteLog(format("SELFTEST cut %s path %d side %+d at %.3f: end %s%s, start %s%s, of %.3f m", Name.c_str(), Rail.path, Rail.side, Cut.at, cut_end_text(Rail, Cut.end, Cut.ending.offsets).c_str(), Cut.ending.extents.empty() ? "" : " extents", cut_end_text(Rail, Cut.start, Cut.starting.offsets).c_str(), Cut.starting.extents.empty() ? "" : " extents", track_rails::path_segment(*Rail.track, Rail.path).GetLength()));
	if (false == Knots)
		return;
	for (auto const &[label, line] : std::initializer_list<std::pair<char const *, knots::line const *>>{
		     {"endoffsets", &Cut.ending.offsets}, {"endextents", &Cut.ending.extents}, {"endinner", &Cut.ending.inner}, {"endrolled", &Cut.ending.rolled}, {"endworn", &Cut.ending.worn},
		     {"startoffsets", &Cut.starting.offsets}, {"startextents", &Cut.starting.extents}, {"startinner", &Cut.starting.inner}, {"startrolled", &Cut.starting.rolled}, {"startworn", &Cut.starting.worn}})
		if (false == line->empty())
			WriteLog(format("SELFTEST   %s%s", label, knots_text(*line).c_str()));
}

// cuts <track> [knots]: the cuts of the rails of a track, with the ends of the parts in the world; with knots the lines of the parts too
void selftest_cuts(std::istringstream &Words)
{
	std::string name;
	std::string detail;
	Words >> name >> detail;
	auto *track{simulation::Paths.find(name)};
	if (track == nullptr)
		return;
	auto const layout{track->plan_rails(true)};
	auto const head{track->rail_dimensions().head};
	for (int path = 0; path < (track->eType == tt_Switch ? 2 : 1); ++path)
		for (int onright = 0; onright < 2; ++onright)
			for (auto const &cut : layout.plans[path][onright].cuts)
				log_cut({track, path, onright ? 1 : -1, head}, name, cut, detail == "knots");
}

using layout_map = std::map<TTrack const *, track_rails::rail_layout>;

// the layout of the rails of a track, laid out the first time it's asked for
track_rails::rail_layout const &layout_of(layout_map &Layouts, TTrack *Track)
{
	auto found{Layouts.find(Track)};
	if (found == Layouts.end())
		found = Layouts.try_emplace(Track, Track->plan_rails(true)).first;
	return found->second;
}

// whether a point out from the gauge line of a rail lies on the head of a part of it at a distance along it, of the head Head wide
bool part_holds(track_rails::rail_cut_part const &Part, double const Head, double const At, double const Lateral)
{
	auto const offset{knots::sample(Part.offsets, At, 0.0)};
	return Lateral >= offset + std::max(0.0, knots::sample(Part.inner, At, 0.0)) && Lateral <= offset + std::min(Head, knots::sample(Part.extents, At, Head));
}

// whether the head of a rail lies at a distance along it and out from its gauge line, the part of the nearest cut there
bool head_occupied(track_rails::rail_plan const &Plan, double const Head, double const At, double const Lateral)
{
	if (Plan.cuts.empty())
		return Lateral >= 0.0 && Lateral <= Head;
	auto const *cut{&Plan.cuts.front()};
	for (auto const &each : Plan.cuts)
		if (std::abs(each.at - At) < std::abs(cut->at - At))
			cut = &each;
	return (At <= cut->end && part_holds(cut->ending, Head, At, Lateral)) || (At >= cut->start && part_holds(cut->starting, Head, At, Lateral));
}

// a rail of a track whose parts bent out at the cuts are looked at against the heads of the rails around: the head of its profile, the length
// of its path, Quiet with only the faults logged
struct wing_rail
{
	TTrack const *track{nullptr};
	int path{0};
	int side{0};
	double head{0.0};
	double length{0.0};
	std::vector<TTrack *> const *others{nullptr};
	layout_map *layouts{nullptr};
	bool quiet{false};
};

// the rails of another track whose heads a bent part lies over at a point of it, each put into Hits once
void wing_hits_on(std::string &Hits, wing_rail const &Rail, TTrack *Other, glm::dvec3 const &Point, double const At)
{
	for (int otherpath = 0; otherpath < (Other->eType == tt_Switch ? 2 : 1); ++otherpath)
		for (auto const otherside : {-1, 1})
		{
			if (Other == Rail.track && otherpath == Rail.path && otherside == Rail.side)
				continue;
			double d{0.0};
			double l{0.0};
			track_rails::rail_offset(*Other, otherpath, otherside, Point, d, l);
			if (d < 0.0 || d > path_length(*Other, otherpath) || l < -0.3 || l > 0.3)
				continue;
			// a wing rail planed flush against another head from the outside covers its rounded shoulder
			if (auto const theirs{Other->rail_dimensions()}; false == head_occupied(layout_of(*Rail.layouts, Other).plans[otherpath][otherside > 0], theirs.head - theirs.shoulder, d, l))
				continue;
			auto const hit{format(" %s/%d%+d", Other->name().c_str(), otherpath, otherside)};
			if (Hits.find(hit) == std::string::npos)
				Hits += hit + format(" at %.2f m (theirs %.2f m, %.0f mm in)", At, d, l * 1000.0);
		}
}

// the rails around whose heads a part of the rail bent out at a cut lies over, along it
std::string wing_hits(wing_rail const &Rail, track_rails::rail_cut_part const &Part, double const From, double const To)
{
	std::string hits;
	for (int k = 0; From + k * 0.02 <= To; ++k)
	{
		auto const at{From + k * 0.02};
		auto const offset{knots::sample(Part.offsets, at, 0.0)};
		if (offset <= 0.002)
			continue;
		// only what is left of the head where the wing rail is planed
		auto const lo{offset + std::max(0.0, knots::sample(Part.inner, at, 0.0))};
		auto const hi{offset + std::min(Rail.head, knots::sample(Part.extents, at, Rail.head))};
		if (hi - lo < 0.012)
			continue;
		for (auto const lateral : {lo + 0.005, 0.5 * (lo + hi), hi - 0.005})
		{
			auto const point{track_rails::rail_point(*Rail.track, Rail.path, Rail.side, at, lateral)};
			for (auto *other : *Rail.others)
				wing_hits_on(hits, Rail, other, point, at);
		}
	}
	return hits;
}

// a part of the rail bent out at a cut, coming to it or going on from it, against the heads of the rails around
void log_wing_part(wing_rail const &Rail, track_rails::rail_cut const &Cut, bool const Coming, fault_count &Tally)
{
	auto const &part{Coming ? Cut.ending : Cut.starting};
	auto const &offsets{part.offsets};
	if (offsets.size() < 2 || offsets.back().x - offsets.front().x < 0.05)
		return;
	auto const from{std::max(0.0, Coming ? offsets.front().x : std::max(offsets.front().x, Cut.start))};
	auto const to{std::min(Rail.length, Coming ? std::min(offsets.back().x, Cut.end) : offsets.back().x)};
	if (to - from < 0.05)
		return;
	++Tally.count;
	auto const hits{wing_hits(Rail, part, from, to)};
	if (false == hits.empty())
		++Tally.faults;
	if (false == hits.empty() || false == Rail.quiet)
		WriteLog(format("SELFTEST wing %s path %d side %+d cut at %.3f, %s part %.3f-%.3f m, out %.0f-%.0f mm%s", Rail.track->name().c_str(), Rail.path, Rail.side, Cut.at, Coming ? "coming" : "going", from, to, offsets.front().y * 1000.0,
		                offsets.back().y * 1000.0, hits.empty() ? ", clear" : (", over" + hits + " FAULT").c_str()));
}

// the parts of the rail coming to a cut and going on from it
void log_wing_cut(wing_rail const &Rail, track_rails::rail_cut const &Cut, fault_count &Tally)
{
	for (auto const coming : {true, false})
		log_wing_part(Rail, Cut, coming, Tally);
}

// the parts of the rails of a track bent out at the cuts
void log_wings_of(TTrack *Track, layout_map &Layouts, bool const Quiet, fault_count &Tally)
{
	auto const &layout{layout_of(Layouts, Track)};
	auto const head{Track->rail_dimensions().head};
	auto const others{tracks_around(*Track, with_paths)};
	for (int path = 0; path < (Track->eType == tt_Switch ? 2 : 1); ++path)
		for (int onright = 0; onright < 2; ++onright)
		{
			wing_rail const rail{Track, path, onright ? 1 : -1, head, path_length(*Track, path), &others, &Layouts, Quiet};
			for (auto const &cut : layout.plans[path][onright].cuts)
				log_wing_cut(rail, cut, Tally);
		}
}

// wings <track|all>: the heads of the rails bent out at the cuts, the wing rails mostly, against the heads of the other rails as they're laid out
void selftest_wings(std::istringstream &Words)
{
	std::string name;
	Words >> name;
	layout_map layouts;
	fault_count wings;
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == live_railway(track) || track->m_paths.empty())
			continue;
		if (name != "all" && track->name() != name)
			continue;
		log_wings_of(track, layouts, name == "all", wings);
	}
	WriteLog(format("SELFTEST wings: %d bent parts, %d faults", wings.count, wings.faults));
}

// a part of a rail between two of its cuts, or a cut and an end
struct rail_piece
{
	double from{-1e9};
	double to{1e9};
	track_rails::rail_cut const *before{nullptr};
	track_rails::rail_cut const *after{nullptr};
};

// a rail of a track as laid out: from how far along it it's laid past the blade, the heel of the blades of a switch, its parts
struct laid_rail
{
	TTrack const *track{nullptr};
	int path{0};
	int side{0};
	double length{0.0};
	double laidfrom{0.0};
	double heel{0.0};
	track_rails::rail_size size;
	track_rails::gauge_line const *line{nullptr};
	std::vector<rail_piece> pieces;
};

// the layouts of the rails of the tracks, and their rails laid out
struct laid_rails
{
	layout_map layouts;
	std::map<TTrack const *, std::vector<laid_rail>> rails;
};

// the parts of a rail between its cuts, the cuts taken by where along the rail they lie
std::vector<rail_piece> rail_pieces(track_rails::rail_plan const &Plan)
{
	std::vector<track_rails::rail_cut const *> cuts;
	for (auto const &cut : Plan.cuts)
		cuts.push_back(&cut);
	std::ranges::stable_sort(cuts, [](track_rails::rail_cut const *A, track_rails::rail_cut const *B) { return A->at < B->at; });
	std::vector<rail_piece> pieces;
	for (std::size_t j = 0; j <= cuts.size(); ++j)
	{
		rail_piece part;
		part.before = (j > 0 ? cuts[j - 1] : nullptr);
		part.after = (j < cuts.size() ? cuts[j] : nullptr);
		if (part.before != nullptr)
			part.from = part.before->start;
		if (part.after != nullptr)
			part.to = part.after->end;
		pieces.push_back(part);
	}
	return pieces;
}

// from how far along the rail of a path it's laid: past the blade for the rail the blade of the path of a switch lies against
double laid_from(TTrack const &Track, int const Path, int const Side)
{
	if (Track.eType != tt_Switch)
		return 0.0;
	auto const bladeside{Track.SwitchExtension->RightSwitch == (Path == 0) ? 1 : -1};
	return Side == bladeside ? track_rails::blade_length(Track, Path) : 0.0;
}

// the rails of the track as laid out
std::vector<laid_rail> lay_out(TTrack const &Track, track_rails::rail_layout const &Layout)
{
	std::vector<laid_rail> result;
	auto const heel{Track.eType == tt_Switch ? std::max(track_rails::blade_length(Track, 0), track_rails::blade_length(Track, 1)) : 0.0};
	for (int path = 0; path < track_rails::path_count(Track); ++path)
		for (int onright = 0; onright < 2; ++onright)
		{
			laid_rail each;
			each.track = &Track;
			each.path = path;
			each.side = (onright ? 1 : -1);
			each.length = track_rails::path_segment(Track, path).GetLength();
			each.laidfrom = laid_from(Track, path, each.side);
			each.heel = heel;
			each.size = Track.rail_dimensions();
			each.line = &track_rails::rail_line(Track, path, onright, false);
			each.pieces = rail_pieces(Layout.plans[path][onright]);
			result.push_back(each);
		}
	return result;
}

// the rails of the track as laid out, laid out the first time they're asked for
std::vector<laid_rail> const &rails_of(laid_rails &Laid, TTrack *Track)
{
	if (auto const found{Laid.rails.find(Track)}; found != Laid.rails.end())
		return found->second;
	auto &result{Laid.rails[Track]};
	auto const &layout{Laid.layouts.try_emplace(Track, Track->plan_rails(true)).first->second};
	result = lay_out(*Track, layout);
	return result;
}

// the lines of a part of a rail
struct part_lines
{
	knots::line const *offsets;
	knots::line const *extents;
	knots::line const *inner;
};

// the lines of the part from the nearer of its cuts
part_lines lines_at(rail_piece const &Part, double const S)
{
	static knots::line const no_line;
	if (Part.before != nullptr && (Part.after == nullptr || std::abs(S - Part.before->at) <= std::abs(S - Part.after->at)))
		return {&Part.before->starting.offsets, &Part.before->starting.extents, &Part.before->starting.inner};
	if (Part.after != nullptr)
		return {&Part.after->ending.offsets, &Part.after->ending.extents, &Part.after->ending.inner};
	return {&no_line, &no_line, &no_line};
}

// the head of a part of a rail at a distance along it: how far it's offset out from the gauge line, and how far its edges lie out
struct head_span
{
	double offset{0.0};
	double low{0.0};
	double high{0.0};
};

head_span head_at(laid_rail const &Rail, rail_piece const &Part, double const S)
{
	auto const l{lines_at(Part, S)};
	head_span head;
	head.offset = knots::sample(*l.offsets, S, 0.0);
	head.low = head.offset + std::max(0.0, knots::sample(*l.inner, S, 0.0));
	head.high = head.offset + std::min(Rail.size.head, knots::sample(*l.extents, S, Rail.size.head));
	return head;
}

// whether the head of a rail covers a point out from its gauge line, past its rounded shoulder and Tolerance
bool covers(laid_rail const &Rail, double const D, double const L, double const Tolerance)
{
	for (auto const &part : Rail.pieces)
	{
		if (D < part.from || D > part.to)
			continue;
		auto const head{head_at(Rail, part, D)};
		if (L > head.low + Tolerance && L < std::min(head.high, head.offset + Rail.size.head - Rail.size.shoulder) - Tolerance)
			return true;
	}
	return false;
}

// the parts near an obtuse crossing of either rail are designed to meet
bool obtuse_near(rail_piece const &Part, double const S)
{
	return std::ranges::any_of(std::array{Part.before, Part.after}, [S](track_rails::rail_cut const *Cut) { return Cut != nullptr && Cut->obtuse && std::abs(S - Cut->at) < 0.3; });
}

// whether a distance along a rail lies in a part of it near an obtuse crossing
bool in_obtuse(laid_rail const &Rail, double const D)
{
	return std::ranges::any_of(Rail.pieces, [D](rail_piece const &Part) { return D >= Part.from && D <= Part.to && obtuse_near(Part, D); });
}

bool throwable(rail_piece const &Part)
{
	return Part.before != nullptr && Part.before->throwable;
}

// the first fault of each kind found along a part of a rail
struct part_faults
{
	std::array<bool, 5> found{};
	std::array<std::string, 5> where;
};

void add_fault(part_faults &Faults, std::size_t const Kind, std::string_view const What)
{
	if (false == Faults.found[Kind])
	{
		Faults.found[Kind] = true;
		Faults.where[Kind] = What;
	}
}

// a part of one of our rails looked at, from where to where along the rail, and how far the heads may reach over each other
struct part_probe
{
	laid_rail const *ours{nullptr};
	rail_piece const *part{nullptr};
	double from{0.0};
	double to{0.0};
	double tolerance{0.0};
};

// past the last knots of its lines, the lines planing on as they end (C)
void find_planing_on(part_faults &Faults, part_probe const &Probe)
{
	for (int k = 0; Probe.from + k * 0.02 <= Probe.to + 1e-9; ++k)
	{
		auto const s{Probe.from + k * 0.02};
		auto const l{lines_at(*Probe.part, s)};
		for (auto const *line : {l.extents, l.inner})
		{
			if (line->empty())
				continue;
			auto const value{knots::sample(*line, s, 0.0)};
			auto const planing{line == l.extents ? value < Probe.ours->size.head - 1e-4 : value > 1e-4};
			// the lines have knots every 2 cm at most
			if (planing && (s > line->back().x + 0.025 || s < line->front().x - 0.025))
				add_fault(Faults, 2, format("%s at %.2f m, past its knots %.2f-%.2f m", line == l.extents ? "extents" : "inner", s, line->front().x, line->back().x));
		}
	}
}

// from how far along the part the head is looked at: past a sharp end before it, where enough of the head is left
double head_start(part_probe const &Probe)
{
	auto const start{Probe.from + 0.05};
	auto const &part{*Probe.part};
	if (part.before == nullptr || part.before->starting.kind != track_rails::rail_end::sharp)
		return start;
	for (int k = 0; Probe.from + k * 0.02 < Probe.to; ++k)
	{
		auto const s{Probe.from + k * 0.02};
		if (auto const head{head_at(*Probe.ours, part, s)}; head.high - head.low >= 0.001)
			return std::max(start, s + 0.04);
	}
	return start;
}

// along the part: nothing of the head left (V), the head jumping in width (J)
void find_narrowing(part_faults &Faults, part_probe const &Probe)
{
	auto const start{head_start(Probe)};
	auto const end{Probe.to - 0.05};
	std::vector<double> stations;
	std::vector<double> widths;
	auto const first{std::ceil(start / 0.02) * 0.02};
	for (int k = 0; first + k * 0.02 <= end; ++k)
	{
		auto const s{first + k * 0.02};
		auto const head{head_at(*Probe.ours, *Probe.part, s)};
		auto const width{std::max(0.0, head.high - head.low)};
		if (width < 0.001 && false == obtuse_near(*Probe.part, s))
			add_fault(Faults, 4, format("at %.2f m, %.1f mm left", s, width * 1000.0));
		stations.push_back(s);
		widths.push_back(width);
	}
	// a step in width well past those around it: a head tapering off steeply at a wide angle aside
	for (std::size_t i = 1; i < widths.size(); ++i)
	{
		auto const step{std::abs(widths[i] - widths[i - 1])};
		auto const around{std::max(i > 1 ? std::abs(widths[i - 1] - widths[i - 2]) : 0.0, i + 1 < widths.size() ? std::abs(widths[i + 1] - widths[i]) : 0.0)};
		if (step > 0.003 && step > 2.0 * around + 0.001 && false == obtuse_near(*Probe.part, stations[i]))
			add_fault(Faults, 3, format("at %.2f m, %.1f to %.1f mm", stations[i], widths[i - 1] * 1000.0, widths[i] * 1000.0));
	}
}

// whether their rail is ours going on past a joint, at a station of ours
bool past_joint(part_probe const &Probe, laid_rail const &Theirs, double const S, double const Distance)
{
	auto const &ours{*Probe.ours};
	auto d{Distance};
	return std::abs(track_rails::lateral(*Theirs.line, Theirs.side, track_rails::rail_point(*ours.track, ours.path, ours.side, S, 0.0), d)) < 0.002;
}

// points across the head of our part at a station of it against one of their rails: in their flangeway (F), over their head (O)
void find_overlaps_at(part_faults &Faults, part_probe const &Probe, laid_rail const &Theirs, double const S, head_span const &Head, double const Distance)
{
	auto const &ours{*Probe.ours};
	auto const tolerance{Probe.tolerance};
	// the head of ours keeps out of the flangeways, the top of it out from under other heads: their rounded shoulders lie under it, ours under
	// theirs
	auto const outer{Head.high - tolerance};
	auto const crown{std::min(Head.high, Head.offset + ours.size.head - ours.size.shoulder) - tolerance};
	for (auto const lateral : {Head.low + tolerance, 0.5 * (Head.low + crown), crown, outer})
	{
		auto const top{lateral <= crown + 1e-9};
		auto const point{track_rails::rail_point(*ours.track, ours.path, ours.side, S, lateral)};
		auto d{Distance};
		auto const l{track_rails::lateral(*Theirs.line, Theirs.side, point, d)};
		// rails meeting end on at joints aside
		if (d < 0.05 || d > Theirs.length - 0.05 || S < 0.05 || S > ours.length - 0.05)
			continue;
		if (l > -track_rails::flangeway + tolerance && l < -tolerance)
			add_fault(Faults, 0, format("at %.2f m, %.1f mm into the flangeway of %s/%d%+d at %.2f m", S, (-l) * 1000.0, Theirs.track->name().c_str(), Theirs.path, Theirs.side, d));
		if (top && false == obtuse_near(*Probe.part, S) && covers(Theirs, d, l, tolerance) && false == in_obtuse(Theirs, d))
			add_fault(Faults, 1, format("at %.2f m, over the head of %s/%d%+d at %.2f m, %.1f mm in", S, Theirs.track->name().c_str(), Theirs.path, Theirs.side, d, l * 1000.0));
	}
}

// our part against one of their rails, every 2 cm along it
void find_overlaps(part_faults &Faults, part_probe const &Probe, laid_rail const &Theirs)
{
	auto const &ours{*Probe.ours};
	if (Theirs.track == ours.track && Theirs.path == ours.path)
		return;
	auto const &low{Theirs.line->low};
	auto const &high{Theirs.line->high};
	auto distance{-1e9};
	auto const first{std::ceil(Probe.from / 0.02) * 0.02};
	for (int k = 0; first + k * 0.02 <= Probe.to; ++k)
	{
		auto const s{first + k * 0.02};
		auto const head{head_at(ours, *Probe.part, s)};
		if (head.high - head.low < 2.0 * Probe.tolerance)
			continue;
		auto const centre{track_rails::rail_point(*ours.track, ours.path, ours.side, s, head.offset + 0.5 * ours.size.head)};
		if (centre.x < low.x - 0.5 || centre.x > high.x + 0.5 || centre.z < low.y - 0.5 || centre.z > high.y + 0.5)
			continue;
		if (distance < -1e8)
			distance = track_rails::nearest_distance(*Theirs.line, centre);
		if (auto const across{track_rails::lateral(*Theirs.line, Theirs.side, centre, distance)}; distance < 0.0 || distance > Theirs.length || std::abs(across) > 0.3)
			continue;
		if (Theirs.track == ours.track && ((Theirs.side == ours.side && distance < Theirs.heel + 0.1) || throwable(*Probe.part)))
			continue;
		// the same rail going on past a joint
		if (past_joint(Probe, Theirs, s, distance))
			continue;
		find_overlaps_at(Faults, Probe, Theirs, s, head, distance);
	}
}

// what the space test keeps over the tracks: their rails laid out, the tolerance, the parts looked at and the faults of each kind
struct space_scan
{
	laid_rails laid;
	double tolerance{0.0};
	int parts{0};
	std::array<int, 5> counts{};
};

// the kinds of the faults of the space test, by their letters
std::string_view constexpr space_kinds{"FOCJV"};

// a part of one of our rails against itself and the rails around, the first fault of each kind found logged
void look_at_part(space_scan &Scan, laid_rail const &Ours, rail_piece const &Part, std::vector<TTrack *> const &Others)
{
	auto const from{std::max({Part.from, Ours.laidfrom, 0.0})};
	auto const to{std::min(Part.to, Ours.length)};
	if (to - from < 0.04)
		return;
	++Scan.parts;
	part_faults faults;
	part_probe const probe{&Ours, &Part, from, to, Scan.tolerance};
	find_planing_on(faults, probe);
	find_narrowing(faults, probe);
	// against the rails around
	for (auto *other : Others)
		for (auto const &theirs : rails_of(Scan.laid, other))
			find_overlaps(faults, probe, theirs);
	for (std::size_t kind = 0; kind < space_kinds.size(); ++kind)
		if (faults.found[kind])
		{
			++Scan.counts[kind];
			WriteLog(format("SELFTEST space %c %s/%d%+d part %.3f-%.3f m: %s", space_kinds[kind], Ours.track->name().c_str(), Ours.path, Ours.side, from, to, faults.where[kind].c_str()));
		}
}

// space [track|all] [tolerance mm]: the rails as laid out, every 2 cm along their parts: a head in the flangeway of another rail where its
// wheels run (F), over the head of another rail past its shoulder (O), a line of a part planing on past its last knot (C), the head jumping in
// width (J), nothing left of the head inside a part (V)
void selftest_space(std::istringstream &Words)
{
	std::string name;
	double tolerance{0.5};
	Words >> name >> tolerance;
	if (name.empty())
		name = "all";
	space_scan scan;
	scan.tolerance = tolerance * 0.001;
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == live_railway(track) || track->m_paths.empty() || false == track_rails::cut_rails(*track) || track->rail_dimensions().grooved)
			continue;
		if (name != "all" && track->name() != name)
			continue;
		auto const others{tracks_around(*track, with_cut_rails)};
		for (auto const &ours : rails_of(scan.laid, track))
			for (auto const &part : ours.pieces)
				look_at_part(scan, ours, part, others);
	}
	WriteLog(format("SELFTEST space: %d parts, F %d, O %d, C %d, J %d, V %d", scan.parts, scan.counts[0], scan.counts[1], scan.counts[2], scan.counts[3], scan.counts[4]));
}

// a rail of a track its guard rails lie beside: the laterals of the edges of the profile laid as a guard rail there, the length of the path,
// the tracks whose rails the rails of the track cross
struct guard_side
{
	TTrack const *track{nullptr};
	int path{0};
	int side{0};
	std::array<double, 2> laterals{};
	double length{0.0};
	std::vector<TTrack *> const *crossed{nullptr};
};

// the laterals of the edges of the profile laid as a guard rail: its face toward the running rail, its back
std::array<double, 2> guard_laterals(track_rails::rail_size const &Size)
{
	auto const offset{-(track_rails::flangeway + Size.head)};
	return {Size.low + offset, track_rails::flangeway + Size.head + Size.low - 0.002 + offset};
}

// whether the end of a guard rail cut along Miter, or square at Square, lies along the path
bool guard_drawn(guard_side const &Rail, glm::dvec2 const &Miter, double const Square)
{
	auto const at{Miter.y == 0.0 ? Square : Miter.x + Miter.y * 0.5 * (Rail.laterals[0] + Rail.laterals[1])};
	return at > 0.0 && at < Rail.length;
}

// how far along a guard rail with the edges at Laterals its end cut along Miter, or square at Square, reaches: by the latest of the edges or
// the earliest
double guard_reach(std::array<double, 2> const &Laterals, glm::dvec2 const &Miter, double const Square, bool const Latest)
{
	if (Miter.y == 0.0)
		return Square;
	auto const a{Miter.x + Miter.y * Laterals[0]};
	auto const b{Miter.x + Miter.y * Laterals[1]};
	return Latest ? std::max(a, b) : std::min(a, b);
}

// the rail of the tracks crossed the cut at an end of a guard rail runs along best, with the least skew
struct guard_cut
{
	TTrack *against{nullptr};
	int path{0};
	int side{0};
	double best{1e9};
	double lateral{0.0};
	double along{0.0};
};

// the rails of a track crossed against the best one the cut through the points of the edges of the guard rail runs along yet
void cut_along_rails_of(guard_cut &Best, TTrack *Other, std::array<glm::dvec3, 2> const &Points)
{
	for (int otherpath = 0; otherpath < (Other->eType == tt_Switch ? 2 : 1); ++otherpath)
		for (auto const otherside : {-1, 1})
		{
			std::array<double, 2> d{};
			std::array<double, 2> l{};
			track_rails::rail_offset(*Other, otherpath, otherside, Points[0], d[0], l[0]);
			track_rails::rail_offset(*Other, otherpath, otherside, Points[1], d[1], l[1]);
			if (std::abs(l[0]) > 0.5 || std::abs(l[1]) > 0.5 || std::abs(l[0] - l[1]) >= Best.best)
				continue;
			Best.best = std::abs(l[0] - l[1]);
			Best.lateral = 0.5 * (l[0] + l[1]);
			Best.along = 0.5 * (d[0] + d[1]);
			Best.against = Other;
			Best.path = otherpath;
			Best.side = otherside;
		}
}

// the rail of the tracks crossed the cut through the points of the edges of the guard rail runs along
guard_cut cut_along(std::vector<TTrack *> const &Crossed, std::array<glm::dvec3, 2> const &Points)
{
	guard_cut best;
	for (auto *other : Crossed)
		cut_along_rails_of(best, other, Points);
	return best;
}

// whether a guard rail of the rail the cut runs along lies where the cut is
bool on_guard_rail(guard_cut const &Cut)
{
	auto const theirlaterals{guard_laterals(Cut.against->rail_dimensions())};
	auto covered{false};
	// the plan kept while its guard rails are gone through: one made in the loop would go before them
	auto const plan{Cut.against->plan_rails(true)};
	for (auto const &theirs : plan.guards[Cut.path][Cut.side > 0])
		covered |= (Cut.along > guard_reach(theirlaterals, theirs.frommiter, theirs.from, false) - 0.002 && Cut.along < guard_reach(theirlaterals, theirs.tomiter, theirs.to, true) + 0.002);
	return covered;
}

// an end of a guard rail or of a break in it: square or planed, or cut along the rail of a track crossed; Glued for an end that goes on into
// the guard rail of that track
std::string guard_end(guard_side const &Rail, glm::dvec2 const &Miter, bool const Flared, bool const Glued, int &Faults)
{
	if (Miter.y == 0.0)
		return std::string{Flared ? "planed" : "square"};
	if (false == guard_drawn(Rail, Miter, 0.0))
		return std::string{"past the end of the track"};
	std::array<glm::dvec3, 2> const points{track_rails::rail_point(*Rail.track, Rail.path, Rail.side, Miter.x + Miter.y * Rail.laterals[0], Rail.laterals[0]), track_rails::rail_point(*Rail.track, Rail.path, Rail.side, Miter.x + Miter.y * Rail.laterals[1], Rail.laterals[1])};
	auto const cut{cut_along(*Rail.crossed, points)};
	if (cut.against == nullptr)
	{
		++Faults;
		return std::string{"cut along nothing FAULT"};
	}
	auto result{format("along %s/%d%+d at %.3f m, %.1f mm off its gauge line, skew %.1f mm", cut.against->name().c_str(), cut.path, cut.side, cut.along, cut.lateral * 1000.0, cut.best * 1000.0)};
	auto fault{cut.best > 0.002};
	if (Glued)
	{
		auto const covered{on_guard_rail(cut)};
		result += covered ? ", on its guard rail" : ", no guard rail there";
		fault |= (false == covered || std::abs(cut.lateral + track_rails::flangeway) > 0.002);
	}
	if (fault)
	{
		++Faults;
		result += " FAULT";
	}
	return result;
}

// the guard rails beside a rail of the track, each logged with how its ends and the ends of its breaks are cut
void log_guards(guard_side const &Rail, std::vector<track_rails::guard_rail> const &Guards, fault_count &Tally)
{
	for (auto const &guard : Guards)
	{
		if (false == guard_drawn(Rail, guard.frommiter, guard.from) && false == guard_drawn(Rail, guard.tomiter, guard.to) && (guard.to <= 0.0 || guard.from >= Rail.length))
			continue;
		++Tally.count;
		std::string text;
		text += "from " + guard_end(Rail, guard.frommiter, guard.flarefrom, false == guard.flarefrom, Tally.faults) + "; to " + guard_end(Rail, guard.tomiter, guard.flareto, false == guard.flareto, Tally.faults);
		for (auto const &cut : guard.plan.cuts)
			if (cut.end > 0.0 && cut.start < Rail.length)
				text += format("; break %.3f-%.3f: ", cut.end, cut.start) + guard_end(Rail, cut.ending.miter, false, false, Tally.faults) + " / " + guard_end(Rail, cut.starting.miter, false, false, Tally.faults);
		WriteLog(format("SELFTEST guard %s path %d side %+d %.3f-%.3f m: %s", Rail.track->name().c_str(), Rail.path, Rail.side, guard.from, guard.to, text.c_str()));
	}
}

// guards [track]: the guard rails of the tracks, how their ends are cut along the rails they're cut against
void selftest_guards(std::istringstream &Words)
{
	std::string name;
	Words >> name;
	fault_count guards;
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == live_railway(track) || false == named(name, *track))
			continue;
		auto const layout{track->plan_rails(true)};
		auto const laterals{guard_laterals(track->rail_dimensions())};
		for (int path = 0; path < 2; ++path)
			for (int onright = 0; onright < 2; ++onright)
				log_guards({track, path, onright ? 1 : -1, laterals, path_length(*track, path), &layout.crossed}, layout.guards[path][onright], guards);
	}
	WriteLog(format("SELFTEST guards: %d guard rails, %d faults", guards.count, guards.faults));
}

// throw <path> [part] [name]: the switches set over to the path, by a part of the way as in the middle of the animation
void selftest_throw(std::istringstream &Words)
{
	int path{0};
	double part{1.0};
	std::string name;
	Words >> path;
	if (false == static_cast<bool>(Words >> part))
		part = 1.0;
	Words >> name;
	int count{0};
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == live_switch(track) || false == named(name, *track))
			continue;
		auto const before{track->SwitchExtension->fOffset};
		track->Switch(path);
		track->SwitchExtension->fOffset = before + static_cast<float>(part) * (track->SwitchExtension->fDesiredOffset - before);
		track->RaAnimate();
		++count;
	}
	WriteLog(format("SELFTEST switches: %d switches set to path %d", count, path));
}

// railtime: how long the rails of all the tracks take to lay out, and their crossings to find
void selftest_railtime(std::istringstream &)
{
	track_rails::forget();
	auto const started{std::chrono::steady_clock::now()};
	int tracks{0};
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == live_railway(track))
			continue;
		track->rail_crossings(false);
		++tracks;
	}
	auto const crossed{std::chrono::steady_clock::now()};
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == live_railway(track))
			continue;
		track->plan_rails(true);
	}
	auto const finished{std::chrono::steady_clock::now()};
	auto const crossings{std::chrono::duration<double, std::milli>(crossed - started).count()};
	auto const elapsed{std::chrono::duration<double, std::milli>(finished - started).count()};
	WriteLog(format("SELFTEST railtime: the rails of %d tracks laid out in %.1f ms, of which their crossings %.1f ms", tracks, elapsed, crossings));
}

// the point on the gauge line of the rail at a crossing, in the world
glm::dvec3 crossing_gauge(TTrack const *Track, track_rails::rail_crossing const &Crossing)
{
	auto const &segment{Track->eType == tt_Switch ? *Track->SwitchExtension->Segments[Crossing.path] : *Track->Segment};
	glm::dvec3 point;
	glm::dvec3 across;
	segment.plan_frame(Crossing.at, point, across);
	return point + across * (Crossing.side * 0.5 * std::abs(Track->fTrackWidth));
}

// what a crossing of the rails is, by its angle and how the rails face each other
char const *crossing_kind(track_rails::rail_crossing const &Crossing, bool const Middle)
{
	if (Crossing.angle > track_rails::wide_angle)
		return "wide";
	if (Crossing.facing < 0.0)
		return Middle ? "middle acute" : "acute";
	return Middle ? "middle obtuse" : "obtuse";
}

// an acute crossing of a rail of the track against the crossings of the other track: the one of them at the same point, the nose of the two
void log_acute(TTrack const *Track, track_rails::rail_crossing const &Crossing, int &Mismatched)
{
	auto const here{crossing_gauge(Track, Crossing)};
	auto partners{0};
	auto noses{Crossing.nose ? 1 : 0};
	for (auto const &other : Crossing.track->rail_crossings(false))
	{
		if (other.track != Track || other.angle > track_rails::wide_angle || other.facing >= 0.0)
			continue;
		if (auto const there{crossing_gauge(Crossing.track, other)}; glm::length(glm::dvec2{there.x - here.x, there.z - here.z}) > 0.05)
			continue;
		++partners;
		noses += other.nose ? 1 : 0;
	}
	if (partners != 1 || noses != 1)
		++Mismatched;
	WriteLog(format("SELFTEST acute %s %s path %d side %d at %.3f m with %s (%.2f %.2f %.2f): partners %d, noses %d", partners != 1 || noses != 1 ? "mismatch" : "ok", Track->name().c_str(), Crossing.path, Crossing.side, Crossing.at, Crossing.track->name().c_str(), here.x, here.y, here.z, partners, noses));
}

// crossings [track]: the crossings of the rails of the tracks, and the acute ones against those of the other tracks
void selftest_crossings(std::istringstream &Words)
{
	std::string name;
	Words >> name;
	auto const started{std::chrono::steady_clock::now()};
	int tracks{0};
	int found{0};
	int acutes{0};
	int middles{0};
	int mismatched{0};
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == live_railway(track) || false == named(name, *track))
			continue;
		++tracks;
		for (auto const &crossing : track->rail_crossings())
		{
			++found;
			auto const middle{crossing.angle > track_rails::middle_angle && crossing.angle <= track_rails::wide_angle};
			middles += middle ? 1 : 0;
			auto const kind{crossing_kind(crossing, middle)};
			auto const where{crossing_gauge(track, crossing)};
			WriteLog(format("SELFTEST crossing %s path %d side %d at %.3f m: %s with %s, 1:%.2f, into %.3f, facing %.3f, nose %d, plan %.2f %.2f", track->name().c_str(), crossing.path, crossing.side, crossing.at, kind,
			    crossing.track->name().c_str(), 1.0 / std::tan(crossing.angle), crossing.into, crossing.facing, crossing.nose ? 1 : 0, where.x, where.z));
		}
		for (auto const &crossing : track->rail_crossings(false))
		{
			if (crossing.angle > track_rails::wide_angle || crossing.facing >= 0.0)
				continue;
			++acutes;
			log_acute(track, crossing, mismatched);
		}
	}
	auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count()};
	WriteLog(format("SELFTEST crossings: %d rail crossings on %d tracks, %d of them middle, %d acute, %d mismatched, %.1f ms", found, tracks, middles, acutes, mismatched, elapsed));
}

// the commands of the group which need nothing of the editor
struct rails_command
{
	std::string_view name;
	void (*run)(std::istringstream &);
};

auto constexpr rails_commands{std::to_array<rails_command>({
    {"blade", selftest_blade},
    {"frog", selftest_frog},
    {"beddump", selftest_beddump},
    {"raildump", selftest_raildump},
    {"geomhash", selftest_geomhash},
    {"headgap", selftest_headgap},
    {"checks", selftest_checks},
    {"cuts", selftest_cuts},
    {"wings", selftest_wings},
    {"space", selftest_space},
    {"guards", selftest_guards},
    {"throw", selftest_throw},
    {"railtime", selftest_railtime},
    {"crossings", selftest_crossings},
})};

} // namespace

editor_mode::selftest_result editor_mode::selftest_rails(std::string const &command, std::istringstream &words) // NOSONAR: every group of the self-test commands has this type, selftest_dispatch keeps them all in one table
{
	if (auto const found{std::ranges::find(rails_commands, std::string_view{command}, &rails_command::name)}; found != rails_commands.end())
	{
		found->run(words);
		return selftest_result::handled;
	}
	if (command == "switchtimbers")
	{
		std::string model;
		words >> model;
		if (model == "none")
			model.clear();
		WriteLog(format("SELFTEST switchtimbers %s: %zu tracks", model.c_str(), set_switch_timbers(model)));
		return selftest_result::handled;
	}
	if (command == "frogkey")
	{
		selftest_frogkey(words, [this](TTrack *Track, editor_track::state const &Before) { push_track_snapshot({{Track, Before}}); });
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}
