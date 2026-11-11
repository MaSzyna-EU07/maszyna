/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/deer.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <format>
#include <iterator>
#include <numbers>
#include <string>
#include <string_view>
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "scene/scene.h"
#include "scene/scenenode.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "rendering/renderer.h"
#include "vehicle/DynObj.h"
#include "McZapkie/MOVER.h"
#include "world/Track.h"
#include "utilities/Timer.h"
#include "utilities/Logs.h"

namespace
{

// the animations of the deer (textures of the vertices, shader vat with vat_random -1: the game picks the clip and the moment of every
// instance, see vat_turn()). made by jelen_anim2.py; the numbers below are the ones it bakes with
struct vat_clip
{
	std::string_view model; // models/ and the extension left out
	double fps; // frames a second
	double frames; // of one clip
	int clips;
};
// grazing, 12 s: the head at the grass, raised about 4.5-9.3 s
constexpr vat_clip Grazing{"animals/jelen_je", 20.0, 240.0, 1};
// startled: from the moments k * Window of the grazing (clip k) to the alarm, in Startletime; two more frames hold the end
constexpr vat_clip Startle{"animals/jelen_zryw", 30.0, 15.0, 16};
// the flight from the alarm: looks at the train (clip k: the head turned by Looks[k] degrees, to the left when more than 0), bounds
// away and stops in the grazing pose of the moment Grazeafter, in Flighttime; two more frames hold the end
constexpr vat_clip Flight{"animals/jelen_skok", 30.0, 168.0, 6};
constexpr std::array<double, 6> Looks{50.0, 80.0, 110.0, -50.0, -80.0, -110.0};
constexpr double Window{0.75};
constexpr double Startletime{0.4};
constexpr double Flighttime{5.5};
constexpr double Grazeafter{5.6};
// the way it covers in the flight: the speed rises from 0 to Flightspeed in Flight0-Flight1 and falls back in Flight2-Flight3
// (smoothstep), as the animation moves its hooves
constexpr double Flightspeed{7.0};
constexpr double Flight0{0.80};
constexpr double Flight1{1.20};
constexpr double Flight2{3.95};
constexpr double Flight3{4.85};
// the frame an instance starts from at the moment of the change, a little after the frame proper: never the end of the one before
constexpr double Startframe{0.02};
// the ground under the way of the deer, at equal steps from its start to its end (deer_herd::Groundsamples)
constexpr std::size_t Groundsamples{9};
// the ground under the way of the deer is looked for up to this much above the place it starts from, metres
constexpr double Groundceiling{3.0};
// the way it runs: flat ground found all along it (no slope of an embankment or a cutting, no water). the ground beside the
// track no farther than this below or above the sleepers, metres
constexpr double Waybelow{1.2};
constexpr double Wayabove{0.8};
// rising or falling at most this much between the samples of the ground (3 m apart), and this much all along, metres
constexpr double Waystep{0.45};
constexpr double Wayrange{1.2};
// where there's no such ground at the place, the next deer is looked for this much farther, metres
constexpr double Retrydistance{200.0};
// the track at the place: plain (no switch) from the train to this much past the place, metres
constexpr double Clearpast{100.0};
// no deer this near a platform (the reach of its model added): an instance of a model with peron in its path (peronowe/...), metres
constexpr double Platformrange{100.0};
// the platforms are looked for among the instances placed this near, metres (the longest platform model, 400 m, by its middle)
constexpr double Platformsearch{350.0};
// nor where the ground beside the track, this far from its axis, rises this much over the sleepers (a platform of the terrain)
constexpr std::array<double, 2> Platformside{2.5, 4.0};
constexpr double Platformrise{0.45};
// the train past the deer: it came this close, and is this much farther away now, metres
constexpr double Passedclose{300.0};
constexpr double Passedmargin{200.0};

double fraction(double const Value)
{
	return Value - std::floor(Value);
}

// the integral of smoothstep((T - A) / (B - A)) from minus infinity to T
double ramp_integral(double const T, double const A, double const B)
{
	if (T <= A)
	{
		return 0.0;
	}
	if (T >= B)
	{
		return (B - A) * 0.5 + (T - B);
	}
	auto const x{(T - A) / (B - A)};
	return (B - A) * (x * x * x - 0.5 * x * x * x * x);
}

// metres covered in the flight Time seconds after its start
double flight_way(double const Time)
{
	return Flightspeed * (ramp_integral(Time, Flight0, Flight1) - ramp_integral(Time, Flight2, Flight3));
}

// the height of the triangle at (X, Z), false when the point isn't in it
bool triangle_height(glm::dvec3 const &A, glm::dvec3 const &B, glm::dvec3 const &C, double const X, double const Z, double &Height)
{
	auto const d{(B.z - C.z) * (A.x - C.x) + (C.x - B.x) * (A.z - C.z)};
	if (std::abs(d) < 1e-9)
	{
		return false;
	}
	auto const a{((B.z - C.z) * (X - C.x) + (C.x - B.x) * (Z - C.z)) / d};
	auto const b{((C.z - A.z) * (X - C.x) + (A.x - C.x) * (Z - C.z)) / d};
	auto const c{1.0 - a - b};
	if ((a < 0.0) || (b < 0.0) || (c < 0.0))
	{
		return false;
	}
	Height = a * A.y + b * B.y + c * C.y;
	return true;
}

// a shape of water: its material named so (water, woda), or with the shader of water
bool is_water(scene::shape_node const &Shape)
{
	if (GfxRenderer == nullptr)
	{
		return false;
	}
	auto const *material{GfxRenderer->Material(Shape.data().material)};
	if (material == nullptr)
	{
		return false;
	}
	auto name{material->GetName()};
	std::transform(name.begin(), name.end(), name.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	return (name.find("water") != std::string::npos) || (name.find("woda") != std::string::npos) || (name.find("wody") != std::string::npos);
}

// what's looked for under a point: its place, the highest ground found so far
struct ground_probe
{
	glm::dvec3 point{0.0};
	double ceiling{0.0};
	bool found{false};
	double height{0.0};
	bool water{false}; // the highest surface found is water
	bool shape_water{false}; // of the shape looked through

	void triangle(glm::dvec3 const &A, glm::dvec3 const &B, glm::dvec3 const &C)
	{
		if (double y{0.0}; triangle_height(A, B, C, point.x, point.z, y) && (y <= ceiling) && ((false == found) || (y > height)))
		{
			found = true;
			height = y;
			water = shape_water;
		}
	}

	void geometry(gfx::vertex_array const &Vertices, gfx::index_array const &Indices, glm::dvec3 const &Origin)
	{
		auto const vertex{[&Vertices, &Origin](std::size_t const Index) { return Origin + glm::dvec3{Vertices[Index].position}; }};
		if (Indices.empty())
		{
			for (std::size_t index = 0; index + 2 < Vertices.size(); index += 3)
			{
				triangle(vertex(index), vertex(index + 1), vertex(index + 2));
			}
			return;
		}
		for (std::size_t index = 0; index + 2 < Indices.size(); index += 3)
		{
			triangle(vertex(Indices[index]), vertex(Indices[index + 1]), vertex(Indices[index + 2]));
		}
	}

	void shape(scene::shape_node const &Shape)
	{
		auto const &data{Shape.data()};
		if ((data.area.radius >= 0.f) && (glm::length(glm::dvec2{data.area.center.x - point.x, data.area.center.z - point.z}) > data.area.radius))
		{
			return;
		}
		shape_water = is_water(Shape);
		if (false == data.vertices.empty())
		{
			for (std::size_t index = 0; index + 2 < data.vertices.size(); index += 3)
			{
				triangle(data.vertices[index].position, data.vertices[index + 1].position, data.vertices[index + 2].position);
			}
			return;
		}
		// uploaded, the renderer keeps the geometry relative to the origin of the shape
		if ((GfxRenderer == nullptr) || ((data.geometry.bank == 0) && (data.geometry.chunk == 0)))
		{
			return;
		}
		geometry(GfxRenderer->Vertices(data.geometry), GfxRenderer->Indices(data.geometry), data.origin);
	}
};

// the ground of the scenery (its shapes: the terrain) under a point
struct ground
{
	bool found{false}; // none: the ground of terrain models, or none loaded
	double height{0.0};
	bool water{false};
};

// the ground under the point, not higher than Ceiling
ground ground_at(glm::dvec3 const &Point, double const Ceiling)
{
	ground result;
	if (simulation::Region == nullptr)
	{
		return result;
	}
	ground_probe probe;
	probe.point = Point;
	probe.ceiling = Ceiling;
	for (auto *section : simulation::Region->sections(Point, 1.f))
	{
		for (auto const &shape : section->m_shapes)
		{
			probe.shape(shape);
		}
		for (auto const &cell : section->m_cells)
		{
			if (glm::length(glm::dvec2{cell.m_area.center.x - Point.x, cell.m_area.center.z - Point.z}) > cell.m_area.radius)
			{
				continue;
			}
			for (auto const &shape : cell.m_shapesopaque)
			{
				probe.shape(shape);
			}
			for (auto const &shape : cell.m_shapestranslucent)
			{
				probe.shape(shape);
			}
		}
	}
	result.found = probe.found;
	result.height = probe.height;
	result.water = probe.water;
	return result;
}

// a place on a path: the point of its axis and the way along it, level, of length 1
struct path_place
{
	glm::dvec3 position{0.0};
	glm::dvec3 forward{0.0, 0.0, 1.0};
};

// the place Distance from Track at Offset along it, going the way of Direction (1 towards its end, -1 towards its start), through the
// connected paths. false where the paths end earlier, or reach a crossing of roads or a switch
bool path_ahead(TTrack *Track, double Offset, double Direction, double Distance, path_place &Place)
{
	auto *track{Track};
	auto left{Distance};
	// as far as a train could go; a loop of paths ends it as well
	for (int step = 0; (step < 10000) && (track != nullptr); ++step)
	{
		auto const length{track->Length()};
		auto const room{Direction > 0.0 ? length - Offset : Offset};
		if (left > room)
		{
			left -= room;
			track = track->Connected(Direction > 0.0 ? 1 : -1, Direction);
			if ((track == nullptr) || (track->eType != tt_Normal))
			{
				// the end, a crossing of roads, or a switch: the way may go elsewhere by the time the train gets there
				return false;
			}
			Offset = (Direction > 0.0 ? 0.0 : track->Length());
			continue;
		}
		auto const segment{track->CurrentSegment()};
		if (segment == nullptr)
		{
			return false;
		}
		auto const at{std::clamp(Offset + Direction * left, 0.0, length)};
		glm::vec3 rotation;
		segment->RaPositionGet(at, Place.position, rotation);
		// the way of the path from two points about the place
		glm::dvec3 behind;
		glm::dvec3 ahead;
		segment->RaPositionGet(std::clamp(at - 0.5, 0.0, length), behind, rotation);
		segment->RaPositionGet(std::clamp(at + 0.5, 0.0, length), ahead, rotation);
		auto way{(ahead - behind) * Direction};
		way.y = 0.0;
		if (glm::length(way) < 1e-6)
		{
			return false;
		}
		Place.forward = glm::normalize(way);
		// on the sleepers, below the heads of the rails
		Place.position.y -= track->rail_height();
		return true;
	}
	return false;
}

// the way ahead of the train to Distance and Clearpast past it plain track (straight or curved, no switch): Place the place
bool plain_way(TTrack *Track, double const Offset, double const Direction, double const Distance, path_place &Place)
{
	if ((Track == nullptr) || (Track->eType != tt_Normal))
	{
		return false;
	}
	path_place past;
	return path_ahead(Track, Offset, Direction, Distance + Clearpast, past) && path_ahead(Track, Offset, Direction, Distance, Place);
}

// a model of a platform or of what stands on one: peron in the name of its file or of its folder (peronowe/)
bool platform_model(TAnimModel const &Instance)
{
	auto const *model{Instance.Model()};
	if (model == nullptr)
	{
		return false;
	}
	auto name{model->NameGet()};
	std::transform(name.begin(), name.end(), name.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	return name.find("peron") != std::string::npos;
}

// a platform about the place: a model of one near it, or the ground beside the track raised to its height
bool near_platform(path_place const &Place)
{
	for (auto *section : simulation::Region->sections(Place.position, static_cast<float>(Platformsearch)))
	{
		for (auto const &cell : section->m_cells)
		{
			if (glm::length(cell.m_area.center - Place.position) > cell.m_area.radius + Platformsearch)
			{
				continue;
			}
			// a long platform (a model of 400 m) is placed by its middle: the reach of the model counts as well
			auto const nearby{[&Place](TAnimModel *Instance) {
				return (Instance != nullptr) && (glm::length(Instance->location() - Place.position) < Platformrange + Instance->radius()) &&
					   platform_model(*Instance);
			}};
			if (std::ranges::any_of(cell.m_instancesopaque, nearby) || std::ranges::any_of(cell.m_instancetranslucent, nearby))
			{
				return true;
			}
		}
	}
	glm::dvec3 const across{Place.forward.z, 0.0, -Place.forward.x};
	for (auto const side : {-1.0, 1.0})
	{
		for (auto const distance : Platformside)
		{
			auto const here{ground_at(Place.position + across * (side * distance), Place.position.y + Groundceiling)};
			if (here.found && (here.height > Place.position.y + Platformrise))
			{
				return true;
			}
		}
	}
	return false;
}

// the clip of the flight which looks the nearest to Towards (from the deer to the train) for a deer looking along Heading
int look_clip(glm::dvec3 const &Heading, glm::dvec3 const &Towards)
{
	// the left of a heading h is (h.z, 0, -h.x)
	auto const left{Towards.x * Heading.z - Towards.z * Heading.x};
	auto const ahead{Towards.x * Heading.x + Towards.z * Heading.z};
	auto const angle{std::atan2(left, ahead) * 180.0 / std::numbers::pi};
	auto const nearest{std::ranges::min_element(Looks, {}, [angle](double const Look) { return std::abs(Look - angle); })};
	return static_cast<int>(std::distance(Looks.begin(), nearest));
}

// the yaw of an instance whose model looks along -z, to look along Heading, degrees (-180, 180]
double model_yaw(glm::dvec3 const &Heading)
{
	return std::atan2(-Heading.x, -Heading.z) * 180.0 / std::numbers::pi;
}

// the turn about the vertical (degrees) for an instance looking along Heading, which plays clip Clip of Animation so that it's at
// frame Frame at the time Time. the whole tenths of a degree come from the heading, the fraction of the tenth tells the shader the
// clip and the moment
float vat_turn(glm::dvec3 const &Heading, vat_clip const &Animation, int const Clip, double const Frame, double const Time)
{
	auto const phase{fraction((Frame - Time * Animation.fps) / Animation.frames)};
	auto const spin{(static_cast<double>(Clip) + 0.25 + 0.5 * phase) / static_cast<double>(Animation.clips)};
	return static_cast<float>((std::floor(model_yaw(Heading) * 10.0) + spin) / 10.0);
}

// a deer put in the scenery, not a part of it. Moving: the game moves it, so it's drawn on its own
TAnimModel *make_deer(vat_clip const &Animation, glm::dvec3 const &Position, float const Turn, bool const Moving)
{
	auto const range{static_cast<int>(std::ceil(Global.deer_distance + 500.0))};
	auto const definition{std::format("node {} 0 none model 0 0 0 0 {}.t3d none endmodel", range, Animation.model)};
	auto *model{simulation::State.create_model(definition, "none", Position)};
	if (model == nullptr)
	{
		return nullptr;
	}
	// not a part of the scenery: no layer of it, not saved, not edited
	scene::Layers.count(model->layer(), scene::layer_item::model, -1);
	model->layer(null_handle);
	model->m_dirty = false;
	model->m_generated = true;
	simulation::Region->erase(model);
	model->Angles(glm::vec3{0.f, Turn, 0.f});
	if (Moving)
	{
		// out of the groups of the instances of its model, which keep their places
		model->draw_alone();
	}
	simulation::Region->insert(model);
	++scene::node_state_version;
	return model;
}

// the deer moved to another place
void move_deer(TAnimModel &Model, glm::dvec3 const &Position)
{
	if (glm::length(Model.location() - Position) < 1e-4)
	{
		return;
	}
	simulation::Region->erase(&Model);
	Model.location(Position);
	simulation::Region->insert(&Model);
}

void remove_deer(TAnimModel *Model)
{
	if (Model != nullptr)
	{
		simulation::State.delete_model(Model);
		++scene::node_state_version;
	}
}

// the deer goes on with another animation, at frame Frame of clip Clip at the time Time; where the model can't be made, as it was
void change_deer(TAnimModel *&Model, glm::dvec3 const &Heading, vat_clip const &Animation, int const Clip, double const Frame, double const Time, bool const Moving)
{
	auto *model{make_deer(Animation, Model->location(), vat_turn(Heading, Animation, Clip, Frame, Time), Moving)};
	if (model == nullptr)
	{
		return;
	}
	remove_deer(Model);
	Model = model;
}

// the way a deer would run away from Start along Heading: true where it's all over flat ground, Ground then the heights of the ground
// at equal steps along it (the first one Start)
bool flat_way(glm::dvec3 const &Start, glm::dvec3 const &Heading, std::array<double, Groundsamples> &Ground)
{
	Ground.front() = Start.y;
	auto const length{flight_way(Flighttime)};
	auto low{Start.y};
	auto high{Start.y - 1e9};
	for (std::size_t sample = 1; sample < Groundsamples; ++sample)
	{
		auto const point{Start + Heading * (length * static_cast<double>(sample) / static_cast<double>(Groundsamples - 1))};
		auto const here{ground_at(point, Start.y + Groundceiling)};
		if ((false == here.found) || here.water || (here.height < Start.y - Waybelow) || (here.height > Start.y + Wayabove))
		{
			return false;
		}
		if ((sample > 1) && (std::abs(here.height - Ground[sample - 1]) > Waystep))
		{
			return false;
		}
		Ground[sample] = here.height;
		low = (sample == 1 ? here.height : std::min(low, here.height));
		high = std::max(high, here.height);
	}
	return (high - low <= Wayrange);
}

} // namespace

simulation::deer_herd simulation::Deer;

void simulation::deer_herd::update(double const Deltatime)
{
	if (m_region != simulation::Region)
	{
		// another scenery: the deer went with the one they were put in
		m_deer.clear();
		m_travelled = 0.0;
		m_region = simulation::Region;
	}
	auto const *vehicle{simulation::Train != nullptr ? simulation::Train->Dynamic() : nullptr};
	if ((false == Global.deer) || (vehicle == nullptr) || (simulation::Region == nullptr))
	{
		for (auto const &animal : m_deer)
		{
			remove_deer(animal.model);
		}
		m_deer.clear();
		m_travelled = 0.0;
		return;
	}
	auto const *mover{vehicle->MoverParameters};
	m_travelled += std::abs(mover->V) * Deltatime;
	if ((Global.deer_interval > 0.0) && (m_travelled >= Global.deer_interval))
	{
		m_travelled = 0.0;
		if ((Random(0.0, 1.0) < Global.deer_chance) && (false == spawn(*vehicle)))
		{
			// no flat ground, a switch or a platform there: soon again
			m_travelled = std::max(0.0, Global.deer_interval - Retrydistance);
		}
	}
	auto const train{vehicle->GetPosition()};
	auto const horn{(mover->WarningSignal & 7) != 0};
	auto const now{Timer::GetTime()};
	std::erase_if(m_deer, [this, &train, horn, now](deer &Animal) { return tend(Animal, train, horn, now); });
}

bool simulation::deer_herd::tend(deer &Animal, glm::dvec3 const &Train, bool const Horn, double const Now) const
{
	auto const distance{glm::length(Animal.model->location() - Train)};
	Animal.closest = std::min(Animal.closest, distance);
	if (((Animal.closest < Passedclose) && (distance > Animal.closest + Passedmargin)) || (distance > Global.deer_distance + 1500.0))
	{
		// the train is past it, or went another way
		remove_deer(Animal.model);
		return true;
	}
	switch (Animal.state)
	{
	case deer_state::grazing:
		if ((Horn && (distance < Global.deer_hornrange)) || (distance < Global.deer_fleedistance))
		{
			alarm(Animal, Train, Now);
		}
		break;
	case deer_state::alarmed:
		if (Now >= Animal.time)
		{
			change_deer(Animal.model, Animal.heading, Startle, Animal.clip, Startframe, Animal.time, false);
			Animal.state = deer_state::startled;
		}
		break;
	case deer_state::startled:
		if (Now >= Animal.time + Startletime)
		{
			Animal.time += Startletime;
			change_deer(Animal.model, Animal.heading, Flight, Animal.look, Startframe, Animal.time, true);
			flee(Animal);
		}
		break;
	case deer_state::fleeing:
		if (run(Animal, Now))
		{
			// grazing again, where it stopped, from the moment of the grazing the flight ends in
			Animal.frame = Grazeafter * Grazing.fps + Startframe;
			Animal.time += Flighttime;
			change_deer(Animal.model, Animal.heading, Grazing, 0, Animal.frame, Animal.time, false);
			Animal.state = deer_state::resting;
		}
		break;
	default:
		break;
	}
	return false;
}

bool simulation::deer_herd::spawn(TDynamicObject const &Vehicle)
{
	// the end of the train the way it goes, or the cab occupied when it stands
	auto const *mover{Vehicle.MoverParameters};
	auto front{mover->V > 0.1};
	if (std::abs(mover->V) <= 0.1)
	{
		front = (mover->CabOccupied >= 0);
	}
	auto const &axle{front ? Vehicle.Axle0 : Vehicle.Axle1};
	auto const direction{front ? axle.GetDirection() : -axle.GetDirection()};
	path_place place;
	if ((false == plain_way(axle.GetTrack(), axle.GetTranslation(), direction, Global.deer_distance, place)) || near_platform(place))
	{
		return false;
	}
	// across the path, a little askew, to the side with flat ground to run away over (one picked at random first)
	deer animal;
	animal.start = place.position;
	auto const askew{Random(-25.0, 25.0) * std::numbers::pi / 180.0};
	auto const first{Random(0.0, 1.0) < 0.5 ? -1.0 : 1.0};
	auto found{false};
	for (auto const side : {first, -first})
	{
		glm::dvec3 const across{place.forward.z * side, 0.0, -place.forward.x * side};
		animal.heading = glm::normalize(across * std::cos(askew) + place.forward * std::sin(askew));
		if (flat_way(animal.start, animal.heading, animal.ground))
		{
			found = true;
			break;
		}
	}
	if (false == found)
	{
		return false;
	}
	// grazing from a moment of its own
	animal.frame = Random(0.0, Grazing.frames);
	animal.time = Timer::GetTime();
	animal.model = make_deer(Grazing, place.position, vat_turn(animal.heading, Grazing, 0, animal.frame, animal.time), false);
	if (animal.model == nullptr)
	{
		return false;
	}
	WriteLog(std::format("Deer put at {:.1f} {:.1f} {:.1f}", place.position.x, place.position.y, place.position.z));
	m_deer.emplace_back(animal);
	return true;
}

void simulation::deer_herd::alarm(deer &Animal, glm::dvec3 const &Train, double const Now) const
{
	// it starts from the moment of the grazing nearest ahead whose start it has (every Window seconds)
	auto const grazed{fraction((Animal.frame + (Now - Animal.time) * Grazing.fps) / Grazing.frames) * Grazing.frames / Grazing.fps};
	auto const length{Grazing.frames / Grazing.fps};
	auto const windows{static_cast<int>(std::lround(length / Window))};
	auto window{static_cast<int>(std::ceil(grazed / Window - 1e-6))};
	auto wait{static_cast<double>(window) * Window - grazed};
	window %= windows;
	Animal.clip = window;
	Animal.time = Now + std::max(0.0, wait);
	// it looks at the train
	Animal.look = look_clip(Animal.heading, Train - Animal.model->location());
	Animal.state = deer_state::alarmed;
}

void simulation::deer_herd::flee(deer &Animal) const
{
	// the ground along its way was looked at when it was put there
	Animal.state = deer_state::fleeing;
	Animal.start = Animal.model->location();
}

bool simulation::deer_herd::run(deer &Animal, double const Now) const
{
	auto const elapsed{std::clamp(Now - Animal.time, 0.0, Flighttime)};
	auto const way{flight_way(elapsed)};
	auto const step{way / flight_way(Flighttime) * static_cast<double>(Groundsamples - 1)};
	auto const sample{std::min(static_cast<std::size_t>(step), Groundsamples - 2)};
	auto position{Animal.start + Animal.heading * way};
	position.y = std::lerp(Animal.ground[sample], Animal.ground[sample + 1], step - static_cast<double>(sample));
	move_deer(*Animal.model, position);
	return (Now - Animal.time >= Flighttime);
}
