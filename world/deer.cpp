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
#include <cmath>
#include <format>
#include <numbers>
#include <string>
#include <string_view>
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "model/AnimModel.h"
#include "scene/scene.h"
#include "scene/scenenode.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "rendering/renderer.h"
#include "vehicle/DynObj.h"
#include "McZapkie/MOVER.h"
#include "world/Track.h"
#include "utilities/Timer.h"

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
// the flight from the alarm: looks at the train (clip 0 to the left, 1 to the right), bounds away and stops in the grazing pose of the
// moment Grazeafter, in Flighttime; two more frames hold the end
constexpr vat_clip Flight{"animals/jelen_skok", 30.0, 168.0, 2};
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
// the ground under the way of the deer is looked for up to this much above the place it starts from, metres
constexpr double Groundceiling{3.0};
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

// what's looked for under a point: its place, the highest ground found so far
struct ground_probe
{
	glm::dvec3 point{0.0};
	double ceiling{0.0};
	bool found{false};
	double height{0.0};

	void triangle(glm::dvec3 const &A, glm::dvec3 const &B, glm::dvec3 const &C)
	{
		if (double y{0.0}; triangle_height(A, B, C, point.x, point.z, y) && (y <= ceiling) && ((false == found) || (y > height)))
		{
			found = true;
			height = y;
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

// the height of the ground of the scenery (its shapes: the terrain) under the point, not higher than Ceiling. false where there's
// none: the ground of terrain models, or none loaded
bool ground_height(glm::dvec3 const &Point, double const Ceiling, double &Height)
{
	if (simulation::Region == nullptr)
	{
		return false;
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
		}
	}
	Height = probe.height;
	return probe.found;
}

// a place on a path: the point of its axis and the way along it, level, of length 1
struct path_place
{
	glm::dvec3 position{0.0};
	glm::dvec3 forward{0.0, 0.0, 1.0};
};

// the place Distance from Track at Offset along it, going the way of Direction (1 towards its end, -1 towards its start), through the
// connected paths. false where the paths end earlier (or reach a crossing of roads)
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
			if ((track == nullptr) || (track->eType == tt_Cross))
			{
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
		if (Random(0.0, 1.0) < Global.deer_chance)
		{
			spawn(*vehicle);
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

void simulation::deer_herd::spawn(TDynamicObject const &Vehicle)
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
	if (false == path_ahead(axle.GetTrack(), axle.GetTranslation(), direction, Global.deer_distance, place))
	{
		return;
	}
	// across the path, to one side or the other, a little askew
	auto const side{Random(0.0, 1.0) < 0.5 ? -1.0 : 1.0};
	auto const askew{Random(-25.0, 25.0) * std::numbers::pi / 180.0};
	glm::dvec3 const across{place.forward.z * side, 0.0, -place.forward.x * side};
	deer animal;
	animal.heading = glm::normalize(across * std::cos(askew) + place.forward * std::sin(askew));
	animal.start = place.position;
	// grazing from a moment of its own
	animal.frame = Random(0.0, Grazing.frames);
	animal.time = Timer::GetTime();
	animal.model = make_deer(Grazing, place.position, vat_turn(animal.heading, Grazing, 0, animal.frame, animal.time), false);
	if (animal.model == nullptr)
	{
		return;
	}
	m_deer.emplace_back(animal);
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
	// it looks at the train: to its left or to its right (the left of a heading h is (h.z, 0, -h.x))
	auto const towards{Train - Animal.model->location()};
	Animal.look = ((towards.x * Animal.heading.z - towards.z * Animal.heading.x) > 0.0 ? 0 : 1);
	Animal.state = deer_state::alarmed;
}

void simulation::deer_herd::flee(deer &Animal) const
{
	Animal.state = deer_state::fleeing;
	Animal.start = Animal.model->location();
	// the ground along the way it'll run, where there's none found the height of the step before
	Animal.ground.front() = Animal.start.y;
	auto const length{flight_way(Flighttime)};
	for (std::size_t sample = 1; sample < Groundsamples; ++sample)
	{
		auto const point{Animal.start + Animal.heading * (length * static_cast<double>(sample) / static_cast<double>(Groundsamples - 1))};
		double height{0.0};
		Animal.ground[sample] = (ground_height(point, Animal.start.y + Groundceiling, height) ? height : Animal.ground[sample - 1]);
	}
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
