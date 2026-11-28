/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// collisions of the ends, their crush and repair, derailment and putting back on the rails

#include "stdafx.h"
#include "physics/mover.h"
#include "vehicle/DynObj.h"
#include "physics/oerlikonest.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "utilities/parser.h"
#include "simulation/simulation.h"
#include "vehicle/fieldshunting.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <utility>
#include <vector>
#include "physics/components/internal.h"


double ComputeCollision(double &v1, double &v2, double m1, double m2, double beta, bool vc)
{ // oblicza zmiane predkosci i przyrost pedu wskutek kolizji
	assert(beta < 1.0);

	if (v1 < v2 && vc == true)
		return 0;
	else
	{
		double sum = m1 + m2;
		double w1 = (m2 * v2 * 2.0 + v1 * (m1 - m2)) / sum;
		double w2 = (m1 * v1 * 2.0 + v2 * (m2 - m1)) / sum;
		// the collision changes the velocities directly: the bounce loses energy in the velocities relative to the
		// centre of mass only, the pair keeps its momentum. scaling the velocities themselves braked the vehicles
		// down with every step their couplers spent stretched or pressed in all the way (eu07/maszyna#553)
		auto const centre{(m1 * v1 + m2 * v2) / sum};
		auto const restitution{std::sqrt(1.0 - beta)};
		v1 = std::lerp(centre, w1, restitution);
		v2 = std::lerp(centre, w2, restitution);
		return m1 * (w2 - w1) * (1 - beta);
	}
}
namespace
{

// a derailed vehicle tears its brake hoses: the brake pipe vents and the rest of the train brakes in emergency
void tear_brake_hoses(TMoverParameters &Vehicle)
{
	if (std::any_of(std::begin(Vehicle.Couplers), std::end(Vehicle.Couplers), [](auto const &Coupler) { return Coupler.CouplingFlag.HasAll(Coupling::brakeHose); }))
	{
		Vehicle.AlarmChainFlag = true;
	}
}

} // namespace

// *************************************************************************************************
// Q: 20160716
// Wykrywanie kolizji
// *************************************************************************************************
void TMoverParameters::CollisionDetect(int const End, double const dt)
{
	if (Neighbours[End].vehicle == nullptr)
	{
		return;
	} // shouldn't normally happen but, eh

	auto const &thiscoupler{Couplers[End]};
	auto *othervehicle{Neighbours[End].vehicle->MoverParameters};
	auto const otherend{Neighbours[End].vehicle_end};
	auto const &othercoupler{othervehicle->Couplers[otherend]};

	auto const physical{coupler_physics_at(End)};
	// physics.collisions new: past the buffers the ends of rail vehicles crush (collision.h); a hit the steel takes stays
	// the legacy one, and so do the collisions of the road vehicles (the traffic of a scenery), with the legacy rule
	auto const rails{TestFlag(CategoryFlag, 1) && TestFlag(othervehicle->CategoryFlag, 1)};
	auto const crushing{Global.collisions == global_settings::collision_model::physical && rails};
	if (crushing && (thiscoupler.hit_handled || crush_hit(End, dt)))
	{
		return;
	}
	// physics.collisions monitor: the legacy collision follows, the log tells what the crush would be
	if (Global.collisions == global_settings::collision_model::monitor && rails)
	{
		crush_hit(End, dt);
	}
	if (physical && false == coupler_stop_hit(End, *othervehicle, otherend))
	{
		return;
	}
	// the share of the energy of the relative motion the hit takes: in the physical model the springs gone solid give
	// for good, rather than spring back as the legacy model has it
	auto const loss{physical ? coupler_physics::solid_hit_loss(thiscoupler.physics, othercoupler.physics) : (thiscoupler.beta + othercoupler.beta) / 2.0};

	// the velocities along this vehicle's axis: the other vehicle's one is turned around when the vehicles face each
	// other (touch with the same ends), as in CouplerForce()
	auto const otherdirection{DirPatch(End, otherend)};
	auto velocity{V};
	auto othervehiclevelocity{othervehicle->V * otherdirection};
	// calculate collision force and new velocities for involved vehicles
	auto const VirtualCoupling{(thiscoupler.CouplingFlag == Coupling::none)};
	auto CCF{0.0};

	switch (End)
	{
	case 0:
	{
		CCF = ComputeCollision(velocity, othervehiclevelocity, TotalMass, othervehicle->TotalMass, loss, VirtualCoupling) / dt;
		break; // yB: ej ej ej, a po
	}
	case 1:
	{
		CCF = ComputeCollision(othervehiclevelocity, velocity, othervehicle->TotalMass, TotalMass, loss, VirtualCoupling) / dt;
		break;
	}
	default:
	{
		break;
	}
	}

	// the new model crushes the ends and derails the vehicles by the crush, see crush_hit()
	if (Global.crash_damage && false == crushing)
	{
		// the physical model breaks the coupling and crushes the buffers by the force of the hit (coupler_stop_hit())
		if (false == physical && -thiscoupler.Dist >= thiscoupler.DmaxB && FuzzyLogic(std::abs(CCF), 5.0 * (thiscoupler.FmaxC + 1.0), p_coupldmg))
		{
			// small chance to smash the coupler if it's hit with excessive force
			damage_coupler(End);
		}

		// HACK: limit excessive speed derailment checks to vehicles which aren't part of the same consist
		auto const outsideconsist{thiscoupler.CouplingFlag == Coupling::none || true == othervehicle->DamageFlag.HasAll(VehicleDamage::derailed)};
		auto const velocitydifference{outsideconsist ? CollisionVelocity(End, *othervehicle, otherend) : 0.0};

		if (velocitydifference > SafeCollisionVelocity)
		{
			// HACK: crude estimation for potential derail, will take place with velocity difference > 15 km/h adjusted for vehicle mass ratio
			if (false == DamageFlag.HasAll(VehicleDamage::derailed) || false == othervehicle->DamageFlag.HasAll(VehicleDamage::derailed))
			{
				WriteLog("Bad driving: " + Name + " and " + othervehicle->Name + " collided with velocity " + to_string(velocitydifference, 0) + " km/h");
			}

			if (velocitydifference > SafeCollisionVelocity * (TotalMass / othervehicle->TotalMass))
			{
				Derail(COLLISION);
			}
			if (velocitydifference > SafeCollisionVelocity * (othervehicle->TotalMass / TotalMass))
			{
				othervehicle->Derail(COLLISION);
			}
		}
	}

	// adjust velocity and acceleration of affected vehicles. the physical model changes the velocities alone: the
	// integration of the next step (ComputeMovement()) would carry a quarter of an acceleration spike over into the
	// velocity once more, and the vehicles would bounce off the stop faster than they hit it
	auto const accelerationspike{physical ? 0.0 : 1.0};
	// derailed the legacy way a vehicle is a wall, off the rails by the new criteria it slides on and takes the hit. with
	// the crush, the vehicles coupled to the wall push it on (crush_hit())
	auto const coupled{thiscoupler.CouplingFlag != Coupling::none};
	auto const wall = [crushing, coupled](TMoverParameters const &Vehicle) {
		return Vehicle.DamageFlag.HasAll(VehicleDamage::derailed) && (false == crushing || (Vehicle.Derailed == derailment::outcome::none && false == coupled));
	};
	if (false == wall(*this))
	{
		auto const accelerationchange{(velocity - V) / dt};
		AccS += accelerationchange * accelerationspike;
		V = velocity;
	}
	if (false == wall(*othervehicle))
	{
		auto const othervehiclenewvelocity{othervehiclevelocity * otherdirection}; // back along the other vehicle's axis
		auto const othervehicleaccelerationchange{(othervehiclenewvelocity - othervehicle->V) / dt};
		othervehicle->AccS += othervehicleaccelerationchange * accelerationspike;
		othervehicle->V = othervehiclenewvelocity;
	}
}

void TMoverParameters::damage_coupler(int const End)
{

	auto &thiscoupler{Couplers[End]};

	if (thiscoupler.type() == TCouplerType::Articulated)
	{
		return;
	} // HACK: don't break articulated couplings no matter what

	if (DamageFlag.Set(VehicleDamage::coupling))
		EventFlag = true;

	auto const hosetorn{thiscoupler.CouplingFlag.HasAll(Coupling::brakeHose)};
	if (hosetorn)
	{
		// hamowanie nagle - zerwanie przewodow hamulcowych
		AlarmChainFlag = true;
	}

	thiscoupler.CouplingFlag = {};
	thiscoupler.broken = true;

	if (thiscoupler.Connected != nullptr)
	{
		// break connection with other vehicle, if there's any: on the end it's coupled with, which is the front one too
		// for vehicles facing each other
		auto &othercoupler{thiscoupler.Connected->Couplers[thiscoupler.ConnectedNr]};
		othercoupler.CouplingFlag = Coupling::none;
		// the coupling broke between the two, either end can be repaired
		othercoupler.broken = true;
		if (hosetorn && coupler_physics_at(End))
		{
			// the brake pipe of the other part of the train vents through the torn hose as well
			thiscoupler.Connected->AlarmChainFlag = true;
		}
	}

	WriteLog("Bad driving: " + Name + " broke a coupler");
}

// *************************************************************************************************
// physics.collisions new: the collisions past the buffers, see collision.h
// *************************************************************************************************
collision::vehicle_kind TMoverParameters::collision_kind() const
{
	auto const powered{EngineType != TEngineType::None && EngineType != TEngineType::Dumb};
	auto const unit{is_multiple_unit()};
	auto const passengers{std::any_of(LoadAttributes.begin(), LoadAttributes.end(), [](auto const &Load) { return ToLower(Load.name).starts_with("passenger"); })};
	// a coach heats its compartments; the crash zone is a matter of each end (crush_of())
	return collision::kind_of(powered, unit || passengers || (false == powered && HeatingPower > 0.0), false);
}

collision::crush_curve TMoverParameters::crush_curve_at(int const End) const
{
	return collision::crush_of(Couplers[End].collision, collision_kind());
}

double TMoverParameters::buffer_height_at(int const End) const
{
	auto const loadshare{MaxLoad > 0.f ? LoadAmount / MaxLoad : 0.0};
	return collision::buffer_height(Couplers[End].collision, collision_kind(), loadshare);
}

void TMoverParameters::derail_in_collision(DerailReason const Reason, std::string const &Why)
{
	if (false == DamageFlag.Set(VehicleDamage::derailed))
	{
		return;
	}
	// upright, it slides on the sleepers and keeps passing the forces on
	Derailed = derailment::outcome::off_rails;
	DerailCause = derail_motion::cause::collision;
	DerailAcceleration = 0.0;
	DerailSide = 0;
	CollisionDerail = Reason;
	EventFlag = true;
	MainSwitch(false, range_t::local);
	if (Vel < 5.0)
	{
		// HACK: prevent permanent axle spin in static vehicle after a collision
		nrot = 0.0;
		SlippingWheels = false;
	}
	// the brake hoses tear: the brake pipe vents and the rest of the train brakes in emergency
	tear_brake_hoses(*this);
	derailment_monitor.reset();
	pushed_off_monitor.reset();
	ErrorLog(std::format("Bad driving: {} derailed in a collision, {}: {}", Name, Reason == COLLISION_OVERRIDE ? "overriding" : "pushed off the track", Why));
}
namespace
{


char const *end_name(int const End)
{
	return End == end::front ? "front" : "rear";
}

char const *kind_name(collision::vehicle_kind const Kind)
{
	switch (Kind)
	{
	case collision::vehicle_kind::locomotive:
		return "locomotive";
	case collision::vehicle_kind::coach:
		return "coach";
	case collision::vehicle_kind::crash_zone:
		return "crash zone";
	case collision::vehicle_kind::wagon:
	default:
		return "wagon";
	}
}

} // namespace

// the force pressing the end in, of the coupling or of the buffers touching [N]: forwards at the front pulls
double compression_at(TCoupling const &Coupler, int const End)
{
	return std::max(0.0, (End == end::front ? -1.0 : 1.0) * Coupler.CForce);
}
namespace
{

// the steel parts of both ends in series [N/m]
double steel_stiffness(TCoupling const &First, TCoupling const &Second)
{
	auto const first{std::max(First.physics.link_stiffness, 1.0)};
	auto const second{std::max(Second.physics.link_stiffness, 1.0)};
	return first * second / (first + second);
}

// the hit of the pair in the step, for the push off the track: the peak force and how long it acted
void record_hit(TCoupling &Coupler, double const Force, double const Duration)
{
	Coupler.hit_force = std::max(Coupler.hit_force, Force);
	Coupler.hit_duration = std::max(Coupler.hit_duration, Duration);
}

} // namespace

bool TMoverParameters::crush_hit(int const End, double const dt)
{
	auto &thiscoupler{Couplers[End]};
	auto *othervehicle{Neighbours[End].vehicle->MoverParameters};
	auto const otherend{Neighbours[End].vehicle_end};
	auto &othercoupler{othervehicle->Couplers[otherend]};

	auto const physical{coupler_physics_at(End)};
	// the buffers pressed all the way, not the draw gear pulled all the way, and the pair still closing in
	auto const buffers{physical ? thiscoupler.physics_state.stop < 0 : thiscoupler.Dist < 0.0};
	auto const closing{-CouplerVelocityDifference(End, *othervehicle, otherend) * DirF(End)};
	if (false == Global.crash_damage || false == buffers || closing <= 0.0)
	{
		// the hit is over, the next one is another one for the monitor
		thiscoupler.crush_monitored = false;
		othercoupler.crush_monitored = false;
		return false;
	}
	auto const monitor{Global.collisions == global_settings::collision_model::monitor};
	// put down by the scenery inside each other: pressed past the stop of the buffers and the crush of the ends by more
	// than a hit takes them in a step. the legacy collision pushes them apart; their springs, the stops at both ends
	// and the neighbours found at either end there make up any closing velocity, not a hit
	auto const stop{std::max(thiscoupler.DmaxB + othercoupler.DmaxB, thiscoupler.physics.buffers.travel + othercoupler.physics.buffers.travel) + thiscoupler.crush + othercoupler.crush};
	auto const overlap{-static_cast<double>(Neighbours[End].distance)};
	if (overlap > stop + collision::inside_margin + closing * dt)
	{
		// nor is what their springs took in pressed past the stop and back a hit, for as long as the buffers touch
		thiscoupler.contact.inside();
		othercoupler.contact.inside();
		if (false == thiscoupler.inside_reported)
		{
			thiscoupler.inside_reported = true;
			othercoupler.inside_reported = true;
			WriteLog(std::format("Bad scenery: {} and {} are inside each other, their buffers pressed {:.2f} m past their stop, no crush", Name, othervehicle->Name, overlap - stop));
		}
		return false;
	}
	// parting at the start of the step: what closes the pair in now is the knock of a hit of either vehicle in the step,
	// or of the vehicles inside each other next to it, not a hit of this pair. the legacy collision takes it; a pair
	// still closing in at the start of the next step is a hit
	if (thiscoupler.closing < 0.0)
	{
		return false;
	}
	// a vehicle derailed the legacy way (the end of the track, say) stays where it is, whatever hits it from outside its
	// train. the vehicles coupled to it push it on, as the train behind a derailed vehicle does: a wall to them, it
	// stopped the whole train at once, and the vehicles behind it crushed by the energy of the train
	auto const coupled{thiscoupler.CouplingFlag != Coupling::none};
	auto const wall{false == coupled && DamageFlag.HasAll(VehicleDamage::derailed) && Derailed == derailment::outcome::none};
	auto const otherwall{false == coupled && othervehicle->DamageFlag.HasAll(VehicleDamage::derailed) && othervehicle->Derailed == derailment::outcome::none};
	if (wall && otherwall)
	{
		return false;
	}
	constexpr double immovable{1e12};
	auto const mass{wall ? immovable : TotalMass};
	auto const othermass{otherwall ? immovable : othervehicle->TotalMass};
	auto const loss{physical ? coupler_physics::solid_hit_loss(thiscoupler.physics, othercoupler.physics) : (thiscoupler.beta + othercoupler.beta) / 2.0};

	auto const firstcurve{crush_curve_at(End)};
	auto const secondcurve{othervehicle->crush_curve_at(otherend)};
	auto const stiffness{steel_stiffness(thiscoupler, othercoupler)};
	auto const springs{compression_at(thiscoupler, End)};
	collision::hit_input input;
	input.velocity = closing;
	input.mass1 = mass;
	input.mass2 = othermass;
	input.restitution = std::sqrt(std::max(0.0, 1.0 - loss));
	input.stiffness = stiffness;
	input.springs = springs;
	// the monitor takes the whole hit at once, over a second, longer than the crush of any hit lasts
	input.dt = monitor ? 1.0 : dt;
	input.start_velocity = thiscoupler.closing;
	// the crush takes only what the hit brings past what the buffers of EN 15551 of both ends take in. an automatic
	// coupler (or its adapter) has no buffers at the side: its draft gear takes in what its springs of the .fiz do
	auto const reduced{mass * othermass / (mass + othermass)};
	auto const central{thiscoupler.type() == TCouplerType::Automatic || othercoupler.type() == TCouplerType::Automatic || thiscoupler.has_adapter() || othercoupler.has_adapter()};
	auto const capacity{central ? 0.0 : collision::buffer_capacity(thiscoupler.collision, collision_kind()) + collision::buffer_capacity(othercoupler.collision, othervehicle->collision_kind())};
	auto const hitvelocity{std::max(closing, thiscoupler.closing)};
	input.budget = thiscoupler.contact.budget(reduced, hitvelocity, capacity);
	auto const hit{collision::hit({firstcurve, thiscoupler.crush}, {secondcurve, othercoupler.crush}, input)};
	// the override, by the crush of both ends
	auto const climbing_of = [&](double const Crush1, double const Crush2) {
		collision::override_input climbing;
		climbing.crush1 = Crush1;
		climbing.crush2 = Crush2;
		climbing.mass1 = TotalMass - Mred;
		climbing.mass2 = othervehicle->TotalMass - othervehicle->Mred;
		climbing.height1 = buffer_height_at(End);
		climbing.height2 = othervehicle->buffer_height_at(otherend);
		climbing.anticlimbers = collision::anticlimber_of(thiscoupler.collision, collision_kind()) && collision::anticlimber_of(othercoupler.collision, othervehicle->collision_kind());
		climbing.zone1 = firstcurve.densify;
		climbing.zone2 = secondcurve.densify;
		return climbing;
	};
	if (monitor)
	{
		if (hit.crushing && false == thiscoupler.crush_monitored)
		{
			thiscoupler.crush_monitored = true;
			othercoupler.crush_monitored = true;
			++crush_reports;
			auto const crush1{thiscoupler.crush + hit.crush1};
			auto const crush2{othercoupler.crush + hit.crush2};
			auto const climbed{collision::override_of(climbing_of(crush1, crush2))};
			auto const torn{std::max(crush1, crush2) >= collision::draw_gear_crush};
			std::string const outcome{climbed.first && climbed.second ? std::string{"both would leave the rails"}
			                          : climbed.first                 ? Name + " would ride up onto " + othervehicle->Name
			                          : climbed.second                ? othervehicle->Name + " would ride up onto " + Name
			                                                          : std::string{"both would stay on the rails"}};
			WriteLog(std::format("collision monitor: {} and {} collided at {:.0f} km/h: the {} end of {} ({:.1f} t {}) would crush by {:.2f} m, the {} end of {} ({:.1f} t {}) by {:.2f} m, "
			                     "at {:.0f} kN; draw gear {}; {}",
			                     Name, othervehicle->Name, closing * 3.6, end_name(End), Name, TotalMass / 1000.0, kind_name(collision_kind()), crush1, end_name(otherend), othervehicle->Name,
			                     othervehicle->TotalMass / 1000.0, kind_name(othervehicle->collision_kind()), crush2, hit.force / 1000.0, torn ? "torn off" : "holds", outcome));
		}
		return false;
	}
	if (false == hit.crushing)
	{
		// the steel takes it, the legacy collision follows; its peak, under the force an end gives at, for the push
		// off the track
		auto const weaker{std::min(firstcurve.force(thiscoupler.crush), secondcurve.force(othercoupler.crush))};
		auto const peak{std::min(springs + coupler_physics::impact_force(closing, stiffness, mass, othermass), weaker)};
		auto const duration{std::min(dt, (1.0 + input.restitution) * std::sqrt(reduced / stiffness))};
		record_hit(thiscoupler, peak, duration);
		record_hit(othercoupler, peak, duration);
		return false;
	}
	// the other vehicle doesn't take it again in this step
	thiscoupler.hit_handled = true;
	othercoupler.hit_handled = true;

	// the hit at the force the weaker end gives at pushes the vehicles apart: the velocities alone change, along this
	// vehicle's axis as in CollisionDetect()
	auto const otherdirection{DirPatch(End, otherend)};
	auto const apart{End == end::front ? -1.0 : 1.0};
	if (false == wall)
	{
		V += apart * hit.impulse / mass;
	}
	if (false == otherwall)
	{
		othervehicle->V -= apart * hit.impulse / othermass * otherdirection;
	}
	record_hit(thiscoupler, hit.force, hit.duration);
	record_hit(othercoupler, hit.force, hit.duration);

	// the energy the crush took, for both vehicles' count of the contact
	thiscoupler.contact.crushed(hit.work);
	othercoupler.contact.crushed(hit.work);
	// what the log tells of the pair, for the analysis of a crush
	auto const kinetic{0.5 * reduced * hitvelocity * hitvelocity};
	auto const pair = [&]() {
		return std::format("{} {:.1f} t {}, {} end crushed {:.2f} m, gives at {:.0f} kN; {} {:.1f} t {}, {} end crushed {:.2f} m, gives at {:.0f} kN; force {:.0f} kN, springs {:.0f} kN, "
		                   "closing at {:.1f} km/h at the start of the step; energy {:.0f} kJ of the motion and {:.0f} kJ in the buffers, past the {:.0f} kJ they take in, "
		                   "{:.0f} kJ crushed in the contact",
		                   Name, TotalMass / 1000.0, kind_name(collision_kind()), end_name(End), thiscoupler.crush, firstcurve.force(thiscoupler.crush) / 1000.0, othervehicle->Name,
		                   othervehicle->TotalMass / 1000.0, kind_name(othervehicle->collision_kind()), end_name(otherend), othercoupler.crush,
		                   secondcurve.force(othercoupler.crush) / 1000.0, hit.force / 1000.0, springs / 1000.0, input.start_velocity * 3.6, kinetic / 1000.0,
		                   thiscoupler.contact.absorbed() / 1000.0, capacity / 1000.0, thiscoupler.contact.crush_work() / 1000.0);
	};
	auto const crushed = [&](TMoverParameters const &Vehicle, int const VehicleEnd, double const Before, double const After) {
		if (Before < collision::crush_mark && After >= collision::crush_mark)
		{
			ErrorLog(std::format("Bad driving: {} and {} collided at {:.0f} km/h, the {} end of {} crushed ({})", Name, othervehicle->Name, closing * 3.6, end_name(VehicleEnd), Vehicle.Name, pair()));
		}
	};
	auto const before{thiscoupler.crush};
	auto const otherbefore{othercoupler.crush};
	thiscoupler.crush += hit.crush1;
	othercoupler.crush += hit.crush2;
	crushed(*this, End, before, thiscoupler.crush);
	crushed(*othervehicle, otherend, otherbefore, othercoupler.crush);

	// the crushed headstock tears off the draw gear, with the brake hose: both parts of the train brake in emergency.
	// the coupler of an end not coupled is gone as well, it doesn't couple any more
	if (thiscoupler.CouplingFlag != Coupling::none && std::max(thiscoupler.crush, othercoupler.crush) >= collision::draw_gear_crush)
	{
		auto const hose{thiscoupler.CouplingFlag.HasAll(Coupling::brakeHose)};
		damage_coupler(End);
		if (hose)
		{
			othervehicle->AlarmChainFlag = true;
		}
	}
	else if (thiscoupler.CouplingFlag == Coupling::none)
	{
		if (thiscoupler.crush >= collision::draw_gear_crush && false == thiscoupler.broken)
		{
			damage_coupler(End);
		}
		if (othercoupler.crush >= collision::draw_gear_crush && false == othercoupler.broken)
		{
			othervehicle->damage_coupler(otherend);
		}
	}

	// the end crushed deep enough lets one vehicle climb onto the other one
	auto const climbing{climbing_of(thiscoupler.crush, othercoupler.crush)};
	auto const climbed{collision::override_of(climbing)};
	auto const why = [&](TMoverParameters const &Climber, TMoverParameters const &Other) {
		return std::format("{} rode up onto {} at {:.0f} km/h, the ends crushed by {:.2f} m, buffers at {:.0f} / {:.0f} mm, anticlimbers {} ({})", Climber.Name, Other.Name, closing * 3.6,
		                   thiscoupler.crush + othercoupler.crush, climbing.height1 * 1000.0, climbing.height2 * 1000.0, climbing.anticlimbers ? "yes" : "no", pair());
	};
	if (climbed.first)
	{
		derail_in_collision(COLLISION_OVERRIDE, why(*this, *othervehicle));
	}
	if (climbed.second)
	{
		othervehicle->derail_in_collision(COLLISION_OVERRIDE, why(*othervehicle, *this));
	}
	return true;
}

void TMoverParameters::check_pushed_off(double const dt, double const Distance)
{
	if (Global.collisions != global_settings::collision_model::physical || false == Global.crash_damage || DamageFlag.HasAll(VehicleDamage::derailed) || false == TestFlag(CategoryFlag, 1) ||
	    DebugModeFlag)
	{
		pushed_off_monitor.reset();
		return;
	}
	collision::pushed_vehicle vehicle;
	vehicle.mass = TotalMass - Mred;
	vehicle.axles = NAxles > 0 ? NAxles : 4;
	vehicle.length = Dim.L;
	vehicle.pivots = BDist;
	vehicle.axle_base = ADist > 0.0 ? ADist : BDist;

	auto climbs{false};
	auto distance{0.0};
	auto lifted{false};
	auto duration{0.0};
	auto worst{0.0};
	auto worstforce{0.0};
	auto worstend{static_cast<int>(end::front)};
	for (int end = end::front; end <= end::rear; ++end)
	{
		auto const *other{Neighbours[end].vehicle};
		if (other == nullptr)
		{
			continue;
		}
		auto const &coupler{Couplers[end]};
		collision::pushed_end pushed;
		pushed.other_length = other->MoverParameters->Dim.L;
		pushed.crush = coupler.crush;
		// the steady push of the springs over the step, or the peak of a hit for as long as it lasted: the flange climbs
		// over the distance the wheels rolled meanwhile, a wheel lifted off the rail goes in a short time, rolling or not
		for (auto const &[force, time] : {std::pair{compression_at(coupler, end), dt}, std::pair{coupler.hit_force, coupler.hit_duration}})
		{
			pushed.force = force;
			auto const wheel{collision::pushed_forces(vehicle, pushed, RunningShape.R)};
			auto const ratio{wheel.ratio()};
			if (ratio < derailment::climbing_limit)
			{
				continue;
			}
			climbs = true;
			distance = std::max(distance, Distance * std::clamp(time / std::max(dt, 1e-9), 0.0, 1.0));
			if (wheel.lifted())
			{
				lifted = true;
				duration = std::max(duration, time);
			}
			if (ratio > worst)
			{
				worst = ratio;
				worstforce = force;
				worstend = end;
			}
		}
	}
	if (pushed_off_monitor.update(climbs, distance, lifted, duration))
	{
		derail_in_collision(COLLISION_PUSHED_OFF, std::format("{:.0f} kN pressing its {} end in, {}, at {:.0f} km/h, curve radius {:.0f} m: Y/Q {:.2f} ({:.1f} t {}, {} axles, "
		                                                      "springs {:.0f} / {:.0f} kN, hits {:.0f} / {:.0f} kN, crushed {:.2f} / {:.2f} m)",
		                                                      worstforce / 1000.0, end_name(worstend),
		                                                      lifted ? std::format("its wheels lifted off the rail for {:.3f} s", pushed_off_monitor.time())
		                                                             : std::format("the flange climbing over {:.2f} m", pushed_off_monitor.distance()),
		                                                      Vel, std::abs(RunningShape.R), std::min(worst, 99.0),
		                                                      vehicle.mass / 1000.0, kind_name(collision_kind()), vehicle.axles, compression_at(Couplers[end::front], end::front) / 1000.0,
		                                                      compression_at(Couplers[end::rear], end::rear) / 1000.0, Couplers[end::front].hit_force / 1000.0, Couplers[end::rear].hit_force / 1000.0,
		                                                      Couplers[end::front].crush, Couplers[end::rear].crush));
	}
}
namespace
{


// the end as it was built (physics.collisions new): not crushed, and nothing of a contact or a hit kept
void restore_end(TCoupling &Coupler)
{
	Coupler.crush = 0.0;
	Coupler.contact.reset();
	Coupler.hit_handled = false;
	Coupler.hit_force = 0.0;
	Coupler.hit_duration = 0.0;
	Coupler.crush_monitored = false;
	Coupler.inside_reported = false;
}

// the coupler as good as new, its end with it (the crush that tore the draw gear off), the vehicle without the coupler
// damage (and the emergency braking of the torn brake hose) once neither of its couplers is broken
void repair(TMoverParameters &Vehicle, TCoupling &Coupler)
{
	Coupler.broken = false;
	Coupler.stretch_duration = 0.f;
	Coupler.physics_state.couple();
	restore_end(Coupler);
	if (Vehicle.Couplers[end::front].broken || Vehicle.Couplers[end::rear].broken)
	{
		return;
	}
	Vehicle.DamageFlag.Clear(VehicleDamage::coupling);
	// either vehicle of the pair may have been braking through the torn hose
	Vehicle.AlarmChainFlag = false;
}

} // namespace

bool TMoverParameters::repair_coupler(int const End)
{
	auto &thiscoupler{Couplers[End]};
	if (false == thiscoupler.broken)
	{
		return false;
	}
	repair(*this, thiscoupler);
	// the other half of the coupling, which may be the vehicle flagged with the damage
	if (auto *other{thiscoupler.Connected}; other != nullptr && other->Couplers[thiscoupler.ConnectedNr].broken)
	{
		repair(*other, other->Couplers[thiscoupler.ConnectedNr]);
	}
	WriteLog(std::format("Coupler of {} repaired, at its {} end", Name, End == end::front ? "front" : "rear"));
	return true;
}

static_assert(derailment::track_damage::railwear == dtrack_railwear && derailment::track_damage::freerail == dtrack_freerail && derailment::track_damage::thinrail == dtrack_thinrail &&
              derailment::track_damage::railbend == dtrack_railbend);

double TMoverParameters::centre_of_gravity() const
{
	auto const kind{EngineType == TEngineType::None ? derailment::vehicle_class::unpowered : derailment::vehicle_class::powered};
	auto const empty{CentreOfGravity > 0.0 ? CentreOfGravity : derailment::default_centre_of_gravity(Dim.H, kind)};
	auto const loadmass{TotalMass - Mass - Mred};
	auto const loadshare{MaxLoad > 0.f ? LoadAmount / MaxLoad : 1.0};
	return derailment::loaded_centre_of_gravity(empty, Mass, loadmass, loadshare, Floor, Dim.H);
}

void TMoverParameters::check_derailment(TTrackShape const &Shape, TTrackParam const &Track, double const Distance)
{
	if (DamageFlag.HasAll(VehicleDamage::derailed) || false == TestFlag(CategoryFlag, 1) || DebugModeFlag)
	{
		derailment_monitor.reset();
		return;
	}
	derailment::vehicle_state vehicle;
	vehicle.mass = TotalMass - Mred;
	vehicle.axles = NAxles > 0 ? NAxles : 4;
	vehicle.length = Dim.L;
	vehicle.axle_base = ADist > 0.0 ? ADist : BDist;
	vehicle.centre_of_gravity = centre_of_gravity();
	for (int end = end::front; end <= end::rear; ++end)
	{
		auto const *other{Neighbours[end].vehicle};
		if (other == nullptr)
		{
			continue;
		}
		// the force of the coupling, or of the buffers touching, along this vehicle: forwards at the front pulls
		vehicle.ends[end].compression = (end == end::front ? -1.0 : 1.0) * Couplers[end].CForce;
		vehicle.ends[end].other_length = other->MoverParameters->Dim.L;
	}
	derailment::track_state track;
	track.radius = Shape.R;
	track.cant = Shape.dHrail;
	track.gauge = TrackW;
	track.uneven_curvature = Shape.uneven_curvature;
	track.uneven_cross = Shape.uneven_cross;
	track.buckle_curvature = Shape.buckle_curvature;
	track.damage = Track.DamageFlag;

	auto const verdict{derailment_monitor.update(vehicle, track, V, Distance)};
	if (verdict.result == derailment::outcome::none)
	{
		return;
	}
	if (Global.derailment == global_settings::derailment_model::physical)
	{
		derail(verdict, vehicle, track, Track.Name);
		return;
	}
	// physics.derailment monitor: the legacy draw decides, the log tells what the criteria would do, once on a track
	if (derailment_reported.has_value() && *derailment_reported == Track.Name)
	{
		return;
	}
	derailment_reported = Track.Name;
	++derailment_reports;
	WriteLog("derailment monitor: " + Name + " " + derailment::report(verdict, vehicle, track, V, Track.Name != nullptr ? *Track.Name : std::string{}));
}

void TMoverParameters::derail(derailment::verdict const &Verdict, derailment::vehicle_state const &Vehicle, derailment::track_state const &Track, std::string const *TrackName)
{
	if (false == DamageFlag.Set(VehicleDamage::derailed))
	{
		return;
	}
	Derailed = Verdict.result;
	DerailCause = Verdict.result == derailment::outcome::overturned ? derail_motion::cause::overturned : derail_motion::cause::off_rails;
	// out of the curve; a buckle bending the line against the curve more than the curve does throws it into the curve
	DerailAcceleration = std::abs(Track.radius) > 1e-3 && derailment::line_curvature(Track) < 0.0 ? -Verdict.acceleration : Verdict.acceleration;
	DerailSide = 0;
	EventFlag = true;
	MainSwitch(false, range_t::local);
	// the brake hoses tear: the brake pipe vents and the rest of the train brakes in emergency
	tear_brake_hoses(*this);
	ErrorLog("Bad driving: " + Name + " " + derailment::report(Verdict, Vehicle, Track, V, TrackName != nullptr ? *TrackName : std::string{}));
	if (Track.buckle_curvature != 0.0)
	{
		ErrorLog(std::format("Bad driving: {} derailed on buckled track, the buckle bending the line by 1/{:.0f} m {} the curve", Name, 1.0 / std::abs(Track.buckle_curvature),
		                     Track.buckle_curvature > 0.0 ? "with" : "against"));
	}
}

void TMoverParameters::rerail()
{
	DamageFlag.Clear(VehicleDamage::derailed);
	Derailed = derailment::outcome::none;
	// and back on the track; what derailed it before has no say in how it leaves the track the next time
	OffTrack.reset();
	DerailCause = derail_motion::cause::off_rails;
	DerailAcceleration = 0.0;
	DerailSide = 0;
	derailment_monitor.reset();
	derailment_distance = 0.0;
	derailment_reported.reset();
	// nor derailed in a collision
	CollisionDerail = NONE;
	pushed_off_monitor.reset();
	// back on the axis of the track, which the derailment shifted the vehicle off aside and down (a road vehicle keeps
	// its place on the road)
	if (TestFlag(CategoryFlag, 1))
	{
		OffsetTrackH = 0.0;
		OffsetTrackV = 0.0;
	}
	// a derailed vehicle which stopped sleeps (Global.physics_sleepheld) with the friction of the derailment in FStand and
	// where the derailment put it, until something wakes it: the physics on, for the friction of the rails and its place on
	// them. it is put back by a command (resetconsist, SetDamage) or by the road spawner, which sets it going anyway
	switch_physics(true, wake_cause::command);
}

void TMoverParameters::repair_damage()
{
	DamageFlag = {};
	EngDmgFlag = 0;
	WheelFlat = 0.0;
	AlarmChainFlag = false;
	// the crushed ends rebuilt
	for (auto &coupler : Couplers)
	{
		restore_end(coupler);
	}
	rerail();
}

void TMoverParameters::Derail(DerailReason const Reason)
{
	if (DamageFlag.Set(VehicleDamage::derailed))
	{
		EventFlag = true;
		MainSwitch(false, range_t::local);

		auto const velocity{V};
		AccS *= 0.65;
		V *= 0.65;
		// physics.collisions new: the vehicles coupled to it lose the speed with it. the cut in a step stands for the
		// train coming to a stop with the vehicle; the coupled ones keeping their speed would run into it at a third of
		// the speed of the train, and crush their ends and ride up onto it for a hit no train had
		if (Global.collisions == global_settings::collision_model::physical && TestFlag(CategoryFlag, 1))
		{
			change_train_velocity(V - velocity);
		}
		RunningShape.R = 0;
		if (Vel < 5.0)
		{
			// HACK: prevent permanent axle spin in static vehicle after a collision
			nrot = 0.0;
			SlippingWheels = false;
		}
		// physics.derailment new: off the end of the track the vehicle runs on straight on its wheels, sliding on the
		// ground as the vehicles the criteria derail do, and its brake hoses tear
		if (Reason == END_OF_TRACK && Global.derailment == global_settings::derailment_model::physical && TestFlag(CategoryFlag, 1))
		{
			Derailed = derailment::outcome::off_rails;
			DerailCause = derail_motion::cause::end_of_track;
			DerailAcceleration = 0.0;
			DerailSide = 0;
			tear_brake_hoses(*this);
		}

		// Print a message in the log.
		if (Reason == END_OF_TRACK)
			ErrorLog("Bad driving: " + Name + " derailed due to end of track");
		else if (Reason == TOO_HIGH_SPEED)
			ErrorLog("Bad driving: " + Name + " derailed due to too high speed");
		else if (Reason == GAUGE_MISMATCH)
			ErrorLog("Bad dynamic: " + Name + " derailed due to track width"); // błąd w scenerii
		else if (Reason == WRONG_TRACK_TYPE)
			ErrorLog("Bad dynamic: " + Name + " derailed due to wrong track type"); // błąd w scenerii
		else if (Reason == COLLISION)
			WriteLog("Bad driving: " + Name + " derailed"); // This reason also generates its own message in `TMoverParameters::CollisionDetect()`
		else if (Reason == TRAILED_SWITCH)
			ErrorLog("Bad driving: " + Name + " derailed trailing through a switch with a movable nose set the other way");
	}
}

bool TMoverParameters::leaves_track() const
{
	return Global.derailment == global_settings::derailment_model::physical && TestFlag(CategoryFlag, 1) && DamageFlag.HasAll(VehicleDamage::derailed) && Derailed != derailment::outcome::none &&
	       false == OffTrack.active();
}

void TMoverParameters::pull_off(int const Side, double const Acceleration, std::string const &Why)
{
	if (Global.derailment != global_settings::derailment_model::physical || false == DamageFlag.Set(VehicleDamage::derailed))
	{
		return;
	}
	// upright, its flange climbs the rail the coupling pulls it over
	Derailed = derailment::outcome::off_rails;
	DerailCause = derail_motion::cause::pulled_off;
	DerailAcceleration = Acceleration;
	DerailSide = Side;
	EventFlag = true;
	MainSwitch(false, range_t::local);
	tear_brake_hoses(*this);
	derailment_monitor.reset();
	ErrorLog(std::format("Bad driving: {} pulled off the track by the vehicle coupled to it, {}", Name, Why));
}
