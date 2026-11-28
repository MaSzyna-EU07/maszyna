/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// attaching and detaching, the forces in the couplers and the automatic coupling

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

// Ra: tu należy przenosić funcje z mover.pas, które nie są z niego wywoływane.
// Jeśli jakieś zmienne nie są używane w mover.pas, też można je przenosić.
// Przeniesienie wszystkiego na raz zrobiło by zbyt wielki chaos do ogarnięcia.

const double dEpsilon = 0.01; // 1cm (zależy od typu sprzęgu...)

double TMoverParameters::Distance(const TLocation &Loc1, const TLocation &Loc2, const TDimension &Dim1, const TDimension &Dim2)
{ // zwraca odległość pomiędzy pojazdami (Loc1) i (Loc2) z uwzględnieneim ich długości (kule!)
	return hypot(Loc2.X - Loc1.X, Loc1.Y - Loc2.Y) - 0.5 * (Dim2.L + Dim1.L);
};

double TMoverParameters::CouplerDist(TMoverParameters const *Left, TMoverParameters const *Right)
{ // obliczenie odległości pomiędzy sprzęgami (kula!)
	return Distance(Left->Loc, Right->Loc, Left->Dim, Right->Dim); // odległość pomiędzy sprzęgami (kula!)
};

bool TMoverParameters::Attach(int ConnectNo, int ConnectToNr, TMoverParameters *ConnectTo, CouplingSet CouplingType, bool Enforce, bool Audible)
{ // łączenie do swojego sprzęgu (ConnectNo) pojazdu (ConnectTo) stroną (ConnectToNr)
	// Ra: zwykle wykonywane dwukrotnie, dla każdego pojazdu oddzielnie
	// Ra: trzeba by odróżnić wymóg dociśnięcia od uszkodzenia sprzęgu przy podczepianiu AI do składu

	if (ConnectTo == nullptr || CouplingType == Coupling::none)
	{
		return false;
	}

	auto &thiscoupler{Couplers[ConnectNo]};
	auto const otherend{ConnectToNr != 2 ? ConnectToNr : thiscoupler.ConnectedNr};
	auto &othercoupler = ConnectTo->Couplers[otherend];
	auto const distance{CouplerDist(this, ConnectTo) - (thiscoupler.adapter_length + othercoupler.adapter_length)};


	if (auto const couplercheck{Enforce || (distance <= dEpsilon && thiscoupler.type() != TCouplerType::NoCoupler && thiscoupler.type() == othercoupler.type())}; false == couplercheck)
	{
		return false;
	}

	// no multiple-unit control between an electric and a diesel locomotive, whoever asks for it: the scenario, the AI
	// or the player (eu07/maszyna#520)
	if (auto const compatible{CompatibleCoupling(ConnectNo, *ConnectTo, otherend, CouplingType)}; compatible != CouplingType)
	{
		WriteLog(std::format("Vehicles \"{}\" and \"{}\" can't share their control, coupled without the control connection", Name, ConnectTo->Name));
		CouplingType = compatible;
		if (CouplingType == Coupling::none)
		{
			return false;
		}
	}

	// stykaja sie zderzaki i kompatybilne typy sprzegow, chyba że łączenie na starcie
	if (thiscoupler.CouplingFlag == Coupling::none)
	{
		// a new coupling: the play of the coupling as set up anew, without the stretch of the one before
		thiscoupler.physics_state.couple();
		othercoupler.physics_state.couple();
		// jeśli wcześniej nie było połączone, ustalenie z której strony rysować sprzęg
		thiscoupler.Render = true; // tego rysować
		othercoupler.Render = false; // a tego nie
	}
	auto const couplingchange{CouplingType ^ thiscoupler.CouplingFlag};
	thiscoupler.Connected = ConnectTo;
	thiscoupler.CouplingFlag = CouplingType; // ustawienie typu sprzęgu
	if (ConnectToNr != 2)
	{
		thiscoupler.ConnectedNr = ConnectToNr; // 2=nic nie podłączone
	}
	othercoupler.Connected = this;
	othercoupler.CouplingFlag = CouplingType;
	othercoupler.ConnectedNr = ConnectNo;

	if (true == Audible && false == couplingchange.Empty())
	{
		// set sound event flag
		int soundflag{sound::none};
		std::vector<std::pair<Coupling, sound>> const soundmappings = {{Coupling::coupler, sound::attachcoupler},   {Coupling::brakeHose, sound::attachbrakehose},
		                                                               {Coupling::mainHose, sound::attachmainhose}, {Coupling::control, sound::attachcontrol},
		                                                               {Coupling::gangway, sound::attachgangway},   {Coupling::heating, sound::attachheating}};
		for (auto const &[mappingcoupling, mappingsound] : soundmappings)
		{
			if (couplingchange.HasAny(mappingcoupling))
			{
				soundflag |= mappingsound;
			}
		}
		SetFlag(thiscoupler.sounds, soundflag);
	}

	return true;
}

int TMoverParameters::DettachStatus(int ConnectNo)
{ // Ra: sprawdzenie, czy odległość jest dobra do rozłączania
	// powinny być 3 informacje: =0 sprzęg już rozłączony, <0 da się rozłączyć. >0 nie da się rozłączyć
	if (!Couplers[ConnectNo].Connected)
		return 0; // nie ma nic, to rozłączanie jest OK
	if (false == Couplers[ConnectNo].CouplingFlag.HasAny(Coupling::coupler))
		return -Couplers[ConnectNo].CouplingFlag.Bits(); // hak nie połączony - rozłączanie jest OK
	if (DamageFlag.HasAll(VehicleDamage::coupling))
		return -Couplers[ConnectNo].CouplingFlag.Bits(); // hak urwany - rozłączanie jest OK
	//    CouplerDist(ConnectNo);
	if (Couplers[ConnectNo].type() != TCouplerType::Screw || Neighbours[ConnectNo].distance < 0.01)
		return -Couplers[ConnectNo].CouplingFlag.Bits(); // można rozłączać, jeśli dociśnięty
	return Neighbours[ConnectNo].distance > 0.2 ? -Couplers[ConnectNo].CouplingFlag.Bits() : Couplers[ConnectNo].CouplingFlag.Bits();
};

bool TMoverParameters::Dettach(int ConnectNo)
{ // rozlaczanie

	auto &thiscoupler{Couplers[ConnectNo]};
	auto &othervehicle{thiscoupler.Connected};
	auto &othercoupler{othervehicle->Couplers[thiscoupler.ConnectedNr]};

	if (othervehicle == nullptr)
	{
		return true;
	} // nie ma nic, to odczepiono

	auto couplingchange{thiscoupler.CouplingFlag}; // presume we'll uncouple all active flags
	auto const couplingstate{DettachStatus(ConnectNo)}; // stan sprzęgu
	if (couplingstate < 0)
	{
		// gdy scisniete zderzaki, chyba ze zerwany sprzeg (wirtualnego nie odpinamy z drugiej strony)
		std::tie(othercoupler.Connected, othercoupler.ConnectedNr, othercoupler.CouplingFlag) = std::make_tuple(nullptr, -1, Coupling::none);
		std::tie(thiscoupler.Connected, thiscoupler.ConnectedNr, thiscoupler.CouplingFlag) = std::make_tuple(nullptr, -1, Coupling::none);
	}
	else if (couplingstate > 0)
	{ // odłączamy węże i resztę, pozostaje sprzęg fizyczny, który wymaga dociśnięcia (z wirtualnym nic)
		thiscoupler.CouplingFlag &= Coupling::coupler;
		othercoupler.CouplingFlag &= Coupling::coupler;
	}
	// set sound event flag
	couplingchange ^= thiscoupler.CouplingFlag; // remaining bits were removed from coupling
	if (false == couplingchange.Empty())
	{
		int soundflag{sound::detach}; // HACK: use detach flag to indicate removal of listed coupling
		std::vector<std::pair<Coupling, sound>> const soundmappings = {{Coupling::coupler, sound::attachcoupler},   {Coupling::brakeHose, sound::attachbrakehose},
		                                                               {Coupling::mainHose, sound::attachmainhose}, {Coupling::control, sound::attachcontrol},
		                                                               {Coupling::gangway, sound::attachgangway},   {Coupling::heating, sound::attachheating}};
		for (auto const &[mappingcoupling, mappingsound] : soundmappings)
		{
			if (couplingchange.HasAny(mappingcoupling))
			{
				soundflag |= mappingsound;
			}
		}
		SetFlag(thiscoupler.sounds, soundflag);
	}

	return couplingstate < 0;
};

// *************************************************************************************************
// prędkość zbliżania się stykających się końców pojazdów przy zderzeniu, w km/h
// *************************************************************************************************
double TMoverParameters::CollisionVelocity(int const End, TMoverParameters const &Other, int const OtherEnd) const
{
	auto const velocitydifference{CouplerVelocityDifference(End, Other, OtherEnd)};
	// the front end closes in on the other vehicle when this one moves forward faster, the rear end when it's slower
	return (End == end::front ? velocitydifference : -velocitydifference) * 3.6; // m/s -> km/h
}

double TMoverParameters::CouplerVelocityDifference(int const End, TMoverParameters const &Other, int const OtherEnd) const
{
	// V runs along the vehicle's own axis, towards its front; the other vehicle's velocity is turned around when the
	// vehicles face each other (touch with the same ends)
	return V - Other.V * DirPatch(End, OtherEnd);
}

bool TMoverParameters::AutomaticCouplingSafe(int const End, TMoverParameters const &Other, int const OtherEnd) const
{
	return false == Global.crash_damage || CollisionVelocity(End, Other, OtherEnd) <= SafeCollisionVelocity;
}
namespace
{


// the kind of traction a vehicle's engine stands for, as far as its multiple-unit control goes
enum class traction_kind
{
	none,
	electric,
	diesel
};

traction_kind traction_of(TEngineType const Engine)
{
	switch (Engine)
	{
	case TEngineType::ElectricSeriesMotor:
	case TEngineType::ElectricInductionMotor:
		return traction_kind::electric;
	case TEngineType::DieselEngine:
	case TEngineType::DieselElectric:
		return traction_kind::diesel;
	default:
		return traction_kind::none;
	}
}

} // namespace

bool TMoverParameters::ControlCouplingAllowed(int const End, TMoverParameters const &Other, int const OtherEnd) const
{
	if (false == Global.control_coupling_engine_check)
	{
		return true;
	}
	// the same control system named on both couplers is the vehicle authors' word that they work together
	if (auto const &controltype{Couplers[End].control_type}; false == controltype.empty() && controltype == Other.Couplers[OtherEnd].control_type)
	{
		return true;
	}
	auto const traction{traction_of(EngineType)};
	auto const othertraction{traction_of(Other.EngineType)};
	return traction == traction_kind::none || othertraction == traction_kind::none || traction == othertraction;
}

CouplingSet TMoverParameters::CompatibleCoupling(int const End, TMoverParameters const &Other, int const OtherEnd, CouplingSet const CouplingType) const
{
	if (false == CouplingType.HasAny(Coupling::control) || CouplingType.HasAny(Coupling::permanent) || ControlCouplingAllowed(End, Other, OtherEnd))
	{
		return CouplingType;
	}
	return CouplingType.Without(Coupling::control);
}

// *************************************************************************************************
// Q: 20160713
// Obliczanie sił dzialających na sprzęgach
// *************************************************************************************************
double TMoverParameters::CouplerForce(int const End, double dt)
{
	if (coupler_physics_at(End))
	{
		return coupler_force_physical(End, dt);
	}

	auto &thiscoupler{Couplers[End]};
	auto *othervehicle{Neighbours[End].vehicle->MoverParameters};
	auto const otherend{Neighbours[End].vehicle_end};
	auto const &othercoupler{othervehicle->Couplers[otherend]};

	auto const othervehiclemove{(othervehicle->dMoveLen * DirPatch(End, otherend))};
	auto const initialdistance{Neighbours[End].distance}; // odległość od sprzęgu sąsiada
	auto const distancedelta{(End == end::front ? othervehiclemove - dMoveLen : dMoveLen - othervehiclemove)};

	// the ends crushed for good (physics.collisions new, see collision.h) let the buffers meet that much closer
	auto const crushed{thiscoupler.crush + othercoupler.crush};
	auto const rawdistance{initialdistance + 10.0 * distancedelta};
	auto const newdistance{rawdistance < 0.0 ? std::min(0.0, rawdistance + crushed) : rawdistance};

	auto const dV{CouplerVelocityDifference(End, *othervehicle, otherend)};
	auto const absdV{std::abs(dV)};

	// potentially generate sounds on clash or stretch
	if (newdistance < 0.0 && thiscoupler.Dist > newdistance && dV < -0.1 && false == thiscoupler.has_adapter())
	{ // HACK: with adapter present we presume buffers won't clash
		// 090503: dzwieki pracy zderzakow
		SetFlag(thiscoupler.sounds, absdV > 5.0 ? sound::bufferclash | sound::loud : sound::bufferclash);
	}
	else if (thiscoupler.CouplingFlag != Coupling::none && newdistance > 0.001 && thiscoupler.Dist <= 0.001 && absdV > 0.005 && Vel > 1.0)
	{
		// 090503: dzwieki pracy sprzegu
		SetFlag(thiscoupler.sounds, absdV > 0.035 ? sound::couplerstretch | sound::loud : sound::couplerstretch);
	}

	thiscoupler.CheckCollision = false;
	thiscoupler.Dist = 0.0;

	double CF{0.0};

	if (thiscoupler.CouplingFlag == Coupling::none && initialdistance > 0.05)
	{ // arbitrary distance
		// potentially reset auto coupling lock
		thiscoupler.AutomaticCouplingAllowed = true;
	}

	if (thiscoupler.CouplingFlag != Coupling::none || initialdistance < 0)
	{

		thiscoupler.Dist = safe_clamp(newdistance, thiscoupler.has_adapter() ? 0 : -thiscoupler.DmaxB, thiscoupler.DmaxC);

		double BetaAvg = 0;
		double Fmax = 0;

		if (thiscoupler.CouplingFlag == Coupling::none)
		{

			BetaAvg = thiscoupler.beta;
			Fmax = (thiscoupler.FmaxC + thiscoupler.FmaxB) * CouplerTune;
		}
		else
		{
			// usrednij bo wspolny sprzeg
			BetaAvg = 0.5 * (thiscoupler.beta + othercoupler.beta);
			Fmax = 0.5 * (thiscoupler.FmaxC + thiscoupler.FmaxB + othercoupler.FmaxC + othercoupler.FmaxB) * CouplerTune;
		}
		auto const distDelta{std::abs(newdistance) - std::abs(thiscoupler.Dist)}; // McZapkie-191103: poprawka na histereze

		if (newdistance > 0)
		{

			if (distDelta > 0)
			{
				CF = -(thiscoupler.SpringKC + othercoupler.SpringKC) * thiscoupler.Dist / 2.0 * DirF(End) - Fmax * dV * BetaAvg;
			}
			else
			{
				CF = -(thiscoupler.SpringKC + othercoupler.SpringKC) * thiscoupler.Dist / 2.0 * DirF(End) * BetaAvg - Fmax * dV * BetaAvg;
			}
			// liczenie sily ze sprezystosci sprzegu
			if (newdistance > thiscoupler.DmaxC + othercoupler.DmaxC)
			{
				// zderzenie
				thiscoupler.CheckCollision = true;
			}
			CouplerStretchCheck(End, CF, dt);
		}
		if (newdistance < 0)
		{

			if (distDelta > 0)
			{
				CF = -(thiscoupler.SpringKB + othercoupler.SpringKB) * thiscoupler.Dist / 2.0 * DirF(End) - Fmax * dV * BetaAvg;
			}
			else
			{
				CF = -(thiscoupler.SpringKB + othercoupler.SpringKB) * thiscoupler.Dist / 2.0 * DirF(End) * BetaAvg - Fmax * dV * BetaAvg;
			}
			// liczenie sily ze sprezystosci zderzaka
			auto const collisiondistance{(thiscoupler.has_adapter() || othercoupler.has_adapter() ?
			                                  std::min(thiscoupler.DmaxB, othercoupler.DmaxB) : // HACK: only take into account buffering ability of automatic coupler
			                                  thiscoupler.DmaxB + othercoupler.DmaxB)};
			if (-newdistance > collisiondistance)
			{
				// zderzenie
				thiscoupler.CheckCollision = true;
			}
			if (-newdistance >= std::min(collisiondistance, dEpsilon))
			{
				automatic_coupling(End, *othervehicle, otherend);
			}
		}
	}

	if (thiscoupler.CouplingFlag != Coupling::none)
	{
		// uzgadnianie prawa Newtona, on the coupler of the other vehicle this one is coupled with, along that vehicle's
		// axis: turned around once more when the vehicles face each other
		othervehicle->Couplers[otherend].CForce = -CF * DirPatch(End, otherend);
	}

	return CF;
}

// a coupler stretched with more than its strength for over a second breaks, the more likely the longer, with crash damage on
void TMoverParameters::CouplerStretchCheck(int const End, double const CF, double dt)
{
	auto &thiscoupler{Couplers[End]};
	if (std::abs(CF) > thiscoupler.FmaxC)
	{
		// coupler is stretched with excessive force, may break
		thiscoupler.stretch_duration += static_cast<float>(dt);
		// give coupler 1 sec of leeway to account for simulation glitches, before checking whether it breaks
		// (arbitrary) chance to break grows from 10-100% over 10 sec period
		if (Global.crash_damage && thiscoupler.stretch_duration > 1.f && Random() < thiscoupler.stretch_duration * 0.1f * dt)
		{
			damage_coupler(End);
		}
	}
	else
	{
		thiscoupler.stretch_duration = 0.f;
	}
}

bool TMoverParameters::coupler_physics_at(int const End) const
{
	if (false == Global.coupler_physics)
	{
		return false;
	}
	if (Global.coupler_physics_all || CouplerPhysics)
	{
		return true;
	}
	// either vehicle of the pair covered takes the pair in, so both of them compute it the same way
	auto const *other{Neighbours[End].vehicle};
	return other != nullptr && other->MoverParameters->CouplerPhysics;
}

void TMoverParameters::automatic_coupling(int const End, TMoverParameters &Other, int const OtherEnd)
{
	auto &thiscoupler{Couplers[End]};
	auto &othercoupler{Other.Couplers[OtherEnd]};

	if (thiscoupler.type() != TCouplerType::Automatic || thiscoupler.type() != othercoupler.type() || thiscoupler.CouplingFlag != Coupling::none || false == thiscoupler.AutomaticCouplingAllowed ||
	    false == othercoupler.AutomaticCouplingAllowed)
	{
		return;
	}
	// a coupler torn off by the crush of its end (physics.collisions new) doesn't catch
	if (Global.collisions == global_settings::collision_model::physical && (thiscoupler.broken || othercoupler.broken))
	{
		return;
	}
	// sprzeganie wagonow z samoczynnymi sprzegami
	auto couplingtype{thiscoupler.AutomaticCouplingFlag & othercoupler.AutomaticCouplingFlag};
	// potentially exclude incompatible control coupling
	if (thiscoupler.control_type != othercoupler.control_type)
	{
		couplingtype = couplingtype.Without(Coupling::control);
	}
	couplingtype = CompatibleCoupling(End, Other, OtherEnd, couplingtype);

	// too fast for the couplers to catch, the vehicles collide instead (and may derail, see CollisionDetect())
	auto const safe{AutomaticCouplingSafe(End, Other, OtherEnd)};
	auto const caught{safe && Attach(End, OtherEnd, &Other, couplingtype)};
	// HACK: we're reusing sound enum to mark whether vehicle was connected to another
	AIFlag |= (caught ? sound::attachcoupler : 0);
	// coupled, or locked until the vehicles part, so they don't couple as soon as the collision slows them down
	thiscoupler.AutomaticCouplingAllowed = safe && false == caught;
	othercoupler.AutomaticCouplingAllowed = safe && false == caught;
}
namespace
{


// the gap between the ends of a coupled pair, taking up the play of the coupling, opens up enough to jerk the train
constexpr double coupler_stretch_threshold{0.001};

// the sounds of the buffers hitting each other, and of the coupling taking up its play
struct coupler_sound_input
{
	double gap{0.0}; // between the ends [m]
	double previous_gap{0.0};
	double gap_rate{0.0}; // [m/s]
	double draw_gear{0.0}; // deflection [m]
	double previous_draw_gear{0.0};
	bool coupled{false};
	bool adapter{false};
	bool moving{false};
};

int coupler_sounds(coupler_sound_input const &Input)
{
	if (Input.gap < 0.0 && Input.gap < Input.previous_gap && Input.gap_rate < -0.1 && false == Input.adapter)
	{
		// 090503: dzwieki pracy zderzakow
		return std::abs(Input.gap_rate) > 5.0 ? sound::bufferclash | sound::loud : sound::bufferclash;
	}
	if (Input.coupled && Input.moving && Input.draw_gear > coupler_stretch_threshold && Input.previous_draw_gear <= coupler_stretch_threshold && std::abs(Input.gap_rate) > 0.005)
	{
		// 090503: dzwieki pracy sprzegu
		return std::abs(Input.gap_rate) > 0.035 ? sound::couplerstretch | sound::loud : sound::couplerstretch;
	}
	return sound::none;
}

} // namespace

double TMoverParameters::coupler_force_physical(int const End, double const dt)
{
	auto &thiscoupler{Couplers[End]};
	auto *othervehicle{Neighbours[End].vehicle->MoverParameters};
	auto const otherend{Neighbours[End].vehicle_end};
	auto &othercoupler{othervehicle->Couplers[otherend]};
	auto &state{thiscoupler.physics_state};

	// the gap as it is now: the distance found at the start of the frame, and the movement of both vehicles since
	auto const othervehiclemove{othervehicle->dMoveLen * DirPatch(End, otherend)};
	auto const gap{Neighbours[End].distance + (End == end::front ? othervehiclemove - dMoveLen : dMoveLen - othervehiclemove)};
	// the gap opens up when the front end falls behind the vehicle ahead of it, or the rear end runs ahead of the one behind
	auto const gaprate{CouplerVelocityDifference(End, *othervehicle, otherend) * DirF(End)};
	auto const coupled{thiscoupler.CouplingFlag != Coupling::none};
	auto const adapter{thiscoupler.has_adapter() || othercoupler.has_adapter()};

	if (false == coupled && Neighbours[End].distance > 0.05)
	{ // arbitrary distance
		// potentially reset auto coupling lock
		thiscoupler.AutomaticCouplingAllowed = true;
	}

	auto const previousgap{state.gap};
	auto const previousdrawgear{state.draw_gear.deflection};
	auto const result{coupler_physics::update_pair(thiscoupler.physics, othercoupler.physics, state, {gap, gaprate, coupled, adapter, Global.crash_damage, thiscoupler.crush + othercoupler.crush})};

	SetFlag(thiscoupler.sounds, coupler_sounds({gap, previousgap, gaprate, state.draw_gear.deflection, previousdrawgear, coupled, adapter, Vel > 1.0}));

	thiscoupler.CheckCollision = state.stop != 0;
	// for the buffers drawn pressed in, see TDynamicObject: twice the deflection of the buffers of this end
	thiscoupler.Dist = result.compression > 0.0 ? -2.0 * result.buffers_deflection : std::max(gap, 0.0);
	// the link pulled past its yield force, which the AI drivers ease off on
	thiscoupler.stretch_duration = result.overloaded ? thiscoupler.stretch_duration + static_cast<float>(dt) : 0.f;

	if (result.broken)
	{
		damage_coupler(End);
	}
	if (false == coupled && gap < 0.0)
	{
		auto const collisiondistance{adapter ? std::min(thiscoupler.DmaxB, othercoupler.DmaxB) : thiscoupler.DmaxB + othercoupler.DmaxB};
		if (-gap >= std::min(collisiondistance, dEpsilon))
		{
			automatic_coupling(End, *othervehicle, otherend);
		}
	}

	// tension pulls this vehicle towards the other one
	auto const force{result.force * -DirF(End)};
	if (thiscoupler.CouplingFlag != Coupling::none)
	{
		// uzgadnianie prawa Newtona, as in CouplerForce()
		othercoupler.CForce = -force * DirPatch(End, otherend);
	}
	return force;
}

bool TMoverParameters::coupler_stop_hit(int const End, TMoverParameters &Other, int const OtherEnd)
{
	auto &thiscoupler{Couplers[End]};
	auto &othercoupler{Other.Couplers[OtherEnd]};
	auto &state{thiscoupler.physics_state};

	// the draw gear went solid with the gap opening up, the buffers with it closing
	auto const gaprate{CouplerVelocityDifference(End, Other, OtherEnd) * DirF(End)};
	auto const closing{state.stop > 0 ? gaprate : -gaprate};
	if (state.stop == 0 || closing <= 0.0)
	{
		// parting from the stop already, after the hit on the other vehicle's end
		return false;
	}
	if (false == Global.crash_damage)
	{
		return true;
	}
	auto const hit{coupler_physics::impact(thiscoupler.physics, othercoupler.physics, state, closing, TotalMass, Other.TotalMass)};
	// the other vehicle keeps the state of the pair alike
	othercoupler.physics_state.elongation = state.elongation;
	// with physics.collisions new the crush of the ends past the buffers is crush_hit()'s
	if (hit.broken || (hit.crushed && Global.collisions != global_settings::collision_model::physical))
	{
		damage_coupler(End);
	}
	// a broken coupling lets the vehicles part
	return false == hit.broken;
}
