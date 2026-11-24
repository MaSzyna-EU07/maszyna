#pragma once

#include "mover/core.h"
#include "mover/power_coupling.h"

namespace mover
{
struct TCoupling
{
	/*parametry*/
	double SpringKB = 1.0; /*stala sprezystosci zderzaka/sprzegu, %tlumiennosci */
	double DmaxB = 0.1; /*tolerancja scisku/rozciagania, sila rozerwania*/
	double FmaxB = 1000.0;
	double SpringKC = 1.0;
	double DmaxC = 0.1;
	double FmaxC = 1000.0;
	double beta = 0.0;
	TCouplerType CouplerType = TCouplerType::NoCoupler; /*typ sprzegu*/
	int AutomaticCouplingFlag = coupling::coupler;
	int AllowedFlag = coupling::coupler | coupling::brakehose; // Ra: maska dostępnych
	int PowerFlag = coupling::power110v | coupling::power24v;
	int PowerCoupling = coupling::permanent; // type of coupling required for power transfer
	/*zmienne*/
	bool AutomaticCouplingAllowed{true}; // whether automatic coupling can be currently performed
	int CouplingFlag = 0; /*0 - wirtualnie, 1 - sprzegi, 2 - pneumatycznie, 4 - sterowanie, 8 - kabel mocy*/
	class TMoverParameters *Connected = nullptr; /*co jest podlaczone*/
	int ConnectedNr = 0; // Ra: od której strony podłączony do (Connected): 0=przód, 1=tył
	double CForce = 0.0; /*sila z jaka dzialal*/
	double Dist = 0.0; /*strzalka ugiecia zderzaków*/
	bool CheckCollision = false; /*czy sprawdzac sile czy pedy*/
	float stretch_duration{0.f}; // seconds, elapsed time with excessive force applied to the coupler
	bool broken{false}; // the coupler broke (damage_coupler()), until it's repaired (repair_coupler())
	// optional adapter piece
	double adapter_length{0.0}; // meters, value added on the given end to standard vehicle (half)length
	double adapter_height{0.0}; // meters, distance from rail level
	TCouplerType adapter_type = TCouplerType::NoCoupler; // CouplerType override if other than NoCoupler

	power_coupling power_high;
	power_coupling power_110v;
	power_coupling power_24v;

	// the physical model of the coupling (couplerphysics in eu07.ini), see couplerphysics.h: the parameters of this end,
	// and the state of the coupling with the vehicle at this end, which that vehicle keeps alike on its own end
	coupler_physics::end_parameters physics;
	coupler_physics::pair_state physics_state;

	int sounds{0}; // sounds emitted by the coupling devices
	bool Render = false; /*ABu: czy rysowac jak zaczepiony sprzeg*/
	std::string control_type; // abstraction of control coupling interface and communication standard

	inline bool has_adapter() const
	{
		return adapter_type != TCouplerType::NoCoupler;
	}
	inline TCouplerType type() const
	{
		return adapter_type == TCouplerType::NoCoupler ? CouplerType : adapter_type;
	}
};

} // namespace mover
