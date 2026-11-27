/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the vocabulary of the vehicles: their ends, sides, couplings, lights, sounds, types of drive and brake, the
// flags and the small structures the rest of the simulator passes around without needing the physics itself

#pragma once

#include <map>
#include <string>

#include "physics/vehiclecommand.h"

/// <summary>Global counter incremented when a string-to-numeric conversion fails during config parsing.</summary>
extern int ConversionError;

/// <summary>Static friction coefficient for steel-on-steel contact (used for adhesion at standstill).</summary>
const double Steel2Steel_friction = 0.15; // tarcie statyczne
/// <summary>Earth gravitational acceleration [m/s^2].</summary>
const double g = 9.81; // przyspieszenie ziemskie
/// <summary>Sand consumption rate when sanding is active [kg/s].</summary>
const double SandSpeed = 0.1; // ile kg/s}
/// <summary>2*pi shortcut.</summary>
const double Pirazy2 = 6.2831853071794f;

//-- var, const, procedure ---------------------------------------------------
/// <summary>Truthy alias used by CheckLocomotiveParameters() to ignore a check failure.</summary>
static bool const Go = true;
/// <summary>Falsy alias used by CheckLocomotiveParameters() to abort on a check failure.</summary>
static bool const Hold = false; /*dla CheckLocomotiveParameters*/
/// <summary>Maximum number of resistor / scheme entries for electric motor traction.</summary>
static int const ResArraySize = 64; /*dla silnikow elektrycznych*/
/// <summary>Maximum number of motor-parameter table entries (gear stages).</summary>
static int const MotorParametersArraySize = 10;
/// <summary>Maximum number of pantograph current collectors per vehicle.</summary>
static int const maxcc = 4; /*max. ilosc odbierakow pradu*/
// static int const LocalBrakePosNo = 10;         /*ilosc nastaw hamulca pomocniczego*/
// static int const MainBrakeMaxPos = 10;          /*max. ilosc nastaw hamulca zasadniczego*/
/// <summary>Number of positions on the manual (parking) brake handle.</summary>
static int const ManualBrakePosNo = 20; /*ilosc nastaw hamulca recznego*/
/// <summary>Number of positions on the lights selector switch.</summary>
static int const LightsSwitchPosNo = 16;
/// <summary>Maximum number of positions of the universal (auxiliary) controller.</summary>
static int const UniversalCtrlArraySize = 32; /*max liczba pozycji uniwersalnego nastawnika*/

/*uszkodzenia toru*/
/// <summary>Track damage flag: rail wear.</summary>
static int const dtrack_railwear = 2;
/// <summary>Track damage flag: free / loose rail.</summary>
static int const dtrack_freerail = 4;
/// <summary>Track damage flag: thin (worn-out) rail.</summary>
static int const dtrack_thinrail = 8;
/// <summary>Track damage flag: bent rail.</summary>
static int const dtrack_railbend = 16;
/// <summary>Track damage flag: vegetation overgrowth.</summary>
static int const dtrack_plants = 32;
/// <summary>Track damage flag: track impassable for movement.</summary>
static int const dtrack_nomove = 64;
/// <summary>Track damage flag: rails missing entirely.</summary>
static int const dtrack_norail = 128;

/*uszkodzenia taboru*/
/// <summary>Vehicle damage flag: thin wheel (locomotives).</summary>
static int const dtrain_thinwheel = 1; /*dla lokomotyw*/
/// <summary>Vehicle damage flag: shifted load (wagons).</summary>
static int const dtrain_loadshift = 1; /*dla wagonow*/
/// <summary>Vehicle damage flag: wheel wear.</summary>
static int const dtrain_wheelwear = 2;
/// <summary>Vehicle damage flag: bearing failure.</summary>
static int const dtrain_bearing = 4;
/// <summary>Vehicle damage flag: coupling damage.</summary>
static int const dtrain_coupling = 8;
/// <summary>Vehicle damage flag: ventilator failure (electric locomotive).</summary>
static int const dtrain_ventilator = 16; /*dla lokomotywy el.*/
/// <summary>Vehicle damage flag: load damage (wagons).</summary>
static int const dtrain_loaddamage = 16; /*dla wagonow*/
/// <summary>Vehicle damage flag: engine failure (locomotives).</summary>
static int const dtrain_engine = 32; /*dla lokomotyw*/
/// <summary>Vehicle damage flag: load destroyed (wagons).</summary>
static int const dtrain_loaddestroyed = 32; /*dla wagonow*/
/// <summary>Vehicle damage flag: axle damage.</summary>
static int const dtrain_axle = 64;
/// <summary>Vehicle damage flag: derailment.</summary>
static int const dtrain_out = 128; /*wykolejenie*/
/// <summary>Vehicle damage flag: broken pantograph.</summary>
static int const dtrain_pantograph = 256; /*polamanie pantografu*/

/// <summary>Reason for a derailment event.</summary>
enum DerailReason
{
	/// <summary>No derailment.</summary>
	NONE = 0,
	/// <summary>End of track reached.</summary>
	END_OF_TRACK = 1, // Ra: powód wykolejenia: brak szyn
	/// <summary>Speed too high for curve (overturn).</summary>
	TOO_HIGH_SPEED = 2, // Ra: powód wykolejenia: przewrócony na łuku
	/// <summary>Track gauge does not match wheelset gauge.</summary>
	GAUGE_MISMATCH = 3, // Ra: powód wykolejenia: za szeroki tor
	/// <summary>Track type incompatible with this vehicle.</summary>
	WRONG_TRACK_TYPE = 4, // Ra: powód wykolejenia: nieodpowiednia trajektoria
	/// <summary>Collision with another vehicle.</summary>
	COLLISION = 5, //     powód wykolejenia: zderzenie z innym pojazdem
	/// <summary>Trailing through a switch with a movable nose set the other way.</summary>
	TRAILED_SWITCH = 6, // powód wykolejenia: rozprucie rozjazdu z ruchomym dziobem
	/// <summary>Climbed onto the vehicle it hit, its end crushed (physics.collisions new).</summary>
	COLLISION_OVERRIDE = 7, // najechanie
	/// <summary>Pushed off the track by the force pressing its end (physics.collisions new).</summary>
	COLLISION_PUSHED_OFF = 8, // wyciśnięcie z toru
};

/*wagi prawdopodobienstwa dla funkcji FuzzyLogic*/
/// <summary>Fuzzy-logic probability weight: electric engine general problem.</summary>
#define p_elengproblem (1e-02)
/// <summary>Fuzzy-logic probability weight: electric engine damage.</summary>
#define p_elengdamage (1e-01)
/// <summary>Fuzzy-logic probability weight: coupler damage.</summary>
#define p_coupldmg (2e-03)
/// <summary>Fuzzy-logic probability weight: derailment.</summary>
#define p_derail (1e-03)
/// <summary>Fuzzy-logic probability weight: acceleration-related event.</summary>
#define p_accn (1e-01)
/// <summary>Fuzzy-logic probability weight: damage caused by wheel slip.</summary>
#define p_slippdmg (1e-03)

/// <summary>Vehicle ends — front or rear (mutually exclusive).</summary>
enum end
{
	/// <summary>Front end.</summary>
	front = 0,
	/// <summary>Rear end.</summary>
	rear = 1
};

/// <summary>Vehicle sides — right or left (mutually exclusive).</summary>
enum side
{
	/// <summary>Right side.</summary>
	right = 0,
	/// <summary>Left side.</summary>
	left = 1
};

/// <summary>The kinds of connection a coupling between two vehicles makes. The numbers are the ones of the scenery
/// files and the vehicle definitions (AllowedFlag, the coupling of a vehicle in a train), where they add up.</summary>
enum class Coupling : unsigned
{
	none = 0x0, // the vehicles only see each other (virtual)
	coupler = 0x1, // the mechanical coupler
	brakeHose = 0x2, // the brake pipe
	control = 0x4, // the control cable of the multiple unit
	highVoltage = 0x8, // the high voltage cable of the traction supply
	gangway = 0x10, // the gangway for the passengers
	mainHose = 0x20, // the main reservoir pipe (8 bar, yellow)
	heating = 0x40, // the high voltage cable of the train heating
	permanent = 0x80, // not parted by usual shunting (between the sections of a unit); negative in a definition
	power24v = 0x100, // the 24 V cable
	power110v = 0x200, // the 110 V cable
	power3x400v = 0x400, // the 3x400 V cable
};

/// <summary>A set of the kinds of connection of a coupling: what a coupling joins, what a vehicle allows, what a
/// command needs to pass. Read from and written to the numbers of the files only through FromBits() and Bits().</summary>
class CouplingSet
{
public:
	constexpr CouplingSet() = default;
	constexpr CouplingSet(Coupling const kind) : bits{static_cast<unsigned>(kind)} {} // NOSONAR: a kind is a set of one
	// the set the number of a file gives (the sum of the numbers of the kinds)
	static constexpr CouplingSet FromBits(int const number)
	{
		CouplingSet set;
		set.bits = static_cast<unsigned>(number);
		return set;
	}
	// every kind of coupling: a coupling of any kind has one of them
	static constexpr CouplingSet All()
	{
		return FromBits(-1);
	}
	// any kind at all: as the old number was true when it wasn't zero
	constexpr explicit operator bool() const
	{
		return bits != 0;
	}
	// the number of the set, for the files and the log
	constexpr int Bits() const
	{
		return static_cast<int>(bits);
	}
	constexpr bool Empty() const
	{
		return bits == 0;
	}
	// all the kinds of the other set are in this one
	constexpr bool HasAll(CouplingSet const other) const
	{
		return (bits & other.bits) == other.bits;
	}
	// any of the kinds of the other set is in this one
	constexpr bool HasAny(CouplingSet const other) const
	{
		return (bits & other.bits) != 0;
	}
	// adds the kinds; returns whether any of them was missing
	constexpr bool Set(CouplingSet const other)
	{
		if (HasAll(other))
		{
			return false;
		}
		bits |= other.bits;
		return true;
	}
	// takes the kinds away, where all of them are there; returns whether it did
	constexpr bool Clear(CouplingSet const other)
	{
		if (false == HasAll(other))
		{
			return false;
		}
		bits &= ~other.bits;
		return true;
	}
	// this set without the kinds of the other
	constexpr CouplingSet Without(CouplingSet const other) const
	{
		return FromBits(static_cast<int>(bits & ~other.bits));
	}
	constexpr CouplingSet &operator|=(CouplingSet const other)
	{
		bits |= other.bits;
		return *this;
	}
	constexpr CouplingSet &operator&=(CouplingSet const other)
	{
		bits &= other.bits;
		return *this;
	}
	friend constexpr CouplingSet operator|(CouplingSet left, CouplingSet const right)
	{
		return left |= right;
	}
	friend constexpr CouplingSet operator&(CouplingSet left, CouplingSet const right)
	{
		return left &= right;
	}
	// the kinds in one of the sets and not in the other: what changed between them
	constexpr CouplingSet &operator^=(CouplingSet const other)
	{
		bits ^= other.bits;
		return *this;
	}
	friend constexpr CouplingSet operator^(CouplingSet left, CouplingSet const right)
	{
		return left ^= right;
	}
	friend constexpr bool operator==(CouplingSet const &left, CouplingSet const &right) = default;

private:
	unsigned bits{0};
};

constexpr CouplingSet operator|(Coupling const left, Coupling const right)
{
	return CouplingSet{left} | CouplingSet{right};
}
/// <summary>Range of effect for a control command (mutually exclusive).</summary>
enum class range_t
{
	/// <summary>Affects only this vehicle.</summary>
	local,
	/// <summary>Affects this multiple-unit (vehicles permanently coupled into a unit).</summary>
	unit,
	/// <summary>Affects the entire consist.</summary>
	consist
};
/// <summary>State of the door closure signalling (SOD) of a vehicle or a consist, ordered from the least to the most
/// severe: the state of a consist is the most severe state of its vehicles.</summary>
enum class door_closure_t
{
	/// <summary>No doors worked by the crew: the vehicle doesn't take part in the signalling.</summary>
	none,
	/// <summary>All doors closed and locked: they can't be opened.</summary>
	locked,
	/// <summary>All doors closed, but released: a permit is given, or no lock holds them shut.</summary>
	released,
	/// <summary>A door is open, or not fully closed yet.</summary>
	open
};
/// <summary>Whether the door closure signalling lamp is lit: steadily for locked doors, in the lit phase of the
/// blinking for released ones, never with a door open or with no doors signalled.</summary>
bool door_closure_lamp(door_closure_t const State, bool const BlinkLit);
/// <summary>Possible settings for an enable/disable input pair (mutually exclusive).</summary>
enum class operation_t
{
	/// <summary>No input.</summary>
	none = 0,
	/// <summary>Enable input held active.</summary>
	enable,
	/// <summary>Disable input held active.</summary>
	disable,
	/// <summary>Enable button press transition (rising edge).</summary>
	enable_on,
	/// <summary>Enable button release transition.</summary>
	enable_off,
	/// <summary>Disable button press transition.</summary>
	disable_on,
	/// <summary>Disable button release transition.</summary>
	disable_off,
};
/// <summary>What else the line contactors of a vehicle need before they can close (LineContactorsInterlock in the Cntrl. section).</summary>
enum class line_contactors_interlock
{
	/// <summary>Nothing more, the legacy behaviour.</summary>
	none,
	/// <summary>The converter of the vehicle itself running, where it drives the traction motor fans (ET42).</summary>
	converter,
	/// <summary>The traction motor blowers of the vehicle running.</summary>
	motor_blowers
};
/// <summary>Auto-start method for ancillary devices (mutually exclusive).</summary>
enum class start_t
{
	/// <summary>Device cannot be started.</summary>
	disabled,
	/// <summary>Manual start only.</summary>
	manual,
	/// <summary>Starts automatically.</summary>
	automatic,
	/// <summary>Manual start with automatic fallback.</summary>
	manualwithautofallback,
	/// <summary>Driven by the converter state.</summary>
	converter,
	/// <summary>Driven by the battery state.</summary>
	battery,
	/// <summary>Driven by direction selector state.</summary>
	direction
};
/// <summary>Bit flags representing vehicle external lights (positions and types). Combinable.</summary>
enum light
{

	/// <summary>Headlight, left.</summary>
	headlight_left = 1 << 0,
	/// <summary>Red marker, left.</summary>
	redmarker_left = 1 << 1,
	/// <summary>Headlight, upper centre.</summary>
	headlight_upper = 1 << 2,
	// TBD, TODO: redmarker_upper support?
	/// <summary>Headlight, right.</summary>
	headlight_right = 1 << 4,
	/// <summary>Red marker, right.</summary>
	redmarker_right = 1 << 5,
	/// <summary>Rear-end markers (combined).</summary>
	rearendsignals = 1 << 6,
	/// <summary>Auxiliary light, left (e.g. ditch light).</summary>
	auxiliary_left = 1 << 7,
	/// <summary>Auxiliary light, right.</summary>
	auxiliary_right = 1 << 8,
	/// <summary>High-beam headlight, left.</summary>
	highbeamlight_left = 1 << 9,
	/// <summary>High-beam headlight, right.</summary>
	highbeamlight_right = 1 << 10
};

/// <summary>The cab interior light a position of the light preset selector sets (eu07/maszyna#529).</summary>
struct lights_preset_cablight
{
	/// <summary>Bits of the optional third column of a LightsList row.</summary>
	enum flag
	{
		/// <summary>The cab light is on.</summary>
		light_on = 1 << 0,
		/// <summary>The cab light is dimmed.</summary>
		light_dimmed = 1 << 1
	};
	bool light{false};
	bool dim{false};
};

/// <summary>Door operation method (who controls them and how) — mutually exclusive.</summary>
enum control_t
{
	/// <summary>Local; passengers operate manually, doors open/close for the duration of loading.</summary>
	passenger, // local, opened/closed for duration of loading
	/// <summary>Remote; driver-operated.</summary>
	driver, // remote, operated by the driver
	/// <summary>Local autonomous; close when the vehicle moves and/or after a timeout.</summary>
	autonomous, // local, closed when vehicle moves and/or after timeout
	/// <summary>Remote; conductor-operated.</summary>
	conductor, // remote, operated by the conductor
	/// <summary>Primarily manual but also responds to remote control.</summary>
	mixed // primary manual but answers also to remote control
};
/*typ hamulca elektrodynamicznego*/
/// <summary>Electrodynamic-brake type: no ED brake.</summary>
static int const dbrake_none = 0;
/// <summary>Electrodynamic-brake type: passive (always on while moving).</summary>
static int const dbrake_passive = 1;
/// <summary>Electrodynamic-brake type: switchable (driver toggles).</summary>
static int const dbrake_switch = 2;
/// <summary>Electrodynamic-brake type: reversal (engaged via reverser).</summary>
static int const dbrake_reversal = 4;
/// <summary>Electrodynamic-brake type: automatic (engages from main brake).</summary>
static int const dbrake_automatic = 8;

/*dzwieki*/
/// <summary>Bit flags identifying sound events emitted by a vehicle. Combinable.</summary>
enum sound
{
	/// <summary>No sound event.</summary>
	none,
	/// <summary>Generic "loud" event flag (e.g. impact / forced action).</summary>
	loud = 1 << 0,
	/// <summary>Coupler stretching to its limit.</summary>
	couplerstretch = 1 << 1,
	/// <summary>Buffer clash (vehicles colliding).</summary>
	bufferclash = 1 << 2,
	/// <summary>Contactor / relay clicking.</summary>
	relay = 1 << 3,
	/// <summary>Series-parallel switch operating.</summary>
	parallel = 1 << 4,
	/// <summary>Field-shunt contactor operating.</summary>
	shuntfield = 1 << 5,
	/// <summary>Generic pneumatic event.</summary>
	pneumatic = 1 << 6,
	/// <summary>Coupling detached.</summary>
	detach = 1 << 7,
	/// <summary>Mechanical coupler attached.</summary>
	attachcoupler = 1 << 8,
	/// <summary>Brake hose attached.</summary>
	attachbrakehose = 1 << 9,
	/// <summary>Main air hose attached.</summary>
	attachmainhose = 1 << 10,
	/// <summary>Control cable attached.</summary>
	attachcontrol = 1 << 11,
	/// <summary>Gangway attached.</summary>
	attachgangway = 1 << 12,
	/// <summary>Heating cable attached.</summary>
	attachheating = 1 << 13,
	/// <summary>Adapter piece attached to coupler.</summary>
	attachadapter = 1 << 14,
	/// <summary>Adapter piece removed from coupler.</summary>
	removeadapter = 1 << 15,
	/// <summary>Door-open permission signalling.</summary>
	doorpermit = 1 << 16,
};

/// <summary>Bit flags for the customizable reset button — which protective relay it resets. Combinable.</summary>
enum relay_t
{
	/// <summary>Main circuit ground-fault relay.</summary>
	maincircuitground = 1 << 0,
	/// <summary>Auxiliary circuit ground-fault relay.</summary>
	auxiliarycircuitground = 1 << 1,
	/// <summary>Traction motor overload relay.</summary>
	tractionnmotoroverload = 1 << 2,
	/// <summary>Primary converter overload relay.</summary>
	primaryconverteroverload = 1 << 3,
	/// <summary>Secondary converter overload relay.</summary>
	secondaryconverteroverload = 1 << 4,
	/// <summary>Ventilator overload relay.</summary>
	ventillatoroverload = 1 << 5,
	/// <summary>Heating overload relay.</summary>
	heatingoverload = 1 << 6,
	/// <summary>Electrodynamic-brake overload relay.</summary>
	electrodynamicbrakesoverload = 1 << 7,
};

/// <summary>Bit flags for actions triggered on cab activation / deactivation. Combinable.</summary>
enum activation
{
	/// <summary>Engage the emergency brake.</summary>
	emergencybrake = 1 << 0,
	/// <summary>Operate cab mirrors.</summary>
	mirrors = 1 << 1,
	/// <summary>Raise the pantographs.</summary>
	pantographsup = 1 << 2,
	/// <summary>Switch on red end-of-train markers.</summary>
	redmarkers = 1 << 3,
	/// <summary>Set door-open permit.</summary>
	doorpermition = 1 << 4,
	/// <summary>Engage the spring brake.</summary>
	springbrakeon = 1 << 5,
	/// <summary>Release the spring brake.</summary>
	springbrakeoff = 1 << 6,
	/// <summary>Place the reverser in neutral.</summary>
	neutraldirection = 1 << 7,
};

/// <summary>The vehicles the code handles in a way of their own, by the Type of the Param section of the .fiz
/// (TrainType). A vehicle is of one of them; a .fiz type not listed is generic.</summary>
enum class train_type
{
	/// <summary>No handling of its own.</summary>
	generic,
	/// <summary>Electric multiple unit (Type=EZT).</summary>
	emu,
	/// <summary>Diesel multiple unit (Type=DMU).</summary>
	dmu,
	/// <summary>ET22 locomotive (Type=ET22, since the Megapack).</summary>
	et22,
	/// <summary>ET40 locomotive.</summary>
	et40,
	/// <summary>ET41 locomotive.</summary>
	et41,
	/// <summary>ET42 locomotive.</summary>
	et42,
	/// <summary>EP05 locomotive.</summary>
	ep05,
	/// <summary>SN61 railcar: the type is read, but nothing handles it on its own.</summary>
	sn61,
	/// <summary>Pseudo-diesel (an electric drivetrain with the input model of a diesel).</summary>
	pseudodiesel,
	/// <summary>Škoda 181 and 182 locomotives (Type=181 or 182).</summary>
	skoda181,
};

// stałe dla asynchronów
/// <summary>EIM config index: dfic — frequency rise rate at start.</summary>
static int const eimc_s_dfic = 0;
/// <summary>EIM config index: dfmax — maximum frequency rate of change.</summary>
static int const eimc_s_dfmax = 1;
/// <summary>EIM config index: p — number of pole pairs.</summary>
static int const eimc_s_p = 2;
/// <summary>EIM config index: cfu — voltage / frequency conversion coefficient.</summary>
static int const eimc_s_cfu = 3;
/// <summary>EIM config index: cim — current / current conversion coefficient.</summary>
static int const eimc_s_cim = 4;
/// <summary>EIM config index: icif — magnetisation current ratio.</summary>
static int const eimc_s_icif = 5;
/// <summary>EIM config index: Uzmax — maximum DC link voltage.</summary>
static int const eimc_f_Uzmax = 7;
/// <summary>EIM config index: Uzh — DC link voltage at field-weakening transition.</summary>
static int const eimc_f_Uzh = 8;
/// <summary>EIM config index: DU — voltage drop at the converter.</summary>
static int const eimc_f_DU = 9;
/// <summary>EIM config index: I0 — no-load magnetising current.</summary>
static int const eimc_f_I0 = 10;
/// <summary>EIM config index: cfu — V/f gain in normal mode.</summary>
static int const eimc_f_cfu = 11;
/// <summary>EIM config index: cfuH — V/f gain in field-weakening region.</summary>
static int const eimc_f_cfuH = 12;
/// <summary>EIM config index: F0 — starting force at zero speed.</summary>
static int const eimc_p_F0 = 13;
/// <summary>EIM config index: a1 — force-frequency slope coefficient.</summary>
static int const eimc_p_a1 = 14;
/// <summary>EIM config index: Pmax — maximum power.</summary>
static int const eimc_p_Pmax = 15;
/// <summary>EIM config index: Fh — force at field-weakening point.</summary>
static int const eimc_p_Fh = 16;
/// <summary>EIM config index: Ph — power at field-weakening point.</summary>
static int const eimc_p_Ph = 17;
/// <summary>EIM config index: Vh0 — speed where field weakening begins.</summary>
static int const eimc_p_Vh0 = 18;
/// <summary>EIM config index: Vh1 — speed of full field weakening.</summary>
static int const eimc_p_Vh1 = 19;
/// <summary>EIM config index: Imax — maximum motor current.</summary>
static int const eimc_p_Imax = 20;
/// <summary>EIM config index: abed — anti-skid set point.</summary>
static int const eimc_p_abed = 21;
/// <summary>EIM config index: eped — EP brake intensity bias.</summary>
static int const eimc_p_eped = 22;

// zmienne dla asynchronów
/// <summary>EIM variable index: FMAXMAX — absolute maximum allowable force.</summary>
static int const eimv_FMAXMAX = 0;
/// <summary>EIM variable index: Fmax — current force limit.</summary>
static int const eimv_Fmax = 1;
/// <summary>EIM variable index: ks — slip coefficient.</summary>
static int const eimv_ks = 2;
/// <summary>EIM variable index: df — frequency offset.</summary>
static int const eimv_df = 3;
/// <summary>EIM variable index: fp — base frequency.</summary>
static int const eimv_fp = 4;
/// <summary>EIM variable index: U — applied voltage.</summary>
static int const eimv_U = 5;
/// <summary>EIM variable index: pole — pole-pair indicator.</summary>
static int const eimv_pole = 6;
/// <summary>EIM variable index: Ic — control current.</summary>
static int const eimv_Ic = 7;
/// <summary>EIM variable index: If — field current.</summary>
static int const eimv_If = 8;
/// <summary>EIM variable index: M — torque.</summary>
static int const eimv_M = 9;
/// <summary>EIM variable index: Fr — actual force.</summary>
static int const eimv_Fr = 10;
/// <summary>EIM variable index: Ipoj — line current.</summary>
static int const eimv_Ipoj = 11;
/// <summary>EIM variable index: Pm — mechanical power.</summary>
static int const eimv_Pm = 12;
/// <summary>EIM variable index: Pe — electrical power.</summary>
static int const eimv_Pe = 13;
/// <summary>EIM variable index: eta — efficiency.</summary>
static int const eimv_eta = 14;
/// <summary>EIM variable index: fkr — critical frequency.</summary>
static int const eimv_fkr = 15;
/// <summary>EIM variable index: Uzsmax — maximum measured DC link voltage.</summary>
static int const eimv_Uzsmax = 16;
/// <summary>EIM variable index: Pmax — instantaneous power limit.</summary>
static int const eimv_Pmax = 17;
/// <summary>EIM variable index: Fzad — set-point force.</summary>
static int const eimv_Fzad = 18;
/// <summary>EIM variable index: Imax — maximum allowed current.</summary>
static int const eimv_Imax = 19;
/// <summary>EIM variable index: Fful — full force.</summary>
static int const eimv_Fful = 20;

/// <summary>Brake-of-mode flag: PS (pneumatic, primary).</summary>
static int const bom_PS = 1;
/// <summary>Brake-of-mode flag: PN (pneumatic, no electrodynamic).</summary>
static int const bom_PN = 2;
/// <summary>Brake-of-mode flag: EP (electro-pneumatic).</summary>
static int const bom_EP = 4;
/// <summary>Brake-of-mode flag: MED (mixed pneumatic+ED).</summary>
static int const bom_MED = 8;

/// <summary>Bit flags reporting vehicle problems that prevent driving.</summary>
enum TProblem // lista problemów taboru, które uniemożliwiają jazdę
{ // flagi bitowe
  /// <summary>The vehicle has the brake applied or seized axles.</summary>
	pr_Hamuje = 1, // pojazd ma załączony hamulec lub zatarte osie
	               /// <summary>The vehicle requires pantographs to be inflated/raised.</summary>
	pr_Pantografy = 2, // pojazd wymaga napompowania pantografów
	                   /// <summary>Reserved sentinel — last bit flag.</summary>
	pr_Ostatni = 0x80000000 // ostatnia flaga bitowa
};

/// <summary>Compressor parameter list — indices into the compressor programmer table.</summary>
enum TCompressorList // lista parametrów w programatorze sprężarek
{ // pozycje kolejne
  /// <summary>Compressor enable permission (0/1).</summary>
	cl_Allow = 0, // zezwolenie na pracę sprężarek
	              /// <summary>Compressor output multiplier.</summary>
	cl_SpeedFactor = 1, // mnożnik wydajności
	                    /// <summary>Pressure-switch lower threshold multiplier.</summary>
	cl_MinFactor = 2, // mnożnik progu załącznika ciśnieniowego
	                  /// <summary>Pressure-switch upper threshold multiplier.</summary>
	cl_MaxFactor = 3 // mnożnik progu wyłącznika ciśnieniowego
};

/// <summary>3-D position in world coordinates [m].</summary>
struct TLocation
{
	/// <summary>X coordinate.</summary>
	double X;
	/// <summary>Y coordinate (vertical).</summary>
	double Y;
	/// <summary>Z coordinate.</summary>
	double Z;
};

/// <summary>Euler-angle rotation (radians) around X / Y / Z axes.</summary>
struct TRotation
{
	/// <summary>Rotation around X axis.</summary>
	double Rx;
	/// <summary>Rotation around Y axis.</summary>
	double Ry;
	/// <summary>Rotation around Z axis.</summary>
	double Rz;
};

/// <summary>Vehicle bounding box (width × length × height [m]).</summary>
struct TDimension
{
	/// <summary>Width.</summary>
	double W = 0.0;
	/// <summary>Length.</summary>
	double L = 0.0;
	/// <summary>Height.</summary>
	double H = 0.0;
};

/// <summary>A pending command queued for the vehicle, with its arguments and propagation rules.</summary>
struct TCommand
{
	/// <summary>The command.</summary>
	VehicleCommand Command{VehicleCommand::none};
	/// <summary>The command as it came, where it came as text (an event, a script): the AI reads it.</summary>
	std::string Text;
	/// <summary>The name of the load of Load= and UnLoad=.</summary>
	std::string Argument;
	/// <summary>First numeric argument.</summary>
	double Value1 = 0.0; /*argumenty komendy*/
	/// <summary>Second numeric argument.</summary>
	double Value2 = 0.0;
	/// <summary>Coupling flag controlling how the command propagates between vehicles.</summary>
	CouplingSet couplings{Coupling::control}; // the couplings the command passes through
	/// <summary>World-space location associated with the command.</summary>
	TLocation Location;
};

/*tory*/
/// <summary>Geometric shape of a track segment under the vehicle.</summary>
struct TTrackShape
{ /*ksztalt odcinka*/
	/// <summary>Curvature radius [m]; 0 means straight track.</summary>
	double R = 0.0; // promien
	/// <summary>Segment length [m].</summary>
	double Len = 0.0; // dlugosc
	/// <summary>Track gradient (rise/run).</summary>
	double dHtrack = 0.0; // nachylenie
	/// <summary>Track cant / superelevation.</summary>
	double dHrail = 0.0; // przechylka
	/// <summary>Lateral curvature of the track unevenness under the vehicle [1/m], see world/TrackUneven.h.</summary>
	double uneven_curvature = 0.0;
	/// <summary>Cross level unevenness of the track [mm].</summary>
	double uneven_cross = 0.0;
	/// <summary>Curvature a buckle of the track adds under the body [1/m], positive with the curve, see world/TrackBuckle.h.</summary>
	double buckle_curvature = 0.0;
};

/// <summary>Per-segment track parameters: gauge, friction, category, load capacity, damage.</summary>
struct TTrackParam
{ /*parametry odcinka - szerokosc, tarcie statyczne, kategoria,  obciazalnosc w t/os, uszkodzenia*/
	/// <summary>Track gauge / width [m].</summary>
	double Width = 0.0;
	/// <summary>Static friction coefficient on this segment.</summary>
	double friction = 0.0;
	/// <summary>Track category bit flags.</summary>
	int CategoryFlag = 0;
	/// <summary>Track-quality bit flags.</summary>
	int QualityFlag = 0;
	/// <summary>Track-damage bit flags (dtrack_*).</summary>
	int DamageFlag = 0;
	/// <summary>Maximum allowed velocity on this segment (used by AI driver).</summary>
	double Velmax; /*dla uzytku maszynisty w ai_driver*/
	/// <summary>Name of the track, for the log; nullptr where there is none.</summary>
	std::string const *Name = nullptr;
};

/// <summary>Traction-supply parameters at the vehicle's current location.</summary>
struct TTractionParam
{
	/// <summary>Catenary voltage [V].</summary>
	double TractionVoltage = 0.0; /*napiecie*/
	/// <summary>Catenary frequency [Hz] (0 for DC).</summary>
	double TractionFreq = 0.0; /*czestotliwosc*/
	/// <summary>Maximum current capacity [A].</summary>
	double TractionMaxCurrent = 0.0; /*obciazalnosc*/
	/// <summary>Pantograph-catenary contact resistance [Ω].</summary>
	double TractionResistivity = 0.0; /*rezystancja styku*/
};
/*powyzsze parametry zwiazane sa z torem po ktorym aktualnie pojazd jedzie*/

// what a horn needs to sound: air from the main reservoir, or the low voltage
enum class horn_supply
{
	air,
	electric
};

/// <summary>High-level brake-system family.</summary>
enum class TBrakeSystem
{
	/// <summary>Individual (vehicle-only) brake.</summary>
	Individual,
	/// <summary>Pneumatic brake (Westinghouse-style).</summary>
	Pneumatic,
	/// <summary>Electro-pneumatic brake.</summary>
	ElectroPneumatic
};
/// <summary>Brake-system subtype identifying the manufacturer's distributor family.</summary>
enum class TBrakeSubSystem
{
	ss_None,
	ss_W,
	ss_K,
	ss_KK,
	ss_Hik,
	ss_ESt,
	ss_KE,
	ss_LSt,
	ss_MT,
	ss_Dako
};
/// <summary>Specific brake-distributor (valve) model.</summary>
enum class TBrakeValve
{
	NoValve,
	W,
	W_Lu_VI,
	W_Lu_L,
	W_Lu_XR,
	K,
	Kg,
	Kp,
	Kss,
	Kkg,
	Kkp,
	Kks,
	Hikg1,
	Hikss,
	Hikp1,
	KE,
	SW,
	EStED,
	NESt3,
	ESt3,
	LSt,
	ESt4,
	ESt3AL2,
	EP1,
	EP2,
	M483,
	CV1_L_TR,
	CV1,
	CV1_R,
	Other
};
/// <summary>Specific driver's brake-handle model (cab valve).</summary>
enum class TBrakeHandle
{
	NoHandle,
	West,
	FV4a,
	M394,
	M254,
	FVE408,
	FVel6,
	D2,
	Knorr,
	FD1,
	BS2,
	testH,
	St113,
	MHZ_P,
	MHZ_T,
	MHZ_EN57,
	MHZ_K5P,
	MHZ_K8P,
	MHZ_6P
};
/// <summary>Type of the auxiliary (independent) brake.</summary>
enum class TLocalBrake
{
	NoBrake,
	ManualBrake,
	PneumaticBrake,
	HydraulicBrake
};
/// <summary>Brake delay parameter table (apply/release for passenger and freight).</summary>
using TBrakeDelayTable = double[4];

/// <summary>One row of the brake pressure / pipe pressure / flow-speed table for a brake handle position.</summary>
struct TBrakePressure
{
	/// <summary>Brake pipe pressure target [bar].</summary>
	double PipePressureVal = 0.0;
	/// <summary>Brake cylinder pressure target [bar].</summary>
	double BrakePressureVal = 0.0;
	/// <summary>Flow speed (orifice scaling).</summary>
	double FlowSpeedVal = 0.0;
	/// <summary>Brake system this entry applies to.</summary>
	TBrakeSystem BrakeType = TBrakeSystem::Pneumatic;
};

/// <summary>Brake pressure table indexed by handle position.</summary>
using TBrakePressureTable = std::map<int, TBrakePressure>;

/// <summary>Engine / drive type.</summary>
enum class TEngineType
{
	None,
	Dumb,
	WheelsDriven,
	ElectricSeriesMotor,
	ElectricInductionMotor,
	DieselEngine,
	SteamEngine,
	DieselElectric,
	Main
};
/// <summary>Form of energy supplied by the power source.</summary>
enum class TPowerType
{
	NoPower,
	BioPower,
	MechPower,
	ElectricPower,
	SteamPower
};
/// <summary>Fuel type for combustion engines.</summary>
enum class TFuelType
{
	Undefined,
	Coal,
	Oil
};

class TDynamicObject;
struct neighbour_data
{
	TDynamicObject *vehicle{nullptr}; // detected obstacle
	int vehicle_end{-1}; // facing end of the obstacle
	float distance{10000.f}; // distance to the obstacle // NOTE: legacy value. TBD, TODO: use standard -1 instead?
};
