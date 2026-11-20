/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/Driver.h"

#include "simulation/simulation.h"
#include "simulation/simulationsnapshotworld.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Event.h"

#include <array>
#include <format>
#include <limits>
#include <vector>

// the part of a simulation snapshot kept by the driver of a train (TController): the orders it has and which one it's
// on, where it is in its timetable, the consist it drives, and what it has made up its mind about: the speeds it
// holds to, why it stands, what it waits for, and its table of what's ahead on the track with the limits it latched

namespace {

namespace snapshot = simulation::snapshot;

// a variable of the driver kept under a key of the entry, and for a number the range a value of the file has to be in
template <typename Type_> struct member {
	std::string_view key;
	Type_ TController::*variable;
	Type_ min{lowest()};
	Type_ max{highest()};

	static constexpr Type_ lowest()
	{
		if constexpr (std::is_arithmetic_v<Type_>)
			return std::numeric_limits<Type_>::lowest();
		else
			return Type_{};
	}
	static constexpr Type_ highest()
	{
		if constexpr (std::is_arithmetic_v<Type_>)
			return std::numeric_limits<Type_>::max();
		else
			return Type_{};
	}
};

template <typename Table_> void save_members(snapshot::record &Entry, TController const &Driver, Table_ const &Table)
{
	for (auto const &item : Table)
		Entry.set(item.key, Driver.*(item.variable));
}

template <typename Table_> void load_members(TController &Driver, snapshot::record const &Entry, Table_ const &Table)
{
	for (auto const &item : Table)
	{
		// the whole numbers in their ranges; a number of another kind is a measure, which can be anything
		if constexpr (std::is_integral_v<decltype(item.min)> && false == std::is_same_v<decltype(item.min), bool>)
			Entry.get(item.key, Driver.*(item.variable), item.min, item.max);
		else
			Entry.get(item.key, Driver.*(item.variable));
	}
}

// the most events sharing a name a snapshot looks among
constexpr int most_siblings{100000};
// every order the driver knows, as bits (TOrders)
constexpr int every_order{(Jump_to_first_order << 1) - 1};

// the orders as their numbers joined with commas: "1,128,16,32"
std::string join(std::span<TOrders const> const Orders)
{
	std::string text;
	for (auto const order : Orders)
	{
		if (false == text.empty())
			text += ',';
		text += std::to_string(static_cast<int>(order));
	}
	return text;
}

// the whole numbers of a list between Min and Max. returns: empty if one of them isn't such a number
std::optional<std::vector<int>> split(std::string_view Text, int const Min, int const Max)
{
	std::vector<int> numbers;
	while (false == Text.empty())
	{
		auto const comma{Text.find(',')};
		auto const number{snapshot::text_number(Text.substr(0, comma))};
		auto const whole{number.has_value() ? snapshot::whole_number(*number, Min, Max) : std::nullopt};
		if (false == whole.has_value())
			return std::nullopt;
		numbers.emplace_back(*whole);
		Text.remove_prefix(comma == std::string_view::npos ? Text.size() : comma + 1);
	}
	return numbers;
}

// an event under the keys "<key>" (its name) and "<key>sibling" (its place among the events sharing the name)
void save_event(snapshot::record &Entry, std::string const &Key, basic_event const *Event)
{
	if (Event == nullptr)
		return;
	Entry.set(Key, Event->m_name);
	if (auto const sibling{snapshot::event_sibling(Event)}; sibling != 0)
		Entry.set(Key + "sibling", sibling);
}

basic_event *load_event(snapshot::record const &Entry, std::string const &Key)
{
	auto const name{Entry.text(Key)};
	auto sibling{0};
	if (name.empty() || (false == Entry.get(Key + "sibling", sibling, 0, most_siblings) && Entry.has(Key + "sibling")))
		return nullptr;
	return snapshot::find_event(name, sibling);
}

// a point of the table of what's ahead: its numbers under "<key>", its track and its event beside them
void save_point(snapshot::record &Entry, std::string const &Key, TSpeedPos const &Point)
{
	std::array const numbers{Point.fDist,  Point.fVelNext, Point.fSectionVelocityDist, Point.fPointDist, static_cast<double>(Point.iFlags),
	                         Point.bMoved ? 1.0 : 0.0, Point.fMoved,   Point.vPos.x,               Point.vPos.y,     Point.vPos.z};
	Entry.set(Key, snapshot::numbers_text(numbers));
	if (Point.trTrack != nullptr)
		Entry.set(Key + "track", snapshot::track_reference(Point.trTrack));
	save_event(Entry, Key + "event", Point.evEvent);
}

// returns: false if the entry has no such point, or it's of another make
bool load_point(snapshot::record const &Entry, std::string const &Key, TSpeedPos &Point)
{
	auto const numbers{snapshot::text_numbers(Entry.text(Key))};
	if (numbers.size() != 10)
		return false;
	// the flags of a point are bits of a whole number
	auto const flags{snapshot::whole_number(numbers[4], std::numeric_limits<int>::min(), std::numeric_limits<int>::max())};
	if (false == flags.has_value())
	{
		Entry.refuse(Key, "its flags aren't a whole number");
		return false;
	}
	Point.fDist = numbers[0];
	Point.fVelNext = numbers[1];
	Point.fSectionVelocityDist = numbers[2];
	Point.fPointDist = numbers[3];
	Point.iFlags = *flags;
	Point.bMoved = numbers[5] != 0.0;
	Point.fMoved = numbers[6];
	Point.vPos = {numbers[7], numbers[8], numbers[9]};
	Point.trTrack = snapshot::find_track(Entry.text(Key + "track"));
	Point.evEvent = load_event(Entry, Key + "event");
	// a point whose track or event the scenery no longer has can't be told from one which had none
	return (Point.trTrack != nullptr) == Entry.has(Key + "track") && (Point.evEvent != nullptr) == Entry.has(Key + "event");
}

} // namespace

// the variables kept one for one. the tables are members so that they can name the private ones
struct TController::snapshot_members {
	static constexpr std::array speeds{
	    member<double>{"accpreferred", &TController::AccPreferred},
	    member<double>{"accdesired", &TController::AccDesired},
	    member<double>{"veldesired", &TController::VelDesired},
	    member<double>{"accdesiredav", &TController::fAccDesiredAv},
	    member<double>{"accdesiredgravity", &TController::fAccDesiredGravity},
	    member<double>{"velfordriver", &TController::VelforDriver},
	    member<double>{"velsignal", &TController::VelSignal},
	    member<double>{"vellimit", &TController::VelLimit},
	    member<double>{"velsignallast", &TController::VelSignalLast},
	    member<double>{"velsignalnext", &TController::VelSignalNext},
	    member<double>{"vellimitlast", &TController::VelLimitLast},
	    member<double>{"velroad", &TController::VelRoad},
	    member<double>{"velnext", &TController::VelNext},
	    member<double>{"velrestricted", &TController::VelRestricted},
	    member<double>{"shuntvelocity", &TController::fShuntVelocity},
	    member<double>{"velmax", &TController::fVelMax},
	    member<double>{"velplus", &TController::fVelPlus},
	    member<double>{"velminus", &TController::fVelMinus},
	};
	static constexpr std::array distances{
	    member<double>{"firstsemaphordist", &TController::FirstSemaphorDist},
	    member<double>{"actualproximitydist", &TController::ActualProximityDist},
	    member<double>{"switchcleardist", &TController::SwitchClearDist},
	    member<double>{"signalareaenddist", &TController::SignalAreaEndDist},
	    member<double>{"minproximitydist", &TController::fMinProximityDist},
	    member<double>{"maxproximitydist", &TController::fMaxProximityDist},
	    member<double>{"laststopexpdist", &TController::fLastStopExpDist},
	    member<double>{"driverdist", &TController::fDriverDist},
	    member<double>{"brakedist", &TController::fBrakeDist},
	    member<double>{"movelen", &TController::dMoveLen},
	    member<double>{"length", &TController::fLength},
	    member<double>{"mass", &TController::fMass},
	};
	static constexpr std::array timers{
	    member<double>{"brakeheldtime", &TController::fBrakeHeldTime},
	    member<double>{"shortfall", &TController::fShortfall},
	    member<double>{"shortfalltime", &TController::fShortfallTime},
	    member<double>{"uphillstarttime", &TController::fUphillStartTime},
	    member<double>{"warningduration", &TController::fWarningDuration},
	    member<double>{"waitingtime", &TController::WaitingTime},
	    member<double>{"waitingexpiretime", &TController::WaitingExpireTime},
	    member<double>{"idletime", &TController::IdleTime},
	    member<double>{"stoptime", &TController::fStopTime},
	    member<double>{"reactiontime", &TController::ReactionTime},
	    member<double>{"lastreactiontime", &TController::LastReactionTime},
	    member<double>{"braketime", &TController::fBrakeTime},
	    member<double>{"actiontime", &TController::fActionTime},
	    member<double>{"radiocontroltime", &TController::m_radiocontroltime},
	    member<double>{"brakechargingcooldown", &TController::BrakeChargingCooldown},
	    member<double>{"brakechargingstart", &TController::BrakeChargingStart},
	    member<double>{"departblockedstart", &TController::DepartBlockedStart},
	    member<double>{"dbtvelocitybrake", &TController::DBT_VelocityBrake},
	    member<double>{"dbtvelocityrelease", &TController::DBT_VelocityRelease},
	    member<double>{"dbtvelocityfinish", &TController::DBT_VelocityFinish},
	    member<double>{"dbtbrakingtime", &TController::DBT_BrakingTime},
	    member<double>{"dbtreleasingtime", &TController::DBT_ReleasingTime},
	    member<double>{"dbtmidpointacc", &TController::DBT_MidPointAcc},
	    member<double>{"lastupdatedtime", &TController::LastUpdatedTime},
	    member<double>{"elapsedtime", &TController::ElapsedTime},
	};
	static constexpr std::array levels{
	    member<double>{"brakectrlposition", &TController::BrakeCtrlPosition},
	    member<double>{"overhead1", &TController::fOverhead1},
	    member<double>{"overhead2", &TController::fOverhead2},
	    member<double>{"voltage", &TController::fVoltage},
	    member<double>{"driverbraking", &TController::fDriverBraking},
	    member<double>{"brakereaction", &TController::fBrakeReaction},
	    member<double>{"nominalaccthreshold", &TController::fNominalAccThreshold},
	    member<double>{"accthreshold", &TController::fAccThreshold},
	    member<double>{"absaccs", &TController::AbsAccS},
	    member<double>{"brakinginitiallevel", &TController::BrakingInitialLevel},
	    member<double>{"brakinglevelincrease", &TController::BrakingLevelIncrease},
	    member<double>{"accgravity", &TController::fAccGravity},
	    member<double>{"ready", &TController::fReady},
	    member<double>{"consistshade", &TController::ConsistShade},
	    member<double>{"medamax", &TController::fMedAmax},
	};
	static constexpr std::array numbers{
	    member<int>{"flags", &TController::iDrivigFlags},
	    member<int>{"direction", &TController::iDirection, -1, 1},
	    member<int>{"directionorder", &TController::iDirectionOrder, -1, 1},
	    member<int>{"vehiclecount", &TController::iVehicleCount},
	    member<int>{"coupler", &TController::iCoupler},
	    member<int>{"failcount", &TController::iDriverFailCount},
	    member<int>{"overheadzero", &TController::iOverheadZero},
	    member<int>{"overheaddown", &TController::iOverheadDown},
	    member<int>{"universalbrakebuttons", &TController::UniversalBrakeButtons},
	    member<int>{"dizelpercentage", &TController::DizelPercentage},
	    member<int>{"dizelpercentagespeed", &TController::DizelPercentage_Speed},
	    member<int>{"helperstate", &TController::HelperState},
	    member<int>{"shortfallgrowth", &TController::iShortfallGrowth},
	    member<int>{"dynamicbraketest", &TController::DynamicBrakeTest},
	    member<int>{"staticbraketest", &TController::StaticBrakeTest},
	    member<int>{"radiochannel", &TController::iRadioChannel},
	    member<int>{"guardradio", &TController::iGuardRadio},
	    member<int>{"vehicles", &TController::iVehicles},
	    member<int>{"engines", &TController::ControlledEnginesCount},
	    member<int>{"lastexchangeplatforms", &TController::m_lastexchangeplatforms},
	    member<int>{"lastexchangedirection", &TController::m_lastexchangedirection},
	};
	static constexpr std::array flags{
	    member<bool>{"stoptargetispassengerstop", &TController::bStopTargetIsPassengerStop},
	    member<bool>{"uphillstart", &TController::bUphillStart},
	    member<bool>{"needtryagain", &TController::Need_TryAgain},
	    member<bool>{"heatingtemperatureok", &TController::IsHeatingTemperatureOK},
	    member<bool>{"heatingtemperaturetoolow", &TController::IsHeatingTemperatureTooLow},
	    member<bool>{"atpassengerstop", &TController::IsAtPassengerStop},
	    member<bool>{"scheduledstopvisible", &TController::IsScheduledPassengerStopVisible},
	    member<bool>{"forcepnbrake", &TController::ForcePNBrake},
	    member<bool>{"brakereinspection", &TController::BrakeReinspectionPending},
	    member<bool>{"engineactive", &TController::iEngineActive},
	    member<bool>{"passengertrain", &TController::IsPassengerTrain},
	    member<bool>{"cargotrain", &TController::IsCargoTrain},
	    member<bool>{"heavycargotrain", &TController::IsHeavyCargoTrain},
	    member<bool>{"isready", &TController::Ready},
	    member<bool>{"consistbraked", &TController::IsConsistBraked},
	    member<bool>{"nextstopannouncement", &TController::m_makenextstopannouncement},
	    member<bool>{"doorneedsopening", &TController::DoesAnyDoorNeedOpening},
	};
	static constexpr std::array texts{
	    member<std::string>{"nextstop", &TController::asNextStop},
	    member<std::string>{"lastexchangestop", &TController::m_lastexchangestop},
	    member<std::string>{"assignment", &TController::m_assignment},
	};
};

void TController::snapshot_save(simulation::snapshot::record &Entry) const
{
	// who drives, and what
	Entry.set("ai", AIControllFlag);
	Entry.set("front", snapshot::vehicle_reference(pVehicles[end::front]));
	Entry.set("rear", snapshot::vehicle_reference(pVehicles[end::rear]));
	// the orders: the one it's on, in words for the reader of the file, then all of them
	Entry.set("order", Order2Str(OrderCurrentGet()));
	Entry.set("orderpos", OrderPos);
	Entry.set("orders", join(std::span{OrderList}.first(static_cast<std::size_t>(std::clamp(OrderTop, 0, maxorders)))));
	// the timetable
	Entry.set("train", TrainParams.TrainName);
	Entry.set("station", TrainParams.StationIndex);
	Entry.set("stationstart", TrainParams.StationStart);
	Entry.set("nextstation", TrainParams.NextStationName);
	Entry.set("latency", TrainParams.LastStationLatency);
	Entry.set("timetabledirection", TrainParams.Direction);
	Entry.set("ttvmax", TrainParams.TTVmax);
	// the service flags of the stop the train is at or left last: central doors, doorstep, conductor, brake test
	Entry.set("stopflags", std::format("{},{},{},{}", static_cast<int>(m_stopflags.central_doors), static_cast<int>(m_stopflags.doorstep),
	                                   static_cast<int>(m_stopflags.conductor), static_cast<int>(m_stopflags.brake_test)));
	save_members(Entry, *this, snapshot_members::texts);
	// what it does and why
	Entry.set("action", eAction);
	Entry.set("stopreason", eStopReason);
	save_members(Entry, *this, snapshot_members::numbers);
	save_members(Entry, *this, snapshot_members::flags);
	save_members(Entry, *this, snapshot_members::speeds);
	save_members(Entry, *this, snapshot_members::distances);
	save_members(Entry, *this, snapshot_members::timers);
	save_members(Entry, *this, snapshot_members::levels);
	Entry.set("exchangetime", ExchangeTime);
	// what the brake of the consist gives at each speed, as worked out when the consist was last looked over
	Entry.set("brakea0", snapshot::numbers_text(fBrake_a0));
	Entry.set("brakea1", snapshot::numbers_text(fBrake_a1));
	Entry.set("brakesystem", BrakeSystem);
	Entry.set("doorclosure", DoorClosure);
	Entry.set("lastannouncement", m_lastannouncement);
	Entry.set("vellimitlastdist0", VelLimitLastDist.first).set("vellimitlastdist1", VelLimitLastDist.second);
	Entry.set("lighthint0", m_lighthints[end::front]).set("lighthint1", m_lighthints[end::rear]);
	if (iDirectionBackup.has_value())
		Entry.set("directionbackup", *iDirectionBackup);
	if (iCouplingVehicle.has_value() && iCouplingVehicle->first != nullptr)
		Entry.set("couplingvehicle", snapshot::vehicle_reference(iCouplingVehicle->first)).set("couplingend", iCouplingVehicle->second);
	save_event(Entry, "signskip", eSignSkip);
	save_event(Entry, "signnext", eSignNext);
	snapshot_save_table(Entry);
}

// what the driver sees ahead: the table of the tracks and signs on its way, and the speed limits of points it has
// passed the signs of
void TController::snapshot_save_table(simulation::snapshot::record &Entry) const
{
	Entry.set("table", sSpeedTable.size());
	Entry.set("tablelast", iLast).set("tabledirection", iTableDirection).set("tablelastvel", fLastVel);
	if (tLast != nullptr)
		Entry.set("tablelasttrack", snapshot::track_reference(tLast));
	if (SemNextIndex != npos)
		Entry.set("semnext", SemNextIndex);
	if (SemNextStopIndex != npos)
		Entry.set("semnextstop", SemNextStopIndex);
	for (std::size_t index{0}; index < sSpeedTable.size(); ++index)
		save_point(Entry, std::format("p{}", index), sSpeedTable[index]);

	Entry.set("limits", m_pointlimits.size());
	for (std::size_t index{0}; index < m_pointlimits.size(); ++index)
	{
		auto const &limit{m_pointlimits[index]};
		auto const key{std::format("l{}", index)};
		std::array const numbers{limit.velocity, limit.distance, limit.celldistance, limit.heading.x, limit.heading.y, limit.heading.z, limit.end, limit.endknown ? 1.0 : 0.0};
		Entry.set(key, snapshot::numbers_text(numbers));
		save_event(Entry, key + "event", limit.source);
	}
	if (m_pointlimitsvehicle != nullptr)
	{
		std::array const position{m_pointlimitsposition.x, m_pointlimitsposition.y, m_pointlimitsposition.z, m_pointlimitslength};
		Entry.set("limitsvehicle", snapshot::vehicle_reference(m_pointlimitsvehicle)).set("limitsposition", snapshot::numbers_text(position));
		Entry.set("limitsend0", snapshot::vehicle_reference(m_pointlimitsends[0])).set("limitsend1", snapshot::vehicle_reference(m_pointlimitsends[1]));
	}
}

// returns: false if the table of the entry can't be put back whole; the driver then looks at the track again
bool TController::snapshot_load_table(simulation::snapshot::record const &Entry)
{
	TableClear();
	m_pointlimits.clear();
	m_pointlimitsvehicle = nullptr;
	// each point of the table has a key of its own: a count past the keys of the entry can't be the count of its points
	auto const points{snapshot::read_count(Entry, "table")};
	if (false == points.has_value())
		return false;
	auto const count{*points};
	std::vector<TSpeedPos> table(count);
	for (std::size_t index{0}; index < count; ++index)
	{
		if (false == load_point(Entry, std::format("p{}", index), table[index]))
			return false;
	}
	// the indices into the table point at one of its points, or at none (-1 and npos, which the entry doesn't write)
	auto last{-1};
	auto const next{snapshot::read_index(Entry, "semnext", count)};
	auto const nextstop{snapshot::read_index(Entry, "semnextstop", count)};
	if ((false == Entry.get("tablelast", last, -1, static_cast<int>(count) - 1) && Entry.has("tablelast")) || (false == next.has_value() && Entry.has("semnext")) ||
	    (false == nextstop.has_value() && Entry.has("semnextstop")))
		return false;
	sSpeedTable = std::move(table);
	iLast = last;
	Entry.get("tabledirection", iTableDirection, -1, 1);
	Entry.get("tablelastvel", fLastVel);
	tLast = snapshot::find_track(Entry.text("tablelasttrack"));
	SemNextIndex = next.value_or(npos);
	SemNextStopIndex = nextstop.value_or(npos);
	eSignNext = load_event(Entry, "signnext");

	auto const limits{snapshot::read_count(Entry, "limits").value_or(0)};
	for (std::size_t index{0}; index < limits; ++index)
	{
		auto const key{std::format("l{}", index)};
		auto const numbers{snapshot::text_numbers(Entry.text(key))};
		auto const *source{load_event(Entry, key + "event")};
		if (numbers.size() != 8 || source == nullptr)
			continue;
		auto &limit{m_pointlimits.emplace_back()};
		limit.source = source;
		limit.velocity = numbers[0];
		limit.distance = numbers[1];
		limit.celldistance = numbers[2];
		limit.heading = {numbers[3], numbers[4], numbers[5]};
		limit.end = numbers[6];
		limit.endknown = numbers[7] != 0.0;
	}
	if (auto const position{snapshot::text_numbers(Entry.text("limitsposition"))}; position.size() == 4)
	{
		m_pointlimitsvehicle = snapshot::find_vehicle(Entry.text("limitsvehicle"));
		m_pointlimitsposition = {position[0], position[1], position[2]};
		m_pointlimitslength = position[3];
		m_pointlimitsends[0] = snapshot::find_vehicle(Entry.text("limitsend0"));
		m_pointlimitsends[1] = snapshot::find_vehicle(Entry.text("limitsend1"));
	}
	return true;
}

void TController::snapshot_load(simulation::snapshot::record const &Entry)
{
	// the timetable: the scenery gave the train the one it started with, which another may have replaced since
	if (auto const train{Entry.text("train", TrainParams.TrainName)}; train != TrainParams.TrainName)
	{
		TrainParams = TTrainParameters(train);
		if (train != "none" && false == TrainParams.LoadTTfile(Global.asCurrentSceneryPath, 0, Entry.value("ttvmax", 0.0)))
			ErrorLog(std::format("snapshot: timetable \"{}\" of {} can't be loaded", train, OwnerName()));
	}
	// the stations are entries of the table of the timetable, 1..StationCount, and the one after the last
	auto const laststation{std::clamp(TrainParams.StationCount + 1, 0, MaxTTableSize)};
	Entry.get("station", TrainParams.StationIndex, 0, laststation);
	Entry.get("stationstart", TrainParams.StationStart, 0, laststation);
	Entry.get("nextstation", TrainParams.NextStationName);
	Entry.get("latency", TrainParams.LastStationLatency);
	Entry.get("timetabledirection", TrainParams.Direction, -1, 1);
	Entry.get("ttvmax", TrainParams.TTVmax);
	m_stopflags = TrainParams.stop_flags();
	if (auto const stopflags{split(Entry.text("stopflags"), 0, static_cast<int>(timetable::tristate::no)).value_or(std::vector<int>{})}; stopflags.size() == 4)
	{
		using timetable::tristate;
		m_stopflags.central_doors = static_cast<tristate>(stopflags[0]);
		m_stopflags.doorstep = static_cast<tristate>(stopflags[1]);
		m_stopflags.conductor = static_cast<tristate>(stopflags[2]);
		m_stopflags.brake_test = static_cast<tristate>(stopflags[3]);
	}
	load_members(*this, Entry, snapshot_members::texts);

	// the orders
	if (Entry.has("orders"))
	{
		// the orders are bits, and the stack holds so many of them
		if (auto const orders{split(Entry.text("orders"), 0, every_order)}; orders.has_value() && orders->size() <= static_cast<std::size_t>(maxorders))
		{
			OrdersClear();
			OrderTop = static_cast<int>(orders->size());
			for (auto index{0}; index < OrderTop; ++index)
				OrderList[index] = static_cast<TOrders>((*orders)[index]);
		}
		else
			Entry.refuse("orders", std::format("they aren't up to {} numbers of orders", maxorders));
	}
	// the order the driver is on is an index into the stack
	Entry.get("orderpos", OrderPos, 0, maxorders - 1);

	Entry.get("action", eAction, TAction::actUnknown, TAction::actTrial);
	Entry.get("stopreason", eStopReason, stopNone, stopError);
	load_members(*this, Entry, snapshot_members::numbers);
	load_members(*this, Entry, snapshot_members::flags);
	load_members(*this, Entry, snapshot_members::speeds);
	load_members(*this, Entry, snapshot_members::distances);
	load_members(*this, Entry, snapshot_members::timers);
	load_members(*this, Entry, snapshot_members::levels);
	Entry.get("exchangetime", ExchangeTime);
	if (auto const numbers{snapshot::text_numbers(Entry.text("brakea0"))}; numbers.size() == std::size(fBrake_a0))
		std::ranges::copy(numbers, fBrake_a0);
	if (auto const numbers{snapshot::text_numbers(Entry.text("brakea1"))}; numbers.size() == std::size(fBrake_a1))
		std::ranges::copy(numbers, fBrake_a1);
	Entry.get("brakesystem", BrakeSystem, TBrakeSystem::Individual, TBrakeSystem::ElectroPneumatic);
	Entry.get("doorclosure", DoorClosure, door_closure_t::none, door_closure_t::open);
	Entry.get("lastannouncement", m_lastannouncement, announcement_t::idle, announcement_t::end);
	Entry.get("vellimitlastdist0", VelLimitLastDist.first);
	Entry.get("vellimitlastdist1", VelLimitLastDist.second);
	Entry.get("lighthint0", m_lighthints[end::front]);
	Entry.get("lighthint1", m_lighthints[end::rear]);
	iDirectionBackup.reset();
	if (auto direction{0}; Entry.get("directionbackup", direction, -1, 1))
		iDirectionBackup = direction;
	iCouplingVehicle.reset();
	// the end of the vehicle to couple to is one of its two couplers
	if (auto *vehicle{snapshot::find_vehicle(Entry.text("couplingvehicle"))}; vehicle != nullptr)
	{
		if (auto end{0}; Entry.get("couplingend", end, 0, 1) || false == Entry.has("couplingend"))
			iCouplingVehicle = std::make_pair(vehicle, end);
	}

	// the consist: its ends as they were, the vehicles driven and braked found anew among what's coupled now
	if (auto *front{snapshot::find_vehicle(Entry.text("front"))}; front != nullptr)
		pVehicles[end::front] = front;
	if (auto *rear{snapshot::find_vehicle(Entry.text("rear"))}; rear != nullptr)
		pVehicles[end::rear] = rear;
	ControllingSet();
	if (auto ai{AIControllFlag}; Entry.get("ai", ai))
	{
		AIControllFlag = ai;
		pVehicle->Controller = ai;
	}
	// what's ahead on the track as the driver had it; without it the table is filled anew from where the train is now
	if (false == snapshot_load_table(Entry))
		TableClear();
	eSignSkip = load_event(Entry, "signskip");
	// the hints of what the driver is about to do are for the player, and made again as the driver goes on
	m_hints.clear();
	BrakeChargingPipeOverchargeStart.clear();
}
