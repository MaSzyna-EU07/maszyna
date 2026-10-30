#include "stdafx.h"
#include "drive.h"
#include "consist.h"
#include "golden.h"
#include "doctest.h"

#include "McZapkie/hamulce.h"

namespace test {

void mark(trace *Trace, std::string const &Event) {
	if (Trace != nullptr) {
		Trace->mark(Event);
	}
}

bool is_powered(TMoverParameters &Vehicle) {
	auto const type{Vehicle.EngineType};
	return Vehicle.Power > 1.0 &&
	       (type == TEngineType::ElectricSeriesMotor || type == TEngineType::ElectricInductionMotor || type == TEngineType::DieselEngine ||
	        type == TEngineType::DieselElectric);
}

driven driving(vehicle &Vehicle) {
	return {Vehicle.mover(), Vehicle.mover(), Vehicle.mover(), [&Vehicle](double Seconds, track const &Track) { Vehicle.run(Seconds, Track); }};
}

driven driving(consist &Consist) {
	// the AI's mvControlling and mvPantographUnit: the first vehicle with an engine, and with pantographs
	auto *controlling{&Consist[0].mover()};
	auto *pantographs{&Consist[0].mover()};
	for (auto i = Consist.size(); i-- > 0;) {
		auto &mover{Consist[i].mover()};
		if (is_powered(mover)) {
			controlling = &mover;
		}
		if (mover.EnginePowerSource.SourceType == TPowerSource::CurrentCollector && mover.EnginePowerSource.CollectorParameters.CollectorsNo > 0) {
			pantographs = &mover;
		}
	}
	return {Consist[0].mover(), *controlling, *pantographs, [&Consist](double Seconds, track const &Track) { Consist.run(Seconds, Track); }};
}

namespace {

void attend(TMoverParameters &Mover) {
	if (Mover.SecuritySystem.is_cabsignal_blinking() && Mover.SecuritySystem.has_separate_acknowledge()) {
		Mover.SecuritySystem.cabsignal_reset();
	}
	if (Mover.SecuritySystem.is_blinking()) {
		Mover.SecuritySystemReset();
	}
}

} // namespace

void drive(driven Driven, track const &Track, double Seconds) {
	auto const seconds{std::lround(Seconds)};
	for (long second = 0; second < seconds; ++second) {
		attend(Driven.cab);
		Driven.run(1.0, Track);
	}
}

void release_brakes(TMoverParameters &Cab) {
	Cab.BrakeLevelSet(Cab.Handle->GetPos(bh_RP));
	Cab.DecLocalBrakeLevel(LocalBrakePosNo);
	Cab.DecManualBrakeLevel(ManualBrakePosNo);
	Cab.SpringBrakeActivate(false);
	// the hand and spring brakes of the rest of the train, as TController::AutoRewident() releases them
	std::vector<TMoverParameters *> train{&Cab};
	for (std::size_t i = 0; i < train.size(); ++i) {
		for (auto const &coupler : train[i]->Couplers) {
			if (coupler.Connected != nullptr && coupler.CouplingFlag != coupling::faux && std::ranges::find(train, coupler.Connected) == train.end()) {
				coupler.Connected->DecManualBrakeLevel(ManualBrakePosNo);
				coupler.Connected->SpringBrake.Activate = false;
				train.push_back(coupler.Connected);
			}
		}
	}
}

bool is_ready(driven const &Driven) {
	auto const &mover{Driven.controlling};
	auto const air{mover.VeselVolume <= 0.0 || mover.ScndPipePress >= 4.5};
	// the converter feeds the 110 V circuits (the electrodynamic brake's control among them); the AI switches
	// it on once the line breaker is closed, so a train already charged with air isn't ready before it runs
	auto const converter{mover.ConverterStart != start_t::manual || mover.ConverterFlag};
	return mover.Mains && air && converter;
}

bool start_up(driven Driven, track const &Track, double Seconds, trace *Trace) {
	auto &cab{Driven.cab};
	auto &controlling{Driven.controlling};
	auto &pantographs{Driven.pantographs};
	mark(Trace, "start up");
	auto const diesel{controlling.EngineType == TEngineType::DieselEngine || controlling.EngineType == TEngineType::DieselElectric};
	auto const collector{pantographs.EnginePowerSource.SourceType == TPowerSource::CurrentCollector &&
	                     pantographs.EnginePowerSource.CollectorParameters.CollectorsNo > 0};
	// the converter and the compressor get switched on once the line breaker is closed, wherever in the train
	// they are; a train already charged with air isn't ready before that
	auto auxiliaries{false};
	// one round of the driver's actions a second, as the AI reacts
	for (auto second = 0.0; second < Seconds; second += 1.0) {
		cab.BatterySwitch(true);
		cab.CabActivisation();
		if (diesel) {
			cab.OilPumpSwitch(true);
			controlling.WaterPumpBreakerSwitch(true);
			controlling.WaterPumpSwitch(true);
			controlling.WaterHeaterBreakerSwitch(true);
			controlling.WaterHeaterSwitch(true);
			cab.FuelPumpSwitch(true);
		}
		if (collector) {
			auto const enough{(pantographs.TrainType == dt_EZT ? 2.5 : 3.5) + 0.1};
			if (pantographs.PantPress < enough) {
				if (false == pantographs.PantAutoValve) {
					pantographs.bPantKurek3 = false; // the pantographs on their own air
				}
				pantographs.PantCompFlag = true;
			}
			else if (false == pantographs.bPantKurek3 || pantographs.PantPress <= pantographs.ScndPipePress) {
				pantographs.PantCompFlag = false;
			}
			cab.OperatePantographsValve(operation_t::enable);
			cab.OperatePantographValve(end::rear, operation_t::enable);
		}
		auto const power{false == collector ||
		                 std::max(controlling.GetTrainsetHighVoltage(), controlling.PantographVoltage) > controlling.EnginePowerSource.CollectorParameters.MinV};
		if (power) {
			// a direction first, as in TController::PrepareDirection(): the relays reset only with one set
			for (int i = 0; i < 3 && cab.DirActive <= 0; ++i) {
				cab.DirectionForward();
			}
			if (false == controlling.Mains) {
				// the line breaker closes only with the controller at zero, and after the relays are reset
				while (controlling.DecMainCtrl(2)) {
				}
				controlling.DecScndCtrl(2);
				cab.RelayReset(relay_t::maincircuitground | relay_t::tractionnmotoroverload | relay_t::primaryconverteroverload);
				if (controlling.EngineType == TEngineType::DieselEngine) {
					// a mechanical diesel starts at idle, the first position giving the engine fuel
					while (controlling.RList[controlling.MainCtrlPos].Mn == 0 && controlling.IncMainCtrl(1)) {
					}
				}
				cab.MainSwitch(true);
			}
			else {
				cab.ConverterSwitch(true);
				cab.CompressorSwitch(true);
				auxiliaries = true;
				if (cab.MotorBlowers[end::front].speed != 0) {
					cab.MotorBlowersSwitchOff(false, end::front);
					cab.MotorBlowersSwitch(true, end::front);
				}
				if (cab.MotorBlowers[end::rear].speed != 0) {
					cab.MotorBlowersSwitchOff(false, end::rear);
					cab.MotorBlowersSwitch(true, end::rear);
				}
			}
		}
		drive(Driven, Track, 1.0);
		if (auxiliaries && is_ready(Driven)) {
			mark(Trace, "ready");
			return true;
		}
	}
	return false;
}

void drive_cycle(driven Driven, track const &Track, trace &Trace) {
	auto &cab{Driven.cab};
	auto &controlling{Driven.controlling};
	if (false == start_up(Driven, Track, 300.0, &Trace)) {
		MESSAGE("didn't get ready: Mains " << controlling.Mains << ", main reservoir " << controlling.ScndPipePress);
	}
	Trace.mark("release brakes");
	release_brakes(cab);
	drive(Driven, Track, 30.0);

	Trace.mark("forward");
	for (int i = 0; i < 3 && cab.DirActive <= 0; ++i) {
		cab.DirectionForward();
	}
	drive(Driven, Track, 2.0);

	Trace.mark("notch up");
	for (int i = 0; i < 60 && false == controlling.IsMainCtrlMaxPowerPos(); ++i) {
		controlling.IncMainCtrl(1);
		drive(Driven, Track, 2.0);
	}
	Trace.mark("full power");
	drive(Driven, Track, 30.0);

	Trace.mark("second controller up");
	for (int i = 0; i < 20 && controlling.IncScndCtrl(1); ++i) {
		drive(Driven, Track, 2.0);
	}
	drive(Driven, Track, 20.0);
	Trace.mark("second controller down");
	for (int i = 0; i < 20 && controlling.DecScndCtrl(1); ++i) {
		drive(Driven, Track, 1.0);
	}

	Trace.mark("notch down");
	for (int i = 0; i < 60 && controlling.DecMainCtrl(1); ++i) {
		drive(Driven, Track, 1.0);
	}
	Trace.mark("coast");
	drive(Driven, Track, 10.0);

	Trace.mark("brake");
	cab.BrakeLevelSet(cab.Handle->GetPos(bh_FB));
	drive(Driven, Track, 60.0);
	Trace.mark("release");
	cab.BrakeLevelSet(cab.Handle->GetPos(bh_RP));
	drive(Driven, Track, 20.0);
}

} // namespace test
