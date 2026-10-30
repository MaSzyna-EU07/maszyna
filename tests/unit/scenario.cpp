#include "stdafx.h"
#include "scenario.h"

#include "McZapkie/hamulce.h"

namespace test {

scenario::scenario(std::string Name, std::vector<channel> Channels, track Track) : m_name(std::move(Name)), m_channels(std::move(Channels)), m_track(Track) {
	reseed_random();
}

scenario &scenario::add(std::string const &Path, int Cab, int Coupling, double Velocity, double Gap) {
	m_train.add(definition::fiz(Path), Cab, Coupling, Velocity, Gap);
	return *this;
}

void scenario::record() {
	// on the first action, once the train is complete
	if (false == m_traces.empty()) {
		return;
	}
	for (std::size_t i = 0; i < m_train.size(); ++i) {
		m_traces.push_back(std::make_unique<trace>(m_name + "/" + std::to_string(i), m_channels, 100));
		m_train[i].record(*m_traces.back());
	}
}

void scenario::act(std::string const &Event) {
	record();
	for (auto &trace : m_traces) {
		trace->mark(Event);
	}
}

void scenario::wait(double Seconds) {
	record();
	drive(driver(), m_track, Seconds);
}

bool scenario::start_up() {
	record();
	return test::start_up(driver(), m_track, 300.0, m_traces.front().get());
}

void scenario::get_going() {
	act("get going");
	release_brakes(cab());
	for (int i = 0; i < 3 && cab().DirActive <= 0; ++i) {
		cab().DirectionForward();
	}
	wait(20.0);
}

void scenario::full_power(double Seconds) {
	auto &controlling{driver().controlling};
	controlling.IncMainCtrl(1);
	for (int i = 0; i < 90 && false == controlling.StLinFlag; ++i) {
		wait(1.0);
	}
	for (int i = 0; i < 60 && false == controlling.IsMainCtrlMaxPowerPos(); ++i) {
		controlling.IncMainCtrl(1);
		wait(1.0);
	}
	wait(Seconds);
}

void scenario::notch_back(int Position) {
	auto &controlling{driver().controlling};
	while (controlling.MainCtrlPos > Position && controlling.DecMainCtrl(1)) {
		wait(1.0);
	}
}

void scenario::check() {
	for (auto &trace : m_traces) {
		trace->check();
	}
}

} // namespace test
