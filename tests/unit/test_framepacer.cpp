#include "stdafx.h"
#include "doctest.h"
#include "utilities/framepacer.h"

// the main loop's frame limiter (fpslimit in eu07.ini, eu07/maszyna#416), driven with made-up clock readings
namespace
{
using namespace std::chrono_literals;
using clock_type = frame_pacer::clock;

clock_type::time_point const kStart{clock_type::time_point{} + 100s};
constexpr clock_type::duration kFrame{std::chrono::duration_cast<clock_type::duration>(std::chrono::duration<float>(1.f / 30.f))};

// runs frames that take Work each, with every wait overshooting by Oversleep; returns when the last of Frames ends
clock_type::time_point run_frames(frame_pacer &Pacer, int const Frames, clock_type::duration const Work, clock_type::duration const Oversleep)
{
	auto now{kStart};
	for (auto frame = 0; frame < Frames; ++frame)
	{
		now += Work;
		if (auto const next{Pacer.next_frame(now, kFrame)})
		{
			CHECK(*next > now);
			now = *next + Oversleep;
		}
	}
	return now;
}
} // namespace

TEST_CASE("frame pacer: without a limit nothing waits")
{
	frame_pacer pacer;
	CHECK_FALSE(pacer.next_frame(kStart, clock_type::duration::zero()).has_value());
	CHECK_FALSE(pacer.next_frame(kStart + 1ms, clock_type::duration::zero()).has_value());
}

TEST_CASE("frame pacer: a quick frame waits out the rest of the frame time")
{
	frame_pacer pacer;
	// the first frame only sets the cadence
	CHECK_FALSE(pacer.next_frame(kStart, kFrame).has_value());
	auto const next{pacer.next_frame(kStart + 5ms, kFrame)};
	REQUIRE(next.has_value());
	CHECK(*next == kStart + kFrame);
}

TEST_CASE("frame pacer: a slow frame doesn't wait")
{
	frame_pacer pacer;
	CHECK_FALSE(pacer.next_frame(kStart, kFrame).has_value());
	CHECK_FALSE(pacer.next_frame(kStart + kFrame + 5ms, kFrame).has_value());
}

TEST_CASE("frame pacer: the frame rate holds the limit with fast frames")
{
	frame_pacer pacer;
	auto const end{run_frames(pacer, 301, 2ms, clock_type::duration::zero())};
	// 300 paced frames after the first
	CHECK(end == kStart + 2ms + 300 * kFrame);
}

TEST_CASE("frame pacer: a wait that overshoots is made up in the next frame")
{
	// the windows timer wakes a sleeping thread up to ~15.6 ms late; sleeping off what is left of each frame
	// then drops 30 fps to about 22. on a fixed cadence the frame rate still averages to the limit
	frame_pacer pacer;
	auto const end{run_frames(pacer, 301, 2ms, 12ms)};
	auto const elapsed{std::chrono::duration<double>(end - kStart).count()};
	CHECK(300.0 / elapsed == doctest::Approx(30.0).epsilon(0.01));
}

TEST_CASE("frame pacer: after a long frame the cadence starts anew instead of catching up")
{
	frame_pacer pacer;
	CHECK_FALSE(pacer.next_frame(kStart, kFrame).has_value());
	// a hitch of half a second
	auto const late{kStart + 500ms};
	CHECK_FALSE(pacer.next_frame(late, kFrame).has_value());
	// the next quick frame waits a whole frame time from there rather than running at once
	auto const next{pacer.next_frame(late + 1ms, kFrame)};
	REQUIRE(next.has_value());
	CHECK(*next == late + kFrame);
}

TEST_CASE("frame pacer: turning the limit off and on again starts a new cadence")
{
	frame_pacer pacer;
	CHECK_FALSE(pacer.next_frame(kStart, kFrame).has_value());
	CHECK_FALSE(pacer.next_frame(kStart + 1ms, clock_type::duration::zero()).has_value());
	CHECK_FALSE(pacer.next_frame(kStart + 2s, kFrame).has_value());
	auto const next{pacer.next_frame(kStart + 2s + 1ms, kFrame)};
	REQUIRE(next.has_value());
	CHECK(*next == kStart + 2s + kFrame);
}

TEST_CASE("frame pacer: the wait stays exact after days of system uptime")
{
	// fpslimit 60 gives the frame time as float seconds; the old limiter handed the float rest of the frame
	// to sleep_for, which adds it to steady_clock::now() in float nanoseconds. after 95 hours of uptime that
	// rounds the wake-up time to steps of ~33 ms, and a 14 ms wait came out as no wait at all
	frame_pacer pacer;
	std::chrono::duration<float> const frametime{1.f / 60.f};
	auto const uptime{clock_type::time_point{} + 95h};
	CHECK_FALSE(pacer.next_frame(uptime, frametime).has_value());
	auto const next{pacer.next_frame(uptime + 2ms, frametime)};
	REQUIRE(next.has_value());
	auto const wait{std::chrono::duration<double, std::milli>(*next - (uptime + 2ms)).count()};
	CHECK(wait == doctest::Approx(1000.0 / 60.0 - 2.0).epsilon(0.001));
}
