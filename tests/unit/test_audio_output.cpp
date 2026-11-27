#include "stdafx.h"

#include "doctest.h"

#include "audio/outputfollow.h"

// the following of the audio output: when the renderer is told to open its playback device again

namespace
{

using audio::output_follower;
using action = audio::output_follower::action;
using std::chrono::milliseconds;

// a follower with a device that answers the checks of its connection as told, and counts them
struct output_rig
{
	output_follower follower;
	output_follower::clock::time_point now{};
	bool connected{true};
	int checks{0};

	explicit output_rig(bool const Pinned = false, bool const Fallback = false)
	{
		follower.start(Pinned, Fallback, now);
	}
	action step(milliseconds const Elapsed, unsigned const Events = 0u)
	{
		now += Elapsed;
		return follower.update(Events, now, [this]() {
			++checks;
			return connected;
		});
	}
};

} // namespace

TEST_CASE("audio output: a working device is left alone and checked about once a second")
{
	output_rig rig;
	for (int frame = 0; frame < 300; ++frame) // 3 s at 100 fps
		CHECK(rig.step(milliseconds{10}) == action::none);
	CHECK(rig.checks == 3);
}

TEST_CASE("audio output: a lost connection is found without any event from the system")
{
	// the stream invalidated, the device not removed and the default output unchanged: nothing is reported
	output_rig rig;
	rig.connected = false;
	CHECK(rig.step(milliseconds{500}) == action::none); // not yet checked
	CHECK(rig.step(milliseconds{500}) == action::reopen);
	CHECK(rig.checks == 1);
}

TEST_CASE("audio output: a removed device has the connection checked at once")
{
	output_rig rig;
	CHECK(rig.step(milliseconds{10}, output_follower::device_removed) == action::none); // another device was removed
	CHECK(rig.checks == 1);
	rig.connected = false;
	CHECK(rig.step(milliseconds{10}, output_follower::device_removed) == action::reopen);
}

TEST_CASE("audio output: a change of the default output is followed, the arrival of some device is not")
{
	output_rig rig;
	CHECK(rig.step(milliseconds{10}, output_follower::device_added) == action::none);
	CHECK(rig.step(milliseconds{10}, output_follower::default_changed) == action::reopen);
	CHECK(rig.checks == 0);
}

TEST_CASE("audio output: opening that failed is tried again, with growing delays and at once on any event")
{
	output_rig rig;
	rig.connected = false;
	CHECK(rig.step(output_follower::check_interval) == action::reopen);
	CHECK(rig.follower.failed(rig.now)); // the first failure in a row is the one to log
	CHECK(rig.follower.lost());

	// the following attempts come after 0.5, 1, 2, 4 and then every 4 s
	for (auto const delay : {500, 1000, 2000, 4000, 4000})
	{
		CHECK(rig.step(milliseconds{delay - 1}) == action::none);
		CHECK(rig.step(milliseconds{1}) == action::reopen);
		CHECK_FALSE(rig.follower.failed(rig.now));
	}
	// whatever the system reports may be the output coming back: the next attempt at once, but not sooner than
	// event_retry_min after the last one, as a device coming reports a burst of events
	CHECK(rig.step(milliseconds{1}, output_follower::device_added) == action::none);
	CHECK(rig.step(output_follower::event_retry_min - milliseconds{2}) == action::none);
	CHECK(rig.step(milliseconds{1}) == action::reopen); // the event brought the attempt forward and wasn't forgotten
	CHECK_FALSE(rig.follower.failed(rig.now));
	CHECK(rig.step(output_follower::event_retry_min, output_follower::default_changed) == action::reopen);

	// a lost device isn't asked about its connection, it is opened again
	CHECK(rig.checks == 1);

	rig.connected = true;
	rig.follower.succeeded(false, rig.now);
	CHECK_FALSE(rig.follower.lost());
	CHECK(rig.step(milliseconds{10}) == action::none);
	// and the next loss starts from the short delay, with its own line in the log
	rig.connected = false;
	CHECK(rig.step(output_follower::check_interval) == action::reopen);
	CHECK(rig.follower.failed(rig.now));
	CHECK(rig.step(output_follower::retry_min) == action::reopen);
}

TEST_CASE("audio output: a device set in eu07.ini doesn't follow the default output while it works")
{
	output_rig rig{true};
	CHECK_FALSE(rig.follower.follows_default());
	CHECK(rig.step(milliseconds{10}, output_follower::default_changed) == action::none);
	CHECK(rig.step(milliseconds{10}, output_follower::device_added) == action::none);
	// but it is opened again when it loses the connection
	rig.connected = false;
	CHECK(rig.step(output_follower::check_interval) == action::reopen);
}

TEST_CASE("audio output: the default output standing in for the device set in eu07.ini")
{
	output_rig rig{true};
	rig.connected = false;
	CHECK(rig.step(output_follower::check_interval) == action::reopen);
	// the device set is gone, the default one was opened instead
	rig.connected = true;
	rig.follower.succeeded(true, rig.now);
	CHECK(rig.follower.fallback());
	CHECK(rig.follower.follows_default());

	// the default output is followed for as long as it stands in
	CHECK(rig.step(milliseconds{10}, output_follower::default_changed) == action::reopen);
	rig.follower.succeeded(true, rig.now);
	// and a device that appears may be the one set
	CHECK(rig.step(milliseconds{10}, output_follower::device_added) == action::restore_preferred);
	CHECK(rig.follower.fallback()); // it wasn't, nothing changes
	CHECK(rig.step(milliseconds{10}, output_follower::device_added) == action::restore_preferred);
	rig.follower.succeeded(false, rig.now);
	CHECK_FALSE(rig.follower.fallback());
	CHECK(rig.step(milliseconds{10}, output_follower::default_changed) == action::none);
}

TEST_CASE("audio output: the game started on the default output, the device set in eu07.ini being absent")
{
	output_rig rig{true, true};
	CHECK(rig.follower.fallback());
	CHECK(rig.step(milliseconds{10}, output_follower::device_added) == action::restore_preferred);
}

TEST_CASE("audio output: without a device set in eu07.ini there is nothing to fall back from")
{
	output_rig rig{false, true};
	CHECK_FALSE(rig.follower.fallback());
	rig.follower.succeeded(true, rig.now);
	CHECK_FALSE(rig.follower.fallback());
	CHECK(rig.step(milliseconds{10}, output_follower::device_added) == action::none);
}

TEST_CASE("audio output: after a minute without an output the attempts thin out, the events still count at once")
{
	output_rig rig;
	rig.connected = false;
	CHECK(rig.step(output_follower::check_interval) == action::reopen);
	CHECK(rig.follower.failed(rig.now));
	auto const loststart{rig.now};

	// 0.5, 1, 2, then 4 s until a minute has gone by since the first failure
	while (rig.now - loststart < output_follower::retry_late_after)
	{
		auto const before{rig.now};
		auto next{rig.step(milliseconds{50})};
		while (next != action::reopen)
			next = rig.step(milliseconds{50});
		CHECK(rig.now - before <= output_follower::retry_max);
		rig.follower.failed(rig.now);
	}
	// the failure past the minute puts the next attempt 30 s away
	CHECK(rig.step(output_follower::retry_late - milliseconds{1}) == action::none);
	CHECK(rig.step(milliseconds{1}) == action::reopen);
	rig.follower.failed(rig.now);
	CHECK(rig.step(output_follower::retry_late - milliseconds{1}) == action::none);
	CHECK(rig.step(milliseconds{1}) == action::reopen);
	rig.follower.failed(rig.now);

	// a device coming has the attempt made at once still
	CHECK(rig.step(output_follower::event_retry_min, output_follower::device_added) == action::reopen);
	rig.follower.failed(rig.now);

	// and the next loss starts from the short delays again
	rig.follower.succeeded(false, rig.now);
	CHECK(rig.step(output_follower::check_interval) == action::reopen);
	CHECK(rig.follower.failed(rig.now));
	CHECK(rig.step(output_follower::retry_min) == action::reopen);
}

TEST_CASE("audio output: a move back to the device set in eu07.ini that didn't work out is tried again 3 times")
{
	output_rig rig{true, true};
	CHECK(rig.step(milliseconds{10}, output_follower::device_added) == action::restore_preferred);
	rig.follower.restore_failed(rig.now); // reported before it was ready, say

	// again after 1, 2 and 4 s
	for (auto const delay : {1000, 2000, 4000})
	{
		CHECK(rig.step(milliseconds{delay - 1}) == action::none);
		CHECK(rig.step(milliseconds{1}) == action::restore_preferred);
		rig.follower.restore_failed(rig.now);
	}
	// then not any more: a device that never works doesn't break the sound every few seconds
	for (int second = 0; second < 60; ++second)
		CHECK(rig.step(milliseconds{1000}) == action::none);

	// until a device appears again, which starts a fresh round
	CHECK(rig.step(milliseconds{10}, output_follower::device_added) == action::restore_preferred);
	rig.follower.restore_failed(rig.now);
	CHECK(rig.step(output_follower::restore_retry_min) == action::restore_preferred);

	// the device works this time: nothing more to try
	rig.follower.succeeded(false, rig.now);
	CHECK_FALSE(rig.follower.fallback());
	for (int second = 0; second < 10; ++second)
		CHECK(rig.step(milliseconds{1000}) == action::none);
}

TEST_CASE("audio output: the default output opened anew after a failed move back keeps the round of attempts")
{
	// the device set opened but didn't start, and the library had let go of the default output before: that's opened
	// again, and the move back is still retried, with no endless round
	output_rig rig{true, true};
	CHECK(rig.step(milliseconds{10}, output_follower::device_added) == action::restore_preferred);
	for (int attempt = 0; attempt < output_follower::restore_retries; ++attempt)
	{
		rig.follower.succeeded(true, rig.now);
		rig.follower.restore_failed(rig.now);
		CHECK(rig.follower.fallback());
		action next{action::none};
		for (int frame = 0; frame < 1000 && next == action::none; ++frame)
			next = rig.step(milliseconds{10});
		CHECK(next == action::restore_preferred);
	}
	rig.follower.succeeded(true, rig.now);
	rig.follower.restore_failed(rig.now);
	for (int second = 0; second < 60; ++second)
		CHECK(rig.step(milliseconds{1000}) == action::none);
}

TEST_CASE("audio output: the settings applied drop a pending move back")
{
	output_rig rig{true, true};
	CHECK(rig.step(milliseconds{10}, output_follower::device_added) == action::restore_preferred);
	rig.follower.restore_failed(rig.now);
	// the settings applied: the device set may be another one now, a pending attempt is dropped
	rig.follower.start(true, true, rig.now);
	for (int second = 0; second < 10; ++second)
		CHECK(rig.step(milliseconds{1000}) == action::none);
}
