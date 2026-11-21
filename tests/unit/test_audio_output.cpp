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
	// whatever the system reports may be the output coming back
	CHECK(rig.step(milliseconds{1}, output_follower::device_added) == action::reopen);
	CHECK_FALSE(rig.follower.failed(rig.now));
	CHECK(rig.step(milliseconds{1}, output_follower::default_changed) == action::reopen);

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
