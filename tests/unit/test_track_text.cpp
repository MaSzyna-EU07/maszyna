// the tracks of the scenery files read and written back by the game's own TTrack::Load() and export_as_text(), the safety net of the
// track editor, which writes the tracks it changes this way: the entries in data/tracks/ are written the way the game writes them,
// so each read and written back comes out the same, and what a second reading gives is what the first gave. what doesn't survive yet
// is in data/tracks/known_lost.scn, the tests of it fail until it does (doctest::should_fail)
//
// the tracks are read with no scene: the parts of the engine a track reaches for are stand-ins (track_stubs.cpp). the materials are
// kept by the names given, where the game's material manager names them after the files it finds; the rails don't buckle by the heat,
// world/TrackBuckle.cpp isn't a part of the tests, the buckle declared is read and written either way

#include "stdafx.h"
#include "doctest.h"
#include "track_text.h"
#include "rendering/renderer.h"

#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

using test::read_track;
using test::track_reading;
using test::written;

std::string material_name(material_handle const Material) {
	return Material != null_handle ? GfxRenderer->Material(Material)->GetName() : std::string{"-"};
}

void add_path(std::ostringstream &Text, segment_data const &Path) {
	for (auto const &point : Path.points) {
		Text << ' ' << point.x << ' ' << point.y << ' ' << point.z;
	}
	Text << " rolls " << Path.rolls[0] << ' ' << Path.rolls[1] << " radius " << Path.radius;
}

// what the track got of its entry, field by field, one to a line
std::string fields(TTrack &Track) {
	std::ostringstream text;
	text.precision(17);
	text << "name " << Track.name() << '\n'
	     << "type " << Track.eType << " category " << Track.iCategoryFlag << " tributary " << Track.m_tributary << '\n'
	     << "width " << Track.fTrackWidth << " friction " << Track.fFriction << " sound " << Track.fSoundDistance << '\n'
	     << "quality " << Track.iQualityFlag << " damage " << Track.iDamageFlag << " environment " << Track.eEnvironment << '\n'
	     << "visible " << Track.visible() << '\n'
	     << "materials " << material_name(Track.m_material1) << ' ' << material_name(Track.m_material2) << '\n'
	     << "texture " << Track.fTexLength << ' ' << Track.fTexHeight1 - Track.fTexHeightOffset << ' ' << Track.fTexWidth << ' ' << Track.fTexSlope << '\n';
	for (auto const &path : Track.m_paths) {
		text << "path";
		add_path(text, path);
		text << '\n';
		for (auto const &piece : path.pieces) {
			text << "  piece";
			add_path(text, piece);
			text << '\n';
		}
	}
	for (auto const *events : {&Track.m_events0, &Track.m_events0all, &Track.m_events1, &Track.m_events1all, &Track.m_events2, &Track.m_events2all}) {
		text << "events";
		for (auto const &event : *events) {
			text << ' ' << event.first;
		}
		text << '\n';
	}
	text << "velocity " << Track.VelocityGet() << '\n';
	if (Track.SwitchExtension) {
		auto const &extension{*Track.SwitchExtension};
		text << "switch velocity " << extension.fVelocity << " trackbed " << material_name(extension.m_material3) << " frogs " << extension.m_frogs << " movable nose " << extension.m_movablenose
		     << " blades " << extension.m_bladeheel << ' ' << extension.m_bladethrow << '\n';
	}
	text << "isolated";
	for (auto const *isolated : Track.Isolated) {
		text << ' ' << isolated->asName;
	}
	text << '\n'
	     << "overhead " << Track.fOverhead << " vertical radius " << Track.fVerticalRadius << '\n'
	     << "rail profile " << Track.m_profile1.first << '\n'
	     << track_uneven::write_attribute(Track.m_uneven, Track.eType == tt_Switch) << '\n'
	     << track_buckle::write_buckle(Track.m_buckledeclared) << track_buckle::write_cwr(Track.m_cwr) << '\n'
	     << "friction cell " << Track.m_friction.first << '\n'
	     << "sleepers " << Track.m_sleeper_enabled << ' ' << Track.m_sleeper_frequency << ' ' << Track.m_sleeper_model_name << ' ' << Track.m_sleeper_skin_name << ' ' << Track.m_sleeper_offset.x << ' '
	     << Track.m_sleeper_offset.y << ' ' << Track.m_sleeper_offset.z << ' ' << Track.m_sleeper_ballast_z << '\n'
	     << "timbers " << Track.m_timber_model_name << ' ' << Track.m_timber_skin_name << '\n';
	return text.str();
}

// the entries of the file of data/tracks/, a line each; lines starting with // are comments
std::vector<std::string> entries(std::string const &File) {
	std::ifstream file{std::string{EU07_UNIT_TESTS_DATA} + "/tracks/" + File, std::ios::binary};
	REQUIRE(file.is_open());
	std::vector<std::string> result;
	for (std::string line; std::getline(file, line);) {
		if (false == line.empty() && line.back() == '\r') {
			line.pop_back();
		}
		if (line.empty() || line.starts_with("//")) {
			continue;
		}
		result.push_back(line + '\n');
	}
	return result;
}

// the entry of the track of the name in the file
std::string entry(std::string const &File, std::string const &Name) {
	for (auto const &text : entries(File)) {
		std::istringstream words{text};
		std::string node, rangemax, rangemin, name;
		words >> node >> rangemax >> rangemin >> name;
		if (name == Name) {
			return text;
		}
	}
	FAIL("no track " << Name << " in " << File);
	return {};
}

// read, written, read again and written again
struct round_trip {
	explicit round_trip(std::string const &Text) : first{read_track(Text)}, text{written(*first)}, second{read_track(text)}, again{written(*second)} {}
	std::unique_ptr<TTrack> first;
	std::string text;
	std::unique_ptr<TTrack> second;
	std::string again;
};

} // namespace

TEST_CASE("track text: each entry of data/tracks/ read and written back comes out the same, and read again gives the same track") {
	track_reading const reading;
	for (auto const *file : {"plain.scn", "switch.scn", "turntable.scn", "pieces.scn", "speeds.scn", "other.scn"}) {
		for (auto const &text : entries(file)) {
			CAPTURE(file);
			CAPTURE(text);
			round_trip const trip{text};
			CHECK(trip.text == text);
			CHECK(fields(*trip.second) == fields(*trip.first));
			CHECK(trip.again == trip.text);
		}
	}
}

TEST_CASE("track text: what the game writes otherwise than it was given, the same from the first save on") {
	track_reading const reading;
	struct normalized {
		char const *what;
		std::string given;
		std::string written;
	};
	std::string const plain{"node -1 0 n track normal 10 1.435 0.15 25 20 0 flat unvis 0 0 0 0 0 0 0 0 0 0 0 0 10 0 0 "};
	std::string const turntable{"1.435 0.15 25 20 0 flat unvis -10 0 0 0 0 0 0 0 0 0 10 0 0 0 0 endtrack\n"};
	std::string const switchpaths{"0 0 0 0 0 0 0 0 0 0 0 0 33.23 0 0 0 0 0 0 0 0 11 0.6 0 -11 1.8 0 33.1 0 300 endtrack\n"};
	std::vector<normalized> const cases{
	    {"a turntable entered as table is written as turn", "node -1 0 t track table 20 " + turntable, "node -1 0 t track turn 20 " + turntable},
	    {"mountain is written as mountains", "node -1 0 m track normal 10 1.435 0.15 25 20 0 mountain unvis 0 0 0 0 0 0 0 0 0 0 0 0 10 0 0 endtrack\n",
	     "node -1 0 m track normal 10 1.435 0.15 25 20 0 mountains unvis 0 0 0 0 0 0 0 0 0 0 0 0 10 0 0 endtrack\n"},
	    {"an environment the game doesn't know is written as none", "node -1 0 e track normal 10 1.435 0.15 25 20 0 desert unvis 0 0 0 0 0 0 0 0 0 0 0 0 10 0 0 endtrack\n",
	     "node -1 0 e track normal 10 1.435 0.15 25 20 0 none unvis 0 0 0 0 0 0 0 0 0 0 0 0 10 0 0 endtrack\n"},
	    {"the length is the one of the path, whatever the entry says", "node -1 0 n track normal 12 1.435 0.15 25 20 0 flat unvis 0 0 0 0 0 0 0 0 0 0 0 0 10 0 0 endtrack\n", plain + "endtrack\n"},
	    {"a switch is heard from 10 m, whatever the entry says", "node -1 0 s track switch 33.23 1.435 0.25 25 20 0 flat unvis " + switchpaths,
	     "node -1 0 s track switch 33.23 1.435 0.25 10 20 0 flat unvis " + switchpaths},
	    {"the events go in the order of their kinds", plain + "event2 b eventall1 c event0 a endtrack\n", plain + "event0 a eventall1 c event2 b endtrack\n"},
	    {"the names are read in lower case, the paths of the models as they are, with forward slashes",
	     "node -1 0 N track normal 10 1.435 0.15 25 20 0 flat unvis 0 0 0 0 0 0 0 0 0 0 0 0 10 0 0 event0 Sem_A isolated Iso_B sleepermodel 0.6 Models\\Tory\\Podklad.e3d Skin 0 0 0 0 endtrack\n",
	     plain + "event0 sem_a isolated iso_b sleepermodel 0.6 Models/Tory/Podklad.e3d Skin 0 0 0 0 endtrack\n"},
	    {"a texture repeated at less than 1 cm is repeated at 4 m", "node -1 0 n track normal 10 1.435 0.15 25 20 0 flat vis rail 0 ballast 0.2 0.5 1.1 0 0 0 0 0 0 0 0 0 0 0 0 10 0 0 endtrack\n",
	     "node -1 0 n track normal 10 1.435 0.15 25 20 0 flat vis rail 4 ballast 0.2 0.5 1.1 0 0 0 0 0 0 0 0 0 0 0 0 10 0 0 endtrack\n"},
	    {"the rolls are written as the floats they're read into", "node -1 0 n track normal 10 1.435 0.15 25 20 0 flat unvis 0 0 0 3.4377 0 0 0 0 0 0 0 0 10 -3.4377 0 endtrack\n",
	     "node -1 0 n track normal 10 1.435 0.15 25 20 0 flat unvis 0 0 0 3.43770003318787 0 0 0 0 0 0 0 0 10 -3.43770003318787 0 endtrack\n"},
	    {"no overhead line (-1) and no vertical radius (0) aren't written", plain + "overhead -1 vradius 0 endtrack\n", plain + "endtrack\n"},
	};
	for (auto const &[what, given, expected] : cases) {
		CAPTURE(what);
		round_trip const trip{given};
		CHECK(trip.text == expected);
		CHECK(trip.again == trip.text);
	}
}

// what doesn't survive yet: each fails until it does, then doctest reports the test passing against should_fail, and the entry goes
// over to the files of the entries which come out the same

TEST_CASE("track text: the angles the ends are cut at are written back (known lost: read and dropped)" * doctest::should_fail()) {
	track_reading const reading;
	round_trip const trip{entry("known_lost.scn", "angles")};
	CHECK(trip.text.find(" angle1 15 ") != std::string::npos);
	CHECK(trip.text.find(" angle2 -15 ") != std::string::npos);
}

TEST_CASE("track text: the fouling points are written back (known lost: read and dropped)" * doctest::should_fail()) {
	track_reading const reading;
	round_trip const trip{entry("known_lost.scn", "fouling")};
	CHECK(trip.text.find(" fouling1 ukres_a ") != std::string::npos);
	CHECK(trip.text.find(" fouling2 ukres_b ") != std::string::npos);
}

TEST_CASE("track text: a name with the isolated circuit after @ keeps its letters (known lost: the one before the @)" * doctest::should_fail()) {
	track_reading const reading;
	round_trip const trip{entry("known_lost.scn", "tor1@iso1")};
	// the circuit is taken out of the name and written as an attribute, which reads the same
	REQUIRE(trip.first->Isolated.size() == 1);
	CHECK(trip.first->Isolated.front()->asName == "iso1");
	CHECK(trip.first->name() == "tor1");
}

TEST_CASE("track text: the attributes keep every digit of the floats they're read into (known lost: written to 6 digits)" * doctest::should_fail()) {
	track_reading const reading;
	round_trip const trip{entry("known_lost.scn", "digits")};
	CHECK(fields(*trip.second) == fields(*trip.first));
}
