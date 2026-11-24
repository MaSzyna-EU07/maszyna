#pragma once

// tracks read from the text of their entries and written back by the game's own TTrack::Load() and export_as_text(), with no scene:
// the parts of the engine a track reaches for are stand-ins (track_stubs.cpp)

#include "doctest.h"
#include "world/Track.h"
#include "utilities/Globals.h"
#include "utilities/parser.h"

#include <memory>
#include <sstream>
#include <string>

void unit_tests_material_renderer(bool Enabled); // track_stubs.cpp

namespace test {

// for as long as it lives the tracks can be read: a renderer to keep their materials, the buckling of the rails off
class track_reading {
  public:
	track_reading() : m_buckling{Global.track_buckling} {
		unit_tests_material_renderer(true);
		Global.track_buckling = global_settings::track_buckling_mode::off;
	}
	~track_reading() {
		Global.track_buckling = m_buckling;
		unit_tests_material_renderer(false);
		TIsolated::DeleteAll();
	}
	track_reading(track_reading const &) = delete;
	track_reading &operator=(track_reading const &) = delete;

  private:
	global_settings::track_buckling_mode m_buckling;
};

// the track of the node entry, read as the scenery loader reads it (state_serializer::deserialize_node and deserialize_path)
inline std::unique_ptr<TTrack> read_track(std::string const &Text) {
	// with a spare endtrack: Load() reads the attributes until it meets one, and an entry short of a number would have it read on past
	// the end of the text forever, where now it stops and the entry comes out different
	cParser parser{Text + " endtrack", cParser::buffer_TEXT};
	parser.getTokens();
	std::string keyword;
	parser >> keyword;
	REQUIRE(keyword == "node");
	scene::node_data data;
	parser.getTokens(4);
	parser >> data.range_max >> data.range_min >> data.name >> data.type;
	if (data.name == "none") {
		data.name.clear();
	}
	REQUIRE(data.type == "track");
	auto track{std::make_unique<TTrack>(data)};
	track->Load(&parser, glm::dvec3{0.0});
	return track;
}

// the entry of the track, as the scenery save writes it
inline std::string written(TTrack const &Track) {
	std::ostringstream text;
	Track.export_as_text(text);
	return text.str();
}

} // namespace test
