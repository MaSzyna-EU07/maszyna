#pragma once

#include "simulation/simulation.h"
#include "world/Event.h"
#include "scene/scene.h"
#include "world/MemCell.h"
#include <chrono>
#include <cstdint>

namespace map
{

struct map_object
{
	std::string name;
	glm::vec3 location;

	virtual ~map_object() = default;
	virtual gfx::basic_vertex vertex() {
		return gfx::basic_vertex(location, glm::vec3(), glm::vec2(0.0f, 0.20f));
	}
};

using object_list = std::vector<std::shared_ptr<map::map_object>>;
using sorted_object_list = std::map<float, std::shared_ptr<map::map_object>>;

// semaphore description (only for minimap purposes)
struct semaphore : public map_object
{
	struct light_cache
	{
		TAnimModel *model { nullptr };
		std::uint64_t revision { 0 };
		std::chrono::steady_clock::time_point refreshed_at {};
		glm::dvec2 direction {};
		std::vector<std::uint32_t> colors;
		bool valid { false };
		bool direction_valid { false };
	};

	std::vector<TAnimModel *> models;
	std::vector<light_cache> light_caches;
	light_cache fallback_cache;
	std::vector<basic_event *> events;
	TMemCell *memcell = nullptr;

	gfx::basic_vertex vertex() override {
		bool stop_signal = memcell->IsVelocity() && (memcell->Value1() == 0.0);
		return gfx::basic_vertex(location, glm::vec3(),
		    (!stop_signal) ? glm::vec2(0.0f, 0.2f) : glm::vec2(0.2f, 0.4f));
	}
};

// event launcher description (only for minimap purposes)
struct launcher : public map_object
{
	basic_event *first_event = nullptr;
	basic_event *second_event = nullptr;

	enum type_e {
		track_switch,
		level_crossing
	};
	type_e type;

	gfx::basic_vertex vertex() override {
		return gfx::basic_vertex(location, glm::vec3(),
		    type == track_switch ? glm::vec2(0.4f, 0.6f) : glm::vec2(0.6f, 0.8f));
	}
};

// switch description (only for minimap purposes)
struct track_switch : public map_object
{
    std::array<basic_event*, 4> action = { nullptr };
    std::array<char[4], 4> preview;
    std::array<TTrack*, 4> track = { nullptr };

    gfx::basic_vertex vertex() override {
        return gfx::basic_vertex(location, glm::vec3(), glm::vec2(0.4f, 0.6f));
    }
};

// training obstacle description
struct obstacle : public map_object
{
	std::string model_name;

	gfx::basic_vertex vertex() override {
		return gfx::basic_vertex(location, glm::vec3(), glm::vec2(0.8f, 1.0f));
	}
};

// vehicle wrapper for map display
struct vehicle : public map_object
{
	TDynamicObject *dynobj = nullptr;
};

struct objects
{
	std::vector<std::shared_ptr<map_object>> entries;

	// returns objects in range from vec3, NaN in Y ignores it
	sorted_object_list find_in_range(glm::vec3 from, float distance) const;
};

extern objects Objects;
} // namespace map
