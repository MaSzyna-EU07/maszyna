#pragma once

#include "simulation/simulation.h"
#include "world/Event.h"
#include "scene/scene.h"
#include "world/MemCell.h"
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

using object_list = std::vector<std::shared_ptr<map_object>>;
using sorted_object_list = std::map<float, std::shared_ptr<map_object>>;

// semaphore description (only for minimap purposes)
struct semaphore : map_object
{
	std::vector<TAnimModel *> models;
	std::vector<basic_event *> events;
	TMemCell *memcell = nullptr;
	bool shunting { false };

	gfx::basic_vertex vertex() override {
		bool stop_signal = memcell && memcell->IsVelocity() && (memcell->Value1() == 0.0);
		return gfx::basic_vertex(location, glm::vec3(),
		    (!stop_signal) ? glm::vec2(0.0f, 0.2f) : glm::vec2(0.2f, 0.4f));
	}
};

// event launcher description (only for minimap purposes)
struct launcher : map_object
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
struct track_switch : map_object
{
    std::array<basic_event*, 4> action = { nullptr };
    std::array<char[4], 4> preview;
    std::array<TTrack*, 4> track = { nullptr };

    gfx::basic_vertex vertex() override {
        return gfx::basic_vertex(location, glm::vec3(), glm::vec2(0.4f, 0.6f));
    }
};

// training obstacle description
struct obstacle : map_object
{
	std::string model_name;

	gfx::basic_vertex vertex() override {
		return gfx::basic_vertex(location, glm::vec3(), glm::vec2(0.8f, 1.0f));
	}
};

struct vehicle : map_object
{
	TDynamicObject *dynobj = nullptr;
};

struct objects
{
	struct prepared_section_geometry
	{
		glm::dvec2 minimum;
		glm::dvec2 maximum;
		gfx::geometrybank_handle geometry;
	};

	object_list entries;
	gfx::geometrybank_handle geometry_bank;
	std::vector<prepared_section_geometry> section_geometry;
	std::vector<prepared_section_geometry> road_geometry;
	bool poi_dirty = true;
	sorted_object_list find_in_range(glm::vec3 from, float distance) const;
};

void prepare();

extern objects Objects;

} // namespace map
