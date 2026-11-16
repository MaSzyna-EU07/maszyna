/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/mapscreen.h"

#ifdef WITH_RMLUI

#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include <RmlUi/Core.h>

#include "application/driveruipanels.h"
#include "application/rmluidocument.h"
#include "application/rmluilayer.h"
#include "imgui/imgui.h"
#include "model/AnimModel.h"
#include "simulation/simulation.h"
#include "utilities/translation.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"
#include "widgets/map.h"
#include "world/Event.h"

namespace
{

// staticFiles/ui/map.rml of the sources, written out by CMake as the bytes of map_rml
#include "map_rml.inc"

// a train on the map, as the document places it
struct mark_row
{
	Rml::String key; // the name of the vehicle
	Rml::String name;
	int speed{0};
	Rml::String left; // px within the view
	Rml::String top;
	Rml::String heading; // rotate(...deg) of the arrow
	bool own{false};
	bool selected{false};

	bool operator==(mark_row const &) const = default;
};

// a train of the list
struct train_row
{
	Rml::String key;
	Rml::String name;
	int speed{0};
	Rml::String route;
	Rml::String next;
	bool late{false};
	Rml::String delay;
	bool ai{false};
	bool own{false};
	bool selected{false};

	bool operator==(train_row const &) const = default;
};

// a vehicle drawn to scale
struct vehicle_row
{
	Rml::String left;
	Rml::String top;
	Rml::String width; // its length
	Rml::String height; // its width
	Rml::String transform;
	bool powered{false};
	bool own{false};

	bool operator==(vehicle_row const &) const = default;
};

// a line of the grid
struct line_row
{
	Rml::String left;
	Rml::String top;
	Rml::String width;
	Rml::String rotate;

	bool operator==(line_row const &) const = default;
};

struct lamp_row
{
	Rml::String colour;

	bool operator==(lamp_row const &) const = default;
};

struct head_row
{
	Rml::String name;
	std::vector<lamp_row> lamps;

	bool operator==(head_row const &) const = default;
};

// an object to choose of those at the point clicked, or a button of the object chosen
struct choice_row
{
	Rml::String name;
	Rml::String kind;

	bool operator==(choice_row const &) const = default;
};

// the values of the data model of the document, see staticFiles/ui/map.rml
struct map_values
{
	bool shown{false};
	bool available{false};
	bool closed{false}; // the button of the bar closed the map
	// the layers
	bool layer_names{false};
	bool layer_objects{true};
	bool layer_grid{false};
	// the trains
	std::vector<mark_row> marks;
	std::vector<train_row> trains;
	std::vector<vehicle_row> vehicles;
	bool vehicles_drawn{false};
	Rml::String selected_key;
	train_row card;
	bool card_shown{false};
	// the scale, the grid
	Rml::String scale_width;
	Rml::String scale_text;
	std::vector<line_row> grid;
	// the nearest signal ahead of the train driven
	bool sig_shown{false};
	Rml::String sig_name;
	Rml::String sig_distance;
	std::vector<head_row> sig_heads;
	// the object under the cursor
	bool hover_shown{false};
	Rml::String hover_left;
	Rml::String hover_top;
	Rml::String hover_kind;
	Rml::String hover_name;
	std::vector<head_row> hover_heads;
	// the object clicked with the right button
	bool obj_shown{false};
	Rml::String obj_left;
	Rml::String obj_top;
	Rml::String obj_kind;
	Rml::String obj_name;
	std::vector<head_row> obj_heads;
	std::vector<choice_row> obj_choices;
	Rml::String obj_actions_title;
	std::vector<choice_row> obj_actions;
};

// the object clicked with the right button: the objects at the point to choose of, the one chosen and what can be done to it,
// or the obstacles to put on the track there
struct object_panel
{
	bool open{false};
	Rml::Vector2f at{0.f, 0.f}; // where it was clicked, px within the view
	map::sorted_object_list choices;
	std::shared_ptr<map::map_object> object;
	Rml::String title; // above the buttons
	std::vector<Rml::String> labels;
	std::vector<std::function<void()>> actions;
	std::vector<std::vector<std::pair<TTrack *, int>>> previews; // of the positions of a switch
	bool close_after{false}; // the panel closes when a button was pressed
};

// a click of the view, kept until the frame of imgui the map acts on it in
struct pending_click
{
	int button{0};
	Rml::Vector2f position;
};

// the mouse on the view of the map: what it did is kept until the map is drawn, in the frame of imgui
class view_input final : public Rml::EventListener
{
  public:
	void ProcessEvent(Rml::Event &Event) override
	{
		auto *view{Event.GetCurrentElement()};
		Rml::Vector2f const mouse{Event.GetParameter<float>("mouse_x", 0.f), Event.GetParameter<float>("mouse_y", 0.f)};
		auto const local{mouse - view->GetAbsoluteOffset(Rml::BoxArea::Border)};
		switch (Event.GetId())
		{
		case Rml::EventId::Mousedown:
			press(Event.GetParameter<int>("button", 0), mouse, local);
			break;
		case Rml::EventId::Mousemove:
			move(mouse, local);
			break;
		case Rml::EventId::Mouseout:
			hovering = false;
			break;
		case Rml::EventId::Mousescroll:
			// up zooms in
			zoom -= Event.GetParameter<float>("wheel_delta_y", 0.f);
			zoomed = true;
			zoom_at = local;
			Event.StopPropagation();
			break;
		default:
			break;
		}
	}

	bool dragging{false};
	Rml::Vector2f drag{0.f, 0.f}; // since the last frame
	bool dragged{false};
	float zoom{0.f}; // steps since the last frame
	bool zoomed{false};
	Rml::Vector2f zoom_at{0.f, 0.f};
	bool hovering{false};
	Rml::Vector2f hover_at{0.f, 0.f};
	std::vector<pending_click> clicks;

  private:
	void press(int const Button, Rml::Vector2f const Mouse, Rml::Vector2f const Local)
	{
		if (Button == 0)
		{
			dragging = true;
			m_last = Mouse;
		}
		clicks.emplace_back(Button, Local);
	}

	void move(Rml::Vector2f const Mouse, Rml::Vector2f const Local)
	{
		if (dragging)
		{
			drag += Mouse - m_last;
			dragged = true;
			m_last = Mouse;
		}
		hovering = true;
		hover_at = Local;
	}

	Rml::Vector2f m_last{0.f, 0.f};
};

// the look of the map of RmlUi: the tracks on green land; the lines wide while there are few of them seen
ui::map_panel::style map_style(bool const Objects, bool const Wide)
{
	ui::map_panel::style style;
	style.background = glm::vec3{0.125f, 0.19f, 0.115f};
	style.track = glm::vec3{0.80f, 0.80f, 0.74f};
	style.future = glm::vec3{0.95f, 0.70f, 0.15f};
	style.switches = glm::vec3{0.25f, 0.85f, 0.95f};
	style.occupied = glm::vec3{1.00f, 0.25f, 0.25f};
	style.highlighted = glm::vec3{0.40f, 0.55f, 1.00f};
	style.thickness = Wide ? 1 : 0;
	style.icons = Objects;
	return style;
}

// the nearest of 1, 2 or 5 times a power of 10 metres from Metres up
float nice_metres(float const Metres)
{
	auto const power{std::pow(10.f, std::floor(std::log10(std::max(Metres, 1.f))))};
	for (auto const factor : {1.f, 2.f, 5.f})
	{
		if (factor * power >= Metres)
			return factor * power;
	}
	return 10.f * power;
}

Rml::String metres_text(float const Metres)
{
	if (Metres >= 1000.f)
		return std::format("{:g} km", Metres / 1000.f);
	return std::format("{:.0f} m", Metres);
}

// whole pixels, so that a part moving less than a pixel isn't laid out again
Rml::String pixels(float const Value)
{
	return std::format("{:.0f}px", Value);
}

Rml::String rotation(float const Degrees)
{
	return std::format("rotate({:.1f}deg)", Degrees);
}

Rml::String colour_text(glm::vec3 const &Colour)
{
	auto const channel = [](float const Value) { return static_cast<int>(std::lround(std::clamp(Value, 0.f, 1.f) * 255.f)); };
	return std::format("rgb({},{},{})", channel(Colour.r), channel(Colour.g), channel(Colour.b));
}

// the list of the trains and the card of the chosen one
train_row train_of(ui::map_panel::train const &Train, Rml::String const &Selected)
{
	auto const *vehicle{Train.vehicle};
	auto const &timetable{vehicle->Mechanik->TrainTimetable()};
	train_row row;
	row.key = vehicle->name();
	row.name = rmlui::game_text(Train.name, true);
	row.speed = static_cast<int>(std::floor(vehicle->MoverParameters->Vel));
	if (false == timetable.Relation1.empty() || false == timetable.Relation2.empty())
		row.route = rmlui::game_text(timetable.Relation1, true) + " \xE2\x80\x93 " + rmlui::game_text(timetable.Relation2, true); // an en dash
	row.next = rmlui::game_text(timetable.NextStationName, true);
	auto const delay{static_cast<int>(std::lround(std::max(0.0, -timetable.CheckTrainLatency())))};
	row.late = delay > 0;
	row.delay = std::format("+{} min", delay);
	row.ai = vehicle->Mechanik->AIControllFlag;
	row.own = simulation::Train != nullptr && simulation::Train->Dynamic() == vehicle;
	row.selected = row.key == Selected;
	return row;
}

mark_row mark_of(ui::map_panel::train const &Train, train_row const &Row)
{
	mark_row mark;
	mark.key = Row.key;
	mark.name = Row.name;
	mark.speed = Row.speed;
	mark.left = pixels(Train.position.x);
	mark.top = pixels(Train.position.y);
	mark.heading = rotation(Train.heading);
	mark.own = Row.own;
	mark.selected = Row.selected;
	return mark;
}

void fill_trains(map_values &Values, std::vector<ui::map_panel::train> const &Trains)
{
	Values.trains.clear();
	Values.marks.clear();
	Values.card_shown = false;
	for (auto const &train : Trains)
	{
		auto row{train_of(train, Values.selected_key)};
		if (train.shown)
			Values.marks.push_back(mark_of(train, row));
		if (row.selected)
		{
			Values.card = row;
			Values.card_shown = true;
		}
		Values.trains.push_back(std::move(row));
	}
	// the train driven first, the others by their names
	std::ranges::stable_sort(Values.trains, [](train_row const &Left, train_row const &Right) { return Left.own != Right.own ? Left.own : Left.name < Right.name; });
	if (false == Values.card_shown)
		Values.selected_key.clear();
}

// the rolling stock to scale, while a vehicle is at least 6 pixels long
void fill_vehicles(map_values &Values, ui::map_panel const &Map)
{
	Values.vehicles.clear();
	for (auto const &vehicle : Map.vehicles(6.f))
	{
		vehicle_row row;
		row.left = pixels(vehicle.position.x);
		row.top = pixels(vehicle.position.y);
		row.width = pixels(vehicle.length);
		row.height = pixels(vehicle.width);
		row.transform = std::format("translate(-50%, -50%) rotate({:.0f}deg)", vehicle.angle);
		row.powered = vehicle.powered;
		row.own = vehicle.own;
		Values.vehicles.push_back(std::move(row));
	}
	Values.vehicles_drawn = false == Values.vehicles.empty();
}

void fill_scale(map_values &Values, ui::map_panel const &Map)
{
	auto const perpixel{Map.pixels_per_metre()};
	if (perpixel <= 0.f)
		return;
	// a bar of about 120 px, a grid of about 150 px
	auto const length{nice_metres(120.f / perpixel)};
	Values.scale_width = pixels(length * perpixel);
	Values.scale_text = metres_text(length);
	Values.grid.clear();
	if (false == Values.layer_grid)
		return;
	for (auto const &segment : Map.grid(nice_metres(150.f / perpixel)))
	{
		auto const delta{segment.to - segment.from};
		line_row line;
		line.left = pixels(segment.from.x);
		line.top = pixels(segment.from.y);
		line.width = pixels(glm::length(delta));
		line.rotate = rotation(glm::degrees(std::atan2(delta.y, delta.x)));
		Values.grid.push_back(std::move(line));
	}
}

std::vector<head_row> heads_of(ui::map_panel::object_info const &Object)
{
	std::vector<head_row> heads;
	for (auto const &head : Object.heads)
	{
		head_row row;
		row.name = rmlui::game_text(head.name, false);
		for (auto const &lamp : head.lamps)
			row.lamps.emplace_back(colour_text(lamp));
		heads.push_back(std::move(row));
	}
	return heads;
}

// the lamps of the heads of a signal as they shine now
std::vector<head_row> heads_of(map::semaphore const &Semaphore)
{
	ui::map_panel::object_info info;
	for (auto *model : Semaphore.models)
	{
		std::vector<glm::vec3> colours;
		for (auto const &lamp : ui::map_actions::lamps(model))
			colours.push_back(lamp.colour);
		info.heads.emplace_back(model->name(), std::move(colours));
	}
	return heads_of(info);
}

// the signal the driver of the train driven sees next, found once for each event of a signal
struct signal_cache
{
	basic_event const *event{nullptr};
	std::shared_ptr<map::semaphore> semaphore;
};

void fill_signal(map_values &Values, signal_cache &Cache)
{
	Values.sig_shown = false;
	auto const aid{drivingaid_panel::read()};
	if (false == aid.has_value() || aid->next_signal == nullptr)
		return;
	if (aid->next_signal != Cache.event)
	{
		Cache.event = aid->next_signal;
		Cache.semaphore = ui::map_panel::semaphore_at(aid->next_signal->input_location());
	}
	if (Cache.semaphore == nullptr)
		return;
	Values.sig_shown = true;
	Values.sig_name = rmlui::game_text(Cache.semaphore->name, true);
	auto const distance{std::max(0.0, aid->next_signal_distance)};
	Values.sig_distance = distance < 1000.0 ? std::format("{:.0f} m", distance) : std::format("{:.1f} km", distance / 1000.0);
	Values.sig_heads = heads_of(*Cache.semaphore);
}

void fill_hover(map_values &Values, ui::map_panel const &Map, view_input const &Input)
{
	Values.hover_shown = false;
	if (false == Input.hovering || Input.dragging || false == Values.layer_objects)
		return;
	auto const object{Map.object_at(glm::vec2{Input.hover_at.x, Input.hover_at.y})};
	if (false == object.has_value())
		return;
	Values.hover_shown = true;
	Values.hover_left = pixels(Input.hover_at.x + 18.f);
	Values.hover_top = pixels(Input.hover_at.y + 18.f);
	Values.hover_kind = Translations.lookup_s(object->kind);
	Values.hover_name = rmlui::game_text(object->name, false);
	Values.hover_heads = heads_of(*object);
}

void fill_panel(map_values &Values, object_panel const &Panel)
{
	Values.obj_shown = Panel.open;
	if (false == Panel.open)
		return;
	Values.obj_left = pixels(Panel.at.x + 14.f);
	Values.obj_top = pixels(Panel.at.y + 14.f);
	Values.obj_choices.clear();
	for (auto const &[distance, object] : Panel.choices)
		Values.obj_choices.emplace_back(rmlui::game_text(object->name, false), Translations.lookup_s(ui::map_actions::kind(*object)));
	Values.obj_kind = Panel.object != nullptr ? Translations.lookup_s(ui::map_actions::kind(*Panel.object)) : Rml::String{};
	Values.obj_name = Panel.object != nullptr ? rmlui::game_text(Panel.object->name, false) : Rml::String{};
	auto const *semaphore{dynamic_cast<map::semaphore const *>(Panel.object.get())};
	Values.obj_heads = semaphore != nullptr ? heads_of(*semaphore) : std::vector<head_row>{};
	Values.obj_actions_title = Panel.title;
	Values.obj_actions.clear();
	for (auto const &label : Panel.labels)
		Values.obj_actions.emplace_back(label, Rml::String{});
}

void add_action(object_panel &Panel, Rml::String Label, std::function<void()> Action)
{
	Panel.labels.push_back(std::move(Label));
	Panel.actions.push_back(std::move(Action));
}

// the buttons of a signal: the aspects its events set
void add_signal_actions(object_panel &Panel, map::semaphore const &Semaphore)
{
	for (auto const &aspect : ui::map_actions::aspects(Semaphore))
		add_action(Panel, rmlui::game_text(aspect.name, false), [event = aspect.event] { ui::map_actions::run(*event); });
}

// the buttons of a switch of more tracks, with the tracks each position sets
void add_switch_actions(object_panel &Panel, map::track_switch const &Switch)
{
	for (auto const &position : ui::map_actions::positions(Switch))
	{
		add_action(Panel, position.position.name, [event = position.position.event] { ui::map_actions::run(*event); });
		Panel.previews.push_back(position.preview);
	}
}

// the light signals a vehicle can show
void add_vehicle_actions(object_panel &Panel, std::string const &Vehicle)
{
	Panel.title = Translations.lookup_s("Light signal:");
	add_action(Panel, Translations.lookup_s("none"), [Vehicle] { ui::map_actions::set_vehicle_signal(Vehicle, 0); });
	add_action(Panel, Translations.lookup_s("Pc6"), [Vehicle] { ui::map_actions::set_vehicle_signal(Vehicle, Signal_Pc6); });
	add_action(Panel, Translations.lookup_s("A1"), [Vehicle] { ui::map_actions::set_vehicle_signal(Vehicle, Signal_A1); });
	Panel.close_after = true;
}

// an object of the map chosen: what can be done to it, as the windows of the map of imgui do it
void choose(object_panel &Panel, std::shared_ptr<map::map_object> const &Object)
{
	Panel.choices.clear();
	Panel.object = Object;
	Panel.title.clear();
	Panel.labels.clear();
	Panel.actions.clear();
	Panel.previews.clear();
	Panel.close_after = false;
	Panel.open = true;
	if (auto const semaphore = std::dynamic_pointer_cast<map::semaphore>(Object))
		add_signal_actions(Panel, *semaphore);
	else if (auto const launcher = std::dynamic_pointer_cast<map::launcher>(Object))
	{
		for (auto const &position : ui::map_actions::positions(*launcher))
			add_action(Panel, position.name, [event = position.event] { ui::map_actions::run(*event); });
		Panel.close_after = true;
	}
	else if (auto const track_switch = std::dynamic_pointer_cast<map::track_switch>(Object))
		add_switch_actions(Panel, *track_switch);
	else if (auto const obstacle = std::dynamic_pointer_cast<map::obstacle>(Object))
	{
		add_action(Panel, Translations.lookup_s("Delete obstacle"), [obstacle] { ui::map_actions::remove_obstacle(obstacle); });
		Panel.close_after = true;
	}
	else if (auto const vehicle = std::dynamic_pointer_cast<map::vehicle>(Object))
		add_vehicle_actions(Panel, vehicle->name);
}

// the right button at the point: the object there, a choice of the objects there, or the obstacles to put on the track there
void close_panel(object_panel &Panel, ui::map_panel &Map)
{
	if (false == Panel.previews.empty())
		Map.preview_switch({});
	Panel = object_panel{};
}

void open_panel(object_panel &Panel, ui::map_panel &Map, Rml::Vector2f const At)
{
	close_panel(Panel, Map);
	Panel.at = At;
	auto objects{Map.objects_at(glm::vec2{At.x, At.y})};
	if (objects.size() > 1)
	{
		Panel.choices = std::move(objects);
		Panel.open = true;
		return;
	}
	if (objects.size() == 1)
	{
		choose(Panel, objects.begin()->second);
		Panel.at = At;
		return;
	}
	auto const point{Map.track_point_at(glm::vec2{At.x, At.y})};
	auto const bank{ui::map_actions::obstacle_bank()};
	if (false == point.has_value() || bank.empty())
		return;
	Panel.title = Translations.lookup_s("Insert obstacle:");
	for (auto const &entry : bank)
		add_action(Panel, rmlui::game_text(entry.first, false), [position = *point, entry] { ui::map_actions::insert_obstacle(position, entry); });
	Panel.close_after = true;
	Panel.open = true;
}

void register_types(Rml::DataModelConstructor &Constructor)
{
	if (auto mark{Constructor.RegisterStruct<mark_row>()})
	{
		mark.RegisterMember("key", &mark_row::key);
		mark.RegisterMember("name", &mark_row::name);
		mark.RegisterMember("speed", &mark_row::speed);
		mark.RegisterMember("left", &mark_row::left);
		mark.RegisterMember("top", &mark_row::top);
		mark.RegisterMember("heading", &mark_row::heading);
		mark.RegisterMember("own", &mark_row::own);
		mark.RegisterMember("selected", &mark_row::selected);
	}
	Constructor.RegisterArray<std::vector<mark_row>>();
	if (auto train{Constructor.RegisterStruct<train_row>()})
	{
		train.RegisterMember("key", &train_row::key);
		train.RegisterMember("name", &train_row::name);
		train.RegisterMember("speed", &train_row::speed);
		train.RegisterMember("route", &train_row::route);
		train.RegisterMember("next", &train_row::next);
		train.RegisterMember("late", &train_row::late);
		train.RegisterMember("delay", &train_row::delay);
		train.RegisterMember("ai", &train_row::ai);
		train.RegisterMember("own", &train_row::own);
		train.RegisterMember("selected", &train_row::selected);
	}
	Constructor.RegisterArray<std::vector<train_row>>();
	if (auto vehicle{Constructor.RegisterStruct<vehicle_row>()})
	{
		vehicle.RegisterMember("left", &vehicle_row::left);
		vehicle.RegisterMember("top", &vehicle_row::top);
		vehicle.RegisterMember("width", &vehicle_row::width);
		vehicle.RegisterMember("height", &vehicle_row::height);
		vehicle.RegisterMember("transform", &vehicle_row::transform);
		vehicle.RegisterMember("powered", &vehicle_row::powered);
		vehicle.RegisterMember("own", &vehicle_row::own);
	}
	Constructor.RegisterArray<std::vector<vehicle_row>>();
	if (auto line{Constructor.RegisterStruct<line_row>()})
	{
		line.RegisterMember("left", &line_row::left);
		line.RegisterMember("top", &line_row::top);
		line.RegisterMember("width", &line_row::width);
		line.RegisterMember("rotate", &line_row::rotate);
	}
	Constructor.RegisterArray<std::vector<line_row>>();
	if (auto lamp{Constructor.RegisterStruct<lamp_row>()})
		lamp.RegisterMember("colour", &lamp_row::colour);
	Constructor.RegisterArray<std::vector<lamp_row>>();
	if (auto head{Constructor.RegisterStruct<head_row>()})
	{
		head.RegisterMember("name", &head_row::name);
		head.RegisterMember("lamps", &head_row::lamps);
	}
	Constructor.RegisterArray<std::vector<head_row>>();
	if (auto choice{Constructor.RegisterStruct<choice_row>()})
	{
		choice.RegisterMember("name", &choice_row::name);
		choice.RegisterMember("kind", &choice_row::kind);
	}
	Constructor.RegisterArray<std::vector<choice_row>>();
}

void bind_values(Rml::DataModelConstructor &Constructor, rmlui::bound_values<map_values> &Values)
{
	Values.bind(Constructor, "shown", &map_values::shown);
	Values.bind(Constructor, "available", &map_values::available);
	Values.bind(Constructor, "layer_names", &map_values::layer_names);
	Values.bind(Constructor, "layer_objects", &map_values::layer_objects);
	Values.bind(Constructor, "layer_grid", &map_values::layer_grid);
	Values.bind(Constructor, "marks", &map_values::marks);
	Values.bind(Constructor, "trains", &map_values::trains);
	Values.bind(Constructor, "vehicles", &map_values::vehicles);
	Values.bind(Constructor, "vehicles_drawn", &map_values::vehicles_drawn);
	Values.bind(Constructor, "card", &map_values::card);
	Values.bind(Constructor, "card_shown", &map_values::card_shown);
	Values.bind(Constructor, "scale_width", &map_values::scale_width);
	Values.bind(Constructor, "scale_text", &map_values::scale_text);
	Values.bind(Constructor, "sig_shown", &map_values::sig_shown);
	Values.bind(Constructor, "sig_name", &map_values::sig_name);
	Values.bind(Constructor, "sig_distance", &map_values::sig_distance);
	Values.bind(Constructor, "sig_heads", &map_values::sig_heads);
	Values.bind(Constructor, "grid", &map_values::grid);
	Values.bind(Constructor, "hover_shown", &map_values::hover_shown);
	Values.bind(Constructor, "hover_left", &map_values::hover_left);
	Values.bind(Constructor, "hover_top", &map_values::hover_top);
	Values.bind(Constructor, "hover_kind", &map_values::hover_kind);
	Values.bind(Constructor, "hover_name", &map_values::hover_name);
	Values.bind(Constructor, "hover_heads", &map_values::hover_heads);
	Values.bind(Constructor, "obj_shown", &map_values::obj_shown);
	Values.bind(Constructor, "obj_left", &map_values::obj_left);
	Values.bind(Constructor, "obj_top", &map_values::obj_top);
	Values.bind(Constructor, "obj_kind", &map_values::obj_kind);
	Values.bind(Constructor, "obj_name", &map_values::obj_name);
	Values.bind(Constructor, "obj_heads", &map_values::obj_heads);
	Values.bind(Constructor, "obj_choices", &map_values::obj_choices);
	Values.bind(Constructor, "obj_actions_title", &map_values::obj_actions_title);
	Values.bind(Constructor, "obj_actions", &map_values::obj_actions);
}

// whether a layer of the map is shown: names, objects, or the grid
bool &layer_shown(map_values &Values, Rml::String const &Layer)
{
	if (Layer == "names")
		return Values.layer_names;
	if (Layer == "objects")
		return Values.layer_objects;
	return Values.layer_grid;
}

// the index the event was called with, -1 without one
int index_of(Rml::VariantList const &Arguments, std::size_t const Which = 0)
{
	return Arguments.size() > Which ? Arguments[Which].Get<int>(-1) : -1;
}

// what the events of the document do, to the values not worked out each frame and to the map
struct screen_events
{
	map_values *values;
	ui::map_panel *map;
	object_panel *panel;
	bool *refresh; // the values are worked out anew right away
};

void bind_view_events(Rml::DataModelConstructor &Constructor, screen_events const &Events)
{
	Constructor.BindFunc("mode", [map = Events.map](Rml::Variant &Value) { Value = map->view_mode(); });
	Constructor.BindFunc("following", [map = Events.map](Rml::Variant &Value) { Value = false == map->followed().empty(); });
	Constructor.BindEventCallback("view", [map = Events.map](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &Arguments) {
		if (false == Arguments.empty())
			map->set_view_mode(Arguments.front().Get<int>());
	});
	Constructor.BindEventCallback("zoom", [map = Events.map](Rml::DataModelHandle, Rml::Event &Event, Rml::VariantList const &Arguments) {
		// about the middle of the view
		auto *view{Event.GetTargetElement()->GetOwnerDocument()->GetElementById("view")};
		if (Arguments.empty() || view == nullptr)
			return;
		auto const size{view->GetBox().GetSize(Rml::BoxArea::Border)};
		map->zoom_by(Arguments.front().Get<float>(), glm::vec2{size.x * 0.5f, size.y * 0.5f});
	});
	Constructor.BindEventCallback("layer", [values = Events.values](Rml::DataModelHandle Model, Rml::Event &, Rml::VariantList const &Arguments) {
		if (Arguments.empty())
			return;
		auto const layer{Arguments.front().Get<Rml::String>()};
		auto &shown{layer_shown(*values, layer)};
		shown = !shown;
		Model.DirtyVariable("layer_" + layer);
	});
	// a train chosen: its card shown, the map following it
	Constructor.BindEventCallback("select", [values = Events.values, map = Events.map](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &Arguments) {
		if (Arguments.empty())
			return;
		values->selected_key = Arguments.front().Get<Rml::String>();
		map->follow(values->selected_key);
	});
	Constructor.BindEventCallback("unselect", [values = Events.values, map = Events.map](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &) {
		values->selected_key.clear();
		if (false == map->followed().empty())
			map->set_view_mode(map->view_mode());
	});
	Constructor.BindEventCallback("close", [values = Events.values](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &) { values->closed = true; });
}

// a lamp of a head of the signal chosen, set to its next level
void cycle_lamp(object_panel const &Panel, int const Head, int const Lamp)
{
	auto const *semaphore{dynamic_cast<map::semaphore const *>(Panel.object.get())};
	if (semaphore == nullptr || Head < 0 || Head >= static_cast<int>(semaphore->models.size()))
		return;
	auto *model{semaphore->models[static_cast<std::size_t>(Head)]};
	auto const lamps{ui::map_actions::lamps(model)};
	if (Lamp >= 0 && Lamp < static_cast<int>(lamps.size()))
		ui::map_actions::cycle_lamp(model, lamps[static_cast<std::size_t>(Lamp)]);
}

void bind_panel_events(Rml::DataModelConstructor &Constructor, screen_events const &Events)
{
	Constructor.BindEventCallback("pick", [Events](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &Arguments) {
		auto const index{index_of(Arguments)};
		if (index < 0 || index >= static_cast<int>(Events.panel->choices.size()))
			return;
		auto const at{Events.panel->at};
		// a copy, the choices go with the choice
		auto const object{std::next(Events.panel->choices.begin(), index)->second};
		choose(*Events.panel, object);
		Events.panel->at = at;
		*Events.refresh = true;
	});
	Constructor.BindEventCallback("act", [Events](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &Arguments) {
		auto const index{index_of(Arguments)};
		if (index < 0 || index >= static_cast<int>(Events.panel->actions.size()))
			return;
		Events.panel->actions[static_cast<std::size_t>(index)]();
		if (Events.panel->close_after)
			close_panel(*Events.panel, *Events.map);
		*Events.refresh = true;
	});
	Constructor.BindEventCallback("preview", [Events](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &Arguments) {
		auto const index{index_of(Arguments)};
		if (index >= 0 && index < static_cast<int>(Events.panel->previews.size()))
			Events.map->preview_switch(Events.panel->previews[static_cast<std::size_t>(index)]);
		else if (false == Events.panel->previews.empty())
			Events.map->preview_switch({});
		*Events.refresh = true;
	});
	Constructor.BindEventCallback("lamp", [Events](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &Arguments) {
		cycle_lamp(*Events.panel, index_of(Arguments, 0), index_of(Arguments, 1));
		*Events.refresh = true;
	});
	Constructor.BindEventCallback("panel_close", [Events](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &) {
		close_panel(*Events.panel, *Events.map);
		*Events.refresh = true;
	});
}

// what the mouse did since the last frame, done to the map and to the panel of the object. returns: the view changed
bool apply_input(view_input &Input, ui::map_panel &Map, object_panel &Panel)
{
	auto const moved{Input.zoomed || Input.dragged || false == Input.clicks.empty()};
	// a drag ends where the button is let go, over the view or not
	if (false == ImGui::IsMouseDown(ImGuiMouseButton_Left))
		Input.dragging = false;
	if (Input.zoomed)
		Map.zoom_by(Input.zoom, glm::vec2{Input.zoom_at.x, Input.zoom_at.y});
	if (Input.dragged)
		Map.drag_by(glm::vec2{Input.drag.x, Input.drag.y});
	for (auto const &click : Input.clicks)
	{
		if (click.button == 1)
			open_panel(Panel, Map, click.position);
		else if (click.button == 0 && Panel.open)
			close_panel(Panel, Map);
		else
			Map.click(click.button, glm::vec2{click.position.x, click.position.y});
	}
	Input.zoom = 0.f;
	Input.zoomed = false;
	Input.drag = Rml::Vector2f{0.f, 0.f};
	Input.dragged = false;
	Input.clicks.clear();
	return moved;
}

} // namespace

struct map_screen::state
{
	rmlui::bound_values<map_values> values;
	view_input input;
	object_panel panel;
	signal_cache signal;
	bool refresh{false};
	// the image of the map of its last draw, drawn again when the view changes and at most 30 times a second otherwise
	std::optional<ui::map_panel::image> image;
	double drawn_at{-1.0};
	Rml::Vector2f drawn_size{0.f, 0.f};
	bool drawn_objects{true};
	// last, the document is closed before the listener it calls goes
	rmlui::document document;
};

map_screen::map_screen(ui::map_panel &Map) : m_map(&Map) {}

map_screen::~map_screen() = default;

bool map_screen::open()
{
	auto opened{std::make_unique<state>()};
	screen_events const events{&opened->values.current(), m_map, &opened->panel, &opened->refresh};
	auto const bound = [values = &opened->values, events](Rml::DataModelConstructor &Constructor) {
		register_types(Constructor);
		bind_values(Constructor, *values);
		bind_view_events(Constructor, events);
		bind_panel_events(Constructor, events);
	};
	if (false == opened->document.open("map_screen", "ui/map.rml", map_rml, bound))
		return false;
	if (auto *view{opened->document.element()->GetElementById("view")}; view != nullptr)
	{
		for (auto const id : {Rml::EventId::Mousedown, Rml::EventId::Mousemove, Rml::EventId::Mouseout, Rml::EventId::Mousescroll})
			view->AddEventListener(id, &opened->input);
	}
	m_state = std::move(opened);
	return true;
}

bool map_screen::update(bool const Shown)
{
	if (m_failed || false == rmlui::active())
		return false;
	if (m_state == nullptr)
	{
		// loaded when it's needed
		if (false == Shown)
			return true;
		if (false == open())
		{
			m_failed = true;
			return false;
		}
	}
	auto &values{m_state->values.current()};
	auto &model{m_state->document.model()};
	if (values.available != m_map->available())
	{
		values.available = m_map->available();
		model.DirtyVariable("available");
	}
	if (Shown != values.shown)
	{
		values.shown = Shown;
		values.closed = false;
		model.DirtyVariable("shown");
		if (Shown)
			m_state->document.show(true);
		else
		{
			close_panel(m_state->panel, *m_map);
			m_state->document.hide();
		}
	}
	if (Shown)
	{
		// read from the map
		model.DirtyVariable("mode");
		model.DirtyVariable("following");
	}
	return true;
}

void map_screen::draw()
{
	if (m_state == nullptr || false == m_state->values.current().shown || false == m_map->available())
		return;
	auto *view{m_state->document.element()->GetElementById("view")};
	if (view == nullptr)
		return;
	auto &drawn{*m_state};
	auto const moved{apply_input(drawn.input, *m_map, drawn.panel) || std::exchange(drawn.refresh, false)};
	auto const offset{view->GetAbsoluteOffset(Rml::BoxArea::Border)};
	auto const size{view->GetBox().GetSize(Rml::BoxArea::Border)};
	auto const objects{drawn.values.current().layer_objects};
	auto const now{glfwGetTime()};
	if (moved || false == drawn.image.has_value() || size != drawn.drawn_size || objects != drawn.drawn_objects || now - drawn.drawn_at >= 1.0 / 30.0)
	{
		// the lines are drawn wide while there are few of them seen, closer than about 50 km across
		drawn.image = m_map->draw(glm::vec2{size.x, size.y}, map_style(objects, m_map->pixels_per_metre() >= 0.02f));
		drawn.drawn_at = now;
		drawn.drawn_size = size;
		drawn.drawn_objects = objects;
		// what's placed over the map goes with the image
		auto values{drawn.values.current()};
		fill_trains(values, m_map->trains());
		fill_vehicles(values, *m_map);
		fill_scale(values, *m_map);
		fill_signal(values, drawn.signal);
		fill_hover(values, *m_map, drawn.input);
		fill_panel(values, drawn.panel);
		drawn.values.update(values, drawn.document);
		// not bound, forgotten when the train is gone
		drawn.values.current().selected_key = values.selected_key;
	}
	if (false == drawn.image.has_value())
		return;
	// under the documents of RmlUi, which are drawn into the same list after it
	auto *viewport{ImGui::GetMainViewport()};
	ImVec2 const topleft{viewport->Pos.x + offset.x, viewport->Pos.y + offset.y};
	ImGui::GetBackgroundDrawList(viewport)->AddImage(drawn.image->texture, topleft, ImVec2{topleft.x + size.x, topleft.y + size.y}, drawn.image->uv0, drawn.image->uv1);
	m_map->render_popups();
}

bool map_screen::take_closed()
{
	return m_state != nullptr && std::exchange(m_state->values.current().closed, false);
}

#else

struct map_screen::state
{
};

map_screen::map_screen(ui::map_panel &Map) : m_map(&Map) {}

map_screen::~map_screen() = default;

bool map_screen::open()
{
	return false;
}

bool map_screen::update(bool)
{
	return false;
}

void map_screen::draw()
{
	// built without RmlUi
}

bool map_screen::take_closed()
{
	return false;
}

#endif
