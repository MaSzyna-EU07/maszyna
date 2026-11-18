#include "application/uilayer.h"
#include "utilities/translation.h"
#include "input/command.h"

namespace ui
{
// the time, the date and the weather of the simulation: the hour and the minute, the day and the month, a kind of weather of the
// climate of the place drawn for the month, or the wind, the temperatures of the day, the moisture of the air, the clouds, what
// falls, the visibility and the snow lying set by hand
class time_panel : public ui_panel
{
	command_relay m_relay;

	int minute{0}; // of the day
	int day{1};
	int month{0}; // 0..11
	// 0: by hand, 1: random as often as each comes, then the kinds of climate::preset
	int kind{0};
	float wind_direction{270.f};
	float wind_speed{0.f};
	float wind_gust{0.f};
	float temperature_min{15.f};
	float temperature_max{15.f};
	float dew_point{10.f};
	float cover{0.f}; // of the sky, 0..1
	int precipitation{0}; // as precipitation_type
	float precipitation_intensity{0.f};
	float visibility{10000.f};
	float snow_depth{0.f}; // cm
	bool weather_edited{false};
	bool snow_edited{false};

	void render_time();
	void render_weather();
	void render_now() const;
	// the weather of the controls applied; with a kind of weather, one drawn of the climate for the month first
	void apply_weather();
	// the controls set to the weather as it is
	void read_weather();

  public:
	time_panel();

	void render_contents() override;
	void open();
};
} // namespace ui
