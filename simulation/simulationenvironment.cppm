module;

export module eu07.simulation.simulationenvironment;
import eu07.audio.sound;
import eu07.environment.moon;
import eu07.environment.sky;
import eu07.environment.skydome;
import eu07.environment.stars;
import eu07.environment.sun;
import eu07.rendering.precipitation;
import eu07.glm;
export {
/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/




// wrapper for environment elements -- sky, sun, stars, clouds etc
class world_environment {
public:
// methods
    void init();
    void update();
    void update_precipitation();
    void update_moon();
    void time( int const Hour = -1, int const Minute = -1, int const Second = -1 );
    // switches between static and dynamic daylight calculation
    void on_daylight_change();
    // calculates current season of the year based on set simulation date
	void compute_season( int const Yearday );
    // calculates current weather
	void compute_weather();
    // data access
    inline auto const &
        sun() const {
            return m_sun; }
    inline auto const &
        moon() const {
            return m_moon; }
    inline auto const &
        light_intensity() const {
            return m_lightintensity; }
    inline auto const &
        skydome() const {
            return m_skydome; }
    inline auto &
        skydome() {
            return m_skydome; }
    inline auto const &
        precipitation() const {
            return m_precipitation; }
    inline auto const &
        wind() const {
            return m_wind.vector; }
    inline auto const &
        wind_azimuth() const {
            return m_wind.azimuth; }

private:
// types
    struct basic_wind {
        // internal state data
        float azimuth;
        float azimuth_change;
        float velocity;
        float velocity_change;
        float change_time;
        // output
        glm::vec3 vector;
    };
// methods
    void update_wind();
// members
public:
// The renderers reach straight into these; they used to be friends, but a friend
// declaration would make this module the first declarer of classes defined in the
// renderer modules, which C++20 modules reject. Same treatment as in sky/stars.
    CSkyDome m_skydome;
    cStars m_stars;
    cSun m_sun;
    cMoon m_moon;
    TSky m_clouds;
    basic_precipitation m_precipitation;
private:
    float m_lightintensity { 1.f };
    sound_source m_rainsound { sound_placement::external, -1 };
    basic_wind m_wind;
};

namespace simulation {

extern world_environment Environment;

} // simulation

//---------------------------------------------------------------------------

}  // export
