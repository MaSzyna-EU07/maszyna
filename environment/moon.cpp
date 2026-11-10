#include "stdafx.h"
#include "environment/moon.h"
#include "utilities/Globals.h"
#include "world/mtable.h"
#include "utilities/utilities.h"
#include "simulation/simulationtime.h"

//////////////////////////////////////////////////////////////////////////////////////////
// cMoon -- class responsible for dynamic calculation of position, phase and intensity of the Moon

namespace {

// direction to a body on the sky of the observer (x to the west, y up, z to the north) from its hour angle and declination,
// at the given (refracted) elevation. degrees
glm::vec3
horizontal_direction( double const Hourangle, double const Declination, double const Latitude, double const Elevation ) {

    auto const hourangle { glm::radians( Hourangle ) };
    auto const declination { glm::radians( Declination ) };
    auto const latitude { glm::radians( Latitude ) };
    auto const elevation { glm::radians( Elevation ) };
    auto const west { std::cos( declination ) * std::sin( hourangle ) };
    auto const north { std::cos( latitude ) * std::sin( declination ) - std::sin( latitude ) * std::cos( declination ) * std::cos( hourangle ) };
    auto const horizontal { std::hypot( west, north ) };
    if( horizontal < 1e-9 ) {
        return { 0.f, ( Elevation >= 0.0 ? 1.f : -1.f ), 0.f };
    }
    auto const scale { std::cos( elevation ) / horizontal };
    return glm::normalize( glm::vec3( static_cast<float>( west * scale ), static_cast<float>( std::sin( elevation ) ), static_cast<float>( north * scale ) ) );
}

} // namespace

cMoon::cMoon() {

	setLocation( 19.00f, 52.00f );					// default location roughly in centre of Poland
	m_observer.press = 1013.0;						// surface pressure, millibars
	m_observer.temp = 15.0;							// ambient dry-bulb temperature, degrees C
}

cMoon::~cMoon() = default;

void
cMoon::init() {

    m_observer.timezone = -1.0 * simulation::Time.zone_bias();
    move();
    phase();
}

void
cMoon::update( [[maybe_unused]] bool const Includephase ) {

    m_observer.temp = Global.AirTemperature;

    move();
    // the phase follows the positions of the moon and the sun, worked out in move(); the moon is drawn lit from the side
    // of the sun, so it changes smoothly and is kept up to date on every update
    phase();
    m_position = horizontal_direction( m_body.hrang, m_body.declin, m_observer.latitude, m_body.elevref );
}

void
cMoon::render() {

    //m7t
}

glm::vec3
cMoon::getDirection() const {

    return m_position;
}

float
cMoon::getAngle() const {
    
    return (float)m_body.elevref;
}
	
float cMoon::getIntensity() {

	irradiance();
    // NOTE: we don't have irradiance model for the moon so we cheat here
    // calculating intensity of the sun instead, and returning 15% of the value,
    // which roughly matches how much sunlight is reflected by the moon
    // We alter the intensity further based on current phase of the moon
	return static_cast<float>( m_body.etr / 1399.0 * getBrightness() * 0.15 ); // arbitrary scaling factor taken from etrn value
}

// brightness of the moon relative to the full moon. it falls with the phase much faster than the lit part of the disc
// shrinks, the quarter moon gives about a tenth of the light of the full one (phase law of the lunar magnitude, Allen 1976)
float
cMoon::getBrightness() const {

    auto const angle { std::abs( m_phaseangle ) };
    return std::pow( 10.f, -0.4f * ( 0.026f * angle + 4.0e-9f * angle * angle * angle * angle ) );
}

// lit fraction of the disc, 0 at the new moon, 1 at the full moon
float
cMoon::getIllumination() const {

    return 0.5f * ( 1.f + std::cos( glm::radians( m_phaseangle ) ) );
}

// angular radius of the disc seen from the earth, radians
float
cMoon::getAngularRadius() const {

    // radius of the moon in earth radii, over the distance in earth radii
    return static_cast<float>( std::asin( 0.2725 / std::max( m_body.distance, 50.0 ) ) );
}

void cMoon::setLocation( float const Longitude, float const Latitude ) {

	// convert fraction from geographical base of 6o minutes
	m_observer.longitude = (int)Longitude + (Longitude - static_cast<float>((int)Longitude)) * 100.0 / 60.0;
	m_observer.latitude = (int)Latitude + (Latitude - static_cast<float>((int)Latitude)) * 100.0 / 60.0 ;
}

// sets current time, overriding one acquired from the system clock
void cMoon::setTime( int const Hour, int const Minute, int const Second ) {

    m_observer.hour = std::clamp( Hour, -1, 23 );
    m_observer.minute = std::clamp( Minute, -1, 59 );
    m_observer.second = std::clamp( Second, -1, 59 );
}

void cMoon::setTemperature( float const Temperature ) {
    
    m_observer.temp = Temperature;
}

void cMoon::setPressure( float const Pressure ) {
    
    m_observer.press = Pressure;
}

void cMoon::move() {

	static double radtodeg = 57.295779513; // converts from radians to degrees
	static double degtorad = 0.0174532925; // converts from degrees to radians

    SYSTEMTIME localtime = simulation::Time.data(); // time for the calculation

    if( m_observer.hour >= 0 ) { localtime.wHour = static_cast<uint16_t>(m_observer.hour); }
    if( m_observer.minute >= 0 ) { localtime.wMinute = static_cast<uint16_t>(m_observer.minute); }
    if( m_observer.second >= 0 ) { localtime.wSecond = static_cast<uint16_t>(m_observer.second); }

    double localut =
        localtime.wHour
        + localtime.wMinute / 60.0 // too low resolution, noticeable skips
        + localtime.wSecond / 3600.0; // good enough in normal circumstances
/*
        + localtime.wMilliseconds / 3600000.0; // for really smooth movement
*/
    double daynumber
        = 367 * localtime.wYear
        - 7 * ( localtime.wYear + ( localtime.wMonth + 9 ) / 12 ) / 4
        + 275 * localtime.wMonth / 9
        + localtime.wDay
        - 730530
        + ( localut - m_observer.timezone ) / 24.0; // the orbital elements count days of universal time

    // Universal Coordinated (Greenwich standard) time
    m_observer.utime = localut - m_observer.timezone;

    // obliquity of the ecliptic
    m_body.oblecl = clamp_circular( 23.4393 - 3.563e-7 * daynumber );
    // moon parameters
    double longascnode = clamp_circular( 125.1228 - 0.0529538083  * daynumber ); // N, degrees
    double const inclination = 5.1454; // i, degrees
    double const mndistance = 60.2666; // a, in earth radii
    // argument of perigee
    double const perigeearg = clamp_circular( 318.0634 + 0.1643573223  * daynumber ); // w, degrees
    // mean anomaly
    m_body.mnanom = clamp_circular( 115.3654 + 13.0649929509 * daynumber ); // M, degrees
    // eccentricity
    double constexpr e = 0.054900;
    // eccentric anomaly
    double E0 = m_body.mnanom + radtodeg * e * std::sin( degtorad * m_body.mnanom ) * ( 1.0 + e * std::cos( degtorad * m_body.mnanom ) );
    double E1 = E0 - ( E0 - radtodeg * e * std::sin( degtorad * E0 ) - m_body.mnanom ) / ( 1.0 - e * std::cos( degtorad * E0 ) );
    while( std::abs( E0 - E1 ) > 0.005 ) { // arbitrary precision tolerance threshold
        E0 = E1;
        E1 = E0 - ( E0 - radtodeg * e * std::sin( degtorad * E0 ) - m_body.mnanom ) / ( 1.0 - e * std::cos( degtorad * E0 ) );
    }
    double const E = E1;
    // lunar orbit plane rectangular coordinates
    double const xv = mndistance * ( std::cos( degtorad * E ) - e );
    double const yv = mndistance * std::sin( degtorad * E ) * std::sqrt( 1.0 - sq(e) );
    // distance
	m_body.distance = std::sqrt( xv*xv + yv*yv ); // r
    // true anomaly
    m_body.tranom = clamp_circular( radtodeg * std::atan2( yv, xv ) ); // v
    // ecliptic rectangular coordinates
    double const vpluswinrad = degtorad * ( m_body.tranom + perigeearg );
    double const xeclip = m_body.distance * ( std::cos( degtorad * longascnode ) * std::cos( vpluswinrad ) - std::sin( degtorad * longascnode ) * std::sin( vpluswinrad ) * std::cos( degtorad * inclination ) );
    double const yeclip = m_body.distance * ( std::sin( degtorad * longascnode ) * std::cos( vpluswinrad ) + std::cos( degtorad * longascnode ) * std::sin( vpluswinrad ) * std::cos( degtorad * inclination ) );
    double const zeclip = m_body.distance * std::sin( vpluswinrad ) * std::sin( degtorad * inclination );
    // ecliptic coordinates
    m_body.eclong = clamp_circular( radtodeg * std::atan2( yeclip, xeclip ) );
    // distance
    m_body.distance = std::sqrt( xeclip*xeclip + yeclip*yeclip + zeclip*zeclip );
    // perturbations
    // NOTE: perturbation calculation can be safely disabled if we don't mind error of 1-2 degrees
    // Sun's  mean anomaly:          Ms     (already computed)
    double const sunmnanom = clamp_circular( 356.0470 + 0.9856002585 * daynumber ); // M
    // Sun's  mean longitude:        Ls     (already computed)
    double const sunphlong = clamp_circular( 282.9404 + 4.70935e-5 * daynumber );
    double const sunmnlong = clamp_circular( sunphlong + sunmnanom ); // L = w + M
    // Moon's mean anomaly:          Mm     (already computed)
    // Moon's mean longitude:        Lm  =  N + w + M (for the Moon)
    m_body.mnlong = clamp_circular( longascnode + perigeearg + m_body.mnanom );
    // Moon's mean elongation:       D   =  Lm - Ls
    double const mnelong = clamp_circular( m_body.mnlong - sunmnlong );
    // longitude perturbations
    double const pertevection = -1.274 * std::sin( degtorad * ( m_body.mnanom - 2.0 * mnelong ) ); // Evection
    double const pertvariation = +0.658 * std::sin( degtorad * ( 2.0 * mnelong ) ); // Variation
    double const pertyearlyeqt = -0.186 * std::sin( degtorad * sunmnanom ); // Yearly equation
    // smaller longitude perturbations, all of them together up to about a quarter of a degree
    // Moon's argument of latitude:  F   =  Lm - N
    double const latarg = clamp_circular( m_body.mnlong - longascnode );
    auto const sindeg = [&]( double const Angle ) { return std::sin( degtorad * Angle ); };
    double const pertsmall =
        - 0.059 * sindeg( 2.0 * m_body.mnanom - 2.0 * mnelong )
        - 0.057 * sindeg( m_body.mnanom - 2.0 * mnelong + sunmnanom )
        + 0.053 * sindeg( m_body.mnanom + 2.0 * mnelong )
        + 0.046 * sindeg( 2.0 * mnelong - sunmnanom )
        + 0.041 * sindeg( m_body.mnanom - sunmnanom )
        - 0.035 * sindeg( mnelong ) // Parallactic equation
        - 0.031 * sindeg( m_body.mnanom + sunmnanom )
        - 0.015 * sindeg( 2.0 * latarg - 2.0 * mnelong )
        + 0.011 * sindeg( m_body.mnanom - 4.0 * mnelong );
    // latitude perturbations
    double const pertlatitude =
        - 0.173 * sindeg( latarg - 2.0 * mnelong )
        - 0.055 * sindeg( m_body.mnanom - latarg - 2.0 * mnelong )
        - 0.046 * sindeg( m_body.mnanom + latarg - 2.0 * mnelong )
        + 0.033 * sindeg( latarg + 2.0 * mnelong )
        + 0.017 * sindeg( 2.0 * m_body.mnanom + latarg );
    // distance perturbations, earth radii
    m_body.distance +=
        - 0.58 * std::cos( degtorad * ( m_body.mnanom - 2.0 * mnelong ) )
        - 0.46 * std::cos( degtorad * ( 2.0 * mnelong ) );

    m_body.eclong = clamp_circular( m_body.eclong + pertevection + pertvariation + pertyearlyeqt + pertsmall );
    // ecliptic latitude: the orbit is tilted by about 5 degrees, which moves the moon as much off the ecliptic
    m_body.eclat = radtodeg * std::atan2( zeclip, std::sqrt( xeclip * xeclip + yeclip * yeclip ) ) + pertlatitude;

    // equatorial coordinates
    double const xequat = std::cos( degtorad * m_body.eclong ) * std::cos( degtorad * m_body.eclat );
    double const yeclon = std::sin( degtorad * m_body.eclong ) * std::cos( degtorad * m_body.eclat );
    double const zeclat = std::sin( degtorad * m_body.eclat );
    double const yequat = yeclon * std::cos( degtorad * m_body.oblecl ) - zeclat * std::sin( degtorad * m_body.oblecl );
    double const zequat = yeclon * std::sin( degtorad * m_body.oblecl ) + zeclat * std::cos( degtorad * m_body.oblecl );
    // declination
    m_body.declin = radtodeg * std::atan2( zequat, std::sqrt( xequat * xequat + yequat * yequat ) );
    // right ascension
    m_body.rascen = clamp_circular( radtodeg * std::atan2( yequat, xequat ) );

    // phase: the angle between the sun and the moon seen from the earth (elongation), from the true longitude of the sun
    double const sunlong = sunmnlong + 1.915 * sindeg( sunmnanom ) + 0.020 * sindeg( 2.0 * sunmnanom );
    m_elongation = clamp_circular( m_body.eclong - sunlong );
    double const coselongation = std::cos( degtorad * m_body.eclat ) * std::cos( degtorad * m_elongation );
    // the phase angle (sun - moon - earth) is the supplement of the elongation, closely enough at the distance of the sun
    m_phaseangle = static_cast<float>( 180.0 - radtodeg * std::acos( std::clamp( coselongation, -1.0, 1.0 ) ) );

	// Greenwich mean sidereal time
	// (Schlyter: GMST0 = sun's mean longitude + 180 degrees, for the day number counted from 2000 Jan 0.0 UT)
	m_observer.gmst = 6.5991604 + 0.0657098242 * daynumber + m_observer.utime;

	m_observer.gmst -= 24.0 * (int)( m_observer.gmst / 24.0 );
	if( m_observer.gmst < 0.0 ) m_observer.gmst += 24.0;

	// local mean sidereal time
	m_observer.lmst = m_observer.gmst * 15.0 + m_observer.longitude;

	m_observer.lmst -= 360.0 * (int)( m_observer.lmst / 360.0 );
	if( m_observer.lmst < 0.0 ) m_observer.lmst += 360.0;

	// hour angle
	m_body.hrang = m_observer.lmst - m_body.rascen;

		 if( m_body.hrang < -180.0 )	m_body.hrang += 360.0;	// (force it between -180 and 180 degrees)
    else if( m_body.hrang >  180.0 )	m_body.hrang -= 360.0;

	double cz;	// cosine of the solar zenith angle

	double tdatcd = std::cos( degtorad * m_body.declin );
	double tdatch = std::cos( degtorad * m_body.hrang );
	double tdatcl = std::cos( degtorad * m_observer.latitude );
	double tdatsd = std::sin( degtorad * m_body.declin );
	double tdatsl = std::sin( degtorad * m_observer.latitude );

	cz = tdatsd * tdatsl + tdatcd * tdatcl * tdatch;

    // (watch out for the roundoff errors)
	if( fabs (cz) > 1.0 ) {	cz >= 0.0 ? cz = 1.0 : cz = -1.0; }

	m_body.zenetr = std::acos( cz ) * radtodeg;
	m_body.elevetr = 90.0 - m_body.zenetr;
	// topocentric parallax: the moon is close enough to be seen about a degree lower than from the centre of the earth
	m_body.elevetr -= radtodeg * std::asin( 1.0 / m_body.distance ) * std::cos( degtorad * m_body.elevetr );
	refract();
}

void cMoon::refract() {

	static double degtorad = 0.0174532925;					// converts from degrees to radians

	double prestemp;	// temporary pressure/temperature correction
	double refcor;		// temporary refraction correction
	double tanelev;		// tangent of the solar elevation angle

	// if the sun is near zenith, the algorithm bombs; refraction near 0.
	if( m_body.elevetr > 85.0 )
		refcor = 0.0;
	else {

		tanelev = tan( degtorad * m_body.elevetr );
		if( m_body.elevetr >= 5.0 )
			refcor	= 58.1 / tanelev
					- 0.07 / pow( tanelev, 3 )
					+ 0.000086 / pow( tanelev, 5 );
		else if( m_body.elevetr >= -0.575 )
			refcor  = 1735.0
					+ m_body.elevetr * ( -518.2 + m_body.elevetr *
					  ( 103.4 + m_body.elevetr * ( -12.79 + m_body.elevetr * 0.711 ) ) );
		else
			refcor  = -20.774 / tanelev;

		prestemp = m_observer.press * 283.0 / ( 1013.0 * ( 273.0 + m_observer.temp ) );
		refcor *= prestemp / 3600.0;
	}

	// refracted solar elevation angle
	m_body.elevref = m_body.elevetr + refcor;

	// refracted solar zenith angle
	m_body.zenref  = 90.0 - m_body.elevref;
}

void cMoon::irradiance() {

	static double degtorad = 0.0174532925;					// converts from degrees to radians

	m_body.dayang = ( simulation::Time.year_day() - 1 ) * 360.0 / 365.0;
	double sd = sin( degtorad * m_body.dayang );				// sine of the day angle
	double cd = cos( degtorad * m_body.dayang );				// cosine of the day angle or delination
	m_body.erv = 1.000110 + 0.034221*cd + 0.001280*sd;
	double d2 = 2.0 * m_body.dayang;
	double c2 = cos( degtorad * d2 );
	double s2 = sin( degtorad * d2 );
	m_body.erv += 0.000719*c2 + 0.000077*s2;

	double solcon = 1367.0;									// Solar constant, 1367 W/sq m

	m_body.coszen  = cos( degtorad * m_body.zenref );
	if( m_body.coszen > 0.0 ) {
		m_body.etrn = solcon * m_body.erv;
		m_body.etr  = m_body.etrn * m_body.coszen;
	}
	else {
		m_body.etrn = 0.0;
		m_body.etr  = 0.0;
	}
}

void
cMoon::phase() {
    SYSTEMTIME lt = simulation::Time.data();
	if (lt.wMonth == 5 && lt.wDay == 4) //May the forth be with you!
		m_phase = 50;
	else {
		// moon's age in days from new moon, from the elongation worked out in move()
		m_phase = normalize( static_cast<float>( m_elongation / 360.0 ) ) * 29.53f;
	}
}

// normalize values to range 0...1
float
cMoon::normalize( const float Value ) const {

    auto value = static_cast<float>(Value - floor( Value ));
    if( value < 0.f ) { ++value; }

    return value;
}
