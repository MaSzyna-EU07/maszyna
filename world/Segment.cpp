/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/Segment.h"

#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"
#include "world/Track.h"
#include "rendering/renderer.h"

void
segment_data::deserialize( cParser &Input, glm::dvec3 const &Offset ) {

    points[ segment_data::point::start ] = LoadPoint( Input ) + Offset;
    Input.getTokens();
    Input >> rolls[ 0 ];
    points[ segment_data::point::control1 ] = LoadPoint( Input );
    points[ segment_data::point::control2 ] = LoadPoint( Input );
    points[ segment_data::point::end ] = LoadPoint( Input ) + Offset;
    Input.getTokens( 2 );
    Input
        >> rolls[ 1 ]
        >> radius;
}


void
segment_data::translate( glm::dvec3 const &Offset ) {

    points[ segment_data::point::start ] += Offset;
    points[ segment_data::point::end ] += Offset;
    for( auto &piece : pieces ) {
        piece.translate( Offset );
    }
}

TSegment::TSegment(TTrack *owner) :
                   pOwner( owner )
{}

bool TSegment::Init(glm::dvec3 NewPoint1, glm::dvec3 NewPoint2, double fNewStep, double fNewRoll1, double fNewRoll2)
{ // wersja dla prostego - wyliczanie punktów kontrolnych
	glm::dvec3 dir;

    // NOTE: we're enforcing division also for straight track, to ensure dense enough mesh for per-vertex lighting
/*
    if (fNewRoll1 == fNewRoll2)
    { // faktyczny prosty
        dir = Normalize(NewPoint2 - NewPoint1); // wektor kierunku o długości 1
        return TSegment::Init(NewPoint1, dir, -dir, NewPoint2, fNewStep, fNewRoll1, fNewRoll2,
                              false);
    }
    else
*/
    { // prosty ze zmienną przechyłką musi być segmentowany jak krzywe
        dir = (NewPoint2 - NewPoint1) / 3.0; // punkty kontrolne prostego są w 1/3 długości
        return TSegment::Init(
            NewPoint1, NewPoint1 + dir,
            NewPoint2 - dir, NewPoint2,
            fNewStep, fNewRoll1, fNewRoll2, true);
    }
};

bool TSegment::Init(glm::dvec3 &NewPoint1, glm::dvec3 NewCPointOut, glm::dvec3 NewCPointIn, glm::dvec3 &NewPoint2, double fNewStep, double fNewRoll1, double fNewRoll2, bool bIsCurve)
{ // wersja uniwersalna (dla krzywej i prostego)
    m_pieces.clear();
    m_starts.clear();
    Point1 = NewPoint1;
    CPointOut = NewCPointOut;
    CPointIn = NewCPointIn;
    Point2 = NewPoint2;
    // poprawienie przechyłki
    fRoll1 = static_cast<float>(glm::radians(fNewRoll1)); // Ra: przeliczone jest bardziej przydatne do obliczeń
    fRoll2 = static_cast<float>(glm::radians(fNewRoll2));
    m_setting.designrolls = { fRoll1, fRoll2 };
    m_setting.joints = {};
    bCurve = bIsCurve;
    if (Global.bRollFix)
    { // Ra: poprawianie przechyłki
        // Przechyłka powinna być na środku wewnętrznej szyny, a standardowo jest w osi
        // toru. Dlatego trzeba podnieść tor oraz odpowiednio podwyższyć podsypkę.
        // Nie wykonywać tej funkcji, jeśli podwyższenie zostało uwzględnione w edytorze.
        // Problematyczne mogą byc rozjazdy na przechyłce - lepiej je modelować w edytorze.
        // Na razie wszystkie scenerie powinny być poprawiane.
        // Jedynie problem będzie z podwójną rampą przechyłkową, która w środku będzie
        // mieć moment wypoziomowania, ale musi on być również podniesiony.
        if (fRoll1 != 0.0)
        { // tylko jeśli jest przechyłka
            double w1 = std::abs(std::sin(fRoll1) * 0.75); // 0.5*w2+0.0325; //0.75m dla 1.435
            Point1.y += w1; // modyfikacja musi być przed policzeniem dalszych parametrów
            if (bCurve)
                CPointOut.y += w1; // prosty ma wektory jednostkowe
            pOwner->MovedUp1(static_cast<float>(w1)); // zwrócić trzeba informację o podwyższeniu podsypki
        }
        if (fRoll2 != 0.f)
        {
            double w2 = std::abs(std::sin(fRoll2) * 0.75); // 0.5*w2+0.0325; //0.75m dla 1.435
            Point2.y += w2; // modyfikacja musi być przed policzeniem dalszych parametrów
            if (bCurve)
                CPointIn.y += w2; // prosty ma wektory jednostkowe
            // zwrócić trzeba informację o podwyższeniu podsypki
        }
    }
    // kąt w planie, żeby nie liczyć wielokrotnie
    // Ra: ten kąt jeszcze do przemyślenia jest
    fDirection = -std::atan2(Point2.x - Point1.x, Point2.z - Point1.z);
    if (bCurve)
    { // przeliczenie współczynników wielomianu, będzie mniej mnożeń i można policzyć pochodne
        vC = 3.0 * (CPointOut - Point1); // t^1
        vB = 3.0 * (CPointIn - CPointOut) - vC; // t^2
        vA = Point2 - Point1 - vC - vB; // t^3
        fLength = ComputeLength();
    }
    else {
		fLength = glm::length(Point1 - Point2);
    }
    if( false == std::isfinite( fLength ) ) {

        ErrorLog( "Bad track: spline \"" + pOwner->name() + "\" with no finite length (location: " + to_string( glm::dvec3{ Point1 } ) + ")" );
        fLength = 0.01;
    }
    if (fLength <= 0) {

        ErrorLog( "Bad track: zero length spline \"" + pOwner->name() + "\" (location: " + to_string( glm::dvec3{ Point1 } ) + ")" );
        fLength = 0.01; // crude workaround TODO: fix this properly
    }

    fStoop = std::atan2(Point2.y - Point1.y, fLength); // pochylenie toru prostego, żeby nie liczyć wielokrotnie

    fStep = ( fNewStep > 0.0 && std::isfinite( fNewStep ) ) ? fNewStep : fLength;
    if( m_setting.uneven && pOwner->eType != tt_Switch ) {
        // dense enough for the waves; a switch keeps its division, the blades depend on it
        fStep = std::min( fStep, 2.0 );
    }
    // NOTE: optionally replace this part with the commented version, after solving geometry issues with double switches
    if( pOwner->eType == tt_Switch
     && fStep * (3.0 * Global.SplineFidelity) > fLength ) {
        // NOTE: a workaround for too short switches (less than 3 segments) messing up animation/generation of blades
        fStep = fLength / ( 3.0 * Global.SplineFidelity );
    }
//    iSegCount = static_cast<int>( std::ceil( fLength / fStep ) ); // potrzebne do VBO
    iSegCount = pOwner->eType == tt_Switch ? static_cast<int>(6 * Global.SplineFidelity) : static_cast<int>(std::clamp(std::ceil(fLength / fStep), 1.0, 100000.0)); // potrzebne do VBO

    fStep = fLength / iSegCount; // update step to equalize size of individual pieces

	InitTFromSInterpolateData();
    return true;
}

bool TSegment::InitChain(std::vector<std::shared_ptr<TSegment>> Pieces)
{
    if( Pieces.empty() )
        return false;
    m_pieces = std::move( Pieces );
    m_starts.clear();
    fLength = 0.0;
    iSegCount = 0;
    for( auto const &piece : m_pieces ) {
        m_starts.push_back( fLength );
        fLength += piece->fLength;
        iSegCount += piece->iSegCount;
    }
    auto const &first { *m_pieces.front() };
    auto const &last { *m_pieces.back() };
    Point1 = first.Point1;
    CPointOut = first.CPointOut;
    CPointIn = last.CPointIn;
    Point2 = last.Point2;
    fRoll1 = first.fRoll1;
    fRoll2 = last.fRoll2;
    bCurve = true;
    fDirection = -std::atan2(Point2.x - Point1.x, Point2.z - Point1.z);
    fStoop = std::atan2(Point2.y - Point1.y, fLength);
    fStep = fLength / iSegCount;
    fTsBuffer.resize( iSegCount + 1 );
    for( int i = 0; i <= iSegCount; ++i ) {
        fTsBuffer[ i ] = static_cast<double>( i ) / iSegCount;
    }
    fTsSlope.assign( iSegCount + 1, fStep / fLength );
    return true;
}

std::size_t TSegment::piece_at(double const s) const
{
    auto const next { std::ranges::upper_bound( m_starts, s ) };
    return next == std::begin( m_starts ) ? 0 : static_cast<std::size_t>( std::distance( std::begin( m_starts ), next ) - 1 );
}

void TSegment::InitTFromSInterpolateData()
{
    // pre-compute the curve parameter t at equal distance intervals
    fTsBuffer.resize( iSegCount + 1 );
    fTsBuffer[ 0 ] = 0.0;
    for( int i = 1; i < iSegCount; ++i ) {
        fTsBuffer[ i ] = GetTFromS( i * fStep );
    }
    fTsBuffer[ iSegCount ] = 1.0;

    // pre-compute Hermite slopes dt/du = fStep / |B'(t)| for FastGetTFromS
    // (dt/ds = 1/|B'(t)|, scaled by fStep to the interval coordinate u)
    fTsSlope.resize( iSegCount + 1 );
    for( int i = 0; i <= iSegCount; ++i ) {
        // compute ds/dt derivative = |B'(t)|
        double const speed = glm::length( GetFirstDerivative( fTsBuffer[ i ] ) );
        // compute dt/du derivative = fStep / (ds/dt)
        fTsSlope[ i ] = speed > 0.0 ? fStep / speed : 0.0;
    }
    // clamp slopes to keep t(s) monotone (PCHIP / Fritsch-Carlson condition)
    for( int i = 0; i <= iSegCount; ++i ) {
        double limit = std::numeric_limits<double>::max();
        if( i > 0 )         { limit = std::min( limit, 3.0 * ( fTsBuffer[ i ] - fTsBuffer[ i - 1 ] ) ); }
        if( i < iSegCount ) { limit = std::min( limit, 3.0 * ( fTsBuffer[ i + 1 ] - fTsBuffer[ i ] ) ); }
        fTsSlope[ i ] = std::min( fTsSlope[ i ], limit );
    }
}

glm::dvec3 TSegment::GetFirstDerivative(double const fTime) const
{
	// derivative is 3*A*t^2 + 2*B*t + C
	return fTime * ( fTime * 3.0 * vA + vB + vB ) + vC;
}

double TSegment::RombergIntegral(double const fA, double const fB) const
{
    double fH = fB - fA;

    const int ms_iOrder = 5;

    double ms_apfRom[2][ms_iOrder];

    ms_apfRom[0][0] =
        0.5 * fH * (glm::length(GetFirstDerivative(fA)) + glm::length(GetFirstDerivative(fB)));
    for (int i0 = 2, iP0 = 1; i0 <= ms_iOrder; i0++, iP0 *= 2, fH *= 0.5)
    {
        // approximations via the trapezoid rule
        double fSum = 0.0;
        int i1;
        for (i1 = 1; i1 <= iP0; i1++)
            fSum += glm::length(GetFirstDerivative(fA + fH * (i1 - 0.5)));

        // Richardson extrapolation
        ms_apfRom[1][0] = 0.5 * (ms_apfRom[0][0] + fH * fSum);
        for (int i2 = 1, iP2 = 4; i2 < i0; i2++, iP2 *= 4)
        {
            ms_apfRom[1][i2] = (iP2 * ms_apfRom[1][i2 - 1] - ms_apfRom[0][i2 - 1]) / (iP2 - 1);
        }

        for (i1 = 0; i1 < i0; i1++)
            ms_apfRom[0][i1] = ms_apfRom[1][i1];
    }

    return ms_apfRom[0][ms_iOrder - 1];
}

double TSegment::GetTFromS(double const s) const
{
    // initial guess for Newton's method
    double fTolerance = 0.001;
    double fRatio = s / fLength; // fLength is approx RombergIntegral(0, 1), good enough to start Newton
    double fTime = std::lerp( 0.0, 1.0, fRatio );

    int iteration = 0;
    double fDifference {}; // exposed for debug down the road
	do {
        fDifference = RombergIntegral(0, fTime) - s;
		if( std::abs( fDifference ) < fTolerance ) {
			return fTime;
		}
		fTime -= fDifference / glm::length(GetFirstDerivative(fTime));
		++iteration;
	}
	while( iteration < 10 ); // arbitrary limit

    // Newton's method failed.  If this happens, increase iterations or
    // tolerance or integration accuracy.
    // return -1; //Ra: tu nigdy nie dojdzie
    ErrorLog( "Bad track: shape estimation failed for spline \"" + pOwner->name() + "\" (location: " + to_string( glm::dvec3{ Point1 } ) + ")" );
	// MessageBox(0,"Too many iterations","GetTFromS",MB_OK);
	return fTime;
};

double TSegment::FastGetTFromS(double const s) const
{ // Faster version of GetTFromS: cubic Hermite interpolation using
	// the precomputed values in fTsBuffer and slopes in fTsSlope. It
	// avoids the expensive RombergIntegral+Newton per call. Hermite
	// interpolation is C1-continuous, so no sudden speed changes when
	// moving from one interval to the next.
	// NOTE: for s outside <0, fLength> the result is extrapolated
	// outside <0, 1>, which is the expected behavior (see e.g. usage
	// in GetDirection())
	if( false == m_pieces.empty() )
		return s / fLength;
	int const i = std::clamp(static_cast<int>(s / fStep), 0, iSegCount - 1); // interval index
	double const u = (s - i * fStep) / fStep; // position within interval (possibly outside)
	double const u2 = u * u;
	double const u3 = u2 * u;
	double const h01 = -2.0 * u3 + 3.0 * u2;
	double const t = ( ( 1.0 - h01 )         * fTsBuffer[ i ] +
					   ( u3 - 2.0 * u2 + u ) * fTsSlope [ i ] +
					   h01               * fTsBuffer[ i + 1 ] +
					   ( u3 - u2 )           * fTsSlope [ i + 1 ] );
	return t;
}

glm::dvec3 TSegment::RaInterpolate(double const t) const
{ // wyliczenie XYZ na krzywej Beziera z użyciem współczynników
    return t * (t * (t * vA + vB) + vC) + Point1; // 9 mnożeń, 9 dodawań
};

glm::dvec3 TSegment::RaInterpolate0(double const t) const
{ // wyliczenie XYZ na krzywej Beziera, na użytek liczenia długości nie jest dodawane Point1
    return t * (t * (t * vA + vB) + vC); // 9 mnożeń, 6 dodawań
};

double TSegment::ComputeLength() const // McZapkie-150503: dlugosc miedzy punktami krzywej
{ // obliczenie długości krzywej Beziera za pomocą interpolacji odcinkami
    // Ra: zamienić na liczenie rekurencyjne średniej z cięciwy i łamanej po kontrolnych
    // Ra: koniec rekurencji jeśli po podziale suma długości nie różni się więcej niż 0.5mm od
    // poprzedniej
    // Ra: ewentualnie rozpoznać łuk okręgu płaskiego i liczyć ze wzoru na długość łuku
    if( false == m_pieces.empty() )
        return fLength;
    double t;
    double l = 0;
	glm::dvec3 last{0, 0, 0}; // długość liczona po przesunięciu odcinka do początku układu
	glm::dvec3 tmp = Point2 - Point1;
    auto m = static_cast<int>(20.0 * glm::length(tmp)); // było zawsze do 10000, teraz jest liczone odcinkami po około 5cm
    for (int i = 1; i <= m; i++)
    {
        t = double(i) / double(m); // wyznaczenie parametru na krzywej z przedziału (0,1>
        // tmp=std::lerp(t,p1,cp1,cp2,p2);
        tmp = RaInterpolate0(t); // obliczenie punktu dla tego parametru
		t = glm::length(tmp - last); // obliczenie długości wektora
        l += t; // zwiększenie wyliczanej długości
        last = tmp;
    }
    return l;
}

// finds point on segment closest to specified point in 3d space. returns: point on segment as value in range 0-1
double
TSegment::find_nearest_point( glm::dvec3 const &Point ) const {

    if( false == bCurve || iSegCount == 1 ) {
        // for straight track just treat it as a single segment
        return
            nearest_segment_point(
                glm::dvec3{ FastGetPoint_0() },
                glm::dvec3{ FastGetPoint_1() },
                Point );
    }
    else {
        // for curves iterate through segment chunks, and find the one which gives us the least distance to the specified point
        double distance = std::numeric_limits<double>::max();
        double nearest { 0.0 };
        // NOTE: we're reusing already created segment chunks, which are created based on splinefidelity setting
        // this means depending on splinefidelity the results can be potentially slightly different
        for( int segmentidx = 0; segmentidx < iSegCount; ++segmentidx ) {

            auto const segmentpoint =
                std::clamp(
                    nearest_segment_point(
                        glm::dvec3{ FastGetPoint( fTsBuffer[ segmentidx ] ) },
                        glm::dvec3{ FastGetPoint( fTsBuffer[ segmentidx + 1 ] ) },
                        Point ) // point in range 0-1 on current segment
                    * ( fTsBuffer[ segmentidx + 1 ] - fTsBuffer[ segmentidx ] ) // segment length
                    + fTsBuffer[ segmentidx ], // segment offset
                    0.0, 1.0 ); // we clamp the range in case there's some floating point math inaccuracies

            auto const segmentdistance = glm::length2( Point - glm::dvec3{ FastGetPoint( segmentpoint ) } );
            if( segmentdistance < distance ) {

                nearest = segmentpoint;
                distance = segmentdistance;
            }
        }
        return nearest; // NOSONAR initialised above, the PR analysis keeps a stale report of the old declaration
    }
}

const double fDirectionOffset = 0.1; // długość wektora do wyliczenia kierunku

glm::dvec3 TSegment::GetDirection(double const fDistance) const
{ // takie toporne liczenie pochodnej dla podanego dystansu od Point1
    double t1 = FastGetTFromS(fDistance - fDirectionOffset);
    if (t1 <= 0.0)
        return CPointOut - Point1; // na zewnątrz jako prosta
    double t2 = FastGetTFromS(fDistance + fDirectionOffset);
    if (t2 >= 1.0)
        return Point1 - CPointIn; // na zewnątrz jako prosta
    return FastGetPoint(t2) - FastGetPoint(t1);
}

glm::dvec3 TSegment::FastGetDirection(double fDistance, double fOffset) const
{ // takie toporne liczenie pochodnej dla parametru 0.0÷1.0
    double t1 = fDistance - fOffset;
    if (t1 <= 0.0)
        return CPointOut - Point1; // wektor na początku jest stały
    double t2 = fDistance + fOffset;
    if (t2 >= 1.0)
        return Point2 - CPointIn; // wektor na końcu jest stały
    return FastGetPoint(t2) - FastGetPoint(t1);
}
/*
Math3D::vector3 TSegment::GetPoint(double const fDistance) const
{ // wyliczenie współrzędnych XYZ na torze w odległości (fDistance) od Point1
    if (bCurve)
    { // można by wprowadzić uproszczony wzór dla okręgów płaskich
        double t = FastGetTFromS(fDistance); // aproksymacja dystansu na krzywej Beziera
        // return std::lerp(t,Point1,CPointOut,CPointIn,Point2);
        return RaInterpolate(t);
    }
    else {
        // wyliczenie dla odcinka prostego jest prostsze
        return
            std::lerp(
                Point1, Point2,
                std::clamp(
                    fDistance / fLength,
                    0.0, 1.0 ) );
    }
};
*/

glm::dvec3 TSegment::stray( glm::dvec3 const &Point, track_uneven::amounts const &Amount, double const Distance ) const {
    auto const field { track_uneven::field( Point ) };
    glm::dvec3 result { field.x * Amount.side, field.y * Amount.up, field.z * Amount.side };
    result *= track_uneven::stray_per_mm;
    result.y -= pOwner->joint_dip( Distance );
    return result;
}

glm::dvec3 TSegment::uneven_bend( double const Distance ) const {
    if( false == m_setting.uneven ) {
        return glm::dvec3 { 0.0 };
    }
    // over about a bogie
    auto const reach { 1.5 };
    auto const strayed = [ this ]( double const At ) {
        // past the ends of the path straight on: the polynomial would run off on a short curved one
        auto const on { std::clamp( At, 0.0, fLength ) };
        auto const t { FastGetTFromS( on ) };
        auto point { FastGetPoint( t ) };
        if( auto const direction { FastGetDirection( t, 0.1 / fLength ) }; on != At && glm::length( direction ) > 0.0 ) {
            point += direction * ( ( At - on ) / glm::length( direction ) );
        }
        return stray( point, pOwner->uneven_at( At, m_setting.path ), At ); };
    return ( strayed( Distance + reach ) - 2.0 * strayed( Distance ) + strayed( Distance - reach ) ) / ( reach * reach );
}

void TSegment::settle_vertex( gfx::basic_vertex &Vertex, glm::dvec3 const &Origin, glm::vec3 const &Position, glm::vec3 const &Parallel, double const Distance ) const {
    if( false == m_setting.uneven ) {
        return;
    }
    // the rails settling apart turn about the middle of the track, what's outside them follows the rail; the foot of the ballast stays
    auto const gauge { std::max( std::abs( pOwner->fTrackWidth ), 0.1f ) };
    auto const across { std::clamp( glm::dot( Vertex.position - Position, Parallel ), -0.5f * gauge, 0.5f * gauge ) };
    auto const share { pOwner->uneven_bed_share( Position.y - Vertex.position.y ) };
    if( share <= 0.0 ) {
        return;
    }
    auto const amount { pOwner->uneven_at( Distance, m_setting.path ) };
    Vertex.position.y -= static_cast<float>( share * track_uneven::stray_per_mm * amount.cross * track_uneven::level( Origin + glm::dvec3 { Position }, glm::dvec3 { Parallel } ) * across / gauge );
    Vertex.position += glm::vec3 { share * stray( Origin + glm::dvec3 { Vertex.position }, amount, Distance ) };
}

void TSegment::RaPositionGet(double const fDistance, glm::dvec3 &position, glm::vec3 &rotation) const {
    if( false == m_pieces.empty() ) {
        auto const piece { piece_at( fDistance ) };
        m_pieces[ piece ]->RaPositionGet( fDistance - m_starts[ piece ], position, rotation );
    }
    else if (bCurve) {
        // można by wprowadzić uproszczony wzór dla okręgów płaskich
        auto const t = FastGetTFromS(fDistance); // aproksymacja dystansu na krzywej Beziera na parametr (t)
        position = FastGetPoint( t );
        // przechyłka w danym miejscu (zmienia się liniowo)
        rotation.x = static_cast<float>(std::lerp( fRoll1, fRoll2, t ));
        // pochodna
        auto const tangent = GetFirstDerivative( t );
        // pochylenie krzywej (w pionie)
        rotation.y = static_cast<float>(std::atan( tangent.y ));
        // kierunek krzywej w planie
        rotation.z = static_cast<float>(-std::atan2( tangent.x, tangent.z ));
    }
    else {
        // wyliczenie dla odcinka prostego jest prostsze
        auto const t = fDistance / fLength; // zerowych torów nie ma
        position = FastGetPoint( t );
        // przechyłka w danym miejscu (zmienia się liniowo)
        rotation.x = static_cast<float>(std::lerp( fRoll1, fRoll2, t ));
        rotation.y = static_cast<float>(fStoop); // pochylenie toru prostego
        rotation.z = static_cast<float>(fDirection); // kierunek toru w planie
    }
    if( false == m_setting.uneven ) {
        return;
    }
    // direction and slope stay as designed: the physics takes the curve radius and the grade from them
    auto const gauge { std::max( std::abs( pOwner->fTrackWidth ), 0.1f ) };
    glm::dvec3 const across { -std::cos( rotation.z ), 0.0, -std::sin( rotation.z ) };
    auto const amount { pOwner->uneven_at( fDistance, m_setting.path ) };
    rotation.x += static_cast<float>( track_uneven::stray_per_mm * (
        amount.up * track_uneven::roll( position, across, gauge )
      + amount.cross * track_uneven::level( position, across ) / gauge ) );
    position += track_uneven::across_only( stray( position, amount, fDistance ), across );
};

void TSegment::plan_frame( double const Distance, glm::dvec3 &Point, glm::dvec3 &Across ) const {
    if( fTsBuffer.empty() ) {
        // never made into a path: tt_Cross and the like
        Point = Point1;
        Across = glm::dvec3 { 0.0 };
        return;
    }
    auto const t { FastGetTFromS( std::clamp( Distance, 0.0, fLength ) ) };
    Point = FastGetPoint( t );
    Across = track_uneven::across_of( FastGetDirection( t, 0.1 / fLength ), FastGetPoint_1() - FastGetPoint_0() );
}

glm::dvec3 TSegment::FastGetPoint(double const t) const
{
    // return (bCurve?std::lerp(t,Point1,CPointOut,CPointIn,Point2):((1.0-t)*Point1+(t)*Point2));
    if( false == m_pieces.empty() ) {
        auto const distance { t * fLength };
        auto const index { piece_at( distance ) };
        auto const &piece { *m_pieces[ index ] };
        return piece.FastGetPoint( piece.FastGetTFromS( distance - m_starts[ index ] ) );
    }
    return true == bCurve || iSegCount != 1 ? RaInterpolate(t) : glm::mix(Point1, Point2, t);
}

bool TSegment::RenderLoft( gfx::vertex_array &Output, glm::dvec3 const &Origin, const gfx::vertex_array &ShapePoints, bool const Transition, double fTextureLength, double Texturescale, int iSkip, int iEnd, std::pair<float, float> fOffsetX, glm::vec3 **p, bool bRender)
{ // generowanie trójkątów dla odcinka trajektorii ruchu
    // standardowo tworzy triangle_strip dla prostego albo ich zestaw dla łuku
    // po modyfikacji - dla ujemnego (iNumShapePoints) w dodatkowych polach tabeli podany jest przekrój końcowy

    if( fTsBuffer.empty() )
        return false; // prowizoryczne zabezpieczenie przed wysypem - ustalić faktyczną przyczynę

    glm::vec3 pos1;
    glm::vec3 pos2;
    glm::vec3 dir;
    glm::vec3 parallel1;
    glm::vec3 parallel2;
    glm::vec3 pt;
    glm::vec3 norm;
    float s;
    float step;
    float fOffset;
    float tv1;
    float tv2;
    float t;
    float fEnd;
    auto const iNumShapePoints = Transition ? ShapePoints.size() / 2 : ShapePoints.size();
    auto const texturelength = static_cast<float>(fTextureLength * Texturescale);
    auto const texturescale = static_cast<float>(Texturescale);

    float m1;
    float jmm1;
    float m2;
    float jmm2; // pozycje względne na odcinku 0...1 (ale nie parametr Beziera)
    step = static_cast<float>(fStep);
    tv1 = 1.0; // Ra: to by można było wyliczać dla odcinka, wyglądało by lepiej
    s = static_cast<float>(fStep * iSkip); // iSkip - ile odcinków z początku pominąć
    int i = iSkip; // domyślnie 0
    t = static_cast<float>(fTsBuffer[ i ]); // tabela wattości t dla segmentów
    // BUG: length of spline can be 0, we should skip geometry generation for such cases
    fOffset = static_cast<float>(0.1 / fLength); // pierwsze 10cm
    pos1 = glm::dvec3{ FastGetPoint( t ) - Origin }; // wektor początku segmentu
    dir = glm::dvec3{ FastGetDirection( t, fOffset ) }; // wektor kierunku
    parallel1 = glm::vec3{ -dir.z, 0.f, dir.x }; // wektor poprzeczny
    if( glm::length2( parallel1 ) == 0.f ) {
        // temporary workaround for malformed situations with control points placed above endpoints
        parallel1 = glm::vec3{ glm::dvec3{ FastGetPoint_1() - FastGetPoint_0() } };
    }
    parallel1 = glm::normalize( parallel1 );
    if( iSkip == 0 && m_setting.joints[ 0 ] ) {
        pos1 = glm::vec3{ m_setting.joints[ 0 ]->first - Origin };
        parallel1 = glm::vec3{ m_setting.joints[ 0 ]->second };
    }
    if( iEnd == 0 )
        iEnd = iSegCount;
    fEnd = static_cast<float>(fLength * double( iEnd ) / double( iSegCount ));
/*
    m2 = s / fEnd;
*/
    m2 = static_cast<float>( i - iSkip ) / ( static_cast<float>(iEnd - iSkip) );

    jmm2 = 1.f - m2;

    while( i < iEnd ) {

        ++i; // kolejny punkt łamanej
        s += step; // końcowa pozycja segmentu [m]
        m1 = m2;
        jmm1 = jmm2; // stara pozycja
/*
        m2 = s / fEnd;
*/
        m2 = static_cast<float>( i - iSkip ) / ( static_cast<float>(iEnd - iSkip) );

        jmm2 = 1.f - m2; // nowa pozycja
        if( i == iEnd ) { // gdy przekroczyliśmy koniec - stąd dziury w torach...
            step -= s - fEnd; // jeszcze do wyliczenia mapowania potrzebny
            s = fEnd;
            m2 = 1.f;
            jmm2 = 0.f;
        }
/*
        while( tv1 < 0.0 ) {
            tv1 += 1.0;
        }
*/
        tv1 = clamp_circular( tv1, 1.0f );
        tv2 = tv1 - step / texturelength; // mapowanie na końcu segmentu

        t = static_cast<float>(fTsBuffer[ i ]); // szybsze od GetTFromS(s);
        pos2 = glm::dvec3{ FastGetPoint( t ) - Origin };
        dir = glm::dvec3{ FastGetDirection( t, fOffset ) }; // nowy wektor kierunku
        parallel2 = glm::vec3{ -dir.z, 0.f, dir.x }; // wektor poprzeczny
        if( glm::length2( parallel2 ) == 0.f ) {
            // temporary workaround for malformed situations with control points placed above endpoints
            parallel2 = glm::vec3{ glm::dvec3{ FastGetPoint_1() - FastGetPoint_0() } };
        }
        parallel2 = glm::normalize( parallel2 );
        if( i == iSegCount && m_setting.joints[ 1 ] ) {
            pos2 = glm::vec3{ m_setting.joints[ 1 ]->first - Origin };
            parallel2 = glm::vec3{ m_setting.joints[ 1 ]->second };
        }

        // TODO: refactor the loop, there's no need to calculate starting points for each segment when we can copy the end points of the previous one
        if( Transition ) {
            for( int j = 0; j < iNumShapePoints; ++j ) {
                pt = parallel1 * ( jmm1 * ( ShapePoints[ j ].position.x - fOffsetX.first ) + m1 * ( ShapePoints[ j + iNumShapePoints ].position.x - fOffsetX.second ) ) + pos1;
                pt.y += jmm1 * ShapePoints[ j ].position.y + m1 * ShapePoints[ j + iNumShapePoints ].position.y;
                norm = ( jmm1 * ShapePoints[ j ].normal.x + m1 * ShapePoints[ j + iNumShapePoints ].normal.x ) * parallel1;
                norm.y += jmm1 * ShapePoints[ j ].normal.y + m1 * ShapePoints[ j + iNumShapePoints ].normal.y;
                if( bRender ) {
                    // skrzyżowania podczas łączenia siatek mogą nie renderować poboczy, ale potrzebować punktów
                    Output.emplace_back(
                        pt,
                        glm::normalize( norm ),
                        glm::vec2 { ( jmm1 * ShapePoints[ j ].texture.x + m1 * ShapePoints[ j + iNumShapePoints ].texture.x ) / texturescale, tv1 } );
                    roll_vertex( Output.back(), pos1, parallel1, s - step );
                    settle_vertex( Output.back(), Origin, pos1, parallel1, s - step );
                }
                // jeśli jest wskaźnik do tablicy
                if (p && *p && !j) // to dla pierwszego punktu
                {
                    **p = pt;
                    ( *p )++;
                } // zapamiętanie brzegu jezdni
                // dla trapezu drugi koniec ma inne współrzędne
                pt = parallel2 * ( jmm2 * ( ShapePoints[ j ].position.x - fOffsetX.first ) + m2 * ( ShapePoints[ j + iNumShapePoints ].position.x - fOffsetX.second ) ) + pos2;
                pt.y += jmm2 * ShapePoints[ j ].position.y + m2 * ShapePoints[ j + iNumShapePoints ].position.y;
                norm = ( jmm2 * ShapePoints[ j ].normal.x + m2 * ShapePoints[ j + iNumShapePoints ].normal.x ) * parallel2;
                norm.y += jmm2 * ShapePoints[ j ].normal.y + m2 * ShapePoints[ j + iNumShapePoints ].normal.y;
                if( bRender ) {
                    // skrzyżowania podczas łączenia siatek mogą nie renderować poboczy, ale potrzebować punktów
                    Output.emplace_back(
                        pt,
                        glm::normalize( norm ),
                        glm::vec2 { ( jmm2 * ShapePoints[ j ].texture.x + m2 * ShapePoints[ j + iNumShapePoints ].texture.x ) / texturescale, tv2 } );
                    roll_vertex( Output.back(), pos2, parallel2, s );
                    settle_vertex( Output.back(), Origin, pos2, parallel2, s );
                }
                // jeśli jest wskaźnik do tablicy
                // to dla pierwszego punktu
                if (p && *p && !j && i == iSegCount) {
                    **p = pt;
                    ( *p )++;
                } // zapamiętanie brzegu jezdni
            }
        }
        else {
            if( bRender  ) {
                for( int j = 0; j < iNumShapePoints; ++j ) {
                    //łuk z jednym profilem
                    pt = parallel1 * ( jmm1 * ( ShapePoints[ j ].position.x - fOffsetX.first ) + m1 * ( ShapePoints[ j ].position.x - fOffsetX.second ) ) + pos1;
                    pt.y += ShapePoints[ j ].position.y;
                    norm = ShapePoints[ j ].normal.x * parallel1;
                    norm.y += ShapePoints[ j ].normal.y;

                    Output.emplace_back(
                        pt,
                        glm::normalize( norm ),
                        glm::vec2 { ShapePoints[ j ].texture.x / texturescale, tv1 } );
                    roll_vertex( Output.back(), pos1, parallel1, s - step );
                    settle_vertex( Output.back(), Origin, pos1, parallel1, s - step );

                    pt = parallel2 * ( jmm2 * ( ShapePoints[ j ].position.x - fOffsetX.first ) + m2 * ( ShapePoints[ j ].position.x - fOffsetX.second ) ) + pos2;
                    pt.y += ShapePoints[ j ].position.y;
                    norm = ShapePoints[ j ].normal.x * parallel2;
                    norm.y += ShapePoints[ j ].normal.y;

                    Output.emplace_back(
                        pt,
                        glm::normalize( norm ),
                        glm::vec2 { ShapePoints[ j ].texture.x / texturescale, tv2 } );
                    roll_vertex( Output.back(), pos2, parallel2, s );
                    settle_vertex( Output.back(), Origin, pos2, parallel2, s );
                }
            }
        }
        pos1 = pos2;
        parallel1 = parallel2;
        tv1 = tv2;
    }
    return true;
};

void TSegment::roll_vertex( gfx::basic_vertex &Vertex, glm::vec3 const &Position, glm::vec3 const &Parallel, double const Distance ) const {
    if( false == m_baked_roll.has_value() ) {
        return;
    }
    auto const delta { GetRoll( std::clamp( Distance, 0.0, fLength ) ) - *m_baked_roll };
    if( std::abs( delta ) < 1e-6f ) {
        return;
    }
    auto const c { std::cos( delta ) };
    auto const s { std::sin( delta ) };
    // across the path and up turned the way the profiles are turned by the roll, along the path as it was
    auto const turn = [&]( glm::vec3 const &Vector ) {
        auto const across { glm::dot( Vector, Parallel ) };
        auto const along { Vector - Parallel * across - glm::vec3 { 0.f, Vector.y, 0.f } };
        return along + Parallel * ( across * c + Vector.y * s ) + glm::vec3 { 0.f, -across * s + Vector.y * c, 0.f }; };
    Vertex.position = Position + turn( Vertex.position - Position );
    Vertex.normal = turn( Vertex.normal );
}

bool TSegment::RenderLoftSteps( gfx::vertex_array &Output, glm::dvec3 const &Origin, gfx::vertex_array const &ShapePoints, double fTextureLength, int const Steps ) const
{
    if( fTsBuffer.empty() || Steps < 1 || fLength <= 0.0 )
        return false;

    auto const shapepoints { ShapePoints.size() / 2 };
    auto const texturelength { static_cast<float>( fTextureLength ) };
    auto const step { static_cast<float>( fLength / Steps ) };
    auto const offset { 0.1 / fLength };
    // the curve parameter at the k-th of the Steps stations
    auto const parameter = [ this, Steps ]( int const k ) {
        if( k == 0 ) {
            return 0.0; }
        return k == Steps ? 1.0 : FastGetTFromS( fLength * k / Steps ); };
    // point and the unit vector across the path at the k-th of the Steps stations
    auto const station = [&]( int const k, glm::vec3 &Position, glm::vec3 &Parallel ) {
        if( k == 0 && m_setting.joints[ 0 ] ) {
            Position = glm::vec3{ m_setting.joints[ 0 ]->first - Origin };
            Parallel = glm::vec3{ m_setting.joints[ 0 ]->second };
            return; }
        if( k == Steps && m_setting.joints[ 1 ] ) {
            Position = glm::vec3{ m_setting.joints[ 1 ]->first - Origin };
            Parallel = glm::vec3{ m_setting.joints[ 1 ]->second };
            return; }
        auto const t { parameter( k ) };
        Position = glm::vec3{ FastGetPoint( t ) - Origin };
        auto const dir { FastGetDirection( t, offset ) };
        Parallel = glm::vec3{ -dir.z, 0.0, dir.x };
        if( glm::length2( Parallel ) == 0.f ) {
            // the same workaround as in RenderLoft for control points placed above endpoints
            Parallel = glm::vec3{ FastGetPoint_1() - FastGetPoint_0() };
        }
        Parallel = glm::normalize( Parallel ); };
    // a vertex of the profile blended Share of the way from its start to its end shape
    auto const vertex = [&]( std::size_t const j, float const Share, glm::vec3 const &Position, glm::vec3 const &Parallel, float const V ) {
        auto const &from { ShapePoints[ j ] };
        auto const &to { ShapePoints[ j + shapepoints ] };
        auto const rest { 1.f - Share };
        auto point { Parallel * ( rest * from.position.x + Share * to.position.x ) + Position };
        point.y += rest * from.position.y + Share * to.position.y;
        auto normal { ( rest * from.normal.x + Share * to.normal.x ) * Parallel };
        normal.y += rest * from.normal.y + Share * to.normal.y;
        Output.emplace_back( point, glm::normalize( normal ), glm::vec2 { rest * from.texture.x + Share * to.texture.x, V } );
        roll_vertex( Output.back(), Position, Parallel, Share * fLength );
        settle_vertex( Output.back(), Origin, Position, Parallel, Share * fLength ); };

    glm::vec3 pos1;
    glm::vec3 pos2;
    glm::vec3 parallel1;
    glm::vec3 parallel2;
    station( 0, pos1, parallel1 );
    auto tv1 { 1.f };
    for( int k = 1; k <= Steps; ++k ) {
        station( k, pos2, parallel2 );
        tv1 = clamp_circular( tv1, 1.0f );
        auto const tv2 { tv1 - step / texturelength };
        auto const m1 { static_cast<float>( k - 1 ) / Steps };
        auto const m2 { static_cast<float>( k ) / Steps };
        for( std::size_t j = 0; j < shapepoints; ++j ) {
            vertex( j, m1, pos1, parallel1, tv1 );
            vertex( j, m2, pos2, parallel2, tv2 );
        }
        pos1 = pos2;
        parallel1 = parallel2;
        tv1 = tv2;
    }
    return true;
}

namespace {

// how far ahead of and behind a distance along the path the direction of the path and of the limits of a span are taken
double constexpr chord_reach { 0.001 };

// how far out from the gauge line a point of the profile lofted along a span reaches
double reach_of( loft_span const &Span, gfx::basic_vertex const &Point ) {

    return static_cast<double>( static_cast<float>( Span.side ) * Point.position.x - Span.halfwidth );
}

// stations at the steps of the path along a span, and at the denser ones up to Dense
void step_stations( std::vector<double> &Stations, loft_span const &Span, double const Step, double const Dense ) {

    auto distance { std::ceil( Span.from / Step ) * Step };
    while( distance < Span.to ) {
        Stations.push_back( distance );
        distance += Step;
    }
    auto dense { std::ceil( Span.from / 0.5 ) * 0.5 };
    while( dense < std::min( Span.to, Dense ) ) {
        Stations.push_back( dense );
        dense += 0.5;
    }
}

// stations at the knots of the lines of a span, inside it
void knot_stations( std::vector<double> &Stations, loft_span const &Span ) {

    for( auto const *line : { &Span.offsets, &Span.extents, &Span.inner, &Span.drops } ) {
        for( auto const &knot : *line ) {
            if( knot.x > Span.from && knot.x < Span.to ) {
                Stations.push_back( knot.x );
            }
        }
    }
}

// stations where the points of the profile meet the piece of a limit of a span from knot A to knot B, inside the span
void crossing_stations( std::vector<double> &Stations, loft_span const &Span, gfx::vertex_array const &ShapePoints, glm::dvec2 const &A, glm::dvec2 const &B ) {

    for( auto const &point : ShapePoints ) {
        auto const reach { reach_of( Span, point ) };
        auto const t { ( reach - A.y ) / ( B.y - A.y ) };
        auto const distance { glm::mix( A.x, B.x, t ) };
        if( t > 0.0 && t < 1.0 && distance > Span.from && distance < Span.to ) {
            Stations.push_back( distance );
        }
    }
}

// stations where the points of the profile meet the limits of a span it's cut to
void cut_stations( std::vector<double> &Stations, loft_span const &Span, gfx::vertex_array const &ShapePoints ) {

    for( auto const *line : { &Span.extents, &Span.inner } ) {
        for( std::size_t i = 1; i < line->size(); ++i ) {
            auto const &a { ( *line )[ i - 1 ] };
            auto const &b { ( *line )[ i ] };
            if( std::abs( b.y - a.y ) < 1e-9 ) {
                continue;
            }
            crossing_stations( Stations, Span, ShapePoints, a, b );
        }
    }
}

// no stations of a span with an end cut aslant so near that end that the cut reaches past them
void trim_bent_ends( std::vector<double> &Stations, loft_span const &Span, gfx::vertex_array const &ShapePoints ) {

    if( Span.fromend.bend == 0.0 && Span.toend.bend == 0.0 ) {
        return;
    }
    auto widest { 0.0 };
    for( auto const &point : ShapePoints ) {
        widest = std::max( widest, std::abs( reach_of( Span, point ) ) );
    }
    auto const frommargin { std::abs( Span.fromend.bend ) * widest + 0.001 };
    auto const tomargin { std::abs( Span.toend.bend ) * widest + 0.001 };
    std::erase_if( Stations, [ &Span, frommargin, tomargin ]( double const Distance ) {
        return ( Distance > Span.from && Distance < Span.to )
            && ( Distance < Span.from + frommargin || Distance > Span.to - tomargin ); } );
}

// the distances along a span the profile is lofted at, in order: the steps of the path, the knots of the lines of the span and where
// the points of the profile meet its limits
std::vector<double> loft_stations( loft_span const &Span, gfx::vertex_array const &ShapePoints, double const Step, double const Dense ) {

    std::vector stations { Span.from, Span.to };
    step_stations( stations, Span, Step, Dense );
    knot_stations( stations, Span );
    cut_stations( stations, Span, ShapePoints );
    std::ranges::sort( stations );
    auto const duplicates { std::ranges::unique( stations, []( double const A, double const B ) { return B - A < 0.001; } ) };
    stations.erase( duplicates.begin(), duplicates.end() );
    trim_bent_ends( stations, Span, ShapePoints );
    return stations;
}

// a station of a span: the frame of the path there, and how the profile is moved and cut there
struct span_station {
    glm::vec3 position { 0.f }; // the point of the path, from the origin
    glm::vec3 sideways { 0.f }; // the vector across the path, towards the right rail profile
    double offset { 0.0 }; // how far the profile is moved outwards
    double extent { 0.0 }; // how far out from the gauge line the profile is cut, taken a bit towards the middle of the step
    double innerextent { 0.0 }; // how far in it's cut, likewise
    double middleextent { 0.0 }; // how far out it's cut at the middle of the step
    double middleinner { 0.0 }; // how far in it's cut there
    glm::vec3 outerwall { 0.f }; // the normal of the face the profile is cut to on its outer side, none where it isn't cut to walls
    glm::vec3 innerwall { 0.f }; // the same on its inner side
    double worn { 0.0 }; // from how far out from the gauge line the wear of the head ramps
    double drop { 0.0 }; // how far the head is lowered
    glm::vec3 along { 0.f }; // how far along the path a point goes for each metre out, at an end cut aslant
};

// the normals of the faces the profile is cut to on its outer and its inner side, none where it isn't cut to walls
struct cut_wall_normals {
    glm::vec3 outer { 0.f };
    glm::vec3 inner { 0.f };
};

// the difference between the points of the path a bit ahead of and behind a distance along it, in the plan
glm::dvec3 plan_chord( TSegment const &Segment, double const Distance ) {

    glm::dvec3 ahead;
    glm::dvec3 behind;
    glm::dvec3 unused;
    Segment.plan_frame( std::min( Distance + chord_reach, Segment.GetLength() ), ahead, unused );
    Segment.plan_frame( std::max( Distance - chord_reach, 0.0 ), behind, unused );
    return glm::dvec3 { ahead.x - behind.x, 0.0, ahead.z - behind.z };
}

// the faces the profile is cut to at a distance along a span: across the path, turned as the limits run along it
cut_wall_normals cut_walls( TSegment const &Segment, loft_span const &Span, double const Distance, glm::dvec3 const &Across ) {

    if( false == Span.walls ) {
        return {};
    }
    auto tangent { plan_chord( Segment, Distance ) };
    tangent = ( glm::length2( tangent ) > 0.0 ? glm::normalize( tangent ) : glm::dvec3{ 0.0 } );
    auto const outward { Across * static_cast<double>( Span.side ) };
    auto const slope = [ &Span, Distance ]( knots::line const &Knots, double const Default ) {
        return ( knots::sample( Knots, Distance + chord_reach, Default ) - knots::sample( Knots, Distance - chord_reach, Default ) + knots::sample( Span.offsets, Distance + chord_reach, 0.0 ) - knots::sample( Span.offsets, Distance - chord_reach, 0.0 ) ) / ( 2.0 * chord_reach ); };
    return { glm::vec3{ glm::normalize( outward - tangent * slope( Span.extents, 1e6 ) ) }, glm::vec3{ glm::normalize( tangent * slope( Span.inner, -1e6 ) - outward ) } };
}

// the end of a span a distance along it is at, a plain one between the ends
loft_end const &end_at( loft_span const &Span, double const Distance ) {

    static loft_end const unshaped;
    if( Distance <= Span.from ) {
        return Span.fromend;
    }
    if( Distance >= Span.to ) {
        return Span.toend;
    }
    return unshaped;
}

// how far along the path a point of the profile goes for each metre out, at an end of a span cut aslant; nothing elsewhere
glm::vec3 bend_along( TSegment const &Segment, loft_span const &Span, double const Distance ) {

    auto const bend { end_at( Span, Distance ).bend };
    if( bend == 0.0 ) {
        return glm::vec3 { 0.f };
    }
    if( auto const tangent { plan_chord( Segment, Distance ) }; glm::length2( tangent ) > 0.0 ) {
        return glm::vec3 { glm::normalize( tangent ) } * static_cast<float>( bend );
    }
    return glm::vec3 { 0.f };
}

// the station of a span at a distance along it, from the point of the path there and the vector across it, in the step with its middle at Middle
span_station make_station( TSegment const &Segment, loft_span const &Span, glm::vec3 const &Position, glm::dvec3 const &Across, double const Distance, double const Middle ) {

    span_station station;
    station.position = Position;
    station.sideways = glm::vec3{ Across };
    station.offset = knots::sample( Span.offsets, Distance, 0.0 );
    auto const inward { Distance + std::copysign( 1e-6, Middle - Distance ) };
    station.extent = knots::sample( Span.extents, inward, 1e6 );
    station.innerextent = knots::sample( Span.inner, inward, -1e6 );
    station.middleextent = knots::sample( Span.extents, Middle, 1e6 );
    station.middleinner = knots::sample( Span.inner, Middle, -1e6 );
    auto const walls { cut_walls( Segment, Span, Distance, Across ) };
    station.outerwall = walls.outer;
    station.innerwall = walls.inner;
    station.worn = knots::sample( Span.ramp, Distance, -1e6 );
    station.drop = knots::sample( Span.drops, Distance, 0.0 );
    station.along = bend_along( Segment, Span, Distance );
    return station;
}

// the profile lofted along a span as it's cut to the limits at one of its stations
struct profile_cut {
    gfx::vertex_array const &points;
    loft_span const &span;
    span_station const &station;
    // how far out from the gauge line a point of the profile reaches
    double reach( std::size_t const Index ) const {
        return reach_of( span, points[ Index ] ); }
    // two points of the profile at the same place: a corner of it
    bool twin( std::size_t const A, std::size_t const B ) const {
        return points[ A ].position.x == points[ B ].position.x && points[ A ].position.y == points[ B ].position.y; }
    // which limit at the middle of the step a point of the profile is beyond: 1 the outer one, -1 the inner one, 0 neither
    int clamped( std::size_t const Index ) const {
        auto const outside { reach( Index ) };
        if( outside > station.middleextent ) {
            return 1; }
        if( outside < station.middleinner ) {
            return -1; }
        return 0; }
    // a point of the profile within the limit of the Side at the middle of the step, 1 the outer one, -1 the inner one
    bool open( std::size_t const Index, int const Side ) const {
        return Index < points.size() && ( Side > 0 ? reach( Index ) <= station.middleextent : reach( Index ) >= station.middleinner ); }
};

// a point of the profile beyond a limit at a station of a span
struct beyond_limit {
    std::size_t index { 0 };
    int side { 0 }; // 1 beyond the outer limit, -1 beyond the inner one
    double limit { 0.0 }; // how far out from the gauge line the limit is there
    double outside { 0.0 }; // how far out from the gauge line the point reaches
};

// the first and the last of the points of the profile at the same place as the one at Index, the corner of a wall the profile is cut to;
// that one alone where it isn't cut to walls
std::pair<std::size_t, std::size_t> twin_run( profile_cut const &Cut, std::size_t const Index ) {

    auto first { Index };
    auto last { Index };
    if( false == Cut.span.walls ) {
        return { first, last };
    }
    while( first > 0 && Cut.twin( first - 1, Index ) ) {
        --first;
    }
    while( last + 1 < Cut.points.size() && Cut.twin( last + 1, Index ) ) {
        ++last;
    }
    return { first, last };
}

// a point of the profile beyond a limit takes the height, the texture and the normal of the profile where it crosses the limit towards
// the first of the Neighbours within it; which one that is, the count of the points of the profile if neither is
std::size_t slide_to_neighbour( profile_cut const &Cut, beyond_limit const &Point, std::array<std::size_t, 2> const &Neighbours, gfx::basic_vertex &Shape ) {

    for( auto const neighbour : Neighbours ) {
        if( false == Cut.open( neighbour, Point.side ) ) {
            continue;
        }
        auto const &other { Cut.points[ neighbour ] };
        auto const t { std::clamp( static_cast<float>( ( Point.limit - Cut.reach( neighbour ) ) / ( Point.side > 0 ? std::max( Point.outside - Cut.reach( neighbour ), 1e-6 ) : std::min( Point.outside - Cut.reach( neighbour ), -1e-6 ) ) ), 0.f, 1.f ) };
        Shape.position.y = glm::mix( other.position.y, Shape.position.y, t );
        Shape.texture.x = glm::mix( other.texture.x, Shape.texture.x, t );
        auto const blend { glm::mix( other.normal, Shape.normal, t ) };
        Shape.normal = ( glm::length2( blend ) > 1e-6f ? blend : other.normal );
        return neighbour;
    }
    return Cut.points.size();
}

// a point of the profile beyond a limit with no neighbour within it to slide to keeps between the heights the profile crosses the limit at,
// on either side of the run of points beyond it
void cap_height( profile_cut const &Cut, beyond_limit const &Point, gfx::basic_vertex &Shape ) {

    auto const count { static_cast<std::ptrdiff_t>( Cut.points.size() ) };
    auto top { -1e6f };
    auto bottom { 1e6f };
    auto ends { 0 };
    for( auto const way : { -1, 1 } ) {
        auto index { static_cast<std::ptrdiff_t>( Point.index ) };
        while( index + way >= 0 && index + way < count && Cut.clamped( static_cast<std::size_t>( index + way ) ) == Point.side ) {
            index += way;
        }
        auto const neighbour { index + way };
        if( neighbour < 0 || false == Cut.open( static_cast<std::size_t>( neighbour ), Point.side ) ) {
            continue;
        }
        auto const t { static_cast<float>( ( Point.limit - Cut.reach( static_cast<std::size_t>( neighbour ) ) ) / ( Cut.reach( static_cast<std::size_t>( index ) ) - Cut.reach( static_cast<std::size_t>( neighbour ) ) ) ) };
        auto const height { glm::mix( Cut.points[ static_cast<std::size_t>( neighbour ) ].position.y, Cut.points[ static_cast<std::size_t>( index ) ].position.y, t ) };
        top = std::max( top, height );
        bottom = std::min( bottom, height );
        ++ends;
    }
    if( ends > 0 ) {
        Shape.position.y = std::min( Shape.position.y, top );
    }
    if( ends > 1 ) {
        Shape.position.y = std::max( Shape.position.y, bottom );
    }
}

// a point of the profile beyond a limit at a station pulled in to it, taking the height of the profile where that crosses the limit;
// the normal of the wall it then lies on, none if it doesn't lie on one
glm::vec3 pull_in( profile_cut const &Cut, beyond_limit const &Point, gfx::basic_vertex &Shape, double &Outside ) {

    auto const count { Cut.points.size() };
    auto const [ first, last ] { twin_run( Cut, Point.index ) };
    auto const paired { last > first && first > 0 && last + 1 < count };
    auto const before { paired ? first - 1 : Point.index - 1 };
    auto const after { paired ? last + 1 : Point.index + 1 };
    auto const partner { Point.index == last ? after : before };
    auto const opposite { partner == before ? after : before };
    auto const moved { slide_to_neighbour( Cut, Point, { paired ? partner : before, paired ? opposite : after }, Shape ) };
    if( moved == count ) {
        cap_height( Cut, Point, Shape );
    }
    Outside = ( Point.side > 0 ? std::min( Outside, Point.limit ) : std::max( Outside, Point.limit ) );
    if( auto const onwall { paired ? ( false == Cut.open( partner, Point.side ) && Cut.clamped( partner ) == Point.side ) : moved == count }; false == onwall ) {
        return glm::vec3 { 0.f };
    }
    auto wall { Point.side > 0 ? Cut.station.outerwall : Cut.station.innerwall };
    // a face of the profile squeezed onto the wall from the other side of the rail keeps facing that side: the thin plate a
    // planed blade gets to near its tip is lit from the middle of the track as its face there, not as the one against the stock rail
    if( static_cast<float>( Cut.span.side * Point.side ) * Cut.points[ Point.index ].normal.x < 0.f ) {
        wall = -wall;
    }
    return wall;
}

// how much of the wear of the head a point of the profile takes, from how far out it reaches past where the wear ramps from
float head_wear( loft_span const &Span, double const Outside, double const Worn ) {

    if( Span.rampwidth == 0.0 ) {
        return 1.f;
    }
    auto const beyond { Span.rampwidth > 0.0 ? Outside - Worn : Worn - Outside };
    return static_cast<float>( std::clamp( beyond / std::abs( Span.rampwidth ) + 1.0, 0.0, 1.0 ) );
}

// a point of the profile at a station of a span onto the ring there: pulled in to the limits, the head lowered where it drops
void ring_point( profile_cut const &Cut, std::size_t const Index, std::vector<gfx::basic_vertex> &Ring, std::vector<float> &Wear ) {

    auto const &span { Cut.span };
    auto const &station { Cut.station };
    auto shape { Cut.points[ Index ] };
    auto outside { Cut.reach( Index ) };
    auto wall { glm::vec3{ 0.f } };
    if( auto const side { Cut.clamped( Index ) }; side != 0 ) {
        beyond_limit const beyond { Index, side, side > 0 ? station.extent : station.innerextent, outside };
        wall = pull_in( Cut, beyond, shape, outside );
    }
    if( station.drop > 0.0 && shape.position.y > span.head.x && span.head.y > span.head.x ) {
        shape.position.y -= static_cast<float>( station.drop * ( shape.position.y - span.head.x ) / ( span.head.y - span.head.x ) );
    }
    auto pt { station.sideways * static_cast<float>( span.side * ( span.halfwidth + outside + station.offset ) ) + station.position + station.along * static_cast<float>( outside + station.offset ) };
    pt.y += shape.position.y + span.lift;
    auto norm { shape.normal.x * station.sideways };
    norm.y += shape.normal.y;
    if( glm::length2( wall ) > 0.f ) {
        norm = wall;
        if( span.wallu >= 0.f ) {
            shape.texture.x = span.wallu;
        }
    }
    Ring.emplace_back( pt, glm::normalize( norm ), glm::vec2{ shape.texture.x, 0.f } );
    Wear.push_back( head_wear( span, outside, station.worn ) );
}

// the ring of a span where nothing of the profile is between the limits: all its points at one, so the step draws nothing
void shrunk_ring( profile_cut const &Cut, double const Nearest, double const Farthest, std::vector<gfx::basic_vertex> &Ring, std::vector<float> &Wear ) {

    auto const &span { Cut.span };
    auto const &station { Cut.station };
    auto const outside { std::clamp( 0.5 * ( std::min( station.extent, Farthest ) + std::max( station.innerextent, Nearest ) ), Nearest, Farthest ) };
    auto pt { station.sideways * static_cast<float>( span.side * ( span.halfwidth + outside + station.offset ) ) + station.position };
    pt.y += Cut.points.front().position.y + span.lift;
    for( auto const &point : Cut.points ) {
        Ring.emplace_back( pt, glm::vec3{ 0.f, 1.f, 0.f }, glm::vec2{ point.texture.x, 0.f } );
        Wear.push_back( 1.f );
    }
}

// the ring of points of the profile at a station of a span, cut to the limits there, and how worn each one is; false where it shrinks to
// one point, which is left as it is, neither rolled nor settled
bool lay_ring( gfx::vertex_array const &ShapePoints, loft_span const &Span, span_station const &Station, std::vector<gfx::basic_vertex> &Ring, std::vector<float> &Wear ) {

    Ring.clear();
    Wear.clear();
    profile_cut const cut { ShapePoints, Span, Station };
    auto const count { ShapePoints.size() };
    auto nearest { 1e6 };
    auto farthest { -1e6 };
    for( std::size_t j = 0; j < count; ++j ) {
        nearest = std::min( nearest, cut.reach( j ) );
        farthest = std::max( farthest, cut.reach( j ) );
    }
    if( Station.middleextent <= Station.middleinner || Station.middleextent < nearest || Station.middleinner > farthest ) {
        // nothing of the profile between the limits there: the ring shrinks to one point, the step draws nothing
        shrunk_ring( cut, nearest, farthest, Ring, Wear );
        return false;
    }
    for( std::size_t j = 0; j < count; ++j ) {
        ring_point( cut, j, Ring, Wear );
    }
    return true;
}

} // namespace

bool TSegment::RenderLoftSpan( gfx::vertex_array &Output, glm::dvec3 const &Origin, gfx::vertex_array const &ShapePoints, loft_span const &Span ) const {

    if( fTsBuffer.empty() || Span.to - Span.from < 0.001 || ShapePoints.empty() )
        return false;

    auto const stations { loft_stations( Span, ShapePoints, std::max( fStep, 0.2 ), m_dense ) };

    // the ring of the profile at a station, cut to the limits there, turned with the roll of the path and settled with its unevenness
    auto const ring = [ this, &Origin, &ShapePoints, &Span ]( double const Distance, double const Middle, std::vector<gfx::basic_vertex> &Ring, std::vector<float> &Wear ) {
        glm::dvec3 point;
        glm::dvec3 across;
        plan_frame( Distance, point, across );
        if( auto const &joint { m_setting.joints[ Distance < 0.001 ? 0 : 1 ] }; joint && ( Distance < 0.001 || Distance > fLength - 0.001 ) ) {
            point = joint->first;
            across = joint->second;
        }
        auto const station { make_station( *this, Span, glm::vec3{ point - Origin }, across, Distance, Middle ) };
        if( false == lay_ring( ShapePoints, Span, station, Ring, Wear ) ) {
            return;
        }
        for( auto &vertex : Ring ) {
            roll_vertex( vertex, station.position, station.sideways, Distance );
            settle_vertex( vertex, Origin, station.position, station.sideways, Distance );
        }
    };

    auto const texturelength { static_cast<float>( Span.texturelength ) };
    std::vector<gfx::basic_vertex> ring1;
    std::vector<gfx::basic_vertex> ring2;
    std::vector<float> wear1;
    std::vector<float> wear2;
    float tv1 { clamp_circular( 1.f - static_cast<float>( std::fmod( Span.from, Span.texturelength ) / Span.texturelength ), 1.0f ) };
    for( std::size_t i = 1; i < stations.size(); ++i ) {
        auto const middle { 0.5 * ( stations[ i - 1 ] + stations[ i ] ) };
        ring( stations[ i - 1 ], middle, ring1, wear1 );
        ring( stations[ i ], middle, ring2, wear2 );
        tv1 = clamp_circular( tv1, 1.0f );
        auto const tv2 { tv1 - static_cast<float>( stations[ i ] - stations[ i - 1 ] ) / texturelength };
        for( std::size_t j = 0; j < ring1.size(); ++j ) {
            Output.emplace_back( ring1[ j ].position, ring1[ j ].normal, glm::vec2{ ring1[ j ].texture.x, tv1 } );
            Output.emplace_back( ring2[ j ].position, ring2[ j ].normal, glm::vec2{ ring2[ j ].texture.x, tv2 } );
            if( Span.userdata != nullptr ) {
                Span.userdata->push_back( { glm::vec4{ wear1[ j ], 0.f, 0.f, 0.f } } );
                Span.userdata->push_back( { glm::vec4{ wear2[ j ], 0.f, 0.f, 0.f } } );
            }
        }
        tv1 = tv2;
    }
    return true;
}

void TSegment::render_lines(std::vector<gfx::basic_vertex> &out, float quality) const
{
	float step = 1.0f / iSegCount / quality;
	auto const stepcount = static_cast<int>(static_cast<float>(iSegCount) * quality);

	glm::vec3 previous = FastGetPoint(0.0);

	for (int i = 1; i <= stepcount; ++i) {
		float const x = std::min(1.0f, static_cast<float>(i) * step);
		out.push_back(gfx::basic_vertex(previous, glm::vec3(0.0f), glm::vec2(0.0f)));

		previous = glm::vec3(FastGetPoint(x));
		out.emplace_back(previous, glm::vec3(0.0f), glm::vec2(0.0f));
	}

	out.emplace_back(previous, glm::vec3(0.0f), glm::vec2(0.0f));

	previous = glm::vec3(FastGetPoint(1.0));
	out.emplace_back(previous, glm::vec3(0.0f), glm::vec2(0.0f));
}

void TSegment::plan_bounds(glm::dvec2 &minimum, glm::dvec2 &maximum) const
{
	if (!bCurve && iSegCount == 1)
	{
		minimum = glm::min(glm::dvec2(Point1.x, Point1.z), glm::dvec2(Point2.x, Point2.z));
		maximum = glm::max(glm::dvec2(Point1.x, Point1.z), glm::dvec2(Point2.x, Point2.z));
		return;
	}

	std::array<glm::dvec3, 4> const control_points { Point1, CPointOut, CPointIn, Point2 };
	minimum = glm::dvec2(std::numeric_limits<double>::max());
	maximum = glm::dvec2(std::numeric_limits<double>::lowest());
	for (glm::dvec3 const &control_point : control_points)
	{
		glm::dvec2 const point(control_point.x, control_point.z);
		minimum = glm::min(minimum, point);
		maximum = glm::max(maximum, point);
	}
}

glm::vec3 TSegment::get_nearest_point(const glm::dvec3 &point, float quality) const
{
	float step = 1.0f / iSegCount / quality;
	auto const stepcount = static_cast<int>(static_cast<float>(iSegCount) * quality);

	glm::vec3 nearest;
	float min = std::numeric_limits<float>::max();

	for (int i = 1; i <= stepcount; ++i) {
		float const x = std::min(1.0f, static_cast<float>(i) * step);
        glm::vec3 p1 = FastGetPoint(x);

        if (glm::vec3 p2 = FastGetPoint(glm::min(1.0f, x + step)); p1 != p2) {
            float l2 = glm::distance2(p1, p2);
            float t = glm::max(0.0f, glm::min(1.0f, glm::dot((glm::vec3)point - p1, p2 - p1) / l2));
            glm::vec3 proj = p1 + t * (p2 - p1);
            p1 = proj;
        }

        float dist2 = glm::distance2(p1, (glm::vec3)point);

		if (dist2 < min) {
            nearest = p1;
			min = dist2;
		}
	}

	return nearest;
}
