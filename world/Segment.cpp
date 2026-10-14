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
    Point1 = NewPoint1;
    CPointOut = NewCPointOut;
    CPointIn = NewCPointIn;
    Point2 = NewPoint2;
    // poprawienie przechyłki
    fRoll1 = static_cast<float>(glm::radians(fNewRoll1)); // Ra: przeliczone jest bardziej przydatne do obliczeń
    fRoll2 = static_cast<float>(glm::radians(fNewRoll2));
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
    if (fLength <= 0) {

        ErrorLog( "Bad track: zero length spline \"" + pOwner->name() + "\" (location: " + to_string( glm::dvec3{ Point1 } ) + ")" );
        fLength = 0.01; // crude workaround TODO: fix this properly
    }

    fStoop = std::atan2(Point2.y - Point1.y, fLength); // pochylenie toru prostego, żeby nie liczyć wielokrotnie

    fStep = fNewStep;
    // NOTE: optionally replace this part with the commented version, after solving geometry issues with double switches
    if( pOwner->eType == tt_Switch
     && fStep * (3.0 * Global.SplineFidelity) > fLength ) {
        // NOTE: a workaround for too short switches (less than 3 segments) messing up animation/generation of blades
        fStep = fLength / ( 3.0 * Global.SplineFidelity );
    }
//    iSegCount = static_cast<int>( std::ceil( fLength / fStep ) ); // potrzebne do VBO
    iSegCount = pOwner->eType == tt_Switch ? static_cast<int>(6 * Global.SplineFidelity) : static_cast<int>(std::ceil(fLength / fStep)); // potrzebne do VBO

    fStep = fLength / iSegCount; // update step to equalize size of individual pieces

	InitTFromSInterpolateData();
    return true;
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

void TSegment::RaPositionGet(double const fDistance, glm::dvec3 &position, glm::vec3 &rotation) const {
    if (bCurve) {
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
};

glm::dvec3 TSegment::FastGetPoint(double const t) const
{
    // return (bCurve?std::lerp(t,Point1,CPointOut,CPointIn,Point2):((1.0-t)*Point1+(t)*Point2));
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
    if( iSkip == 0 && m_joints[ 0 ] ) {
        pos1 = glm::vec3{ m_joints[ 0 ]->first - Origin };
        parallel1 = glm::vec3{ m_joints[ 0 ]->second };
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
        if( i == iSegCount && m_joints[ 1 ] ) {
            pos2 = glm::vec3{ m_joints[ 1 ]->first - Origin };
            parallel2 = glm::vec3{ m_joints[ 1 ]->second };
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

                    pt = parallel2 * ( jmm2 * ( ShapePoints[ j ].position.x - fOffsetX.first ) + m2 * ( ShapePoints[ j ].position.x - fOffsetX.second ) ) + pos2;
                    pt.y += ShapePoints[ j ].position.y;
                    norm = ShapePoints[ j ].normal.x * parallel2;
                    norm.y += ShapePoints[ j ].normal.y;

                    Output.emplace_back(
                        pt,
                        glm::normalize( norm ),
                        glm::vec2 { ShapePoints[ j ].texture.x / texturescale, tv2 } );
                }
            }
        }
        pos1 = pos2;
        parallel1 = parallel2;
        tv1 = tv2;
    }
    return true;
};

void TSegment::plan_frame( double const Distance, glm::dvec3 &Point, glm::dvec3 &Across ) const {
    auto const t { FastGetTFromS( std::clamp( Distance, 0.0, fLength ) ) };
    Point = FastGetPoint( t );
    auto const dir { FastGetDirection( t, 0.1 / fLength ) };
    Across = glm::dvec3{ -dir.z, 0.0, dir.x };
    if( glm::length2( Across ) == 0.0 ) {
        auto const chord { FastGetPoint_1() - FastGetPoint_0() };
        Across = glm::dvec3{ -chord.z, 0.0, chord.x };
    }
    Across = glm::normalize( Across );
}

bool TSegment::RenderLoftSpan( gfx::vertex_array &Output, glm::dvec3 const &Origin, gfx::vertex_array const &ShapePoints, int const Side, float const Halfwidth, double const fTextureLength, double const From, double const To, std::vector<glm::dvec2> const &Offsets, std::vector<glm::dvec2> const &Extents, float const Lift, std::vector<glm::dvec2> const &Inner, gfx::userdata_array *Userdata, std::vector<glm::dvec2> const &Ramp, double const Rampwidth, bool const Walls, float const Wallu, glm::dvec2 const &Bends, std::vector<glm::dvec2> const &Fitfrom, std::vector<glm::dvec2> const &Fitto ) const {

    if( fTsBuffer.empty() || To - From < 0.001 || ShapePoints.empty() )
        return false;

    auto const sample = []( std::vector<glm::dvec2> const &Knots, double const Distance, double const Default ) {
        if( Knots.empty() )
            return Default;
        if( Distance <= Knots.front().x )
            return Knots.front().y;
        for( std::size_t i = 1; i < Knots.size(); ++i ) {
            if( Distance <= Knots[ i ].x ) {
                return glm::mix( Knots[ i - 1 ].y, Knots[ i ].y, ( Distance - Knots[ i - 1 ].x ) / std::max( Knots[ i ].x - Knots[ i - 1 ].x, 1e-6 ) );
            }
        }
        return Knots.back().y; };

    std::vector<double> stations { From, To };
    auto const step { std::max( fStep, 0.2 ) };
    for( auto distance { std::ceil( From / step ) * step }; distance < To; distance += step ) {
        stations.push_back( distance );
    }
    for( auto const *knots : { &Offsets, &Extents, &Inner } ) {
        for( auto const &knot : *knots ) {
            if( knot.x > From && knot.x < To ) {
                stations.push_back( knot.x );
            }
        }
    }
    for( auto const *knots : { &Extents, &Inner } ) {
        for( std::size_t i = 1; i < knots->size(); ++i ) {
            auto const &a { ( *knots )[ i - 1 ] };
            auto const &b { ( *knots )[ i ] };
            if( std::abs( b.y - a.y ) < 1e-9 ) {
                continue;
            }
            for( auto const &point : ShapePoints ) {
                auto const reach { static_cast<double>( Side * point.position.x - Halfwidth ) };
                auto const t { ( reach - a.y ) / ( b.y - a.y ) };
                auto const distance { glm::mix( a.x, b.x, t ) };
                if( t > 0.0 && t < 1.0 && distance > From && distance < To ) {
                    stations.push_back( distance );
                }
            }
        }
    }
    std::sort( stations.begin(), stations.end() );
    stations.erase( std::unique( stations.begin(), stations.end(), []( double const A, double const B ) { return B - A < 0.001; } ), stations.end() );
    if( Bends.x != 0.0 || Bends.y != 0.0 || false == Fitfrom.empty() || false == Fitto.empty() ) {
        auto widest { 0.0 };
        for( auto const &point : ShapePoints ) {
            widest = std::max( widest, std::abs( static_cast<double>( Side * point.position.x - Halfwidth ) ) );
        }
        auto offsetmost { 0.0 };
        for( auto const &knot : Offsets ) {
            offsetmost = std::max( offsetmost, std::abs( knot.y ) );
        }
        auto const margin = [&]( double const Bend, std::vector<glm::dvec2> const &Fit ) {
            auto fitmost { 0.0 };
            for( auto const &knot : Fit ) {
                fitmost = std::max( fitmost, std::abs( knot.y ) );
            }
            return std::abs( Bend ) * ( Fit.empty() ? widest : widest + offsetmost ) + fitmost + 0.001; };
        auto const frommargin { margin( Bends.x, Fitfrom ) };
        auto const tomargin { margin( Bends.y, Fitto ) };
        stations.erase( std::remove_if( stations.begin(), stations.end(), [&]( double const Distance ) {
            return ( Distance > From && Distance < To )
                && ( Distance < From + frommargin || Distance > To - tomargin ); } ), stations.end() );
    }

    auto const ring = [&]( double const Distance, double const Middle, std::vector<gfx::basic_vertex> &Ring, std::vector<float> &Wear ) {
        glm::dvec3 point;
        glm::dvec3 across;
        plan_frame( Distance, point, across );
        if( auto const &joint { m_joints[ Distance < 0.001 ? 0 : 1 ] }; joint && ( Distance < 0.001 || Distance > fLength - 0.001 ) ) {
            point = joint->first;
            across = joint->second;
        }
        auto const position { glm::vec3{ point - Origin } };
        auto const parallel { glm::vec3{ across } };
        auto const offset { sample( Offsets, Distance, 0.0 ) };
        auto const extent { sample( Extents, Distance, 1e6 ) };
        auto const innerextent { sample( Inner, Distance, -1e6 ) };
        auto const middleextent { sample( Extents, Middle, 1e6 ) };
        auto const middleinner { sample( Inner, Middle, -1e6 ) };
        glm::vec3 outerwall { 0.f }, innerwall { 0.f };
        if( Walls ) {
            auto const delta { 0.001 };
            glm::dvec3 ahead, behind, unused;
            plan_frame( std::min( Distance + delta, static_cast<double>( fLength ) ), ahead, unused );
            plan_frame( std::max( Distance - delta, 0.0 ), behind, unused );
            auto tangent { glm::dvec3{ ahead.x - behind.x, 0.0, ahead.z - behind.z } };
            tangent = ( glm::length2( tangent ) > 0.0 ? glm::normalize( tangent ) : glm::dvec3{ 0.0 } );
            auto const outward { glm::dvec3{ across } * static_cast<double>( Side ) };
            auto const slope = [&]( std::vector<glm::dvec2> const &Knots, double const Default ) {
                return ( sample( Knots, Distance + delta, Default ) - sample( Knots, Distance - delta, Default ) + sample( Offsets, Distance + delta, 0.0 ) - sample( Offsets, Distance - delta, 0.0 ) ) / ( 2.0 * delta ); };
            outerwall = glm::vec3{ glm::normalize( outward - tangent * slope( Extents, 1e6 ) ) };
            innerwall = glm::vec3{ glm::normalize( tangent * slope( Inner, -1e6 ) - outward ) };
        }
        auto const worn { sample( Ramp, Distance, -1e6 ) };
        auto const bend { Distance <= From ? Bends.x : Distance >= To ? Bends.y : 0.0 };
        static std::vector<glm::dvec2> const unfitted;
        auto const &fit { Distance <= From ? Fitfrom : Distance >= To ? Fitto : unfitted };
        auto along { glm::vec3 { 0.f } };
        auto forward { glm::vec3 { 0.f } };
        if( bend != 0.0 || false == fit.empty() ) {
            glm::dvec3 ahead, behind, unused;
            plan_frame( std::min( Distance + 0.001, static_cast<double>( fLength ) ), ahead, unused );
            plan_frame( std::max( Distance - 0.001, 0.0 ), behind, unused );
            auto const tangent { glm::dvec3 { ahead.x - behind.x, 0.0, ahead.z - behind.z } };
            if( glm::length2( tangent ) > 0.0 ) {
                forward = glm::vec3 { glm::normalize( tangent ) };
                along = forward * static_cast<float>( bend );
            }
        }
        Ring.clear();
        Wear.clear();
        auto const count { ShapePoints.size() };
        auto const reach = [&]( std::size_t const Index ) {
            return static_cast<double>( Side * ShapePoints[ Index ].position.x - Halfwidth ); };
        auto const twin = [&]( std::size_t const A, std::size_t const B ) {
            return ShapePoints[ A ].position.x == ShapePoints[ B ].position.x && ShapePoints[ A ].position.y == ShapePoints[ B ].position.y; };
        auto const clamped = [&]( std::size_t const Index ) {
            auto const outside { reach( Index ) };
            return ( outside > middleextent ? 1 : outside < middleinner ? -1 : 0 ); };
        for( std::size_t j = 0; j < count; ++j ) {
            auto shape { ShapePoints[ j ] };
            auto outside { reach( j ) };
            auto wall { glm::vec3{ 0.f } };
            auto const side { clamped( j ) };
            if( side != 0 ) {
                auto const limit { side > 0 ? extent : innerextent };
                auto const open = [&]( std::size_t const Index ) {
                    return Index < count && ( side > 0 ? reach( Index ) <= middleextent : reach( Index ) >= middleinner ); };
                auto first { j };
                auto last { j };
                if( Walls ) {
                    while( first > 0 && twin( first - 1, j ) ) {
                        --first;
                    }
                    while( last + 1 < count && twin( last + 1, j ) ) {
                        ++last;
                    }
                }
                auto const paired { last > first && first > 0 && last + 1 < count };
                auto const before { paired ? first - 1 : j - 1 };
                auto const after { paired ? last + 1 : j + 1 };
                auto const partner { j == last ? after : before };
                auto moved { count };
                for( auto const neighbour : { paired ? partner : before, paired ? ( partner == before ? after : before ) : after } ) {
                    if( false == open( neighbour ) ) {
                        continue;
                    }
                    auto const &other { ShapePoints[ neighbour ] };
                    auto const t { static_cast<float>( ( limit - reach( neighbour ) ) / ( side > 0 ? std::max( outside - reach( neighbour ), 1e-6 ) : std::min( outside - reach( neighbour ), -1e-6 ) ) ) };
                    shape.position.y = glm::mix( other.position.y, shape.position.y, t );
                    shape.texture.x = glm::mix( other.texture.x, shape.texture.x, t );
                    auto const blend { glm::mix( other.normal, shape.normal, t ) };
                    shape.normal = ( glm::length2( blend ) > 1e-6f ? blend : other.normal );
                    moved = neighbour;
                    break;
                }
                if( moved == count ) {
                    auto top { -1e6f };
                    auto bottom { 1e6f };
                    auto ends { 0 };
                    for( auto const step : { -1, 1 } ) {
                        auto index { static_cast<std::ptrdiff_t>( j ) };
                        while( index + step >= 0 && index + step < static_cast<std::ptrdiff_t>( count ) && clamped( static_cast<std::size_t>( index + step ) ) == side ) {
                            index += step;
                        }
                        auto const neighbour { index + step };
                        if( neighbour < 0 || false == open( static_cast<std::size_t>( neighbour ) ) ) {
                            continue;
                        }
                        auto const t { static_cast<float>( ( limit - reach( static_cast<std::size_t>( neighbour ) ) ) / ( reach( static_cast<std::size_t>( index ) ) - reach( static_cast<std::size_t>( neighbour ) ) ) ) };
                        auto const height { glm::mix( ShapePoints[ neighbour ].position.y, ShapePoints[ index ].position.y, t ) };
                        top = std::max( top, height );
                        bottom = std::min( bottom, height );
                        ++ends;
                    }
                    if( ends > 0 ) {
                        shape.position.y = std::min( shape.position.y, top );
                    }
                    if( ends > 1 ) {
                        shape.position.y = std::max( shape.position.y, bottom );
                    }
                }
                outside = limit;
                auto const onwall { paired ? ( false == open( partner ) && clamped( partner ) == side ) : moved == count };
                if( onwall ) {
                    wall = ( side > 0 ? outerwall : innerwall );
                }
            }
            auto pt { parallel * static_cast<float>( Side * ( Halfwidth + outside + offset ) ) + position + along * static_cast<float>( outside + offset ) + forward * static_cast<float>( sample( fit, shape.position.y, 0.0 ) ) };
            pt.y += shape.position.y + Lift;
            auto norm { shape.normal.x * parallel };
            norm.y += shape.normal.y;
            if( glm::length2( wall ) > 0.f ) {
                norm = wall;
                if( Wallu >= 0.f ) {
                    shape.texture.x = Wallu;
                }
            }
            Ring.emplace_back( pt, glm::normalize( norm ), glm::vec2{ shape.texture.x, 0.f } );
            Wear.push_back( Rampwidth != 0.0 ? static_cast<float>( std::clamp( ( Rampwidth > 0.0 ? outside - worn : worn - outside ) / std::abs( Rampwidth ) + 1.0, 0.0, 1.0 ) ) : 1.f );
        }
    };

    auto const texturelength { static_cast<float>( fTextureLength ) };
    std::vector<gfx::basic_vertex> ring1, ring2;
    std::vector<float> wear1, wear2;
    float tv1 { clamp_circular( 1.f - static_cast<float>( std::fmod( From, fTextureLength ) / fTextureLength ), 1.0f ) };
    for( std::size_t i = 1; i < stations.size(); ++i ) {
        auto const middle { 0.5 * ( stations[ i - 1 ] + stations[ i ] ) };
        ring( stations[ i - 1 ], middle, ring1, wear1 );
        ring( stations[ i ], middle, ring2, wear2 );
        tv1 = clamp_circular( tv1, 1.0f );
        auto const tv2 { tv1 - static_cast<float>( stations[ i ] - stations[ i - 1 ] ) / texturelength };
        for( std::size_t j = 0; j < ring1.size(); ++j ) {
            Output.emplace_back( ring1[ j ].position, ring1[ j ].normal, glm::vec2{ ring1[ j ].texture.x, tv1 } );
            Output.emplace_back( ring2[ j ].position, ring2[ j ].normal, glm::vec2{ ring2[ j ].texture.x, tv2 } );
            if( Userdata != nullptr ) {
                Userdata->push_back( { glm::vec4{ wear1[ j ], 0.f, 0.f, 0.f } } );
                Userdata->push_back( { glm::vec4{ wear2[ j ], 0.f, 0.f, 0.f } } );
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
