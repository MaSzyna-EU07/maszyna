/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "utilities/Classes.h"
#include "rendering/geometrybank.h"
#include <glm/glm.hpp>

struct map_colored_paths {
	std::vector<gfx::geometrybank_handle> switches;
	std::vector<gfx::geometrybank_handle> occupied;
	std::vector<gfx::geometrybank_handle> future;
    std::vector<gfx::geometrybank_handle> highlighted;
};

struct segment_data {
// types
    enum point {
        start = 0,
        control1,
        control2,
        end
    };
// members
    std::array<glm::dvec3, 4> points {};
    std::array<float, 2> rolls {};
    float radius {};
// constructors
    segment_data() = default;
// methods
    void deserialize( cParser &Input, glm::dvec3 const &Offset );
};

class TSegment
{ // aproksymacja toru (zwrotnica ma dwa takie, jeden z nich jest aktywny)
  private:
    glm::dvec3 Point1, CPointOut, CPointIn, Point2;
    float
        fRoll1 { 0.f },
        fRoll2 { 0.f }; // przechyłka na końcach
    double fLength { -1.0 }; // długość policzona
    std::vector<double> fTsBuffer; // wartości parametru krzywej dla równych odcinków
    double fStep = 0.0;
    int iSegCount = 0; // ilość odcinków do rysowania krzywej
    double fDirection = 0.0; // Ra: kąt prostego w planie; dla łuku kąt od Point1
    double fStoop = 0.0; // Ra: kąt wzniesienia; dla łuku od Point1
	glm::dvec3 vA, vB, vC; // współczynniki wielomianów trzeciego stopnia vD==Point1
    TTrack *pOwner = nullptr; // wskaźnik na właściciela

    glm::dvec3 GetFirstDerivative(double const fTime) const;
	double RombergIntegral(double const fA, double const fB) const;
	double GetTFromS(double const s) const;
	glm::dvec3 RaInterpolate(double const t) const;
	glm::dvec3 RaInterpolate0(double const t) const;

public:
    bool bCurve = false;

    TSegment(TTrack *owner);
    bool Init(glm::dvec3 NewPoint1, glm::dvec3 NewPoint2, double fNewStep, double fNewRoll1 = 0, double fNewRoll2 = 0);
	bool Init(glm::dvec3 &NewPoint1, glm::dvec3 NewCPointOut, glm::dvec3 NewCPointIn, glm::dvec3 &NewPoint2, double fNewStep, double fNewRoll1 = 0, double fNewRoll2 = 0, bool bIsCurve = true);
    double
        ComputeLength() const; // McZapkie-150503
    // finds point on segment closest to specified point in 3d space. returns: point on segment as value in range 0-1
    double
        find_nearest_point( glm::dvec3 const &Point ) const;
    inline 
    glm::dvec3
        GetDirection1() const {
            return bCurve ? CPointOut - Point1 : CPointOut; };
    inline
    glm::dvec3
        GetDirection2() const {
            return bCurve ? CPointIn - Point2 : CPointIn; };
	glm::dvec3
        GetDirection(double const fDistance) const;
    inline
    glm::dvec3
        GetDirection() const {
            return CPointOut; };
	glm::dvec3
        FastGetDirection(double const fDistance, double const fOffset);
/*
    Math3D::vector3
        GetPoint(double const fDistance) const;
*/

	/// <summary>
	/// ustalenie pozycji osi na torze, przechyłki, pochylenia i kierunku jazdy
	/// </summary>
	/// <param name="fDistance">Distance from p1</param>
	/// <param name="position">Calculated position</param>
	/// <param name="rotation">Calculated rotation</param>
	void RaPositionGet(double const fDistance, glm::dvec3 &position, glm::vec3 &rotation) const;
	glm::dvec3 FastGetPoint(double const t) const;
    inline
    glm::dvec3
        FastGetPoint_0() const {
            return Point1; };
    inline
    glm::dvec3
        FastGetPoint_1() const {
            return Point2; };
    inline
    float
        GetRoll(double const Distance) const {
            return std::lerp( fRoll1, fRoll2, static_cast<float>(Distance / fLength) ); }
    inline
    void
        GetRolls(float &r1, float &r2) const {
            // pobranie przechyłek (do generowania trójkątów)
            r1 = fRoll1;
            r2 = fRoll2; }

    bool
        RenderLoft( gfx::vertex_array &Output, glm::dvec3 const &Origin, gfx::vertex_array const &ShapePoints, bool const Transition, double fTextureLength, double Texturescale = 1.0, int iSkip = 0, int iEnd = 0, std::pair<float, float> fOffsetX = {0.f, 0.f}, glm::vec3 **p = nullptr, bool bRender = true );
/*
    void
        Render();
*/
    inline
    double
        GetLength() const {
            return fLength; };
    inline
    int
        RaSegCount() const {
            return fTsBuffer.empty() ? 1 : iSegCount; };

	void
	    render_lines(std::vector<gfx::basic_vertex> &out, float quality = 1.0f) const;

	glm::vec3
	    get_nearest_point(const glm::dvec3 &point, float quality = 1.0f) const;
};

//---------------------------------------------------------------------------
