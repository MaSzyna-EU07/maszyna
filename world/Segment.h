/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <optional>
#include <utility>

#include "utilities/Classes.h"
#include "rendering/geometrybank.h"
#include "world/TrackUneven.h"
#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <optional>
#include "world/Knots.h"

struct map_colored_paths {
	std::vector<gfx::geometrybank_handle> roads;
	std::vector<gfx::geometrybank_handle> switches;
	std::vector<gfx::geometrybank_handle> occupied;
	std::vector<gfx::geometrybank_handle> future;
    std::vector<gfx::geometrybank_handle> highlighted;
};

struct loft_end {
    double bend { 0.0 };
};

struct loft_span {
    int side { 1 };
    float halfwidth { 0.f };
    double texturelength { 1.0 };
    double from { 0.0 };
    double to { 0.0 };
    knots::line offsets;
    knots::line extents;
    knots::line inner;
    float lift { 0.f };
    gfx::userdata_array *userdata { nullptr };
    knots::line ramp;
    double rampwidth { 0.0 };
    bool walls { false };
    float wallu { -1.f };
    loft_end fromend;
    loft_end toend;
    knots::line drops;
    glm::dvec2 head { 0.0 };
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
    std::vector<segment_data> pieces; // consecutive parts of the path, which then is their approximation for older versions
// constructors
    segment_data() = default;
// methods
    void deserialize( cParser &Input, glm::dvec3 const &Offset );
    void translate( glm::dvec3 const &Offset );
    // maps the ends of the path and of its pieces with Point and their control vectors with Vector
    template <typename Pointmap, typename Vectormap>
    void transform( Pointmap const &Point, Vectormap const &Vector ) {
        for( auto const index : { point::start, point::end } ) {
            points[ index ] = Point( points[ index ] ); }
        for( auto const index : { point::control1, point::control2 } ) {
            points[ index ] = Vector( points[ index ] ); }
        for( auto &piece : pieces ) {
            piece.transform( Point, Vector ); } }
};

class TSegment
{ // aproksymacja toru (zwrotnica ma dwa takie, jeden z nich jest aktywny)
  private:
    glm::dvec3 Point1;
    glm::dvec3 CPointOut;
    glm::dvec3 CPointIn;
    glm::dvec3 Point2;
    float fRoll1 { 0.f };
    float fRoll2 { 0.f }; // przechyłka na końcach
    // how the path sits among the others
    struct setting {
        std::array<float, 2> designrolls { 0.f, 0.f }; // as entered; fRoll1 and fRoll2 can differ at a joint
        bool uneven { false }; // see TrackUneven.h
        int path { 0 }; // which path of a switch
        // where the path meets the next one: the middle of their ends and their common vector across
        std::array<std::optional<std::pair<glm::dvec3, glm::dvec3>>, 2> joints;
    };
    setting m_setting;
    double fLength { -1.0 }; // długość policzona
    std::vector<double> fTsBuffer; // wartości parametru krzywej t dla równych odcinków s
    std::vector<double> fTsSlope; // nachylenie dt/du (= dt/ds * fStep) w tych samych punktach
    double fStep = 0.0;
    int iSegCount = 0; // ilość odcinków do rysowania krzywej
    double fDirection = 0.0; // Ra: kąt prostego w planie; dla łuku kąt od Point1
    double fStoop = 0.0; // Ra: kąt wzniesienia; dla łuku od Point1
	glm::dvec3 vA;
	glm::dvec3 vB;
	glm::dvec3 vC; // współczynniki wielomianów trzeciego stopnia vD==Point1
    TTrack *pOwner = nullptr; // wskaźnik na właściciela
    std::vector<std::shared_ptr<TSegment>> m_pieces; // parts of a path made of several curves, parametrized by distance
    std::vector<double> m_starts; // distance from Point1 to the start of each part
    double m_dense { 0.0 };
    // a switch makes the profiles of its rails and its bed at the mean of the rolls of its paths: lofted along a path, they're turned
    // from it to the roll of the path there, so a switch on a cant ramp follows it; none for the other paths
    std::optional<float> m_baked_roll;

    glm::dvec3 GetFirstDerivative(double const fTime) const;
	double RombergIntegral(double const fA, double const fB) const;
	void InitTFromSInterpolateData();
	double GetTFromS(double const s) const;
	double FastGetTFromS(double const s) const;
	glm::dvec3 RaInterpolate(double const t) const;
	glm::dvec3 RaInterpolate0(double const t) const;
    std::size_t piece_at(double const s) const;
    // the vertex of a profile lofted at the point and across the path there, turned from the baked roll to the one of the path at the distance
    void roll_vertex( gfx::basic_vertex &Vertex, glm::vec3 const &Position, glm::vec3 const &Parallel, double const Distance ) const;

    // moves a lofted vertex with the unevenness of the track
    void settle_vertex( gfx::basic_vertex &Vertex, glm::dvec3 const &Origin, glm::vec3 const &Position, glm::vec3 const &Parallel, double const Distance ) const;
    // displacement of a point by the unevenness, at a distance along the path
    glm::dvec3 stray( glm::dvec3 const &Point, track_uneven::amounts const &Amount, double const Distance ) const;

public:
    bool bCurve = false;

    explicit TSegment(TTrack *owner);
    bool Init(glm::dvec3 NewPoint1, glm::dvec3 NewPoint2, double fNewStep, double fNewRoll1 = 0, double fNewRoll2 = 0);
	bool Init(glm::dvec3 &NewPoint1, glm::dvec3 NewCPointOut, glm::dvec3 NewCPointIn, glm::dvec3 &NewPoint2, double fNewStep, double fNewRoll1 = 0, double fNewRoll2 = 0, bool bIsCurve = true);
    bool InitChain(std::vector<std::shared_ptr<TSegment>> Pieces);
    inline
    std::size_t
        PieceCount() const {
            return m_pieces.size(); };
    inline
    double
        PieceStart( std::size_t const Piece ) const {
            return m_starts[ Piece ]; };
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
        FastGetDirection(double const fDistance, double const fOffset) const;
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
    // call before Init, it sets the density of the geometry
    void
        set_uneven( bool const Uneven, int const Path = 0 ) {
            m_setting.uneven = Uneven;
            m_setting.path = Path; }
    // a point of the path and the unit vector across it in the plan, towards the right rail profile; zero for a path never made
    void
        plan_frame( double const Distance, glm::dvec3 &Point, glm::dvec3 &Across ) const;
    float
        design_roll( int const End ) const {
            return m_setting.designrolls[ End ]; }
    void
        set_joint( int const End, std::optional<std::pair<glm::dvec3, glm::dvec3>> const &Joint, float const Roll ) {
            m_setting.joints[ End ] = Joint;
            ( End == 0 ? fRoll1 : fRoll2 ) = Roll; }
    // second derivative of the displacement along the path; times speed squared: acceleration of an axle
    glm::dvec3
        uneven_bend( double const Distance ) const;
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
            if( false == m_pieces.empty() ) {
                auto const piece { piece_at( Distance ) };
                return m_pieces[ piece ]->GetRoll( Distance - m_starts[ piece ] ); }
            return std::lerp( fRoll1, fRoll2, static_cast<float>(Distance / fLength) ); }
    inline
    void
        GetRolls(float &r1, float &r2) const {
            // pobranie przechyłek (do generowania trójkątów)
            r1 = fRoll1;
            r2 = fRoll2; }

    bool
        RenderLoft( gfx::vertex_array &Output, glm::dvec3 const &Origin, gfx::vertex_array const &ShapePoints, bool const Transition, double fTextureLength, double Texturescale = 1.0, int iSkip = 0, int iEnd = 0, std::pair<float, float> fOffsetX = {0.f, 0.f}, glm::vec3 **p = nullptr, bool bRender = true );
    // a transition profile lofted in Steps pieces of equal length, so two paths of different make-up line up piece by piece
    bool
        RenderLoftSteps( gfx::vertex_array &Output, glm::dvec3 const &Origin, gfx::vertex_array const &ShapePoints, double fTextureLength, int const Steps ) const;
    // the profile lofted from distance From to To, moved outwards by Offsets and cut on its outer side to Extents beyond the gauge line, both as (distance, value) knots
    bool
        RenderLoftSpan( gfx::vertex_array &Output, glm::dvec3 const &Origin, gfx::vertex_array const &ShapePoints, loft_span const &Span ) const;
    void
        set_dense( double const Distance ) {
            m_dense = Distance; }
    void
        set_baked_roll( std::optional<float> const Roll ) {
            m_baked_roll = Roll; }
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
	void
	    plan_bounds(glm::dvec2 &minimum, glm::dvec2 &maximum) const;

	glm::vec3
	    get_nearest_point(const glm::dvec3 &point, float quality = 1.0f) const;
};

//---------------------------------------------------------------------------
