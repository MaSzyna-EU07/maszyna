/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#ifndef TrkFollH
#define TrkFollH

#include "utilities/Classes.h"
#include "world/Segment.h"
#include "world/switchfork.h"

// oś poruszająca się po torze
class TTrackFollower {

public:
// constructors
    TTrackFollower() = default;
// destructor
    ~TTrackFollower();
// methods
    TTrack * SetCurrentTrack(TTrack *pTrack, int end);
    bool Move(double fDistance, bool bPrimary);
    inline TTrack * GetTrack() const {
        return pCurrentTrack; };
    // przechyłka policzona przy ustalaniu pozycji
    inline double GetRoll() {
        return vAngles.x; };
    //{return pCurrentSegment->GetRoll(fCurrentDistance)*fDirection;}; //zamiast liczyć można pobrać
    // zwrot na torze
    inline double GetDirection() const {
        return fDirection; };
    // ABu-030403
    inline double GetTranslation() const {
        return fCurrentDistance; };
    // inline double GetLength(vector3 p1, vector3 cp1, vector3 cp2, vector3 p2)
    //{ return pCurrentSegment->ComputeLength(p1,cp1,cp2,p2); };
    // inline double GetRadius(double L, double d);  //McZapkie-150503
    inline glm::dvec3 UnevenBend() const {
        return pCurrentSegment ? pCurrentSegment->uneven_bend( fCurrentDistance ) : glm::dvec3 { 0.0 }; };
    // TTrack::rail_phase of the point Offset ahead of the axle along the vehicle
    double RailPhase( double const Offset ) const;
    // TTrack::buckle_curvature at the axle, averaged over Base around it
    double BuckleCurvature( double const Base ) const;
    // on a switch: the way the axle ran onto, which it keeps whichever way the switch is thrown under it (-1 off a switch)
    inline int GetPath() const {
        return iPath; };
    // the way the axle ran over the switch the last time it did, none if it isn't remembered
    inline std::optional<int> PathOf( TTrack const *Switch ) const {
        return m_paths.path_of( Switch ); };
    // the switch the axle ran onto from its points since the last call, none: the vehicle looks whether the axle ahead
    // of it took the same way (switch_fork::forked())
    TTrack *TakeFacingEntry();
    bool Init(TTrack *pTrack, TDynamicObject *NewOwner, double fDir);
	void Reset();
    void Render(float fNr);
// members
    double fOffsetH = 0.0; // Ra: odległość środka osi od osi toru (dla samochodów) - użyć do wężykowania
	glm::dvec3 pPosition; // współrzędne XYZ w układzie scenerii
	glm::vec3 vAngles; // x:przechyłka, y:pochylenie, z:kierunek w planie (w radianach)
  private:
// methods
    bool ComputatePosition(); // przeliczenie pozycji na torze
    // the code of the end of the next track joined to the End of the current one: on a switch thrown since the axle ran
    // onto it, of the end of the way the axle is on
    int JoinedEnd( int const End ) const;
    // the way to leave the current switch by, -1 as it is set (switch_fork::leaving_path())
    int LeavingPath() const;
// members
    TTrack *pCurrentTrack = nullptr; // na którym torze siê znajduje
	std::shared_ptr<TSegment> pCurrentSegment; // zwrotnice mog¹ mieæ dwa segmenty
	double fCurrentDistance = 0.0; // przesuniêcie wzglêdem Point1 w stronê Point2
    double fDirection = 1.0; // ustawienie wzglêdem toru: -1.0 albo 1.0, mno¿one przez dystans // jest przodem do Point2
    TDynamicObject *Owner = nullptr; // pojazd posiadający
    int iEventFlag = 0; // McZapkie-020602: informacja o tym czy wyzwalac zdarzenie: 0,1,2,3
    int iEventallFlag = 0;
    int iSegment = 0; // który segment toru jest używany (żeby nie przeskakiwało po przestawieniu zwrotnicy pod taborem)
    int iPath = -1; // the way of the switch the axle ran onto, -1 off a switch
    switch_fork::path_memory m_paths; // the ways of the switches it ran over last
    TTrack *m_facingentry = nullptr; // the switch it ran onto from the points, until the vehicle takes it
};
//---------------------------------------------------------------------------
#endif
