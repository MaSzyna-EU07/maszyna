/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <memory>

#include "scene/scenenode.h"
#include "world/Segment.h"
#include "model/material.h"
#include "utilities/Names.h"
#include "world/TractionTail.h"

class TTractionPowerSource;
namespace gfx
{
class scene_walk;
}
struct catenary_span;

class TTraction : public scene::basic_node
{
    friend opengl46_renderer;
    friend gfx::scene_walk;

  public: // na razie
    TTractionPowerSource *psPower[ 2 ] { nullptr, nullptr }; // najbliższe zasilacze z obu kierunków
    TTractionPowerSource *psPowered { nullptr }; // ustawione tylko dla bezpośrednio zasilanego przęsła
    TTraction *hvNext[ 2 ] { nullptr, nullptr }; //łączenie drutów w sieć
    int iNext[ 2 ] { 0, 0 }; // do którego końca się łączy
    int iLast { 0 }; //że niby ostatni drut // ustawiony bit 0, jeśli jest ostatnim drutem w sekcji; bit1 - przedostatni
    glm::dvec3 pPoint1;
    glm::dvec3 pPoint2;
    glm::dvec3 pPoint3;
    glm::dvec3 pPoint4;
    glm::dvec3 vParametric; // współczynniki równania parametrycznego odcinka
    double fHeightDifference { 0.0 };
    int iNumSections { 0 };
    float NominalVoltage { 0.0f };
    float MaxCurrent { 0.0f };
    float fResistivity { 0.0f }; //[om/m], przeliczone z [om/km]
    unsigned int Material { 0 }; // 1: Cu, 2: Al
    float WireThickness { 0.0f };
    unsigned int DamageFlag { 0 }; // 1: zasniedziale, 128: zerwana
    int Wires { 2 };
    float WireOffset { 0.0f };
    std::string asPowerSupplyName; // McZapkie: nazwa podstacji trakcyjnej
    TTractionPowerSource *psSection { nullptr }; // zasilacz (opcjonalnie może to być pulpit sterujący EL2 w hali!)
    std::string asParallel; // nazwa przęsła, z którym może być bieżnia wspólna
    TTraction *hvParallel { nullptr }; // jednokierunkowa i zapętlona lista przęseł ewentualnej bieżni wspólnej
    float fResistance[ 2 ] { -1.0f, -1.0f }; // rezystancja zastępcza do punktu zasilania (0: przęsło zasilane, <0: do policzenia)
    int iTries { 1 }; // 0 is used later down the road to mark directly powered pieces
    int PowerState { 0 }; // type of incoming power, if any
    // visualization data
    glm::dvec3 m_origin;
    gfx::geometry_handle m_geometry;
    // stan przęsła rysowanego wg typu sieci (dopisek "type"); pusty = przęsło rysowane po staremu. szczegóły w world/catenary_span.h
    std::shared_ptr<catenary_span> m_catenaryspan;
    // dopiski wpisu "tension" i "rope2" w postaci wczytanej, zapisywane z powrotem przez export_as_text_
    traction_tail m_tail;

    explicit TTraction( scene::node_data const &Nodedata );

    void Load( cParser *parser, glm::dvec3 const &pOrigin );
    // set origin point
    void
        origin( glm::dvec3 Origin ) {
            m_origin = Origin; }
    // retrieves list of the track's end points
    std::vector<glm::dvec3>
        endpoints() const;
    // creates geometry data in specified geometry bank. returns: number of created elements, or NULL
    // NOTE: deleting nodes doesn't currently release geometry data owned by the node. TODO: implement erasing individual geometry chunks and banks
    std::size_t create_geometry( gfx::geometrybank_handle const &Bank );
    // the wind blowing the wires of the span aside (physics.weather.wirewind, weather_physics::wire_sway()): the horizontal unit
    // across the span, how far the middle of the span is blown along it now (m, the renderers draw it so), and where the wire at
    // Parameter (0..1 along the span, as in pPoint1 + Parameter * vParametric) is blown to (the pantographs meet it there)
    glm::dvec3 sway_axis() const;
    double sway() const;
    glm::dvec3 sway_at( double const Parameter ) const;
    // the lines (Lines) or the triangles of the drawn span made ready to be blown aside by traction.vert: the place of each vertex
    // along the span in its texture x, the long lines cut into pieces short enough to bend
    void sway_vertices( gfx::vertex_array &Vertices, bool const Lines ) const;
    int TestPoint(glm::dvec3 const &Point) const;
    void Connect(int my, TTraction *with, int to);
    void Init();
    bool WhereIs();
    void ResistanceCalc(int d = -1, double r = 0, TTractionPowerSource *ps = nullptr);
    void PowerSet(TTractionPowerSource *ps);
    double VoltageGet(double u, double i);

private:
// methods
    glm::vec3 wire_color() const;
    // radius() subclass details, calculates node's bounding radius
    float radius_() const override;
    // serialize() subclass details, sends content of the subclass to provided stream
    void serialize_( std::ostream &Output ) const override;
    // deserialize() subclass details, restores content of the subclass from provided stream
    void deserialize_( std::istream &Input ) override;
    // export() subclass details, sends basic content of the class in legacy (text) format to provided stream
    void export_as_text_( std::ostream &Output ) const override;

};



// collection of virtual tracks and roads present in the scene
class traction_table : public basic_table<TTraction> {

public:
    // legacy method, initializes traction after deserialization from scenario file
    void
        InitTraction();
};

//---------------------------------------------------------------------------
