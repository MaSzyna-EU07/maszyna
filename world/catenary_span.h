/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Przęsło sieci trakcyjnej rysowane wg typu sieci: powiązanie TTraction z katalogiem typów i solverem.
// Cały stan potrzebny do tego trzymany jest w strukturze catenary_span, do której TTraction ma tylko wskaźnik,
// żeby zmiany w tej części nie wymagały ruszania nagłówka Traction.h

#pragma once

#include <string>

#include "rendering/geometrybank.h"

class TTraction;

// ustawienia rysowania sieci, zmieniane z eu07.ini i z panelu ustawień
struct catenary_settings {
    float solid_range { 150.f }; // zasięg rysowania przewodów, lin i uchwytów jako brył [m]; 0 wyłącza bryły
};
extern catenary_settings CatenarySettings;

// przypisuje przęsłu typ sieci z dopisku "type" i dobiera rozstaw wieszaków; wołane po wczytaniu punktów przęsła.
// pusty symbol albo wyłączone rysowanie wg typu zostawia przęsło rysowane po staremu
void catenary_span_assign( TTraction &Traction, std::string const &Symbol );
// symbol typu sieci przypisany przęsłu; pusty gdy brak
std::string catenary_span_symbol( TTraction const &Traction );
// odcinki przęsła policzone wg typu sieci, dopisywane do Vertices; false gdy przęsło ma być narysowane po staremu
bool catenary_span_lines( TTraction &Traction, gfx::vertex_array &Vertices );
// bryły przęsła do oglądania z bliska: przewody i liny w prawdziwej grubości oraz osprzęt (bryły zastępcze w miejscach połączeń).
// osobno, bo rysowane są z różnych materiałów; pusty uchwyt oznacza, że nie ma czego rysować
struct catenary_solid {
    gfx::geometry_handle wires;
    gfx::geometry_handle fittings;
};
// bryły tworzone są, gdy kamera jest blisko przęsła, i zwalniane, gdy się oddali
catenary_solid catenary_span_solid( TTraction &Traction, glm::dvec3 const &Camera );
// przęsło po zmianie sąsiadów (np. w edytorze): linie liczone od nowa, bryły zrobią się od nowa
void catenary_span_refresh( TTraction &Traction );
// zwalnia linie i bryły przęsła przed jego usunięciem
void catenary_span_release( TTraction &Traction );

// wygląd metalu brył: kolor własny (sRGB), metaliczność, połysk i siła odbicia otoczenia, każde w zakresie 0–1
struct catenary_metal {
    float color[ 3 ];
    float metalness;
    float gloss;
    float reflection;
};
// wygląd przewodów (Fitting = false) albo osprzętu przęsła o podanym materiale i stanie z wpisu scenerii.
// wartości domyślne można zmienić bez kompilacji plikiem data/traction_materials.txt, czytanym przy pierwszym użyciu
catenary_metal const &catenary_span_metal( TTraction const &Traction, bool Fitting );
