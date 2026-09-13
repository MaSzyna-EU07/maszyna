/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <string>
#include <vector>


export module eu07.editor.turnouts;

export {

namespace editor
{

// one piece of a preset's diverging path, in catalogue order from PR. mirrors
// maj0sted::editor::TurnoutPieceSpec so this header stays free of the library
struct turnout_preset_piece
{
	int part;            // 0 przediglicowy, 1 iglica, 2 luk, 3 krzyzownicowa
	double length;       // metres, always explicit - nothing here is fitted
	double radius_start; // metres; 0 = straight
	double radius_end;   // metres; different from radius_start = clothoid
	double turn_in;      // kat nagiecia iglicy (beta), radians; 0 = tangent to the stock rail
};

// a standard turnout template: its designation, the catalogue numbers the laid path is
// checked against (skos 1:n and the length PR->KR along the through track), the pieces it
// is laid from, and the two blade dimensions that place its real, cut tip
struct turnout_preset
{
	std::string name;
	double crossing_n;    // skos 1:n
	double length;        // catalogue length: a + b, the two tangent legs of the drawing, metres
	double tip_thickness; // grubosc iglicy na koncu zestruganego dzioba (u), metres
	double nose;          // dziob iglicy: zestrugana dlugosc od ostrza (d), metres
	double railtop_width; // szerokosc glowki szyny (w), metres
	std::vector<turnout_preset_piece> pieces;
};

// typical Polish turnouts (rozjazdy zwyczajne). the (skos, R) pairs are the recognised
// standard; confirm the piece lengths and the blade against Id-1
std::vector<turnout_preset> const &turnout_presets();

} // namespace editor

}  // export
