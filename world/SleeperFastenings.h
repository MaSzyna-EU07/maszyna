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
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "world/TrackSleepers.h"

// the fastenings of the rails on the timbers: where a rail lies on a timber, the part of the model of the timbers which holds it
// there. a stock rail with the blade beside it lies in a slide chair, a rail with a check rail or another rail of the frog close
// beside it in a fastening of both, the blade on the chair of its stock rail, and the other rails each in a plate of its own
namespace track_sleepers {

// a fastening on a timber
struct fastening {
    int part { part_plate };
    glm::dvec3 point { 0.0 }; // under the middle of the rail, on the top of the timber
    glm::dvec2 across { 1.0, 0.0 }; // unit vector in plan square to the rail, the way the +x of the part goes
    float scale { 1.f }; // along its x, a plate or a slide plate narrowed where it'd lie over the fastenings of rails close beside
    float pivot { 0.f }; // the x of the part kept in place as it's narrowed
    float shift { 0.f }; // along its x, how far the pivot is moved
    // where an x of the part lands along its across, at the scale
    double placed( double const X, double const Scale ) const {
        return pivot + shift + Scale * ( X - pivot ); }
};

// the rails of the tracks around a track, which lie on its timbers
class fastened_rails {

public:
    explicit fastened_rails( TTrack &Track );
    // the fastenings of the rails on the timber, of the parts the model has
    std::vector<fastening> on( beam const &Beam, beam_model const &Model ) const;

private:
    // what a rail is where it lies on a timber
    enum class role {
        plain,
        blade, // a blade of a switch, which slides on the chair of its stock rail
        stock, // the stock rail of a switch with the blade beside it
        checked // a rail with a check rail beside it
    };
    // a rail by points along its middle
    struct rail {
        TTrack *track { nullptr };
        int path { 0 };
        int side { 1 };
        std::vector<glm::dvec2> points;
        std::vector<double> heights;
        std::vector<glm::dvec2> inward; // unit vectors in plan from the rail toward the middle of its path
        std::vector<double> distances;
        glm::dvec2 low { 0.0 };
        glm::dvec2 high { 0.0 };
        double foot { 0.0 }; // from the middle of the rail to the edge of its foot toward the middle of the path
        // the stretches along the path the rail is a blade, a stock rail with a chair, checked along
        glm::dvec2 blade { -1.0 };
        glm::dvec2 stock { -1.0 };
        std::vector<glm::dvec2> checks;
        role at( double const Distance ) const;
    };
    // where a rail crosses a timber
    struct hit {
        double at { 0.0 }; // along the timber, from its from end
        role kind { role::plain };
        glm::dvec2 inward { 1.0, 0.0 };
        double foot { 0.0 };
    };

    // the fastenings of rails close by fitted to the room between them: narrowed across, and a rail lying on the fastening of the
    // rail beside it left without one of its own
    static void fit( std::vector<fastening> &Fastenings, beam const &Beam, beam_model const &Model );

    std::vector<rail> m_rails;
};

// the fastenings of a track with them in the model of its timbers, as laid on the timbers laid with it
struct fastening_row {
    std::string track;
    std::size_t beams { 0 };
    std::array<std::size_t, part_count> parts {};
    std::size_t overlaps { 0 }; // fastenings lying over one another on a timber, along it
    double worst { 0.0 };
    glm::dvec3 worstat { 0.0 };
    std::string worstparts;
    struct overlap {
        int parts[ 2 ];
        double apart; // of the rails the two lie under
        double over;
        glm::dvec3 at;
    };
    std::vector<overlap> cases;
};
// those of all the tracks laying timbers with fastenings
std::vector<fastening_row> check_fastenings();

} // namespace track_sleepers
