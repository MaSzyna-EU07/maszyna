/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scene/eu7/eu7_types.h"

#include <glm/glm.hpp>

namespace simulation {
class state_serializer;
}

namespace scene::eu7 {

constexpr int kSectionStreamBootstrapRadiusKm { 4 };
constexpr int kSectionStreamGameplayRadiusKm { 4 };

void
init_section_stream(
    Eu7Module const &RootModule,
    std::string const &ResolvedPath,
    simulation::state_serializer &Serializer );

void
prime_section_stream( Eu7Module const &RootModule );

[[nodiscard]] glm::dvec3
resolve_stream_position();

// Kamera / pojazd moga byc w innej przestrzeni wspolrzednych niz PACK — uzyj kotwicy scenerii.
[[nodiscard]] glm::dvec3
resolve_section_stream_position( glm::dvec3 const &Hint );

// Opcjonalnie blokuje do zaladowania pierścienia bootstrap wokol pozycji (debug/narzedzia).
void
bootstrap_section_stream( glm::dvec3 const &WorldPosition );

void
update_section_stream( glm::dvec3 const &WorldPosition );

// Worker dekoduje sekcje; watek mapy wstawia po wskazniku do bufora (bez re-read pliku).
void
drain_section_stream(
    std::size_t MaxColdMeshes = 0,
    std::size_t MaxInstances = 0 );

[[nodiscard]] bool
section_stream_active();

[[nodiscard]] bool
section_stream_needs_bootstrap();

// Po wejsciu w symulacje: kolejkuje bootstrap bez blokowania petli renderu.
void
kick_section_stream_bootstrap();

// Blokujacy bootstrap (preload / narzedzia).
void
try_bootstrap_section_stream();

// Po zaladowaniu scenariusza: bootstrap + drain PACK przed wejsciem w jazde.
void
preload_section_stream( double MaxDrainMs = 300.0 );

// Wszystkie sekcje PACK w pierścieniu RadiusKm wokol pozycji wczytane i zaaplikowane.
[[nodiscard]] bool
section_stream_ready_around(
    glm::dvec3 const &WorldPosition,
    int RadiusKm = kSectionStreamBootstrapRadiusKm );

// ready_around stabilnie przez kilka klatek — bez migania overlay/sceny.
[[nodiscard]] bool
section_stream_presentable_around(
    glm::dvec3 const &WorldPosition,
    int RadiusKm = kSectionStreamBootstrapRadiusKm );

// Renderer/UI: trzymaj czarny ekran + overlay, nie rysuj swiata 3D.
// Jednorazowo przy starcie — po dismiss nie wraca przy streamingu w locie.
[[nodiscard]] bool
loading_screen_blocks_world(
    glm::dvec3 const &WorldPosition,
    int RadiusKm = kSectionStreamBootstrapRadiusKm );

[[nodiscard]] bool
loading_screen_dismissed();

void
dismiss_loading_screen();

[[nodiscard]] glm::dvec3
stream_loading_position();

// 0..1 — ile sekcji PACK z pierścienia juz siedzi w Region.
[[nodiscard]] float
section_stream_ring_progress(
    glm::dvec3 const &WorldPosition,
    int RadiusKm = kSectionStreamBootstrapRadiusKm );

void
reset_section_stream();

} // namespace scene::eu7
