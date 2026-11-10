// Solver geometrii przęsła sieci trakcyjnej (lina nośna, podwieszenie Y, wieszaki).
// Kod samodzielny: C++17, bez zależności od symulatora. Jednostki SI (m, kg, N).
// Model: lina jako cięgno bez sztywności, obciążone ciężarem własnym i siłami od wieszaków;
// przewód jezdny poziomy, jego ciężar rozłożony na wieszaki po połowie odcinków sąsiednich.
#pragma once
#include <string>
#include <utility>
#include <vector>

namespace catenary {

constexpr double gravity = 9.80665;

// opis typu sieci (jeden wpis katalogu typów)
struct network_type {
    std::string symbol;                 // np. "YwsC120-2C"
    int         code { 0 };             // kod sieci wg karty 0002
    int         contact_count { 2 };    // liczba przewodów jezdnych
    double      contact_mass { 0.890 }; // kg/m jednego przewodu jezdnego
    double      contact_tension { 0.0 }; // N, naciąg łączny przewodów jezdnych; 0 = nieznany
    int         messenger_count { 1 };  // liczba lin nośnych
    double      messenger_mass { 1.060 };   // kg/m jednej liny
    double      messenger_tension { 0.0 };  // N, naciąg łączny lin nośnych
    double      y_tension { 0.0 };      // N, naciąg linki Y (dla sieci z dwiema linami: naciąg dolnej liny)
    double      y_mass { 0.0 };         // kg/m linki Y
    double      structure_height { 1.70 };  // m, wysokość konstrukcyjna przy podwieszeniu
    double      hanger_mass { 0.0 };    // kg, masa wieszaka z uchwytami
    double      messenger_stack { 0.0 };// m, odstęp pionowy lin przy dwóch linach (wieszaki wiszą na dolnej)
    double      y_junction_drop { 0.0 };// m, o ile zacisk linki Y (dolnej liny) leży poniżej górnej liny nośnej
    double      y_hanger_drop { 0.0 };  // m, korekta długości wieszaków linki Y (dopasowana do tablic, nie z katalogu)
    // długość podwieszenia Y zależna od rozpiętości: pary (minimalna rozpiętość, Ly), malejąco wg rozpiętości
    std::vector<std::pair<double, double>> y_length_by_span;

    double y_length( double Span ) const;
};

struct span_input {
    double              length { 0.0 };         // m, rozpiętość przęsła
    std::vector<double> hanger_gaps;            // m, odstępy: oś podwieszenia–1, 1–2, …, ostatni–oś podwieszenia
    double              height_start { -1.0 };  // m, wysokość liny nad przewodem przy pierwszym podwieszeniu (<0: z typu)
    double              height_end { -1.0 };    // m, to samo przy drugim podwieszeniu
    bool                y_at_start { true };    // czy przy pierwszym podwieszeniu jest linka Y (gdy typ ją przewiduje)
    bool                y_at_end { true };      // to samo przy drugim podwieszeniu
};

struct hanger {
    double x { 0.0 };       // m od pierwszego podwieszenia
    double length { 0.0 };  // m, od przewodu jezdnego do liny, na której wisi
    bool   on_y { false };  // wisi na lince Y
};

struct point2 { double x; double y; };  // x wzdłuż przęsła, y nad przewodem jezdnym

// połowa linki Y po jednej stronie podwieszenia: dane potrzebne do policzenia całej linki razem z sąsiednim przęsłem
struct y_half {
    double reach { 0.0 };       // m, odległość zacisku na linie nośnej od osi podwieszenia; 0 = brak linki Y
    double junction { 0.0 };    // m, wysokość zacisku nad przewodem jezdnym
    std::vector<std::pair<double, double>> loads;   // wieszaki: odległość od osi podwieszenia [m], siła [N]
};

struct span_result {
    std::vector<hanger> hangers;
    std::vector<point2> messenger;      // górna lina nośna, od podwieszenia do podwieszenia
    std::vector<point2> y_start;        // linka Y przy pierwszym podwieszeniu (od osi podwieszenia do zacisku)
    std::vector<point2> y_end;          // linka Y przy drugim podwieszeniu (od zacisku do osi podwieszenia)
    double              y_length { 0.0 };   // użyta długość Ly
    double              sag { 0.0 };        // największe obniżenie liny względem cięciwy
    y_half              y_half_start;       // połowy linki Y przy obu podwieszeniach
    y_half              y_half_end;
    // przewód jezdny względem prostej łączącej jego końce (y <= 0), liczony tylko w przęśle bez wieszaków; pusty = prosty
    std::vector<point2> contact;
};

// liczy geometrię przęsła. Rzuca std::invalid_argument przy sprzecznych danych
// Linka Y przy każdym podwieszeniu jest tu liczona tak, jakby sąsiednie przęsło było lustrzanym odbiciem
span_result solve_span( network_type const &Type, span_input const &Input, double Samplestep = 1.0 );

// przęsło bez wieszaków (odcinek do kotwienia albo urządzenia naprężającego): liny wiszą tylko pod własnym ciężarem.
// Heightstart, Heightend: wysokość liny nad przewodem jezdnym na końcach
span_result solve_free_span( network_type const &Type, double Length, double Heightstart, double Heightend, double Samplestep = 2.0 );

// przelicza linkę Y przy jednym podwieszeniu jako jedno cięgno od zacisku w tym przęśle do zacisku w sąsiednim
// (linka przechodzi swobodnie pod podwieszeniem). Poprawia przebieg linki i długości jej wieszaków w Span.
// Obie strony liczone z tych samych danych dają tę samą wysokość w osi podwieszenia, więc połówki się spotykają.
// Atend: podwieszenie na końcu przęsła; Neighbour: połowa linki z sąsiedniego przęsła przy tym samym podwieszeniu
void join_y( network_type const &Type, span_result &Span, double Length, bool Atend, y_half const &Neighbour );

} // namespace catenary
