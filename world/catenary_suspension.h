// Sieć trakcyjna: wpisy podwieszeń (wysięgniki, zwisy, podwieszenia na bramkach) z pliku dołączanego dyrektywą "catenary"
// oraz rejestr konstrukcji wsporczych. Kod niezależny od reszty symulatora (C++17, tylko biblioteka standardowa).
#pragma once
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace catenary {

// jeden wpis "node <max> <min> <nazwa> suspension ... endsuspension"
struct suspension {
    std::string name;                       // nazwa wpisu
    double      range_max { 0.0 };          // zasięg widoczności jak w innych wpisach node
    double      range_min { 0.0 };
    std::string support;                    // nazwa wpisu node model z konstrukcją wsporczą
    int         card { 0 };                 // numer karty katalogowej schematu, np. 1110; 0 = dobór automatyczny
    int         execution { 0 };            // wykonanie z karty
    std::vector<std::string> messenger;     // przęsła, których linę nośną trzyma (0–2)
    std::vector<std::string> contact;       // przęsła, których przewód jezdny trzyma (dowolna liczba)
    std::string insulators;                 // zestaw podmian izolatorów; pusty = domyślny
    std::vector<std::pair<std::string, std::string>> overrides; // nadpisania pojedynczych ról: rola, część
    std::string skin;                       // wymienna tekstura całego podwieszenia; pusta = domyślna z pliku
    // punkty podane wprost, we współrzędnych scenerii: wpis bez konstrukcji wsporczej i bez przęseł.
    // "at" to oś przewodów jezdnych przy podwieszeniu, "attach" to pion przez zaczepy izolatorów (wysokość bez znaczenia),
    // "rope" to oś liny nośnej; bez niej lina jest nad przewodami na wysokości konstrukcyjnej schematu
    bool   has_at { false };
    bool   has_attach { false };
    bool   has_rope { false };
    double at[ 3 ] { 0.0, 0.0, 0.0 };
    double attach[ 3 ] { 0.0, 0.0, 0.0 };
    double rope[ 3 ] { 0.0, 0.0, 0.0 };
    // słup z katalogu wysięgników zamiast "attach": typ i punkt w osi słupa na wysokości główki szyny.
    // zaczepy izolatorów, mocowania i sam słup liczy wtedy exe
    bool   has_pole { false };
    std::string pole_type;
    double pole[ 3 ] { 0.0, 0.0, 0.0 };
    // parasol: dwa wpisy ze słupem i tą samą nazwą parasola to dwa wysięgniki na jednym słupie, po przeciwnych stronach.
    // słup stawia pierwszy z nich; mocowania regulowane (z połówką z nakrętkami) idą na wspólną wysokość
    std::string umbrella;
};

// łańcuch sieci: kolejne przęsła między podwieszeniami z listy (od kotwy do kotwy), wpis
// "node <max> <min> <nazwa> chain ... suspensions <nazwy podwieszeń> endchain". Exe robi z niego zwykłe przęsła (TTraction):
// przęsło k idzie od podwieszenia k do k+1, przewód jezdny przez punkty "at", lina nośna przez punkty "rope"
struct chain {
    std::string name;                       // nazwa łańcucha; przęsła dostają nazwy <nazwa>_<numer od 1>
    double      range_max { -1.0 };
    double      range_min { 0.0 };
    std::string power;                      // zasilacz (jak w node traction)
    double      voltage { 3000.0 };         // napięcie nominalne, V
    double      current { 4500.0 };         // prąd maksymalny, A
    double      resistivity { 0.01 };       // jak w node traction (0,01 = wartość domyślna scenerii)
    std::string material { "cu" };          // cu, al
    double      thickness { 3.0 };          // grubość rysowanego drutu (rysowanie bez typu sieci)
    int         state { 0 };                // stan drutów: 0 nowe, 2 przejściowe, 1 zaśniedziałe
    std::string type;                       // symbol typu sieci, np. YwsC120-2C
    double      hangers[ 4 ] { 0.4, 4.0, 3.0, 0.04 }; // jak w node traction: najniższy wieszak, odstęp, liczba drutów, odstęp drutów
    std::string parallel;                   // przęsło bieżni wspólnej (jak "parallel" w node traction); puste = brak
    std::string anchor_start, anchor_end;   // kotwienia (wpisy "anchor") przed pierwszym i za ostatnim podwieszeniem; puste = brak
    std::vector<std::string> suspensions;   // nazwy podwieszeń z tego samego pliku, po kolei
};

// kotwienie ciężarowe (urządzenie naprężające z pliku traction_tensioners.txt) na słupie kotwowym, wpis
// "node <max> <min> <nazwa> anchor device <karta> network <symbol typu> pole <słup> x y z toward x y z distance <m>
//  [color <kolor>] [top_color <kolor>] [elongation <m>] [missing <n>] [ground <m>] [skin <tekstura>] endanchor".
// pole: oś słupa na wysokości główki szyny; toward: punkt, z którego przychodzą druty (urządzenie leży w tym kierunku od
// słupa); distance: odległość od kotwienia środkowego albo stałego. Łańcuch przypina się do niego "anchor_start/anchor_end",
// a ostatnie przęsło (bez wieszaków) kończy się na uchwytach krańcowych urządzenia
struct anchor {
    std::string name;
    double      range_max { 1000.0 };
    double      range_min { 0.0 };
    std::string device { "0461" };
    std::string network;                    // symbol typu sieci (traction_<symbol>.txt): numer typu i liczba przewodów jezdnych
    std::string pole_type;
    double      pole[ 3 ] { 0.0, 0.0, 0.0 };
    double      toward[ 3 ] { 0.0, 0.0, 0.0 };
    double      distance { 700.0 };
    std::string color, top_color;
    double      elongation { 0.0 };
    int         missing { 0 };
    double      ground { -0.50 };
    std::string skin;
};

// bramka: dwa słupy i dźwigar między nimi, wpis
// "node <max> <min> <nazwa> gantry beam <dźwigar> level low|high side 1|-1 mast <słup> x y z mast <słup> x y z endgantry".
// Punkty to osie słupów na wysokości główki szyny; dźwigar biegnie od pierwszego do drugiego, na prześwicie dolnego albo górnego
// mocowania pierwszego słupa, przesunięty wzdłuż toru o offset słupa na stronę "side". Zwisy z wysięgnikami to zwykłe podwieszenia
// ze "pole <zwis> x y z" (punkt w spodzie dźwigara).
// Pojedynczy element dźwigara (wspornik na jednym słupie przelotowym, np. 3017-3025): "mast <słup> x y z tip x y z" -
// masts[1] puste, points[1] to punkt w kierunku końca wspornika
struct gantry {
    std::string name;
    double      range_max { 1000.0 };
    double      range_min { 0.0 };
    std::string beam;
    bool        high { false };             // górne mocowanie dźwigara
    int         side { 1 };
    std::string masts[ 2 ];
    double      points[ 2 ][ 3 ] { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
    std::string skin;
};

// zawartość pliku podwieszeń
struct suspension_file {
    std::string skin;                       // domyślna wymienna tekstura dla wpisów bez własnej
    std::vector<suspension> entries;
    std::vector<chain> chains;
    std::vector<gantry> gantries;
    std::vector<anchor> anchors;
};

// rozbiera plik podwieszeń podany jako kolejne słowa (już bez komentarzy). Source: nazwa pliku do komunikatów.
// Błędne wpisy są pomijane, a opis każdego błędu trafia do Errors
void parse_suspensions( std::vector<std::string> const &Tokens, std::string const &Source, suspension_file &Output, std::vector<std::string> &Errors );

// wpis podwieszenia w postaci tekstu do pliku (jedna linia, bez znaku końca linii), w tej samej składni, którą czyta parse_suspensions.
// punkty z dokładnością do milimetra
std::string write_suspension( suspension const &Entry );
// wpis łańcucha w postaci tekstu do pliku (jedna linia)
std::string write_chain( chain const &Entry );
// wpis bramki w postaci tekstu do pliku (jedna linia)
std::string write_gantry( gantry const &Entry );
// wpis kotwienia w postaci tekstu do pliku (jedna linia)
std::string write_anchor( anchor const &Entry );

// dane konstrukcji wsporczej z rejestru
struct support_data {
    std::string type;                   // mast, gantry_mast, beam, drop
    double      height { 0.0 };         // m, wysokość nad punktem zerowym modelu (główka szyny)
    double      width_base { 0.0 };     // m, szerokość w kierunku toru u podstawy
    double      width_top { 0.0 };      // m, to samo na wysokości height
    // połowa szerokości na podanej wysokości: odległość czoła od osi konstrukcji
    double half_width( double Height ) const;
};

// rejestr konstrukcji wsporczych: dane czytane z pliku, szukane po ścieżce modelu
class support_registry {
public:
    // wczytuje plik; błędy trafiają do Errors. zwraca false, gdy pliku nie da się otworzyć
    bool load( std::string const &Filename, std::vector<std::string> &Errors );
    // dane konstrukcji o podanej ścieżce modelu (wielkość liter i rodzaj ukośników bez znaczenia); nullptr gdy brak
    support_data const *find( std::string const &Model ) const;
    std::size_t size() const { return m_supports.size(); }

private:
    std::map<std::string, support_data> m_supports;
};

} // namespace catenary
