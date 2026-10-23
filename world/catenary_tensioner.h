/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Sieć trakcyjna: urządzenie naprężające (kotwienie ciężarowe), Katalog ESN tom 1 dział 0400 (karta 23-0461 z Uzupełnienia
// nr 2, dawniej 63-0401) i tablica montażowa 33-0011-1. Dane w pliku traction_tensioners.txt.
// Kod niezależny od reszty symulatora (C++17, tylko biblioteka standardowa).
//
// Układ urządzenia: zero na czole konstrukcji wsporczej (od strony kotwionego przęsła), na wysokości główki szyny;
// X poziomo od konstrukcji w stronę kotwionego przęsła (wzdłuż kierunku drutów), Z w górę, Y = Z x X (w lewo patrząc
// wzdłuż X). Wymiary w metrach.
//
// Budowa (rolki równoległe, przekładnia 1:4): na konstrukcji uchwyt przegubowy 4450 i uchwyt 4440 trzymają rolkę stałą
// (podwójną 4460-1), w odległości b od czoła wisi rolka ruchoma (też podwójna), od niej uchwyt 4440 do osi dźwigni 6406.
// Do otworów dźwigni idą łańcuchy części do liny nośnej i do przewodów jezdnych (łącznik, izolator, łącznik, śruba
// rzymska, wyrównywacz, uchwyty krańcowe). Lina stalowa (cięgło 4490): koniec zaczepiony przy rolce stałej, cztery żyły
// między rolkami (rowek A, potem B), wolny koniec z rolki stałej pionowo w dół do pręta ciężarów; na pręcie stos
// ciężarów, którego spód wisi s nad główką szyny. b i s z tablicy montażowej (temperatura, odległość od kotwienia
// stałego albo środkowego). ZAŁOŻENIE przebiegu liny (Jan 2026-10-23: do sprawdzenia na zdjęciach).
//
// Ułożenie modeli (w układzie części):
//   rolka 4460          zero w osi, koło w płaszczyźnie X-Z, oś wzdłuż Y
//   części liniowe      (uchwyty 4440/4450, łączniki, izolatory, śruby, wyrównywacz, uchwyty krańcowe, łączniki na linie)
//                       zero w sworzniu od strony konstrukcji/dźwigni, biegną w +X do drugiego sworznia (długość z danych)
//   dźwignia 6406       zero w otworze I, otwory w +Z (I na dole), otwór III odsunięty w +X; przy otworze liny poniżej
//                       otworu przewodów dźwignia odwrócona (obrót o 180 stopni wokół Y)
//   pręt 6401/6402      zero na dole (pod stosem), w +Z do zaczepu na górze (długość z danych)
//   ciężar 63xx         zero w środku spodu, w +Z; kolejne obracane o "turn" wokół Z; kolor numerem wykonania modelu
//   lina                linka rozciągana (część typu cord, zero w zaczepie, w +Z, skalowana wzdłuż Z)
#pragma once
#include <map>
#include <string>
#include <vector>

namespace catenary {

// tablica montażowa urządzenia naprężającego: b (odległość osi rolki ruchomej od czoła konstrukcji) i s (wysokość spodu
// ciężarów nad główką szyny) w funkcji temperatury i odległości od kotwienia stałego/środkowego
struct tensioner_table {
    std::string name;
    double height { 6.70 };                 // oś rolek nad główką szyny
    double rod { 2.00 };                    // długość pręta ciężarów, dla której liczono tablicę
    std::vector<double> temperatures;       // °C, rosnąco
    std::vector<double> distances;          // m, rosnąco
    std::vector<std::vector<double>> b, s;  // [odległość][temperatura]
    // wzór z karty 13-0011 (stary katalog): b = bn - as·α·(t - tn), s = sn - 4·as·α·(t - tn); gdy podany, liczy się z niego
    // (wiersze tablicy zostają do porównania), bez przycinania do zakresu
    bool formula { false };
    double bn { 1.75 }, sn { 2.25 }, tn { 10.0 }, alpha { 17e-6 };
    // ze wzoru, niezależnie od tego, czy tablica go używa
    void formula_values( double Temperature, double Distance, double &B, double &S ) const;
    // wartości dla temperatury i odległości (interpolacja dwuliniowa, poza zakresem przycięte do brzegu);
    // zwraca false, gdy którakolwiek wartość była poza zakresem tablicy
    bool lookup( double Temperature, double Distance, double &B, double &S ) const;
};

struct tensioner_weight {
    std::string name;
    double height { 0.055 };
    double mass { 27.0 };                   // kg
    double turn { 0.0 };                    // stopnie: ciężar na n-tym miejscu stosu (od 0) obrócony o n·turn wokół pionu (wszystkie 90)
    std::string model;                      // rdzeń nazwy modelu, np. 6321: model "<rdzeń>-<NN>", NN = offset + numer koloru
    int offset { 0 };                       // 6322: 0 niskie (56 mm), 4 wysokie (92 mm) - dwie wysokości produkowane (Jan)
};

// numer koloru ciężaru w modelach (wykonania 1-4, Jan 2026-10-23): 1 żółty, 2 szary ładny, 3 żółty pędzlowany, 4 szary brzydki;
// przyjmuje nazwę (zolty, szary, zolty_pedzel, szary_brzydki) albo cyfrę; 0 gdy nieznany
int tensioner_color( std::string const &Name );

struct tensioner_pulley {
    std::string name;
    double diameter { 0.220 };              // po dnie rowka
    double width { 0.080 };
    int grooves { 2 };
};

struct tensioner_lever {
    std::string name;
    std::map<std::string, std::pair<double, double>> holes; // nazwa otworu -> (wzdłuż osi od otworu I, odsunięcie w bok)
};

// część liniowa w łańcuchu
struct tensioner_link {
    std::string name;
    double length { 0.0 };
};

// stos ciężarów i otwory dźwigni dla typu sieci (numer typu z karty 0002 = traction_code)
struct tensioner_type {
    int code { -1 };
    std::vector<std::pair<std::string, int>> stack;   // od dołu: ciężar, liczba
    std::string messenger_hole, contact_hole, axis_hole;
    std::string rod;                    // pręt, gdy inny niż urządzenia
    std::string messenger_clamp;        // uchwyt krańcowy liny nośnej, gdy inny niż urządzenia
};

struct tensioner_device {
    std::string name;                   // karta, np. 0461
    std::string table, pulley, lever, rod, rope;
    double rope_diameter { 0.010 };
    std::vector<tensioner_link> mount;  // od czoła konstrukcji do osi rolki stałej
    std::vector<tensioner_link> holder; // od osi rolki ruchomej do osi dźwigni
    std::vector<tensioner_link> hanger; // od rolki stałej w dół do pręta (bez liny)
    std::vector<tensioner_link> messenger; // od otworu dźwigni do uchwytu krańcowego liny nośnej (bez uchwytu)
    std::vector<tensioner_link> contact;   // od otworu dźwigni do wyrównywacza albo uchwytu krańcowego przewodu
    tensioner_link messenger_clamp, contact_clamp;
    tensioner_link equalizer;           // wyrównywacz przewodów jezdnych (tylko przy dwóch przewodach)
    double equalizer_spread { 0.08 };   // rozstaw przewodów za wyrównywaczem
    std::vector<tensioner_type> types;
    tensioner_type const *type( int Code ) const;
};

class tensioner_catalog {
public:
    bool load( std::string const &Filename, std::vector<std::string> &Errors );
    void parse( std::vector<std::string> const &Lines, std::string const &Source, std::vector<std::string> &Errors );
    tensioner_table const *table( std::string const &Name ) const;
    tensioner_weight const *weight( std::string const &Name ) const;
    tensioner_pulley const *pulley( std::string const &Name ) const;
    tensioner_lever const *lever( std::string const &Name ) const;
    tensioner_device const *device( std::string const &Name ) const;
    std::vector<tensioner_device> const &devices() const { return m_devices; }
    std::map<std::string, double> const &part_lengths() const { return m_lengths; } // długości części liniowych (pręt)

private:
    std::vector<tensioner_table> m_tables;
    std::vector<tensioner_weight> m_weights;
    std::vector<tensioner_pulley> m_pulleys;
    std::vector<tensioner_lever> m_levers;
    std::vector<tensioner_device> m_devices;
    std::map<std::string, double> m_lengths;
};

struct tensioner_input {
    int type { 3 };                     // numer typu sieci (traction_code)
    int contacts { 2 };                 // liczba przewodów jezdnych
    double temperature { 10.0 };        // °C
    double distance { 700.0 };          // m, od kotwienia stałego albo środkowego
    double elongation { 0.0 };          // m, trwałe wydłużenie sieci od ostatniej regulacji (rolka bliżej konstrukcji)
    int missing { 0 };                  // brakujące ciężary (z góry stosu)
    std::string color;                  // kolor ciężarów (tensioner_color); pusty = szary
    std::string top_color;              // kolor najwyższego ciężaru; pusty = jak reszta
    double ground { -0.50 };            // najniższe położenie spodu ciężarów względem główki szyny (grunt, fundament); ZAŁOŻENIE
    double gravity { 0.981 };           // daN/kg
};

struct tensioner_placement {
    std::string role;
    std::string part;
    double position[ 3 ] { 0.0, 0.0, 0.0 };
    double rotation[ 9 ] { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };  // wierszami, kolumny to osie części
    double scale[ 3 ] { 1.0, 1.0, 1.0 };
};

struct tensioner_result {
    std::vector<tensioner_placement> parts;
    std::vector<std::vector<double>> rope;  // punkty liny (x, y, z) po kolei, od zaczepu przy rolce stałej do pręta
    double b { 0.0 }, s { 0.0 };            // położenie rolki ruchomej i spodu ciężarów
    double stack_height { 0.0 };
    int weights { 0 };                      // zawieszone ciężary
    double tension { 0.0 };                 // daN, całkowity naciąg sieci od ciężarów (zero, gdy ciężary stoją na gruncie)
    double messenger_tension { 0.0 }, contact_tension { 0.0 };
    double messenger_end[ 3 ] { 0.0, 0.0, 0.0 };         // gdzie kończy się lina nośna (uchwyt krańcowy)
    std::vector<std::vector<double>> contact_ends;       // gdzie kończą się przewody jezdne
    bool grounded { false };                // ciężary oparte o grunt: sieć bez naciągu od urządzenia
    std::vector<std::string> warnings;
};

// składa urządzenie. false, gdy brakuje danych (urządzenia, typu, tablicy, rolki, dźwigni, otworów)
bool solve_tensioner( tensioner_catalog const &Catalog, tensioner_device const &Device, tensioner_input const &Input, tensioner_result &Result );

// lina jako odcinki linki rozciąganej (część Cord o długości spoczynkowej Length, skala w poprzek Girth)
void tensioner_rope_parts( tensioner_result &Result, std::string const &Cord, double Length, double Girth );

} // namespace catenary
