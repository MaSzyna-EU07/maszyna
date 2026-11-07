// Sieć trakcyjna: zasady rozstawiania konstrukcji wsporczych i doboru podwieszeń z katalogu (Kolprojekt, karty 0002, 0007, 0008,
// stary katalog 13-0002, 13-0005, 13-0008). Kod niezależny od reszty symulatora (C++17, tylko biblioteka standardowa).
// Edytor używa tego do sprawdzania tego, co postawił użytkownik (rozpiętości przęseł, pasmo A) i do doboru karty -
// niczego sam nie rozstawia.
#pragma once
#include <map>
#include <string>
#include <vector>

namespace catenary {

// tablice z pliku data/traction_layout.txt
class layout_tables {
public:
    // wczytuje plik; zwraca false, gdy pliku nie da się otworzyć. błędy w treści trafiają do Errors
    bool load( std::string const &Filename, std::vector<std::string> &Errors );
    // rozbiera treść podaną jako kolejne słowa (już bez komentarzy)
    void parse( std::vector<std::string> const &Tokens, std::string const &Source, std::vector<std::string> &Errors );
    // rozpiętość nominalna przęsła sieci o podanym symbolu w strefie wiatrowej Zone (1 albo 2); 0 gdy sieci nie ma w tablicy
    double nominal_span( std::string const &Symbol, int Zone ) const;
    // odsuw sieci na łuku przyjmowany do rozstawienia konstrukcji (karta 0008), m
    double curve_stagger() const { return m_curvestagger; }
    // symbole sieci z tablicy, w kolejności alfabetycznej
    std::vector<std::string> symbols() const;

private:
    std::map<std::string, std::pair<double, double>> m_spans;
    double m_curvestagger { 0.40 };
};

// największa rozpiętość przęsła na łuku o promieniu Radius przy odsuwie Stagger: a = 2·√(2·R·z) (karta 0008)
double curve_span( double Radius, double Stagger );
// największa rozpiętość przęsła: nominalna sieci, a na łuku nie więcej niż z wykresu 0008. Radius <= 0 = prosta.
// zwraca 0, gdy sieci nie ma w tablicy
double span_limit( layout_tables const &Tables, std::string const &Symbol, int Zone, double Radius, double Stagger );

// promień okręgu przez trzy punkty w planie (x, z); 0 gdy leżą na prostej (albo prawie)
double plan_radius( double const A[ 2 ], double const B[ 2 ], double const C[ 2 ] );

// pasmo A w kartach podwieszeń: ostatnia cyfra karty = (A − 1,45) / 0,30, czyli 0 dla 1,45–1,75 m … 9 dla 4,15–4,45 m.
// zwraca -1 poza zakresem
int reach_digit( double Reach );

// rodzina kart podwieszeń przelotowych (dziesiątka katalogu): rodzaj wysięgnika, prosta czy łuk, odsuw do słupa czy od słupa
struct suspension_family {
    bool swinging { true };     // przechylny (11xx); false = nieprzechylny (12xx)
    bool curve { false };       // łuk (113x/114x, 123x/124x); false = prosta
    bool toward_pole { true };  // odsuw do słupa; false = od słupa
};
// numer dziesiątki (np. 1110, 1120, 1230) dla rodziny wysięgników rurowych z tomu 3 nowego katalogu
int family_decade( suspension_family const &Family );

} // namespace catenary
