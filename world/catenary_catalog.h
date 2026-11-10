/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Sieć trakcyjna: opisany katalog konstrukcji dla edytora (data/traction_catalog.txt, plik pisany ręcznie).
// Karty wysięgników z traction_cantilevers.txt i słupy z traction_supports.txt pogrupowane w kategorie, grupy i rodziny
// z opisami po ludzku, żeby wstawiający wiedział, co wstawia. Z rodziny i odległości od słupa (pasmo A) wybierana jest karta.
// Kod niezależny od reszty symulatora (C++17, tylko biblioteka standardowa).
#pragma once
#include <string>
#include <vector>

namespace catenary {

class cantilever_catalog;
struct cantilever_scheme;

struct catalog_category {
    std::string id;
    std::string title;
    std::string text;                   // opis (linie "text")
};

// grupa: to, co wybiera użytkownik (np. "rurowe przechylne, tom 3"); rodzina w grupie dobiera się sama z położenia
struct catalog_group {
    std::string id;
    std::string category;
    std::string title;
    std::string text;
    bool execution_user { true };       // wykonanie wybiera użytkownik (false: z pasma A, jak w teownikach)
    std::vector<std::pair<int, std::string>> executions; // opisy wykonań (exec <n> : <opis>)
};

// rodzina: karty o jednym przeznaczeniu (np. prosta, odsuw do słupa)
struct catalog_family {
    std::string id;
    std::string group;
    std::string title;
    std::string text;
    std::string image;                  // obrazek (ścieżka tekstury względem katalogu symulatora); pusty = brak
    std::vector<int> cards;             // karty w kolejności doboru
    int curve { -1 };                   // 1 łuk, 0 prosta, -1 bez znaczenia
    int toward { -1 };                  // 1 odsuw do słupa, 0 od słupa, -1 bez znaczenia, 2 z numeru karty (nieparzysta = do słupa)
    bool fits( bool Curve, bool Toward, int Card ) const;
};

// grupa słupów (np. "kratowe 161x na fundamentach palowych")
struct catalog_polegroup {
    std::string id;
    std::string title;
    std::string text;
    std::string foundation;             // piles, cast; puste = nieokreślone
    std::string site;                   // line (szlak), station (równia stacyjna), platform (peron); puste = nieokreślone
    bool no_switch { false };           // słupy tej grupy nie stoją przy rozjazdach (edytor zgłasza błąd)
    std::vector<std::string> poles;
};

class structure_catalog {
public:
    // wczytuje plik; zwraca false, gdy pliku nie da się otworzyć. błędy w treści trafiają do Errors
    bool load( std::string const &Filename, std::vector<std::string> &Errors );
    // rozbiera treść podaną jako linie (bez znaków końca linii)
    void parse( std::vector<std::string> const &Lines, std::string const &Source, std::vector<std::string> &Errors );

    std::vector<catalog_category> const &categories() const { return m_categories; }
    std::vector<catalog_group> const &groups() const { return m_groups; }
    std::vector<catalog_family> const &families() const { return m_families; }
    std::vector<catalog_polegroup> const &polegroups() const { return m_polegroups; }
    catalog_group const *group( std::string const &Id ) const;
    catalog_family const *family( std::string const &Id ) const;
    // rodzina, do której należy karta; nullptr gdy żadna
    catalog_family const *family_of( int Card ) const;
    // grupa słupów, do której należy słup; nullptr gdy żadna
    catalog_polegroup const *polegroup_of( std::string const &Pole ) const;
    bool empty() const { return m_groups.empty(); }

private:
    std::vector<catalog_category> m_categories;
    std::vector<catalog_group> m_groups;
    std::vector<catalog_family> m_families;
    std::vector<catalog_polegroup> m_polegroups;
};

// wynik doboru karty
struct catalog_pick {
    catalog_family const *family { nullptr };
    cantilever_scheme const *scheme { nullptr };
    bool execution_changed { false };   // wybranego wykonania nie ma dla tego A - wzięte inne
    std::string problem;                // dlaczego nie ma karty; puste gdy jest
};

// dobór karty z grupy: rodzina pasująca do łuku i odsuwu (albo Family, gdy niepusta), w niej pierwsza karta, której pasmo A
// obejmuje Reach - wykonanie Execution, gdy wykonanie wybiera użytkownik (a gdy takiego nie ma, inne z tym pasmem),
// w przeciwnym razie dowolne z tym pasmem. Card > 0: tylko ta karta
catalog_pick pick_card( structure_catalog const &Catalog, cantilever_catalog const &Cantilevers, std::string const &Group, std::string const &Family,
    int Card, bool Curve, bool Toward, int Execution, double Reach );

} // namespace catenary
