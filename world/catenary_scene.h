/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Podwieszenia sieci trakcyjnej w scenerii: plik dołączany dyrektywą "catenary <plik> endcatenary".
// Starsze wersje symulatora nie znają tej dyrektywy, zgłaszają trzy nieznane słowa i pliku nie otwierają

#pragma once

#include <string>

class cParser;

// obsługa dyrektywy "catenary" ze scenerii: wczytuje wskazany plik podwieszeń
void catenary_scene_load( cParser &Input );
// po wczytaniu scenerii: wiąże podwieszenia z konstrukcjami wsporczymi i przęsłami po nazwach, zgłasza braki
void catenary_scene_resolve();
// przed wczytaniem nowej scenerii: usuwa podwieszenia poprzedniej
void catenary_scene_clear();
// zmienia katalog, z którego czytany jest katalog wysięgników (traction_cantilevers.txt i traction_supports.txt); domyślnie "data/"
void catenary_scene_data( std::string const &Directory );

// ---- edytor scenerii: podwieszenia zmieniane na żywo -----------------------------------------------------------------
// Każde podwieszenie ma numer (miejsce na liście, stały do końca sesji; usunięte zostaje na liście jako puste).
// Zmiany trafiają do plików sieci dopiero przy catenary_scene_save().

#include <vector>
#include <utility>
#include "world/catenary_suspension.h"
#include "world/catenary_cantilever.h"
#include "world/catenary_layout.h"
#include "world/catenary_catalog.h"
#include "world/catenary_tensioner.h"

// katalog wysięgników (części, karty, słupy, mocowania); nullptr gdy nie dało się go wczytać
catenary::cantilever_catalog const *catenary_scene_catalog();
// zasady rozstawiania konstrukcji (data/traction_layout.txt); pusta tablica, gdy pliku nie ma
catenary::layout_tables const &catenary_scene_layout();
// opisany katalog konstrukcji dla edytora (traction_catalog.txt): kategorie, grupy, rodziny kart, grupy słupów; pusty gdy brak pliku
catenary::structure_catalog const &catenary_scene_structures();
// liczba miejsc na liście podwieszeń (z usuniętymi)
std::size_t catenary_scene_size();
// podwieszenie o podanym numerze; nullptr gdy usunięte albo numer spoza listy
catenary::suspension const *catenary_scene_get( std::size_t Index );
// plik sieci, z którego pochodzi podwieszenie (albo do którego trafi)
std::string catenary_scene_file_of( std::size_t Index );
// liczba części wstawionych do scenerii dla podwieszenia; 0 gdy wysięgnik się nie złożył
std::size_t catenary_scene_parts( std::size_t Index );
// pliki sieci znane scenerii: dołączone dyrektywą i założone w edytorze
std::vector<std::string> catenary_scene_files();
// dodaje podwieszenie do podanego pliku (nowy plik jest zakładany) i od razu składa wysięgnik. zwraca numer
std::size_t catenary_scene_add( std::string const &File, catenary::suspension const &Data );
// zmienia podwieszenie i składa wysięgnik od nowa (w parasolu oba wysięgniki). false gdy numer jest zły
bool catenary_scene_update( std::size_t Index, catenary::suspension const &Data );
// usuwa podwieszenie razem z jego częściami
void catenary_scene_remove( std::size_t Index );
// przywraca usunięte podwieszenie (cofnięcie w edytorze) z podanymi danymi
bool catenary_scene_restore( std::size_t Index, catenary::suspension const &Data );
// próbne złożenie bez wstawiania części: wynik solvera z uwagami; false gdy wysięgnika nie da się złożyć
// (przyczyna w Result.warnings)
bool catenary_scene_probe( catenary::suspension const &Data, catenary::cantilever_result &Result );
// czy są zmiany do zapisania
bool catenary_scene_changed();
// pliki założone w edytorze, których scenerii jeszcze nie dołączono dyrektywą "catenary"; po zapisie trzeba ją dopisać
std::vector<std::string> catenary_scene_new_files();
// sceneria dołącza już plik (edytor dopisał dyrektywę "catenary")
void catenary_scene_file_included( std::string const &File );
// warstwa scenerii (uchwyt scene::Layers), której plik dołącza plik sieci, a dla pliku założonego w edytorze ta, do której ma
// trafić jego dyrektywa; 0 gdy nieznana (wtedy plik scenariusza)
std::size_t catenary_scene_file_layer( std::string const &File );
void catenary_scene_file_layer( std::string const &File, std::size_t Layer );
// zapisuje zmienione pliki sieci do katalogu scenerii. Message: co zapisano albo co poszło źle
bool catenary_scene_save( std::string &Message );

// ---- łańcuchy sieci (wpisy "chain" w pliku sieci): przęsła od podwieszenia do podwieszenia ------------------------------
// nowa, wolna w pliku nazwa podwieszenia (s0001, s0002, ...) i łańcucha (<plik>_l1, ...)
std::string catenary_scene_new_name( std::string const &File );
std::string catenary_scene_new_chain_name( std::string const &File );
std::size_t catenary_chain_size();
// łańcuch o podanym numerze; nullptr gdy usunięty
catenary::chain const *catenary_chain_get( std::size_t Index );
std::string catenary_chain_file( std::size_t Index );
// numery podwieszeń łańcucha po kolei (catenary_scene_size() w miejscu brakującego)
std::vector<std::size_t> catenary_chain_members( std::size_t Index );
// dodaje łańcuch (podwieszenia muszą mieć nazwy i być w tym samym pliku) i od razu robi jego przęsła. zwraca numer
std::size_t catenary_chain_add( std::string const &File, catenary::chain const &Data );
// zmienia łańcuch i robi jego przęsła od nowa (mniej niż dwa podwieszenia = łańcuch usunięty)
bool catenary_chain_update( std::size_t Index, catenary::chain const &Data );
void catenary_chain_remove( std::size_t Index );
// kotwienie środkowe łańcucha: liczba wstawionych części (na słupach sąsiednich podwieszeń) i trójkątów brył (lina, V, zaciski);
// false, gdy łańcuch go nie ma albo się nie złożyło
bool catenary_chain_middle( std::size_t Index, std::size_t &Parts, std::size_t &Triangles );
// łańcuch, do którego należy podwieszenie, i jego miejsce w łańcuchu; -1 gdy żaden
int catenary_chain_of( std::size_t Suspension, std::size_t &Position );

// ---- kotwienia (wpisy "anchor" w pliku sieci): słup kotwowy z urządzeniem naprężającym ---------------------------------
std::size_t catenary_anchor_size();
catenary::anchor const *catenary_anchor_get( std::size_t Index );
std::string catenary_anchor_file( std::size_t Index );
std::size_t catenary_anchor_parts( std::size_t Index );
// wynik solvera ostatniego złożenia (b, s, ciężary, naciągi, uwagi); nullptr gdy usunięte
catenary::tensioner_result const *catenary_anchor_result( std::size_t Index );
// obrys złożonego urządzenia dla edytora (współrzędne scenerii): czoło słupa i oś rolki ruchomej na wysokości rolek, góra i spód
// kolumny ciężarów; false gdy usunięte albo niezłożone
bool catenary_anchor_shape( std::size_t Index, double ( &Shape )[ 4 ][ 3 ] );
// karty urządzeń kotwienia do wyboru (bez odmian dla typów sieci, np. 0461-2x2): nazwa, kotwienie stałe, rolki szeregowe
struct catenary_anchor_card {
    std::string card;
    bool fixed { false };
    bool series { false };
    bool split { false };   // dwa stosy (lina i przewody osobno)
    std::string title;      // opis z danych (odmiany pokazane osobno, np. 0461-2x2)
};
// Only: puste - kotwienia całości; "messenger" / "contact" - samej liny / samych przewodów (przewody zakotwione osobno)
std::vector<catenary_anchor_card> catenary_anchor_devices( std::string const &Only = {} );
// katalog urządzeń naprężających (traction_tensioners.txt); nullptr, gdy go nie ma
catenary::tensioner_catalog const *catenary_scene_tensioners();
// łańcuch, który kończy się tym kotwieniem (Start: na początku), -1 gdy żaden
int catenary_anchor_chain( std::size_t Index, bool &Start );
// dodaje kotwienie, składa je i przebudowuje łańcuchy, które go używają. zwraca numer
std::size_t catenary_anchor_add( std::string const &File, catenary::anchor const &Data );
// zmienia kotwienie (także przywraca usunięte) i przebudowuje łańcuchy, które go używają
bool catenary_anchor_update( std::size_t Index, catenary::anchor const &Data );
void catenary_anchor_remove( std::size_t Index );
std::string catenary_scene_new_anchor_name( std::string const &File );
// po zmianie temperatury powietrza: kotwienia złożone przy innej (różnica od 0,5 °C) składa od nowa, z przęsłami do nich
void catenary_scene_temperature();

// ---- bramki (wpisy "gantry" w pliku sieci): słupy i dźwigar; zwisy z wysięgnikami to zwykłe podwieszenia -----------------
std::size_t catenary_gantry_size();
catenary::gantry const *catenary_gantry_get( std::size_t Index );
std::string catenary_gantry_file( std::size_t Index );
std::size_t catenary_gantry_parts( std::size_t Index );
std::size_t catenary_gantry_add( std::string const &File, catenary::gantry const &Data );
// zmienia bramkę (także przywraca usuniętą) i wstawia jej modele od nowa
bool catenary_gantry_update( std::size_t Index, catenary::gantry const &Data );
void catenary_gantry_remove( std::size_t Index );
std::string catenary_scene_new_gantry_name( std::string const &File );
// końce spodu dźwigara bramki (każdy na mocowaniu swojego słupa), tam gdzie wiszą zwisy
// Second == nullptr: pojedynczy element dźwigara o długości Span od ściany pierwszego słupa
void gantry_beam_ends( catenary::gantry const &Data, catenary::gantry_mast const &First, catenary::gantry_mast const *Second, double Span, double *From, double *To );
