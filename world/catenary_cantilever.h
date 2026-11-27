// Sieć trakcyjna: wysięgnik składany z części katalogowych.
// Katalog (części, zestawy ramion odciągowych, schematy kart, sposoby składania) jest czytany z pliku danych.
// Solver nie zna żadnego konkretnego wysięgnika: składa go z klocków połączeń (człon z punktu w kierunku, styczna do
// ucha, uchwyt na danej wysokości, dobór skokowy długości, część wisząca w linii z linką...) w kolejności zapisanej
// w bloku "assembly" pliku danych. Opis klocków jest w komentarzu nad blokami w pliku danych.
// Kod niezależny od reszty symulatora (C++17, tylko biblioteka standardowa).
//
// Układ wysięgnika: zero w osi przewodów jezdnych przy podwieszeniu, Y poziomo w stronę konstrukcji wsporczej,
// Z w górę, X wzdłuż toru. Wszystkie wymiary w metrach.
//
// Układ części liczonych przez solver: model części jest ułożony tak, jak część siedzi w wysięgniku o poziomych rurach
// (Y w stronę konstrukcji, Z w górę, sworznie wzdłuż X), z punktem zerowym w głównym zaczepie. Solver obraca część
// wokół osi X tylko o tyle, o ile jej rzeczywisty kierunek różni się od spoczynkowego:
//   izolatory 7xxx        zero w zaczepie przy konstrukcji, biegną w -Y (do toru)
//                         izolator odciągu wiszący na szekli 4700 solver dodatkowo obraca o 90 stopni wokół osi Y
//   ukośnik 6001          zero w osi otworu sworznia przy izolatorze, biegnie w -Y
//   odciąg 6007           zero w osi ucha przy izolatorze, biegnie w -Y
//   końcówka 4400         zero w osi ucha przy uchwycie 4070, biegnie w +Y (rura z otworami w stronę konstrukcji)
//   szekla 4700           zero od strony mocowania, biegnie w -Y do punktu "eye" (sworzeń w uchu izolatora)
//   wysięg pomocniczy     6011: zero w osi haka (na uchu leży punkt "hook" z danych), 6111: w osi pierwszego otworu; biegnie w -Y
//   uchwyt 4050           zero w osi rury, ucho w -Z (w dół); "eye" to krawędź ucha, na której leży hak
//   uchwyt 4070           zero w osi rury, sworzeń w +Z (w górę)
//   obejma linki 4210     zero w osi rury, śruba linki w +Z
//   uchwyt liny 4140      zero w osi liny nośnej, punkt "hook" (ucho uchwytu 4050) nad nim, "end" (linka) pod nim
//   linka 4490            zero w dolnym zaczepie (przy obejmie 4210), biegnie w +Z; skalowana wzdłuż Z
// Części zestawu ramion odciągowych mają położenie i obrót przeniesione wprost z pliku danych.
#pragma once
#include <map>
#include <set>
#include <string>
#include <vector>

namespace catenary {

// składnik zestawu ramion złożonego z osobnych modeli (np. para 6207-1 + 6207-2 z dwoma 4110): położenie zera modelu
// względem zera zestawu (osi przewodu odniesienia) i obrót wokół pionu, jak w złożeniu z Blendera
struct cantilever_member {
    std::string part;
    double position[ 3 ] { 0.0, 0.0, 0.0 };  // X wzdłuż toru, Y w stronę konstrukcji, Z w górę
    double yaw { 0.0 };                       // stopnie, wokół Z, dodatni przeciwnie do wskazówek zegara patrząc z góry
};

// wymiary części potrzebne mechanice
struct cantilever_part {
    std::string name;               // numer katalogowy, zarazem nazwa modelu
    std::string kind;               // strut, stay, stay_end, steady, clamp, messenger_clamp, insulator, shackle, cord
    double length { 0.0 };          // od głównego zaczepu do końca albo do osi drugiego połączenia
    double clearance { 0.0 };       // izolator: dodatkowy odstęp za "length", którego nie może zająć uchwyt na rurze (gruby koniec izolatora 25 kV)
    double pin { 0.0 };             // rura: odległość osi otworu sworznia od końca rury
    double radius { 0.019 };        // rura: promień (6011, 6111: 38 mm; 6001, 6002, 6003: 60 mm - Jan)
    double eye { 0.0 };             // uchwyt: odległość ucha od osi rury
    double hook[ 2 ] { 0.0, 0.0 };  // punkt, którym część wisi na uchu uchwytu 4050 (Y, Z w układzie części): uchwyt liny, hak wysięgu 6011
    double end[ 2 ] { 0.0, 0.0 };   // uchwyt liny: punkt mocowania linki (Y, Z w układzie części)
    std::vector<double> holes;      // końcówka odciągu: odległości otworów od osi ucha
    std::map<std::string, std::pair<double, double>> points; // nazwane punkty modelu (Y, Z w układzie części), np. holder, eye, slide_min
    std::vector<cantilever_member> members; // zestaw ramion z osobnych modeli: składniki stawiane zamiast samego zestawu
    std::map<std::string, double> points_x; // składowa X (wzdłuż toru) nazwanych punktów, gdy model ją ma (ramiona 6207/6208: obejma 4210, ucho)
    std::string model;              // inny plik modelu niż numer katalogowy (lina 9832-2 = model linki 4490-00), pusty - jak name
    double girth { 1.0 };           // część rozciągana (linka): skala w poprzek osi (grubsza lina z modelu cieńszej linki)
};

// część o stałym położeniu względem przewodów jezdnych
struct cantilever_item {
    std::string part;
    double position[ 3 ] { 0.0, 0.0, 0.0 };
    double rotation[ 9 ] { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };  // wierszami, kolumny to osie części
};

// zestaw ramion odciągowych jednego wykonania
struct cantilever_registration {
    std::string name;
    double tube { 0.22 };           // wysokość osi wysięgu pomocniczego nad przewodem jezdnym
    double clamp { 0.17 };          // nominalne położenie Y obejmy linki na wysięgu pomocniczym
    std::vector<cantilever_item> items;
};

// schemat: jedna karta katalogowa w jednym wykonaniu
struct cantilever_scheme {
    int    card { 0 };
    int    execution { 0 };
    double reach_min { 0.0 };       // pasmo A z karty: od osi przewodów do czoła konstrukcji, razem z mocowaniem
    double reach_max { 0.0 };
    double height { 1.70 };         // wysokość konstrukcyjna
    double mount_stay { 0.0 };      // nominalne wysokości zaczepów izolatorów nad przewodem jezdnym
    double mount_strut { 0.0 };
    double mount_steady { 0.0 };
    bool   steady_mounted { false };// wysięg pomocniczy ma własny izolator przy konstrukcji (karty z trzema zaczepami)
    std::string registration;       // nazwa zestawu ramion odciągowych
    std::string assembly;           // sposób składania (blok "assembly")
    std::vector<double> tube_steps; // regulowany uchwyt ramion (np. 4290): możliwe podniesienia rury wysięgu względem zestawu
                                    // ramion; dolny zaczep ukośnika idzie w górę razem z rurą. Solver bierze najniższe, przy którym
                                    // wysięgnik składa się bez ostrzeżeń (albo z najmniejszą ich liczbą), próbując od najbliższego
    double tube_prefer { 0.0 };     // podniesieniu preferowanemu (np. położeniu ze złożeń)
    std::map<std::string, std::string> roles; // rola -> część
};

// ustawienia montażowe, wspólne dla wszystkich schematów
struct cantilever_rules {
    double stay_gap_min { 0.10 };       // odstęp uchwytu odciągu od uchwytu głównego wzdłuż ukośnika: najmniejszy,
    double stay_gap { 0.15 };           // zalecany
    double stay_gap_max { 0.30 };       // i największy (z kart: "max 0,30 m")
    double clamp_margin { 0.03 };       // ile rury musi zostać za osią uchwytu
    double cord_clamp_inset { 0.0 };    // najmniejsza odległość osi obejmy linki od końca wysięgu pomocniczego
    double umbrella_spread { 0.10 };    // parasol: największa różnica wysokości mocowań obu wysięgników, przy której idą na wspólne mocowanie
};

// słup: wymiary potrzebne do położenia czoła na dowolnej wysokości i mocowania, które się na nim stosuje.
// wysokości liczone od główki szyny; zero modelu słupa w jego osi na wysokości główki szyny
struct cantilever_pole {
    std::string name;
    std::string model;                  // ścieżka modelu względem katalogu modeli symulatora, bez rozszerzenia
    std::string foundation;             // model fundamentu (ścieżka jak wyżej) stawiany w tym samym zerze co słup; puste = bez fundamentu
    double top { 0.0 };                 // góra słupa nad główką szyny
    double length { 0.0 };              // długość, na której biegnie zbieżność, liczona od góry
    double depth_top { 0.0 };           // wymiar od toru w głąb (prostopadle do toru) u góry
    double depth_base { 0.0 };          // to samo na końcu długości zbieżności
    double width { 0.0 };               // wzdłuż toru (słup kratowy 161x: wysokość ceownika, karta 33-1611-E wymiar a); 0 = nieznana
    double mount_max { 0.0 };           // najwyżej położona oś śrub mocowania; 0 = bez ograniczenia (zwis na bramce: ujemna, pod dźwigarem)
    double mount_min { 0.0 };           // najniżej położona oś śrub mocowania (zwis: nad jego końcem); tylko gdy has_mount_min
    bool   has_mount_min { false };
    double turn { 0.0 };                // obrót modelu wokół pionu względem płaszczyzny wysięgnika [stopnie] (zwis: 90)
    std::string drop;                   // zwis na bramce: rodzina zwisów (np. 4850), do której pasuje dźwigar; pusta = zwykły słup
    std::string strut;                  // mocowanie ukośnika (część z punktem axis i zawiasem)
    std::string stay;                   // mocowanie izolatora odciągu
    std::string steady;                 // mocowanie izolatora wysięgu pomocniczego (schematy z trzema zaczepami)
    std::string console;                // konsola wysięgników rozstawionych (wpis "mount ... console"); pusta = słup ich nie nosi
    std::string steady_console;         // belka 4690 rozstawionych wysięgów pomocniczych (karta 33-0519, wysięgi 0167); mocowanie wysięgu na niej
    // słup bramki z wysięgnikiem (zrobiony z wpisu "mast" z mocowaniami): model stawia bramka, nie wysięgnik; mount_max pod
    // dźwigarem bramki liczy sceneria wg jej poziomu (mast_low/mast_high minus below_beam), tu - pod dolnym mocowaniem
    bool   gantry { false };
    double mast_low { 0.0 };
    double mast_high { 0.0 };
    double below_beam { 0.15 };
    // głębokość słupa na wysokości Height nad główką szyny
    double depth( double Height ) const;
};

// mocowanie do słupa: przód z punktem zaczepu, opcjonalnie tył po drugiej stronie słupa, zawias albo pręt z uchem.
// układ jak w pozostałych częściach: zero na płaszczyźnie przylegania do słupa w połowie między śrubami, słup w +Y
struct cantilever_mount {
    std::string name;
    std::string back;                   // plik tyłu (płaskownik z nakrętkami), stawiany na tylnej ścianie słupa
    std::string hinge;                  // zawias obrotowy stawiany w punkcie axis (wspornik ukośnika)
    std::string rod;                    // pręt z uchem stawiany w punkcie eye (uchwyt odciągu)
    std::string twin;                   // połówka z nakrętkami mocowania dwustronnego (parasol); pusta = mocowanie bez regulacji, parasol z dwóch niezależnych
    // konsola wysięgników rozstawionych (4890, karty 0517/0518): dwa punkty mocowań w X (wzdłuż toru) i ich odsunięcie od
    // płaszczyzny słupa (Y, ujemne - w stronę toru); na konsoli mocowania -06 (4610-06, 4620-06, 4630-06)
    bool   console { false };
    double console_x[ 2 ] { 0.0, 0.0 };
    double console_face { 0.0 };
    double point[ 2 ] { 0.0, 0.0 };     // Y, Z punktu axis (gdy jest zawias) albo eye
    double hinge_eye[ 2 ] { 0.0, 0.0 }; // Y, Z ucha zawiasu względem jego zera
    // punkt zaczepu izolatora względem zera mocowania
    double attach_y() const { return point[ 0 ] + hinge_eye[ 0 ]; }
    double attach_z() const { return point[ 1 ] + hinge_eye[ 1 ]; }
};

// jeden klocek sposobu składania: słowo kluczowe i jego argumenty, jak w pliku danych
struct cantilever_step {
    std::string op;
    std::vector<std::string> words;
    std::size_t end { 0 };              // "repeat": indeks kroku za pasującym "endrepeat"
};

// sposób składania wysięgnika: klocki w kolejności wykonywania
struct cantilever_assembly {
    std::string name;
    std::vector<cantilever_step> steps;
};

// katalog wysięgników wczytany z pliku danych
// słup bramki: model z zerem w osi słupa na wysokości główki szyny, Y modelu wzdłuż dźwigara, X wzdłuż toru
struct gantry_mast {
    std::string name;
    std::string model;
    double width { 0.0 };               // wzdłuż toru
    double length { 0.0 };              // wzdłuż dźwigara
    double height { 0.0 };
    double mount_low { 0.0 };           // prześwit (spód dźwigara nad główką szyny) w dolnym i górnym mocowaniu
    double mount_high { 0.0 };
    double offset { 0.0 };              // przesunięcie dźwigara wzdłuż toru względem osi słupa
    bool   through { false };           // przelotowy (dźwigary z obu stron); false = końcowy
    // wysięgnik wprost na słupie bramki: mocowania (karta 53-0501 wyk. 04 - słupy bramek o rozstawie 400x600) i odstęp
    // najwyższej osi śrub od spodu dźwigara; bez mocowań na słupie bramki nie da się postawić wysięgnika
    std::string strut;
    std::string stay;
    std::string steady;
    double below_beam { 0.15 };
};
// dźwigar bramki: model z zerem w osi pierwszego słupa na spodzie dźwigara, X modelu wzdłuż dźwigara
struct gantry_beam {
    std::string name;
    std::string model;
    double span { 0.0 };                // rozstaw osi słupów; pojedynczy element: długość od ściany słupa
    std::string drop;                   // rodzina pasujących zwisów
    bool single { false };              // pojedynczy element na jednym słupie przelotowym (3017-3025, 1910): z jednej strony
                                        // mocowanie do słupa, z drugiej do konsoli
};

// rozstaw osi słupów bramki: między ścianami słupów o 0,20 m mniej niż numer dźwigara (E101: 9,9 m), do osi dochodzą offsety słupów
inline double gantry_spacing( gantry_beam const &Beam, gantry_mast const &First, gantry_mast const &Second ) {
    return Beam.span - 0.20 + First.offset + Second.offset;
}

class cantilever_catalog {
public:
    // rozbiera plik podany jako kolejne słowa (już bez komentarzy); błędy trafiają do Errors
    void parse( std::vector<std::string> const &Tokens, std::string const &Source, std::vector<std::string> &Errors );
    // wczytuje plik; zwraca false, gdy pliku nie da się otworzyć
    bool load( std::string const &Filename, std::vector<std::string> &Errors );
    cantilever_part const *part( std::string const &Name ) const;
    cantilever_registration const *registration( std::string const &Name ) const;
    cantilever_scheme const *scheme( int Card, int Execution ) const;
    cantilever_pole const *pole( std::string const &Name ) const;
    cantilever_mount const *mount( std::string const &Name ) const;
    cantilever_assembly const *assembly( std::string const &Name ) const;
    // karta z tej samej dziesiątki co Card, której pasmo A obejmuje podaną odległość; nullptr gdy żadna
    cantilever_scheme const *scheme_for_reach( int Card, int Execution, double Reach ) const;
    gantry_mast const *mast( std::string const &Name ) const;
    gantry_beam const *beam( std::string const &Name ) const;
    std::map<std::string, gantry_mast> const &masts() const { return m_masts; }
    std::map<std::string, gantry_beam> const &beams() const { return m_beams; }
    // katalog modeli części względem katalogu modeli symulatora, z ukośnikiem na końcu (wpis "models" w pliku danych)
    std::string const &models() const { return m_models; }
    std::size_t part_count() const { return m_parts.size(); }
    std::size_t scheme_count() const { return m_schemes.size(); }
    std::size_t registration_count() const { return m_registrations.size(); }
    std::map<std::pair<int, int>, cantilever_scheme> const &schemes() const { return m_schemes; }
    std::map<std::string, cantilever_part> const &parts() const { return m_parts; }
    std::map<std::string, cantilever_pole> const &poles() const { return m_poles; }
    cantilever_rules rules;

private:
    std::string m_models { "siec_trakcyjna/components/" };
    std::map<std::string, cantilever_part> m_parts;
    std::map<std::string, cantilever_registration> m_registrations;
    std::map<std::pair<int, int>, cantilever_scheme> m_schemes;
    std::map<std::string, cantilever_pole> m_poles;
    std::map<std::string, cantilever_mount> m_mounts;
    std::map<std::string, cantilever_assembly> m_assemblies;
    std::map<std::string, gantry_mast> m_masts;
    std::map<std::string, gantry_beam> m_beams;
};

// punkt w płaszczyźnie wysięgnika
struct cantilever_point { double y { 0.0 }; double z { 0.0 }; };

// dane jednego podwieszenia
struct cantilever_input {
    cantilever_point stay_mount;        // zaczep izolatora odciągu przy konstrukcji (po odjęciu mocowania i szekli)
    cantilever_point strut_mount;       // zaczep izolatora ukośnika
    cantilever_point steady_mount;      // zaczep izolatora wysięgu pomocniczego; tylko dla schematów z trzema zaczepami
    cantilever_point messenger;         // oś liny nośnej
    std::map<std::string, std::string> overrides; // podmiany części: rola -> część
    std::map<std::string, cantilever_point> points; // dodatkowe punkty wejściowe dla sposobu składania (nazwa -> punkt)
};

// nominalne dane podwieszenia: zaczepy na wysokościach ze schematu, w odległości Reach od osi przewodów,
// lina nośna nad przewodami na wysokości konstrukcyjnej schematu
cantilever_input cantilever_nominal_input( cantilever_scheme const &Scheme, double Reach );

// jedna część złożonego wysięgnika
struct cantilever_placement {
    std::string role;
    std::string part;
    std::string model;                  // ścieżka modelu względem katalogu modeli symulatora, gdy część leży poza katalogiem części (słup)
    double position[ 3 ] { 0.0, 0.0, 0.0 };
    double rotation[ 9 ] { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };  // wierszami, kolumny to osie części
    double scale[ 3 ] { 1.0, 1.0, 1.0 };
};

struct cantilever_result {
    std::vector<cantilever_placement> parts;
    std::vector<std::string> warnings;  // czego nie dało się złożyć zgodnie z zasadami; części są mimo to ustawione
    double strut_angle { 0.0 };         // rad, nachylenie ukośnika do poziomu
    double strut_overhang { 0.0 };      // ile ukośnika wystaje za osią uchwytu głównego
    int    stay_hole { -1 };            // wybrany otwór końcówki odciągu, licząc od ucha
    double stay_gap { 0.0 };            // odstęp uchwytu odciągu od uchwytu głównego wzdłuż ukośnika
    double steady_overhang { 0.0 };     // ile wysięgu pomocniczego wystaje za osią obejmy linki
    double cord_length { 0.0 };         // długość linki między zaczepami
    std::map<std::string, double> values; // wartości nazwane przez klocki (np. "strut.overhang", "stay_clamp.hole")
    cantilever_placement const *find( std::string const &Role ) const;
};

// składa wysięgnik. zwraca false, gdy brakuje danych (części albo zestawu ramion) albo geometria jest niemożliwa;
// przyczyna trafia wtedy do Result.warnings
bool solve_cantilever( cantilever_catalog const &Catalog, cantilever_scheme const &Scheme, cantilever_input const &Input, cantilever_result &Result );

// wysięgnik na słupie. Układ jak w solve_cantilever (zero w osi przewodów jezdnych, Y poziomo w stronę słupa, Z w górę);
// oś słupa leży w Y = Distance, główka szyny na Z = RailHead (zwykle ujemnym), lina nośna w Messenger.
// Zaczepy izolatorów biorą się z mocowań słupa: wysokości ze schematu (zaczep odciągu idzie za liną nośną, gdy ta wisi
// wyżej niż w schemacie), odległość od osi z głębokości słupa na wysokości mocowania i z punktów mocowania. Oś śrub
// mocowania odciągu nie wychodzi ponad mount_max słupa. Do Result.parts trafia też słup, mocowania, tyły, zawias i pręt.
bool solve_cantilever_on_pole( cantilever_catalog const &Catalog, cantilever_scheme const &Scheme, cantilever_pole const &Pole,
    double Distance, double RailHead, cantilever_point const &Messenger, std::map<std::string, std::string> const &Overrides, cantilever_result &Result );

// parasol: drugi wysięgnik na tym samym słupie, po przeciwnej stronie. Wysokości mocowań (stay, strut, steady) liczone od główki
// szyny; mocowanie z listy Shared idzie na podaną wysokość jako połówka mocowania dwustronnego (bez tyłu; role z Nuts dostają
// połówkę z nakrętkami, czyli "twin" mocowania). Wysokości wyliczone przez solver trafiają do Result.values jako "mount.<rola>".
struct cantilever_umbrella {
    std::map<std::string, double> shared;   // rola mocowania -> wspólna wysokość osi śrub nad główką szyny
    std::set<std::string> nuts;             // role mocowań, które po tej stronie są połówką z nakrętkami
    bool pole { true };                     // czy stawiać słup (stawia go tylko jedna strona)
    // wysięgniki rozstawione: oba po tej samej stronie słupa, na konsolach (konsola słupa), mocowania -06 na wspólnych
    // wysokościach z shared; płaszczyzna wysięgnika przechodzi przez punkt mocowania na konsoli, oś słupa leży w X = ConsoleX
    bool spread { false };
    double console_x { 0.0 };
    bool console_parts { true };            // czy stawiać konsole (stawia je jedna strona)
};
bool solve_cantilever_on_pole( cantilever_catalog const &Catalog, cantilever_scheme const &Scheme, cantilever_pole const &Pole,
    double Distance, double RailHead, cantilever_point const &Messenger, std::map<std::string, std::string> const &Overrides,
    cantilever_umbrella const &Umbrella, cantilever_result &Result );
// wspólne mocowania parasola z wysokości wyliczonych osobno dla obu stron (wartości "mount.<rola>" z wyników): pary mocowań
// z połówką z nakrętkami, których wysokości różnią się najwyżej o rules.umbrella_spread, idą na średnią wysokość. Ta sama rola
// po obu stronach (4640/4650/4660): nakrętki po stronie Second. Ukośnik jednej strony z odciągiem drugiej (4670, Jan
// 2026-10-17): śruby po stronie ukośnika (4610-NN), nakrętki po stronie odciągu (4650-NN). Pary brane od najmniejszej różnicy,
// każde mocowanie najwyżej raz. Średnia nie wychodzi ponad mount_max słupa (chyba że obie strony są wyżej - wtedy niższa).
// Wypełnia shared i nuts obu stron.
void umbrella_mounts( cantilever_catalog const &Catalog, cantilever_pole const &Pole, cantilever_result const &First, cantilever_result const &Second,
    cantilever_umbrella &FirstSide, cantilever_umbrella &SecondSide );

// wspólne wysokości mocowań wysięgników rozstawionych: każda rola obu wysięgników na jednej konsoli, na średniej wysokości
// (nie ponad mount_max słupa). Wypełnia shared obu stron
void spread_mounts( cantilever_pole const &Pole, cantilever_result const &First, cantilever_result const &Second,
    cantilever_umbrella &FirstSide, cantilever_umbrella &SecondSide );

// część ustawiona w układzie scenerii symulatora (X, Y w górę, Z), względem punktu osi przewodów jezdnych
struct cantilever_scene_part {
    double offset[ 3 ] { 0.0, 0.0, 0.0 };   // przesunięcie od punktu osi przewodów jezdnych
    double angles[ 3 ] { 0.0, 0.0, 0.0 };   // kąty jak w dopisku "angles" wpisu node model [stopnie]: obroty Y, potem X, potem Z
    double scale[ 3 ] { 1.0, 1.0, 1.0 };    // skala jak w dopisku "scale" wpisu node model
};

// przelicza część z układu wysięgnika na układ scenerii. Heading to kierunek poziomy od osi przewodów do konstrukcji
// wsporczej, jako kąt wokół osi Y scenerii [rad]: atan2( x, z ) tego kierunku.
// uwzględnia zamianę osi, którą symulator robi przy wczytaniu modelu t3d (odwrócenie X, zamiana Y z Z)
cantilever_scene_part cantilever_to_scene( cantilever_placement const &Placement, double Heading );

} // namespace catenary
