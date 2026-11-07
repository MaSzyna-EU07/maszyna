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
//                       zero w sworzniu od strony konstrukcji/dźwigni, biegną w -X do drugiego sworznia (pusty węzeł,
//                       np. hole_1 uchwytu 4440; długość z danych); "~część" w łańcuchu - postawiona odwrotnie (4440: zero
//                       w wąskim końcu, widełki z osią rolki na -X; przy rolce ruchomej widełki są od strony słupa)
//   dźwignia 6406       zero w otworze osi II (mount_2 = III), otwory w +Z (I na dole), otwory na druty odsunięte w -X
//                       modelu (w stronę drutów); przy otworze liny poniżej otworu przewodów dźwignia odwrócona (180 wokół Y)
//   pręt 6401/6402      zero na dole (pod stosem), w +Z do zaczepu na górze (długość z danych)
//   ciężar 63xx         zero w środku spodu, w +Z; kolejne obracane o "turn" wokół Z; kolor numerem wykonania modelu
//   prowadnica 5070     zero w osi przesuwu pierścienia na dole, w +Z; mocowania do wsporników w -X (w stronę konstrukcji)
//   pierścień 5080      zero w osi stosu, poziomo, punkt przesuwu po prowadnicy w -X (w stronę konstrukcji); ring_at nad spodem stosu
//   lina                linka rozciągana (część typu cord, zero w zaczepie, w +Z, skalowana wzdłuż Z)
#pragma once
#include <functional>
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
    // krążek drugiego rowka jako osobny model (4460-11), stawiany w osi rolki; model rolki niesie wtedy oś i krążek
    // pierwszego rowka. Oba obracają się osobno o przewiniętą linę; pusty = cała rolka jednym modelem, obracana z rowkiem A
    std::string sheave;
};

struct tensioner_lever {
    std::string name;
    std::map<std::string, std::pair<double, double>> holes; // nazwa otworu -> (wzdłuż osi od otworu I, odsunięcie w bok)
};

// część liniowa w łańcuchu
struct tensioner_link {
    std::string name;
    double length { 0.0 };
    double roll { 0.0 };    // stopnie wokół osi łańcucha (w danych "<część>@<stopnie>"), np. 90 - sworzeń obrócony do następnej części
};

// stos ciężarów i otwory dźwigni dla typu sieci (numer typu z karty 0002 = traction_code)
struct tensioner_type {
    int code { -1 };
    std::vector<std::pair<std::string, int>> stack;   // od dołu: ciężar, liczba
    std::string messenger_hole, contact_hole, axis_hole;
    // dźwignie szeregowo (2 liny + 2 przewody na jednym stosie): otwór przewodów pierwszej dźwigni ciągnie łańcuchem "cascade"
    // drugą dźwignię za jej otwór osi; druga dzieli dalej na drugą linę nośną i przewody. Puste = jedna dźwignia
    std::string second_messenger_hole, second_contact_hole, second_axis_hole;
    std::string rod;                    // pręt, gdy inny niż urządzenia
    std::string messenger_clamp;        // uchwyt krańcowy liny nośnej, gdy inny niż urządzenia
};

struct tensioner_device {
    std::string name;                   // karta, np. 0461
    std::string table, pulley, lever, rod, rope;
    std::string lever2;                 // druga dźwignia (szeregowo), gdy inna niż pierwsza
    double rope_diameter { 0.010 };
    std::string rope_eye;               // ucho (kausza) na obu końcach liny, nieskalowane; pusty = lina bez uch
    double rope_eye_length { 0.0 };     // od zera ucha (styk z liną) do punktu "eye" (zaczep), wzdłuż +Z modelu
    std::string rope_spacer;            // przekładka na sworzniu ucha przy pręcie (6411, oś w +Z modelu); pusta = bez
    double rope_spacer_length { 0.0 };
    std::vector<tensioner_link> mount;  // od czoła konstrukcji do osi rolki stałej
    std::vector<tensioner_link> holder; // od osi rolki ruchomej do osi dźwigni
    std::vector<tensioner_link> hanger; // od rolki stałej w dół do pręta (bez liny)
    std::vector<tensioner_link> messenger; // od otworu dźwigni do uchwytu krańcowego liny nośnej (bez uchwytu)
    std::vector<tensioner_link> contact;   // od otworu dźwigni do wyrównywacza albo uchwytu krańcowego przewodu
    // dźwignie szeregowo: od otworu przewodów pierwszej do osi drugiej (z izolatorem) i od otworu liny drugiej do uchwytu drugiej
    // liny nośnej; przewody ("contact") wtedy z drugiej dźwigni
    std::vector<tensioner_link> cascade, messenger2;
    // kotwienie stałe (kind fixed): bez rolek i ciężarów; na konstrukcji dwa przegubowe uchwyty kotwienia (pierwsza część "mount",
    // 4430) - liny na wysokości liny nośnej, przewodów na wysokości przewodów, każdy w parze uchwytów poz. 1 (karta 43-0453).
    // Za wyrównywaczem łańcuch każdego drutu osobno (…_each), dwie liny nośne przez messenger_equalizer (sieć 32, 35)
    bool fixed { false };
    // rolki szeregowe (karty 0421, 0471, 0480): przy konstrukcji duża rolka (pulley) i za nią mała (small_pulley) o small_spacing
    // dalej na uchwycie fixed_holder (4440-6), na wózku mała i duża (uchwyt 4440-5 = holder); lina od zaczepu na uchwycie stałym
    // (dead_end od osi dużej) przez małą ruchomą, małą stałą, dużą ruchomą i dużą stałą w dół. b z tablicy = oś dużej ruchomej.
    // Puste small_pulley = rolki równoległe (podwójne)
    std::string small_pulley, fixed_holder;
    // kotwienie rozdzielone: urządzenie tylko liny albo tylko przewodów (only), bez dźwigni - łańcuch wprost z uchwytu rolki
    // ruchomej; dwa takie składa urządzenie "kind split" (split <liny> <przewodów>). Prowadnica na ramach 5090 (frame <rama>
    // <od czoła do osi prowadnic> <prowadnica w Y> <strona stosu: -1 przy konstrukcji, +1 dalej> [owner]): stos pod liną (Y 0),
    // pierścień obrócony tak, żeby szekla trafiała w prowadnicę; owner stawia ramy i ich uchwyty
    std::string only;
    bool split { false };
    std::string split_messenger, split_contact;
    std::string frame;
    double frame_length { 0.0 }, frame_y { 0.0 };
    int frame_side { -1 };
    bool frame_owner { false };
    double messenger_spread { 0.08 };   // rozstaw dwóch lin za messenger_equalizer
    bool messenger_equalizer_z { false }; // otwory messenger_equalizer w Z modelu (dźwignia 6403), a nie w Y (wyrównywacz 4410)
    double small_spacing { 0.336 }, dead_end { 0.703 };
    std::vector<tensioner_link> messenger_each, contact_each;
    tensioner_link messenger_equalizer;
    // stare złożenia: izolator rolkowy ESN-2025 zamiast 7150 (Jan 2026-10-24: ESN-2025 - 4420 - [wyrównywacz] - uchwyt);
    // puste = jak nowe
    std::vector<tensioner_link> messenger_old, contact_old;
    tensioner_link messenger_clamp, contact_clamp;
    tensioner_link contact_clamp_old;   // uchwyt krańcowy przewodów w starych złożeniach (5110, stara karta 33-0401); pusty = jak nowe
    tensioner_link equalizer;           // wyrównywacz przewodów jezdnych (tylko przy dwóch przewodach)
    double equalizer_spread { 0.08 };   // rozstaw przewodów za wyrównywaczem
    // zabezpieczenie 4600 wyk. 1: dwa ramiona z osi rolek (stałej i ruchomej) zbiegające się na dolnej śrubie, para po obu stronach rolek
    std::string safety_fixed, safety_moving;
    double safety_arm { 1.0 };          // rozstaw otworów ramienia
    double safety_offset { 0.06 };      // odsunięcie każdej pary od środka rolki w bok
    // zabezpieczenie 4600 wyk. 2 (rolki szeregowe, karta 13-0421 poz. 10): ramiona z osi małych rolek (stałej i ruchomej) w dół,
    // na końcach drążek; puste = bez niego. safety_series <ramię przy stałej> <ramię przy ruchomej> <drążek> <ramię> <drążek> <w bok>
    std::string series_arm_fixed, series_arm_moving, series_bar;
    double series_arm { 0.46268 }, series_bar_length { 1.30 };
    // tytuł do wyboru w edytorze (karta z odmianą widoczna na liście mimo "-" w nazwie); puste = bez
    std::string title;
    // prowadnica ciężarów (5070, karta 33-0455) i pierścień prowadzący na wierzchu stosu (5080). Model prowadnicy: zero w osi
    // przesuwu pierścienia na dole, w +Z (długość), punkty "mount"/"end" mocowań do wsporników o guide_mount w -X (w stronę
    // konstrukcji). Pierścień: zero w osi stosu, poziomo, "mount" o ring_arm w -X (w osi przesuwu po prowadnicy); pusty = bez nich
    std::string guide, ring;
    double guide_length { 4.50 };
    double guide_mount { 0.07 };
    double guide_bottom { 0.80 };       // spód prowadnicy nad główką szyny (karta 23-0018-2: 0,80, prowadnica 4,50 do 5,30)
    double ring_arm { 0.200 };          // od osi stosu do osi szekli, która chodzi po prowadnicy (karta 23-5080 wyk. 1: 200)
    double ring_turn { 0.0 };           // stopnie wokół osi stosu: 0 = prowadnica w stronę konstrukcji, 90 = w bok (+Y)
    double ring_at { 1.00 };            // wysokość pierścienia nad spodem stosu (karta 23-0018-2: 1,00 m); ujemna = na wierzchu stosu
    // uchwyty do konstrukcji (karta 33-0455): para przód + tył ściśnięta śrubami przez słup, na ścianach wzdłuż drutów (obrócone
    // o 90 stopni względem wysięgnika; rozstaw śrub obejmuje słup w poprzek). Przód: zero na czole konstrukcji, śruby w +X modelu
    // (w głąb), hole_1 o hole_out przed czołem i hole_drop pod zerem. Tył: ten sam układ, po drugiej stronie.
    // Uchwyt przegubowy 4450 (pierwsza część "mount") trzymają dwa: górny wprost za mount_top, dolny odwrócony o 180 stopni za
    // mount_bottom (pivot_top / pivot_bottom nad / pod zerem 4450)
    struct bracket_set {
        std::vector<std::string> poles; // przedrostki nazw słupów, np. 1611-11 (najdłuższy pasujący wygrywa)
        double hole_out { 0.04 }, hole_drop { 0.04 }, bolt { 0.217 };
        std::string top_front, top_back, bottom_front, bottom_back;
    };
    std::vector<bracket_set> brackets;
    double pivot_top { 0.0 }, pivot_bottom { 0.0 };
    // otwór na końcu uchwytu (oś stałej rolki) nad jego zerem: uchwyt schodzi o tyle, żeby rolka została na swojej wysokości
    // (5040: hole_1 0,03 nad mount_bottom; wpis "pivot <góra> <dół> [rise <m>]")
    double mount_rise { 0.0 };
    bracket_set const *brackets_for( std::string const &Pole ) const;
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
    // numer typu sieci dla urządzeń: sieć bez własnych danych bierze dane innej (wpis "network <symbol> as <kod>"), inaczej jej kod
    int code_for( std::string const &Symbol, int Code ) const;

private:
    std::vector<tensioner_table> m_tables;
    std::vector<tensioner_weight> m_weights;
    std::vector<tensioner_pulley> m_pulleys;
    std::vector<tensioner_lever> m_levers;
    std::vector<tensioner_device> m_devices;
    std::map<std::string, double> m_lengths;
    std::map<std::string, int> m_networks;
};

struct tensioner_input {
    int type { 3 };                     // numer typu sieci (traction_code)
    int contacts { 2 };                 // liczba przewodów jezdnych
    int messengers { 1 };               // liczba lin nośnych (kotwienie stałe sieci 32, 35: dwie)
    double messenger_stack { 0.04 };    // odstęp pionowy dwóch lin nośnych w przęśle (druga celuje o tyle niżej)
    double temperature { 10.0 };        // °C
    double distance { 700.0 };          // m, od kotwienia stałego albo środkowego
    double elongation { 0.0 };          // m, trwałe wydłużenie sieci od ostatniej regulacji (rolka bliżej konstrukcji)
    int missing { 0 };                  // brakujące ciężary (z góry stosu)
    // sieć do skrócenia przy naciągu mniejszym niż od pełnego stosu (brakujące ciężary): sztywności EA wszystkich lin
    // nośnych i wszystkich przewodów jezdnych (daN), ciężar sieci na metr wiszący na linie (daN/m) i średnia rozpiętość
    // przęseł (m). Lina i przewody skracają się sprężyście, a lina dodatkowo zwisa bardziej, więc urządzenie ściąga je ku
    // przęsłom: rolka dalej od konstrukcji, ciężary wyżej. Sztywność 0 = bez tego (tablica jak przy pełnym stosie)
    double messenger_stiffness { 0.0 }, contact_stiffness { 0.0 };
    double load { 0.0 };
    double span { 0.0 };
    std::string color;                  // kolor ciężarów (tensioner_color); pusty = szary
    std::string top_color;              // kolor najwyższego ciężaru; pusty = jak reszta
    bool old_fittings { false };            // uchwyty krańcowe przewodów 5110 (druty zaśniedziałe)
    bool old_messenger { false }, old_contact { false }; // izolatory rolkowe (łańcuchy messenger_old / contact_old) na linie / przewodach
    // punkty, w które celują łańcuchy do liny nośnej i do przewodów jezdnych (ostatnie podwieszenie, układ urządzenia);
    // bez nich łańcuchy poziomo wzdłuż +X
    bool aimed { false };
    double messenger_target[ 3 ] { 0.0, 0.0, 0.0 };
    double contact_target[ 3 ] { 0.0, 0.0, 0.0 };        // stare złożenie (druty zaśniedziałe): uchwyty krańcowe przewodów 5110 zamiast 5120
    double height { 0.0 };              // oś rolek nad główką szyny; 0 = z tablicy. Wyżej/niżej: ciężary o tyle samo (lina ta sama)
    double rope_height { 0.0 };         // kotwienie stałe: uchwyt liny nośnej nad główką; 0 = jak drut przy ostatnim podwieszeniu
    double ground { -0.50 };            // najniższe położenie spodu ciężarów względem główki szyny (grunt, fundament); ZAŁOŻENIE
    double gravity { 0.981 };           // daN/kg
    // konstrukcja: nazwa słupa (dobór uchwytów) i jej grubość od czoła do tyłu (wzdłuż drutów; słup 161x - szerokość wzdłuż toru,
    // przez nią idą śruby uchwytów) na wysokości nad główką; puste = bez uchwytów
    std::string pole;
    std::function<double( double )> depth;
    // kąt osi konstrukcji (od czoła w stronę kotwionego przęsła) w układzie urządzenia, radiany wokół +Z: urządzenie od przegubu
    // 4450 ciągnie do drutów, a uchwyty, prowadnica, pierścień i stos zostają przy konstrukcji (0 = konstrukcja w linii drutów)
    double structure_angle { 0.0 };
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
    double height { 0.0 };                  // oś rolek nad główką szyny
    double lever_tilt { 0.0 };              // radiany: odchylenie uchwytu dźwigni od poziomu w dół (dźwignia od pionu w stronę konstrukcji u góry)
    // obrót krążków (radiany wokół +Y urządzenia) od położenia przy +10 °C: lina przewinięta przez rowek / promień
    double turn_fixed_a { 0.0 }, turn_fixed_b { 0.0 }, turn_moving_a { 0.0 }, turn_moving_b { 0.0 };
    int weights { 0 };                      // zawieszone ciężary
    double tension { 0.0 };                 // daN, całkowity naciąg sieci od ciężarów (zero, gdy ciężary stoją na gruncie)
    double nominal_tension { 0.0 };         // daN, naciąg od pełnego stosu (jak w tablicy)
    double shortening { 0.0 };              // m, o ile rolka odjechała od tablicy przez mniejszy naciąg (sprężystość i zwis)
    double messenger_tension { 0.0 }, contact_tension { 0.0 };
    double messenger_end[ 3 ] { 0.0, 0.0, 0.0 };         // gdzie kończy się lina nośna (uchwyt krańcowy)
    std::vector<std::vector<double>> rope2;  // kotwienie rozdzielone: lina urządzenia przewodów (rope - urządzenia liny)
    // dźwignie szeregowo: druga lina nośna (z drugiej dźwigni) i naciągi obu lin osobno (messenger_tension to ich suma)
    bool cascade { false };
    bool fixed { false };                   // kotwienie stałe (bez ciężarów; b, s, naciąg zero)
    double rope_height { 0.0 };             // kotwienie stałe: wysokość uchwytu liny nośnej (height - przewodów)
    double messenger2_end[ 3 ] { 0.0, 0.0, 0.0 };        // druga lina nośna: z drugiej dźwigni albo za wyrównywaczem kotwienia stałego
    double messenger_tensions[ 2 ] { 0.0, 0.0 };
    std::vector<std::vector<double>> contact_ends;       // gdzie kończą się przewody jezdne
    bool grounded { false };                // ciężary oparte o grunt: sieć bez naciągu od urządzenia
    // mocowania prowadnicy do wsporników na konstrukcji (dolne i górne, układ urządzenia); has_guide = false bez prowadnicy
    bool has_guide { false };
    double guide_mount[ 3 ] { 0.0, 0.0, 0.0 }, guide_end[ 3 ] { 0.0, 0.0, 0.0 };
    std::vector<std::string> warnings;
};

// składa urządzenie. false, gdy brakuje danych (urządzenia, typu, tablicy, rolki, dźwigni, otworów)
bool solve_tensioner( tensioner_catalog const &Catalog, tensioner_device const &Device, tensioner_input const &Input, tensioner_result &Result );

// wierzchołek bryły proceduralnej w układzie urządzenia; kolejne trójki to trójkąty, obchodzone przeciwnie do ruchu
// wskazówek zegara patrząc z zewnątrz (w prawoskrętnym układzie urządzenia)
struct tensioner_vertex {
    double position[ 3 ];
    double normal[ 3 ];
};

// lina jako jedna bryła proceduralna: rura o promieniu Radius i Sides ścianach po całej ścieżce Result.rope (łuki na
// rolkach co 5°), bez łączeń. Eye (gdy niepusty): ucho o długości EyeLength (zero w styku z liną, punkt "eye" w +Z w
// zaczepie) na obu końcach, dopisywane do Result.parts; rura skrócona o ucha
// Spacer: przekładka ślizgowa (6411) na sworzniu ucha przy pręcie ciężarów (koniec liny), oś wzdłuż sworznia; pusta = bez
void tensioner_rope_solid( tensioner_result &Result, double Radius, int Sides, std::string const &Eye, double EyeLength, std::vector<tensioner_vertex> &Triangles,
    std::string const &Spacer = {}, double SpacerLength = 0.0 );

} // namespace catenary
