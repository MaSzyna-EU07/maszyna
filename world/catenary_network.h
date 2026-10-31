// Sieć trakcyjna: katalog typów sieci czytany z plików danych oraz geometria przęsła liczona solverem.
// Kod niezależny od reszty symulatora (C++17, tylko biblioteka standardowa), żeby dało się go testować osobno.
#pragma once
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "world/catenary_solver.h"

namespace catenary {

// przewód albo lina z tabeli przewodów
struct wire_data {
    double mass { 0.0 };        // kg/m
    double diameter { 0.0 };    // m
    double area { 0.0 };        // m², przekrój czynny (do wydłużeń sprężystych); 0 = nieznany
    double modulus { 0.0 };     // Pa, moduł sprężystości przewodu (liny skręcane: niższy niż drutu); 0 = nieznany
};

// pełny opis typu sieci: dane dla solvera i dane potrzebne tylko do rysowania
struct network_definition {
    network_type type;
    std::string  hanger_table;              // nazwa tablicy rozstawu wieszaków
    double       contact_spacing { 0.0 };   // m, rozstaw dwóch przewodów jezdnych; 0 = wziąć ze scenerii
    wire_data    contact_wire;
    wire_data    messenger_wire;
    wire_data    y_wire;
    wire_data    hanger_wire;               // linka wieszaków
    double       y_plan_offset { 0.5 };     // przebieg linki Y w planie na łuku: 0 = przez oś podwieszenia, 1 = prosto od zacisku do zacisku
};

// katalog typów sieci. Pliki są czytane przy pierwszym użyciu; każdy błąd zgłaszany jest tylko raz
class type_registry {
public:
    // katalog z plikami traction_<symbol>.txt i traction_tables.txt
    void directory( std::string const &Path );
    // odbiorca komunikatów o błędach w danych
    std::function<void( std::string const & )> on_error;
    // zwraca opis typu albo nullptr, gdy pliku nie ma lub jest niekompletny. Wskaźnik pozostaje ważny do końca programu
    network_definition const *find( std::string const &Symbol );
    // odstępy wieszaków dla przęsła o podanej rozpiętości: oś podwieszenia–1, 1–2, …, ostatni–oś podwieszenia.
    // rozpiętość niecałkowita: wiersz najbliższej całkowitej z rozciągniętymi środkowymi odstępami;
    // rozpiętość spoza tablicy: odstępy wyznaczone regułą z odstępem skrajnym i największym odstępem z najbliższego wiersza
    std::vector<double> hanger_gaps( std::string const &Table, double Length );

private:
    void load_tables();
    std::unique_ptr<network_definition> load_type( std::string const &Symbol );
    void report( std::string const &Message );

    std::mutex m_mutex;
    std::string m_directory { "data/" };
    bool m_tablesloaded { false };
    std::map<std::string, wire_data> m_wires;
    std::map<std::string, std::map<int, std::vector<double>>> m_spacing;
    std::map<std::string, std::unique_ptr<network_definition>> m_types; // nullptr = typ błędny, już zgłoszony
    std::map<std::string, bool> m_reported;
};

// wspólny katalog typów
type_registry &types();

struct vec3 { double x, y, z; };

// położenie przęsła w scenerii: końce przewodu jezdnego (osi sieci) i końce liny nośnej
struct span_points {
    vec3 contact_start, contact_end;
    vec3 messenger_start, messenger_end;
    // druga (dolna) lina nośna kończy się gdzie indziej niż o odstęp lin pod pierwszą (uchwyt krańcowy w kotwieniu z dwiema liniami)
    bool messenger2_start_set { false }, messenger2_end_set { false };
    vec3 messenger2_start, messenger2_end;
};

// policzone przęsło
struct span_solution {
    double      length { 0.0 };     // m, rozpiętość w poziomie
    span_result result;
};

// liczy przęsło. Ystart, Yend: czy przy danym podwieszeniu jest linka Y (o ile typ ją przewiduje dla tej rozpiętości).
// pusty Hangergaps oznacza odcinek bez wieszaków (do kotwienia albo urządzenia naprężającego): same liny pod własnym ciężarem
// zwraca false i opis w Error, gdy przęsła nie da się policzyć
bool solve_span_points( network_definition const &Definition, std::vector<double> const &Hangergaps, span_points const &Points,
                        bool Ystart, bool Yend, span_solution &Solution, std::string &Error );

// uzgadnia linkę Y przy jednym podwieszeniu z połową linki z sąsiedniego przęsła (patrz join_y w solverze)
void join_y( network_definition const &Definition, span_solution &Solution, bool Atend, y_half const &Neighbour );

// połowa linki Y w układzie sceny: dane do wyznaczenia przebiegu linki w planie razem z sąsiednim przęsłem
struct y_side {
    bool   valid { false };     // false = przy tym podwieszeniu nie ma linki Y
    double reach { 0.0 };       // m, odległość zacisku od osi podwieszenia
    vec3   junction;            // zacisk na linie nośnej
    vec3   axis;                // punkt liny nośnej w osi podwieszenia
};

// przebieg linki Y w planie po jednej stronie podwieszenia: łagodny łuk od zacisku do zacisku
struct y_plan {
    bool   valid { false };     // false = linka w planie pod cięciwą liny nośnej
    double reach { 0.0 };       // m, odległość zacisku w tym przęśle od osi podwieszenia
    double total { 0.0 };       // m, odległość między zaciskami mierzona wzdłuż liny nośnej
    vec3   start, control, end; // łuk: zacisk w tym przęśle, punkt kierujący, zacisk w sąsiednim (liczą się x i z)
    // punkt linki w odległości Distance od osi podwieszenia, w głąb tego przęsła
    vec3 point( double Distance ) const;
};

// zbiera dane połowy linki Y z policzonego przęsła
y_side make_y_side( span_solution const &Solution, span_points const &Points, bool Atend );

// wyznacza przebieg linki Y w planie. Na łuku lina nośna łamie się w osi podwieszenia, a linka Y, zaciśnięta na linie
// po obu stronach, przechodzi obok tego załomu łagodnym łukiem. O tym, jak daleko od załomu, decyduje wpis typu
// y_plan_offset: 0 = linka przechodzi przez oś podwieszenia, 1 = biegnie prosto od zacisku do zacisku.
// Obie strony liczone z tych samych danych dają ten sam przebieg
y_plan solve_y_plan( network_definition const &Definition, y_side const &Mine, y_side const &Other );

// elementy przęsła do narysowania
enum class element_kind { contact, messenger, y_rope, hanger, link };
enum class fitting_kind { rope_clamp, wire_clamp, y_clamp, link_clamp };

// odcinek przewodu, liny albo linki
struct span_element {
    vec3 from, to;
    element_kind kind;
};

// miejsce połączenia: uchwyt wieszaka na linie i na przewodzie, zacisk linki Y na linie nośnej, zacisk łączący dwie liny
struct span_fitting {
    vec3 position;
    vec3 along;         // kierunek przewodu w tym miejscu, poziomy, jednostkowy
    fitting_kind kind;
};

struct span_geometry {
    std::vector<span_element> elements;
    std::vector<span_fitting> fittings;
};

// zbiera elementy przęsła w układzie sceny.
// Fallbackoffset: połowa rozstawu przewodów jezdnych ze scenerii, używana gdy typ nie podaje rozstawu.
// Ystart, Yend: przebieg linek Y w planie; domyślnie pod cięciwą liny nośnej
void build_geometry( network_definition const &Definition, span_solution const &Solution, span_points const &Points,
                     double Fallbackoffset, span_geometry &Output, y_plan const &Ystart = {}, y_plan const &Yend = {} );

// odcinki do narysowania przęsła liniami: kolejne pary punktów w Output
void build_lines( network_definition const &Definition, span_solution const &Solution, span_points const &Points,
                  double Fallbackoffset, std::vector<vec3> &Output, y_plan const &Ystart = {}, y_plan const &Yend = {} );

// wierzchołek bryły: położenie i wektor normalny do cieniowania
struct solid_vertex {
    vec3 position;
    vec3 normal;
};

// bryły do oglądania z bliska: przewody i liny jako graniastosłupy o średnicach z tabeli przewodów (Wires), w miejscach
// połączeń prostopadłościany zastępcze, do zastąpienia modelami uchwytów (Fittings). Kolejne trójki wierzchołków to trójkąty.
// Przewody i osprzęt są osobno, bo rysowane są z różnych materiałów
void build_solid( network_definition const &Definition, span_geometry const &Geometry, std::vector<solid_vertex> &Wires, std::vector<solid_vertex> &Fittings );

// średnica przewodu: z tabeli, a gdy jej tam nie ma, oszacowana z masy jak dla linki miedzianej
double wire_diameter( wire_data const &Wire );

// to samo w jednym kroku dla przęsła liczonego osobno, z linką Y przy obu podwieszeniach
bool build_span_lines( network_definition const &Definition, std::vector<double> const &Hangergaps, span_points const &Points,
                       double Fallbackoffset, std::vector<vec3> &Output, std::string &Error );

} // namespace catenary
