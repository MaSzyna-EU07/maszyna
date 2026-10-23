#include "world/catenary_network.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <locale>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace catenary {

namespace {

constexpr double dan { 10.0 }; // naciągi w plikach są w daN, jak w katalogu

std::string trim( std::string const &Text ) {
    auto const first { Text.find_first_not_of( " \t\r\n" ) };
    if( first == std::string::npos ) { return {}; }
    auto const last { Text.find_last_not_of( " \t\r\n" ) };
    return Text.substr( first, last - first + 1 );
}

std::string strip_comment( std::string const &Line ) {
    auto const comment { Line.find( "//" ) };
    return comment == std::string::npos ? Line : Line.substr( 0, comment );
}

bool to_number( std::string const &Text, double &Value ) {
    std::istringstream stream { Text };
    stream.imbue( std::locale::classic() );
    stream >> Value;
    return false == stream.fail();
}

std::vector<double> to_numbers( std::string const &Text ) {
    std::vector<double> values;
    std::istringstream stream { Text };
    stream.imbue( std::locale::classic() );
    double value;
    while( stream >> value ) { values.push_back( value ); }
    return values;
}

// odstępy z reguły: odstęp skrajny przy podwieszeniach, środek podzielony równo na odcinki nie dłuższe od Maxgap
std::vector<double> rule_gaps( double const Length, double Endgap, double Maxgap ) {
    Endgap = std::max( Endgap, 0.5 );
    Maxgap = std::max( Maxgap, 1.0 );
    if( Length <= 2.0 * Endgap + 0.5 ) { return { 0.5 * Length, 0.5 * Length }; } // jeden wieszak w środku
    auto const inner { Length - 2.0 * Endgap };
    auto const count { std::max( 1, static_cast<int>( std::ceil( inner / Maxgap - 1e-9 ) ) ) };
    std::vector<double> gaps;
    gaps.push_back( Endgap );
    gaps.insert( gaps.end(), static_cast<std::size_t>( count ), inner / count );
    gaps.push_back( Endgap );
    return gaps;
}

} // namespace

type_registry &types() {
    static type_registry registry;
    return registry;
}

void type_registry::directory( std::string const &Path ) {
    std::lock_guard<std::mutex> lock { m_mutex };
    m_directory = Path;
    if( false == m_directory.empty() && m_directory.back() != '/' && m_directory.back() != '\\' ) { m_directory += '/'; }
}

void type_registry::report( std::string const &Message ) {
    if( m_reported[ Message ] ) { return; }
    m_reported[ Message ] = true;
    if( on_error ) { on_error( Message ); }
}

void type_registry::load_tables() {
    if( m_tablesloaded ) { return; }
    m_tablesloaded = true;
    auto const filename { m_directory + "traction_tables.txt" };
    std::ifstream file { filename };
    if( !file ) {
        report( "Catenary: can't open file \"" + filename + "\", hanger spacing and wire data unavailable" );
        return;
    }
    std::string line;
    int linenumber { 0 };
    while( std::getline( file, line ) ) {
        ++linenumber;
        line = trim( strip_comment( line ) );
        if( line.empty() || line[ 0 ] == '#' ) { continue; }
        std::istringstream stream { line };
        stream.imbue( std::locale::classic() );
        std::string kind, key;
        stream >> kind >> key;
        if( kind == "wire" ) {
            double mass { 0.0 }, diameter { 0.0 };
            stream >> mass >> diameter;
            if( stream.fail() || mass <= 0.0 ) { report( "Catenary: bad wire entry in \"" + filename + "\" (line " + std::to_string( linenumber ) + ")" ); continue; }
            m_wires[ key ] = { mass, diameter * 0.001 };
        }
        else if( kind == "spacing" ) {
            int span { 0 };
            stream >> span;
            std::vector<double> gaps;
            double gap;
            while( stream >> gap ) { gaps.push_back( gap ); }
            auto const sum { std::accumulate( gaps.begin(), gaps.end(), 0.0 ) };
            if( span <= 0 || gaps.size() < 2 || std::abs( sum - span ) > 0.01 ) {
                report( "Catenary: bad spacing entry in \"" + filename + "\" (line " + std::to_string( linenumber ) + "), gaps don't add up to the span length" );
                continue;
            }
            m_spacing[ key ][ span ] = gaps;
        }
        // pozostałe wiersze (np. "length" z tablic wzorcowych) nie są potrzebne w symulatorze
    }
}

std::unique_ptr<network_definition> type_registry::load_type( std::string const &Symbol ) {
    auto const filename { m_directory + "traction_" + Symbol + ".txt" };
    std::ifstream file { filename };
    if( !file ) {
        report( "Catenary: unknown network type \"" + Symbol + "\", can't open file \"" + filename + "\"" );
        return nullptr;
    }
    std::map<std::string, std::string> values;
    std::string line;
    while( std::getline( file, line ) ) {
        line = strip_comment( line );
        auto const colon { line.find( ':' ) };
        if( colon == std::string::npos ) { continue; } // nawiasy, puste wiersze
        values[ trim( line.substr( 0, colon ) ) ] = trim( line.substr( colon + 1 ) );
    }
    bool good { true };
    auto const fail = [ & ]( std::string const &Reason ) {
        report( "Catenary: network type \"" + Symbol + "\" (file \"" + filename + "\"): " + Reason );
        good = false;
    };
    auto const number = [ & ]( char const *Key, double &Value, bool const Required ) {
        auto const lookup { values.find( Key ) };
        if( lookup == values.end() || lookup->second.empty() ) {
            if( Required ) { fail( std::string( "missing entry \"" ) + Key + "\"" ); }
            return false;
        }
        if( false == to_number( lookup->second, Value ) ) { fail( std::string( "entry \"" ) + Key + "\" is not a number" ); return false; }
        return true;
    };
    auto const wire = [ & ]( char const *Key, wire_data &Wire, bool const Required ) {
        auto const lookup { values.find( Key ) };
        if( lookup == values.end() || lookup->second.empty() ) {
            if( Required ) { fail( std::string( "missing entry \"" ) + Key + "\"" ); }
            return false;
        }
        auto const data { m_wires.find( lookup->second ) };
        if( data == m_wires.end() ) { fail( "wire \"" + lookup->second + "\" isn't listed in traction_tables.txt" ); return false; }
        Wire = data->second;
        return true;
    };

    auto definition { std::make_unique<network_definition>() };
    auto &type { definition->type };
    type.symbol = Symbol;
    double value { 0.0 };
    if( number( "traction_code", value, false ) ) { type.code = static_cast<int>( value ); }
    if( number( "contact_wire_num", value, true ) ) { type.contact_count = static_cast<int>( value ); }
    if( number( "catenary_wire_num", value, true ) ) { type.messenger_count = static_cast<int>( value ); }
    if( wire( "contact_wire", definition->contact_wire, true ) ) { type.contact_mass = definition->contact_wire.mass; }
    if( wire( "catenary_wire", definition->messenger_wire, true ) ) { type.messenger_mass = definition->messenger_wire.mass; }
    if( number( "catenary_tension", value, true ) ) { type.messenger_tension = value * dan; }
    number( "contact_wire_spacing", definition->contact_spacing, false );
    number( "catenary_stack", type.messenger_stack, false );
    number( "hanger_mass", type.hanger_mass, false );
    number( "height", type.structure_height, true );
    number( "y_hanger_correction", type.y_hanger_drop, false );
    number( "y_junction_drop", type.y_junction_drop, false );
    number( "y_plan_offset", definition->y_plan_offset, false );
    if( number( "contact_tension", value, false ) ) { type.contact_tension = value * dan; }
    {
        // linka wieszaków: domyślnie L10, jak na kartach wieszaków
        auto const lookup { values.find( "hanger_wire" ) };
        auto const data { m_wires.find( lookup != values.end() && false == lookup->second.empty() ? lookup->second : "L10" ) };
        if( data != m_wires.end() ) { definition->hanger_wire = data->second; }
    }
    if( number( "y_tension", value, false ) && value > 0.0 ) {
        type.y_tension = value * dan;
        if( wire( "y_wire", definition->y_wire, true ) ) { type.y_mass = definition->y_wire.mass; }
        // pary: najmniejsza rozpiętość, długość Y; zapisane malejąco wg rozpiętości
        auto const lookup { values.find( "y_length" ) };
        auto const pairs { lookup != values.end() ? to_numbers( lookup->second ) : std::vector<double>{} };
        if( pairs.empty() || pairs.size() % 2 != 0 ) { fail( "entry \"y_length\" needs pairs of numbers: minimum span and Y length" ); }
        for( std::size_t index = 0; index + 1 < pairs.size(); index += 2 ) { type.y_length_by_span.emplace_back( pairs[ index ], pairs[ index + 1 ] ); }
        std::sort( type.y_length_by_span.begin(), type.y_length_by_span.end(), []( auto const &Left, auto const &Right ) { return Left.first > Right.first; } );
        if( type.y_tension >= type.messenger_tension ) { fail( "\"y_tension\" has to be lower than \"catenary_tension\"" ); }
    }
    {
        auto const lookup { values.find( "hanger_table" ) };
        if( lookup == values.end() || lookup->second.empty() ) { fail( "missing entry \"hanger_table\"" ); }
        else {
            definition->hanger_table = lookup->second;
            if( m_spacing.find( definition->hanger_table ) == m_spacing.end() ) { fail( "hanger table \"" + definition->hanger_table + "\" isn't listed in traction_tables.txt" ); }
        }
    }
    if( good ) {
        if( type.contact_count < 1 || type.contact_count > 2 ) { fail( "\"contact_wire_num\" has to be 1 or 2" ); }
        if( type.messenger_count < 1 || type.messenger_count > 2 ) { fail( "\"catenary_wire_num\" has to be 1 or 2" ); }
        if( type.messenger_tension <= 0.0 ) { fail( "\"catenary_tension\" has to be greater than zero" ); }
    }
    if( false == good ) { return nullptr; }
    return definition;
}

network_definition const *type_registry::find( std::string const &Symbol ) {
    std::lock_guard<std::mutex> lock { m_mutex };
    auto const lookup { m_types.find( Symbol ) };
    if( lookup != m_types.end() ) { return lookup->second.get(); }
    load_tables();
    auto &slot { m_types[ Symbol ] };
    slot = load_type( Symbol );
    return slot.get();
}

std::vector<double> type_registry::hanger_gaps( std::string const &Table, double const Length ) {
    std::lock_guard<std::mutex> lock { m_mutex };
    load_tables();
    auto const table { m_spacing.find( Table ) };
    if( table == m_spacing.end() || table->second.empty() ) { return rule_gaps( Length, 2.0, 4.0 ); }
    auto const &rows { table->second };
    auto const span { static_cast<int>( std::lround( Length ) ) };
    auto const row { rows.find( span ) };
    if( row == rows.end() ) {
        // rozpiętość spoza tablicy: reguła oparta na najbliższym wierszu
        auto nearest { rows.lower_bound( span ) };
        if( nearest == rows.end() ) { --nearest; }
        else if( nearest != rows.begin() ) {
            auto const previous { std::prev( nearest ) };
            if( span - previous->first < nearest->first - span ) { nearest = previous; }
        }
        auto const &gaps { nearest->second };
        return rule_gaps( Length, gaps.front(), *std::max_element( gaps.begin(), gaps.end() ) );
    }
    auto gaps { row->second };
    // różnicę do rozpiętości całkowitej przejmują po równo największe (środkowe) odstępy
    auto const difference { Length - std::accumulate( gaps.begin(), gaps.end(), 0.0 ) };
    auto const largest { *std::max_element( gaps.begin(), gaps.end() ) };
    auto const count { std::count_if( gaps.begin(), gaps.end(), [ & ]( double const Gap ) { return std::abs( Gap - largest ) < 1e-6; } ) };
    for( auto &gap : gaps ) {
        if( std::abs( gap - largest ) < 1e-6 ) { gap += difference / static_cast<double>( count ); }
    }
    return gaps;
}

bool solve_span_points( network_definition const &Definition, std::vector<double> const &Hangergaps, span_points const &Points,
                        bool const Ystart, bool const Yend, span_solution &Solution, std::string &Error ) {

    auto const &type { Definition.type };
    auto const &c0 { Points.contact_start }, &c1 { Points.contact_end }, &m0 { Points.messenger_start }, &m1 { Points.messenger_end };
    auto const length { std::hypot( c1.x - c0.x, c1.z - c0.z ) };
    if( length < 1.0 ) { Error = "span shorter than 1 m"; return false; }

    span_input input;
    input.length = length;
    input.hanger_gaps = Hangergaps;
    input.height_start = std::max( 0.0, m0.y - c0.y );
    input.height_end = std::max( 0.0, m1.y - c1.y );
    input.y_at_start = Ystart;
    input.y_at_end = Yend;
    span_result result;
    try {
        // bez rozstawu wieszaków: odcinek do kotwienia, same liny
        result = ( Hangergaps.empty() ?
            solve_free_span( type, length, input.height_start, input.height_end ) :
            solve_span( type, input, 4.0 ) );
    }
    catch( std::exception const &exception ) {
        Error = exception.what();
        return false;
    }
    // lina poniżej przewodu jezdnego: przęsło za długie albo za mała wysokość konstrukcyjna dla tego typu sieci
    for( auto const *line : { &result.messenger, &result.y_start, &result.y_end } ) {
        for( auto const &point : *line ) {
            if( point.y - ( type.messenger_count > 1 ? type.messenger_stack : 0.0 ) < -0.01 ) {
                Error = "messenger wire would hang below the contact wire (span too long or suspension too low for this network type)";
                return false;
            }
        }
    }
    Solution.length = length;
    Solution.result = std::move( result );
    return true;
}

void join_y( network_definition const &Definition, span_solution &Solution, bool const Atend, y_half const &Neighbour ) {
    join_y( Definition.type, Solution.result, Solution.length, Atend, Neighbour );
}

bool build_span_lines( network_definition const &Definition, std::vector<double> const &Hangergaps, span_points const &Points,
                       double const Fallbackoffset, std::vector<vec3> &Output, std::string &Error ) {
    span_solution solution;
    if( false == solve_span_points( Definition, Hangergaps, Points, true, true, solution, Error ) ) { return false; }
    build_lines( Definition, solution, Points, Fallbackoffset, Output );
    return true;
}

y_side make_y_side( span_solution const &Solution, span_points const &Points, bool const Atend ) {

    y_side side;
    auto const &half { Atend ? Solution.result.y_half_end : Solution.result.y_half_start };
    if( half.reach <= 0.0 ) { return side; }
    auto const &m0 { Points.messenger_start }, &m1 { Points.messenger_end };
    auto const t { ( Atend ? Solution.length - half.reach : half.reach ) / Solution.length };
    side.valid = true;
    side.reach = half.reach;
    side.axis = ( Atend ? m1 : m0 );
    side.junction = { m0.x + ( m1.x - m0.x ) * t, m0.y + ( m1.y - m0.y ) * t, m0.z + ( m1.z - m0.z ) * t };
    return side;
}

vec3 y_plan::point( double const Distance ) const {
    // łuk drugiego stopnia; parametr wg odległości wzdłuż liny nośnej
    auto const t { std::clamp( ( reach - Distance ) / total, 0.0, 1.0 ) };
    auto const a { ( 1.0 - t ) * ( 1.0 - t ) }, b { 2.0 * t * ( 1.0 - t ) }, c { t * t };
    return { a * start.x + b * control.x + c * end.x, 0.0, a * start.z + b * control.z + c * end.z };
}

y_plan solve_y_plan( network_definition const &Definition, y_side const &Mine, y_side const &Other ) {

    y_plan plan;
    if( false == Mine.valid || false == Other.valid ) { return plan; }
    plan.reach = Mine.reach;
    plan.total = Mine.reach + Other.reach;
    plan.start = Mine.junction;
    plan.end = Other.junction;
    // załom liny nośnej w osi podwieszenia i środek cięciwy zacisk–zacisk
    vec3 const axis { 0.5 * ( Mine.axis.x + Other.axis.x ), 0.0, 0.5 * ( Mine.axis.z + Other.axis.z ) };
    vec3 const middle { 0.5 * ( plan.start.x + plan.end.x ), 0.0, 0.5 * ( plan.start.z + plan.end.z ) };
    // środek łuku leży w połowie drogi między punktem kierującym a środkiem cięciwy,
    // więc punkt kierujący jest odsunięty od środka cięciwy o 2 ( 1 - y_plan_offset ) odległości do załomu
    auto const weight { 2.0 * ( 1.0 - std::clamp( Definition.y_plan_offset, 0.0, 1.0 ) ) };
    plan.control = { middle.x + ( axis.x - middle.x ) * weight, 0.0, middle.z + ( axis.z - middle.z ) * weight };
    plan.valid = true;
    return plan;
}

double wire_diameter( wire_data const &Wire ) {
    if( Wire.diameter > 0.0 ) { return Wire.diameter; }
    if( Wire.mass <= 0.0 ) { return 0.004; }
    // linka skręcana z miedzi: przekrój z masy, wypełnienie obrysu ok. 0,76
    constexpr double pi { 3.14159265358979323846 };
    return std::sqrt( 4.0 * ( Wire.mass / 8900.0 ) / ( pi * 0.76 ) );
}

void build_geometry( network_definition const &Definition, span_solution const &Solution, span_points const &Points,
                     double const Fallbackoffset, span_geometry &Output, y_plan const &Ystart, y_plan const &Yend ) {

    auto const &type { Definition.type };
    auto const &c0 { Points.contact_start }, &c1 { Points.contact_end }, &m0 { Points.messenger_start }, &m1 { Points.messenger_end };
    auto const length { Solution.length };
    auto const &result { Solution.result };

    // kierunek przęsła w poziomie i wektor poziomy w bok od osi przęsła
    vec3 const along { ( c1.x - c0.x ) / length, 0.0, ( c1.z - c0.z ) / length };
    vec3 const side { along.z, 0.0, -along.x };
    auto const offset { type.contact_count > 1 ? ( Definition.contact_spacing > 0.0 ? 0.5 * Definition.contact_spacing : Fallbackoffset ) : 0.0 };
    // punkt przewodu jezdnego w odległości X od początku, przesunięty w bok o Shift
    auto const contact = [ & ]( double const X, double const Shift ) {
        auto const t { X / length };
        return vec3 { c0.x + ( c1.x - c0.x ) * t + side.x * Shift, c0.y + ( c1.y - c0.y ) * t, c0.z + ( c1.z - c0.z ) * t + side.z * Shift };
    };
    // punkt liny: w poziomie na cięciwie liny nośnej, w pionie Height nad przewodem jezdnym
    auto const rope = [ & ]( double const X, double const Height ) {
        auto const t { X / length };
        return vec3 { m0.x + ( m1.x - m0.x ) * t, c0.y + ( c1.y - c0.y ) * t + Height, m0.z + ( m1.z - m0.z ) * t };
    };
    // punkt linki Y: w planie wg policzonego przebiegu, a gdy go nie ma (albo w zacisku) na cięciwie liny nośnej
    auto const yrope = [ & ]( double const X, double const Height, y_plan const &Plan, bool const Atend ) {
        auto point { rope( X, Height ) };
        if( false == Plan.valid ) { return point; }
        auto const plan { Plan.point( Atend ? length - X : X ) };
        point.x = plan.x;
        point.z = plan.z;
        return point;
    };
    // linka Y przy jednym podwieszeniu; na łuku dzielona na krótsze odcinki, żeby przebieg w planie był gładki
    auto const yline = [ & ]( std::vector<point2> const &Line, y_plan const &Plan, bool const Atend ) {
        std::vector<vec3> points;
        for( std::size_t index = 0; index < Line.size(); ++index ) {
            if( index > 0 && Plan.valid ) {
                auto const &from { Line[ index - 1 ] }, &to { Line[ index ] };
                auto const pieces { static_cast<int>( std::ceil( ( to.x - from.x ) / 1.0 - 1e-9 ) ) };
                for( int piece = 1; piece < pieces; ++piece ) {
                    auto const t { static_cast<double>( piece ) / pieces };
                    points.push_back( yrope( from.x + ( to.x - from.x ) * t, from.y + ( to.y - from.y ) * t, Plan, Atend ) );
                }
            }
            points.push_back( yrope( Line[ index ].x, Line[ index ].y, Plan, Atend ) );
        }
        return points;
    };
    auto const polyline = [ & ]( std::vector<vec3> const &Line, element_kind const Kind ) {
        for( std::size_t index = 0; index + 1 < Line.size(); ++index ) {
            Output.elements.push_back( { Line[ index ], Line[ index + 1 ], Kind } );
        }
    };
    auto const reachstart { result.y_half_start.reach }, reachend { result.y_half_end.reach };

    // przewody jezdne; w przęśle bez wieszaków ze zwisem, o ile typ podaje ich naciąg
    for( int wire = 0; wire < type.contact_count; ++wire ) {
        auto const shift { type.contact_count > 1 ? ( wire == 0 ? -offset : offset ) : 0.0 };
        std::vector<vec3> line;
        if( result.contact.size() > 1 ) {
            for( auto const &point : result.contact ) {
                auto position { contact( point.x, shift ) };
                position.y += point.y;
                line.push_back( position );
            }
        }
        else {
            line = { contact( 0.0, shift ), contact( length, shift ) };
        }
        polyline( line, element_kind::contact );
    }
    // górna lina nośna
    {
        std::vector<vec3> line;
        for( auto const &point : result.messenger ) { line.push_back( rope( point.x, point.y ) ); }
        polyline( line, element_kind::messenger );
    }
    // linki Y, a przy dwóch linach dolna lina nośna: przy podwieszeniu przebieg jak linka Y, dalej pod górną liną
    {
        auto const start { yline( result.y_start, Ystart, false ) };
        auto const end { yline( result.y_end, Yend, true ) };
        polyline( start, element_kind::y_rope );
        if( type.messenger_count > 1 ) {
            std::vector<vec3> middle;
            if( false == start.empty() ) { middle.push_back( start.back() ); }
            for( auto const &point : result.messenger ) {
                auto const instart { reachstart > 0.0 && point.x < reachstart + 1e-6 };
                auto const inend { reachend > 0.0 && point.x > length - reachend - 1e-6 };
                if( false == instart && false == inend ) { middle.push_back( rope( point.x, point.y - type.messenger_stack ) ); }
            }
            if( false == end.empty() ) { middle.push_back( end.front() ); }
            polyline( middle, element_kind::messenger );
        }
        polyline( end, element_kind::y_rope );
        // zaciski linki Y na linie nośnej
        if( false == start.empty() ) { Output.fittings.push_back( { start.back(), along, fitting_kind::y_clamp } ); }
        if( false == end.empty() ) { Output.fittings.push_back( { end.front(), along, fitting_kind::y_clamp } ); }
    }
    // wieszaki, przy dwóch przewodach jezdnych na przemian
    for( std::size_t index = 0; index < result.hangers.size(); ++index ) {
        auto const &hanger { result.hangers[ index ] };
        auto const top { hanger.length + ( hanger.on_y ? type.y_hanger_drop : 0.0 ) };
        if( top <= 0.01 ) { continue; } // lina przy samym przewodzie, wieszaka nie ma
        auto const shift { type.contact_count > 1 ? ( index % 2 == 0 ? -offset : offset ) : 0.0 };
        auto const atstart { hanger.x < 0.5 * length };
        auto const upper { hanger.on_y ? yrope( hanger.x, top, atstart ? Ystart : Yend, false == atstart ) : rope( hanger.x, top ) };
        auto const lower { contact( hanger.x, shift ) };
        Output.elements.push_back( { upper, lower, element_kind::hanger } );
        Output.fittings.push_back( { upper, along, fitting_kind::rope_clamp } );
        Output.fittings.push_back( { lower, along, fitting_kind::wire_clamp } );
        // przy dwóch linach nośnych: łącznik między liną dolną a górną w każdym punkcie wieszaka,
        // poza strefą przy podwieszeniu, gdzie dolna lina odchodzi od górnej i sama niesie wieszaki
        if( type.messenger_count > 1 && false == hanger.on_y && type.messenger_stack > 0.0 ) {
            auto const above { rope( hanger.x, top + type.messenger_stack ) };
            Output.elements.push_back( { upper, above, element_kind::link } );
            Output.fittings.push_back( { { upper.x, 0.5 * ( upper.y + above.y ), upper.z }, along, fitting_kind::link_clamp } );
        }
    }
}

void build_lines( network_definition const &Definition, span_solution const &Solution, span_points const &Points,
                  double const Fallbackoffset, std::vector<vec3> &Output, y_plan const &Ystart, y_plan const &Yend ) {
    span_geometry geometry;
    build_geometry( Definition, Solution, Points, Fallbackoffset, geometry, Ystart, Yend );
    Output.reserve( Output.size() + 2 * geometry.elements.size() );
    for( auto const &element : geometry.elements ) {
        Output.push_back( element.from );
        Output.push_back( element.to );
    }
}

namespace {

vec3 operator+( vec3 const &Left, vec3 const &Right ) { return { Left.x + Right.x, Left.y + Right.y, Left.z + Right.z }; }
vec3 operator-( vec3 const &Left, vec3 const &Right ) { return { Left.x - Right.x, Left.y - Right.y, Left.z - Right.z }; }
vec3 operator*( vec3 const &Left, double const Right ) { return { Left.x * Right, Left.y * Right, Left.z * Right }; }
vec3 cross( vec3 const &Left, vec3 const &Right ) { return { Left.y * Right.z - Left.z * Right.y, Left.z * Right.x - Left.x * Right.z, Left.x * Right.y - Left.y * Right.x }; }
double norm( vec3 const &Vector ) { return std::sqrt( Vector.x * Vector.x + Vector.y * Vector.y + Vector.z * Vector.z ); }

// ściany boczne graniastosłupa wokół odcinka; trójkąty zwrócone przodem na zewnątrz (kolejność przeciwna do ruchu wskazówek zegara),
// normalne promieniowe, żeby graniastosłup cieniował się jak walec
void add_prism( std::vector<solid_vertex> &Output, vec3 const &From, vec3 const &To, double const Radius, int const Sides ) {
    constexpr double pi { 3.14159265358979323846 };
    auto axis { To - From };
    auto const length { norm( axis ) };
    if( length < 1e-6 ) { return; }
    axis = axis * ( 1.0 / length );
    auto const reference { std::abs( axis.y ) < 0.9 ? vec3 { 0.0, 1.0, 0.0 } : vec3 { 1.0, 0.0, 0.0 } };
    auto u { cross( axis, reference ) };
    u = u * ( 1.0 / norm( u ) );
    auto const v { cross( axis, u ) };
    for( int side = 0; side < Sides; ++side ) {
        auto const a0 { 2.0 * pi * side / Sides }, a1 { 2.0 * pi * ( side + 1 ) / Sides };
        auto const n0 { u * std::cos( a0 ) + v * std::sin( a0 ) }, n1 { u * std::cos( a1 ) + v * std::sin( a1 ) };
        solid_vertex const f0 { From + n0 * Radius, n0 }, f1 { From + n1 * Radius, n1 }, t0 { To + n0 * Radius, n0 }, t1 { To + n1 * Radius, n1 };
        Output.insert( Output.end(), { f0, f1, t0,  f1, t1, t0 } );
    }
}

// prostopadłościan o środku Center; Length wzdłuż Along, Width w bok, Height w pionie
void add_box( std::vector<solid_vertex> &Output, vec3 const &Center, vec3 const &Along, double const Length, double const Width, double const Height ) {
    vec3 const side { Along.z, 0.0, -Along.x }, up { 0.0, 1.0, 0.0 };
    vec3 const a { Along * ( 0.5 * Length ) }, s { side * ( 0.5 * Width ) }, h { up * ( 0.5 * Height ) };
    // narożniki: bit 0 wzdłuż, bit 1 w bok, bit 2 w górę
    vec3 corner[ 8 ];
    for( int index = 0; index < 8; ++index ) {
        corner[ index ] = Center + a * ( index & 1 ? 1.0 : -1.0 ) + s * ( index & 2 ? 1.0 : -1.0 ) + h * ( index & 4 ? 1.0 : -1.0 );
    }
    // ściany jako czwórki narożników obchodzonych przeciwnie do ruchu wskazówek zegara, patrząc z zewnątrz, i ich normalne
    int const faces[ 6 ][ 4 ] { { 1, 3, 7, 5 }, { 0, 4, 6, 2 }, { 2, 6, 7, 3 }, { 0, 1, 5, 4 }, { 4, 5, 7, 6 }, { 0, 2, 3, 1 } };
    vec3 const normals[ 6 ] { Along, Along * -1.0, side, side * -1.0, up, up * -1.0 };
    for( int face = 0; face < 6; ++face ) {
        auto const &f { faces[ face ] };
        auto const &n { normals[ face ] };
        Output.insert( Output.end(), { { corner[ f[ 0 ] ], n }, { corner[ f[ 1 ] ], n }, { corner[ f[ 2 ] ], n },  { corner[ f[ 0 ] ], n }, { corner[ f[ 2 ] ], n }, { corner[ f[ 3 ] ], n } } );
    }
}

} // namespace

void build_solid( network_definition const &Definition, span_geometry const &Geometry, std::vector<solid_vertex> &Wires, std::vector<solid_vertex> &Fittings ) {

    auto const contactradius { 0.5 * wire_diameter( Definition.contact_wire ) };
    auto const messengerradius { 0.5 * wire_diameter( Definition.messenger_wire ) };
    auto const yradius { 0.5 * wire_diameter( Definition.y_wire ) };
    auto const hangerradius { 0.5 * wire_diameter( Definition.hanger_wire ) };
    for( auto const &element : Geometry.elements ) {
        switch( element.kind ) {
            case element_kind::contact:   { add_prism( Wires, element.from, element.to, contactradius, 6 ); break; }
            case element_kind::messenger: { add_prism( Wires, element.from, element.to, messengerradius, 6 ); break; }
            case element_kind::y_rope:    { add_prism( Wires, element.from, element.to, yradius, 5 ); break; }
            case element_kind::hanger:    { add_prism( Wires, element.from, element.to, hangerradius, 4 ); break; }
            case element_kind::link:      { add_prism( Wires, element.from, element.to, hangerradius, 4 ); break; }
        }
    }
    // bryły zastępcze w miejscach połączeń; wymiary przybliżone, do zastąpienia modelami uchwytów z katalogu
    auto const stack { Definition.type.messenger_stack };
    for( auto const &fitting : Geometry.fittings ) {
        auto center { fitting.position };
        switch( fitting.kind ) {
            case fitting_kind::rope_clamp: { center.y -= 0.015; add_box( Fittings, center, fitting.along, 0.05, 0.03, 0.06 ); break; }  // uchwyt wieszaka na linie (5440)
            case fitting_kind::wire_clamp: { center.y += 0.015; add_box( Fittings, center, fitting.along, 0.04, 0.025, 0.05 ); break; } // uchwyt wieszaka na przewodzie (5420)
            case fitting_kind::y_clamp:    { add_box( Fittings, center, fitting.along, 0.08, 0.035, 0.05 ); break; }                    // zacisk linki Y na linie nośnej
            case fitting_kind::link_clamp: { add_box( Fittings, center, fitting.along, 0.05, 0.03, stack + 0.04 ); break; }             // zacisk łączący dwie liny (5430)
        }
    }
}

} // namespace catenary
