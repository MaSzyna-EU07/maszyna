#include "world/catenary_solver.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace catenary {

double network_type::y_length( double const Span ) const {
    for( auto const &entry : y_length_by_span ) {
        if( Span >= entry.first ) { return entry.second; }
    }
    return 0.0;
}

namespace {

struct point_load { double x; double force; };

// cięgno o odcinkowo stałym naciągu poziomym, obciążone ciężarem ciągłym (odcinkowo stałym) i siłami skupionymi.
// wysokości obu końców są dane; reakcja na początku wynika z warunku na różnicę wysokości.
class cable {
public:
    cable( double Length, double Ystart, double Yend ) : m_length( Length ), m_ystart( Ystart ), m_yend( Yend ) {}
    // strefy muszą pokrywać całą długość bez przerw, w kolejności rosnącego x
    void add_zone( double X0, double X1, double Tension, double Weight ) { m_zones.push_back( { X0, X1, Tension, Weight } ); }
    void add_load( double X, double Force ) { m_loads.push_back( { X, Force } ); }
    void solve() {
        std::sort( m_loads.begin(), m_loads.end(), []( point_load const &L, point_load const &R ) { return L.x < R.x; } );
        // punkty podziału: granice stref i siły skupione
        m_nodes.clear();
        m_nodes.push_back( 0.0 );
        for( auto const &zone : m_zones ) { m_nodes.push_back( zone.x1 ); }
        for( auto const &load : m_loads ) { m_nodes.push_back( load.x ); }
        std::sort( m_nodes.begin(), m_nodes.end() );
        m_nodes.erase( std::unique( m_nodes.begin(), m_nodes.end(), []( double L, double R ) { return std::abs( L - R ) < 1e-9; } ), m_nodes.end() );
        // y(L) - y(0) = suma po odcinkach całki (R - W(x)) / H ; W(x) = obciążenie na lewo od x
        // na odcinku bez sił skupionych: W(x) = W0 + w (x - x0)  ->  całka = [ (R - W0) d - w d^2 / 2 ] / H
        double sumcoefficient { 0.0 }, sumconstant { 0.0 }, loadleft { 0.0 };
        m_segments.clear();
        for( std::size_t index = 0; index + 1 < m_nodes.size(); ++index ) {
            auto const x0 { m_nodes[ index ] }, x1 { m_nodes[ index + 1 ] };
            for( auto const &load : m_loads ) {
                if( std::abs( load.x - x0 ) < 1e-9 ) { loadleft += load.force; }
            }
            auto const &zone { zone_at( 0.5 * ( x0 + x1 ) ) };
            auto const d { x1 - x0 };
            sumcoefficient += d / zone.tension;
            sumconstant += ( -loadleft * d - 0.5 * zone.weight * d * d ) / zone.tension;
            m_segments.push_back( { x0, x1, zone.tension, zone.weight, loadleft } );
            loadleft += zone.weight * d;
        }
        // dodatnie y w górę, lina opada gdy R - W > 0, stąd znak minus
        m_reaction = ( ( m_ystart - m_yend ) - sumconstant ) / sumcoefficient;
        double y { m_ystart };
        for( auto &segment : m_segments ) {
            segment.y0 = y;
            auto const d { segment.x1 - segment.x0 };
            y -= ( ( m_reaction - segment.loadleft ) * d - 0.5 * segment.weight * d * d ) / segment.tension;
        }
    }
    double height( double X ) const {
        X = std::clamp( X, 0.0, m_length );
        for( auto const &segment : m_segments ) {
            if( X <= segment.x1 + 1e-9 ) {
                auto const d { X - segment.x0 };
                return segment.y0 - ( ( m_reaction - segment.loadleft ) * d - 0.5 * segment.weight * d * d ) / segment.tension;
            }
        }
        return m_yend;
    }
    std::vector<double> const &nodes() const { return m_nodes; }

private:
    struct zone { double x0, x1, tension, weight; };
    struct segment { double x0, x1, tension, weight, loadleft; double y0 { 0.0 }; };
    zone const &zone_at( double X ) const {
        for( auto const &zone : m_zones ) {
            if( X <= zone.x1 + 1e-9 ) { return zone; }
        }
        return m_zones.back();
    }
    double m_length, m_ystart, m_yend;
    std::vector<zone> m_zones;
    std::vector<point_load> m_loads;
    std::vector<double> m_nodes;
    std::vector<segment> m_segments;
    double m_reaction { 0.0 };
};

} // namespace

span_result solve_span( network_type const &Type, span_input const &Input, double const Samplestep ) {

    auto const length { Input.length };
    if( length <= 0.0 ) { throw std::invalid_argument( "span length must be positive" ); }
    if( Input.hanger_gaps.size() < 2 ) { throw std::invalid_argument( "at least one hanger is required" ); }
    auto const gapsum { std::accumulate( Input.hanger_gaps.begin(), Input.hanger_gaps.end(), 0.0 ) };
    if( std::abs( gapsum - length ) > 0.01 ) { throw std::invalid_argument( "hanger gaps do not add up to span length" ); }
    if( Type.messenger_tension <= 0.0 ) { throw std::invalid_argument( "messenger tension must be positive" ); }

    span_result result;

    // położenia wieszaków
    std::vector<double> positions;
    {
        double x { 0.0 };
        for( std::size_t index = 0; index + 1 < Input.hanger_gaps.size(); ++index ) {
            x += Input.hanger_gaps[ index ];
            positions.push_back( x );
        }
    }
    auto const count { positions.size() };

    // obciążenie każdego wieszaka: przewód jezdny po połowie sąsiednich odcinków plus masa wieszaka
    auto const contactweight { Type.contact_mass * Type.contact_count * gravity };
    std::vector<double> loads( count );
    for( std::size_t index = 0; index < count; ++index ) {
        auto const left { index == 0 ? 0.0 : 0.5 * ( positions[ index - 1 ] + positions[ index ] ) };
        auto const right { index + 1 == count ? length : 0.5 * ( positions[ index ] + positions[ index + 1 ] ) };
        loads[ index ] = contactweight * ( right - left ) + Type.hanger_mass * gravity;
    }

    // strefa podwieszenia Y po obu stronach przęsła
    auto ylength { Type.y_length( length ) };
    if( ylength >= length ) { ylength = 0.0; }
    if( Type.y_tension <= 0.0 ) { ylength = 0.0; }
    auto const reachstart { Input.y_at_start ? 0.5 * ylength : 0.0 };
    auto const reachend { Input.y_at_end ? 0.5 * ylength : 0.0 };
    auto const hasystart { reachstart > 0.0 }, hasyend { reachend > 0.0 };
    auto const hasy { hasystart || hasyend };
    result.y_length = hasy ? ylength : 0.0;

    auto const on_y_start = [ & ]( double X ) { return hasystart && X < reachstart - 1e-9; };
    auto const on_y_end = [ & ]( double X ) { return hasyend && X > length - reachend + 1e-9; };

    auto const ropeweight { Type.messenger_mass * gravity };
    auto const fullweight { ropeweight * Type.messenger_count };
    auto const yweight { Type.y_mass * gravity };

    // siła pionowa przekazywana przez koniec linki Y na linę nośną (połowa linki po jednej stronie podwieszenia)
    double ystartforce { 0.0 }, yendforce { 0.0 };
    for( std::size_t index = 0; index < count; ++index ) {
        if( on_y_start( positions[ index ] ) ) { ystartforce += loads[ index ]; }
        if( on_y_end( positions[ index ] ) ) { yendforce += loads[ index ]; }
    }
    ystartforce += yweight * reachstart;
    yendforce += yweight * reachend;

    auto const heightstart { Input.height_start >= 0.0 ? Input.height_start : Type.structure_height };
    auto const heightend { Input.height_end >= 0.0 ? Input.height_end : Type.structure_height };

    // górna lina nośna
    cable messenger { length, heightstart, heightend };
    if( hasy ) {
        // w strefie Y górna lina niesie naciąg pomniejszony o naciąg linki Y, a przy dwóch linach tylko własny ciężar
        auto const zonetension { Type.messenger_tension - Type.y_tension };
        if( zonetension <= 0.0 ) { throw std::invalid_argument( "Y tension must be lower than messenger tension" ); }
        auto const zoneweight { Type.messenger_count > 1 ? ropeweight : fullweight };
        if( hasystart ) { messenger.add_zone( 0.0, reachstart, zonetension, zoneweight ); }
        messenger.add_zone( reachstart, length - reachend, Type.messenger_tension, fullweight );
        if( hasyend ) { messenger.add_zone( length - reachend, length, zonetension, zoneweight ); }
        if( hasystart ) { messenger.add_load( reachstart, ystartforce ); }
        if( hasyend ) { messenger.add_load( length - reachend, yendforce ); }
    }
    else {
        messenger.add_zone( 0.0, length, Type.messenger_tension, fullweight );
    }
    for( std::size_t index = 0; index < count; ++index ) {
        if( on_y_start( positions[ index ] ) || on_y_end( positions[ index ] ) ) { continue; }
        messenger.add_load( positions[ index ], loads[ index ] );
    }
    messenger.solve();

    // linka Y: liczona od osi podwieszenia (tam styczna pozioma – założenie symetrii z sąsiednim przęsłem) do zacisku
    auto const lowerdrop { Type.y_junction_drop };
    auto const solve_y = [ & ]( bool const Atend ) {
        std::vector<point2> points;
        auto const reach { Atend ? reachend : reachstart };
        if( reach <= 0.0 ) { return points; }
        // współrzędna s: odległość od osi podwieszenia w głąb przęsła
        std::vector<point_load> yloads;
        for( std::size_t index = 0; index < count; ++index ) {
            auto const x { positions[ index ] };
            if( Atend ? on_y_end( x ) : on_y_start( x ) ) { yloads.push_back( { Atend ? length - x : x, loads[ index ] } ); }
        }
        std::sort( yloads.begin(), yloads.end(), []( point_load const &L, point_load const &R ) { return L.x < R.x; } );
        // zacisk może leżeć poniżej górnej liny, np. gdy dolna lina jest do niej przypięta od spodu
        auto const junction { messenger.height( Atend ? length - reach : reach ) - lowerdrop };
        // nachylenie w punkcie s: ( ciężar linki na odcinku 0..s + wieszaki na 0..s ) / naciąg ; wysokość liczona wstecz od zacisku
        std::vector<double> stations { 0.0, reach };
        for( auto const &load : yloads ) { stations.push_back( load.x ); }
        std::sort( stations.begin(), stations.end() );
        stations.erase( std::unique( stations.begin(), stations.end(), []( double L, double R ) { return std::abs( L - R ) < 1e-9; } ), stations.end() );
        std::vector<double> heights( stations.size() );
        heights.back() = junction;
        for( auto index = stations.size() - 1; index > 0; --index ) {
            auto const s0 { stations[ index - 1 ] }, s1 { stations[ index ] };
            double carried { 0.0 };
            for( auto const &load : yloads ) {
                if( load.x <= s0 + 1e-9 ) { carried += load.force; }
            }
            // całka od s0 do s1 z ( yweight * s + carried ) / T
            auto const rise { ( 0.5 * yweight * ( s1 * s1 - s0 * s0 ) + carried * ( s1 - s0 ) ) / Type.y_tension };
            heights[ index - 1 ] = heights[ index ] - rise;
        }
        for( std::size_t index = 0; index < stations.size(); ++index ) {
            points.push_back( { Atend ? length - stations[ index ] : stations[ index ], heights[ index ] } );
        }
        if( Atend ) { std::reverse( points.begin(), points.end() ); }
        return points;
    };
    result.y_start = solve_y( false );
    result.y_end = solve_y( true );
    // połowy linki Y do ewentualnego przeliczenia razem z sąsiednimi przęsłami
    result.y_half_start.reach = reachstart;
    result.y_half_end.reach = reachend;
    if( hasystart ) { result.y_half_start.junction = messenger.height( reachstart ) - lowerdrop; }
    if( hasyend ) { result.y_half_end.junction = messenger.height( length - reachend ) - lowerdrop; }
    for( std::size_t index = 0; index < count; ++index ) {
        auto const x { positions[ index ] };
        if( on_y_start( x ) ) { result.y_half_start.loads.emplace_back( x, loads[ index ] ); }
        if( on_y_end( x ) ) { result.y_half_end.loads.emplace_back( length - x, loads[ index ] ); }
    }

    auto const y_height = []( std::vector<point2> const &Points, double const X ) {
        for( auto const &point : Points ) {
            if( std::abs( point.x - X ) < 1e-6 ) { return point.y; }
        }
        return 0.0;
    };

    // wieszaki
    for( std::size_t index = 0; index < count; ++index ) {
        auto const x { positions[ index ] };
        hanger item;
        item.x = x;
        if( on_y_start( x ) ) { item.on_y = true; item.length = y_height( result.y_start, x ) - Type.y_hanger_drop; }
        else if( on_y_end( x ) ) { item.on_y = true; item.length = y_height( result.y_end, x ) - Type.y_hanger_drop; }
        else { item.length = messenger.height( x ) - Type.messenger_stack; }
        result.hangers.push_back( item );
    }

    // przebieg liny do rysowania: węzły obciążeń i punkty pośrednie co Samplestep
    {
        auto nodes { messenger.nodes() };
        auto const step { std::max( 0.1, Samplestep ) };
        for( double x = step; x < length; x += step ) { nodes.push_back( x ); }
        std::sort( nodes.begin(), nodes.end() );
        nodes.erase( std::unique( nodes.begin(), nodes.end(), []( double L, double R ) { return std::abs( L - R ) < 1e-6; } ), nodes.end() );
        for( auto const x : nodes ) {
            auto const y { messenger.height( x ) };
            result.messenger.push_back( { x, y } );
            auto const chord { heightstart + ( heightend - heightstart ) * x / length };
            result.sag = std::max( result.sag, chord - y );
        }
    }

    return result;
}

span_result solve_free_span( network_type const &Type, double const Length, double const Heightstart, double const Heightend, double const Samplestep ) {

    if( Length <= 0.0 ) { throw std::invalid_argument( "span length must be positive" ); }
    if( Type.messenger_tension <= 0.0 ) { throw std::invalid_argument( "messenger tension must be positive" ); }
    span_result result;
    cable messenger { Length, Heightstart, Heightend };
    messenger.add_zone( 0.0, Length, Type.messenger_tension, Type.messenger_mass * gravity * Type.messenger_count );
    messenger.solve();
    auto const pieces { std::max( 1, static_cast<int>( std::ceil( Length / std::max( 0.5, Samplestep ) ) ) ) };
    for( int piece = 0; piece <= pieces; ++piece ) {
        auto const x { Length * piece / pieces };
        auto const y { messenger.height( x ) };
        result.messenger.push_back( { x, y } );
        result.sag = std::max( result.sag, Heightstart + ( Heightend - Heightstart ) * x / Length - y );
    }
    // bez wieszaków przewód jezdny też wisi pod własnym ciężarem
    if( Type.contact_tension > 0.0 ) {
        cable contact { Length, 0.0, 0.0 };
        contact.add_zone( 0.0, Length, Type.contact_tension, Type.contact_mass * gravity * Type.contact_count );
        contact.solve();
        for( int piece = 0; piece <= pieces; ++piece ) {
            auto const x { Length * piece / pieces };
            result.contact.push_back( { x, contact.height( x ) } );
        }
    }
    return result;
}

void join_y( network_type const &Type, span_result &Span, double const Length, bool const Atend, y_half const &Neighbour ) {

    auto const &mine { Atend ? Span.y_half_end : Span.y_half_start };
    if( mine.reach <= 0.0 || Neighbour.reach <= 0.0 || Type.y_tension <= 0.0 ) { return; }
    // współrzędna u: od zacisku w tym przęśle (0), przez oś podwieszenia (mine.reach), do zacisku w sąsiednim
    auto const total { mine.reach + Neighbour.reach };
    cable rope { total, mine.junction, Neighbour.junction };
    rope.add_zone( 0.0, total, Type.y_tension, Type.y_mass * gravity );
    for( auto const &load : mine.loads ) { rope.add_load( mine.reach - load.first, load.second ); }
    for( auto const &load : Neighbour.loads ) { rope.add_load( mine.reach + load.first, load.second ); }
    rope.solve();
    // punkty linki w tym przęśle: oś podwieszenia, wieszaki, zacisk
    std::vector<double> stations { 0.0, mine.reach };
    for( auto const &load : mine.loads ) { stations.push_back( load.first ); }
    std::sort( stations.begin(), stations.end() );
    stations.erase( std::unique( stations.begin(), stations.end(), []( double L, double R ) { return std::abs( L - R ) < 1e-9; } ), stations.end() );
    std::vector<point2> points;
    for( auto const s : stations ) {
        points.push_back( { Atend ? Length - s : s, rope.height( mine.reach - s ) } );
    }
    if( Atend ) { std::reverse( points.begin(), points.end() ); }
    for( auto &item : Span.hangers ) {
        if( false == item.on_y ) { continue; }
        for( auto const &point : points ) {
            if( std::abs( point.x - item.x ) < 1e-6 ) { item.length = point.y - Type.y_hanger_drop; }
        }
    }
    ( Atend ? Span.y_end : Span.y_start ) = points;
}

} // namespace catenary
