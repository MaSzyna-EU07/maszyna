/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "utilities/Classes.h"
#include "scene/scene.h"

// particle specialized for drawing smoke
// given smoke features we can take certain shortcuts
// -- there's no need to sort the particles, can be drawn in any order with depth write turned off
// -- the colour remains consistent throughout, only opacity changes
// -- randomized particle rotation
// -- initial velocity reduced over time to slow drift upwards (drift speed depends on particle and air temperature difference)
// -- size increased over time
struct smoke_particle {

    glm::dvec3 position; // meters, 3d space;
    float rotation; // radians; local z axis angle
    glm::vec3 velocity; // meters per second, 3d space; current velocity
    float size; // multiplier, billboard size
//    glm::vec4 color; // 0-1 range, rgba; geometry color and opacity
    float opacity; // 0-1 range
//    glm::vec2 uv_offset; // 0-1 range, uv space; for texture animation
    float age; // seconds; time elapsed since creation
//    double distance; // meters; distance between particle and camera
};

enum value_limit {
    min = 0,
    max = 1
};

// helper, adjusts provided variable by fixed amount, keeping resulting value between limits
template <typename Type_>
class fixedstep_modifier {

public:
// methods
    void
        deserialize( cParser &Input );
    // updates state of provided variable
    void
        update( Type_ &Variable, double const Timedelta ) const;
    void
        bind( Type_ const *Modifier ) {
            m_valuechangemodifier = Modifier; }
    Type_
        value_change() const {
            return (
                m_valuechangemodifier == nullptr ?
                    m_valuechange :
                    m_valuechange / *m_valuechangemodifier ); }
    Type_
        limit( value_limit const Limit ) const {
            return m_valuelimits[ Limit ]; }

private:
//types
// methods
// members
//    Type_ m_intialvalue { Type_( 0 ) }; // meters per second; velocity applied to freshly spawned particles
    Type_ m_valuechange { Type_( 0 ) }; // meters per second; change applied to initial velocity
    Type_ m_valuelimits[ 2 ] { Type_( std::numeric_limits<Type_>::lowest() ), Type_( std::numeric_limits<Type_>::max() ) };
    Type_ const *m_valuechangemodifier{ nullptr }; // optional modifier applied to value change
};


// particle emitter
class smoke_source {
// located in scenery
// new particles emitted if distance of source < double view range
// existing particles are updated until dead no matter the range (presumed to have certain lifespan)
// during update pass dead particle slots are filled with new instances, if there's no particles queued the slot is swapped with the last particle in the list
// bounding box/sphere calculated based on position of all owned particles, used by the renderer to include/discard data in a draw pass
    friend class particle_manager;

public:
// types
    using particle_sequence = std::vector<smoke_particle>;
    // what the particles simulated on the gpu (particle_manager::gpu(), opengl46_gpu_particles) take from the source in an
    // update: how many are born, where and how they start, and how they change, all as the cpu update would do it
    struct gpu_state {
        std::uint32_t spawns { 0 }; // particles born in the last update, at most max_particles()
        glm::dvec3 location { 0.0 }; // of the emitter at the last update...
        glm::dvec3 previous { 0.0 }; // ...and at the one before; the particles of an update are born along the way between
        bool started { false };
        float inclination[ 2 ] { 0.f, 0.f }; // radians
        float velocity[ 2 ] { 0.f, 0.f }; // m/s, with the share the owner gives
        float size[ 2 ] { 0.f, 0.f };
        float opacity[ 2 ] { 0.f, 0.f }; // with the smoke fidelity and the share the owner gives
        float sizechange[ 3 ] { 0.f, 0.f, 0.f }; // per second, min, max
        float opacitychange[ 3 ] { 0.f, 0.f, 0.f }; // per second (with the fidelity), min, max
        // the vertical velocity: changes by (rise - damping) * itself per second, and doesn't fall below risemin
        float rise { 0.f };
        float damping { 0.f };
        float risemin { std::numeric_limits<float>::lowest() };
        // with a settling velocity (settles): the velocity goes over to it instead, see smoke_source::m_settling
        bool settles { false };
        float settling { 0.f };
        float drag { 1.f };
        float spread { 0.f }; // see smoke_source::m_spread
        float fadein { 0.f }; // see smoke_source::m_fadein
        glm::vec3 color { 0.f };
    };
// methods
    bool
        deserialize( cParser &Input );
    void
        initialize();
    void
        bind( TDynamicObject const *Vehicle );
    void
        bind( TAnimModel const *Node );
    // updates state of owned particles
    void
        update( double const Timedelta, bool const Onlydespawn );
    // the same for particles simulated on the gpu: only the births and the state of the source they start from
    void
        update_gpu( double const Timedelta, bool const Onlydespawn );
    gpu_state const &
        gpu() const {
            return m_gpu; }
    std::size_t
        max_particles() const {
            return m_max_particles; }
    // share of the opacity of a particle the age has let through so far, 0-1 (fade_in:)
    float
        fade_in( float const Age ) const {
            if( m_fadein <= 0.f ) { return 1.f; }
            auto const share { std::clamp( Age / m_fadein, 0.f, 1.f ) };
            return share * share * ( 3.f - 2.f * share ); }
    glm::vec3 const &
        color() const {
            return m_emitter.color; }
    glm::dvec3
        location() const;
    // provides access to bounding area data
    scene::bounding_area const &
        area() const {
            return m_area; }
    particle_sequence const &
        sequence() const {
            return m_particles; }

private:
// types
    enum class owner_type {
        none = 0,
        vehicle,
        node
    };

    struct particle_emitter {
        float inclination[ 2 ] { 0.f, 0.f };
        float velocity[ 2 ] { 1.f, 1.f };
        float size[ 2 ] { 1.f, 1.f };
        float opacity[ 2 ] { 1.f, 1.f };
        glm::vec3 color { 16.f / 255.f };

        void deserialize( cParser &Input );
        void initialize( smoke_particle &Particle ) const;
    };

    using bounding_box = glm::dvec3[ 2 ]; // bounding box of owned particles

// methods
    // imports member data pair from the config file
    bool
        deserialize_mapping( cParser &Input );
    // adds the particles to spawn in an update of Timedelta to m_spawncount
    void
        accumulate_spawns( double const Timedelta, bool const Onlydespawn );
    // shares of the launch velocity and of the opacity of a particle born now, which the owner gives
    void
        launch_scales( float &Velocity, float &Opacity ) const;
    void
        initialize( smoke_particle &Particle );
    // updates state of provided particle and bounding box. returns: true if particle is still alive afterwards, false otherwise
    bool
        update( smoke_particle &Particle, bounding_box &Boundingbox, double const Timedelta );
// members
    // config/inputs
    // TBD: union and indicator, or just plain owner variables?
    owner_type m_ownertype { owner_type::none };
    union {
        TDynamicObject const * vehicle;
        TAnimModel const * node;
    } m_owner { nullptr }; // optional, scene item carrying this source
    glm::dvec3 m_offset; // meters, 3d space; relative position of the source, either from the owner or the region centre
    float m_spawnrate { 0.f }; // number of particles to spawn per second
    particle_emitter m_emitter;
//    bool m_inheritvelocity { false }; // whether spawned particle should receive velocity of its owner
    // TODO: replace modifiers with configurable interpolator item allowing keyframe-based changes over time
    fixedstep_modifier<float> m_sizemodifier; // particle billboard size
//    fixedstep_modifier<glm::vec3> m_colormodifier; // particle billboard color and opacity
    fixedstep_modifier<float> m_opacitymodifier;
    // optional velocity the particles settle at in still air, m/s, positive upwards (light, hot smoke rises, heavy particles
    // fall), and how fast they get to it: the share of the rest of the way per second is 1 - exp(-drag). when given, it takes
    // the place of the crude dispersion of update( Particle ) (the rise in cold air, the damping in humid air)
    bool m_settles { false };
    float m_settling { 0.f }; // terminal_velocity:
    float m_drag { 1.f }; // drag:
    // optional growth of the particles with the way they go through the air, metres of size per metre: the jet out of the
    // exhaust widens into a cone as it takes in the air around, instead of the puffs swelling in one place
    float m_spread { 0.f }; // spread:
    // optional time the particles take to show up after they're born, seconds, so they don't pop up at the exhaust
    float m_fadein { 0.f }; // fade_in:
//    texture_handle m_texture { -1 }; // texture assigned to particle billboards
    // current state
    float m_spawncount { 0.f }; // number of particles to spawn during next update
    particle_sequence m_particles; // collection of spawned particles
    std::size_t m_max_particles; // maximum number of particles existing
    scene::bounding_area m_area; // bounding sphere of owned particles
    gpu_state m_gpu; // for the particles simulated on the gpu
};


// holds all particle emitters defined in the scene and updates their state
class particle_manager {

public:
// types
    using source_sequence = std::vector<smoke_source>;
// constructors
    particle_manager() = default;
// destructor
//    ~particle_manager();
// methods
    // adds a new particle source of specified type, placing it in specified world location. returns: true on success, false if the specified type definition couldn't be located
    bool
        insert( std::string const &Sourcetemplate, glm::dvec3 const Location );
    bool
        insert( std::string const &Sourcetemplate, TDynamicObject const *Vehicle, glm::dvec3 const Location );
    bool
        insert( std::string const &Sourcetemplate, TAnimModel const *Node, glm::dvec3 const Location );
    // updates state of all owned emitters
    void
        update();
    // whether the renderer simulates the particles on the gpu: the sources then work out only their births (smoke_source::gpu());
    // the renderer sets it before the next update. the particles simulated on the cpu are dropped when it's switched on
    void
        gpu( bool const Gpu );
    bool
        gpu() const {
            return m_gpu; }
    // number of the updates in which the time went on, and the time of the last one: the gpu simulates once per update
    std::uint64_t
        updates() const {
            return m_updates; }
    double
        timedelta() const {
            return m_timedelta; }
    // data access
    source_sequence &
        sequence() {
            return m_sources; }

// members

private:
// types
    using source_map = std::unordered_map<std::string, smoke_source>;
// methods
    smoke_source *
        find( std::string const &Template );
// members
    source_map m_sourcetemplates; // cached particle emitter configurations
    source_sequence m_sources; // all owned particle emitters
    bool m_gpu { false };
    std::uint64_t m_updates { 0 };
    double m_timedelta { 0.0 };
};



template <typename Type_>
void
fixedstep_modifier<Type_>::update( Type_ &Variable, double const Timedelta ) const {
    // HACK: float cast to avoid vec3 and double mismatch
    // TBD, TODO: replace with vector types specialization
    auto const valuechange { (
        m_valuechangemodifier == nullptr ?
            m_valuechange :
            m_valuechange / *m_valuechangemodifier ) };
    Variable += ( valuechange * static_cast<float>( Timedelta ) );
    // clamp down to allowed value range
    Variable = glm::max( Variable, m_valuelimits[ value_limit::min ] );
    Variable = glm::min( Variable, m_valuelimits[ value_limit::max ] );
}

template <typename Type_>
void
fixedstep_modifier<Type_>::deserialize( cParser &Input ) {

    if( Input.getToken<std::string>() != "{" ) { return; }

    std::unordered_map<std::string, Type_ &> const variablemap {
        { "step:", m_valuechange },
        { "min:", m_valuelimits[ value_limit::min ] },
        { "max:", m_valuelimits[ value_limit::max ] } };

    std::string key;

    while( ( false == ( ( key = Input.getToken<std::string>( true, "\n\r\t  ,;[]" ) ).empty() ) )
        && ( key != "}" ) ) {

        auto const lookup { variablemap.find( key ) };
        if( lookup == variablemap.end() ) { continue; }

        lookup->second = Input.getToken<Type_>( true, "\n\r\t  ,;[]" );
    }
}
