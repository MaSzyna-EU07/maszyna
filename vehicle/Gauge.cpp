/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

/*
    MaSzyna EU07 locomotive simulator
    Copyright (C) 2001-2004  Marcin Wozniak

*/

#include "stdafx.h"
#include "vehicle/Gauge.h"
#include "vehicle/gaugepositions.h"
#include "vehicle/gaugeduplicates.h"
#include "utilities/parser.h"
#include "model/Model3d.h"
#include "vehicle/DynObj.h"
#include "utilities/Timer.h"
#include "utilities/Logs.h"
#include "rendering/renderer.h"

TGauge::TGauge( sound_source const &Soundtemplate ) :
    m_soundtemplate( Soundtemplate )
{
    m_soundfxincrease = m_soundtemplate;
    m_soundfxdecrease = m_soundtemplate;
    m_soundfxon = m_soundtemplate;
    m_soundfxoff = m_soundtemplate;
}

void TGauge::Init(TSubModel *Submodel, TSubModel *Submodelon, TGaugeAnimation Type, float Scale, float Offset, float Friction, float Value, float const Endvalue, float const Endscale, bool const Interpolatescale )
{ // ustawienie parametrów animacji submodelu
    SubModel = Submodel;
    SubModelOn = Submodelon;
    m_value = Value;
    m_animation = Type;
    m_scale = Scale;
    m_offset = Offset;
    m_friction = Friction;
    m_interpolatescale = Interpolatescale;
    m_endvalue = Endvalue;
    m_endscale = Endscale;

    if( m_animation == TGaugeAnimation::gt_Digital ) {

        auto *sm { ( SubModel ? SubModel->ChildGet() : nullptr ) };
        while( sm != nullptr ) {
            // pętla po submodelach potomnych i obracanie ich o kąt zależy od cyfry w (fValue)
            if( sm->pName.size() // musi mieć niepustą nazwę
             && std::isdigit(sm->pName[0]) ) {
                sm->WillBeAnimated(); // wyłączenie optymalizacji
            }
            sm = sm->NextGet();
        }
        // do the same for the active version
        sm = SubModelOn ? SubModelOn->ChildGet() : nullptr;
        while( sm != nullptr ) {
            // pętla po submodelach potomnych i obracanie ich o kąt zależy od cyfry w (fValue)
            if( sm->pName.size() // musi mieć niepustą nazwę
             && std::isdigit(sm->pName[0]) ) {
                sm->WillBeAnimated(); // wyłączenie optymalizacji
            }
            sm = sm->NextGet();
        }
    }
    else {
        // wyłączenie ignowania jedynkowego transformu
        // a banan może być z optymalizacją?
        if( SubModel   != nullptr ) { SubModel->WillBeAnimated(); }
        if( SubModelOn != nullptr ) { SubModelOn->WillBeAnimated(); }
    }
    // pass submodel location to defined sounds
    auto const nulloffset { glm::vec3{} };
    auto const offset { model_offset() };
    {
        std::vector soundfxs = {
            &m_soundfxincrease,
            &m_soundfxdecrease,
            &m_soundfxon,
            &m_soundfxoff
        };
        for( auto *soundfx : soundfxs ) {
            if( soundfx->offset() == nulloffset ) {
                soundfx->offset( offset );
            }
        }
    }
    for( auto &[soundfxkey, soundfxsource] : m_soundfxvalues ) {
        if( soundfxsource.offset() == nulloffset ) {
            soundfxsource.offset( offset );
        }
    }
};

void TGauge::Load( cParser &Parser, TDynamicObject const *Owner, double const mul ) {

    std::string submodelname;
    std::string gaugetypename;
    float scale;
    float endscale;
    float endvalue;
    float offset;
    float friction;
    endscale = -1;
    endvalue = -1;
    bool interpolatescale { false };

    // the control as defined so far in this cab, which a definition marked as its duplicate adds a copy to
    // rather than replacing it (eu07/maszyna#527)
    std::optional<TGauge> original;
    if( gauge_duplicates::adds_copy<TModel3d>( true, m_sourcemodel, Owner->mdKabina, Owner->mdLowPolyInt ) ) {
        original = *this;
    }
    scratch_data scratchpad;

    Parser.getTokens();
    if( Parser.peek() != "{" ) {
        // old fixed size config
        // it has no scaletable:, drop the one of an earlier definition of the same control
        m_scaletable.clear();
        Parser >> submodelname;
        gaugetypename = Parser.getToken<std::string>( true );
        Parser.getTokens( 3, false );
        Parser
            >> scale
            >> offset
            >> friction;
        if( gaugetypename == "rotvar"
         || gaugetypename == "movvar" ) {
            interpolatescale = true;
            Parser.getTokens( 2, false );
            Parser
                >> endvalue
                >> endscale;
        }
    }
    else {
        // new, block type config
        // TODO: rework the base part into yaml-compatible flow style mapping
        submodelname = Parser.getToken<std::string>( false );
        gaugetypename = Parser.getToken<std::string>( true );
        Parser.getTokens( 3, false );
        Parser
            >> scale
            >> offset
            >> friction;
        if( gaugetypename == "rotvar"
         || gaugetypename == "movvar" ) {
            interpolatescale = true;
            Parser.getTokens( 2, false );
            Parser
                >> endvalue
                >> endscale;
        }
        // new, variable length section
        {
            while( true == Load_mapping( Parser, scratchpad ) ) {
                // all work done by while()
            }
            // the values of the table are in the units the config uses for the scale, which the multiplier converts the gauge's value to
            for( auto &point : scratchpad.scaletable ) {
                point.value = static_cast<float>( point.value / mul );
            }
            gauge_scale::sort( scratchpad.scaletable );
            m_scaletable = std::move( scratchpad.scaletable );
            // post-deserialization cleanup
            // set provided custom soundproofing to assigned sounds (for sounds without their own custom soundproofing)
            if( scratchpad.soundproofing ) {
                if( !m_soundfxincrease.soundproofing() ) {
                    m_soundfxincrease.soundproofing() = scratchpad.soundproofing;
                }
                if( !m_soundfxdecrease.soundproofing() ) {
                    m_soundfxdecrease.soundproofing() = scratchpad.soundproofing;
                }
                if( !m_soundfxon.soundproofing() ) {
                    m_soundfxon.soundproofing() = scratchpad.soundproofing;
                }
                if( !m_soundfxoff.soundproofing() ) {
                    m_soundfxoff.soundproofing() = scratchpad.soundproofing;
                }
                for( auto &[soundfxkey, soundfxsource] : m_soundfxvalues ) {
                    if( !soundfxsource.soundproofing() ) {
                        soundfxsource.soundproofing() = scratchpad.soundproofing;
                    }
                }
            }
        }
    }

    // bind defined sounds with the button owner
    m_soundfxincrease.owner( Owner );
    m_soundfxdecrease.owner( Owner );
    m_soundfxon.owner( Owner );
    m_soundfxoff.owner( Owner );
    for( auto &[soundfxkey, soundfxsource] : m_soundfxvalues ) {
        soundfxsource.owner( Owner );
    }

	scale *= static_cast<float>(mul);
    if( interpolatescale ) {
        endscale *= static_cast<float>(mul);
    }
    TSubModel *submodel { nullptr };
    m_sourcemodel = nullptr;
    std::array<TModel3d *, 2> sources { Owner->mdKabina, Owner->mdLowPolyInt };
    for( auto const *source : sources ) {
        if( source != nullptr
         && ( submodel = source->GetFromName( submodelname ) ) != nullptr ) {
            // got what we wanted, bail out
            m_sourcemodel = source;
            break;
        }
        // there's a possibility the default submodel was named with _off suffix
        if( source != nullptr
         && ( submodel = source->GetFromName( submodelname + "_off" ) ) != nullptr ) {
            // got what we wanted, bail out
            m_sourcemodel = source;
            break;
        }
    }
    if( submodel == nullptr ) {
        ErrorLog( "Bad model: failed to locate sub-model \"" + submodelname + "\" in 3d model(s) of \"" + Owner->name() + "\"", logtype::model );
    }
    // see if we can locate optional submodel for active state, with _on suffix
    TSubModel *submodelon { nullptr };
    for( auto const *source : sources ) {
        if( source != nullptr
         && ( submodelon = source->GetFromName( submodelname + "_on" ) ) != nullptr ) {
            // got what we wanted, bail out
            break;
        }
    }

    std::map<std::string, TGaugeAnimation, std::less<>> gaugetypes {
        { "rot", TGaugeAnimation::gt_Rotate },
        { "rotvar", TGaugeAnimation::gt_Rotate },
        { "mov", TGaugeAnimation::gt_Move },
        { "movvar", TGaugeAnimation::gt_Move },
        { "wip", TGaugeAnimation::gt_Wiper },
        { "dgt", TGaugeAnimation::gt_Digital }
    };
    auto lookup = gaugetypes.find( gaugetypename );
    auto const type = lookup != gaugetypes.end() ? lookup->second : TGaugeAnimation::gt_Unknown;

    Init( submodel, submodelon, type, scale, offset, friction, 0, endvalue, endscale, interpolatescale );

    if( scratchpad.duplicate && original ) {
        // a duplicate submodel of the control already defined: the control stays as it was, with one more copy
        // animated along with it. the duplicate's own sounds, if any, are left out so the control isn't heard twice
        auto duplicate { MirrorCopy() };
        *this = std::move( *original );
        m_mirrors.emplace_back( std::move( duplicate ) );
    }

//    return md2 != nullptr; // true, gdy podany model zewnętrzny, a w kabinie nie było
};

bool
TGauge::Load_mapping( cParser &Input, TGauge::scratch_data &Scratchpad ) {

    // token can be a key or block end
    auto const key { Input.getToken<std::string>( true, "\n\r\t  ,;" ) };
    if( true == key.empty() || key == "}" ) { return false; }
    // if not block end then the key is followed by assigned value or sub-block
    if( key == "duplicate:" ) {
        // the definition is another submodel of the control defined before it, rather than its replacement
        auto const duplicate { Input.getToken<std::string>( true, "\n\r\t  ,;" ) };
        Scratchpad.duplicate = gauge_duplicates::is_duplicate( duplicate );
    }
    else if( key == "type:" ) {
        using enum TGaugeType;
        auto const gaugetype { Input.getToken<std::string>( true, "\n\r\t  ,;" ) };
        m_type = gaugetype == "push"       ? push :
		         gaugetype == "impulse"    ? push :
		         gaugetype == "return"     ? push :
		         gaugetype == "delayed"    ? push_delayed :
		         gaugetype == "pushtoggle" ? pushtoggle :
		                                     toggle; // "toggle" and default
        m_typedeclared = true;
    }
    else if( key == "soundinc:" ) {
        m_soundfxincrease.deserialize( Input, sound_type::single );
    }
    else if( key == "sounddec:" ) {
        m_soundfxdecrease.deserialize( Input, sound_type::single );
    }
    else if( key == "soundon:" ) {
        m_soundfxon.deserialize( Input, sound_type::single );
    }
    else if( key == "soundoff:" ) {
        m_soundfxoff.deserialize( Input, sound_type::single );
    }
    else if( key == "soundproofing:" ) {
        // custom soundproofing in format [ p1, p2, p3, p4, p5, p6 ]
        Input.getTokens( 6, false, "\n\r\t ,;[]" );
        std::array<float, 6> soundproofing;
        Input
            >> soundproofing[ 0 ]
            >> soundproofing[ 1 ]
            >> soundproofing[ 2 ]
            >> soundproofing[ 3 ]
            >> soundproofing[ 4 ]
            >> soundproofing[ 5 ];
        Scratchpad.soundproofing = soundproofing;
    }
    else if( key == "scaletable:" ) {
        // non-uniform scale in format [ value1 scaled1, value2 scaled2, ... ]
        return Load_scaletable( Input, Scratchpad );
    }
    else if( key.starts_with("sound") ) {
        // sounds assigned to specific gauge values, defined by key soundX: where X = value
        auto const indexstart { key.find_first_of( "-1234567890" ) };
        auto const indexend { key.find_first_not_of( "-1234567890", indexstart ) };
        if( indexstart != std::string::npos ) {
            m_soundfxvalues.try_emplace(
                std::stoi( key.substr( indexstart, indexend - indexstart ) ),
                sound_source( m_soundtemplate ).deserialize( Input, sound_type::single ) );
        }
    }
    return true; // return value marks a key: value pair was extracted, nothing about whether it's recognized
}

// imports the optional non-uniform scale (MaSzyna-EU07/maszyna#74); returns false if it ran into the end of the block
bool
TGauge::Load_scaletable( cParser &Input, TGauge::scratch_data &Scratchpad ) {

    auto table { gauge_scale::read( Input ) };
    if( table.malformed || false == table.closed || table.points.size() < 2 ) {
        ErrorLog( "Bad model: malformed scaletable of a gauge, it needs [ value scaled, value scaled, ... ] with at least two pairs", logtype::model );
    }
    if( table.points.size() >= 2 ) {
        Scratchpad.scaletable = std::move( table.points );
    }
    return false == table.block_end;
}

// ustawienie wartości docelowej. plays provided fallback sound, if no sound was defined in the control itself
bool
TGauge::UpdateValue( float fNewDesired, std::optional<sound_source> &Fallbacksound ) {

    if (false == UpdateValue( fNewDesired ) && Fallbacksound) {
        Fallbacksound->play( m_soundtype );
        return true;
    }
    return false;
}

bool
TGauge::UpdateValue( float fNewDesired ) {

    auto const desiredtimes100 = static_cast<int>( std::round( 100.0 * fNewDesired ) );
    auto const previoustimes100 = static_cast<int>( std::round( 100.0 * m_targetvalue ) );
    if( desiredtimes100 == previoustimes100 ) {
        return true;
    }
    m_targetvalue = fNewDesired;
    if( desiredtimes100 % 100 != 0 ) {
        // toggle the control to continous range/exclusive sound mode from now on
        m_soundtype = sound_flags::exclusive;
    }
    // if there's any sound associated with a position reached on the way to the new requested value, play it
    // check value-specific table first...
    if( auto const reached = gauge_positions::last_reached( m_soundfxvalues, previoustimes100, desiredtimes100 );
        reached != m_soundfxvalues.end() ) {
        reached->second.play();
        return true;
    }
    // ...and if there isn't any, fall back on the basic set...
    // HACK: crude way to discern controls with continuous and quantized value range
    auto const currentvalue = GetValue();
    if( currentvalue < fNewDesired && false == m_soundfxincrease.empty() ) {
        // shift up
        m_soundfxincrease.play( m_soundtype );
        return true;
    }
    if( currentvalue > fNewDesired && false == m_soundfxdecrease.empty() ) {
        // shift down
        m_soundfxdecrease.play( m_soundtype );
        return true;
    }
    return false; // no suitable sound was found
}

void TGauge::PutValue(float fNewDesired)
{ // McZapkie-281102: natychmiastowe wpisanie wartosci
    m_targetvalue = fNewDesired;
    m_value = m_targetvalue;
};

float TGauge::GetValue() const {
    // we feed value in range 0-1 so we should be getting it reported in the same range
    return m_value;
}

float TGauge::GetDesiredValue() const {
    // we feed value in range 0-1 so we should be getting it reported in the same range
    return m_targetvalue;
}

void TGauge::Update( bool const Power ) {
    // update value
    // TODO: remove passing manually power state when LD is in place
    if( m_value != m_targetvalue ) {
        auto dt = static_cast<float>(Timer::GetDeltaTime());
        if( m_friction > 0 && dt < 0.5 * m_friction ) {
            // McZapkie-281102: zabezpieczenie przed oscylacjami dla dlugich czasow
            m_value += dt * ( m_targetvalue - m_value ) / m_friction;
            if( std::abs( m_targetvalue - m_value ) <= 0.0001 ) {
                // close enough, we can stop updating the model
                m_value = m_targetvalue; // set it exactly as requested just in case it matters
            }
        }
        else {
            m_value = m_targetvalue;
        }
    }
    // update submodel visibility
    auto const state { Power && ( m_stateinput ? *m_stateinput : false ) };
    if( state != m_state ) {
        // on state change play assigned sound
        if( true == state ) { m_soundfxon.play(); }
        else                { m_soundfxoff.play(); }
    }
    m_state = state;
    UpdateVisibility();
    // update submodel animations
    UpdateAnimation( SubModel );
    UpdateAnimation( SubModelOn );
    // the copies follow the control's value and state, each with its own animation
    for( auto &mirror : m_mirrors ) {
        mirror.m_value = m_value;
        mirror.m_targetvalue = m_targetvalue;
        mirror.m_state = m_state;
        mirror.UpdateVisibility();
        mirror.UpdateAnimation( mirror.SubModel );
        mirror.UpdateAnimation( mirror.SubModelOn );
    }
};

void TGauge::UpdateVisibility() const {
    // toggle submodel visibility only if the active state submodel is present,
    // keep the default model always visible otherwise
    if( SubModelOn != nullptr ) {
        SubModelOn->iVisible = m_state;
        if( SubModel != nullptr ) {
            SubModel->iVisible = !m_state;
        }
    }
}

TGauge TGauge::MirrorCopy() const {

    TGauge mirror;
    mirror.SubModel = SubModel;
    mirror.SubModelOn = SubModelOn;
    mirror.m_animation = m_animation;
    mirror.m_friction = m_friction;
    mirror.m_targetvalue = m_targetvalue;
    mirror.m_value = m_value;
    mirror.m_offset = m_offset;
    mirror.m_scale = m_scale;
    mirror.m_endscale = m_endscale;
    mirror.m_endvalue = m_endvalue;
    mirror.m_interpolatescale = m_interpolatescale;
    mirror.m_scaletable = m_scaletable;
    mirror.m_state = m_state;
    mirror.m_sourcemodel = m_sourcemodel;
    return mirror;
}

void TGauge::AssignFloat(float *fValue)
{
    m_datatype = 'f';
    fData = fValue;
};

void TGauge::AssignDouble(double *dValue)
{
    m_datatype = 'd';
    dData = dValue;
};

void TGauge::AssignInt(int *iValue)
{
    m_datatype = 'i';
    iData = iValue;
};

void TGauge::AssignBool(bool *bValue)
{
    m_datatype = 'b';
    bData = bValue;
};

void TGauge::UpdateValue()
{ // ustawienie wartości docelowej z parametru
    switch (m_datatype)
    { // to nie jest zbyt optymalne, można by zrobić osobne funkcje
        case 'f': {
            UpdateValue( *fData );
            break;
        }
        case 'd': {
            UpdateValue( static_cast<float>(*dData) );
            break;
        }
        case 'i': {
            UpdateValue( static_cast<float>(*iData) );
            break;
        }
        case 'b': {
            UpdateValue( *bData ? 1.f : 0.f );
            break;
        }
        default: {
            break;
        }
    }
};

void TGauge::AssignState( bool const *State ) {

    m_stateinput = State;
}

float TGauge::GetScaledValue() const {

    if( false == m_scaletable.empty() ) {
        return gauge_scale::interpolate( m_scaletable, m_value );
    }
    return false == m_interpolatescale ? m_value * m_scale + m_offset : m_value * std::lerp(m_scale, m_endscale, std::clamp(m_value / m_endvalue, 0.f, 1.f)) + m_offset;
}

void
TGauge::UpdateAnimation( TSubModel *Submodel ) const {

    if( Submodel == nullptr ) { return; }

    switch (m_animation) {
        case TGaugeAnimation::gt_Rotate: {
            Submodel->SetRotate( float3( 0, 1, 0 ), static_cast<float>(GetScaledValue() * 360.0) );
            break;
        }
        case TGaugeAnimation::gt_Move: {
            Submodel->SetTranslate( float3( 0, 0, GetScaledValue() ) );
            break;
        }
        case TGaugeAnimation::gt_Wiper: {
            auto const scaledvalue { GetScaledValue() };
            Submodel->SetRotate( float3( 0, 1, 0 ), static_cast<float>(scaledvalue * 360.0) );
            if( auto *sm = Submodel->ChildGet(); sm ) {
                sm->SetRotate( float3( 0, 1, 0 ), static_cast<float>(scaledvalue * 360.0) );
                sm = sm->ChildGet();
                if( sm )
                    sm->SetRotate( float3( 0, 1, 0 ), static_cast<float>(scaledvalue * 360.0) );
            }
            break;
        }
        case TGaugeAnimation::gt_Digital: {
            // Ra 2014-07: licznik cyfrowy
            auto *sm = Submodel->ChildGet();
/*  			std::string n = FormatFloat( "0000000000", floor( fValue ) ); // na razie tak trochę bez sensu
*/	    		std::string n( "000000000" + std::to_string( static_cast<int>( std::floor( GetScaledValue() ) ) ) );
            if( n.length() > 10 ) { n.erase( 0, n.length() - 10 ); } // also dumb but should work for now
            do { // pętla po submodelach potomnych i obracanie ich o kąt zależy od cyfry w (fValue)
                if( sm->pName.size()
                 && std::isdigit(sm->pName[0]) ) {
                    sm->SetRotate(
                        float3( 0, 1, 0 ),
                        static_cast<float>(-36.0 * ( n[ '0' + 9 - sm->pName[ 0 ] ] - '0' )) );
                }
                sm = sm->NextGet();
            } while( sm );
            break;
        }
        default: {
            break;
        }
    }
}

// returns offset of submodel associated with the button from the model centre
glm::vec3
TGauge::model_offset() const {

    return SubModel != nullptr ? SubModel->offset(1.f) : glm::vec3();
}

TGaugeType
TGauge::type() const {
    return m_type;
}
//---------------------------------------------------------------------------
