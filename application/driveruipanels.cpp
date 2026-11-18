/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/driveruipanels.h"

#include "utilities/Globals.h"
#include "application/application.h"
#include "utilities/translation.h"
#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Timer.h"
#include "world/Event.h"
#include "world/TractionPower.h"
#include "vehicle/Camera.h"
#include "world/mtable.h"
#include "world/stationnames.h"
#include "vehicle/Train.h"
#include "vehicle/Driver.h"
#include "vehicle/drivingaid.h"
#include "model/AnimModel.h"
#include "vehicle/DynObj.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#include "utilities/Logs.h"
#include "widgets/vehicleparams.h"
#include "utilities/U8.h"

#include "imgui/imgui_internal.h"   // ImTextCharFromUtf8 - UTF-8 codepoint decoding for HUD text
#include "McZapkie/MOVER.h"
#include "utilities/utilities.h"
#ifdef WITH_OPENGL_MODERN
#include "rendering/opengl33renderer.h"
#endif
#include <filesystem>
#include <fstream>
#include <sstream>

#define DRIVER_HINT_CONTENT
#include "application/driverhints.h"

#ifdef WITH_UART
#include "utilities/uart.h"
#endif

#include <format>
#include <string_view>
#include <vector>

#include "audio/audiorenderer.h"

namespace {

// how close to a broken coupler the camera has to be for the driving aid to offer its repair [m]
constexpr float coupler_repair_radius { 5.f };

// the broken coupler nearest to the camera, within reach: the vehicle and its end, or no vehicle
std::pair<TDynamicObject const *, int> broken_coupler_nearby( glm::dvec3 const &Camera ) {
    auto const *vehicle { std::get<TDynamicObject *>( simulation::Region->find_vehicle( Camera, coupler_repair_radius, false, true ) ) };
    if( vehicle == nullptr || vehicle->MoverParameters == nullptr ) { return { nullptr, end::front }; }
    auto const front { glm::distance( vehicle->CouplerPosition( end::front ), Camera ) };
    auto const rear { glm::distance( vehicle->CouplerPosition( end::rear ), Camera ) };
    auto const nearest { front <= rear ? end::front : end::rear };
    if( std::min( front, rear ) > coupler_repair_radius || false == vehicle->MoverParameters->Couplers[ nearest ].broken ) { return { nullptr, end::front }; }
    return { vehicle, nearest };
}

// whether this vehicle has an electric brake at all - a resistor or a regenerative one. only such a
// vehicle may show a NEGATIVE traction current: the minus means current is fed back, and a vehicle
// that cannot feed it back must not be given one.
// a classic drive declares its electrodynamic brake in the "Cntrl." line of the .fiz, as
// "DynamicBrake=" (Passive/Switch/Reversal/Automatic; Mover.cpp:11549-11555 maps the word onto
// DynamicBrakeType, MOVER.h:1841, with none = 0 and the other values at MOVER.h:390-398).
// the inverter drives mostly do NOT use that key - the Vectron (vectron.fiz.inc) declares no
// DynamicBrake at all, yet brakes electrically through its own inverters - so for an EIM the
// evidence is the braking force and power of the drive itself, Fh and Ph of the "Engine" line
// (eimc[eimc_p_Fh] / eimc[eimc_p_Ph], MOVER.h:542/544, parsed at Mover.cpp:12047-12048): Ph is the
// power the model hands to the brake (Mover.cpp:6809) and Fh its force (Mover.cpp:6723). both of
// them 0 means the drive has no electric brake at all.
// always asked of the power unit (ctrlv), never of the cab car: a cab car or a trailer declares the
// ED of the whole trainset in its own .fiz while carrying no engine data.
bool hud_has_electric_brake( TMoverParameters const &mover ) {
    if( mover.DynamicBrakeType != dbrake_none ) { return true; }
    return ( mover.EngineType == TEngineType::ElectricInductionMotor
          && mover.eimc[ eimc_p_Fh ] > 0.0
          && mover.eimc[ eimc_p_Ph ] > 0.0 );
}

// how much current the traction equipment draws right now, in amps, signed: it is negative while
// the machine works as a generator and feeds current back during regenerative braking.
// an inverter drive (EIM) is the one vehicle type that needs a correction here. Mover.cpp:6861
// builds its line current eimv[eimv_Ipoj] as "traction component + I0", where I0 is the no-load
// magnetising current the inverter draws while it is merely alive (eimc[eimc_f_I0], MOVER.h:530,
// read from the "I0" entry of the .fiz, Mover.cpp:12041). that base is ALWAYS there, so showing
// Ipoj as it stands lights the HUD with a constant current at standstill: EN57AKM declares I0=20,
// which is exactly the permanent 20 A the drivers complained about. subtracting I0 leaves the
// traction part alone and keeps the sign during regen, because a regen figure is -X + I0 and
// -X + I0 - I0 is still -X. the guard covers the other way the model can leave Ipoj behind: when
// the main circuit is not powered, or the engine is blocked, it zeroes the whole eimv[] array
// (Mover.cpp:6902-6920), so Ipoj is a plain 0 there and subtracting I0 would invent a phantom -I0
// that shows up as a red -20 A. Im is never a fallback for an EIM: for an inverter drive Im is the
// motor (field) current (Mover.cpp:6866), not the input current this reading is labelled with.
// what comes out of the branches above is only a candidate: the sign is then checked against the
// vehicle at the end, see there.
double hud_traction_current( TMoverParameters const &mover ) {
    double cur {};
    if( mover.EngineType == TEngineType::ElectricInductionMotor ) {
        if( false == mover.MainCircuitPowered()
         || mover.eimv[ eimv_Ipoj ] == 0.0 ) { return 0.0; }
        cur = mover.eimv[ eimv_Ipoj ] - mover.eimc[ eimc_f_I0 ];
        // below half an amp the rounded text ("%.0f A") and the arc would read 0 anyway; snap it
        // there, so a vehicle standing still does not flicker between 0 and +-1 A
        if( std::abs( cur ) < 0.5 ) { cur = 0.0; }
    }
    else {
        // the classic series motor and the diesel-electric keep what they had: the signed total,
        // and Im as the fallback for the vehicles that never fill Itot
        auto const itot { mover.Itot };
        cur = ( std::abs( itot ) < 0.5 ? mover.Im : itot );
    }
    // the sign means something on ONE kind of machine: the inverter drive. its current comes out of
    // the inverter and its direction is the direction of the POWER FLOW, so a machine that feeds back
    // into the line may show a (red) minus. everything else is non-negative here, and not because of
    // taste - the HUD this one replaces summed the two branch ammeters, fHCurrent[1] + fHCurrent[2],
    // and those come out of ShowCurrentP as floor( abs( Im ) ) (Mover.cpp:13526, total variant
    // floor( abs( Itot ) ) at :13529). a DC vehicle therefore never showed a minus, in ANY state: the
    // electrodynamic brake of an EP09 no more than the starting resistors of an EU07. that holds
    // again now, for every non-inverter drive:
    //  - a vehicle with no electric brake cannot feed anything back at all;
    //  - on a DC series motor the sign of Im only says which way the motor turns (MOVER.h:2225-2227),
    //    and Itot is that same current times the number of parallel branches (Mover.cpp:6236-6241),
    //    so a minus there is a direction artefact, never a feed-back.
    // an inverter drive still has to have the hardware behind it - the Vectron declares the ED without
    // any "DynamicBrake" key, see hud_has_electric_brake - and its composite (integrated drive+brake)
    // handle regenerates with DynamicBrakeFlag unset (Mover.cpp:6754-6763), which is why the test
    // stops at the engine type and the hardware.
    if( false == hud_has_electric_brake( mover ) ) { return std::abs( cur ); }
    if( mover.EngineType != TEngineType::ElectricInductionMotor ) { return std::abs( cur ); }
    // -0.0 would come out of "%.0f" as "-0 A"; a zero is a zero
    return ( cur == 0.0 ? 0.0 : cur );
}

// draws one string ALONG an arc: every character gets its own arc position and is rotated to the
// local tangent, which is what the layout draft does with SVG textPath. the sweep, the shared centre
// and the draw list are the frame; size/radius/colour/posPct/spacing/flip are the per-ring text
// parameters. arc-length based, so the spacing stays even whatever the glyph widths are (dividing
// the sweep by the character count would bunch them up). this used to be a lambda inside the
// renderer; it is a named type now (quality gate: a lambda over 20 lines becomes a function), and
// the drawing it does is unchanged.
struct arc_text_frame {
    ImDrawList * dl;
    ImVec2 centre;
    float a0;
    float a1;

    void draw( float size, float radius, ImU32 col, std::string const &text,
               float posPct, float spacing, bool flip ) const
    {
        if ( text.empty() || radius <= 1.0f )
            return;
        float const arclen { std::abs( radius * ( a1 - a0 ) ) };
        if ( arclen <= 1.0f )
            return;
        // split the string into COMPLETE utf-8 codepoints before measuring anything, so the
        // advance (and the arc position) is the width of one whole glyph whatever the language
        // is. The previous loop walked the string one byte at a time and handed each single
        // byte to ImGui as if it were a character: a lead byte on its own decodes to the
        // malformed marker U+FFFD (no glyph -> the fallback "?") and a continuation byte on its
        // own decodes to 0 (zero width, nothing drawn). Polish "Prąd" was therefore rendered
        // "Pr?d" and every accented label came out with wrong spacing. ImTextCharFromUtf8 is
        // ImGui's own decoder and reports the true length of a sequence - 1 byte for ASCII,
        // 2 for Latin Extended/Cyrillic/Greek, 3 for CJK and kana, 4 for the supplementary
        // planes - so no language needs its own special case here.
        struct cpt { std::size_t off; std::size_t len; float w; };
        std::vector<cpt> cps; cps.reserve( text.size() );
        float total { 0.0f };
        for ( std::size_t ci = 0; ci < text.size(); )
        {
            unsigned int cp { 0 };
            int const n { ImTextCharFromUtf8( &cp, text.data() + ci, text.data() + text.size() ) };
            // malformed input: a stray continuation byte or an impossible lead byte makes the
            // decoder report 0 bytes consumed, and a truncated sequence at the end of the
            // string reports 1 so the leftover byte still gets drawn (as U+FFFD, which is
            // exactly what ImGui's own text drawing does with it). Consuming a minimum of one
            // byte here keeps the loop moving - no infinite loop - and never drops a character.
            std::size_t used { n > 0 ? static_cast<std::size_t>( n ) : std::size_t { 1 } };
            used = std::min( used, text.size() - ci );
            std::string const piece { text, ci, used };
            float const w { ui_layer::font_mono->CalcTextSizeA( size, std::numeric_limits<float>::max(), 0.0f, piece.c_str() ).x };
            cps.push_back( { ci, used, w } );
            total += w + spacing;
            ci += used;
        }
        total = std::max( 0.0f, total - spacing );
        float cursor { arclen * ( posPct * 0.01f ) - total * 0.5f };   // centred on posPct
        for ( std::size_t ci = 0; ci < cps.size(); ++ci )
        {
            std::string const piece { text, cps[ ci ].off, cps[ ci ].len };
            float const amid { cursor + cps[ ci ].w * 0.5f };          // arc length of this glyph
            float const ang { a0 + ( a1 - a0 ) * ( amid / arclen ) };
            ImVec2 const gp { centre.x + std::cos( ang ) * radius, centre.y + std::sin( ang ) * radius };
            int const first { dl->VtxBuffer.Size };
            dl->AddText( ui_layer::font_mono, size, ImVec2( gp.x, gp.y - size * 0.5f ), col, piece.c_str() );
            int const last { dl->VtxBuffer.Size };
            float const rot { ang + ( flip ? -1.57079633f : 1.57079633f ) };
            float const ca { std::cos( rot ) };
            float const sa { std::sin( rot ) };
            for ( int vi = first; vi < last; ++vi )
            {
                ImDrawVert &v { dl->VtxBuffer[ vi ] };
                float const dx { v.pos.x - gp.x };
                float const dy { v.pos.y - gp.y };
                v.pos.x = gp.x + dx * ca - dy * sa;
                v.pos.y = gp.y + dx * sa + dy * ca;
            }
            cursor += cps[ ci ].w + spacing;
        }
    }
};

} // namespace

void
drivingaid_panel::update() {

    if( false == is_open ) { return; }

	text_lines.clear();

    std::tie( m_brokenvehicle, m_brokenend ) = broken_coupler_nearby( Global.pCamera.Pos );

    auto const *train { simulation::Train };
    auto const *controlled { ( train ? train->Dynamic() : nullptr ) };

    if( controlled == nullptr
     || controlled->Mechanik == nullptr ) { return; }

    auto const *mover = controlled->MoverParameters;
    auto const *vehicledriver = controlled->Mechanik;
    auto const *owner = controlled->ctOwner != nullptr ? controlled->ctOwner : controlled->Mechanik;
    // the numbers of the throttle and brakes lines, with the controls of the cab the driver is in
    auto const readout { drivingaid::compute( drivingaid::from_movers(
        *mover,
        *vehicledriver->Controlling(),
        drivingaid::cab_controls {
            train->ggLocalBrake.SubModel != nullptr,
            train->ggJointCtrl.SubModel != nullptr } ) ) };

    { // throttle, velocity, speed limits and grade
        std::string expandedtext;
        if( is_expanded ) {
            // grade
            std::string gradetext;
            auto const reverser { ( mover->DirActive > 0 ? 1 : -1 ) };
            if( auto const grade { controlled->VectorFront().y * 100 * ( controlled->DirectionGet() == reverser ? 1 : -1 ) * reverser }; std::abs( grade ) >= 0.25 ) {
                std::snprintf(
                    m_buffer.data(), m_buffer.size(),
				    STR_C(" Grade: %.1f%%%%"),
                    grade );
                gradetext = m_buffer.data();
            }
            // next speed limit
            auto const speedlimit { static_cast<int>( owner->VelDesired ) };
            auto const [ nextspeedlimit, nextspeedlimitdistance ] { owner->next_speed_limit() };
            std::string nextspeedlimittext;
            if( nextspeedlimit != speedlimit ) {
                std::snprintf(
                    m_buffer.data(), m_buffer.size(),
				    STR_C(", new limit: %d km/h in %.1f km"),
                    nextspeedlimit,
                    nextspeedlimitdistance * 0.001 );
                nextspeedlimittext = m_buffer.data();
            }
            // current speed and limit
            std::snprintf(
                m_buffer.data(), m_buffer.size(),
			    STR_C(" Speed: %d km/h (limit %d km/h%s)%s"),
                static_cast<int>( std::floor( mover->Vel ) ),
                speedlimit,
                nextspeedlimittext.c_str(),
                gradetext.c_str() );
            expandedtext = m_buffer.data();
        }
        // base data and optional bits put together
        std::snprintf(
            m_buffer.data(), m_buffer.size(),
            STR_C("Throttle: %3d+%d %c%s"),
            readout.throttle_first,
            readout.throttle_second,
            mover->SpeedCtrlUnit.IsActive ? 'T' :
		              mover->DirActive > 0          ? 'D' :
		              mover->DirActive < 0          ? 'R' :
		                                              'N',
            expandedtext.c_str());

        text_lines.emplace_back( m_buffer.data(), Global.UITextColor );
    }

    { // brakes, air pressure
        std::string expandedtext;
        if( is_expanded ) {
            std::snprintf (
                m_buffer.data(), m_buffer.size(),
			    STR_C(" Pressure: %.2f kPa (train pipe: %.2f kPa)"),
                mover->BrakePress * 100,
                mover->PipePress * 100 );
            expandedtext = m_buffer.data();
        }
        if( readout.ed_percent ) {
            // the ED braking demand, ahead of the expanded data (eu07/maszyna#558)
            std::snprintf(
                m_buffer.data(), m_buffer.size(),
                STR_C(" ED brake: %d%%%%"),
                *readout.ed_percent );
            expandedtext.insert( 0, m_buffer.data() );
        }
        if( readout.show_local_brake ) {
            std::snprintf(
                m_buffer.data(), m_buffer.size(),
                STR_C("Brakes: %5.1f+%-2.0f%c%s"),
                readout.brake_first,
                readout.local_brake,
                mover->SlippingWheels ? '!' : ' ',
                expandedtext.c_str() );
        }
        else {
            // the cab has no local brake control: its setting is left out (eu07/maszyna#555)
            std::snprintf(
                m_buffer.data(), m_buffer.size(),
                STR_C("Brakes: %5.1f%c%s"),
                readout.brake_first,
                mover->SlippingWheels ? '!' : ' ',
                expandedtext.c_str() );
        }

        text_lines.emplace_back( m_buffer.data(), Global.UITextColor );
    }

    { // alerter, hints
        std::string expandedtext;
        if( is_expanded ) {
            auto const stoptime { static_cast<int>( owner->ExchangeTime ) };
            if( stoptime > 0 ) {
                std::snprintf(
                    m_buffer.data(), m_buffer.size(),
				    STR_C(" Loading/unloading in progress (%d s left)"),
                    stoptime );
                expandedtext = m_buffer.data();
            }
            else {
                auto const trackobstacledistance { std::abs( owner->TrackObstacle() ) };
                if( trackobstacledistance <= 75.0 ) {
                    std::snprintf(
                        m_buffer.data(), m_buffer.size(),
					    STR_C(" Another vehicle ahead (distance: %.1f m)"),
                        trackobstacledistance );
                    expandedtext = m_buffer.data();
                }
            }
        }
        std::string textline =
		    mover->SecuritySystem.is_vigilance_blinking() && (train != nullptr ? train->fBlinkTimer > 0 : true) ? STR("!ALERTER! ") : "          ";
        textline +=
		    mover->SecuritySystem.is_cabsignal_blinking() ? STR("!SHP!") : "     ";

        text_lines.emplace_back( textline + "  " + expandedtext, Global.UITextColor );
    }
}

void
drivingaid_panel::render_contents() {
    ImGui::Checkbox(STR_C("expand"), &is_expanded);

    ImGui::SameLine();

    auto const *driver = (simulation::Train && simulation::Train->Dynamic()) ? simulation::Train->Dynamic()->Mechanik : nullptr;
    if (driver && (false == Global.RealisticControlMode || DebugModeFlag)) {
        bool autopilot = driver->AIControllFlag;
        if (ImGui::Checkbox(STR_C("autopilot"), &autopilot)) {
            command_relay commandrelay;
            commandrelay.post(autopilot ? user_command::aidriverenable : user_command::aidriverdisable, 0.0, 0.0, GLFW_PRESS, 0);
        }
    }

    // only the coupler the camera is next to, and the one of the vehicle it was coupled with
    if( m_brokenvehicle != nullptr && ImGui::Button( STR_C( "Repair coupler" ) ) ) {
        command_relay commandrelay;
        commandrelay.post( user_command::repaircoupler, m_brokenend, 0.0, GLFW_PRESS, 0, glm::vec3( 0.0f ), &m_brokenvehicle->name() );
    }

    ui_panel::render_contents();
}

void
scenario_panel::update() {

    if( false == is_open ) { return; }

    text_lines.clear();

    auto const *train { simulation::Train };
    auto const *controlled { ( train ? train->Dynamic() : nullptr ) };
    auto const &camera { Global.pCamera };
    m_nearest = false == FreeFlyModeFlag  ? controlled :
	            camera.m_owner != nullptr ? camera.m_owner :
	                                        std::get<TDynamicObject *>(simulation::Region->find_vehicle(camera.Pos, 20, false, false)); // w trybie latania lokalizujemy wg mapy
    if( m_nearest == nullptr ) { return; }
    auto const *owner { (
        m_nearest->Mechanik != nullptr && m_nearest->Mechanik->primary() ?
            m_nearest->Mechanik :
            m_nearest->ctOwner ) };
    if( owner == nullptr ) { return; }

    std::string textline =
        STR("Current task:") + "\n "
        + owner->OrderCurrent();

    text_lines.emplace_back( textline, Global.UITextColor );
}

void
scenario_panel::render() {

    if( false == is_open ) { return; }
    if( true == text_lines.empty() ) { return; }
    if( m_nearest == nullptr ) { return; } // possibly superfluous given the above but, eh

    auto flags =
        ImGuiWindowFlags_NoFocusOnAppearing
        | ImGuiWindowFlags_NoCollapse
        | ( size.x > 0 ? ImGuiWindowFlags_NoResize : 0 );

    if( size.x > 0 ) {
        ImGui::SetNextWindowSize( ImVec2S( size.x, size.y ) );
    }
    if( size_min.x > 0 ) {
        ImGui::SetNextWindowSizeConstraints( ImVec2S( size_min.x, size_min.y ), ImVec2S( size_max.x, size_max.y ) );
    }
    if( auto const panelname { ( title.empty() ? m_name : title ) + "###" + m_name }; true == ImGui::Begin( panelname.c_str(), &is_open, flags ) ) {
        // potential assignment section
        auto const *owner { (
            m_nearest->Mechanik != nullptr && m_nearest->Mechanik->primary() ?
                m_nearest->Mechanik :
                m_nearest->ctOwner ) };
        if( owner != nullptr ) {
            auto const assignmentheader { STR("Assignment") };
            if( false == owner->assignment().empty()
             && true == ImGui::CollapsingHeader(assignmentheader.c_str()) ) {
                ImGui::TextWrapped( "%s", owner->assignment().c_str() );
                ImGui::Separator();
            }
        }
        // current task
        for( auto const &line : text_lines ) {
            ImGui::TextColored( ImVec4( line.color.r, line.color.g, line.color.b, line.color.a ), "%s", line.data.c_str() );
        }
        // hints
        if (owner != nullptr && true == ImGui::CollapsingHeader( STR_C("Hints"), ImGuiTreeNodeFlags_DefaultOpen )) {
            for( auto const &hint : owner->m_hints ) {
                auto const isdone { std::get<TController::hintpredicate>( hint )( std::get<float>( hint ) ) };
                auto const hintcolor{ (
                    isdone ?
                        colors::uitextgreen :
                        Global.UITextColor ) };
                ImGui::PushStyleColor( ImGuiCol_Text, { hintcolor.r, hintcolor.g, hintcolor.b, hintcolor.a } );
                ImGui::TextWrapped( Translations.lookup_c(driver_hints_texts[(size_t)std::get<driver_hint>( hint )], true), std::get<float>( hint ) );
                ImGui::PopStyleColor();
            }
        }
    }
    ImGui::End();
}

void
timetable_panel::update() {

	if( false == is_open ) { return; }

	text_lines.clear();
	m_tablelines.clear();

	auto const *train { simulation::Train };
	auto const *controlled { ( train ? train->Dynamic() : nullptr ) };
	auto const &camera { Global.pCamera };
	auto const &time { simulation::Time.data() };

    { // current time
        std::snprintf(
            m_buffer.data(), m_buffer.size(),
		    STR_C("%-*.*s    Time: %d:%02d:%02d"),
            37, 37,
		    STR_C("Timetable"),
            time.wHour,
            time.wMinute,
            time.wSecond );

        title = m_buffer.data();
    }

    auto *vehicle { (
        false == FreeFlyModeFlag ? controlled :
        camera.m_owner != nullptr ? camera.m_owner :
        std::get<TDynamicObject *>( simulation::Region->find_vehicle( camera.Pos, 20, false, false ) ) ) }; // w trybie latania lokalizujemy wg mapy

    if( vehicle == nullptr ) { return; }
    // if the nearest located vehicle doesn't have a direct driver, try to query its owner
    auto const *owner = vehicle->Mechanik != nullptr && vehicle->Mechanik->primary() ? vehicle->Mechanik : vehicle->ctOwner;
    if( owner == nullptr ) { return; }

    auto const &table = owner->TrainTimetable();

    // destination
    {
        // the names with the letters their files have, in utf-8 as the user interface takes it
        auto textline = table.relation_caption();
        if( false == textline.empty() ) {
            textline += " (" + data_name_to_utf8( owner->TrainName(), true ) + ")";
        }
        text_lines.emplace_back( textline, Global.UITextColor );
    }

    if( false == is_expanded ) {
        // next station
        auto const nextstation = owner->NextStop();
        if( false == nextstation.empty() ) {
            // jeśli jest podana relacja, to dodajemy punkt następnego zatrzymania
            auto textline = " -> " + nextstation;

            text_lines.emplace_back( textline, Global.UITextColor );
        }
    }

    if( is_expanded ) {

        if( owner->is_train() ) {
            // consist data
            auto consistmass { owner->fMass };
            auto consistlength { owner->fLength };
            if( false == owner->is_dmu()
             && false == owner->is_emu() ) {
				//odejmij lokomotywy czynne, a przynajmniej aktualną
				consistmass -= owner->pVehicle->MoverParameters->TotalMass;
				// subtract potential other half of a two-part vehicle
				if( auto const *previous { owner->pVehicle->Prev( coupling::permanent ) }; previous != nullptr ) { consistmass -= previous->MoverParameters->TotalMass; }
				auto const *next { owner->pVehicle->Next( coupling::permanent ) };
				if( next != nullptr ) { consistmass -= next->MoverParameters->TotalMass; }
			}
            std::snprintf(
                m_buffer.data(), m_buffer.size(),
                STR_C("Consist weight: %d t (specified) %d t (actual)\nConsist length: %d m"),
                static_cast<int>( table.LocLoad ),
                static_cast<int>( consistmass / 1000 ),
                static_cast<int>( consistlength ) );

            text_lines.emplace_back( m_buffer.data(), Global.UITextColor );
            text_lines.emplace_back( "", Global.UITextColor );
        }

        if( 0 == table.StationCount ) {
            // only bother if there's stations to list
            text_lines.emplace_back( STR("(no timetable)"), Global.UITextColor );
        }
        else {
			// header
			m_tablelines.emplace_back(U8("┌─────┬────────────────────────────────────┬─────────┬─────┐"), Global.UITextColor);

            TMTableLine const *tableline;
            for( int i = table.StationStart; i <= table.StationCount; ++i ) {
                // wyświetlenie pozycji z rozkładu
                tableline = table.TimeTable + i; // linijka rozkładu

                bool vmaxchange { true };
                if( i > table.StationStart ) {
                    auto const *previoustableline { tableline - 1 };
                    if( tableline->vmax == previoustableline->vmax ) {
                        vmaxchange = false;
                    }
                }
                std::string vmax { "   " };
                if( true == vmaxchange ) {
                    vmax += to_string( tableline->vmax, 0 );
                    vmax = vmax.substr( vmax.size() - 3, 3 ); // z wyrównaniem do prawej
                }
                // the name as the timetable spells it; in utf-8 a letter can take more than one byte, the column is
                // 34 characters wide
                auto const station { utf8_fit( timetable::station_caption( tableline->StationLabel ), 34 ) };
                auto const location { (
                    ( tableline->km > 0.0 ?
                        to_string( tableline->km, 2 ) :
                        "" )
                    + "                                  " )
                    .substr( 0, 34 - tableline->StationWare.size() ) };
                auto const arrival { (
                    tableline->Ah >= 0 ?
                        std::to_string( int( 100 + tableline->Ah ) ).substr( 1, 2 ) + ":" + to_minutes_str( tableline->Am, true, 3 ) :
                        U8("  │   ") ) };
                auto const departure { (
                    tableline->Dh >= 0 ?
                        std::to_string( int( 100 + tableline->Dh ) ).substr( 1, 2 ) + ":" + to_minutes_str( tableline->Dm, true, 3 ) :
                        U8("  │   ") ) };
                auto const candepart { (
                       table.StationStart < table.StationIndex
                    && i < table.StationIndex
                    && ( tableline->Ah < 0 // pass-through, always valid
                      || tableline->is_maintenance // maintenance stop, always valid
                      || time.wHour * 60 + time.wMinute + time.wSecond * 0.0167 >= static_cast<float>(tableline->Dh * 60) + tableline->Dm ) ) };
                auto const loadchangeinprogress { ( static_cast<int>(std::ceil(-1.0 * owner->fStopTime)) > 0 ) };
                auto const isatpassengerstop { true == owner->IsAtPassengerStop && vehicle->MoverParameters->Vel < 1.0 };
                auto const traveltime { (
                    i < 2 ? "   " :
                    tableline->Ah >= 0 ? to_minutes_str( static_cast<float>(CompareTime( table.TimeTable[ i - 1 ].Dh, table.TimeTable[ i - 1 ].Dm, tableline->Ah, tableline->Am )), false, 3 ) :
                    to_minutes_str( static_cast<float>(std::max( 0.0, CompareTime( table.TimeTable[ i - 1 ].Dh, table.TimeTable[ i - 1 ].Dm, tableline->Dh, tableline->Dm ) - 0.5 )), false, 3 ) ) };
                auto const linecolor { (
                    i != table.StationStart ? Global.UITextColor :
                    loadchangeinprogress ? colors::uitextred :
                    candepart ? colors::uitextgreen : // czas minął i odjazd był, to nazwa stacji będzie na zielono
                    isatpassengerstop ? colors::uitextorange :
                    Global.UITextColor ) };
                std::string const trackcount{ ( tableline->TrackNo == 1 ? U8(" ┃  ") : U8(" ║  " )) };
                m_tablelines.emplace_back(U8("│ ") + vmax + U8(" │ ") + station + trackcount + arrival + U8(" │ ") + traveltime + U8(" │"),
                    linecolor );
                m_tablelines.emplace_back(U8("│     │ ") + location + tableline->StationWare + trackcount + departure + U8(" │     │"),
                    linecolor );
                // divider/footer
                if( i < table.StationCount ) {
                    auto const *nexttableline { tableline + 1 };
                    std::string const vmaxnext{ ( tableline->vmax == nexttableline->vmax ? U8("│     ├") : U8("├─────┼") ) };
                    auto const trackcountnext{ ( nexttableline->TrackNo == 1 ? U8("╂") : U8("╫") ) };
                    m_tablelines.emplace_back(
                        vmaxnext + U8("────────────────────────────────────") + trackcountnext + U8("─────────┼─────┤"),
                        Global.UITextColor );
                }
                else {
                    m_tablelines.emplace_back(
                        U8("└─────┴────────────────────────────────────┴─────────┴─────┘"),
                        Global.UITextColor );
                }
            }
        }
    } // is_expanded
}

void
timetable_panel::render() {

    if( false == is_open ) { return; }
    if( true  == text_lines.empty() ) { return; }

	ImGui::PushFont(ui_layer::font_mono);

    auto flags =
        ImGuiWindowFlags_NoFocusOnAppearing
        | ImGuiWindowFlags_NoCollapse
        | ( size.x > 0 ? ImGuiWindowFlags_NoResize : 0 );

    // HACK: Make sure the timetable window is of correct width when using a larger font size.
    float horizontalScale = Global.ui_fontsize / 13;
    if( size.x > 0 ) {
        ImGui::SetNextWindowSize( ImVec2S( size.x * horizontalScale, size.y ) );
    }
    if( size_min.x > 0 ) {
        ImGui::SetNextWindowSizeConstraints( ImVec2S( size_min.x * horizontalScale, size_min.y ), ImVec2S( size_max.x * horizontalScale, size_max.y ) );
    }
    if( auto const panelname { ( title.empty() ? m_name : title ) + "###" + m_name }; true == ImGui::Begin( panelname.c_str(), &is_open, flags ) ) {
        for( auto const &line : text_lines ) {
            ImGui::TextColored( ImVec4( line.color.r, line.color.g, line.color.b, line.color.a ), "%s", line.data.c_str() );
        }
        if( is_expanded ) {
            ImGui::PushStyleVar( ImGuiStyleVar_ItemSpacing, ImVec2( 1, 0 ) );
            for( auto const &line : m_tablelines ) {
                ImGui::TextColored( ImVec4( line.color.r, line.color.g, line.color.b, line.color.a ), "%s", line.data.c_str() );
            }
            ImGui::PopStyleVar();
        }
    }
    ImGui::End();

	ImGui::PopFont();
}

void debug_panel::update()
{

	if( false == is_open ) { return; }

	// input item bindings
	m_input.train = simulation::Train;
	m_input.controlled = m_input.train ? m_input.train->Dynamic() : nullptr;
	m_input.camera = &Global.pCamera;
	m_input.vehicle = false == FreeFlyModeFlag           ? m_input.controlled :
	                  m_input.camera->m_owner != nullptr ? m_input.camera->m_owner :
	                                                       std::get<TDynamicObject *>(simulation::Region->find_vehicle(m_input.camera->Pos, 20, false, false)); // w trybie latania lokalizujemy wg mapy
	m_input.mover =
	    m_input.vehicle != nullptr ? m_input.vehicle->MoverParameters : nullptr;
	m_input.mechanik = m_input.vehicle != nullptr ? m_input.vehicle->Mechanik : nullptr;

	// header section
	text_lines.clear();

	auto textline = "Version " + Global.asVersion;

	text_lines.emplace_back( textline, Global.UITextColor );

	// sub-sections
	m_vehiclelines.clear();
	m_enginelines.clear();
	m_ailines.clear();
	m_scantablelines.clear();
	m_scenariolines.clear();
	m_eventqueuelines.clear();
	m_powergridlines.clear();
	m_cameralines.clear();
	m_rendererlines.clear();
	m_couplerlines.clear();
#ifdef WITH_UART
    m_uartlines.clear();
#endif

	update_section_vehicle( m_vehiclelines );
	update_section_engine( m_enginelines );
	update_section_ai( m_ailines );
	update_section_scantable( m_scantablelines );
	update_section_scenario( m_scenariolines );
	update_section_eventqueue( m_eventqueuelines );
	update_section_powergrid( m_powergridlines );
	update_section_camera( m_cameralines );
	update_section_renderer( m_rendererlines );
	update_section_coupler( m_couplerlines );
#ifdef WITH_UART
    update_section_uart(m_uartlines);
#endif
}

void
debug_panel::render() {
    if( false == is_open ) { return; }

	ImGui::PushFont(ui_layer::font_mono);

    auto flags =
        ImGuiWindowFlags_NoFocusOnAppearing
        | ImGuiWindowFlags_NoCollapse
        | ( size.x > 0 ? ImGuiWindowFlags_NoResize : 0 );

    if( size.x > 0 ) {
        ImGui::SetNextWindowSize( ImVec2S( size.x, size.y ) );
    }
    if( size_min.x > 0 ) {
        ImGui::SetNextWindowSizeConstraints( ImVec2S( size_min.x, size_min.y ), ImVec2S( size_max.x, size_max.y ) );
    }
    if( auto const panelname { ( title.empty() ? m_name : title ) + "###" + m_name }; true == ImGui::Begin( panelname.c_str(), &is_open, flags ) ) {
        // header section
        for( auto const &line : text_lines ) {
            ImGui::TextColored( ImVec4( line.color.r, line.color.g, line.color.b, line.color.a ), "%s", line.data.c_str() );
        }
        // sections
        ImGui::Separator();
        if (true == render_section( "Vehicle", m_vehiclelines ) && DebugModeFlag && m_input.mover && m_input.mover->DamageFlag != 0 && true == ImGui::Button( "Stop and repair consist" )) {
            command_relay commandrelay;
            commandrelay.post(user_command::resetconsist, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(0.0f), &m_input.vehicle->name());
        }
        render_section_coupler();
        render_section( "Vehicle Engine", m_enginelines );
        render_section( "Vehicle AI", m_ailines );
        render_section( "Vehicle Scan Table", m_scantablelines );
        render_section_scenario();
        render_section_eventqueue();
        if( true == render_section( "Power Grid", m_powergridlines ) ) {
            // traction state debug
            ImGui::Checkbox( "Debug Traction", &DebugTractionFlag );
        }
        render_section( "Camera", m_cameralines );
        render_section( "Gfx Renderer / Statistics", m_rendererlines );
        render_section_settings();
		render_section_developer(); // Developer tools
#ifdef WITH_UART
        if(true == render_section( "UART", m_uartlines)) {
            auto ports_num = static_cast<int>(UartStatus.available_ports.size());
            auto avlports = new char*[ports_num];
            for (int i=0; i < ports_num; i++) {
                avlports[i] = (char *) UartStatus.available_ports[i].c_str();
            }
            ImGui::Combo("Port", &UartStatus.selected_port_index, avlports, ports_num);
            ImGui::Combo("Baud", &UartStatus.selected_baud_index, uart_baudrates_list, static_cast<int>(uart_baudrates_list_num));
            ImGui::Checkbox("Enabled", &UartStatus.enabled);
        }
#endif
        // toggles
        ImGui::Separator();
        bool flag = DebugModeFlag;
        if (ImGui::Checkbox("Debug Mode", &flag)) {
            command_relay commandrelay;
            commandrelay.post(user_command::debugtoggle, 0.0, 0.0, GLFW_RELEASE, 0);
        }
    }

    ImGui::End();
    ImGui::PopFont();
}

bool
debug_panel::render_section_scenario() {

    if( false == render_section( "Scenario", m_scenariolines ) ) { return false; }
    if( Application.is_client() ) {
        // NOTE: simulation clients can't adjust scenarion state, this is reserved for the server/standalone instance
        return true;
    }
    // fog slider
    {
        auto fogrange = std::log( Global.fFogEnd );
        if( ImGui::SliderFloat(
            ( to_string( std::exp( fogrange ), 0, 5 ) + " m###fogend" ).c_str(), &fogrange, std::log( 10.0f ), std::log( 50000.0f ), "Fog distance" ) ) {
            command_relay commandrelay;
            commandrelay.post(
                user_command::setweather,
                std::clamp( std::exp( fogrange ), 10.0f, 50000.0f ),
                Global.Overcast,
                GLFW_PRESS, 0 );
        }
    }
	{
		auto Airtemperature = Global.AirTemperature;
		if (ImGui::SliderFloat(
		        (to_string(Airtemperature, 1) + " deg C###Airtemperature").c_str(),
		        &Airtemperature, -35.0f, 40.0f, "Air Temperature"))
		{
			command_relay commandrelay;
            commandrelay.post(
                user_command::settemperature, 
                std::clamp(Airtemperature, -35.0f, 40.0f),
			           Global.Overcast,
                GLFW_PRESS, 0 );
		}
	}
    // cloud cover slider
    {
        if( ImGui::SliderFloat(
            ( to_string( Global.Overcast, 2, 5 ) + " (" + Global.Weather + ")###overcast" ).c_str(), &Global.Overcast, 0.0f, 2.0f, "Cloud cover" ) ) {
            command_relay commandrelay;
            commandrelay.post(
                user_command::setweather,
                Global.fFogEnd,
                std::clamp( Global.Overcast, 0.0f, 2.0f ),
                GLFW_PRESS, 0 );
        }
    }
    // day of year slider
    {
        if( ImGui::SliderFloat(
            ( to_string( Global.fMoveLight, 0, 5 ) + " (" + Global.Season + ")###movelight" ).c_str(), &Global.fMoveLight, 0.0f, 364.0f, "Day of year" ) ) {
            command_relay commandrelay;
            commandrelay.post(
                user_command::setdatetime,
                std::clamp( Global.fMoveLight, 0.0f, 365.0f ),
                simulation::Time.data().wHour * 60 + simulation::Time.data().wMinute,
                GLFW_PRESS, 0 );
        }
    }
    // dynamic material update checkbox
    ImGui::Checkbox( "Update Item Materials", &Global.UpdateMaterials );
    if (DebugModeFlag && ImGui::Checkbox( "Force Daylight", &Global.FakeLight )) {
        simulation::Environment.on_daylight_change();
    }
    // advanced options, only visible in debug mode
    if( DebugModeFlag ) {
        // time of day slider
        {
            ImGui::PushStyleColor( ImGuiCol_Text, { Global.UITextColor.r, Global.UITextColor.g, Global.UITextColor.b, Global.UITextColor.a } );
            ImGui::TextUnformatted( "CAUTION: time change will affect simulation state" );
            ImGui::PopStyleColor();
            auto time = simulation::Time.data().wHour * 60 + simulation::Time.data().wMinute;
            auto const timestring {
                std::string( std::to_string( int( 100 + simulation::Time.data().wHour ) ).substr( 1, 2 )
                    + ":"
                    + std::string( std::to_string( int( 100 + simulation::Time.data().wMinute ) ).substr( 1, 2 ) ) ) };
            if( ImGui::SliderInt( ( timestring + " (" + Global.Period + ")###simulationtime" ).c_str(), &time, 0, 1439, "Time of day" ) ) {
                command_relay commandrelay;
                commandrelay.post(
                    user_command::setdatetime,
                    Global.fMoveLight,
                    std::clamp( time, 0, 1439 ),
                    GLFW_PRESS, 0 );
            }
        }
        // time acceleration
        {
            auto timerate { ( Global.fTimeSpeed == 60 ? 4 : Global.fTimeSpeed == 20 ? 3 : Global.fTimeSpeed == 5 ? 2 : 1 ) };
            if( ImGui::SliderInt( ( "x " + to_string( Global.fTimeSpeed, 0 ) + "###timeacceleration" ).c_str(), &timerate, 1, 4, "Time acceleration" ) ) {
                Global.fTimeSpeed = timerate == 4 ? 60 : timerate == 3 ? 20 : timerate == 2 ? 5 : 1;
            }
        }
		// base draw range slider
		{
			ImGui::TextUnformatted("CAUTION: drawing range change can affect FPS badly");
			auto drawrange = std::log(Global.BaseDrawRange);
			if (ImGui::SliderFloat(
				(to_string(std::exp(drawrange), 0, 5) + " m###drawrange").c_str(), &drawrange, std::log(100.0f), std::log(50000.0f), "Base drawing range")) {
				Global.BaseDrawRange = std::clamp(std::exp(drawrange), 100.0f, 50000.0f);
			}
		}
    }

    return true;
}

bool
debug_panel::render_section_eventqueue() {

    if( false == ImGui::CollapsingHeader( "Scenario Event Queue" ) ) { return false; }
    // event queue name filter
    ImGui::PushItemWidth( -1 );
    ImGui::InputTextWithHint( "", "Search event queue", m_eventsearch.data(), m_eventsearch.size() );
    ImGui::PopItemWidth();
    // event queue
    render_section( m_eventqueuelines );
    // event queue activator filter
    ImGui::Checkbox( "By This Vehicle Only", &m_eventqueueactivevehicleonly );

    return true;
}

void
debug_panel::update_section_vehicle( std::vector<text_line> &Output ) {

    if( m_input.vehicle == nullptr ) { return; }
    if( m_input.mover == nullptr ) { return; }

    auto const &vehicle { *m_input.vehicle };
    auto const &mover { *m_input.mover };

    auto const isowned { /* ( vehicle.Mechanik == nullptr ) && */ vehicle.ctOwner != nullptr && vehicle.ctOwner->Vehicle() != m_input.vehicle };
    auto const isdieselenginepowered { mover.EngineType == TEngineType::DieselElectric || mover.EngineType == TEngineType::DieselEngine };
    auto const isdieselinshuntmode { mover.ShuntMode && mover.EngineType == TEngineType::DieselElectric };

    std::snprintf(
        m_buffer.data(), m_buffer.size(),
	    STR_C("Name: %s%s\nLoad: %.0f %s\nStatus: %s%s\nCouplers:\n front: %s\n rear:  %s"),
        mover.Name.c_str(),
        std::string( isowned ? STR(", owned by: ") + vehicle.ctOwner->OwnerName() : "" ).c_str(),
        mover.LoadAmount,
        mover.LoadType.name.c_str(),
        mover.EngineDescription( 0 ).c_str(),
        // TODO: put wheel flat reporting in the enginedescription()
        std::string( mover.WheelFlat > 0.01 ? " Flat: " + to_string( mover.WheelFlat, 1 ) + " mm" : "" ).c_str(),
        update_vehicle_coupler( end::front ).c_str(),
        update_vehicle_coupler( end::rear ).c_str() );

    Output.emplace_back( std::string{ m_buffer.data() }, Global.UITextColor );

    std::snprintf(
        m_buffer.data(), m_buffer.size(),
        STR_C("Devices: %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%s%s\nPower transfers: %.0f@%.0f%s[%.0f]%s%.0f@%.0f :: %.0f@%.0f%s[%.0f]%s%.0f@%.0f"),
        // devices
        mover.Battery ? 'B' : '.',
        mover.PantsValve.is_active ? '+' : '.',
        mover.Pantographs[end::rear].valve.is_active  ? 'O' :
	              mover.Pantographs[end::rear].valve.is_enabled ? 'o' :
	                                                              '.',
        mover.Pantographs[end::front].valve.is_active  ? 'P' :
	              mover.Pantographs[end::front].valve.is_enabled ? 'p' :
	                                                               '.',
        mover.PantPressLockActive   ? '!' :
	              mover.PantPressSwitchActive ? '*' :
	                                            '.',
        mover.WaterPump.is_active        ? 'W' :
	              false == mover.WaterPump.breaker ? '-' :
	              mover.WaterPump.is_enabled       ? 'w' :
	                                                 '.',
        mover.WaterHeater.is_damaged       ? '!' :
	              mover.WaterHeater.is_active        ? 'H' :
	              false == mover.WaterHeater.breaker ? '-' :
	              mover.WaterHeater.is_enabled       ? 'h' :
	                                                   '.',
        mover.FuelPump.is_active  ? 'F' :
	              mover.FuelPump.is_enabled ? 'f' :
	                                          '.',
        mover.OilPump.is_active  ? 'O' :
	              mover.OilPump.is_enabled ? 'o' :
	                                         '.',
        mover.Mains ? 'M' : '.',
        mover.FuseFlag ? '!' : '.',
        mover.ConverterFlag                ? 'X' :
	              false == mover.ConverterAllowLocal ? '-' :
	              mover.ConverterAllow               ? 'x' :
	                                                   '.',
        mover.ConvOvldFlag ? '!' : '.',
        mover.CompressorFlag                                                                                  ? 'C' :
	              false == mover.CompressorAllowLocal                                                                   ? '-' :
	              mover.CompressorAllow || (mover.CompressorStart == start_t::automatic && mover.CompressorSpeed > 0.0) ? 'c' :
	                                                                                                                      '.',
        mover.CompressorGovernorLock ? '!' : '.',
        mover.StLinSwitchOff        ? '-' :
	              mover.ControlPressureSwitch ? '!' :
	              mover.StLinFlag             ? '+' :
	                                            '.',
        mover.Heating      ? 'H' :
	              mover.HeatingAllow ? 'h' :
	                                   '.',
        mover.Hamulec->Releaser() ? '^' : '.',
        std::string( m_input.mechanik ? STR(" R") + ( mover.Radio && mover.RadioFuse ? std::to_string( m_input.mechanik->iRadioChannel ) : "-" ) : "" ).c_str(),
        std::string( isdieselenginepowered ? STR(" oil pressure: ") + to_string( mover.OilPump.pressure, 2 )  : "" ).c_str(),
        // power transfers
        // 3000v
        mover.Couplers[ end::front ].power_high.voltage,
        mover.Couplers[ end::front ].power_high.current,
        std::string( mover.Couplers[ end::front ].power_high.is_local ? ":" : ":=" ).c_str(),
        mover.EngineVoltage,
        std::string( mover.Couplers[ end::rear ].power_high.is_local ? ":" : "=:" ).c_str(),
        mover.Couplers[ end::rear ].power_high.voltage,
        mover.Couplers[ end::rear ].power_high.current,
        // 110v
        mover.Couplers[ end::front ].power_110v.voltage,
        mover.Couplers[ end::front ].power_110v.current,
        std::string( mover.Couplers[ end::front ].power_110v.is_local ? ":" : ":=" ).c_str(),
        mover.PowerCircuits[ 1 ].first,
        std::string( mover.Couplers[ end::rear ].power_110v.is_local ? ":" : "=:" ).c_str(),
        mover.Couplers[ end::rear ].power_110v.voltage,
        mover.Couplers[ end::rear ].power_110v.current );

    Output.emplace_back( m_buffer.data(), Global.UITextColor );

    std::snprintf(
        m_buffer.data(), m_buffer.size(),
	    STR_C("Controllers:\n master: %d(%d), secondary: %s\nEngine output: %.1f, current: %.0f\nRevolutions:\n engine: %.0f, motors: %.0f\n engine fans: %.0f, motor fans: %.0f+%.0f, cooling fans: %.0f+%.0f"),
        // controllers
        mover.MainCtrlPos,
        mover.MainCtrlActualPos,
        std::string( isdieselinshuntmode ? to_string( mover.AnPos, 2 ) + STR(" (shunt mode)") : std::to_string( mover.ScndCtrlPos ) + "(" + std::to_string( mover.ScndCtrlActualPos ) + ")" ).c_str(),
        // engine
        mover.EnginePower,
        std::abs( mover.TrainType == dt_EZT ? mover.ShowCurrent( 0 ) : mover.Im ),
        // revolutions
        std::abs( mover.enrot ) * 60,
        std::abs( mover.nrot ) * mover.Transmision.Ratio * 60,
        mover.RventRot * 60,
        std::abs( mover.MotorBlowers[end::front].revolutions ),
        std::abs( mover.MotorBlowers[end::rear].revolutions ),
        mover.dizel_heat.rpmw,
        mover.dizel_heat.rpmw2 );

    std::string textline { m_buffer.data() };

    if( isdieselenginepowered ) {
        std::snprintf(
            m_buffer.data(), m_buffer.size(),
		    STR_C("\nTemperatures:\n engine: %.2f, oil: %.2f, water: %.2f%c%.2f"),
            mover.dizel_heat.Ts,
            mover.dizel_heat.To,
            mover.dizel_heat.temperatura1,
            mover.WaterCircuitsLink ? '-' : '|',
            mover.dizel_heat.temperatura2 );
        textline += m_buffer.data();
    }

    Output.emplace_back( textline, Global.UITextColor );

    std::snprintf(
        m_buffer.data(), m_buffer.size(),
        STR_C("Brakes:\n train: %.2f (mode: %d, delay: %s, load flag: %d)\n independent: %.2f (%.2f), manual: %.2f, spring: %.2f\nBrake cylinder pressures:\n train: %.2f, independent: %.2f, status: 0x%.2x\nPipe pressures:\n brake: %.2f (hat: %.2f), main: %.2f, control: %.2f\nTank pressures:\n auxiliary: %.2f, main: %.2f, control: %.2f"),
        // brakes
        mover.fBrakeCtrlPos,
        mover.BrakeOpModeFlag,
        update_vehicle_brake().c_str(),
        mover.LoadFlag,
        mover.LocalBrakePosA,
        mover.LocalBrakePosAEIM,
        static_cast<float>(mover.ManualBrakePos) / static_cast<float>(ManualBrakePosNo),
        mover.SpringBrake.Activate ? 1.f : 0.f,
        // cylinders
        mover.BrakePress,
        mover.LocBrakePress,
        mover.Hamulec->GetBrakeStatus(),
        // pipes
        mover.PipePress,
        mover.BrakeCtrlPos2,
        mover.ScndPipePress,
        mover.CntrlPipePress,
        // tanks
        mover.Hamulec->GetBRP(),
        mover.Compressor,
        mover.Hamulec->GetCRP() );

    textline = m_buffer.data();

    if( mover.EnginePowerSource.SourceType == TPowerSource::CurrentCollector ) {
        std::snprintf(
            m_buffer.data(), m_buffer.size(),
		    STR_C(" pantograph: %.2f%cMT"),
            mover.PantPress,
            mover.bPantKurek3 ? '-' : '|' );
        textline += m_buffer.data();
    }

	Output.emplace_back( textline, Global.UITextColor );

    std::snprintf(
        m_buffer.data(), m_buffer.size(),
	    STR_C("Forces:\n tractive: %.1f, brake: %.1f, drag: %.2f%s\nAcceleration:\n tangential: %.2f, normal: %.2f (path radius: %s)\nVelocity: %.2f, distance traveled: %.2f\nPosition: [%.2f, %.2f, %.2f]"),
        // forces
        mover.Ft * 0.001f * ( mover.CabOccupied ? mover.CabOccupied : vehicle.ctOwner ? vehicle.ctOwner->Controlling()->CabOccupied : 1 ) + 0.001f,
        mover.Fb * 0.001f,
	    mover.FrictionForce() * 0.001f,
        mover.SlippingWheels ? " (!)" : "",
        // acceleration
        mover.AccSVBased,
        mover.AccN + 0.001f,
        std::string( std::abs( mover.RunningShape.R ) > 15000.0 ? "~0" : to_string( mover.RunningShape.R, 0 ) ).c_str(),
        // velocity
        vehicle.GetVelocity(),
        mover.DistCounter,
        // position
        vehicle.GetPosition().x,
        vehicle.GetPosition().y,
        vehicle.GetPosition().z );

    Output.emplace_back( m_buffer.data(), Global.UITextColor );

    if( mover.EnginePowerSource.SourceType == TPowerSource::CurrentCollector ) {
        std::snprintf(m_buffer.data(), m_buffer.size(), STR_C("Power use:\n drawn:    %.1f kWh\n returned: %.1f kWh\n balance:  %.1f kWh"),
                      mover.EnergyMeter.first, -mover.EnergyMeter.second, mover.EnergyMeter.first + mover.EnergyMeter.second);
        textline = m_buffer.data();
        if( DebugTractionFlag ) {
            for( int i = 0; i < vehicle.iAnimType[ ANIM_PANTS ]; ++i ) { // pętla po wszystkich pantografach
                auto const *p { vehicle.pants[ i ].fParamPants };
                if( p && p->hvPowerWire ) {
                    auto const *powerwire { p->hvPowerWire };
                    std::snprintf(
                        m_buffer.data(), m_buffer.size(),
                        STR_C("\n pantograph %d: drawing from: [%s, %s] through: [%.2f, %.2f, %.2f]"),
                        i,
                        powerwire->psPower[ 0 ] ? powerwire->psPower[ 0 ]->name().c_str() : "none",
                        powerwire->psPower[ 1 ] ? powerwire->psPower[ 1 ]->name().c_str() : "none",
                        powerwire->pPoint1[ 0 ],
                        powerwire->pPoint1[ 1 ],
                        powerwire->pPoint1[ 2 ] );
                    textline += m_buffer.data();
                }
            }
        }
	Output.emplace_back( textline, Global.UITextColor );
    }

	if (!std::isnan(last_time)) {
		double dt = Timer::GetTime() - last_time;
		AccN_jerk_graph.update(static_cast<float>((mover.AccN - last_AccN) / dt));
		AccN_acc_graph.update(static_cast<float>(mover.AccN));
	}

	last_AccN = static_cast<float>(mover.AccN);
	last_time = Timer::GetTime();
}

void debug_panel::graph_data::update(float val) {
	data[pos] = val;

	pos++;
	if (pos >= data.size())
		pos = 0;

	last_val = val;
}

void debug_panel::graph_data::render() {
	ImGui::PushID(this);
	ImGui::SliderFloat(STR_C("##Range"), &range, 0.5f, 60.0f, "%.1f");
	ImGui::PlotLines("##plot", data.data(), static_cast<int>(data.size()), static_cast<int>(pos), nullptr, 0.0f, range, ImVec2(0, 100));
	ImGui::PopID();
}

std::string
debug_panel::update_vehicle_coupler( int const Side ) {
    // NOTE: mover and vehicle are guaranteed to be valid by the caller
    auto const &mover { *m_input.mover };

    std::string const controltype{ ( mover.Couplers[ Side ].control_type.empty() ? "[*]" : "[" + mover.Couplers[ Side ].control_type + "]" ) };
    std::string couplerstatus { STR("none") };
    std::string const adapterstatus { ( mover.Couplers[ Side ].adapter_type == TCouplerType::NoCoupler ? "" : "[A]" ) };

	auto const *connected { m_input.vehicle->MoverParameters->Neighbours[ Side ].vehicle };

    if( connected == nullptr ) {

        return controltype + " " + couplerstatus + " " + adapterstatus;
    }

    std::snprintf(
        m_buffer.data(), m_buffer.size(),
        "%s %s %s[%d] (%.1f m)",
        controltype.c_str(),
        connected->name().c_str(),
        adapterstatus.c_str(),
        mover.Couplers[ Side ].CouplingFlag,
        mover.Neighbours[ Side ].distance );

	return { m_buffer.data() };
}

std::string
debug_panel::update_vehicle_brake() const {
	// NOTE: mover is guaranteed to be valid by the caller
	auto const &mover { *m_input.mover };

	std::string brakedelay;

	std::vector<std::pair<int, std::string>> delays {
		{ bdelay_G, "G" },
		{ bdelay_P, "P" },
		{ bdelay_R, "R" },
		{ bdelay_M, "+Mg" } };

	for( auto const &[delayflag, delayname] : delays ) {
		if( ( mover.BrakeDelayFlag & delayflag ) == delayflag ) {
			brakedelay += delayname;
		}
	}

	return brakedelay;
}

void
debug_panel::update_section_engine( std::vector<text_line> &Output ) const {

	if( m_input.vehicle == nullptr ) { return; }
	if( m_input.mover == nullptr ) { return; }

	auto const &vehicle{ *m_input.vehicle };
	auto const &mover{ *m_input.mover };

	// induction motor data
	if( mover.EngineType == TEngineType::ElectricInductionMotor ) {

		Output.emplace_back( "      eimc:            eimv:            press:", Global.UITextColor );
		for( int i = 0; i <= 20; ++i ) {

			std::string parameters =
			    TMoverParameters::eimc_labels[ i ] + to_string( mover.eimc[ i ], 2, 9 )
			    + " | "
			    + TMoverParameters::eimv_labels[ i ] + to_string( mover.eimv[ i ], 2, 9 );

			if( i < 10 ) {
				parameters +=
                    m_input.train != nullptr && m_input.train->Dynamic() == m_input.vehicle ? " | " + TTrain::fPress_labels[i] + to_string(m_input.train->fPress[i][0], 2, 9) : "";
			}
			else if( i == 12 ) {
				parameters += "        med:";
			}
			else if( i >= 13 ) {
				parameters += " | " + TDynamicObject::MED_labels[ i - 13 ] + to_string( vehicle.MED[ 0 ][ i - 13 ], 2, 9 );
			}

            Output.emplace_back( parameters, Global.UITextColor );
        }
        Output.emplace_back( "Inverter:\n frequency: " + to_string( mover.InverterFrequency, 2 ), Global.UITextColor );
    }
    // diesel engine data
    if( mover.EngineType == TEngineType::DieselEngine ) {

		double fuel_mean = 0.0;
		if (mover.DistCounter > 0.5) {
			fuel_mean = 100 * mover.dizel_FuelConsumptedTotal / mover.DistCounter;
		}

        std::string parameterstext = "param       value";
        std::vector< std::pair <std::string, double> > const paramvalues {
			{ "  rpm: ", mover.enrot * 60.0 },
			{ "efill: ", mover.dizel_fill },
            { "etorq: ", mover.dizel_Torque },
			{ "power: ", mover.dizel_Power },
			{ "f_act: ", mover.dizel_FuelConsumptionActual },
			{ "f_tot: ", mover.dizel_FuelConsumptedTotal },
			{ "fmean: ", fuel_mean },
            { "creal: ", mover.dizel_engage },
            { "cdesi: ", mover.dizel_engagestate },
            { "cdelt: ", mover.dizel_engagedeltaomega },
            { "gears: ", mover.dizel_automaticgearstatus} };
        for( auto const &[parametername, parametervalue] : paramvalues ) {
            parameterstext += "\n" + parametername + to_string( parametervalue, 2, 9 );
        }
        Output.emplace_back( parameterstext, Global.UITextColor );

        parameterstext = "hydro      value";
        std::vector< std::pair <std::string, double> > const hydrovalues {
            { "hTCnI: ", mover.hydro_TC_nIn },
            { "hTCnO: ", mover.hydro_TC_nOut },
            { "hTCTM: ", mover.hydro_TC_TMRatio },
            { "hTCTI: ", mover.hydro_TC_TorqueIn },
            { "hTCTO: ", mover.hydro_TC_TorqueOut },
            { "hTCfl: ", mover.hydro_TC_Fill },
            { "hRtFl: ", mover.hydro_R_Fill } ,
			{ " hRtn: ", mover.hydro_R_n } ,
			{ "hRtTq: ", mover.hydro_R_Torque }

		};
		for( auto const &[parametername, parametervalue] : hydrovalues ) {
			parameterstext += "\n" + parametername + to_string( parametervalue, 2, 9 );
		}
		Output.emplace_back( parameterstext, Global.UITextColor );
	}
}

void
debug_panel::update_section_ai( std::vector<text_line> &Output ) const {

    if( m_input.mover == nullptr )    { return; }
    if( m_input.mechanik == nullptr ) { return; }

    auto const &mover{ *m_input.mover };
    auto const &mechanik{ *m_input.mechanik };

    // biezaca komenda dla AI
    auto textline =
        "Current order: [" + std::to_string( mechanik.OrderPos ) + "] "
        + mechanik.Order2Str( mechanik.OrderCurrentGet() );

	if( mechanik.fStopTime < 0.0 ) {
		textline += "\n stop time: " + to_string( std::abs( mechanik.fStopTime ), 1 );
	}
	if( mechanik.fActionTime < 0.0 ) {
		textline += "\n action time: " + to_string( std::abs( mechanik.fActionTime ), 1 );
	}

    Output.emplace_back( textline, Global.UITextColor );

    if( mechanik.VelNext == 0.0
     && mechanik.eSignNext ) {
        // jeśli ma zapamiętany event semafora, nazwa eventu semafora
        Output.emplace_back( "Current signal: " + data_name_to_utf8( mechanik.eSignNext->m_name ), Global.UITextColor );
    }

    // distances
    textline =
        "Distances:\n proximity: " + to_string( mechanik.ActualProximityDist, 0 )
        + ", braking: " + to_string( mechanik.fBrakeDist, 0 );
    if( mechanik.VelLimitLastDist.second > 0 ) {
        textline += ", last limit: " + (
            mechanik.VelLimitLastDist.second < EU07_AI_SPEEDLIMITEXTENDSBEYONDSCANRANGE ?
                to_string( mechanik.VelLimitLastDist.second, 0 ) :
                "???" );
    }
    if( mechanik.SwitchClearDist > 0 ) {
        textline += ", switches: " + to_string( mechanik.SwitchClearDist, 0 );
    }

    if( mechanik.Obstacle.distance < 5000 ) {
        textline +=
            "\n obstacle: " + to_string( mechanik.Obstacle.distance, 0 )
            + " (" + mechanik.Obstacle.vehicle->asName + ")";
    }

    Output.emplace_back( textline, Global.UITextColor );

    // velocity factors
    textline =
        "Velocity:\n desired: " + to_string( mechanik.VelDesired, 0 )
        + ", next: " + to_string( mechanik.VelNext, 0 )
        + "\n current: " + to_string( mover.Vel + 0.001f, 2 );

    std::vector< std::pair< double, std::string > > const restrictions{
        { mechanik.VelSignalLast, "signal" },
        { mechanik.VelLimitLast, "limit" },
        { mechanik.VelRoad, "road" },
        { mechanik.VelRestricted, "restricted" },
        { mover.RunningTrack.Velmax, "track" } };

    std::string restrictionstext;
    for( auto const &[restrictionspeed, restrictionsource] : restrictions ) {
        if( restrictionspeed < 0.0 ) { continue; }
        if( false == restrictionstext.empty() ) {
            restrictionstext += ", ";
        }
        restrictionstext +=
            to_string( restrictionspeed, 0 )
            + " (" + restrictionsource + ")";
    }

    if( false == restrictionstext.empty() ) {
        textline += "\n restrictions: " + restrictionstext;
    }

    Output.emplace_back( textline, Global.UITextColor );

    // acceleration
    textline =
        "Acceleration:\n desired: " + to_string( mechanik.AccDesired, 2 )
        + ", corrected: " + to_string( mechanik.AccDesired * mechanik.BrakeAccFactor(), 2 )
        + "\n current: " + to_string( mechanik.AbsAccS + 0.001f, 2 )
        + ", slope: " + to_string( mechanik.fAccGravity + 0.001f, 2 ) + " (" + ( mechanik.fAccGravity > 0.01 ? "\\" : mechanik.fAccGravity < -0.01 ? "/" :
	                                           "-" ) + ")"
		+ "\n desired diesel percentage: " + std::to_string(mechanik.DizelPercentage)
	    + "/" + std::to_string(mechanik.DizelPercentage_Speed)
		+ "/" + to_string(100.4*mechanik.mvControlling->eimic_real, 0);

	Output.emplace_back( textline, Global.UITextColor );

    // brakes
    textline =
        "Brakes:\n highest pressure: " + to_string( mechanik.fReady, 2 ) + ( mechanik.Ready ? " (all brakes released)" : "" )
        + "\n activation threshold: " + to_string( mechanik.fAccThreshold, 2 )
        + ", delays: " + to_string( mechanik.fBrake_a0[ 0 ], 2 ) + " + " + to_string( mechanik.fBrake_a1[ 0 ], 2 )
        + "\n virtual brake position: " + to_string( mechanik.BrakeCtrlPosition, 2 )
        + "\n test stage: " + std::to_string( mechanik.DynamicBrakeTest )
        + ", applied at: " + to_string( mechanik.DBT_VelocityBrake, 0 )
        + ", release at: " + to_string( mechanik.DBT_VelocityRelease, 0 )
        + ", done at: " + to_string( mechanik.DBT_VelocityFinish, 0 );

	Output.emplace_back( textline, Global.UITextColor );

    // ai driving flags
    std::vector<std::string> const drivingflagnames {
        "StopCloser", "StopPoint", "Active", "Press", "Connect", "Primary", "Late", "StopHere",
        "StartHorn", "StartHornNow", "StartHornDone", "Oerlikons", "IncSpeed", "TrackEnd", "SwitchFound", "GuardSignal",
        "Visibility", "DoorOpened", "PushPull", "SignalFound", "StopPointFound", "GuardOpenDoor", "DepartureWarned"
    	/*"SemaphorWasElapsed", "TrainInsideStation", "SpeedLimitFound"*/ };

	textline = "Driving flags:";
	for( int idx = 0, flagbit = 1; idx < drivingflagnames.size(); ++idx, flagbit <<= 1 ) {
		if( mechanik.DrivigFlags() & flagbit ) {
			textline += "\n " + drivingflagnames[ idx ];
		}
	}

	Output.emplace_back( textline, Global.UITextColor );
}

void
debug_panel::update_section_scantable( std::vector<text_line> &Output ) const {

	if( m_input.mechanik == nullptr ) { return; }

	Output.emplace_back( "Flags:       Dist:    Vel:  Name:", Global.UITextColor );

	auto const &mechanik{ *m_input.mechanik };

	std::size_t i = 0; std::size_t const speedtablesize = std::clamp( static_cast<int>( mechanik.TableSize() ) - 1, 0, 30 );
	do {
		auto const scanline = mechanik.TableText( i );
		if( scanline.empty() ) { break; }
		Output.emplace_back( data_name_to_utf8( scanline ), Global.UITextColor );
		++i;
	} while( i < speedtablesize );
	if( Output.size() == 1 ) {
		Output.front().data = "(no points of interest)";
	}
}

#ifdef WITH_UART
void
debug_panel::update_section_uart( std::vector<text_line> &Output ) const {
    uart_status const *status = &UartStatus;

    Output.emplace_back(
        ("Port: " + status->port_name).c_str(),
        Global.UITextColor
    );
    Output.emplace_back(
        ("Baud: " + std::to_string(status->baud)).c_str(),
        Global.UITextColor
    );
    if(status->is_connected) {
        std::string synctext = status->is_synced ? "SYNCED" : "NOT SYNCED";
        Output.emplace_back(("CONNECTED, " + synctext).c_str(), Global.UITextColor);
    } else {
        Output.emplace_back("* NOT CONNECTED *", Global.UITextColor);
    }
    Output.emplace_back(
        (
            "Packets sent: "+std::to_string(status->packets_sent)
            +" Packets received: "+std::to_string(status->packets_received)
        ).c_str(),
        Global.UITextColor
    );
}
#endif

void
debug_panel::update_section_scenario( std::vector<text_line> &Output ) const {

    auto textline =
        "vehicles: " + to_string( Timer::subsystem.sim_dynamics.average(), 2 ) + " msec"
        + " update total: " + to_string( Timer::subsystem.sim_total.average(), 2 ) + " msec";

    Output.emplace_back( textline, Global.UITextColor );
    // current luminance level
	textline = "Light level: " + to_string( Global.fLuminance, 3 ) + ( Global.FakeLight ? "(*)" : "" )+ to_string(Global.SunAngle,2);
    textline +=
        "\nWind: azimuth "
        + to_string( simulation::Environment.wind_azimuth(), 0 ) // ma być azymut, czyli 0 na północy i rośnie na wschód
        + " "
        + std::string( "N NEE SES SWW NW" )
        .substr( static_cast<std::size_t>( 2 * std::floor( std::fmod( 8 + ( glm::radians( simulation::Environment.wind_azimuth() ) + 0.5 * M_PI_4 ) / M_PI_4, 8 ) ) ), 2 )
        + ", " + to_string( glm::length( simulation::Environment.wind() ), 1 ) + " m/s";
    textline += "\nAir temperature: " + to_string( Global.AirTemperature, 1 ) + " deg C";

    Output.emplace_back( textline, Global.UITextColor );
}

void
debug_panel::update_section_eventqueue( std::vector<text_line> &Output ) {

	std::string textline;

    // current event queue
    auto const time { Timer::GetTime() };
    auto const *queuedevent { simulation::Events.begin() };
    auto const searchfilter { std::string( m_eventsearch.data() ) };

	Output.emplace_back( "Delay:   Event:", Global.UITextColor );

	while( queuedevent != nullptr
	    && Output.size() < 30 ) {

		if( false == queuedevent->m_ignored
		 && false == queuedevent->m_passive
		 && ( false == m_eventqueueactivevehicleonly
		   || queuedevent->m_activator == m_input.vehicle ) ) {

            auto const label { queuedevent->m_name + ( queuedevent->m_activator ? " (by: " + queuedevent->m_activator->asName + ")" : "" ) };

            if( false == searchfilter.empty()
             && false == contains(label, searchfilter) ) {
                queuedevent = queuedevent->m_next;
                continue;
            }

            auto const delay { "   " + to_string( std::max( 0.0, queuedevent->m_launchtime - time ), 1 ) };
            textline =
                delay.substr( delay.length() - 6 )
                + "   "
                + label + ( queuedevent->m_sibling ? " (joint event)" : "" );

            Output.emplace_back( textline, Global.UITextColor );
        }
        queuedevent = queuedevent->m_next;
    }
    if( Output.size() == 1 ) {
        // event queue can be empty either because no event got through active filters, or because it is genuinely empty
        Output.front().data = simulation::Events.begin() == nullptr ? "(no queued events)" : "(no matching events)";
    }
}

void
debug_panel::update_section_powergrid( std::vector<text_line> &Output ) const {

	auto const lowpowercolor { glm::vec4( 164.0f / 255.0f, 132.0f / 255.0f, 84.0f / 255.0f, 1.f ) };
	auto const nopowercolor { glm::vec4( 164.0f / 255.0f, 84.0f / 255.0f, 84.0f / 255.0f, 1.f ) };

    Output.emplace_back( "Name:               Output: Current: Timeout:", Global.UITextColor );

	std::string textline;

	for( auto const *powerstation : simulation::Powergrid.sequence() ) {

        if( true == powerstation->IsAutogenerated ) { continue; }
        if( true == powerstation->bSection )        { continue; }

		auto const name { (
			powerstation->m_name.empty() ?
			    "(unnamed)" :
			    powerstation->m_name )
			+ "                              " };

        textline =
            name.substr( 0, 20 )
            + " " + to_string( powerstation->OutputVoltage, 0, 5 )
            + " " + to_string( powerstation->TotalCurrent, 1, 8 )
            + " " + to_string( powerstation->FuseTimer, 1, 8 )
            + ( powerstation->FuseCounter == 0 ?
                "" :
                " (x" + std::to_string( powerstation->FuseCounter ) + ")" );

		Output.emplace_back(
		    textline,
		    powerstation->FastFuse || powerstation->SlowFuse                 ? nopowercolor :
		                              powerstation->OutputVoltage < 0.8 * powerstation->NominalVoltage ? lowpowercolor :
		                                                                                                 Global.UITextColor );
	}

	if( Output.size() == 1 ) {
		Output.front().data = "(no power stations)";
	}
}

void
debug_panel::update_section_camera( std::vector<text_line> &Output ) const {

	if( m_input.camera == nullptr ) { return; }

	auto const &camera{ *m_input.camera };

	// camera data
	auto textline =
	    "Position: ["
	    + to_string( camera.Pos.x, 2 ) + ", "
	    + to_string( camera.Pos.y, 2 ) + ", "
	    + to_string( camera.Pos.z, 2 ) + "]";

	Output.emplace_back( textline, Global.UITextColor );

	textline =
	    "Azimuth: "
	    + to_string( 180.0 - glm::degrees( camera.Angle.y ), 0 ) // ma być azymut, czyli 0 na północy i rośnie na wschód
	    + " "
	    + std::string( "S SEE NEN NWW SW" )
	    .substr( static_cast<std::size_t>( 2 * floor( fmod( 8 + ( camera.Angle.y + 0.5 * M_PI_4 ) / M_PI_4, 8 ) ) ), 2 );

	Output.emplace_back( textline, Global.UITextColor );
}

void
debug_panel::update_section_renderer( std::vector<text_line> &Output ) const {

            // gfx renderer data
            auto textline =
                "FoV: " + to_string( Global.FieldOfView / Global.ZoomFactor, 1 )
                + ", Draw range: " + to_string( Global.BaseDrawRange * Global.fDistanceFactor, 0 ) + "m"
//                + "; sectors: " + std::to_string( GfxRenderer->m_drawcount )
//                + ", FPS: " + to_string( Timer::GetFPS(), 2 );
                + ", FPS: " + std::to_string( static_cast<int>(std::round(GfxRenderer->Framerate())) )
                + ( Global.VSync ? " (vsync on)" : "" );
            if( Global.iSlowMotion ) {
                textline += " (slowmotion " + std::to_string( Global.iSlowMotion ) + ")";
            }

            Output.emplace_back( textline, Global.UITextColor );

            textline += "\nRendering mode: ";

	        if (Global.GfxRenderer == "default")
	        {
		        textline += "Shaders";
	        }
            else if (Global.GfxRenderer == "experimental")
            {
		        textline += "NVRHI on ";
            }
	        else
	        {
		        if (Global.BasicRenderer)
		        {
			        textline += "Legacy Simple";
		        }
		        else
		        {
			        textline += "Legacy";
		        }
	        }

	        if (Global.bUseVBO)
	        {
		        textline += ", VBO";
	        }
	        else
	        {
		        textline += ", Display Lists";
	        }

	        textline += " ";

            if( false == Global.LastGLError.empty() ) {
                textline +=
                    "Last openGL error: "
                    + Global.LastGLError;
            }

			Output.emplace_back( textline, Global.UITextColor );

            // renderer stats
            Output.emplace_back( GfxRenderer->info_times(), Global.UITextColor );
            Output.emplace_back( GfxRenderer->info_stats(), Global.UITextColor );

            // CPU related
            Output.emplace_back("CPU:", Global.UITextColor);

            // thread counter
	        textline = "Running threads: " + std::to_string(Global.threads.size() + 1);
	        Output.emplace_back(textline, Global.UITextColor);

}

namespace {

// how far from the camera a coupler shows in the coupler section [m]
constexpr float coupler_pick_radius { 10.f };

std::string_view coupler_type_name( TCouplerType const Type ) {
    switch( Type ) {
        case TCouplerType::Articulated: { return "articulated"; }
        case TCouplerType::Bare: { return "bare"; }
        case TCouplerType::Chain: { return "chain"; }
        case TCouplerType::Screw: { return "screw"; }
        case TCouplerType::Automatic: { return "automatic"; }
        default: { return "none"; }
    }
}

// the couplerphysics setting of eu07.ini
std::string_view coupler_physics_setting() {
    if( Global.coupler_physics_all ) { return "all"; }
    return Global.coupler_physics ? "yes" : "no";
}

std::string_view coupler_stop_name( int const Stop ) {
    if( Stop > 0 ) { return "draw gear solid"; }
    if( Stop < 0 ) { return "buffers solid"; }
    return "none";
}

// a value shown in the Scale of its unit (1000 for kN, 0.001 for mm), dragged within Min and Max of that unit
bool edit_value( char const *Label, double &Value, double const Scale, double const Min, double const Max, char const *Format ) {
    auto shown { Value / Scale };
    if( false == ImGui::DragScalar( Label, ImGuiDataType_Double, &shown, static_cast<float>( ( Max - Min ) / 1000.0 ), &Min, &Max, Format ) ) {
        return false;
    }
    Value = std::clamp( shown, Min, Max ) * Scale;
    return true;
}

bool edit_spring( char const *Name, coupler_physics::spring_curve &Curve, double &Unloading ) {
    ImGui::PushID( Name );
    ImGui::TextUnformatted( Name );
    auto changed { edit_value( "Stiffness [kN/m]", Curve.stiffness, 1000.0, 100.0, 50000.0, "%.0f" ) };
    changed = edit_value( "Travel [mm]", Curve.travel, 0.001, 5.0, 300.0, "%.0f" ) || changed;
    changed = edit_value( "Progression", Curve.progression, 1.0, 0.0, 0.9, "%.2f" ) || changed;
    changed = edit_value( "Unloading ratio", Unloading, 1.0, 0.05, 1.0, "%.2f" ) || changed;
    ImGui::Text( "End force: %.0f kN", Curve.end_force() / 1000.0 );
    ImGui::PopID();
    return changed;
}

bool edit_link( coupler_physics::end_parameters &Parameters ) {
    auto &link { Parameters.link };
    ImGui::PushID( "link" );
    ImGui::TextUnformatted( "Link" );
    auto changed { ImGui::Checkbox( "Breakable", &link.breakable ) };
    changed = edit_value( "Ultimate force [kN]", link.ultimate, 1000.0, 10.0, 5000.0, "%.0f" ) || changed;
    changed = edit_value( "Yield force [kN]", link.yield, 1000.0, 10.0, 5000.0, "%.0f" ) || changed;
    link.yield = std::min( link.yield, link.ultimate );
    // the permanent stretch at which the link breaks, which is what the hardening gives
    auto stretch { ( link.ultimate - link.yield ) / link.hardening };
    if( edit_value( "Stretch to break [mm]", stretch, 0.001, 1.0, 200.0, "%.1f" ) ) {
        link.hardening = std::max( ( link.ultimate - link.yield ) / stretch, 1.0 );
        changed = true;
    }
    changed = edit_value( "Link stiffness [kN/m]", Parameters.link_stiffness, 1000.0, 1000.0, 2000000.0, "%.0f" ) || changed;
    changed = edit_value( "Buffers crush force [kN]", Parameters.crush_force, 1000.0, 100.0, 10000.0, "%.0f" ) || changed;
    changed = edit_value( "Restitution of a solid hit", Parameters.solid_restitution, 1.0, 0.05, 0.95, "%.2f" ) || changed;
    ImGui::PopID();
    return changed;
}

bool edit_coupler_parameters( coupler_physics::end_parameters &Parameters ) {
    auto changed { edit_value( "Slack [mm]", Parameters.slack, 0.001, -100.0, 300.0, "%.1f" ) };
    changed = edit_value( "Damping [kN s/m]", Parameters.damping, 1000.0, 0.0, 1000.0, "%.1f" ) || changed;
    changed = edit_spring( "Buffers", Parameters.buffers, Parameters.buffers_unloading ) || changed;
    changed = edit_spring( "Draw gear", Parameters.draw_gear, Parameters.draw_gear_unloading ) || changed;
    changed = edit_link( Parameters ) || changed;
    return changed;
}

} // namespace

void
debug_panel::update_section_coupler( std::vector<text_line> &Output ) {

    m_coupler = {};
    if( m_input.camera == nullptr ) { return; }
    auto const &camera { m_input.camera->Pos };
    auto const *vehicle { std::get<TDynamicObject *>( simulation::Region->find_vehicle( camera, coupler_pick_radius, false, true ) ) };
    if( vehicle == nullptr || vehicle->MoverParameters == nullptr ) {
        Output.emplace_back( std::format( "No coupler within {:.0f} m of the camera", coupler_pick_radius ), Global.UITextColor );
        return;
    }
    auto const nearerfront { glm::distance( vehicle->CouplerPosition( end::front ), camera ) <= glm::distance( vehicle->CouplerPosition( end::rear ), camera ) };
    m_coupler = { vehicle, nearerfront ? end::front : end::rear };

    auto const &mover { *vehicle->MoverParameters };
    auto const &coupler { mover.Couplers[ m_coupler.end ] };
    auto const &neighbour { mover.Neighbours[ m_coupler.end ] };
    auto const &state { coupler.physics_state };

    Output.emplace_back(
        std::format( "{}, {} end, {} coupler{}", vehicle->name(), nearerfront ? "front" : "rear", coupler_type_name( coupler.type() ), coupler.has_adapter() ? " (adapter)" : "" ),
        Global.UITextColor );
    Output.emplace_back(
        std::format(
            "Model: {} (couplerphysics {}, vehicle {}covered), crash damage {}",
            mover.coupler_physics_at( m_coupler.end ) ? "physical" : "legacy",
            coupler_physics_setting(),
            mover.CouplerPhysics ? "" : "not ",
            Global.crash_damage ? "on" : "off" ),
        Global.UITextColor );
    if( neighbour.vehicle != nullptr ) {
        Output.emplace_back(
            std::format( "Neighbour: {} ({} end), coupling [{}], gap {:.1f} mm", neighbour.vehicle->name(), neighbour.vehicle_end == end::front ? "front" : "rear", coupler.CouplingFlag, neighbour.distance * 1000.0 ),
            Global.UITextColor );
    }
    else {
        Output.emplace_back( "Neighbour: none", Global.UITextColor );
    }
    Output.emplace_back( std::format( "Force: {:.1f} kN", coupler.CForce / 1000.0 ), Global.UITextColor );
    Output.emplace_back(
        std::format( "Draw gear: {:.1f} mm, {:.1f} kN  Buffers: {:.1f} mm, {:.1f} kN", state.draw_gear.deflection * 1000.0, state.draw_gear.force / 1000.0, state.buffers.deflection * 1000.0, state.buffers.force / 1000.0 ),
        Global.UITextColor );
    Output.emplace_back(
        std::format( "Stretch: {:.1f} mm, stop: {}, overloaded for {:.1f} s", state.elongation * 1000.0, coupler_stop_name( state.stop ), coupler.stretch_duration ),
        Global.UITextColor );
    if( coupler.broken ) {
        Output.emplace_back( "Coupler broken (repair it in the driving aid, Shift+F1)", glm::vec4( 1.f, 0.25f, 0.25f, 1.f ) );
    }
}

bool
debug_panel::render_section_coupler() {

    if( false == render_section( "Coupler", m_couplerlines ) ) { return false; }
    // the parameters to try out, in the debug mode, on the simulation itself
    if( false == DebugModeFlag || m_coupler.vehicle == nullptr || Application.is_client() ) { return true; }

    ImGui::Separator();
    ImGui::Checkbox( "Physical coupler model", &Global.coupler_physics );
    ImGui::Checkbox( "Physical model for every train", &Global.coupler_physics_all );
    ImGui::Checkbox( "Crash damage", &Global.crash_damage );
    ImGui::Checkbox( "Apply to the other end of the pair too", &m_couplerbothends );

    auto &mover { *m_coupler.vehicle->MoverParameters };
    auto &coupler { mover.Couplers[ m_coupler.end ] };
    auto const &neighbour { mover.Neighbours[ m_coupler.end ] };
    auto *othercoupler { neighbour.vehicle != nullptr ? &neighbour.vehicle->MoverParameters->Couplers[ neighbour.vehicle_end ] : nullptr };

    if( auto parameters { coupler.physics }; edit_coupler_parameters( parameters ) ) {
        coupler.physics = parameters;
        if( m_couplerbothends && othercoupler != nullptr ) {
            othercoupler->physics = parameters;
        }
    }
    if( ImGui::Button( "Reset the stretch of the coupling" ) ) {
        // both vehicles keep the state of the pair alike
        coupler.physics_state.elongation = 0.0;
        if( othercoupler != nullptr ) {
            othercoupler->physics_state.elongation = 0.0;
        }
    }
    return true;
}

bool
debug_panel::render_section( std::string const &Header, std::vector<text_line> const &Lines ) {

	if( true == Lines.empty() ) { return false; }
	if( false == ImGui::CollapsingHeader( Header.c_str() ) ) { return false; }

    return render_section( Lines );
}

bool
debug_panel::render_section( std::vector<text_line> const &Lines ) const {

    for( auto const &line : Lines ) {
        ImGui::PushStyleColor( ImGuiCol_Text, { line.color.r, line.color.g, line.color.b, line.color.a } );
        ImGui::TextUnformatted( line.data.c_str() );
        ImGui::PopStyleColor();
//        ImGui::TextColored( ImVec4( line.color.r, line.color.g, line.color.b, line.color.a ), line.data.c_str() );
	}
	return true;
}

bool debug_panel::render_section_developer() const
{
	if (false == ImGui::CollapsingHeader("Developer tools"))
		return false;
	ImGui::PushStyleColor(ImGuiCol_Text, {Global.UITextColor.r, Global.UITextColor.g, Global.UITextColor.b, Global.UITextColor.a});
	ImGui::TextUnformatted("Warning! These tools are only for developers.\nDo not use them if you are NOT sure what they do!");
	ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "These settings may crash your simulator!");
	if (ImGui::Button("Reload current vehicle .fiz") == true)
	{
		m_input.vehicle->MoverParameters->reload_FIZ(); // reload fiz
	}
	ImGui::PopStyleColor();
	return true;
}

namespace {

// a combo choosing one of the names, the first entry standing for the library's own choice. the names are asked for
// each time the list opens, as devices come and go. returns: true if the choice changed
template <class Names_>
bool name_combo( char const *Label, std::string &Value, Names_ const &Names, char const *Defaultlabel, std::string_view const Defaultvalue ) {

    auto const preview { Value == Defaultvalue ? std::string{ Defaultlabel } : Value };
    if( false == ImGui::BeginCombo( Label, preview.c_str() ) ) { return false; }

    static std::vector<std::string> names; // one list per call site, as each passes its own lambda
    if( ImGui::IsWindowAppearing() ) {
        names = Names();
    }
    auto changed { false };
    if( ImGui::Selectable( Defaultlabel, Value == Defaultvalue ) ) {
        Value = Defaultvalue;
        changed = true;
    }
    for( auto const &name : names ) {
        if( ImGui::Selectable( name.c_str(), name == Value ) ) {
            Value = name;
            changed = true;
        }
    }
    ImGui::EndCombo();
    return changed;
}

// options of the audio device, applied by reopening it: output device, output mode, HRTF, limiter, mixing rate, sources
void render_audio_device_settings() {

    auto changed { false };
    changed |= name_combo( "Output device###audiodevice", Global.AudioRenderer,
        [] { return audio::renderer.output_devices(); }, "System default", "" );
    changed |= name_combo( "Output mode###audiooutputmode", Global.AudioOutputMode,
        [] { return std::vector<std::string>{ "mono", "stereo", "stereo-basic", "uhj", "headphones", "quad", "5.1", "6.1", "7.1" }; },
        "Automatic", "auto" );
    changed |= name_combo( "HRTF###audiohrtf", Global.AudioHrtf,
        [] { return std::vector<std::string>{ "on", "off" }; }, "Automatic", "auto" );
    changed |= name_combo( "HRTF data set###audiohrtfprofile", Global.AudioHrtfProfile,
        [] { return audio::renderer.hrtf_profiles(); }, "Library's choice", "" );
    changed |= name_combo( "Limiter###audiolimiter", Global.AudioLimiter,
        [] { return std::vector<std::string>{ "on", "off" }; }, "Automatic", "auto" );

    auto frequency { Global.AudioFrequency > 0 ? std::to_string( Global.AudioFrequency ) : std::string{} };
    if( name_combo( "Mixing rate###audiofrequency", frequency,
            [] { return std::vector<std::string>{ "22050", "32000", "44100", "48000", "96000" }; }, "Device's", "" ) ) {
        Global.AudioFrequency = ( frequency.empty() ? 0 : std::stoi( frequency ) );
        changed = true;
    }

    if( changed ) {
        audio::renderer.apply_device_settings();
    }
}

// options of the sources, applied to the playing ones right away: resampler, Doppler effect, EFX
void render_audio_source_settings() {

    auto changed { false };
    changed |= name_combo( "Resampler###audioresampler", Global.AudioResampler,
        [] { return audio::renderer.resamplers(); }, "Library's default", "" );
    changed |= ImGui::SliderFloat( ( to_string( Global.AudioDoppler, 2 ) + "###audiodoppler" ).c_str(), &Global.AudioDoppler, 0.0f, 1.0f, "Doppler effect" );
    changed |= ImGui::Checkbox( "Soundproofing muffles high frequencies###audiolowpass", &Global.AudioLowpass );
    changed |= ImGui::SliderFloat( ( to_string( Global.AudioAirAbsorption, 1 ) + "###audioairabsorption" ).c_str(), &Global.AudioAirAbsorption, 0.0f, 10.0f, "Air absorption" );

    if( changed ) {
        audio::renderer.apply_source_settings();
    }
    ImGui::TextWrapped( "%s", audio::renderer.device_status().c_str() );
}

} // namespace

bool
debug_panel::render_section_settings() const {

    if( false == ImGui::CollapsingHeader( "Settings" ) ) { return false; }

    ImGui::PushStyleColor( ImGuiCol_Text, { Global.UITextColor.r, Global.UITextColor.g, Global.UITextColor.b, Global.UITextColor.a } );
    ImGui::TextUnformatted( "Graphics" );
    ImGui::PopStyleColor();
    // reflection fidelity
    ImGui::SliderInt( ( std::to_string( Global.reflectiontune.fidelity ) + "###reflectionfidelity" ).c_str(), &Global.reflectiontune.fidelity, 0, 2, "Reflection fidelity" );
    ImGui::SliderInt( ( std::to_string( Global.gfx_shadow_rank_cutoff ) + "###shadowrankcutoff" ).c_str(), &Global.gfx_shadow_rank_cutoff, 1, 3, "Shadow ranks" );
    if( ImGui::SliderFloat( ( to_string( std::abs( Global.gfx_shadow_angle_min ), 2 ) + "###shadowanglecutoff" ).c_str(), &Global.gfx_shadow_angle_min, -1.0, -0.2f, "Shadow angle cutoff" ) ) {
        Global.gfx_shadow_angle_min = quantize( Global.gfx_shadow_angle_min, 0.05f );
    }
    if( DebugModeFlag ) {
        // sky sliders
        {
            ImGui::SliderFloat(
                ( to_string( Global.m_skysaturationcorrection, 2, 5 ) + "###skysaturation" ).c_str(), &Global.m_skysaturationcorrection, 0.0f, 3.0f, "Sky saturation" );
        }
        {
            ImGui::SliderFloat(
                ( to_string( Global.m_skyhuecorrection, 2, 5 ) + "###skyhue" ).c_str(), &Global.m_skyhuecorrection, 0.0f, 1.0f, "Sky hue correction" );
        }
    }

    ImGui::PushStyleColor( ImGuiCol_Text, { Global.UITextColor.r, Global.UITextColor.g, Global.UITextColor.b, Global.UITextColor.a } );
    ImGui::TextUnformatted( "Sound" );
    ImGui::PopStyleColor();
    // audio volume sliders
    ImGui::SliderFloat( ( std::to_string( static_cast<int>( Global.AudioVolume * 100 ) ) + "%###volumemain" ).c_str(), &Global.AudioVolume, 0.0f, 2.0f, "Main audio volume" );
    if( ImGui::SliderFloat( ( std::to_string( static_cast<int>( Global.VehicleVolume * 100 ) ) + "%###volumevehicle" ).c_str(), &Global.VehicleVolume, 0.0f, 1.0f, "Vehicle sounds" ) ) {
        audio::event_volume_change = true;
    }
    if( ImGui::SliderFloat( ( std::to_string( static_cast<int>( Global.EnvironmentPositionalVolume * 100 ) ) + "%###volumepositional" ).c_str(), &Global.EnvironmentPositionalVolume, 0.0f, 1.0f, "Positional sounds" ) ) {
        audio::event_volume_change = true;
    }
    if( ImGui::SliderFloat( ( std::to_string( static_cast<int>( Global.EnvironmentAmbientVolume * 100 ) ) + "%###volumeambient" ).c_str(), &Global.EnvironmentAmbientVolume, 0.0f, 1.0f, "Ambient sounds" ) ) {
        audio::event_volume_change = true;
    }
    if (simulation::Train) {
        float val = simulation::Train->get_radiovolume();
        if( ImGui::SliderFloat( ( std::to_string( static_cast<int>( val * 100 ) ) + "%###volumeradio" ).c_str(), &val, 0.0f, 1.0f, "Vehicle radio volume" ) ) {
            command_relay commandrelay;
            commandrelay.post(user_command::radiovolumeset, val, 0.0, GLFW_PRESS, 0);
        }
    }
    if( Global.bSoundEnabled ) {
        render_audio_device_settings();
        render_audio_source_settings();
    }

    return true;
}

void
transcripts_panel::update() {

	if( false == is_open ) { return; }

	text_lines.clear();

	for( auto const &transcript : ui::Transcripts.aLines ) {
		if( Global.fTimeAngleDeg + ( transcript.fShow - Global.fTimeAngleDeg > 180 ? 360 : 0 ) < transcript.fShow ) { continue; }
		std::string text = transcript.asText;
		std::ranges::replace(text, '|', ' ');
		text_lines.emplace_back( text, colors::white );
	}
}

void
transcripts_panel::render() {

    if( false == is_open ) { return; }
    if( true == text_lines.empty() ) { return; }

    auto flags =
        ImGuiWindowFlags_NoFocusOnAppearing
        | ImGuiWindowFlags_NoCollapse
        | ( size.x > 0 ? ImGuiWindowFlags_NoResize : 0 );

    if( size.x > 0 ) {
        ImGui::SetNextWindowSize( ImVec2S( size.x, size.y ) );
    }
    if( size_min.x > 0 ) {
        ImGui::SetNextWindowSizeConstraints( ImVec2S( size_min.x, size_min.y ), ImVec2S( size_max.x, size_max.y ) );
    }
    if( auto const panelname { ( title.empty() ? m_name : title ) + "###" + m_name }; true == ImGui::Begin( panelname.c_str(), &is_open, flags ) ) {
        // header section
        for( auto const &line : text_lines ) {
            ImGui::TextWrapped( "%s", line.data.c_str() );
        }
    }
    ImGui::End();
}


//---------------------------------------------------------------------------
// hudcfg: HUD layout configuration, stored in the HUD's own config file (hud.ini)
// the parameters are read by the common config parser into Global.gui_hud
//---------------------------------------------------------------------------

namespace hudcfg {

// vertical step of the bottom row stack, in 1920x1080 baseline units; the renderer counts rows with
// row_h() below. the panel height does not derive from it - see main_panel_height()
float constexpr ROW_H { 42.0f };

// resolution-relative scale for the whole HUD: contents are laid out at the 1920x1080
// baseline and scaled by the actual window, so 1600x900 shows a proportionally smaller HUD and
// 2560x1440 a larger one. BOTH axes take part and the smaller one wins, so a window that is not
// 16:9 can never push the overlay past an edge. the old floor of 0.7 stopped the HUD from
// shrinking on small windows and made it look oversized there; it is 0.4 now.
// gui.hud.scale (hud.ini) multiplies the result, so the player can size the whole overlay to
// taste on any resolution without a rebuild.
float res_scale()
{
    float const sx { static_cast<float>( Global.fb_size.x ) / 1920.0f };
    float const sy { static_cast<float>( Global.fb_size.y ) / 1080.0f };
    float const res { std::clamp( std::min( sx, sy ), 0.4f, 2.0f ) };
    return res * std::clamp( Global.gui_hud.scale, 0.25f, 4.0f );
}

// effective row height of one bottom row; rows scale with the screen like the rest of the plate
float row_h()
{
    return ROW_H * res_scale();
}

// registry of every displayable HUD datum. stable ids double as the extension point:
// a future developer adds a row here and the item appear in the customisation window.
// group: every item now lives on the single main panel (group 0)
// default_on: false means "hidden by default" - visible only when checked in Custom mode
static constexpr hud_item items[] = {
    // id,          display name,         group, default
    // speed / direction / grade have no separate panel any more: they are part of the main
    // panel, so they belong to its group
    { "speed",     "Speed",                  0, true },
    { "direction", "Direction",              0, true },
    { "grade",     "Grade",                  0, true },
    { "voltage",   "Voltage",                0, true },  // outermost arc (electric vehicles only)
    { "power",     "Power bar",              0, true },  // diesel rpm / traction current: ring only, no bottom row
    { "limit",     "Current limit",          0, true },  // the blue limit box
    { "nextlimit", "Next limit",             0, true },  // the orange limit box
    // the distance readout is an item of its own, so the orange box and the "620m" can be switched
    // on and off independently. the display name stays English on purpose: the game has no msgid
    // for this concept and the project does not add translatable strings (that would mean a new
    // lang/*.po entry), so this label is English-only like the other item names the game lacks
    { "nextlimitdist", "Next limit distance", 0, false }, // "620m" to the next limit, off by default
    { "brakepipe", "Brake pipe",             0, true },
    { "mainres",   "Main reservoir",         0, false }, // supply circuit pressure
    { "trainbrake","train brake",            0, true },
    { "edbrake",   "Dynamic brake",          0, true },  // only with a dedicated ED handle
    { "indbrake",  "independent brake",      0, true },
    { "brakecyl",  "Brake cylinder",         0, true },
    { "brakecyl_last", "Last car brake cyl",   0, false }, // long-train tail cylinder
    { "brakecyl_min",  "Worst brake cyl",      0, false }, // whole-consist worst brake
    { "notch",     "master controller",      0, true },  // notch arc + the position marks
    { "shunt",     "second controller",      0, true },  // field-weakening (second controller) arc
    { "zone",      "Notch zone",             0, true },  // series/parallel/shunt zone name, on its own row
    { "speedctrl", "cruise control",         0, false }, // tempomat: selected speed + activity
    { "sigdist",   "Signal distance",        0, true },
    { "passenger", "Passenger exchange",     0, true },
    { "alarm",     "CA / SHP alarm",         0, true },
    { "doors",     "Doors",                  0, false }, // door state line (EMU/DMU)
};
static int const ITEM_COUNT { static_cast<int>( sizeof( items ) / sizeof( items[ 0 ] ) ) };

static ui_panel *g_panel { nullptr };
static ui_panel *g_customwindow { nullptr };
static bool g_visible { true };
static bool g_dragging { false };
static bool g_panelon { true };  // main panel on/off (shared by both modes)
static int g_mode { standard };
static float g_toast { 0.0f };
// custom-mode checkbox state, indexed like items[]
static std::array<bool, ITEM_COUNT> g_custom_on;
static void apply_visibility();
static void sync_custom_items( std::string const &Csv );
static void load_hud_settings();

// Current scheme is already 0=Standard 1=Custom 2=Off.
// Only remap the pre-HUD 4-mode value 3 (= Off). Mapping 0/1/2 would break persistence
// (saved Off=2 was wrongly restored as Custom).
static int normalize_hud_mode( int const Old )
{
    if ( Old == 3 ) return off;          // legacy 4-mode Off
    return std::clamp<int>( Old, standard, off );
}

static int item_index( char const *Id )
{
    for ( int i = 0; i < ITEM_COUNT; ++i )
        if ( std::string_view( items[ i ].id ) == Id )
            return i;
    return -1;
}

int mode()
{
    return std::clamp<int>( g_mode, standard, off );
}


// Write HUD switch/layout keys back into hud.ini (update existing lines or append).
// Called from every setter so toggles survive the next launch without relying on a
// full-config export path (the app does not rewrite eu07.ini on exit).
static void save_hud_settings()
{
    namespace fs = std::filesystem;
    // same resolution as the other ini files (eu07.ini, eu07_input-keyboard.ini): the player's copy
    // in the per-user config directory wins, the one next to the simulator is the fallback. a bare
    // relative name would follow the working directory, which is not the game directory when the
    // simulator is started from a shortcut or a launcher
    fs::path iniPath { user_config_path( "hud.ini" ) };
    if ( iniPath.empty() )
        iniPath = "hud.ini";

    // collect the keys we own
    std::vector<std::pair<std::string, std::string>> keys;
    keys.emplace_back( "gui.hud.enabled", Global.gui_hud.enabled ? "yes" : "no" );
    // written only once the player has actually chosen a mode: a file without this line leaves
    // mode_saved false on load, so a fresh install keeps starting with the HUD off instead of
    // recording the compiled default as a choice nobody made
    if ( Global.gui_hud.mode_saved )
        keys.emplace_back( "gui.hud.mode", std::to_string( Global.gui_hud.mode ) );
    keys.emplace_back( "gui.hud.panel", Global.gui_hud.panel ? "yes" : "no" );
    keys.emplace_back( "gui.hud.custom", Global.gui_hud.custom_items );
    // compact position lines (name is part of the value so one key token works for match)
    {
        std::ostringstream o;
        o << "main " << Global.gui_hud.panel_x << "," << Global.gui_hud.panel_y << ",0";
        keys.emplace_back( "gui.hud.pos", o.str() );
    }
    // instrument layout (arc skin): one compact line per group so hud.ini stays readable
    {
        std::ostringstream o; o << Global.gui_hud.main_w << "," << Global.gui_hud.main_h;
        keys.emplace_back( "gui.hud.panel_size", o.str() );
    }
    {
        char sb[ 32 ] {};
        std::snprintf( sb, sizeof( sb ), "%g", Global.gui_hud.scale );
        keys.emplace_back( "gui.hud.scale", sb );
    }
    {
        // arc geometry, no band here: the band width is per ring now (see gui.hud.ring.*)
        std::ostringstream o; o << Global.gui_hud.arc_cx << "," << Global.gui_hud.arc_cy << ","
                                << Global.gui_hud.arc_a0 << "," << Global.gui_hud.arc_a1;
        keys.emplace_back( "gui.hud.arc", o.str() );
    }
    // one line per ring, field names identical to the HTML layout page - so a value can be
    // copied straight from the page into hud.ini (and back) without translation
    {
        char const *const ringnames[ 4 ] { "voltage", "current", "notch", "shunt" };
        for ( int i = 0; i < 4; ++i )
        {
            char hcol[ 8 ], hname[ 8 ], hval[ 8 ];
            std::snprintf( hcol, sizeof( hcol ), "%06x", Global.gui_hud.arc_col[ i ] );
            std::snprintf( hname, sizeof( hname ), "%06x", Global.gui_hud.arc_name_col[ i ] );
            std::snprintf( hval, sizeof( hval ), "%06x", Global.gui_hud.arc_val_col[ i ] );
            std::ostringstream o;
            o << "r=" << Global.gui_hud.arc_r[ i ] << " band=" << Global.gui_hud.arc_band_of[ i ]
              << " col=" << hcol
              << " nameoff=" << Global.gui_hud.arc_name_off[ i ]
              << " namepct=" << Global.gui_hud.arc_name_pct[ i ]
              << " namespace=" << Global.gui_hud.arc_name_space[ i ]
              << " namesize=" << Global.gui_hud.arc_name_size[ i ]
              << " namecol=" << hname
              << " valoff=" << Global.gui_hud.arc_val_off[ i ]
              << " valpct=" << Global.gui_hud.arc_val_pct[ i ]
              << " valspace=" << Global.gui_hud.arc_val_space[ i ]
              << " valsize=" << Global.gui_hud.arc_val_size[ i ]
              << " valcol=" << hval;
            keys.emplace_back( std::string { "gui.hud.ring." } + ringnames[ i ], o.str() );
        }
    }
    {
        // only the notch arrow dimensions here: the ring text is laid along the arc now, so its
        // old free-position/rotation keys are gone
        std::ostringstream o; o << Global.gui_hud.arrow_len << "," << Global.gui_hud.arrow_hw;
        keys.emplace_back( "gui.hud.arc_text", o.str() );
    }
    // NOTE: gui.hud.pos has 3 lines with different first value token; handled specially below
    {
        std::ostringstream o; o << Global.gui_hud.warn_low << "," << Global.gui_hud.warn_mid << ","
                                << Global.gui_hud.warn_high;
        keys.emplace_back( "gui.hud.warn", o.str() );
    }
    {
        std::ostringstream o; o << Global.gui_hud.spd_x << "," << Global.gui_hud.spd_y << ","
                                << Global.gui_hud.speed_size;
        keys.emplace_back( "gui.hud.speed", o.str() );
    }
    {
        std::ostringstream o; o << Global.gui_hud.dir_x << "," << Global.gui_hud.dir_y << ","
                                << Global.gui_hud.dir_hw << "," << Global.gui_hud.dir_hh;
        keys.emplace_back( "gui.hud.dir", o.str() );
    }
    {
        std::ostringstream o; o << Global.gui_hud.limit1_x << "," << Global.gui_hud.limit1_y << ","
                                << Global.gui_hud.limit2_x << "," << Global.gui_hud.limit2_y << ","
                                << Global.gui_hud.limit_w << "," << Global.gui_hud.limit_h << ","
                                << Global.gui_hud.limit_bw << "," << Global.gui_hud.limit_radius << ","
                                << Global.gui_hud.limit_textfrac;
        keys.emplace_back( "gui.hud.limitbox", o.str() );
    }
    {
        std::ostringstream o; o << Global.gui_hud.dist_x << "," << Global.gui_hud.dist_y << ","
                                << Global.gui_hud.dist_size;
        keys.emplace_back( "gui.hud.dist", o.str() );
    }
    {
        std::ostringstream o; o << Global.gui_hud.grade_x << "," << Global.gui_hud.grade_y << ","
                                << Global.gui_hud.grade_w;
        keys.emplace_back( "gui.hud.grade", o.str() );
    }
    {
        std::ostringstream o; o << Global.gui_hud.bars_x << "," << Global.gui_hud.bars_y << ","
                                << Global.gui_hud.bars_w << "," << Global.gui_hud.bars_rh << ","
                                << Global.gui_hud.bars_gap << "," << Global.gui_hud.bars_radius << ","
                                << std::hex << Global.gui_hud.bars_bg << std::dec << ","
                                << Global.gui_hud.bars_bga;
        keys.emplace_back( "gui.hud.bars", o.str() );
    }
    {
        std::ostringstream o; o << Global.gui_hud.speedctrl_x << "," << Global.gui_hud.speedctrl_y << ","
                                << Global.gui_hud.speedctrl_size;
        keys.emplace_back( "gui.hud.speedctrl", o.str() );
    }
    {
        std::ostringstream o; o << Global.gui_hud.zone_x << "," << Global.gui_hud.zone_y << ","
                                << Global.gui_hud.zone_size;
        keys.emplace_back( "gui.hud.zone", o.str() );
    }
    // elements fused in from the top signal strip: one line each, same field names as the page
    {
        char const *const signames[ 5 ] { "dist", "pax", "doors", "alerter", "shp" };
        decltype( Global.gui_hud.sig_dist ) const *const sigs[ 5 ] {
            &Global.gui_hud.sig_dist, &Global.gui_hud.sig_pax, &Global.gui_hud.sig_doors,
            &Global.gui_hud.sig_alerter, &Global.gui_hud.sig_shp };
        for ( int i = 0; i < 5; ++i )
        {
            char hc[ 8 ];
            std::snprintf( hc, sizeof( hc ), "%06x", sigs[ i ]->col );
            std::ostringstream o;
            o << "x=" << sigs[ i ]->x << " y=" << sigs[ i ]->y << " size=" << sigs[ i ]->size
              << " col=" << hc << " show=" << ( sigs[ i ]->show ? "yes" : "no" );
            keys.emplace_back( std::string { "gui.hud.sig." } + signames[ i ], o.str() );
        }
    }
    // NOTE: gui.hud.pos has 3 lines with different first value token; handled specially below

    std::string content;
    {
        std::ifstream in( iniPath );
        if ( in )
            content.assign( std::istreambuf_iterator<char>( in ), std::istreambuf_iterator<char>() );
    }

    auto upsert_line = [ &content ]( std::string const &key, std::string const &value, std::string const &match_prefix = {} )
    {
        // match_prefix: when set, the line must also start with "key match_prefix" (for pos lines)
        std::string needle = key;
        std::string newline = key + " " + value + "\n";
        std::istringstream iss( content );
        std::string out;
        std::string line;
        bool found = false;
        while ( std::getline( iss, line ) )
        {
            // strip CR
            if ( !line.empty() && line.back() == '\r' )
                line.pop_back();
            bool is_match = false;
            if ( line.compare( 0, needle.size(), needle ) == 0
              && ( line.size() == needle.size() || line[ needle.size() ] == ' ' || line[ needle.size() ] == '\t' ) )
            {
                if ( match_prefix.empty()
                  || line.compare( needle.size() + 1, match_prefix.size(), match_prefix ) == 0 )
                {
                    is_match = true;
                }
            }
            if ( is_match )
            {
                // keep exactly one copy: a key that somehow appears several times (older builds
                // appended duplicates) collapses into a single, current line
                if ( !found )
                    out += newline;
                found = true;
            }
            else
            {
                out += line;
                out += '\n';
            }
        }
        if ( !found )
            out += newline;
        content = std::move( out );
    };

    // Everything collected into `keys` above gets written here. This loop used to be missing, so
    // the vector was built and then silently dropped and only the handful of keys below ever
    // reached hud.ini - which is why the layout was invisible (and therefore uneditable) there,
    // and why `saw_layout_key` stayed false and the file got rewritten on every start.
    for ( auto const &kv : keys )
        upsert_line( kv.first, kv.second );

    // TODO: gui.hud.enabled still records the raw switch even when the mode makes the overlay
    // invisible (enabled=true with mode=off), so the two keys can disagree; not fixed here
    upsert_line( "gui.hud.enabled", Global.gui_hud.enabled ? "yes" : "no" );
    if ( Global.gui_hud.mode_saved )
        upsert_line( "gui.hud.mode", std::to_string( Global.gui_hud.mode ) );
    upsert_line( "gui.hud.panel", Global.gui_hud.panel ? "yes" : "no" );
    upsert_line( "gui.hud.margin", std::to_string( Global.gui_hud.margin ) );
    upsert_line( "gui.hud.custom", Global.gui_hud.custom_items );

    {
        std::ostringstream o;
        o << "main " << Global.gui_hud.panel_x << "," << Global.gui_hud.panel_y << ",0";
        upsert_line( "gui.hud.pos", o.str(), "main" );
    }
    // ensure parent dir exists (first-run user config folder)
    if ( !iniPath.parent_path().empty() )
    {
        std::error_code ec;
        fs::create_directories( iniPath.parent_path(), ec );
    }
    // write a temporary file and rename it over the target: an interrupted write then leaves the
    // previous hud.ini untouched instead of truncating it to nothing
    fs::path tmpPath { iniPath };
    tmpPath += ".tmp";
    {
        std::ofstream out( tmpPath, std::ios::trunc );
        if ( !out )
            return; // cannot even open the temporary file: keep whatever is there
        out << content;
        out.flush();
        if ( !out )
        {
            std::error_code ec;
            fs::remove( tmpPath, ec ); // the write failed: drop the partial file, keep the old one
            return;
        }
    }
    std::error_code ec;
    fs::rename( tmpPath, iniPath, ec );
    if ( ec )
        fs::remove( tmpPath, ec ); // rename refused (another device, locked): leave the target as it was
}

void set_mode( int const Mode )
{
    g_mode = std::clamp<int>( Mode, standard, off );
    Global.gui_hud.mode = g_mode; // persisted in hud.ini
    Global.gui_hud.mode_saved = true; // honour this choice on next start
    // the Off mode hides the overlay; any other mode shows it again
    set_visible( g_mode != off );
    // entering Custom mode opens the customisation window (it can be closed with X afterwards);
    // entering Off closes it as well - the HUD is off, the setup window is gone with it
    if ( g_customwindow != nullptr )
        g_customwindow->is_open = ( g_mode == custom );
    g_toast = 1.5f;
    save_hud_settings();
}

void set_custom_window( ui_panel *Panel )
{
    g_customwindow = Panel;
}

int mode_count()
{
    return 3;
}

void cycle_mode()
{
    set_mode( ( mode() + 1 ) % mode_count() );
}

char const *mode_name( int const Mode )
{
    static char const *const names[] = { "Standard", "Custom", "Off" };
    return names[ std::clamp<int>( Mode, standard, off ) ];
}

bool panel_group()
{
    return g_panelon;
}

void set_panel_group( bool const On )
{
    g_panelon = On;
    Global.gui_hud.panel = On;
    apply_visibility();
    save_hud_settings();
}

float toast()
{
    return g_toast;
}

void update_toast( float const DeltaTime )
{
    if ( g_toast > 0.0f )
        g_toast -= DeltaTime;
}

int item_count()
{
    return ITEM_COUNT;
}

hud_item const &item_by_index( int const Index )
{
    return items[ std::clamp( Index, 0, ITEM_COUNT - 1 ) ];
}

bool custom_checked( char const *Id )
{
    int const i { item_index( Id ) };
    return i >= 0 ? g_custom_on[ i ] : true;
}

void set_custom_item( char const *Id, bool const Checked )
{
    int const i { item_index( Id ) };
    if ( i < 0 )
        return;
    g_custom_on[ i ] = Checked;
    // rebuild the persisted csv: every item is listed, "id" when on and "!id" when off, so the
    // file records the player's actual choices and an item added by a later build still defaults
    // to its own default_on instead of being switched off by omission
    std::string csv;
    for ( int j = 0; j < ITEM_COUNT; ++j )
    {
        if ( !csv.empty() )
            csv += ",";
        if ( !g_custom_on[ j ] )
            csv += "!";
        csv += items[ j ].id;
    }
    Global.gui_hud.custom_items = csv;
    save_hud_settings();
}

bool item_visible( char const *Id )
{
    if ( mode() == off )
        return false;
    // Standard mode is the READ-ONLY skin of whatever Custom mode was configured to: the same
    // per-item checks apply in both modes. What differs is only the ability to change things
    // (and the background plate / dragging / mouse handling).
    return custom_checked( Id );
}

bool item_available( char const *Id, vehicle_caps const &Caps )
{
    // vehicle-dependent systems: greyed out in the window when the vehicle has none
    if ( std::string_view( Id ) == "edbrake" )
        return Caps.ed;
    if ( std::string_view( Id ) == "shunt" )
        return Caps.shunt;
    if ( std::string_view( Id ) == "doors" )
        return Caps.doors;
    if ( std::string_view( Id ) == "speedctrl" )
        return Caps.speedctrl;
    return true;
}

// how far the layout elements reach, in the 1920-baseline units every gui.hud.* coordinate is
// written in, measured from the panel's top-left corner. The window IS the clip rect of its own
// draw list, so this is the floor the window size has to meet: with a panel_size smaller than the
// layout the content was cut off at the window edge (the right-hand speed-limit box lost half its
// width and the last pressure row was sliced at the bottom) no matter what the player changed.
// translated text is not measured here - the width of a string is only known once a font has laid
// it out - so this covers every element the renderer places by a coordinate: the row stack, both
// limit boxes, the concentric rings (band edge and the ring text just outside it), the speed /
// direction / grade / distance / zone / cruise blocks and the signal text anchors.
static glm::vec2 layout_extent()
{
    auto const &h { Global.gui_hud };
    // the row stack shrinks itself into main_panel_height() (see the slot loop in
    // render_contents()), so a single row is the footprint that is always there
    float x { h.bars_x + h.bars_w };
    float y { h.bars_y + h.bars_rh };
    // the two speed-limit boxes share their size but not their position
    x = std::max( x, std::max( h.limit1_x, h.limit2_x ) + h.limit_w );
    y = std::max( y, std::max( h.limit1_y, h.limit2_y ) + h.limit_h );
    // the concentric rings: the outer edge of the band, or the ring text drawn along it
    for ( int i = 0; i < 4; ++i )
    {
        float const band { h.arc_r[ i ] + h.arc_band_of[ i ] * 0.5f };
        float const text { h.arc_r[ i ] + std::max( h.arc_name_off[ i ], h.arc_val_off[ i ] )
                                       + std::max( h.arc_name_size[ i ], h.arc_val_size[ i ] ) };
        float const reach { std::max( band, text ) };
        x = std::max( x, h.arc_cx + reach );
        y = std::max( y, h.arc_cy + reach );
    }
    // single-line blocks anchored by their top-left corner (a line is never taller than its size)
    x = std::max( { x, h.spd_x + h.speed_size, h.dist_x + h.dist_size, h.grade_x + h.grade_w,
                    h.zone_x + h.zone_size, h.speedctrl_x, h.dir_x + h.dir_hw } );
    y = std::max( { y, h.spd_y + h.speed_size, h.dist_y + h.dist_size, h.zone_y + h.zone_size,
                    h.speedctrl_y + h.speedctrl_size, h.dir_y + h.dir_hh, h.grade_y } );
    // the signal strip elements: x/y anchor plus their font size
    for ( auto const *tc : { &h.sig_dist, &h.sig_pax, &h.sig_doors, &h.sig_alerter, &h.sig_shp } )
    {
        x = std::max( x, tc->x + tc->size );
        y = std::max( y, tc->y + tc->size );
    }
    return { x, y };
}

// panel width comes straight from the configured layout (gui.hud.panel_size), widened to the
// layout's real extent when that is larger; the arcs and the speed column are laid out inside it
// by their own coordinates. NOTE: this and main_panel_height() return WINDOW PIXELS (the layout
// value already scaled by res_scale()), which is what the renderer and the row stack work in.
int main_panel_width()
{
    float const rs { res_scale() };
    // fixed panel size: configuration-driven (hud.ini), default from the HTML layout draft
    float const w { std::max( static_cast<float>( Global.gui_hud.main_w ), layout_extent().x ) };
    return static_cast<int>( w * rs );
}

int main_panel_height()
{
    // configured layout height (or the layout's own extent, if that is taller), scaled - but never
    // more than 80% of the screen height, so the panel fits on short displays. NOTE the bottom row
    // stack derives its own available space from THIS function (not from main_h) and shrinks
    // itself when the list does not fit.
    float const rs { res_scale() };
    float const h { std::max( static_cast<float>( Global.gui_hud.main_h ), layout_extent().y ) };
    int const px { static_cast<int>( h * rs ) };
    return static_cast<int>( std::min<double>( px, Global.fb_size.y * 0.8 ) );
}

static void sync_custom_items( std::string const &Csv )
{
    // CSV semantics: "<id>" = explicitly ON, "!<id>" = explicitly OFF, anything not mentioned at
    // all falls back to the item's own default. The explicit "!" form is what lets a player turn
    // off an item that is on by default, while a newly added item still shows up for someone who
    // already has a hud.ini written by an older build (a plain "list of ON items" would silently
    // switch every new item off, and a plain default-fallback would resurrect anything the player
    // had unchecked).
    for ( int i = 0; i < ITEM_COUNT; ++i )
        g_custom_on[ i ] = items[ i ].default_on;
    if ( Csv.empty() )
        return;
    for ( auto const &idstr : Split( Csv, ',' ) )
    {
        bool const off { !idstr.empty() && idstr[ 0 ] == '!' };
        char const *const id { off ? idstr.c_str() + 1 : idstr.c_str() };
        int const i { item_index( id ) };
        if ( i >= 0 )
            g_custom_on[ i ] = !off;
    }
}

settings const &get()
{
    // the HUD overlay settings are read/written through the HUD's own config file (hud.ini);
    // the engine's main eu07.ini is left untouched
    return Global.gui_hud;
}

// HUD settings live in their own hud.ini, resolved like the other ini files: the copy in the
// per-user config directory is read first, the one next to the simulator is the fallback (that is
// where the builds before this one wrote it). the engine's main eu07.ini is left untouched.
static void load_hud_settings()
{
    auto const cfgPath { user_config_path( "hud.ini" ) };
    std::string const path { ( false == cfgPath.empty() && std::filesystem::exists( cfgPath ) )
                             ? cfgPath.string() : std::string { "hud.ini" } };
    std::ifstream f( path );
    if ( !f.is_open() )
    {
        // first run (or the file was deleted): write a complete hud.ini holding every layout
        // key at its default, so a player always has a file to look at and edit
        save_hud_settings();
        return;
    }
    std::string line;
    auto &h { Global.gui_hud };
    bool saw_layout_key { false };   // does this file come from a build that knows the layout keys?
    while ( std::getline( f, line ) )
    {
        if ( line.empty() || line[ 0 ] == '#' || line[ 0 ] == '\n' )
            continue;
        std::istringstream ss( line );
        std::string token;
        ss >> token;
        if ( token == "gui.hud.enabled" ) { std::string v; ss >> v; h.enabled = ( v == "yes" ); }
        else if ( token == "gui.hud.mode" ) { int v = 0; ss >> v; h.mode = v; h.mode_saved = true; }
        else if ( token == "gui.hud.panel" ) { std::string v; ss >> v; h.panel = ( v == "yes" ); }
        else if ( token == "gui.hud.strip" ) { std::string v; ss >> v; }  // legacy key, ignored
        else if ( token == "gui.hud.speed_panel" ) { std::string v; ss >> v; }  // legacy key, ignored
        else if ( token == "gui.hud.custom" ) { ss >> h.custom_items; }
        else if ( token == "gui.hud.pos" )
        {
            std::string name, coords;
            ss >> name >> coords;
            int x = -1, y = -1, z = 0;
            if ( std::sscanf( coords.c_str(), "%d,%d,%d", &x, &y, &z ) >= 2 )
            {
                if ( name == "main" ) { h.panel_x = x; h.panel_y = y; }
                // "maindrag" / "strip" / "speed" are legacy names from the removed panels: ignored
            }
        }
        else if ( token == "gui.hud.panel_size" )
        {
            std::string v; ss >> v; int w = 0, hh = 0;
            if ( std::sscanf( v.c_str(), "%d,%d", &w, &hh ) == 2 && w > 0 && hh > 0 ) { h.main_w = w; h.main_h = hh; }
        }
        else if ( token == "gui.hud.scale" )
        {
            std::string v; ss >> v;
            float s = 1.0f;
            if ( std::sscanf( v.c_str(), "%f", &s ) == 1 && s > 0.0f ) { h.scale = s; }
        }
        else if ( token == "gui.hud.arc" )
        {
            saw_layout_key = true;
            std::string v; ss >> v;
            float a = 0, b = 0, c = 0, d = 0, e = 0;
            // 4 values now (band moved into the per-ring lines); a 5th one is an older file and
            // simply ignored
            int const n { std::sscanf( v.c_str(), "%f,%f,%f,%f,%f", &a, &b, &c, &d, &e ) };
            if ( n >= 4 )
            { h.arc_cx = a; h.arc_cy = b; h.arc_a0 = c; h.arc_a1 = d; }
        }
        else if ( token == "gui.hud.arc_r" )
        {
            std::string v; ss >> v;
            float r0 = 0, r1 = 0, r2 = 0, r3 = 0;
            if ( std::sscanf( v.c_str(), "%f,%f,%f,%f", &r0, &r1, &r2, &r3 ) == 4 )
            { h.arc_r[ 0 ] = r0; h.arc_r[ 1 ] = r1; h.arc_r[ 2 ] = r2; h.arc_r[ 3 ] = r3; }
        }
        else if ( token == "gui.hud.arc_col" )
        {
            std::string v; ss >> v;
            unsigned c0 = 0, c1 = 0, c2 = 0, c3 = 0;
            if ( std::sscanf( v.c_str(), "%x,%x,%x,%x", &c0, &c1, &c2, &c3 ) == 4 )
            { h.arc_col[ 0 ] = static_cast<int>( c0 ); h.arc_col[ 1 ] = static_cast<int>( c1 );
              h.arc_col[ 2 ] = static_cast<int>( c2 ); h.arc_col[ 3 ] = static_cast<int>( c3 ); }
        }
        else if ( token == "gui.hud.name_pos" || token == "gui.hud.val_pos" )
        {
            // legacy keys: the ring text is laid along the arc now, so these are read and dropped
            std::string v; ss >> v;
        }
        else if ( token == "gui.hud.arc_text" )
        {
            std::string v; ss >> v;
            float len = 0, hw = 0, rot = 0;
            // current format is "len,hw"; an older file wrote "rot,len,hw". Try the triple first
            // and fall back to the pair, so a malformed value can never zero the arrow out
            if ( std::sscanf( v.c_str(), "%f,%f,%f", &rot, &len, &hw ) == 3 )
            { h.arrow_len = len; h.arrow_hw = hw; }
            else if ( std::sscanf( v.c_str(), "%f,%f", &len, &hw ) == 2 )
            { h.arrow_len = len; h.arrow_hw = hw; }
        }
        else if ( token == "gui.hud.margin" )
        {
            std::string v; ss >> v; int m = 0;
            if ( std::sscanf( v.c_str(), "%d", &m ) == 1 && m >= 0 ) h.margin = m;
        }
        else if ( token == "gui.hud.arc_name" || token == "gui.hud.arc_value" )
        {
            // four groups ("off,pct,space,size") separated by semicolons
            std::string v; ss >> v;
            bool const isname { token == "gui.hud.arc_name" };
            std::istringstream vs( v );
            std::string part;
            int idx = 0;
            while ( idx < 4 && std::getline( vs, part, ';' ) )
            {
                float a = 0, b = 0, c = 0, d = 0;
                if ( std::sscanf( part.c_str(), "%f,%f,%f,%f", &a, &b, &c, &d ) == 4 )
                {
                    if ( isname ) { h.arc_name_off[ idx ] = a; h.arc_name_pct[ idx ] = b; h.arc_name_space[ idx ] = c; h.arc_name_size[ idx ] = d; }
                    else          { h.arc_val_off[ idx ]  = a; h.arc_val_pct[ idx ]  = b; h.arc_val_space[ idx ]  = c; h.arc_val_size[ idx ]  = d; }
                }
                ++idx;
            }
        }
        else if ( token == "gui.hud.warn" )
        {
            std::string v; ss >> v; float a = 0, b = 0, c = 0;
            if ( std::sscanf( v.c_str(), "%f,%f,%f", &a, &b, &c ) == 3 ) { h.warn_low = a; h.warn_mid = b; h.warn_high = c; }
        }
        else if ( token.rfind( "gui.hud.ring.", 0 ) == 0 )
        {
            // "gui.hud.ring.<current|voltage|notch|shunt>" followed by the ring's fields as
            // key=value pairs, the same names the HTML layout page uses
            // the prefix "gui.hud.ring." is 13 characters - this used to read substr( 14 ), which
            // dropped the first letter of the name ("voltage" came out as "oltage"), so no ring
            // setting ever matched and the next save wrote the defaults over the player's values
            std::string const rn { token.substr( 13 ) };
            int idx = -1;
            if ( rn == "voltage" ) idx = 0;
            else if ( rn == "current" ) idx = 1;
            else if ( rn == "notch" ) idx = 2;
            else if ( rn == "shunt" ) idx = 3;
            if ( idx >= 0 )
            {
                saw_layout_key = true;
                std::string kv;
                while ( ss >> kv )
                {
                    auto const eq { kv.find( '=' ) };
                    if ( eq == std::string::npos )
                        continue;
                    std::string const key { kv.substr( 0, eq ) };
                    std::string const val { kv.substr( eq + 1 ) };
                    float const fv { static_cast<float>( std::atof( val.c_str() ) ) };
                    int   const hv { static_cast<int>( std::strtol( val.c_str(), nullptr, 16 ) ) };
                    if      ( key == "r" )         h.arc_r[ idx ] = fv;
                    else if ( key == "band" )      h.arc_band_of[ idx ] = fv;
                    else if ( key == "col" )       h.arc_col[ idx ] = hv;
                    else if ( key == "nameoff" )   h.arc_name_off[ idx ] = fv;
                    else if ( key == "namepct" )   h.arc_name_pct[ idx ] = fv;
                    else if ( key == "namespace" ) h.arc_name_space[ idx ] = fv;
                    else if ( key == "namesize" )  h.arc_name_size[ idx ] = fv;
                    else if ( key == "namecol" )   h.arc_name_col[ idx ] = hv;
                    else if ( key == "valoff" )    h.arc_val_off[ idx ] = fv;
                    else if ( key == "valpct" )    h.arc_val_pct[ idx ] = fv;
                    else if ( key == "valspace" )  h.arc_val_space[ idx ] = fv;
                    else if ( key == "valsize" )   h.arc_val_size[ idx ] = fv;
                    else if ( key == "valcol" )    h.arc_val_col[ idx ] = hv;
                }
            }
        }
        else if ( token == "gui.hud.speed" )
        {
            std::string v; ss >> v; float a = 0, b = 0, c = 0;
            if ( std::sscanf( v.c_str(), "%f,%f,%f", &a, &b, &c ) == 3 ) { h.spd_x = a; h.spd_y = b; h.speed_size = c; }
        }
        else if ( token == "gui.hud.dir" )
        {
            std::string v; ss >> v; float a = 0, b = 0, c = 0, d = 0;
            if ( std::sscanf( v.c_str(), "%f,%f,%f,%f", &a, &b, &c, &d ) == 4 ) { h.dir_x = a; h.dir_y = b; h.dir_hw = c; h.dir_hh = d; }
        }
        else if ( token == "gui.hud.limitbox" )
        {
            std::string v; ss >> v;
            float a = 0, b = 0, c = 0, d = 0, e = 0, f = 0, g = 0, i = 0, j = 0;
            if ( std::sscanf( v.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f", &a, &b, &c, &d, &e, &f, &g, &i, &j ) == 9 )
            { h.limit1_x = a; h.limit1_y = b; h.limit2_x = c; h.limit2_y = d; h.limit_w = e; h.limit_h = f;
              h.limit_bw = g; h.limit_radius = i; h.limit_textfrac = j; }
        }
        else if ( token == "gui.hud.dist" )
        {
            std::string v; ss >> v; float a = 0, b = 0, c = 0;
            if ( std::sscanf( v.c_str(), "%f,%f,%f", &a, &b, &c ) == 3 ) { h.dist_x = a; h.dist_y = b; h.dist_size = c; }
        }
        else if ( token == "gui.hud.grade" )
        {
            std::string v; ss >> v; float a = 0, b = 0, c = 0;
            if ( std::sscanf( v.c_str(), "%f,%f,%f", &a, &b, &c ) == 3 ) { h.grade_x = a; h.grade_y = b; h.grade_w = c; }
        }
        else if ( token == "gui.hud.speedctrl" )
        {
            std::string v; ss >> v; float a = 0, b = 0, c = 0;
            if ( std::sscanf( v.c_str(), "%f,%f,%f", &a, &b, &c ) == 3 ) { h.speedctrl_x = a; h.speedctrl_y = b; h.speedctrl_size = c; }
        }
        else if ( token == "gui.hud.zone" )
        {
            std::string v; ss >> v; float a = 0, b = 0, c = 0;
            if ( std::sscanf( v.c_str(), "%f,%f,%f", &a, &b, &c ) == 3 ) { h.zone_x = a; h.zone_y = b; h.zone_size = c; }
        }
        else if ( token == "gui.hud.bars" )
        {
            std::string v; ss >> v;
            float a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
            unsigned bg = 0; int ga = 0;
            if ( std::sscanf( v.c_str(), "%f,%f,%f,%f,%f,%f,%x,%d", &a, &b, &c, &d, &e, &f, &bg, &ga ) == 8 )
            { h.bars_x = a; h.bars_y = b; h.bars_w = c; h.bars_rh = d; h.bars_gap = e; h.bars_radius = f;
              h.bars_bg = static_cast<int>( bg ); h.bars_bga = ga; }
        }
        else if ( token.rfind( "gui.hud.sig.", 0 ) == 0 )
        {
            // "gui.hud.sig.<dist|pax|doors|alerter|shp>" + x/y/size/col/show as key=value pairs
            // the prefix "gui.hud.sig." is 12 characters - this used to read substr( 13 ), which
            // chopped the first letter off the name, so the lookup below never matched and every
            // one of these lines was silently dropped: the elements always kept the compiled-in
            // defaults and editing them in hud.ini had no effect at all.
            std::string const sn { token.substr( 12 ) };
            decltype( h.sig_dist ) *dst { nullptr };
            if ( sn == "dist" ) dst = &h.sig_dist;
            else if ( sn == "pax" ) dst = &h.sig_pax;
            else if ( sn == "doors" ) dst = &h.sig_doors;
            else if ( sn == "alerter" ) dst = &h.sig_alerter;
            else if ( sn == "shp" ) dst = &h.sig_shp;
            if ( dst != nullptr )
            {
                saw_layout_key = true;
                std::string kv;
                while ( ss >> kv )
                {
                    auto const eq { kv.find( '=' ) };
                    if ( eq == std::string::npos )
                        continue;
                    std::string const key { kv.substr( 0, eq ) };
                    std::string const val { kv.substr( eq + 1 ) };
                    if      ( key == "x" )    dst->x = static_cast<float>( std::atof( val.c_str() ) );
                    else if ( key == "y" )    dst->y = static_cast<float>( std::atof( val.c_str() ) );
                    else if ( key == "size" ) dst->size = static_cast<float>( std::atof( val.c_str() ) );
                    else if ( key == "col" )  dst->col = static_cast<int>( std::strtol( val.c_str(), nullptr, 16 ) );
                    else if ( key == "show" ) dst->show = ( val == "yes" || val == "1" || val == "true" );
                }
            }
        }
    }
    // updates the lines it already has and appends the missing ones, so the player's existing
    // choices survive while every tunable key becomes visible in the file
    if ( !saw_layout_key )
        save_hud_settings();
}

void set_panels( ui_panel *Panel )
{
    load_hud_settings();
    g_panel = Panel;
    g_visible = Global.gui_hud.enabled;
    // fresh installs start with the HUD OFF (F1 brings it up); once the player has chosen
    // a mode it is remembered in the ini (gui.hud.mode) and honoured on the next start
    g_mode = Global.gui_hud.mode_saved ? normalize_hud_mode( Global.gui_hud.mode ) : off;
    g_panelon = Global.gui_hud.panel;
    sync_custom_items( Global.gui_hud.custom_items );
    apply_visibility();
}

void set_visible( bool const Show )
{
    g_visible = Show;
    apply_visibility();
}

bool dragging()
{
    return g_dragging;
}

void set_dragging( bool const Drag )
{
    g_dragging = Drag;
}

void apply_visibility()
{
    // the group switch applies in both modes (it can only be changed in Custom mode); Standard is the
    // Off hides everything (g_visible false)
    // panel visibility follows Custom mode in BOTH modes: Standard is simply the read-only
    // rendering of whatever Custom was configured to look like
    if (g_panel != nullptr)
        g_panel->is_open = g_visible && mode() != off && g_panelon;
}

void set_panel_pos( int const X, int const Y )
{
    // one position for both modes: Standard shows whatever Custom mode was set up to look like.
    // Stored in 1920-baseline units (like the whole layout), so the same spot is meant on any
    // resolution: the window pixel position is divided back by the resolution scale here
    float const rs { hudcfg::res_scale() };
    if ( rs > 0.01f )
    {
        Global.gui_hud.panel_x = static_cast<int>( static_cast<float>( X ) / rs );
        Global.gui_hud.panel_y = static_cast<int>( static_cast<float>( Y ) / rs );
    }
    save_hud_settings();
}

}

//---------------------------------------------------------------------------
// hud_panel: heads-up display overlay (borderless, freely movable window)
//---------------------------------------------------------------------------

// walks the coupling chain (mechanical coupler OR permanent consist link, so a loco
// hauling passenger/freight cars counts as well) from the controlled vehicle in both
// directions; returns the farthest "last car" and fills the consist-wide brake data
// (min cylinder pressure across the chain, vehicle count)
static TDynamicObject const *consist_scan( TDynamicObject const *Controlled, double &MinBrakePress, int &Count )
{
    MinBrakePress = std::numeric_limits<double>::max();
    Count = 1;
    auto const *tail { Controlled };
    if ( Controlled->MoverParameters != nullptr )
        MinBrakePress = Controlled->MoverParameters->BrakePress;
    int const link { coupling::coupler | coupling::permanent }; // any real coupling counts
    for ( int dir = 0; dir < 2; ++dir )
    {
        auto const *v { dir == 0 ? Controlled->Prev( link ) : Controlled->Next( link ) };
        auto const *end { Controlled };
        int n { 0 };
        while ( v != nullptr )
        {
            if ( v->MoverParameters != nullptr )
                MinBrakePress = std::min( MinBrakePress, v->MoverParameters->BrakePress );
            end = v;
            ++n;
            v = dir == 0 ? v->Prev( link ) : v->Next( link );
        }
        Count += n;
        if ( n > 0 ) // the farther end wins; both ends at equal distance -> Prev side (head end)
            tail = end;
    }
    return tail;
}

void
hud_panel::render()
{
    if ( !is_open )
        return;

    int flags { window_flags };
    if ( no_title_bar )
        flags |= ImGuiWindowFlags_NoTitleBar;

    // Standard mode is the READ-ONLY skin: no background plate, no dragging, no resizing and -
    // most importantly - no mouse input at all. Cockpit switches and levers that sit under the
    // overlay have to stay clickable, so the window must not swallow anything.
    bool const readonly_skin { hudcfg::mode() == hudcfg::standard };
    if ( readonly_skin )
        flags |= ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoMove
               | ImGuiWindowFlags_NoResize     | ImGuiWindowFlags_NoInputs
               | ImGuiWindowFlags_NoBringToFrontOnFocus;

    // pos is already in window pixels (hud_panel::update() stores the layout position scaled by
    // res_scale()), so ImVec2S would multiply it by ui.scale a SECOND time: with ui.scale 1.5 a
    // dragged position drifted away frame after frame, and the default corner anchor ended up
    // off screen entirely
    if ( pos.x != -1 && pos.y != -1 )
        ImGui::SetNextWindowPos( ImVec2( pos.x, pos.y ), ImGuiCond_Always );

    // auto-sized to content until the player grabs the resize grip (the bottom-right
    // triangle, matching the other windows); then the size becomes manual
    // main_panel_width()/height() ALREADY scale the configured layout by res_scale(), i.e. they
    // are window pixels, which is also what the contents are painted in. Scaling them by
    // res_scale() a second time made the window res_scale times smaller than the layout it
    // clips (res_scale^2 instead of res_scale), and the draw list then cut its own content off:
    // the right-hand limit boxes lost half their width and the bottom pressure row was sliced.
    ImVec2 const autosize { static_cast<float>( hudcfg::main_panel_width() ),
                            static_cast<float>( hudcfg::main_panel_height() ) };
    // the panel follows the computed layout size ONLY until the player resizes it by hand:
    // once m_manual_size is set, the window keeps the player's own size (and position) and is
    // never forced back to the automatic size again. The read-only skin always uses the layout
    // size, since it cannot be resized in the first place.
    if ( !m_manual_size || readonly_skin )
        ImGui::SetNextWindowSize( autosize, ImGuiCond_Always );
    // minimum size: small enough to shrink the panel by hand, big enough to keep the speed digits usable
    // no upper bound: the height may be dragged out freely
    ImGui::SetNextWindowSizeConstraints(
        ImVec2( 120.0f * hudcfg::res_scale(), 80.0f * hudcfg::res_scale() ),
        ImVec2( 1.0e6f, 1.0e6f ) );

    auto const panelname { ( title.empty() ? m_name : title ) + "###" + m_name };
    if ( ImGui::Begin( panelname.c_str(), &is_open, flags ) )
    {
        // the moment the window size diverges from the auto size, the player took over
        ImVec2 const cur { ImGui::GetWindowSize() };
        if ( std::abs( cur.x - autosize.x ) > 0.5f || std::abs( cur.y - autosize.y ) > 0.5f )
            m_manual_size = true;
        render_contents();
        popups.remove_if( []( std::unique_ptr<ui::popup> const &popup ) { return popup->render(); } );
    }
    ImGui::End();
}

void
hud_panel::update()
{
    auto const &fb { Global.fb_size };
    auto const &cfg { hudcfg::get() };
    // main_panel_*() return window pixels (layout value * res_scale), so `size` is already the
    // pixel size: the anchor and the clamp below work in pixels and take it as it is. Multiplying
    // it by res_scale() again (the old code) made the window res_scale^2 while the contents are
    // painted at res_scale, which is what cut the HUD off at its own window edge
    size = { hudcfg::main_panel_width(), hudcfg::main_panel_height() };
    float const rs { hudcfg::res_scale() };
    float const pw { static_cast<float>( size.x ) };
    float const ph { static_cast<float>( size.y ) };
    hudcfg::update_toast( ImGui::GetIO().DeltaTime );
    if ( hudcfg::dragging() )
    {
        pos = { -1, -1 }; // suspend the anchor while the user drags
    }
    // -1/-1 is the only "position not set" marker: any other pair, negative coordinates included,
    // is a spot the player dragged the panel to (a single -1 coordinate used to count as unset,
    // which made a panel dragged past the left or top edge jump back to the corner). a panel that
    // happens to sit exactly at (-1,-1) is still treated as unset
    else if ( cfg.panel_x == -1 && cfg.panel_y == -1 )
        // default: right edge, just above the bottom-right corner
        pos = { fb.x - pw - cfg.margin,
                fb.y - 205.0f - cfg.margin - ph - 8.0f };
    else
        // the stored position is in 1920-baseline units, like every other layout value, so it
        // has to be scaled to the current display - a position that fits at 1920 would otherwise
        // hang off the side of a 1600-wide window
        pos = { static_cast<int>( static_cast<float>( cfg.panel_x ) * rs ),
                static_cast<int>( static_cast<float>( cfg.panel_y ) * rs ) };

    // no position protection: exactly where the player left it (may be off-screen). the panel is
    // re-fitted to the screen only when the display size itself changes - and then just once -
    // so a layout stored at another resolution cannot stay outside the window, while a panel the
    // player parked half off the edge keeps that spot (the frame-by-frame clamp used to push it
    // back under the cursor the moment it was dragged to the bottom of the screen).
    // a drag suspends the anchor (pos is (-1,-1) above), so nothing is corrected or re-fitted
    // while the player drags, nor after the release.
    float const fbw { static_cast<float>( fb.x ) };
    float const fbh { static_cast<float>( fb.y ) };
    if ( m_last_fb.x != fbw || m_last_fb.y != fbh )
    {
        // first frame counts as a change too: m_last_fb starts at (-1,-1)
        m_last_fb = { fbw, fbh };
        if ( pos.x >= 0 && pos.y >= 0 )
        {
            float const px { std::clamp( static_cast<float>( pos.x ), 0.0f, std::max( 0.0f, fbw - pw ) ) };
            float const py { std::clamp( static_cast<float>( pos.y ), 0.0f, std::max( 0.0f, fbh - ph ) ) };
            // the corrected value is for this frame's rendering only; the configuration keeps the
            // player's own coordinates so the fit can be recomputed on the next resolution change
            pos = { px, py };
        }
    }
}

void
hud_panel::render_contents()
{
    auto winpos { ImGui::GetWindowPos() };
    float const dt { ImGui::GetIO().DeltaTime };
    float const now { static_cast<float>( ImGui::GetTime() ) };
    auto *dl { ImGui::GetWindowDrawList() };

    // draggable module: the whole window is a drag zone (position stored in the common settings)
    {
        auto const wsize { ImGui::GetWindowSize() };
        ImGui::SetCursorPos( ImVec2( 0.0f, 0.0f ) );
        ImGui::InvisibleButton( "##hud_drag", wsize );
        if ( ImGui::IsItemActive() )
        {
            m_dragging = true;
            hudcfg::set_dragging( true );
            auto const delta { ImGui::GetIO().MouseDelta };
            winpos = ImVec2( winpos.x + delta.x, winpos.y + delta.y );
            ImGui::SetWindowPos( winpos );
        }
        if ( m_dragging && ImGui::IsMouseReleased( 0 ) )
        {
            m_dragging = false;
            hudcfg::set_dragging( false );
            winpos = ImGui::GetWindowPos();
            hudcfg::set_panel_pos( static_cast<int>( winpos.x ), static_cast<int>( winpos.y ) );
        }
    }

    auto const *train { simulation::Train };
    auto const *controlled { train ? train->Dynamic() : nullptr };
    auto const *mover { controlled ? controlled->MoverParameters : nullptr };
    if (controlled == nullptr || mover == nullptr)
        return;

    // --- data ----------------------------------------------------------------
    // NOTE: mover->Vel is already in km/h (same value the game's own driving aid uses)
    float const speed_kmh { static_cast<float>( std::abs( controlled->GetVelocity() ) ) };

    auto const *owner { controlled->ctOwner != nullptr ? controlled->ctOwner : controlled->Mechanik };
    int const speedlimit { owner ? static_cast<int>( owner->VelDesired ) : 0 };

    // --- next speed limit and its distance -----------------------------------------------------
    // the controller already computes exactly this figure, for the game's own driving aid and for the
    // AI reference (TController::next_speed_limit(), vehicle/Driver.cpp:3005-3058; called there at
    // Driver.cpp:3064 and in this file's own driving aid, drivingaid_panel::update()). the HUD used
    // to carry a line-by-line copy of it; that copy is gone. the ONE difference it had is kept
    // here, because it is what the
    // rows below want: the controller reports a NEGATIVE limit for "there is none" (no timetable limit
    // and VelNext -1 make its min_speed() return -1), while the boxes print "none" whenever the next
    // limit equals the current one - so a negative figure is folded into the current limit. the
    // distance keeps the controller's unit, metres (the drawing converts to km itself).
    int nextspeedlimit { speedlimit };
    double nextspeedlimitdistance { std::numeric_limits<double>::max() };
    if ( owner != nullptr )
    {
        auto const [ limit, distance ] { owner->next_speed_limit() };
        nextspeedlimit = limit;
        nextspeedlimitdistance = distance;
        if ( nextspeedlimit < 0 )
            nextspeedlimit = speedlimit;
    }

    // current track gradient in per-mille, sign convention taken from driving aid panel; positive = uphill
    auto const reverser { ( mover->DirActive > 0 ? 1 : -1 ) };
    double const grade_pm { controlled->VectorFront().y * 1000.0 * ( controlled->DirectionGet() == reverser ? 1.0 : -1.0 ) * reverser };

    // --- speed state coloring: ONLY speed-limit thresholds change the color ---
    // yellow at limit+1, red at limit+4 (e.g. limit 40: 41 km/h yellow, 44 km/h red)
    bool const overspeed_flash { speedlimit > 0 && speed_kmh >= speedlimit + 4.0f };
    bool const overspeed_warn { speedlimit > 0 && speed_kmh >= speedlimit + 1.0f };

    glm::vec4 target;
    bool danger { false };
    if ( overspeed_flash ) { target = { 1.00f, 0.15f, 0.08f, 1.00f }; danger = true; }
    else if ( overspeed_warn ) { target = { 1.00f, 0.80f, 0.05f, 1.00f }; }
    else { target = { 0.70f, 0.88f, 1.00f, 1.00f }; } // default color, no acceleration/braking tints

    // smooth color transition: fast for the only-flashing state, slow otherwise
    float const k { danger ? 12.0f : 4.0f };
    float const t { 1.0f - std::exp( -k * ( dt > 0.0f ? dt : 0.0f ) ) };
    m_speedcolor += ( target - m_speedcolor ) * t;
    // pulse alpha ONLY in the overspeed+4 state
    float const alpha { danger ? ( 0.60f + 0.40f * std::sin( now * 12.0f ) ) : 1.0f };

    // --- vehicle-type dependent bar list ------------------------------------
    // EMU cab cars may carry no engine data; the consist's POWER UNIT holds the controller state.
    // This matches the game's own driving aid, which reads driver->Controlling() for the throttle.
    auto const *ctrlv { owner != nullptr && owner->Controlling() != nullptr ? owner->Controlling() : mover };
    auto const enginetype { ctrlv->EngineType };
    bool const is_electric { enginetype == TEngineType::ElectricSeriesMotor || enginetype == TEngineType::ElectricInductionMotor };
    bool const is_diesel { enginetype == TEngineType::DieselEngine || enginetype == TEngineType::DieselElectric };
    // traction current source: the sim keeps the SIGNED TOTAL in Itot - for the classic series
    // machines it is the motor current times the number of parallel branches (Mover.cpp:6237-6241),
    // for the inverter drives it comes from the inverter's input current (Mover.cpp:6873). The
    // branch ammeters (fHCurrent) are NOT a usable substitute: they only carry a current on a
    // scheme with parallel circuits, and on an ElectricInductionMotor ShowCurrent(1)/(2) report
    // winding quantities, so summing them left the current ring empty on EN57AKM and the rest of
    // the 93 inverter-driven vehicles in the data set.
    // an inverter drive gets one more correction on top: its input current carries the no-load
    // current I0 over the traction part, so the reading is taken from hud_traction_current(),
    // which subtracts I0 - see the note there.
    bool const dc_series { enginetype == TEngineType::ElectricSeriesMotor };
    // electro-dynamic brake: current flows back (shown negative, red), brake handle becomes ED position.
    // composite (integrated drive+brake) handle vehicles are excluded: their wheel already shows
    // drive/brake bidirectionally, and what they do when pulling back is not ED braking
    bool const composite_handle { ctrlv->EIMCtrlType > 0 || ctrlv->UniCtrlIntegratedBrakeCtrl };
    // the minus is an inverter-drive feature only, exactly as in hud_traction_current(): a DC machine
    // must not get one, or the row would print a minus while the arc - which draws the same figure
    // and can no longer go below zero - stays positive, and the two would contradict each other
    bool const signed_ok { enginetype == TEngineType::ElectricInductionMotor };
    // the runtime flag ALONE is not enough to call it an ED brake: the vehicle has to have the ED
    // hardware (a Vectron has it without any "DynamicBrake" key, a diesel-electric never has it),
    // and the flag has to be read off the POWER UNIT - in an EMU the player sits in a cab car which
    // carries no engine state, so mover->DynamicBrakeFlag is the wrong unit's flag
    bool const dynbrake { signed_ok && !composite_handle
                          && hud_has_electric_brake( *ctrlv )
                          && ctrlv->DynamicBrakeFlag };
    auto const current_col { dynbrake ? IM_COL32( 255, 70, 45, 240 ) : 0u };

    auto const bar_col = []( float const frac ) -> ImU32 {
        if ( frac >= 0.95f ) return IM_COL32( 255, 70, 45, 240 );
        if ( frac >= 0.80f ) return IM_COL32( 255, 200, 40, 240 );
        return IM_COL32( 130, 220, 140, 240 );
    };
    char buf[ 64 ];
    // resolution-relative layout: contents are scaled by res_scale() around the 1920 baseline
    float const rs { hudcfg::res_scale() };
    auto const &hcfg { Global.gui_hud };      // panel layout: hud.ini overrides, else draft defaults

    // --- concentric rings -------------------------------------------------------------------------
    // one shared centre; radius, band, colour and text layout come per ring from gui.hud.arc_r,
    // arc_band_of, arc_col and gui.hud.ring.*, so a ring keeps its size even when another is hidden.
    // the rows are collected first because an arc cannot be drawn before its final index is known.
    // fill: 0% at the arc's left end, 100% at its right end; the notch ring of a stepless (EIM)
    // handle instead grows outwards from the arc's midpoint, upwards for power and downwards for
    // brake, so drive and brake still light opposite halves
    struct gauge_row {
        float frac { 0.0f };
        ImU32 col { 0u };
        std::string label;
        std::string value;
    };
    std::vector<gauge_row> rows {};
    // step of the row counter below; nothing is positioned from it any more - the panel height comes
    // from gui.hud.panel_size or the measured layout, see hudcfg::main_panel_height()
    float const gauge_step { 26.0f * rs }; // vertical step per row (label + value share one line)
    float barY { winpos.y + 6.0f * rs };    // rows are only counted here; they are drawn at the end
    auto const row_add = [ &rows, &bar_col, &barY, &gauge_step ]( char const *Label, float const Frac, char const *ValueText, ImU32 const ColOverride = 0 ) {
        float const f { std::clamp( Frac, 0.0f, 1.0f ) };
        rows.push_back( { f, ColOverride != 0 ? ColOverride : bar_col( f ), std::string( Label ), std::string( ValueText ) } );
        barY += gauge_step;
    };

    if ( hudcfg::item_visible( "power" ) )
    {
    if ( is_diesel )
    {
        // real diesel instrument: engine revolutions (RPM), see EngineRPMRatio/EngineMaxRPM
        double const rpmratio { std::clamp( mover->EngineRPMRatio(), 0.0, 1.0 ) };
        double const rpm { mover->EngineMaxRPM() * rpmratio };
        std::snprintf( buf, sizeof( buf ), STR_C("%.0f rpm"), rpm );
        row_add( STR_C("Engine RPM"), static_cast<float>( rpmratio ), buf );
        barY += hudcfg::row_h();
    }
    else if ( is_electric )
    {
        // the traction current feeds the current ring only: branch ammeters are folded into a single
        // figure so the ring shows the total. the "Current" row queued below is deliberately not in
        // the bottom slot list (see slots[]), so no second readout can disagree with the ring
        // ctrlv, not mover: in an EMU the player sits in a cab car that carries no engine data, and
        // the traction current lives on the consist's power unit (ctrlv is exactly that unit)
        float const imax { ctrlv->Imax > 1.0 ? static_cast<float>( ctrlv->Imax )
                          : mover->Imax > 1.0 ? static_cast<float>( mover->Imax ) : 1000.0f };
        // magnitude only in this row; a standing vehicle reads 0 because hud_traction_current()
        // has already taken the no-load current I0 out of the inverter drive's figure
        float const cur { static_cast<float>( std::abs( hud_traction_current( *ctrlv ) ) ) };
        // the minus is only for a real feed-back of an inverter drive (dynbrake carries signed_ok);
        // a zero never gets one either ("%.0f" of -0.0 is "-0")
        std::snprintf( buf, sizeof( buf ), STR_C("%.0f A"), ( dynbrake && cur >= 0.5f ) ? -cur : cur );
        row_add( STR_C("Current"), std::clamp( cur / imax, 0.0f, 1.0f ), buf, current_col );
        barY += hudcfg::row_h();
    }
    } // item power

    // brake pipe pressure (SPKS). Full scale and the warning levels come from the vehicle
    // data itself (HighPipePress / LowPipePress), never from numbers invented in the HUD
    if ( hudcfg::item_visible( "brakepipe" ) )
    {
    double const pipefull { mover->HighPipePress > 0.1 ? mover->HighPipePress : 5.0 };
    float const pipefrac { std::clamp( static_cast<float>( mover->PipePress / pipefull ), 0.0f, 1.0f ) };
    // the game's own driving aid writes every pressure in kPa with TWO decimals
    // (driveruipanels.cpp:296-300: " Pressure: %.2f kPa (train pipe: %.2f kPa)" fed BrakePress * 100 and
    // PipePress * 100). the "%.1f bar" that used to stand here was the HUD's own invention. the *100 is for the
    // TEXT only - the bar fraction above still divides by the bar-based pipefull, so the gauge fills as before.
    std::snprintf( buf, sizeof( buf ), "%.2f kPa", mover->PipePress * 100.0 );
    ImU32 const pipecol { mover->PipePress < mover->LowPipePress ? IM_COL32( 255, 70, 45, 240 )
                        : ( mover->PipePress < mover->HighPipePress ? IM_COL32( 255, 200, 40, 240 )
                                                                    : IM_COL32( 130, 220, 140, 240 ) ) };
    row_add( STR_C("Brake pipe"), pipefrac, buf, pipecol );
    barY += hudcfg::row_h();
    } // item brakepipe

    // main reservoir / supply circuit pressure (off by default; low supply = brake risk).
    // scale = MaxCompressor, warning = MinCompressor (both from the vehicle data)
    if ( hudcfg::item_visible( "mainres" ) )
    {
        double const resfull { mover->MaxCompressor > 0.1 ? mover->MaxCompressor : 10.0 };
        float const resfrac { std::clamp( static_cast<float>( mover->Compressor / resfull ), 0.0f, 1.0f ) };
        // same unit as every other pressure in the game: kPa, two decimals (driveruipanels.cpp:296; the
        // parameters window prints the tanks as "main: %.2f" in the same kPa scale, widgets/vehicleparams.cpp:201).
        // the fill above keeps the bar-based MaxCompressor ratio
        std::snprintf( buf, sizeof( buf ), "%.2f kPa", mover->Compressor * 100.0 );
        ImU32 const rescol { mover->Compressor < mover->MinCompressor ? IM_COL32( 255, 70, 45, 240 )
                         : ( resfrac < hcfg.warn_high ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 130, 220, 140, 240 ) ) };
        row_add( STR_C("Main reservoir"), resfrac, buf, rescol );
        barY += hudcfg::row_h();
    }

    // main brake handle position (same source as the game's driving aid "basicbraking" value);
    // ALWAYS shown: the electro-dynamic brake is its own persistent strip below the notch row
    if ( hudcfg::item_visible( "trainbrake" ) )
    {
    {
        float const brakcfrac { mover->BrakeCtrlPosNo > 0 ? std::clamp( static_cast<float>( std::max( 0.0, mover->fBrakeCtrlPos ) ) / mover->BrakeCtrlPosNo, 0.0f, 1.0f ) : 0.0f };
        std::snprintf( buf, sizeof( buf ), "%.1f", mover->fBrakeCtrlPos );
        row_add( STR_C("train brake"), brakcfrac, buf );
        barY += hudcfg::row_h();
    }
    } // item trainbrake

    if ( hudcfg::item_visible( "indbrake" ) )
    {
    float const indfrac { static_cast<float>( std::clamp( mover->LocalBrakePosA, 0.0, 1.0 ) ) };
    std::snprintf( buf, sizeof( buf ), "%.0f/10", std::floor( indfrac * 10.0f ) );
    row_add( STR_C("independent brake"), indfrac, buf );
    barY += hudcfg::row_h();
    } // item indbrake

    // brake cylinder pressure (cab gauge "CYLINDER HAMULCOWY"). The ratio is computed exactly
    // like the sim does it (BrakePress / MaxBrakePress[3], see DynObj.cpp), so the HUD never
    // invents a full-scale value of its own
    double const cylfull { mover->MaxBrakePress[ 3 ] > 0.1 ? mover->MaxBrakePress[ 3 ] : 6.0 };
    if ( hudcfg::item_visible( "brakecyl" ) )
    {
    float const cylfrac { std::clamp( static_cast<float>( mover->BrakePress / cylfull ), 0.0f, 1.0f ) };
    // the game's " Pressure: %.2f kPa" is exactly this cylinder pressure (driveruipanels.cpp:296-297,
    // BrakePress * 100); only the unit changes, the fill above keeps the BrakePress / MaxBrakePress ratio
    std::snprintf( buf, sizeof( buf ), "%.2f kPa", mover->BrakePress * 100.0 );
    ImU32 const cylcol { cylfrac < hcfg.warn_low ? IM_COL32( 130, 220, 140, 240 ) : ( cylfrac < hcfg.warn_mid ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 255, 70, 45, 240 ) ) };
    row_add( STR_C("Brake cylinder"), cylfrac, buf, cylcol );
    barY += hudcfg::row_h();
    } // item brakecyl

    // consist-wide brake cylinder data (both advanced items, off by default; a single
    // vehicle is its own "last car", so nothing extra is shown for it). one chain scan
    // serves both the last-car pressure and the worst pressure across the consist
    bool const wantlast { hudcfg::item_visible( "brakecyl_last" ) };
    bool const wantmin { hudcfg::item_visible( "brakecyl_min" ) };
    if ( wantlast || wantmin )
    {
        double minpress { 0.0 };
        int consistcount { 0 };
        auto const *tail { consist_scan( controlled, minpress, consistcount ) };
        if ( wantlast && consistcount > 1 && tail != nullptr )
        {
            double const tailpress { tail->MoverParameters != nullptr ? tail->MoverParameters->BrakePress : 0.0 };
            float const tfrac { std::clamp( static_cast<float>( tailpress / cylfull ), 0.0f, 1.0f ) };
            // cylinder pressure, so kPa with two decimals like the game's own readout (driveruipanels.cpp:219)
            std::snprintf( buf, sizeof( buf ), "%.2f kPa", tailpress * 100.0 );
            ImU32 const tcol { tfrac < hcfg.warn_low ? IM_COL32( 130, 220, 140, 240 ) : ( tfrac < hcfg.warn_mid ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 255, 70, 45, 240 ) ) };
            row_add( STR_C("Last car brake cyl"), tfrac, buf, tcol );
            barY += hudcfg::row_h();
        }
        if ( wantmin && consistcount > 1 )
        {
            // colour is INVERTED vs. the local cylinder bar: the lowest pressure in the
            // consist is the danger case (a car without braking), so low = red
            float const worstfrac { std::clamp( static_cast<float>( minpress / cylfull ), 0.0f, 1.0f ) };
            // cylinder pressure, so kPa with two decimals like the game's own readout (driveruipanels.cpp:219)
            std::snprintf( buf, sizeof( buf ), "%.2f kPa", minpress * 100.0 );
            ImU32 const worstcol { worstfrac < hcfg.warn_low ? IM_COL32( 255, 70, 45, 240 ) : ( worstfrac < hcfg.warn_mid ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 130, 220, 140, 240 ) ) };
            row_add( STR_C("Worst brake cyl"), worstfrac, buf, worstcol );
            barY += hudcfg::row_h();
        }
    }

    // cruise control (tempomat): plain numeric row, no bar - it is a state, not a gauge.
    // green = active, grey = speed selected but inactive, OFF = nothing set
    // position and size come from gui.hud.speedctrl (x,y,size in hud.ini) so the line can be moved
    // off the brake-bar stack without a rebuild. barY is no longer a usable anchor for it: the rows
    // above are drawn as arcs at the end of this function, and the counter is a pure row planner
    if ( hudcfg::item_visible( "speedctrl" ) && mover->SpeedCtrl )
    {
        // this line is translated text, so its width is not part of the measured layout extent
        // (hudcfg::layout_extent() cannot know a string before a font has laid it out). the panel
        // window is the clip rect of its own draw list, so a line that starts inside and runs past
        // the window edge is cut there without any extra clip rect
        if ( mover->SpeedCtrlValue > 0.5 )
        {
            std::snprintf( buf, sizeof( buf ), "%s: %.0f km/h", STR_C("cruise control"), mover->SpeedCtrlValue );
            dl->AddText( ui_layer::font_mono, hcfg.speedctrl_size * rs,
                ImVec2( winpos.x + hcfg.speedctrl_x * rs, winpos.y + hcfg.speedctrl_y * rs ),
                mover->SpeedCtrlUnit.IsActive ? IM_COL32( 90, 220, 120, 245 ) : IM_COL32( 190, 200, 215, 220 ), buf );
        }
        else
            dl->AddText( ui_layer::font_mono, hcfg.speedctrl_size * rs,
                ImVec2( winpos.x + hcfg.speedctrl_x * rs, winpos.y + hcfg.speedctrl_y * rs ),
                IM_COL32( 140, 150, 165, 180 ), STR_C("cruise control") );
        // the counter is still stepped for consistency, but no row is positioned from it: the rows
        // are placed by the fixed slot list at gui.hud.bars
        barY += hudcfg::row_h();
    }
    // wheel slip has no readout any more: the item was removed with the top strip

    // the sim uses two controller models; their position SEMANTICS differ:
    //  - MCPN-defined (EU07 etc.): handle position = MainCtrlPos, max = MainCtrlPosNo
    //  - RList-defined (EN57-class): state = MainCtrlActualPos (RList index), max = RlistSize
    // MainCtrlActualPos is an internal scheme pointer and must NOT be mixed in for MCPN vehicles.
    bool const mcpn_ctrl { ctrlv->MainCtrlPosNo > 0 };
    int const notchpos { mcpn_ctrl ? ctrlv->MainCtrlPos : ctrlv->MainCtrlActualPos };
    int const notchmax { mcpn_ctrl ? ctrlv->MainCtrlPosNo : ctrlv->RlistSize };
    // stepless (EIM / unified) handle: the driving trailer and the power unit are not always the
    // same vehicle in an EMU, so accept EITHER of them carrying an EIM definition
    bool const eim_throttle { mover->EIMCtrlType > 0 || ctrlv->EIMCtrlType > 0 };
    char zone[ 32 ];
    float notchfrac { 0.0f };
    std::string notchtext;
    {
        char nbuf[ 48 ];
        if ( eim_throttle )
        {
            // unified drive+brake control (e.g. SM42 handwheel): positive = drive, negative = brake
            double const pct { std::clamp( ctrlv->eimic_real * 100.0, -100.0, 100.0 ) };
            if ( pct > 3.0 ) std::snprintf( zone, sizeof( zone ), "%s", STR_C("Traction") );
            else if ( pct < -3.0 ) std::snprintf( zone, sizeof( zone ), "%s", STR_C("Braking") );
            else std::snprintf( zone, sizeof( zone ), "%s", STR_C("Neutral") );
            notchfrac = static_cast<float>( std::abs( pct ) / 100.0 );
            std::snprintf( nbuf, sizeof( nbuf ), "%+.0f%%", pct );
            notchtext = nbuf;
        }
        else if ( is_diesel )
        {
            // diesel: throttle positions (RList holds fuel fill for diesels)
            std::snprintf( zone, sizeof( zone ), "%s", notchpos > 0 ? STR_C("Throttle") : STR_C("Idle") );
            if ( notchmax > 0 )
            {
                std::snprintf( nbuf, sizeof( nbuf ), "%d/%d", notchpos, notchmax );
                notchtext = nbuf;
            }
            notchfrac = notchmax > 0 ? std::clamp( static_cast<float>( notchpos ) / notchmax, 0.0f, 1.0f ) : 0.0f;
        }
        else
        {
            // electric: series / parallel / shunt zones; scheme lookup uses the physical state index
            if ( notchpos > 0 || ctrlv->ScndCtrlPos > 0 )
            {
                if ( ctrlv->ScndCtrlPos > 0 ) std::snprintf( zone, sizeof( zone ), "%s", STR_C("second controller") );
                // guard the scheme lookup: RList is a fixed array whose valid length is RlistSize
                else if ( ctrlv->MainCtrlActualPos > 0 && ctrlv->MainCtrlActualPos <= ctrlv->RlistSize )
                {
                    auto const &scheme { ctrlv->RList[ctrlv->MainCtrlActualPos] };
                    if ( scheme.ScndAct > 0 ) std::snprintf( zone, sizeof( zone ), "%s", STR_C("second controller") );
                    else if ( scheme.Bn > 1 ) std::snprintf( zone, sizeof( zone ), "%s", STR_C("Parallel") );
                    else if ( scheme.Mn > 1 ) std::snprintf( zone, sizeof( zone ), "%s", STR_C("Series") );
                    else std::snprintf( zone, sizeof( zone ), "%s", STR_C("Traction") );
                }
                else
                    std::snprintf( zone, sizeof( zone ), "%s", STR_C("Traction") );
            }
            else
                std::snprintf( zone, sizeof( zone ), "%s", STR_C("Idle") );
            if ( notchmax > 0 )
            {
                std::snprintf( nbuf, sizeof( nbuf ), "%d/%d", notchpos, notchmax );
                notchtext = nbuf;
            }
            notchfrac = notchmax > 0 ? std::clamp( static_cast<float>( notchpos ) / notchmax, 0.0f, 1.0f ) : 0.0f;
        }
    }
    // --- notch / controller row (flows right after the instrument bars) --------
    char notchvalue[ 128 ]; // row value text; filled right before each row_add call below
    if ( hudcfg::item_visible( "notch" ) )
    {
    // value text = handle position + zone name (e.g. "9/16 parallel"); built BEFORE the row
    // is queued, because row_add copies the text (queuing it first was a bug: the row then
    // received whatever the uninitialised buffer happened to hold - the flickering text)
    std::snprintf( notchvalue, sizeof( notchvalue ), "%s%s%s", notchtext.c_str(), notchtext.empty() ? "" : " ", zone );
    row_add( STR_C("master controller"), notchfrac, notchvalue, eim_throttle ? ( ctrlv->eimic_real < 0.0 ? IM_COL32( 255, 190, 40, 230 ) : IM_COL32( 90, 220, 120, 230 ) ) : IM_COL32( 90, 160, 255, 230 ) );
    } // item notch

    // --- persistent strips under the notch row ------------------------------------
    // electro-dynamic brake strip: shown ONLY when the vehicle has a dedicated ED handle
    // (SplitEDPneumaticBrake=Yes, e.g. Vectron with its DBPN lever); automatic/switch/passive
    // ED types have no separate ED handle, so they get no strip. EP09 (104E) stays excluded
    // regardless of configuration (its automatic-ED data is unreliable per vehicle devs)
    auto const vehname { ToLower( mover->Name ) };
    bool const is_ep09 { vehname.compare( 0, 4, "ep09" ) == 0 || vehname.compare( 0, 4, "104e" ) == 0 };
    bool const has_ed { mover->SplitEDPneumaticBrake && !is_ep09 };
    // exactly as the previous HUD did: read the CONTROLLING vehicle, and require both a positive
    // SCPN and a DC series machine (SCPN alone is a power-step count on induction vehicles)
    bool const has_shunt { ctrlv->ScndCtrlPosNo > 0 && dc_series };
    if ( has_ed && hudcfg::item_visible( "edbrake" ) )
    {
        // dedicated ED handle position (0..1 -> notch count), same style as the shunt strip
        char dbval[ 32 ];
        float const dbno { mover->DynamicBrakeCtrlPosNo > 0 ? static_cast<float>( mover->DynamicBrakeCtrlPosNo ) : 10.0f };
        float const dbfrac { static_cast<float>( mover->DynamicBrakeCtrlPos ) };
        std::snprintf( dbval, sizeof( dbval ), "%.0f/%.0f", std::floor( dbfrac * dbno + 0.5f ), dbno );
        row_add( STR_C("Dynamic brake"), dbfrac, dbval, IM_COL32( 110, 220, 250, 230 ) );
    }
    // field-weakening (shunt) controller row; only DC electrics have real field weakening
    // (on induction vehicles SCPN>0 means a power step switch, not a shunt controller)
    if ( has_shunt && hudcfg::item_visible( "shunt" ) )
    {
        float const shfrac { std::clamp( static_cast<float>( ctrlv->ScndCtrlPos ) / ctrlv->ScndCtrlPosNo, 0.0f, 1.0f ) };
        std::snprintf( notchvalue, sizeof( notchvalue ), "%d/%d", ctrlv->ScndCtrlPos, ctrlv->ScndCtrlPosNo ); // own text, not the notch's
        row_add( STR_C("second controller"), shfrac, notchvalue, IM_COL32( 180, 130, 255, 230 ) );
    }

    // --- draw the collected arcs --------------------------------------------------------------
    // pushed outermost first: voltage, current (or engine rpm), power handle (notch), field
    // weakening (shunt); a vehicle without field weakening gets three. every arc sweeps clockwise
    // from gui.hud.arc_a0 to arc_a1 around the shared centre gui.hud.arc_cx / arc_cy, and the value
    // sits at the arc's far end.
    {
        struct gauge_arc { ImU32 col; char const *name; float frac; std::string text; int cfgidx; };
        std::vector<gauge_arc> arcs;
        int notch_idx { -1 };   // where the notch ring lands in `arcs` (set when it is pushed)
        auto const ring_col = [ &hcfg ]( int const idx ) -> ImU32 {
            int const c { hcfg.arc_col[ idx < 4 ? idx : 3 ] };
            return IM_COL32( ( c >> 16 ) & 0xFF, ( c >> 8 ) & 0xFF, c & 0xFF, 240 );
        };

        // Ring order, outermost -> innermost: VOLTAGE, CURRENT (engine RPM on diesels), NOTCH,
        // SHUNT. NOTE: the order here IS the order on screen, so the voltage ring must be pushed
        // first.
        // 1) overhead voltage. A diesel (or any non-electric) has no catenary at all, so its ring
        //    is simply not added; every ring keeps its own radius and colour (indexed by ring, not by
//    list position), so the remaining rings stay exactly where they were
        if ( is_electric && hudcfg::item_visible( "voltage" ) )
        {
            float const v { static_cast<float>( mover->PantographVoltage ) };
            float const vfull { mover->NominalVoltage > 1.0 ? static_cast<float>( mover->NominalVoltage ) : 3000.0f };
            char b[ 32 ];
            std::snprintf( b, sizeof( b ), "%.1f kV", v / 1000.0f );
            arcs.push_back( { ring_col( 0 ), STR_C("Voltage"), std::clamp( v / vfull, 0.0f, 1.0f ), b, 0 } );
        }
        // 2) traction current on electrics, engine revolutions on diesels. The diesel branch uses
        //    the same source and the same translated format as the "Engine RPM" row below, so the
        //    two can never disagree.
        if ( is_diesel && hudcfg::item_visible( "power" ) )
        {
            double const rpmratio { std::clamp( mover->EngineRPMRatio(), 0.0, 1.0 ) };
            double const rpm { mover->EngineMaxRPM() * rpmratio };
            char b[ 32 ];
            std::snprintf( b, sizeof( b ), STR_C("%.0f rpm"), rpm );
            arcs.push_back( { ring_col( 1 ), STR_C("Engine RPM"), static_cast<float>( rpmratio ), b, 1 } );
        }
        else if ( is_electric && hudcfg::item_visible( "power" ) )
        {
            float const imax { ctrlv->Imax > 1.0 ? static_cast<float>( ctrlv->Imax )
                              : mover->Imax > 1.0 ? static_cast<float>( mover->Imax ) : 1000.0f };
            // the sign is KEPT here: the current is signed, and it goes negative while the machine
            // feeds current back during regenerative braking, so the arc can show it with a minus
            // sign. the I0 subtraction of hud_traction_current() keeps that sign, because a regen
            // figure is -X + I0 there and -X + I0 - I0 is still -X. hud_traction_current() also
            // decides WHO is allowed the minus (an inverter drive with the electric brake behind it;
            // every other machine is returned as a magnitude), so the same figure feeds the row below
            // and this arc - they can never disagree.
            // the lit segment is sized by the magnitude but coloured by the sign - red while the
            // current is negative (regen). the value text keeps its own colour (hcfg.arc_val_col),
            // so the digits stay white and only the lit part turns red.
            float const cur { static_cast<float>( hud_traction_current( *ctrlv ) ) };
            char b[ 32 ];
            std::snprintf( b, sizeof( b ), "%.0f A", cur );
            ImU32 const cur_col { cur < 0.0f ? IM_COL32( 255, 70, 45, 240 ) : ring_col( 1 ) };
            arcs.push_back( { cur_col, STR_C("Current"), std::clamp( std::abs( cur ) / imax, 0.0f, 1.0f ), b, 1 } );
        }
        // 3) power handle / notch (green). The displayed text comes from the SAME source the
        //    bottom row uses (notchtext): a stepless handle reports a percentage there, a
        //    stepped one reports n/N. Never format it here again - re-formatting at the draw
        //    site is exactly how the arc and the row drifted apart before.
        // remember where the notch ring ended up: not every vehicle draws all four rings (a
        // diesel has no voltage ring), so the NOTCH markers must not assume index 2
        if ( hudcfg::item_visible( "notch" ) )
        {
            notch_idx = static_cast<int>( arcs.size() );
            arcs.push_back( { ring_col( 2 ), STR_C("master controller"), notchfrac, notchtext, 2 } );
        }
        // 4) field weakening / shunt (orange) - only for vehicles that really have it
        if ( has_shunt && hudcfg::item_visible( "shunt" ) )
        {
            char b[ 32 ];
            std::snprintf( b, sizeof( b ), "%d/%d", ctrlv->ScndCtrlPos, ctrlv->ScndCtrlPosNo );
            arcs.push_back( { ring_col( 3 ), STR_C("second controller"),
                              std::clamp( static_cast<float>( ctrlv->ScndCtrlPos ) / ctrlv->ScndCtrlPosNo, 0.0f, 1.0f ), b, 3 } );
        }

        int const acount { static_cast<int>( arcs.size() ) };
        float const nbandw { hcfg.arc_band_of[ 2 ] * rs };   // the notch ring's band (identity index 2)
        // all placement is configuration-driven (panel-relative coordinates)
        float const a0 { hcfg.arc_a0 * 0.017453292f };     // deg -> rad
        float const a1 { hcfg.arc_a1 * 0.017453292f };
        float const cx { winpos.x + hcfg.arc_cx * rs };
        float const cy { winpos.y + hcfg.arc_cy * rs };
        ImVec2 const centre { cx, cy };
        // 1) grey tracks first (widest to narrowest)
        for ( int i = 0; i < acount; ++i )
        {
            int const ci { arcs[ i ].cfgidx };
            float const radius { hcfg.arc_r[ ci ] * rs };
            dl->PathClear();
            dl->PathArcTo( centre, radius, a0, a1, 60 );
            float const bnd { hcfg.arc_band_of[ ci ] * rs };   // each ring has its own width
            dl->PathStroke( IM_COL32( 255, 255, 255, 34 ), false, bnd );
        }
        // 2) the lit part. A stepless (EIM) handle gets a centre-zero gauge instead: the arrow
        //    and the fill grow from the arc's MIDPOINT, upwards for power and downwards for brake
        for ( int i = 0; i < acount; ++i )
        {
            auto const &a { arcs[ i ] };
            float const radius { hcfg.arc_r[ a.cfgidx ] * rs };
            float const bnd { hcfg.arc_band_of[ a.cfgidx ] * rs };   // this ring's band width
            if ( eim_throttle && a.cfgidx == 2 )
            {
                float const amid { ( a0 + a1 ) * 0.5f };
                float const ev { std::clamp( static_cast<float>( ctrlv->eimic_real ), -1.0f, 1.0f ) };
                if ( std::abs( ev ) > 0.005f )
                {
                    dl->PathClear();
                    if ( ev > 0.0f )
                        dl->PathArcTo( centre, radius, amid, amid + ( a1 - amid ) * ev, 60 );
                    else
                        dl->PathArcTo( centre, radius, amid + ( a0 - amid ) * std::abs( ev ), amid, 60 );
                    dl->PathStroke( ev > 0.0f ? IM_COL32( 90, 220, 120, 245 ) : IM_COL32( 255, 190, 40, 245 ), false, bnd );
                }
                continue;
            }
            if ( a.frac <= 0.005f )
                continue;
            dl->PathClear();
            dl->PathArcTo( centre, radius, a0, a0 + ( a1 - a0 ) * a.frac, 60 );
            dl->PathStroke( a.col, false, bnd );
        }
        // 3) name and value are laid ALONG their own arc: every character gets its own arc
        // position and is rotated to the local tangent - the same thing the layout draft does
        // with SVG textPath. the drawing lives in arc_text_frame::draw() (anonymous namespace);
        // this is only the frame the two calls below share.
        arc_text_frame const arced_text { dl, centre, a0, a1 };
        for ( int i = 0; i < acount; ++i )
        {
            auto const &a { arcs[ i ] };
            // everything about a ring - colour (picked when it was pushed), radius, text layout -
            // now keys off its IDENTITY, not its position in the list. Hiding one ring therefore
            // leaves the others exactly where they are instead of shifting them outwards.
            int const k { a.cfgidx };
            // name: along its own arc, with its own colour (defaults to white, independent of
            // the ring colour), sitting `arc_name_off` outside the band centre
            int const nc { hcfg.arc_name_col[ k ] };
            arced_text.draw( hcfg.arc_name_size[ k ] * rs, ( hcfg.arc_r[ k ] + hcfg.arc_name_off[ k ] ) * rs,
                        IM_COL32( ( nc >> 16 ) & 0xFF, ( nc >> 8 ) & 0xFF, nc & 0xFF, 245 ),
                        a.name, hcfg.arc_name_pct[ k ], hcfg.arc_name_space[ k ] * rs, false );
            // value: along the same arc near its far end, with its own colour as well
            int const vc { hcfg.arc_val_col[ k ] };
            arced_text.draw( hcfg.arc_val_size[ k ] * rs, ( hcfg.arc_r[ k ] + hcfg.arc_val_off[ k ] ) * rs,
                        IM_COL32( ( vc >> 16 ) & 0xFF, ( vc >> 8 ) & 0xFF, vc & 0xFF, 245 ),
                        a.text, hcfg.arc_val_pct[ k ], hcfg.arc_val_space[ k ] * rs, false );
        }

        // --- NOTCH arc marks (no text: the arc itself carries the information) ---------------
        //   * an arrow pointing at the arc        = the ACTUAL handle position
        //   * a dashed blue tick                  = the position the driver SET by hand
        //   * thin grey ticks                     = positions with NO starting resistor (Bn > 0)
        if ( notch_idx >= 0 && notchmax > 0 )
        {
            float const nr { hcfg.arc_r[ 2 ] * rs };   // the notch ring's radius (identity index 2)
            auto const nrad = [ &a0, &a1, &notchmax ]( float const k ) { return a0 + ( a1 - a0 ) * ( k / static_cast<float>( notchmax ) ); };
            auto const tick = [ &centre, &nr, dl ]( float const a, float const half, ImU32 const col, float const wdt, bool const dashed ) {
                // 'a' is already a mapped arc angle
                float const ca { std::cos( a ) };
                float const sa { std::sin( a ) };
                ImVec2 const p0 { centre.x + ca * ( nr - half ), centre.y + sa * ( nr - half ) };
                ImVec2 const p1 { centre.x + ca * ( nr + half ), centre.y + sa * ( nr + half ) };
                if ( !dashed )
                    dl->AddLine( p0, p1, col, wdt );
                else
                    for ( int s = 0; s < 4; s += 2 )           // four segments -> a dashed look
                    {
                        float const t0 { static_cast<float>( s ) / 4.0f };
                        float const t1 { static_cast<float>( s + 1 ) / 4.0f };
                        dl->AddLine( ImVec2( p0.x + ( p1.x - p0.x ) * t0, p0.y + ( p1.y - p0.y ) * t0 ),
                                     ImVec2( p0.x + ( p1.x - p0.x ) * t1, p0.y + ( p1.y - p0.y ) * t1 ), col, wdt );
                    }
            };
            // a position counts as "no starting resistor in circuit" when its scheme has R == 0
            // on a live traction step (Bn > 0). used for the ticks AND for the arrow colour, so
            // the two can never disagree
            auto const no_resistor = [ ctrlv ]( int const k ) -> bool {
                return k >= 1 && k <= ctrlv->RlistSize
                    && ctrlv->RList[ k ].R == 0.0 && ctrlv->RList[ k ].Bn > 0;
            };
            // the marks only mean something when the handle positions and the resistance table line
            // up one to one (EU07: 43 handle steps, 43 table entries). where they do not (EN57: 3
            // handle steps against a 14-entry table) the entries are internal states the driver
            // cannot even select, so nothing is marked or highlighted on such vehicles
            bool const marks_meaningful { ctrlv->RlistSize > 0 && ctrlv->MainCtrlPosNo == ctrlv->RlistSize };
            // 1) positions that reach full voltage without any starting resistor in circuit
            for ( int k = 1; marks_meaningful && k <= ctrlv->RlistSize; ++k )
                if ( no_resistor( k ) )
                    tick( nrad( static_cast<float>( k ) ), 7.0f * rs, IM_COL32( 255, 255, 255, 80 ), 2.0f * rs, false );
            // 2) RList-defined vehicles may switch to a different index than the one the driver
            //    set: mark the driver's own position with a dashed tick (MCPN/EIM: not applicable)
            if ( !eim_throttle && !mcpn_ctrl && ctrlv->MainCtrlPos != ctrlv->MainCtrlActualPos )
                tick( nrad( static_cast<float>( ctrlv->MainCtrlPos ) ), 10.0f * rs, IM_COL32( 120, 200, 255, 240 ), 3.0f * rs, true );
            // 3) the arrow: for a stepless handle it slides out from the arc's MIDPOINT (up =
            //    power, down = brake); otherwise it follows this vehicle's handle position
            {
                float const a { eim_throttle
                    ? ( ( a0 + a1 ) * 0.5f + ( ( a1 - a0 ) * 0.5f ) * std::clamp( static_cast<float>( ctrlv->eimic_real ), -1.0f, 1.0f ) )
                    : nrad( static_cast<float>( notchpos ) ) };
                float const ca { std::cos( a ) };
                float const sa { std::sin( a ) };
                float const rtip { nr + nbandw * 0.5f + 2.0f * rs };
                float const rbase { rtip + hcfg.arrow_len * rs };
                ImVec2 const tip { centre.x + ca * rtip, centre.y + sa * rtip };
                ImVec2 const base { centre.x + ca * rbase, centre.y + sa * rbase };
                float const nx { -sa };
                float const ny { ca };
                float const hw { hcfg.arrow_hw * rs };
                // the arrow turns green while the handle sits on a position with no starting
                // resistor in circuit: a visual hint that this step is a "free" one
                // use the SAME handle value the arrow is placed with (`notchpos`): MCPN vehicles
                // address their list by the handle position, RList vehicles by the state index -
                // passing 0 here for MCPN types was why the arrow never lit up
                bool const on_free_position { !eim_throttle && marks_meaningful && no_resistor( notchpos ) };
                ImU32 const arrowcol { on_free_position ? IM_COL32( 90, 220, 120, 245 )
                                                        : IM_COL32( 255, 255, 255, 245 ) };
                dl->AddTriangleFilled( tip, ImVec2( base.x + nx * hw, base.y + ny * hw ), ImVec2( base.x - nx * hw, base.y - ny * hw ),
                                       arrowcol );
                tick( a, nbandw * 0.25f, arrowcol, 2.0f * rs, false );
            }
        }
    }

    // (positions below come straight from the HTML layout draft, panel-relative)

        // --- speed readout fused into this panel (was the separate speed panel) -------------
        // big digits + km/h + direction arrows + grade, drawn in the blank area RIGHT of the
        // arcs, vertically centred so the arc wraps around it
        if ( hudcfg::item_visible( "speed" ) )
        {
            float const fontsize { hcfg.speed_size * rs };   // size and placement from the layout draft
            char sbuf[ 16 ];
            std::snprintf( sbuf, sizeof( sbuf ), "%03.0f", speed_kmh );
            ImFont * const font { ui_layer::font_hud };   // ImGui in this base takes a non-const ImFont* in AddText
            ImVec2 const dpos { winpos.x + hcfg.spd_x * rs, winpos.y + hcfg.spd_y * rs };
            ImU32 const col { ImGui::ColorConvertFloat4ToU32( ImVec4( m_speedcolor.x, m_speedcolor.y, m_speedcolor.z, alpha ) ) };
            dl->AddText( font, fontsize, ImVec2( dpos.x + 3.0f * rs, dpos.y + 3.0f * rs ), IM_COL32( 0, 0, 0, 150 ), sbuf );
            dl->AddText( font, fontsize, dpos, col, sbuf );
            // km/h label removed on request
        }   // end of item "speed"

        // direction arrow: independent of the speed digits (it used to be nested inside the
        // speed block, so unchecking "Speed" silently took the arrow away as well)
        if ( hudcfg::item_visible( "direction" ) )
        {
            float const ax { winpos.x + hcfg.dir_x * rs };
            float const ay { winpos.y + hcfg.dir_y * rs };
            float const hw { hcfg.dir_hw * rs };
            float const hh { hcfg.dir_hh * rs };
            ImU32 const arrowcol { IM_COL32( 225, 235, 250, 240 ) };
            if ( mover->DirActive > 0 )
                dl->AddTriangleFilled( ImVec2( ax - hw, ay + hh ), ImVec2( ax + hw, ay + hh ), ImVec2( ax, ay ), arrowcol );
            else if ( mover->DirActive < 0 )
                dl->AddTriangleFilled( ImVec2( ax - hw, ay ), ImVec2( ax + hw, ay ), ImVec2( ax, ay + hh ), arrowcol );
        }

    // --- limit boxes, distance and the grade bar (positions from the layout draft) -----------
    {
        auto const limitbox = [ &winpos, &rs, dl, &hcfg ]( float const X, float const Y, ImU32 const Col, char const *Text ) {
            float const x { winpos.x + X * rs };
            float const y { winpos.y + Y * rs };
            float const w { hcfg.limit_w * rs };
            float const h { hcfg.limit_h * rs };
            dl->AddRect( ImVec2( x, y ), ImVec2( x + w, y + h ), Col, hcfg.limit_radius * rs, 0, hcfg.limit_bw * rs );
            float const size { h * hcfg.limit_textfrac };
            ImVec2 const ts { ui_layer::font_hud->CalcTextSizeA( size, std::numeric_limits<float>::max(), 0.0f, Text ) };
            dl->AddText( ui_layer::font_hud, size, ImVec2( x + ( w - ts.x ) * 0.5f, y + ( h - ts.y ) * 0.5f ), Col, Text );
        };
        char lbuf[ 16 ];
        // current speed limit (blue box) - toggled by its own item
        if ( hudcfg::item_visible( "limit" ) )
        {
            std::snprintf( lbuf, sizeof( lbuf ), "%d", speedlimit );
            limitbox( hcfg.limit1_x, hcfg.limit1_y, IM_COL32( 127, 208, 255, 245 ), lbuf );
        }
        // next speed limit (orange box); "--" when it equals the current one
        if ( hudcfg::item_visible( "nextlimit" ) )
        {
            if ( nextspeedlimit != speedlimit )
                std::snprintf( lbuf, sizeof( lbuf ), "%d", nextspeedlimit );
            else
                std::snprintf( lbuf, sizeof( lbuf ), "%s", STR_C("none") );
            limitbox( hcfg.limit2_x, hcfg.limit2_y, IM_COL32( 224, 164, 60, 245 ), lbuf );
        }
        // distance to that next limit: its own item, so turning the orange box off keeps the
        // readout (and the other way round). the text and its "none" fallback are unchanged
        if ( hudcfg::item_visible( "nextlimitdist" ) )
        {
            if ( nextspeedlimitdistance < 5000.0 )
                std::snprintf( lbuf, sizeof( lbuf ), "%.0fm", nextspeedlimitdistance );
            else
                std::snprintf( lbuf, sizeof( lbuf ), "%s", STR_C("none") );
            dl->AddText( ui_layer::font_mono, hcfg.dist_size * rs, ImVec2( winpos.x + hcfg.dist_x * rs, winpos.y + hcfg.dist_y * rs ), IM_COL32( 255, 255, 255, 235 ), lbuf );
        }
        // grade: a triangle for the direction, with the value in percent under it (toggleable
        // like every other item - it used to be drawn unconditionally, making its F1 checkbox dead)
        if ( hudcfg::item_visible( "grade" ) )
        {
            float const gx { winpos.x + hcfg.grade_x * rs };
            float const gy { winpos.y + hcfg.grade_y * rs };
            float const gw { hcfg.grade_w * rs };
            float const midx { gx + gw * 0.5f };
            // direction triangle only: the horizontal bar and its fill are gone
            if ( grade_pm < -2.5 ) // downhill
                dl->AddTriangleFilled( ImVec2( midx - 14.0f * rs, gy ), ImVec2( midx + 14.0f * rs, gy ),
                                       ImVec2( midx, gy + 24.0f * rs ), IM_COL32( 255, 255, 255, 240 ) );
            else if ( grade_pm > 2.5 ) // uphill
                dl->AddTriangleFilled( ImVec2( midx - 14.0f * rs, gy + 24.0f * rs ), ImVec2( midx + 14.0f * rs, gy + 24.0f * rs ),
                                       ImVec2( midx, gy ), IM_COL32( 255, 255, 255, 240 ) );
            // the gradient itself, right under the triangle. the game writes it as a PERCENT with one
            // decimal (driveruipanels.cpp:249-253: the grade it feeds that row is VectorFront().y * 100, and
            // the row is STR_C(" Grade: %.1f%%%%")), while grade_pm is per-mille (VectorFront().y * 1000).
            // so the text divides by 10 - the triangle thresholds above stay on the per-mille figure
            if ( std::abs( grade_pm ) > 2.5 )
            {
                char gbuf[ 32 ];
                std::snprintf( gbuf, sizeof( gbuf ), "%.1f%%", std::abs( grade_pm ) / 10.0 );
                float const gsize { 15.0f * rs };
                float const gtw { ui_layer::font_mono->CalcTextSizeA( gsize, std::numeric_limits<float>::max(), 0.0f, gbuf ).x };
                dl->AddText( ui_layer::font_mono, gsize, ImVec2( midx - gtw * 0.5f, gy + 30.0f * rs ),
                             IM_COL32( 255, 255, 255, 240 ), gbuf );
            }
        }
    }

    // --- notch zone name (Series / Parallel / Shunt / Traction / Idle / Throttle, ...) ----------
    // the text is composed up in the controller section, but the row that carried it is never
    // drawn (the slot list below deliberately omits Notch), so it is placed as its own element
    if ( hudcfg::item_visible( "zone" ) )
        dl->AddText( ui_layer::font_mono, hcfg.zone_size * rs,
            ImVec2( winpos.x + hcfg.zone_x * rs, winpos.y + hcfg.zone_y * rs ),
            IM_COL32( 255, 255, 255, 245 ), zone );

    // --- the air/brake/power rows along the bottom (positions from the HTML layout draft) ------
    // their values are already collected in `rows`
    {
        // The slot list is FIXED (per the design draft) and in a stable order: the seven
        // air/brake rows first, then the dynamic brake row last. A slot whose row was not
        // collected (item off, or the vehicle has no such system - only one vehicle currently
        // has an electric brake) stays empty instead of pulling the following rows up. Power /
        // Notch / Shunt are deliberately NOT listed: those values are already shown on the arcs,
        // and their item checkboxes still control those arcs.
        char const *const slots[] {
            "Brake pipe", "Main reservoir", "train brake", "independent brake",
            "Brake cylinder", "Last car brake cyl", "Worst brake cyl",
            "Dynamic brake" };
        int const slot_count { static_cast<int>( sizeof( slots ) / sizeof( slots[ 0 ] ) ) };
        {
            float const bx { winpos.x + hcfg.bars_x * rs };
            float const by { winpos.y + hcfg.bars_y * rs };
            float const bw { hcfg.bars_w * rs };
            // how many rows will actually be drawn? (a slot with no row - item off, or the
            // vehicle has no such system - takes no space)
            int visible { 0 };
            for ( int k = 0; k < slot_count; ++k )
                for ( auto const &rr : rows )
                    if ( rr.label == STR_C( slots[ k ] ) )
                    {
                        ++visible;
                        break;
                    }
            // The stack has to fit between bars_y and the bottom of the panel. Rather than assume
            // a fixed number of rows, the row height and the gap SCALE when the list is taller
            // than the space left (the digits scale with them below) - so any number of visible
            // rows stays inside the panel, at any resolution.
            float const base_rh { hcfg.bars_rh * rs };
            float const base_gap { hcfg.bars_gap * rs };
            float rh { base_rh };
            float gap { base_gap };
            if ( visible > 0 )
            {
                // the REAL window height, not the configured one: main_panel_height() is capped by
                // 80% of the screen, so on a very wide and short display the window is shorter than
                // main_h and a shrink computed from main_h would not be enough to keep the stack in
                float const avail { static_cast<float>( hudcfg::main_panel_height() ) - hcfg.bars_y * rs - 6.0f * rs };
                // the last row needs no gap after it
                float const need { static_cast<float>( visible ) * base_rh
                                 + static_cast<float>( visible - 1 ) * base_gap };
                if ( need > avail && need > 0.0f )
                {
                    float const shrink { std::max( 0.25f, avail / need ) };   // never shrink to nothing
                    rh = base_rh * shrink;
                    gap = base_gap * shrink;
                }
            }
            float const rowscale { base_rh > 0.0f ? rh / base_rh : 1.0f };
            float const textsize { 13.0f * rs * rowscale };
            int drawn { 0 };
            for ( int k = 0; k < slot_count; ++k )
            {
                gauge_row const *found { nullptr };
                for ( auto const &rr : rows )
                    if ( rr.label == STR_C( slots[ k ] ) )
                    {
                        found = &rr;
                        break;
                    }
                if ( found == nullptr )
                    continue;
                auto const &r { *found };
                float const y { by + ( rh + gap ) * static_cast<float>( drawn ) };
                ++drawn;
                dl->AddRectFilled( ImVec2( bx, y ), ImVec2( bx + bw, y + rh ),
                                   IM_COL32( ( hcfg.bars_bg >> 16 ) & 0xFF, ( hcfg.bars_bg >> 8 ) & 0xFF, hcfg.bars_bg & 0xFF, hcfg.bars_bga ),
                                   hcfg.bars_radius * rs );
                // keep the fill behind the text readable, but not as dark as the first attempt
                auto const dim = []( ImU32 const c ) -> ImU32 {
                    int const rr { static_cast<int>( ( c >> IM_COL32_R_SHIFT ) & 0xFF ) };
                    int const gg { static_cast<int>( ( c >> IM_COL32_G_SHIFT ) & 0xFF ) };
                    int const bb { static_cast<int>( ( c >> IM_COL32_B_SHIFT ) & 0xFF ) };
                    return IM_COL32( rr * 3 / 4, gg * 3 / 4, bb * 3 / 4, 205 );
                };
                ImU32 const dimcol { dim( r.col ) };
                dl->AddRectFilled( ImVec2( bx + 2.0f * rs, y + 2.0f * rs ),
                                   ImVec2( bx + 2.0f * rs + ( bw - 4.0f * rs ) * std::clamp( r.frac, 0.0f, 1.0f ), y + rh - 2.0f * rs ),
                                   dimcol );
                // the value is right-aligned in the bar, so measure it FIRST and leave the label
                // only the room that is left: the label used to be drawn from the left with no
                // measurement at all, and a longer translation (Polish was reported back in
                // September) ran straight over the number
                float const vw { ui_layer::font_mono->CalcTextSizeA( textsize, std::numeric_limits<float>::max(), 0.0f, r.value.c_str() ).x };
                float const lab_x { bx + 8.0f * rs };
                float const lab_room { bx + bw - 8.0f * rs - vw - 4.0f * rs }; // 4 px of air before the value
                dl->PushClipRect( ImVec2( lab_x, y ), ImVec2( std::max( lab_x, lab_room ), y + rh ), true );
                dl->AddText( ui_layer::font_mono, textsize, ImVec2( lab_x, y + ( rh - textsize ) * 0.5f ), IM_COL32( 232, 240, 250, 235 ), r.label.c_str() );
                dl->PopClipRect();
                dl->AddText( ui_layer::font_mono, textsize, ImVec2( bx + bw - 8.0f * rs - vw, y + ( rh - textsize ) * 0.5f ), IM_COL32( 255, 255, 255, 245 ), r.value.c_str() );
            }
        }
    }

    // --- elements fused in from the old top signal strip ---------------------------------------
    // position / size / colour / visibility come from hud.ini (defaults match the layout page);
    // the strings themselves stay translated
    {
        auto const *sig_train { simulation::Train };
        auto const *sig_controlled { sig_train ? sig_train->Dynamic() : nullptr };
        auto const *sig_mover { sig_controlled ? sig_controlled->MoverParameters : nullptr };
        auto const *sig_owner { sig_controlled ? ( sig_controlled->ctOwner != nullptr ? sig_controlled->ctOwner : sig_controlled->Mechanik ) : nullptr };
        auto const cfgcol = []( int const c, int const a ) -> ImU32 {
            return IM_COL32( ( c >> 16 ) & 0xFF, ( c >> 8 ) & 0xFF, c & 0xFF, a );
        };
        // emit a string at the element's configured anchor, in its configured size and colour.
        // these are translated texts, the one kind of content whose width the layout cannot know up
        // front (hudcfg::layout_extent() measures coordinates, not glyph runs). they are clipped to
        // the PANEL like every other element: the panel window is the mask of the whole overlay, so
        // a banner that starts inside and runs past the plate edge is cut there instead of being
        // painted outside it
        auto const put = [ &dl, &rs, &winpos, &cfgcol ]( decltype( hcfg.sig_dist ) const &tc, char const *txt, int const alpha ) {
            auto const screen { ImGui::GetIO().DisplaySize };
            dl->PushClipRect( ImVec2( 0.0f, 0.0f ), ImVec2( screen.x, screen.y ), true );
            dl->AddText( ui_layer::font_mono, tc.size * rs,
                         ImVec2( winpos.x + tc.x * rs, winpos.y + tc.y * rs ), cfgcol( tc.col, alpha ), txt );
            dl->PopClipRect();
        };
        char sbuf[ 96 ];
        if ( sig_owner != nullptr && hudcfg::item_visible( "sigdist" ) )
        {
            double const sigdist { sig_owner->FirstSemaphorDist };
            if ( sigdist < 5000.0 )
            {
                std::snprintf( sbuf, sizeof( sbuf ), STR_C("Signal %.0f m"), sigdist );
                put( hcfg.sig_dist, sbuf, 230 );
            }
        }
        if ( sig_owner != nullptr && hcfg.sig_pax.show && hudcfg::item_visible( "passenger" ) && sig_owner->ExchangeTime > 0.0 )
        {
            std::snprintf( sbuf, sizeof( sbuf ), STR_C(" Loading/unloading in progress (%d s left)"),
                           static_cast<int>( std::ceil( sig_owner->ExchangeTime ) ) );
            put( hcfg.sig_pax, sbuf, 230 );
        }
        if ( sig_mover != nullptr && hcfg.sig_doors.show && hudcfg::item_visible( "doors" ) )
        {
            bool const doorl { sig_mover->Doors.instances[ 1 ].is_open || sig_mover->Doors.instances[ 1 ].is_opening };
            bool const doorr { sig_mover->Doors.instances[ 0 ].is_open || sig_mover->Doors.instances[ 0 ].is_opening };
            if ( doorl || doorr )
            {
                std::snprintf( sbuf, sizeof( sbuf ), STR_C("Doors: %s %s"), doorl ? "L" : "-", doorr ? "R" : "-" );
                put( hcfg.sig_doors, sbuf, 230 );
            }
        }
        // CA (vigilance) and SHP (cab signal) banners - same triggers as the game's driving aid
        // ("!ALERTER! " / "!SHP!"), pulsing so they read as an alarm
        if ( sig_mover != nullptr && hudcfg::item_visible( "alarm" ) )
        {
            bool const cabflash { sig_mover->SecuritySystem.is_cabsignal_blinking() };
            bool const vflash { sig_mover->SecuritySystem.is_vigilance_blinking()
                                && ( sig_train != nullptr ? sig_train->fBlinkTimer > 0 : true ) };
            float const pulse { 0.55f + 0.45f * std::sin( static_cast<float>( ImGui::GetTime() ) * 10.0f ) };
            int const palpha { static_cast<int>( 255.0f * pulse ) };
            if ( hcfg.sig_alerter.show && vflash )
                put( hcfg.sig_alerter, STR_C("!ALERTER! "), palpha );
            if ( hcfg.sig_shp.show && cabflash )
                put( hcfg.sig_shp, STR_C("!SHP!"), palpha );
        }
    }

    // the separate speed panel is no longer used: its readout lives in this panel now

    // --- mode-name feedback (F1 / menu) ------------------------------------------------
    if ( hudcfg::toast() > 0.0f )
    {
        // translation keys: "HUD: " and the mode name; falls back to the literal until the
        // assets translation update lands (per jakubg1 that is handled later)
        std::string const toastmsg { std::string( STR_C("HUD: ") ) + STR_C( hudcfg::mode_name( hudcfg::mode() ) ) };
        int const toastalpha { static_cast<int>( std::clamp( hudcfg::toast() / 0.5f, 0.0f, 1.0f ) * 255.0f ) };
        dl->AddText( ui_layer::font_mono, 17.0f, ImVec2( winpos.x + 12.0f, winpos.y + 12.0f ), IM_COL32( 255, 255, 255, toastalpha ), toastmsg.c_str() );
    }
}

//---------------------------------------------------------------------------
// hud_custom_panel: HUD customisation window (per-item checkboxes + presets)
//---------------------------------------------------------------------------

void
hud_custom_panel::update()
{
    // nothing to poll: checkbox state is written immediately via hudcfg::set_custom_item
}

void
hud_custom_panel::render_contents()
{
    // vehicle-dependent availability, same rules as the HUD panels:
    // dedicated ED handle only (EP09 excluded), shunt only on DC series machines with SCPN,
    // doors on EMU/DMU, cruise control only when fitted
    auto const *train { simulation::Train };
    auto const *controlled { train ? train->Dynamic() : nullptr };
    auto const *mover { controlled ? controlled->MoverParameters : nullptr };
    auto const *owner { controlled ? ( controlled->ctOwner != nullptr ? controlled->ctOwner : controlled->Mechanik ) : nullptr };
    auto const *ctrlv { ( owner != nullptr && owner->Controlling() != nullptr ) ? owner->Controlling() : mover };
    hudcfg::vehicle_caps caps;
    if ( mover != nullptr )
    {
        auto const vehname { ToLower( mover->Name ) };
        caps.ed = mover->SplitEDPneumaticBrake
            && !( vehname.compare( 0, 4, "ep09" ) == 0 || vehname.compare( 0, 4, "104e" ) == 0 );
        caps.doors = ( mover->TrainType & ( dt_EZT | dt_DMU ) ) != 0;
        caps.speedctrl = mover->SpeedCtrl;
    }
    if ( ctrlv != nullptr )
    {
        // same rule the previous HUD used: the CONTROLLING vehicle must report a positive SCPN
        // and a DC series machine (on induction vehicles SCPN is the power-step count)
        caps.shunt = ctrlv->ScndCtrlPosNo > 0 && ctrlv->EngineType == TEngineType::ElectricSeriesMotor;
    }

    // current mode
    std::string const modestr { std::string( STR_C("Current mode: ") ) + STR_C( hudcfg::mode_name( hudcfg::mode() ) ) };
    ImGui::TextUnformatted( modestr.c_str() );
    ImGui::Separator();

    // single group switch: the whole main panel (applies only in Custom mode; Standard always
    // shows it). The old top-strip and speed-panel switches are gone with those panels.
    if ( hudcfg::mode() == hudcfg::custom )
    {
        bool panelon { hudcfg::panel_group() };
        if ( ImGui::Checkbox( STR_C("Main panel"), &panelon ) )
            hudcfg::set_panel_group( panelon );
    }
    else
    {
        ImGui::TextDisabled( STR_C("Group switches can only be changed in Custom mode") );
    }
    ImGui::Separator();

    // per-item checkboxes: every item now belongs to the single main panel; items the current
    // vehicle does not have are greyed out with a "-" marker
    int const groups[] = { 0 };
    for ( int const group : groups )
    {
        ImGui::TextUnformatted( STR_C("Main panel") );
        for ( int i = 0; i < hudcfg::item_count(); ++i )
        {
            auto const &it { hudcfg::item_by_index( i ) };
            if ( it.group != group )
                continue;
            bool const avail { hudcfg::item_available( it.id, caps ) };
            if ( !avail )
            {
                ImGui::TextDisabled( "%s -", STR_C( it.name ) );
                continue;
            }
            bool on { hudcfg::custom_checked( it.id ) };
            if ( ImGui::Checkbox( STR_C( it.name ), &on ) )
                hudcfg::set_custom_item( it.id, on );
        }
        ImGui::Spacing();
    }
}
