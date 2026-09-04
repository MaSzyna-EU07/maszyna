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
#include "vehicle/Train.h"
#include "vehicle/Driver.h"
#include "model/AnimModel.h"
#include "vehicle/DynObj.h"
#include "McZapkie/MOVER.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#ifdef WITH_OPENGL_MODERN
#include "rendering/opengl33renderer.h"
#endif
#include "widgets/map_objects.h"
#include "utilities/Logs.h"
#include "widgets/vehicleparams.h"
#include "utilities/U8.h"

#define DRIVER_HINT_CONTENT
#include "application/driverhints.h"

#ifdef WITH_UART
#include "utilities/uart.h"
#endif

void
drivingaid_panel::update() {

    if( false == is_open ) { return; }

	text_lines.clear();

    auto const *train { simulation::Train };
    auto const *controlled { ( train ? train->Dynamic() : nullptr ) };

    if( controlled == nullptr
     || controlled->Mechanik == nullptr ) { return; }

    auto const *mover = controlled->MoverParameters;
    auto const *driver = controlled->Mechanik;
    auto const *owner = controlled->ctOwner != nullptr ? controlled->ctOwner : controlled->Mechanik;

    { // throttle, velocity, speed limits and grade
        std::string expandedtext;
        if( is_expanded ) {
            // grade
            std::string gradetext;
            auto const reverser { ( mover->DirActive > 0 ? 1 : -1 ) };
            auto const grade { controlled->VectorFront().y * 100 * ( controlled->DirectionGet() == reverser ? 1 : -1 ) * reverser };
            if( std::abs( grade ) >= 0.25 ) {
                std::snprintf(
                    m_buffer.data(), m_buffer.size(),
				    STR_C(" Grade: %.1f%%%%"),
                    grade );
                gradetext = m_buffer.data();
            }
            // next speed limit
            auto const speedlimit { static_cast<int>( owner->VelDesired ) };
            auto nextspeedlimit { speedlimit };
            auto nextspeedlimitdistance { std::numeric_limits<double>::max() };
            if( speedlimit != 0 ) { // if we aren't allowed to move then any next speed limit is irrelevant
                // nie przekraczać rozkladowej
                auto const schedulespeedlimit { (
                    (owner->OrderCurrentGet() & (Obey_train | Bank)) != 0 && owner->TrainParams.TTVmax > 0.0 ? static_cast<int>( owner->TrainParams.TTVmax ) :
                    (owner->OrderCurrentGet() & (Obey_train | Bank)) == 0 ? static_cast<int>( owner->fShuntVelocity ) :
                        -1 ) };
                // first take note of any speed change which should occur after passing potential current speed limit
                if( owner->VelLimitLastDist.second > 0 ) {
                    nextspeedlimit = min_speed( schedulespeedlimit, static_cast<int>( owner->VelLimitLastDist.first ) );
                    nextspeedlimitdistance = owner->VelLimitLastDist.second;
                }
                // then take into account speed change ahead, compare it with speed after potentially clearing last limit
                // lower of these two takes priority; otherwise limit lasts at least until potential last limit is cleared
                auto const noactivespeedlimit { owner->VelLimitLastDist.second < 0 };
                auto const speedatproximitydistance { min_speed( schedulespeedlimit, static_cast<int>( owner->VelNext ) ) };
                if( speedatproximitydistance == nextspeedlimit ) {
                    if( noactivespeedlimit ) {
                        nextspeedlimit = speedatproximitydistance;
                        nextspeedlimitdistance = owner->ActualProximityDist;
                    }
                }
                else if( speedatproximitydistance < nextspeedlimit ) {
                    // if the speed limit ahead is more strict than our current limit, it's important enough to report
                    if( speedatproximitydistance < owner->VelDesired ) {
                        nextspeedlimit = speedatproximitydistance;
                        nextspeedlimitdistance = owner->ActualProximityDist;
                    }
                    // otherwise report it only if it's located after our current (lower) limit ends
                    else if( owner->ActualProximityDist > nextspeedlimitdistance ) {
                        nextspeedlimit = speedatproximitydistance;
                        nextspeedlimitdistance = owner->ActualProximityDist;
                    }
                }
                else if( noactivespeedlimit ) { // implicit proximity > last, report only if last limit isn't present
                    nextspeedlimit = speedatproximitydistance;
                    nextspeedlimitdistance = owner->ActualProximityDist;
                }
                // HACK: if our current speed limit extends beyond our scan range don't display potentially misleading information about its length
                if( nextspeedlimitdistance >= EU07_AI_SPEEDLIMITEXTENDSBEYONDSCANRANGE ) {
                    nextspeedlimit = speedlimit;
                }
                // HACK: hide next speed limit if the 'limit' is a vehicle in front of us
                else if( owner->ActualProximityDist == std::abs( owner->TrackObstacle() ) ) {
                    nextspeedlimit = speedlimit;
                }
            }
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
            mover->EIMCtrlType > 0 ? std::max(0, static_cast<int>(100.4 * mover->eimic_real)) : driver->Controlling()->MainCtrlPos,
            mover->EIMCtrlType > 0 ? driver->Controlling()->MainCtrlPos : driver->Controlling()->ScndCtrlPos,
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
        auto const basicbraking { mover->fBrakeCtrlPos };
        auto const eimicbraking { std::max( 0.0, -100.0 * mover->eimic_real ) };
        std::snprintf(
            m_buffer.data(), m_buffer.size(),
            STR_C("Brakes: %5.1f+%-2.0f%c%s"),
//            ( mover->EIMCtrlType == 0 ? basicbraking : mover->EIMCtrlType == 3 ? ( mover->UniCtrlIntegratedBrakeCtrl ? eimicbraking : basicbraking ) : eimicbraking ),
            mover->UniCtrlIntegratedBrakeCtrl ? eimicbraking : basicbraking,
            mover->LocalBrakePosA * LocalBrakePosNo,
            mover->SlippingWheels ? '!' : ' ',
            expandedtext.c_str() );

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
    auto const panelname { (
        title.empty() ?
		    m_name :
            title )
		+ "###" + m_name };
    if( true == ImGui::Begin( panelname.c_str(), &is_open, flags ) ) {
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
            ImGui::TextColored( ImVec4( line.color.r, line.color.g, line.color.b, line.color.a ), line.data.c_str() );
        }
        // hints
        if( owner != nullptr ) {
            if( true == ImGui::CollapsingHeader( STR_C("Hints"), ImGuiTreeNodeFlags_DefaultOpen ) ) {
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
        auto textline = Bezogonkow( owner->Relation(), true );
        if( false == textline.empty() ) {
            textline += " (" + Bezogonkow( owner->TrainName(), true ) + ")";
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
				auto const *previous { owner->pVehicle->Prev( coupling::permanent ) };
				if( previous != nullptr ) { consistmass -= previous->MoverParameters->TotalMass; }
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
                auto const station { (
                    Bezogonkow( tableline->StationName, true )
                    + "                                  " )
                    .substr( 0, 34 ) };
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
                      || time.wHour * 60 + time.wMinute + time.wSecond * 0.0167 >= tableline->Dh * 60 + tableline->Dm ) ) };
                auto const loadchangeinprogress { ( static_cast<int>(std::ceil(-1.0 * owner->fStopTime)) > 0 ) };
                auto const isatpassengerstop { true == owner->IsAtPassengerStop && vehicle->MoverParameters->Vel < 1.0 };
                auto const traveltime { (
                    i < 2 ? "   " :
                    tableline->Ah >= 0 ? to_minutes_str( CompareTime( table.TimeTable[ i - 1 ].Dh, table.TimeTable[ i - 1 ].Dm, tableline->Ah, tableline->Am ), false, 3 ) :
                    to_minutes_str( std::max( 0.0, CompareTime( table.TimeTable[ i - 1 ].Dh, table.TimeTable[ i - 1 ].Dm, tableline->Dh, tableline->Dm ) - 0.5 ), false, 3 ) ) };
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
    auto const panelname { (
        title.empty() ?
            m_name :
            title )
        + "###" + m_name };
    if( true == ImGui::Begin( panelname.c_str(), &is_open, flags ) ) {
        for( auto const &line : text_lines ) {
            ImGui::TextColored( ImVec4( line.color.r, line.color.g, line.color.b, line.color.a ), line.data.c_str() );
        }
        if( is_expanded ) {
            ImGui::PushStyleVar( ImGuiStyleVar_ItemSpacing, ImVec2( 1, 0 ) );
            for( auto const &line : m_tablelines ) {
                ImGui::TextColored( ImVec4( line.color.r, line.color.g, line.color.b, line.color.a ), line.data.c_str() );
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
    auto const panelname { (
        title.empty() ?
		    m_name :
            title )
		+ "###" + m_name };
    if( true == ImGui::Begin( panelname.c_str(), &is_open, flags ) ) {
        // header section
        for( auto const &line : text_lines ) {
            ImGui::TextColored( ImVec4( line.color.r, line.color.g, line.color.b, line.color.a ), line.data.c_str() );
        }
        // sections
        ImGui::Separator();
        if( true == render_section( "Vehicle", m_vehiclelines ) ) {
            if( DebugModeFlag && m_input.mover && m_input.mover->DamageFlag != 0 ) {
                if( true == ImGui::Button( "Stop and repair consist" ) ) {
                    command_relay relay;
                    relay.post(user_command::resetconsist, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(0.0f), &m_input.vehicle->name());
                }
            }
        }
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
            int ports_num = UartStatus.available_ports.size();
            char **avlports = new char*[ports_num];
            for (int i=0; i < ports_num; i++) {
                avlports[i] = (char *) UartStatus.available_ports[i].c_str();
            }
            ImGui::Combo("Port", &UartStatus.selected_port_index, avlports, ports_num);
            ImGui::Combo("Baud", &UartStatus.selected_baud_index, uart_baudrates_list, uart_baudrates_list_num);
            ImGui::Checkbox("Enabled", &UartStatus.enabled);
        }
#endif
        // toggles
        ImGui::Separator();
        bool flag = DebugModeFlag;
        if (ImGui::Checkbox("Debug Mode", &flag)) {
            command_relay relay;
            relay.post(user_command::debugtoggle, 0.0, 0.0, GLFW_RELEASE, 0);
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
            command_relay relay;
            relay.post(
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
			command_relay relay;
            relay.post(
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
            command_relay relay;
            relay.post(
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
            command_relay relay;
            relay.post(
                user_command::setdatetime,
                std::clamp( Global.fMoveLight, 0.0f, 365.0f ),
                simulation::Time.data().wHour * 60 + simulation::Time.data().wMinute,
                GLFW_PRESS, 0 );
        }
    }
    // dynamic material update checkbox
    ImGui::Checkbox( "Update Item Materials", &Global.UpdateMaterials );
    if( DebugModeFlag ) {
        if( ImGui::Checkbox( "Force Daylight", &Global.FakeLight ) ) {
            simulation::Environment.on_daylight_change();
        }
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
                command_relay relay;
                relay.post(
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
        std::string( m_input.mechanik ? STR(" R") + ( mover.Radio ? std::to_string( m_input.mechanik->iRadioChannel ) : "-" ) : "" ).c_str(),
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
        mover.ManualBrakePos / static_cast<float>(ManualBrakePosNo),
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
		AccN_jerk_graph.update((mover.AccN - last_AccN) / dt);
		AccN_acc_graph.update(mover.AccN);
	}

	last_AccN = mover.AccN;
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
	ImGui::PlotLines("##plot", data.data(), data.size(), pos, nullptr, 0.0f, range, ImVec2(0, 100));
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

	for( auto const &delay : delays ) {
		if( ( mover.BrakeDelayFlag & delay.first ) == delay.first ) {
			brakedelay += delay.second;
		}
	}

	return brakedelay;
}

void
debug_panel::update_section_engine( std::vector<text_line> &Output ) {

	if( m_input.vehicle == nullptr ) { return; }
	if( m_input.mover == nullptr ) { return; }

	auto const &vehicle{ *m_input.vehicle };
	auto const &mover{ *m_input.mover };

	// induction motor data
	if( mover.EngineType == TEngineType::ElectricInductionMotor ) {

		Output.emplace_back( "      eimc:            eimv:            press:", Global.UITextColor );
		for( int i = 0; i <= 20; ++i ) {

			std::string parameters =
			    mover.eimc_labels[ i ] + to_string( mover.eimc[ i ], 2, 9 )
			    + " | "
			    + mover.eimv_labels[ i ] + to_string( mover.eimv[ i ], 2, 9 );

			if( i < 10 ) {
				parameters +=
                    m_input.train != nullptr && m_input.train->Dynamic() == m_input.vehicle ? " | " + TTrain::fPress_labels[i] + to_string(m_input.train->fPress[i][0], 2, 9) : "";
			}
			else if( i == 12 ) {
				parameters += "        med:";
			}
			else if( i >= 13 ) {
				parameters += " | " + vehicle.MED_labels[ i - 13 ] + to_string( vehicle.MED[ 0 ][ i - 13 ], 2, 9 );
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
        for( auto const &parameter : paramvalues ) {
            parameterstext += "\n" + parameter.first + to_string( parameter.second, 2, 9 );
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
		for( auto const &parameter : hydrovalues ) {
			parameterstext += "\n" + parameter.first + to_string( parameter.second, 2, 9 );
		}
		Output.emplace_back( parameterstext, Global.UITextColor );
	}
}

void
debug_panel::update_section_ai( std::vector<text_line> &Output ) {

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
        Output.emplace_back( "Current signal: " + Bezogonkow( mechanik.eSignNext->m_name ), Global.UITextColor );
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
    for( auto const &restriction : restrictions ) {
        if( restriction.first < 0.0 ) { continue; }
        if( false == restrictionstext.empty() ) {
            restrictionstext += ", ";
        }
        restrictionstext +=
            to_string( restriction.first, 0 )
            + " (" + restriction.second + ")";
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
debug_panel::update_section_scantable( std::vector<text_line> &Output ) {

	if( m_input.mechanik == nullptr ) { return; }

	Output.emplace_back( "Flags:       Dist:    Vel:  Name:", Global.UITextColor );

	auto const &mechanik{ *m_input.mechanik };

	std::size_t i = 0; std::size_t const speedtablesize = std::clamp( static_cast<int>( mechanik.TableSize() ) - 1, 0, 30 );
	do {
		auto const scanline = mechanik.TableText( i );
		if( scanline.empty() ) { break; }
		Output.emplace_back( Bezogonkow( scanline ), Global.UITextColor );
		++i;
	} while( i < speedtablesize );
	if( Output.size() == 1 ) {
		Output.front().data = "(no points of interest)";
	}
}

#ifdef WITH_UART
void
debug_panel::update_section_uart( std::vector<text_line> &Output ) {
    uart_status *status = &UartStatus;

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
debug_panel::update_section_scenario( std::vector<text_line> &Output ) {

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
        .substr( 0 + 2 * std::floor( std::fmod( 8 + ( glm::radians( simulation::Environment.wind_azimuth() ) + 0.5 * M_PI_4 ) / M_PI_4, 8 ) ), 2 )
        + ", " + to_string( glm::length( simulation::Environment.wind() ), 1 ) + " m/s";
    textline += "\nAir temperature: " + to_string( Global.AirTemperature, 1 ) + " deg C";

    Output.emplace_back( textline, Global.UITextColor );
}

void
debug_panel::update_section_eventqueue( std::vector<text_line> &Output ) {

	std::string textline;

    // current event queue
    auto const time { Timer::GetTime() };
    auto const *event { simulation::Events.begin() };
    auto const searchfilter { std::string( m_eventsearch.data() ) };

	Output.emplace_back( "Delay:   Event:", Global.UITextColor );

	while( event != nullptr
	    && Output.size() < 30 ) {

		if( false == event->m_ignored
		 && false == event->m_passive
		 && ( false == m_eventqueueactivevehicleonly
		   || event->m_activator == m_input.vehicle ) ) {

            auto const label { event->m_name + ( event->m_activator ? " (by: " + event->m_activator->asName + ")" : "" ) };

            if( false == searchfilter.empty()
             && false == contains(label, searchfilter) ) {
                event = event->m_next;
                continue;
            }

            auto const delay { "   " + to_string( std::max( 0.0, event->m_launchtime - time ), 1 ) };
            textline =
                delay.substr( delay.length() - 6 )
                + "   "
                + label + ( event->m_sibling ? " (joint event)" : "" );

            Output.emplace_back( textline, Global.UITextColor );
        }
        event = event->m_next;
    }
    if( Output.size() == 1 ) {
        // event queue can be empty either because no event got through active filters, or because it is genuinely empty
        Output.front().data = simulation::Events.begin() == nullptr ? "(no queued events)" : "(no matching events)";
    }
}

void
debug_panel::update_section_powergrid( std::vector<text_line> &Output ) {

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
debug_panel::update_section_camera( std::vector<text_line> &Output ) {

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
	    .substr( 0 + 2 * floor( fmod( 8 + ( camera.Angle.y + 0.5 * M_PI_4 ) / M_PI_4, 8 ) ), 2 );

	Output.emplace_back( textline, Global.UITextColor );
}

void
debug_panel::update_section_renderer( std::vector<text_line> &Output ) {

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

bool
debug_panel::render_section( std::string const &Header, std::vector<text_line> const &Lines ) {

	if( true == Lines.empty() ) { return false; }
	if( false == ImGui::CollapsingHeader( Header.c_str() ) ) { return false; }

    return render_section( Lines );
}

bool
debug_panel::render_section( std::vector<text_line> const &Lines ) {

    for( auto const &line : Lines ) {
        ImGui::PushStyleColor( ImGuiCol_Text, { line.color.r, line.color.g, line.color.b, line.color.a } );
        ImGui::TextUnformatted( line.data.c_str() );
        ImGui::PopStyleColor();
//        ImGui::TextColored( ImVec4( line.color.r, line.color.g, line.color.b, line.color.a ), line.data.c_str() );
	}
	return true;
}

bool debug_panel::render_section_developer()
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

bool
debug_panel::render_section_settings() {

    if( false == ImGui::CollapsingHeader( "Settings" ) ) { return false; }

    ImGui::PushStyleColor( ImGuiCol_Text, { Global.UITextColor.r, Global.UITextColor.g, Global.UITextColor.b, Global.UITextColor.a } );
    ImGui::TextUnformatted( "Graphics" );
    ImGui::PopStyleColor();
    // reflection fidelity
    ImGui::SliderInt( ( std::to_string( Global.reflectiontune.fidelity ) + "###reflectionfidelity" ).c_str(), &Global.reflectiontune.fidelity, 0, 2, "Reflection fidelity" );
    ImGui::SliderInt( ( std::to_string( Global.gfx_shadow_rank_cutoff ) + "###shadowrankcutoff" ).c_str(), &Global.gfx_shadow_rank_cutoff, 1, 3, "Shadow ranks" );
    if( ImGui::SliderFloat( ( to_string( std::abs( Global.gfx_shadow_angle_min ), 2 ) + "###shadowanglecutoff" ).c_str(), &Global.gfx_shadow_angle_min, -1.0, -0.2, "Shadow angle cutoff" ) ) {
        Global.gfx_shadow_angle_min = quantize( Global.gfx_shadow_angle_min, 0.05f );
    };
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
            command_relay relay;
            relay.post(user_command::radiovolumeset, val, 0.0, GLFW_PRESS, 0);
        }
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
    auto const panelname { (
        title.empty() ?
		    m_name :
            title )
		+ "###" + m_name };
    if( true == ImGui::Begin( panelname.c_str(), &is_open, flags ) ) {
        // header section
        for( auto const &line : text_lines ) {
            ImGui::TextWrapped( "%s", line.data.c_str() );
        }
    }
    ImGui::End();
}

//---------------------------------------------------------------------------
// hudcfg: HUD layout configuration, part of the common config file (eu07.ini)
// the parameters are read by the common config parser into Global.gui_hud
//---------------------------------------------------------------------------

namespace hudcfg {

// unified row height for the main panel data zone: evenly spaced rows (bars + controller strips),
// shared by the row planner (main_panel_height) and the renderer
float constexpr ROW_H { 32.0f };

// registry of every displayable HUD datum. stable ids double as the extension point:
// a future developer adds a row here and the item appear in the customisation window.
// group: 0 = main (bottom-right) panel, 1 = top signal strip.
// default_on: false means "hidden by default" - visible only when checked in Custom mode
static constexpr hud_item items[] = {
    // id,          display name,         group, default
    { "speed",     "Speed",                  2, true },
    { "direction", "Direction",              2, true },
    { "grade",     "Grade",                  2, true },
    { "power",     "Power bar",              0, true },  // diesel rpm, or traction current
    { "brakepipe", "Brake pipe",             0, true },
    { "mainres",   "Main reservoir",         0, false }, // supply circuit pressure, off by default
    { "trainbrake","Train brake",            0, true },
    { "edbrake",   "Dynamic brake",          0, true },  // only with a dedicated ED handle
    { "indbrake",  "Independent brake",      0, true },
    { "brakecyl",  "Brake cylinder",         0, true },
    { "brakecyl_last", "Last car brake cyl",   0, true },  // long-train tail cylinder
    { "brakecyl_min",  "Worst brake cyl",      0, true },  // whole-consist worst brake
    { "notch",     "Notch",                  0, true },
    { "shunt",     "Shunt",                  0, true },
    { "slip",      "Wheel slip",             1, false }, // warning banner in the top strip
    { "speedctrl", "Cruise control",         0, false }, // tempomat: selected speed + activity
    { "lamps",     "Signal lamps",         1, false }, // preview: off by default, on in Custom
    { "limit",     "Current limit",        1, true },  // large current-limit digits
    { "nextsiglimit", "Next signal limit", 1, true },  // next signal speed limit preview
    { "sigdist",   "Signal distance",        1, true },
    { "nextlimit", "Next limit",             1, true },
    { "passenger", "Passenger exchange",     1, true },
    { "alarm",     "CA / SHP alarm",         1, true },
    { "doors",     "Doors",                  1, false }, // door state line (EMU/DMU)
};
static int const ITEM_COUNT { static_cast<int>( sizeof( items ) / sizeof( items[ 0 ] ) ) };
int const group_main = 0;
int const group_strip = 1;
int const group_speed = 2;

static ui_panel *g_panel { nullptr };
static ui_panel *g_signalpanel { nullptr };
static ui_panel *g_speedpanel { nullptr };
static ui_panel *g_customwindow { nullptr };
static bool g_visible { true };
static bool g_dragging { false };
static bool g_panelon { true };  // main panel group switch (Custom mode only)
static bool g_stripon { true };  // top strip group switch (Custom mode only)
static bool g_speedon { true };  // speed panel group switch (Custom mode only)
static int g_mode { standard };
static float g_toast { 0.0f };
// custom-mode checkbox state, indexed like items[]
static std::array<bool, ITEM_COUNT> g_custom_on;
static void apply_visibility();
static void sync_custom_items( std::string const &Csv );

// legacy ini values (3-mode era): 0=Minimal 1=Standard 2=Custom 3=Off -> 0=Standard 1=Custom 2=Off
static int normalize_hud_mode( int const Old )
{
    if ( Old <= 1 ) return standard;
    if ( Old == 2 ) return custom;
    return off;
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

void set_mode( int const Mode )
{
    g_mode = std::clamp<int>( Mode, standard, off );
    Global.gui_hud.mode = g_mode; // persisted in eu07.ini
    // the Off mode hides the overlay; any other mode shows it again
    set_visible( g_mode != off );
    // entering Custom mode opens the customisation window (it can be closed with X afterwards);
    // entering Off closes it as well - the HUD is off, the setup window is gone with it
    if ( g_customwindow != nullptr )
        g_customwindow->is_open = ( g_mode == custom );
    g_toast = 1.5f;
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

bool strip_group()
{
    return g_stripon;
}

void set_panel_group( bool const On )
{
    g_panelon = On;
    apply_visibility();
}

void set_strip_group( bool const On )
{
    g_stripon = On;
    apply_visibility();
}

bool speed_group()
{
    return g_speedon;
}

void set_speed_group( bool const On )
{
    g_speedon = On;
    apply_visibility();
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

hud_item const *item( char const *Id )
{
    int const i { item_index( Id ) };
    return i >= 0 ? &items[ i ] : nullptr;
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
    // rebuild the persisted csv: list of ON items (empty = nothing checked); ini write happens on save
    std::string csv;
    for ( int j = 0; j < ITEM_COUNT; ++j )
        if ( g_custom_on[ j ] )
        {
            if ( !csv.empty() )
                csv += ",";
            csv += items[ j ].id;
        }
    Global.gui_hud.custom_items = csv;
}

bool item_visible( char const *Id )
{
    if ( mode() == off )
        return false;
    if ( mode() == standard )
    {
        // Standard = the default set (items with default_on); extras only after user checks them
        int const i { item_index( Id ) };
        return i >= 0 && items[ i ].default_on;
    }
    return custom_checked( Id ); // custom mode
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
    if ( std::string_view( Id ) == "power" )
        return true;
    return true;
}

int main_panel_height()
{
    // shared row plan with render_contents(): bars + controller strips only - the speed
    // zone (digits/arrows/grade) lives in its own split-out speed panel now
    int rows { 6 }; // top padding
    // instrument bars
    for ( char const *const id : { "power", "brakepipe", "mainres", "trainbrake", "indbrake", "brakecyl", "speedctrl", "brakecyl_last", "brakecyl_min" } )
        if ( item_visible( id ) )
            rows += static_cast<int>( ROW_H );
    // controller strips (notch / ED / shunt)
    for ( char const *const id : { "notch", "edbrake", "shunt" } )
        if ( item_visible( id ) )
            rows += static_cast<int>( ROW_H );
    int const h { std::max( 60, rows + 12 ) }; // 12 = bottom padding
    return static_cast<int>( std::min<double>( h, Global.fb_size.y * 0.8 ) );
}

static void sync_custom_items( std::string const &Csv )
{
    // default state = defaults; then apply the persisted csv list (ids of ON items)
    for ( int i = 0; i < ITEM_COUNT; ++i )
        g_custom_on[ i ] = items[ i ].default_on;
    for ( auto const &idstr : Split( Csv, ',' ) )
    {
        int const i { item_index( idstr.c_str() ) };
        if ( i >= 0 )
            g_custom_on[ i ] = true;
    }
}

settings const &get()
{
    // the gui.hud.enabled switch is read from the existing config file (eu07.ini) by the
    // common config parser; the remaining values are in-code defaults, updated at runtime
    return Global.gui_hud;
}

void set_panels( ui_panel *Panel, ui_panel *SignalPanel, ui_panel *SpeedPanel )
{
    g_panel = Panel;
    g_signalpanel = SignalPanel;
    g_speedpanel = SpeedPanel;
    g_visible = Global.gui_hud.enabled;
    // fresh installs start with the HUD OFF (F1 brings it up); once the player has chosen
    // a mode it is remembered in the ini (gui.hud.mode) and honoured on the next start
    g_mode = Global.gui_hud.mode_saved ? normalize_hud_mode( Global.gui_hud.mode ) : off;
    g_panelon = Global.gui_hud.panel;
    g_stripon = Global.gui_hud.strip;
    g_speedon = Global.gui_hud.speed_panel;
    sync_custom_items( Global.gui_hud.custom_items );
    apply_visibility();
}

void set_visible( bool const Show )
{
    g_visible = Show;
    apply_visibility();
}

bool visible()
{
    return g_visible;
}

void toggle()
{
    set_visible( !g_visible );
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
    // group switches only apply in Custom mode; Standard always shows all panels,
    // Off hides everything (g_visible false)
    if (g_panel != nullptr)
        g_panel->is_open = g_visible && mode() != off && ( mode() != custom || g_panelon );
    if (g_signalpanel != nullptr)
        g_signalpanel->is_open = g_visible && mode() != off && ( mode() != custom || g_stripon );
    if (g_speedpanel != nullptr)
        g_speedpanel->is_open = g_visible && mode() != off && ( mode() != custom || g_speedon );
}

void set_panel_pos( int const X, int const Y )
{
    // keep the free-position overrides in the common settings (no separate config file)
    Global.gui_hud.panel_x = X;
    Global.gui_hud.panel_y = Y;
}

void set_signal_pos( int const X, int const Y )
{
    Global.gui_hud.sig_x = X;
    Global.gui_hud.sig_y = Y;
}

void set_speed_pos( int const X, int const Y )
{
    Global.gui_hud.speed_x = X;
    Global.gui_hud.speed_y = Y;
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

    if ( pos.x != -1 && pos.y != -1 )
        ImGui::SetNextWindowPos( ImVec2S( pos.x, pos.y ), ImGuiCond_Always );

    // auto-sized to content until the player grabs the resize grip (the bottom-right
    // triangle, matching the other windows); then the size becomes manual
    ImVec2 const autosize { ImVec2S( Global.gui_hud.panel_width, hudcfg::main_panel_height() ) };
    if ( !m_manual_size )
        ImGui::SetNextWindowSize( autosize, ImGuiCond_Always );
    // floor: never smaller than the content (speed zone must stay visible)
    ImGui::SetNextWindowSizeConstraints(
        ImVec2( 280.0f, static_cast<float>( hudcfg::main_panel_height() ) ),
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
    size = { cfg.panel_width, hudcfg::main_panel_height() };
    hudcfg::update_toast( ImGui::GetIO().DeltaTime );
    if ( hudcfg::dragging() )
    {
        pos = { -1, -1 }; // suspend the anchor while the user drags
    }
    else if ( cfg.panel_x < 0 || cfg.panel_y < 0 )
        // default: right edge, just above the speed panel (which hugs the bottom-right corner)
        pos = { fb.x - size.x - cfg.margin,
                fb.y - 205.0f - cfg.margin - size.y - 8.0f };
    else
        // no position protection: exactly where the player left it (may be off-screen)
        pos = { cfg.panel_x, cfg.panel_y };
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
        static bool dragging { false };
        auto const wsize { ImGui::GetWindowSize() };
        ImGui::SetCursorPos( ImVec2( 0.0f, 0.0f ) );
        ImGui::InvisibleButton( "##hud_drag", wsize );
        if ( ImGui::IsItemActive() )
        {
            dragging = true;
            hudcfg::set_dragging( true );
            auto const delta { ImGui::GetIO().MouseDelta };
            winpos = ImVec2( winpos.x + delta.x, winpos.y + delta.y );
            ImGui::SetWindowPos( winpos );
        }
        if ( dragging && ImGui::IsMouseReleased( 0 ) )
        {
            dragging = false;
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
    float const cur1 { train->fHCurrent[1] };
    float const cur2 { train->fHCurrent[2] };
    // branch ammeters only exist when the current scheme has parallel circuits (Bn>=2),
    // exactly like the sim's own ShowCurrentP: Bn < AmpN returns 0
    bool const dc_series { enginetype == TEngineType::ElectricSeriesMotor };
    bool const parallel_branches { dc_series && ctrlv->MainCtrlActualPos > 0 && ctrlv->MainCtrlActualPos <= ctrlv->MainCtrlPosNo && ctrlv->RList[ctrlv->MainCtrlActualPos].Bn >= 2 };
    bool const two_groups { is_electric && ( parallel_branches || ( !dc_series && ( std::abs( cur1 ) > 0.01f || std::abs( cur2 ) > 0.01f ) ) ) };
    // electro-dynamic brake: current flows back (shown negative, red), brake handle becomes ED position.
    // composite (integrated drive+brake) handle vehicles are excluded: their wheel already shows
    // drive/brake bidirectionally, and what they do when pulling back is not ED braking
    bool const composite_handle { ctrlv->EIMCtrlType > 0 || ctrlv->UniCtrlIntegratedBrakeCtrl };
    bool const dynbrake { is_electric && !composite_handle && mover->DynamicBrakeFlag };
    auto const current_col { dynbrake ? IM_COL32( 255, 70, 45, 240 ) : 0u };

    auto const bar_col = []( float const frac ) -> ImU32 {
        if ( frac >= 0.95f ) return IM_COL32( 255, 70, 45, 240 );
        if ( frac >= 0.80f ) return IM_COL32( 255, 200, 40, 240 );
        return IM_COL32( 130, 220, 140, 240 );
    };
    char buf[ 64 ];
    // fixed content geometry: bars keep their native length; a wider window centres the
    // content (the grip is for extra headroom, NOT for stretching the bars)
    float const winW { ImGui::GetWindowSize().x };
    float const cx0 { std::max( 0.0f, ( winW - 396.0f ) * 0.5f ) }; // content offset (centred)
    float const barW { 230.0f };
    auto const draw_bar = [&]( float const Y, char const *Label, float const Value, float const Vmax, char const *ValueText, ImU32 const ColOverride = 0 )
    {
        float const x0 { winpos.x + cx0 + 8.0f };
        float const w { barW };
        float const frac { Vmax > 0.0f ? std::clamp( Value / Vmax, 0.0f, 1.0f ) : 0.0f };
        dl->AddText( ui_layer::font_default, 15.0f, ImVec2( x0, Y ), IM_COL32( 255, 255, 255, 190 ), Label );
        dl->AddRectFilled( ImVec2( x0 + 70.0f, Y + 20.0f ), ImVec2( x0 + 70.0f + w, Y + 30.0f ), IM_COL32( 255, 255, 255, 30 ), 4.0f );
        if ( frac > 0.005f )
            dl->AddRectFilled( ImVec2( x0 + 70.0f, Y + 20.0f ), ImVec2( x0 + 70.0f + w * frac, Y + 30.0f ), ColOverride != 0 ? ColOverride : bar_col( frac ), 4.0f );
        dl->AddText( ui_layer::font_default, 15.0f, ImVec2( x0 + 70.0f + w + 10.0f, Y ), IM_COL32( 255, 255, 255, 240 ), ValueText );
    };

    float barY { winpos.y + 6.0f };
    if ( hudcfg::item_visible( "power" ) )
    {
    if ( is_diesel )
    {
        // real diesel instrument: engine revolutions (RPM), see EngineRPMRatio/EngineMaxRPM
        double const rpmratio { std::clamp( mover->EngineRPMRatio(), 0.0, 1.0 ) };
        double const rpm { mover->EngineMaxRPM() * rpmratio };
        std::snprintf( buf, sizeof( buf ), STR_C("%.0f rpm"), rpm );
        draw_bar( barY, STR_C("Engine RPM"), static_cast<float>( rpmratio ), 1.0f, buf );
        barY += hudcfg::ROW_H;
    }
    else if ( is_electric )
    {
        if ( two_groups )
        {
            std::snprintf( buf, sizeof( buf ), STR_C("%.0f A"), dynbrake ? -std::abs( cur1 ) : std::abs( cur1 ) );
            draw_bar( barY, STR_C("Current 1"), std::abs( cur1 ), 500.0f, buf, current_col );
            barY += hudcfg::ROW_H;
            std::snprintf( buf, sizeof( buf ), STR_C("%.0f A"), dynbrake ? -std::abs( cur2 ) : std::abs( cur2 ) );
            draw_bar( barY, STR_C("Current 2"), std::abs( cur2 ), 500.0f, buf, current_col );
            barY += hudcfg::ROW_H;
        }
        else
        {
            std::snprintf( buf, sizeof( buf ), STR_C("%.0f A"), dynbrake ? -std::abs( train->fHCurrent[0] ) : std::abs( train->fHCurrent[0] ) );
            draw_bar( barY, STR_C("Current"), std::abs( train->fHCurrent[0] ), 800.0f, buf, current_col );
            barY += hudcfg::ROW_H;
        }
    }
    } // item power

    // brake pipe pressure (SPKS), labelled as the pipe itself
    if ( hudcfg::item_visible( "brakepipe" ) )
    {
    float const pipefrac { std::clamp( static_cast<float>( mover->PipePress ) / 6.0f, 0.0f, 1.0f ) };
    std::snprintf( buf, sizeof( buf ), "%.1f bar", mover->PipePress );
    ImU32 const pipecol { mover->PipePress < 3.0 ? IM_COL32( 255, 70, 45, 240 ) : ( mover->PipePress < 4.5 ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 130, 220, 140, 240 ) ) };
    draw_bar( barY, STR_C("Brake pipe"), pipefrac, 1.0f, buf, pipecol );
    barY += hudcfg::ROW_H;
    } // item brakepipe

    // main reservoir / supply circuit pressure (off by default; low supply = brake risk)
    if ( hudcfg::item_visible( "mainres" ) )
    {
        float const mfrac { std::clamp( static_cast<float>( mover->Compressor ) / 10.0f, 0.0f, 1.0f ) };
        std::snprintf( buf, sizeof( buf ), "%.1f bar", mover->Compressor );
        ImU32 const mcol { mover->Compressor < 6.0 ? IM_COL32( 255, 70, 45, 240 ) : ( mover->Compressor < 7.5 ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 130, 220, 140, 240 ) ) };
        draw_bar( barY, STR_C("Main reservoir"), mfrac, 1.0f, buf, mcol );
        barY += hudcfg::ROW_H;
    }

    // main brake handle position (same source as the game's driving aid "basicbraking" value);
    // ALWAYS shown: the electro-dynamic brake is its own persistent strip below the notch row
    if ( hudcfg::item_visible( "trainbrake" ) )
    {
    {
        float const brakcfrac { mover->BrakeCtrlPosNo > 0 ? std::clamp( static_cast<float>( std::max( 0.0, mover->fBrakeCtrlPos ) ) / mover->BrakeCtrlPosNo, 0.0f, 1.0f ) : 0.0f };
        std::snprintf( buf, sizeof( buf ), "%.1f", mover->fBrakeCtrlPos );
        draw_bar( barY, STR_C("train brake"), brakcfrac, 1.0f, buf );
        barY += hudcfg::ROW_H;
    }
    } // item trainbrake

    if ( hudcfg::item_visible( "indbrake" ) )
    {
    float const indfrac { static_cast<float>( std::clamp( mover->LocalBrakePosA, 0.0, 1.0 ) ) };
    std::snprintf( buf, sizeof( buf ), "%.0f/10", std::floor( indfrac * 10.0f ) );
    draw_bar( barY, STR_C("independent brake"), indfrac, 1.0f, buf );
    barY += hudcfg::ROW_H;
    } // item indbrake

    // brake cylinder pressure (cab gauge "CYLINDER HAMULCOWY"), real SPKS value
    if ( hudcfg::item_visible( "brakecyl" ) )
    {
    float const cylfrac { std::clamp( static_cast<float>( mover->BrakePress ) / 6.0f, 0.0f, 1.0f ) };
    std::snprintf( buf, sizeof( buf ), "%.2f bar", mover->BrakePress );
    ImU32 const cylcol { mover->BrakePress < 0.5f ? IM_COL32( 130, 220, 140, 240 ) : ( mover->BrakePress < 3.5f ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 255, 70, 45, 240 ) ) };
    draw_bar( barY, STR_C("Brake cylinder"), cylfrac, 1.0f, buf, cylcol );
    barY += hudcfg::ROW_H;
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
            float const tfrac { std::clamp( static_cast<float>( tailpress ) / 6.0f, 0.0f, 1.0f ) };
            std::snprintf( buf, sizeof( buf ), "%.2f bar", tailpress );
            ImU32 const tcol { tailpress < 0.5f ? IM_COL32( 130, 220, 140, 240 ) : ( tailpress < 3.5f ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 255, 70, 45, 240 ) ) };
            draw_bar( barY, STR_C("Last car brake cyl"), tfrac, 1.0f, buf, tcol );
            barY += hudcfg::ROW_H;
        }
        if ( wantmin && consistcount > 1 )
        {
            // colour is INVERTED vs. the local cylinder bar: the lowest pressure in the
            // consist is the danger case (a car without braking), so low = red
            float const mfrac { std::clamp( static_cast<float>( minpress ) / 6.0f, 0.0f, 1.0f ) };
            std::snprintf( buf, sizeof( buf ), "%.2f bar", minpress );
            ImU32 const mcol { minpress < 0.3 ? IM_COL32( 255, 70, 45, 240 ) : ( minpress < 3.5 ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 130, 220, 140, 240 ) ) };
            draw_bar( barY, STR_C("Worst brake cyl"), mfrac, 1.0f, buf, mcol );
            barY += hudcfg::ROW_H;
        }
    }

    // cruise control (tempomat): plain numeric row, no bar - it is a state, not a gauge.
    // green = active, grey = speed selected but inactive, OFF = nothing set
    if ( hudcfg::item_visible( "speedctrl" ) && mover->SpeedCtrl )
    {
        if ( mover->SpeedCtrlValue > 0.5 )
        {
            std::snprintf( buf, sizeof( buf ), "%s: %.0f km/h", STR_C("Cruise control"), mover->SpeedCtrlValue );
            dl->AddText( ui_layer::font_default, 15.0f, ImVec2( winpos.x + 8.0f, barY ),
                mover->SpeedCtrlUnit.IsActive ? IM_COL32( 90, 220, 120, 245 ) : IM_COL32( 190, 200, 215, 220 ), buf );
        }
        else
            dl->AddText( ui_layer::font_default, 15.0f, ImVec2( winpos.x + 8.0f, barY ), IM_COL32( 140, 150, 165, 180 ), STR_C("Cruise control: OFF") );
        barY += hudcfg::ROW_H;
    }
    // wheel slip warning moved to the top strip's banner area (same place as the CA/SHP alarm)

    // the sim uses two controller models; their position SEMANTICS differ:
    //  - MCPN-defined (EU07 etc.): handle position = MainCtrlPos, max = MainCtrlPosNo
    //  - RList-defined (EN57-class): state = MainCtrlActualPos (RList index), max = RlistSize
    // MainCtrlActualPos is an internal scheme pointer and must NOT be mixed in for MCPN vehicles.
    bool const mcpn_ctrl { ctrlv->MainCtrlPosNo > 0 };
    int const notchpos { mcpn_ctrl ? ctrlv->MainCtrlPos : ctrlv->MainCtrlActualPos };
    int const notchmax { mcpn_ctrl ? ctrlv->MainCtrlPosNo : ctrlv->RlistSize };
    bool const eim_throttle { ctrlv->EIMCtrlType > 0 };
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
                if ( ctrlv->ScndCtrlPos > 0 ) std::snprintf( zone, sizeof( zone ), "%s", STR_C("Shunt") );
                else
                {
                    auto const &scheme { ctrlv->RList[ctrlv->MainCtrlActualPos] };
                    if ( scheme.ScndAct > 0 ) std::snprintf( zone, sizeof( zone ), "%s", STR_C("Shunt") );
                    else if ( scheme.Bn > 1 ) std::snprintf( zone, sizeof( zone ), "%s", STR_C("Parallel") );
                    else if ( scheme.Mn > 1 ) std::snprintf( zone, sizeof( zone ), "%s", STR_C("Series") );
                    else std::snprintf( zone, sizeof( zone ), "%s", STR_C("Traction") );
                }
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
    float nstripy { barY };
    float const sx0 { winpos.x + cx0 + 78.0f };
    float const sw { barW };
    char notchvalue[ 64 ]; // shared by the notch strip and the shunt strip below
    if ( hudcfg::item_visible( "notch" ) )
    {
    dl->AddText( ui_layer::font_default, 15.0f, ImVec2( winpos.x + 8.0f, nstripy ), IM_COL32( 255, 255, 255, 190 ), STR_C("Notch") );
    // value text: position + zone name, e.g. "9/16 并联"
    std::snprintf( notchvalue, sizeof( notchvalue ), "%s%s%s", notchtext.c_str(), notchtext.empty() ? "" : " ", zone );
    dl->AddText( ui_layer::font_default, 15.0f, ImVec2( sx0 + sw + 10.0f, nstripy ), IM_COL32( 255, 255, 255, 240 ), notchvalue );
    dl->AddRectFilled( ImVec2( sx0, nstripy + 20.0f ), ImVec2( sx0 + sw, nstripy + 30.0f ), IM_COL32( 255, 255, 255, 30 ), 4.0f );
    if ( eim_throttle )
    {
        // center-zero bidirectional gauge: right = drive (green), left = brake (amber)
        float const cx { sx0 + sw * 0.5f };
        dl->AddRectFilled( ImVec2( cx - 1.0f, nstripy + 19.0f ), ImVec2( cx + 1.0f, nstripy + 31.0f ), IM_COL32( 255, 255, 255, 70 ) );
        if ( notchfrac > 0.005f )
        {
            if ( ctrlv->eimic_real < 0.0 )
                dl->AddRectFilled( ImVec2( cx - sw * 0.5f * notchfrac, nstripy + 20.0f ), ImVec2( cx, nstripy + 30.0f ), IM_COL32( 255, 190, 40, 230 ), 4.0f );
            else
                dl->AddRectFilled( ImVec2( cx, nstripy + 20.0f ), ImVec2( cx + sw * 0.5f * notchfrac, nstripy + 30.0f ), IM_COL32( 90, 220, 120, 230 ), 4.0f );
        }
    }
    else if ( notchfrac > 0.005f )
        dl->AddRectFilled( ImVec2( sx0, nstripy + 20.0f ), ImVec2( sx0 + sw * notchfrac, nstripy + 30.0f ), IM_COL32( 90, 160, 255, 230 ), 4.0f );
    nstripy += hudcfg::ROW_H; // advance for the strips below
    } // item notch

    // --- persistent strips under the notch row ------------------------------------
    // electro-dynamic brake strip: shown ONLY when the vehicle has a dedicated ED handle
    // (SplitEDPneumaticBrake=Yes, e.g. Vectron with its DBPN lever); automatic/switch/passive
    // ED types have no separate ED handle, so they get no strip. EP09 (104E) stays excluded
    // regardless of configuration (its automatic-ED data is unreliable per vehicle devs)
    auto const vehname { ToLower( mover->Name ) };
    bool const is_ep09 { vehname.compare( 0, 4, "ep09" ) == 0 || vehname.compare( 0, 4, "104e" ) == 0 };
    bool const has_ed { mover->SplitEDPneumaticBrake && !is_ep09 };
    bool const has_shunt { ctrlv->ScndCtrlPosNo > 0 && dc_series };
    float nstripy2 { nstripy }; // starts below the notch row (which advanced it if visible)
    if ( has_ed && hudcfg::item_visible( "edbrake" ) )
    {
        // dedicated ED handle position (0..1 → notch count), same style as the shunt strip
        char dbval[ 32 ];
        float const dbno { mover->DynamicBrakeCtrlPosNo > 0 ? static_cast<float>( mover->DynamicBrakeCtrlPosNo ) : 10.0f };
        float const dbfrac { static_cast<float>( mover->DynamicBrakeCtrlPos ) };
        std::snprintf( dbval, sizeof( dbval ), "%.0f/%.0f", std::floor( dbfrac * dbno + 0.5f ), dbno );
        dl->AddText( ui_layer::font_default, 15.0f, ImVec2( winpos.x + 8.0f, nstripy2 ), IM_COL32( 255, 255, 255, 190 ), STR_C("Dynamic brake") );
        dl->AddText( ui_layer::font_default, 15.0f, ImVec2( sx0 + sw + 10.0f, nstripy2 ), IM_COL32( 255, 255, 255, 240 ), dbval );
        dl->AddRectFilled( ImVec2( sx0, nstripy2 + 20.0f ), ImVec2( sx0 + sw, nstripy2 + 30.0f ), IM_COL32( 255, 255, 255, 30 ), 4.0f );
        if ( dbfrac > 0.005f )
            dl->AddRectFilled( ImVec2( sx0, nstripy2 + 20.0f ), ImVec2( sx0 + sw * dbfrac, nstripy2 + 30.0f ), IM_COL32( 110, 220, 250, 230 ), 4.0f );
        nstripy2 += hudcfg::ROW_H; // make room for the shunt strip below
    }
    // field-weakening (shunt) controller row; only DC electrics have real field weakening
    // (on induction vehicles SCPN>0 means a power step switch, not a shunt controller)
    if ( has_shunt && hudcfg::item_visible( "shunt" ) )
    {
        float const shfrac { std::clamp( static_cast<float>( ctrlv->ScndCtrlPos ) / ctrlv->ScndCtrlPosNo, 0.0f, 1.0f ) };
        dl->AddText( ui_layer::font_default, 15.0f, ImVec2( winpos.x + 8.0f, nstripy2 ), IM_COL32( 255, 255, 255, 190 ), STR_C("Shunt") );
        std::snprintf( notchvalue, sizeof( notchvalue ), "%d/%d", ctrlv->ScndCtrlPos, ctrlv->ScndCtrlPosNo );
        dl->AddText( ui_layer::font_default, 15.0f, ImVec2( sx0 + sw + 10.0f, nstripy2 ), IM_COL32( 255, 255, 255, 240 ), notchvalue );
        dl->AddRectFilled( ImVec2( sx0, nstripy2 + 20.0f ), ImVec2( sx0 + sw, nstripy2 + 30.0f ), IM_COL32( 255, 255, 255, 30 ), 4.0f );
        if ( shfrac > 0.005f )
            dl->AddRectFilled( ImVec2( sx0, nstripy2 + 20.0f ), ImVec2( sx0 + sw * shfrac, nstripy2 + 30.0f ), IM_COL32( 180, 130, 255, 230 ), 4.0f );
    }

    // speed digits / direction arrows / gradient triangles moved to hud_speed_panel

    // --- mode-name feedback (F1 / menu) ------------------------------------------------
    if ( hudcfg::toast() > 0.0f )
    {
        // translation keys: "HUD: " and the mode name; falls back to the literal until the
        // assets translation update lands (per jakubg1 that is handled later)
        std::string const toastmsg { std::string( STR_C("HUD: ") ) + STR_C( hudcfg::mode_name( hudcfg::mode() ) ) };
        int const toastalpha { static_cast<int>( std::clamp( hudcfg::toast() / 0.5f, 0.0f, 1.0f ) * 255.0f ) };
        dl->AddText( ui_layer::font_default, 17.0f, ImVec2( winpos.x + 12.0f, winpos.y + 12.0f ), IM_COL32( 255, 255, 255, toastalpha ), toastmsg.c_str() );
    }
}

//---------------------------------------------------------------------------
// hud_speed_panel: split-out speed readout (digits + direction arrows + grade)
// kept at the position where the speed zone used to sit, below the data panel
//---------------------------------------------------------------------------

void
hud_speed_panel::render()
{
    if ( !is_open )
        return;

    int flags { window_flags };
    if ( no_title_bar )
        flags |= ImGuiWindowFlags_NoTitleBar;

    if ( pos.x != -1 && pos.y != -1 )
        ImGui::SetNextWindowPos( ImVec2S( pos.x, pos.y ), ImGuiCond_Always );

    // auto-sized (320x205, matching the top signal strip spec) until the player grabs
    // the resize grip; then the size is manual and the contents scale with the window
    ImVec2 const autosize { ImVec2( 320.0f, 205.0f ) };
    if ( !m_manual_size )
        ImGui::SetNextWindowSize( autosize, ImGuiCond_Always );
    ImGui::SetNextWindowSizeConstraints( ImVec2( 200.0f, 140.0f ), ImVec2( 1.0e6f, 1.0e6f ) );

    auto const panelname { ( title.empty() ? m_name : title ) + "###" + m_name };
    if ( ImGui::Begin( panelname.c_str(), &is_open, flags ) )
    {
        ImVec2 const cur { ImGui::GetWindowSize() };
        if ( std::abs( cur.x - autosize.x ) > 0.5f || std::abs( cur.y - autosize.y ) > 0.5f )
            m_manual_size = true;
        render_contents();
        popups.remove_if( []( std::unique_ptr<ui::popup> const &popup ) { return popup->render(); } );
    }
    ImGui::End();
}

void
hud_speed_panel::update()
{
    auto const &fb { Global.fb_size };
    auto const &cfg { hudcfg::get() };
    // anchor reference: the default (auto) size; actual size follows the grip once resized
    size = { 320, 205 };
    if ( hudcfg::dragging() )
    {
        pos = { -1, -1 }; // suspend the anchor while the user drags
    }
    else if ( cfg.speed_x < 0 || cfg.speed_y < 0 )
        // default: bottom-right corner of the screen
        pos = { fb.x - size.x - cfg.margin, fb.y - size.y - cfg.margin };
    else
        // no position protection: exactly where the player left it (may be off-screen)
        pos = { cfg.speed_x, cfg.speed_y };
}

void
hud_speed_panel::render_contents()
{
    auto winpos { ImGui::GetWindowPos() };
    float const dt { ImGui::GetIO().DeltaTime };
    float const now { static_cast<float>( ImGui::GetTime() ) };
    auto *dl { ImGui::GetWindowDrawList() };

    // draggable module: the whole window is a drag zone (position stored in the common settings)
    {
        static bool dragging { false };
        auto const wsize { ImGui::GetWindowSize() };
        ImGui::SetCursorPos( ImVec2( 0.0f, 0.0f ) );
        ImGui::InvisibleButton( "##hud_speed_drag", wsize );
        if ( ImGui::IsItemActive() )
        {
            dragging = true;
            hudcfg::set_dragging( true );
            auto const delta { ImGui::GetIO().MouseDelta };
            winpos = ImVec2( winpos.x + delta.x, winpos.y + delta.y );
            ImGui::SetWindowPos( winpos );
        }
        if ( dragging && ImGui::IsMouseReleased( 0 ) )
        {
            dragging = false;
            hudcfg::set_dragging( false );
            winpos = ImGui::GetWindowPos();
            hudcfg::set_speed_pos( static_cast<int>( winpos.x ), static_cast<int>( winpos.y ) );
        }
    }

    auto const *train { simulation::Train };
    auto const *controlled { train ? train->Dynamic() : nullptr };
    auto const *mover { controlled ? controlled->MoverParameters : nullptr };
    if ( controlled == nullptr || mover == nullptr )
        return;

    // data (same conventions as the old speed zone / driving aid)
    float const speed_kmh { static_cast<float>( std::abs( controlled->GetVelocity() ) ) };
    auto const *owner { controlled->ctOwner != nullptr ? controlled->ctOwner : controlled->Mechanik };
    int const speedlimit { owner ? static_cast<int>( owner->VelDesired ) : 0 };
    auto const reverser { ( mover->DirActive > 0 ? 1 : -1 ) };
    double const grade_pm { controlled->VectorFront().y * 1000.0 * ( controlled->DirectionGet() == reverser ? 1.0 : -1.0 ) * reverser };

    // speed colour: limit thresholds only (yellow at +1, red at +4); pulse on overspeed+4
    bool const overspeed_flash { speedlimit > 0 && speed_kmh >= speedlimit + 4.0f };
    bool const overspeed_warn { speedlimit > 0 && speed_kmh >= speedlimit + 1.0f };
    glm::vec4 target;
    bool danger { false };
    if ( overspeed_flash ) { target = { 1.00f, 0.15f, 0.08f, 1.00f }; danger = true; }
    else if ( overspeed_warn ) { target = { 1.00f, 0.80f, 0.05f, 1.00f }; }
    else { target = { 0.70f, 0.88f, 1.00f, 1.00f }; }
    float const k { danger ? 12.0f : 4.0f };
    float const t { 1.0f - std::exp( -k * ( dt > 0.0f ? dt : 0.0f ) ) };
    m_speedcolor += ( target - m_speedcolor ) * t;
    float const alpha { danger ? ( 0.60f + 0.40f * std::sin( now * 12.0f ) ) : 1.0f };

    // --- big speed digits + km/h (contents scale with the window size) ------------
    ImVec2 const wsz { ImGui::GetWindowSize() };
    float const scale { std::clamp( wsz.x / 320.0f, 0.65f, 2.5f ) };
    float const pad { 12.0f * scale };
    float const fontsize { std::clamp( hudcfg::get().speed_size * 1.15f * scale, 60.0f, 220.0f ) };
    float const contentw { wsz.x };
    char buf[ 16 ];
    std::snprintf( buf, sizeof( buf ), "%03.0f", speed_kmh );
    ImFont const * const font { ui_layer::font_hud };
    ImVec2 const textsize { font->CalcTextSizeA( fontsize, std::numeric_limits<float>::max(), 0.0f, buf ) };
    ImVec2 const dpos { winpos.x + ( contentw - textsize.x ) * 0.5f, winpos.y + pad };
    if ( hudcfg::item_visible( "speed" ) )
    {
    ImU32 const col { ImGui::ColorConvertFloat4ToU32( ImVec4( m_speedcolor.x, m_speedcolor.y, m_speedcolor.z, alpha ) ) };
    dl->AddText( font, fontsize, ImVec2( dpos.x + 3.0f * scale, dpos.y + 3.0f * scale ), IM_COL32( 0, 0, 0, 150 ), buf );
    dl->AddText( font, fontsize, dpos, col, buf );
    dl->AddText( ui_layer::font_default, 15.0f * scale, ImVec2( dpos.x + textsize.x + 5.0f * scale, dpos.y + textsize.y - 6.0f * scale ), IM_COL32( 255, 255, 255, 180 ), STR_C("km/h") );

    // --- direction arrows -------------------------------------------------------
    if ( hudcfg::item_visible( "direction" ) )
    {
    auto const arrow_col { []( bool const Active ) { return Active ? IM_COL32( 255, 255, 255, 245 ) : IM_COL32( 200, 210, 220, 45 ); } };
    float const ax { winpos.x + 30.0f * scale };
    bool const fwd { mover->DirActive > 0 };
    dl->AddTriangleFilled(
        ImVec2( ax - 17.0f * scale, dpos.y + 46.0f * scale ),
        ImVec2( ax + 17.0f * scale, dpos.y + 46.0f * scale ),
        ImVec2( ax, dpos.y + 4.0f * scale ),
        arrow_col( fwd ) );
    bool const bwd { mover->DirActive < 0 };
    dl->AddTriangleFilled(
        ImVec2( ax - 17.0f * scale, dpos.y + 58.0f * scale ),
        ImVec2( ax + 17.0f * scale, dpos.y + 58.0f * scale ),
        ImVec2( ax, dpos.y + 104.0f * scale ),
        arrow_col( bwd ) );
    } // item direction
    } // item speed

    // --- gradient triangles + numeric value (hidden when flat) ----------------------
    if ( hudcfg::item_visible( "grade" ) && ( grade_pm > 2.5 || grade_pm < -2.5 ) )
    {
        ImU32 const slope_col { IM_COL32( 110, 235, 120, 240 ) };
        auto const slope_num_col = []( double const Grade ) -> int {
            double const a { std::abs( Grade ) };
            if ( a >= 35.0 ) return 0;
            if ( a >= 20.0 ) return 1;
            return 2;
        };
        int const ns { slope_num_col( grade_pm ) };
        ImU32 const snumcol { ns == 0 ? IM_COL32( 240, 80, 60, 240 ) : ( ns == 1 ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 110, 235, 120, 240 ) ) };
        char gbuf[ 32 ];
        std::snprintf( gbuf, sizeof( gbuf ), STR_C(" Grade: %.1f%%%%"), std::abs( grade_pm ) * 0.1 );
        float const sx { winpos.x + contentw - 34.0f * scale };
        if ( grade_pm > 2.5 ) // uphill
        {
            dl->AddTriangleFilled( ImVec2( sx - 16.0f * scale, dpos.y + 40.0f * scale ), ImVec2( sx + 16.0f * scale, dpos.y + 40.0f * scale ), ImVec2( sx, dpos.y + 4.0f * scale ), slope_col );
            dl->AddText( ui_layer::font_default, 11.0f * scale, ImVec2( winpos.x + contentw - 78.0f * scale, dpos.y + 44.0f * scale ), snumcol, gbuf );
        }
        else // downhill
        {
            dl->AddTriangleFilled( ImVec2( sx - 16.0f * scale, dpos.y + 52.0f * scale ), ImVec2( sx + 16.0f * scale, dpos.y + 52.0f * scale ), ImVec2( sx, dpos.y + 88.0f * scale ), slope_col );
            dl->AddText( ui_layer::font_default, 11.0f * scale, ImVec2( winpos.x + contentw - 78.0f * scale, dpos.y + 94.0f * scale ), snumcol, gbuf );
        }
    }
}

//---------------------------------------------------------------------------
// hud_signal_panel: top-of-screen signal preview + speed limit strip
//---------------------------------------------------------------------------

// semantic aspect colour (A-layer, stable fallback): what the signal-grade point means
// in lamp terms - computed from the signal type flags and the memcell value, with no
// animation jitter. used only when the model lamps are not drivable
static bool
semantic_aspect_colour( TSpeedPos const &Point, map::semaphore const *Sem, glm::vec3 &Col )
{
    int const flags { Point.iFlags };
    if ( ( flags & spShuntSemaphor ) != 0 ) { Col = { 0.25f, 0.35f, 1.00f }; return true; } // blue tarcza
    if ( ( flags & spPassengerStopPoint ) != 0 ) { Col = { 0.92f, 0.92f, 0.96f }; return true; } // white W4
    double v { Point.fVelNext };
    if ( Sem != nullptr && Sem->memcell != nullptr )
    {
        if ( Sem->memcell->IsVelocity() )
            v = Sem->memcell->Value1();
    }
    if ( v <= 0.0 ) { Col = { 0.85f, 0.10f, 0.05f }; return true; } // red: stop
    if ( v <= 40.0 ) { Col = { 1.00f, 0.62f, 0.05f }; return true; } // yellow
    if ( v <= 90.0 ) { Col = { 0.95f, 0.85f, 0.05f }; return true; } // yellow-green
    Col = { 0.12f, 0.80f, 0.25f }; // green
    return true;
}

// live signal aspect with object locking (NOT state freezing):
//  - the next signal-grade point is read from the AI speed table (friend access);
//    within 1000 m the chosen signal is read EVERY FRAME (live lamps, flashing included)
//    and keeps displaying until it is passed (point elapsed/purged) - then the lock
//    resets and the next signal is picked. no geometry guessing, no display freeze
hud_signal_aspect
hud_signal_panel::scan_aspect( TDynamicObject const *Controlled, TController const *Owner )
{
    hud_signal_aspect result;
    if ( Controlled == nullptr || Owner == nullptr )
        return result;

    // locked signal: re-read its live state each frame; passed/purged -> unlock
    TSpeedPos const *signalpoint { nullptr };
    if ( m_pinned && !m_pinname.empty() )
    {
        for ( auto const &point : Owner->sSpeedTable )
        {
            if ( ( point.iFlags & spEnabled ) == 0 || ( point.iFlags & spElapsed ) != 0 )
                continue;
            if ( point.fDist <= 0.0 )
                continue;
            if ( ( point.iFlags & ( spSemaphor | spShuntSemaphor | spPassengerStopPoint ) ) == 0 )
                continue;
            if ( point.evEvent != nullptr && point.evEvent->name().rfind( m_pinname, 0 ) == 0 )
            {
                signalpoint = &point;
                break;
            }
        }
        if ( signalpoint == nullptr )
        {
            m_pinned = false; // passed: clear the display lock
            m_pinname.clear();
        }
    }

    // no lock yet: pick the nearest, not-yet-passed signal-grade point
    if ( !m_pinned )
    {
        double bestdist { std::numeric_limits<double>::max() };
        for ( auto const &point : Owner->sSpeedTable )
        {
            if ( ( point.iFlags & spEnabled ) == 0 || ( point.iFlags & spElapsed ) != 0 )
                continue;
            if ( ( point.iFlags & ( spSemaphor | spShuntSemaphor | spPassengerStopPoint ) ) == 0 )
                continue;
            if ( point.fDist <= 0.0 )
                continue;
            if ( point.fDist < bestdist )
            {
                bestdist = point.fDist;
                signalpoint = &point;
            }
        }
    }
    if ( signalpoint == nullptr )
        return result;

    // signal point info: drives the next-limit colour regardless of display distance
    result.has_signal = true;
    result.signal_vel = signalpoint->fVelNext;
    result.signal_flags = signalpoint->iFlags;
    {
        glm::vec3 semcol;
        if ( semantic_aspect_colour( *signalpoint, nullptr, semcol ) )
            result.signal_col = semcol;
    }
    // display window: read and keep displaying from 1000 m ahead until the signal is passed
    if ( signalpoint->fDist > 1000.0 )
        return result;

    // match the signal engine-wise: event name shares the group prefix (same rule as
    // scenenodegroups); the point position double-checks the selection
    std::string const eventname { signalpoint->evEvent != nullptr ? signalpoint->evEvent->name() : std::string() };
    map::semaphore const *bestsem { nullptr };
    if ( !eventname.empty() )
    {
        for ( auto const &obj : map::Objects.entries )
        {
            auto const *sem { dynamic_cast<map::semaphore const *>( obj.get() ) };
            if ( sem == nullptr || sem->models.empty() )
                continue;
            if ( eventname.rfind( sem->name, 0 ) != 0 )
                continue;
            if ( sem->memcell != nullptr )
            {
                glm::dvec3 const delta { glm::dvec3( sem->location ) - signalpoint->vPos };
                if ( glm::length( delta ) > 150.0 )
                    continue;
            }
            bestsem = sem;
            break;
        }
    }
    if ( bestsem == nullptr )
        return result;

    // B: the main model only - exact group name, else the shortest matching prefix;
    // the lamp slots mirror THIS single model's lamp order (no cross-model shuffling)
    TAnimModel *mainmodel { nullptr };
    std::size_t bestlen { std::numeric_limits<std::size_t>::max() };
    for ( auto *m : bestsem->models )
    {
        std::string const nm { m->name() };
        if ( nm == bestsem->name ) { mainmodel = m; break; }
        if ( nm.rfind( bestsem->name, 0 ) == 0 && nm.size() < bestlen )
        {
            bestlen = nm.size();
            mainmodel = m;
        }
    }
    if ( mainmodel == nullptr && !bestsem->models.empty() )
        mainmodel = bestsem->models.front();

    int lampidx { 0 };
    for ( int i = 0; i < iMaxNumLights; ++i )
        result.lit[ i ] = false;
    if ( mainmodel != nullptr )
    {
#ifdef WITH_OPENGL_MODERN
        if ( auto *r33 = dynamic_cast<opengl33_renderer *>( GfxRenderer.get() ) )
            r33->Update_AnimModel( mainmodel ); // refresh lamp opacities
#endif
        for ( int i = 0; i < iMaxNumLights && lampidx < 5; ++i )
        {
            auto const state { mainmodel->LightGet( i ) };
            if ( !state )
                continue;
            float const level { std::get<0>( *state ) };
            if ( level < 0.0f )
                continue;
            float const opac { std::get<1>( *state ) };
            glm::vec3 const col { std::get<2>( *state ).value_or( glm::vec3( 0.5f ) ) };
            // unfiltered: lit threshold is low, brightness stays continuous (flashing is real)
            result.lit[ lampidx ] = opac > 0.05f;
            result.bright[ lampidx ] = std::clamp( level * opac, 0.0f, 1.0f );
            result.col[ lampidx ] = col;
            ++lampidx;
        }
    }
    if ( lampidx > 0 )
    {
        result.live = true;
        result.lamps = 5;
        // lock the object (live re-reads continue until the signal is passed)
        m_pinned = true;
        m_pinname = bestsem->name;
        return result;
    }

    // A: semantic fallback - stable single lamp from the signal type + memcell value
    glm::vec3 semcol;
    if ( semantic_aspect_colour( *signalpoint, bestsem, semcol ) )
    {
        result.lit[ 0 ] = true;
        result.bright[ 0 ] = 1.0f;
        result.col[ 0 ] = semcol;
        result.live = true;
        result.lamps = 5;
        // lock the object (live re-reads continue until the signal is passed)
        m_pinned = true;
        m_pinname = bestsem->name;
    }
    return result;
}

void
hud_signal_panel::render()
{
    if ( !is_open )
        return;

    int flags { window_flags };
    if ( no_title_bar )
        flags |= ImGuiWindowFlags_NoTitleBar;

    if ( pos.x != -1 && pos.y != -1 )
        ImGui::SetNextWindowPos( ImVec2S( pos.x, pos.y ), ImGuiCond_Always );

    // auto-sized to content until the player grabs the resize grip (bottom-right triangle,
    // matching the other windows); then the size becomes manual
    ImVec2 const autosize { ImVec2S( Global.gui_hud.sig_width, base_height() ) };
    if ( !m_manual_size )
        ImGui::SetNextWindowSize( autosize, ImGuiCond_Always );
    ImGui::SetNextWindowSizeConstraints(
        ImVec2( Global.gui_hud.sig_width, static_cast<float>( base_height() ) ),
        ImVec2( 1.0e6f, 1.0e6f ) );

    auto const panelname { ( title.empty() ? m_name : title ) + "###" + m_name };
    if ( ImGui::Begin( panelname.c_str(), &is_open, flags ) )
    {
        ImVec2 const cur { ImGui::GetWindowSize() };
        if ( std::abs( cur.x - autosize.x ) > 0.5f || std::abs( cur.y - autosize.y ) > 0.5f )
            m_manual_size = true;
        render_contents();
        popups.remove_if( []( std::unique_ptr<ui::popup> const &popup ) { return popup->render(); } );
    }
    ImGui::End();
}

int
hud_signal_panel::base_height() const
{
    int base { hudcfg::get().sig_height };
    if ( hudcfg::item_visible( "lamps" ) )
        base = std::max( base, 10 + 22 * 5 + 2 * 4 + 8 ); // fixed 5-slot lamp column
    if ( hudcfg::item_visible( "limit" ) || hudcfg::item_visible( "nextsiglimit" ) )
        base = std::max( base, 124 ); // next-signal + current-limit rows
    return base;
}

void
hud_signal_panel::update()
{
    auto const &fb { Global.fb_size };
    auto const &cfg { hudcfg::get() };
    auto const *train { simulation::Train };
    auto const *controlled { train ? train->Dynamic() : nullptr };
    auto const *owner { controlled ? ( controlled->ctOwner != nullptr ? controlled->ctOwner : controlled->Mechanik ) : nullptr };
    auto const *mover { controlled ? controlled->MoverParameters : nullptr };
    // alarm banner triggers, matched to the game's own driving aid:
    // alerter = vigilance blinking while the blink timer is running; SHP = cab signal blinking
    // (the banner itself can be disabled through the customisation window, item "alarm")
    bool const alarm { mover != nullptr && hudcfg::item_visible( "alarm" ) && (
        ( mover->SecuritySystem.is_vigilance_blinking() && ( train != nullptr ? train->fBlinkTimer > 0 : true ) ) ||
        mover->SecuritySystem.is_cabsignal_blinking() ) };
    // wheel slip banner: top strip, same spot as the CA/SHP alarm (below it when both show)
    bool const slip { mover != nullptr && hudcfg::item_visible( "slip" ) && mover->SlippingWheels };
    // strip height: base, or taller when the vertical lamp column needs more room
    hud_signal_aspect const aspect { scan_aspect( controlled, owner ) };
    // strip height: base, or taller when the lamp column / limit rows need more room;
    // the lamp column is always the fixed 5-slot (136 px) block while the item is on
    size = { cfg.sig_width, base_height() + ( alarm ? 40 : 0 ) + ( slip ? 34 : 0 ) };
    if ( hudcfg::dragging() )
    {
        pos = { -1, -1 }; // suspend the anchor while the user drags
    }
    else if ( cfg.sig_x < 0 || cfg.sig_y < 0 )
        pos = { ( fb.x - size.x ) / 2, cfg.sig_top }; // default: top-centred until first drag
    else
        // no position protection: exactly where the player left it (may be off-screen)
        pos = { cfg.sig_x, cfg.sig_y };
}

void
hud_signal_panel::render_contents()
{
    auto winpos { ImGui::GetWindowPos() };
    float const dt { ImGui::GetIO().DeltaTime };
    auto *dl { ImGui::GetWindowDrawList() };

    // draggable module: the whole window is a drag zone (position stored in the common settings)
    {
        static bool dragging { false };
        auto const wsize { ImGui::GetWindowSize() };
        ImGui::SetCursorPos( ImVec2( 0.0f, 0.0f ) );
        ImGui::InvisibleButton( "##hud_sig_drag", wsize );
        if ( ImGui::IsItemActive() )
        {
            dragging = true;
            hudcfg::set_dragging( true );
            auto const delta { ImGui::GetIO().MouseDelta };
            winpos = ImVec2( winpos.x + delta.x, winpos.y + delta.y );
            ImGui::SetWindowPos( winpos );
        }
        if ( dragging && ImGui::IsMouseReleased( 0 ) )
        {
            dragging = false;
            hudcfg::set_dragging( false );
            winpos = ImGui::GetWindowPos();
            hudcfg::set_signal_pos( static_cast<int>( winpos.x ), static_cast<int>( winpos.y ) );
        }
    }

    auto const *train { simulation::Train };
    auto const *controlled { train ? train->Dynamic() : nullptr };
    auto const *owner { controlled ? ( controlled->ctOwner != nullptr ? controlled->ctOwner : controlled->Mechanik ) : nullptr };
    if ( controlled == nullptr || owner == nullptr )
        return;

    auto const *mover { controlled->MoverParameters };

    // --- limit data: same sources and same logic as the game's own driving aid -------------
    // (drivingaid_panel::update() expanded "Speed" block; NOT the raw VelNext/ActualProximityDist)
    int const speedlimit { static_cast<int>( owner->VelDesired ) };

    int nextspeedlimit { speedlimit };
    double nextspeedlimitdistance { std::numeric_limits<double>::max() };
    if ( speedlimit != 0 ) // if we aren't allowed to move then any next speed limit is irrelevant
    {
        // nie przekraczać rozkladowej
        auto const schedulespeedlimit { (
            ( owner->OrderCurrentGet() & ( Obey_train | Bank ) ) != 0 && owner->TrainParams.TTVmax > 0.0 ? static_cast<int>( owner->TrainParams.TTVmax ) :
            ( owner->OrderCurrentGet() & ( Obey_train | Bank ) ) == 0 ? static_cast<int>( owner->fShuntVelocity ) :
                -1 ) };
        // first take note of any speed change which should occur after passing potential current speed limit
        if ( owner->VelLimitLastDist.second > 0 )
        {
            nextspeedlimit = min_speed( schedulespeedlimit, static_cast<int>( owner->VelLimitLastDist.first ) );
            nextspeedlimitdistance = owner->VelLimitLastDist.second;
        }
        // then take into account speed change ahead, compare it with speed after potentially clearing last limit
        // lower of these two takes priority; otherwise limit lasts at least until potential last limit is cleared
        auto const noactivespeedlimit { owner->VelLimitLastDist.second < 0 };
        auto const speedatproximitydistance { min_speed( schedulespeedlimit, static_cast<int>( owner->VelNext ) ) };
        if ( speedatproximitydistance == nextspeedlimit )
        {
            if ( noactivespeedlimit )
            {
                nextspeedlimit = speedatproximitydistance;
                nextspeedlimitdistance = owner->ActualProximityDist;
            }
        }
        else if ( speedatproximitydistance < nextspeedlimit )
        {
            // if the speed limit ahead is more strict than our current limit, it's important enough to report
            if ( speedatproximitydistance < owner->VelDesired )
            {
                nextspeedlimit = speedatproximitydistance;
                nextspeedlimitdistance = owner->ActualProximityDist;
            }
            // otherwise report it only if it's located after our current (lower) limit ends
            else if ( owner->ActualProximityDist > nextspeedlimitdistance )
            {
                nextspeedlimit = speedatproximitydistance;
                nextspeedlimitdistance = owner->ActualProximityDist;
            }
        }
        else if ( noactivespeedlimit ) // implicit proximity > last, report only if last limit isn't present
        {
            nextspeedlimit = speedatproximitydistance;
            nextspeedlimitdistance = owner->ActualProximityDist;
        }
        // HACK: if our current speed limit extends beyond our scan range don't display potentially misleading information about its length
        if ( nextspeedlimitdistance >= EU07_AI_SPEEDLIMITEXTENDSBEYONDSCANRANGE )
            nextspeedlimit = speedlimit;
        // HACK: hide next speed limit if the 'limit' is a vehicle in front of us
        else if ( owner->ActualProximityDist == std::abs( owner->TrackObstacle() ) )
            nextspeedlimit = speedlimit;
    }

    if ( speedlimit != m_prevlimit )
    {
        m_flash = 2.0f;
        m_prevlimit = speedlimit;
    }
    if ( m_flash > 0.0f )
        m_flash -= dt;

    hud_signal_aspect const aspect { scan_aspect( controlled, owner ) };
    auto const &cfg { hudcfg::get() };
    char buf[ 96 ];

    // --- left column: fixed 5-slot lamp column ----------------------------------------------
    // permanently visible while the item is on: unlit slots stay dark (no hiding, no reshuffle)
    if ( hudcfg::item_visible( "lamps" ) )
    {
        float const lx { winpos.x + 8.0f };
        float ly { winpos.y + 10.0f };
        for ( int i = 0; i < 5; ++i )
        {
            bool const lit { aspect.live && aspect.lit[ i ] };
            ImU32 const lum { lit
                ? IM_COL32( static_cast<int>( aspect.col[ i ].r * 255.0f ),
                            static_cast<int>( aspect.col[ i ].g * 255.0f ),
                            static_cast<int>( aspect.col[ i ].b * 255.0f ),
                            static_cast<int>( aspect.bright[ i ] * 255.0f ) )
                : IM_COL32( 55, 58, 65, 170 ) }; // unlit: dark slot keeps the position
            dl->AddRectFilled( ImVec2( lx, ly ), ImVec2( lx + 20.0f, ly + 22.0f ), IM_COL32( 255, 255, 255, 45 ), 3.0f );
            dl->AddRectFilled( ImVec2( lx + 3.0f, ly + 3.0f ), ImVec2( lx + 17.0f, ly + 19.0f ), lum, 3.0f );
            ly += 24.0f;
        }
    }

    // --- middle column: next signal limit + current limit (large digits, fixed spot) ----
    if ( hudcfg::item_visible( "nextsiglimit" ) )
    {
        // next speed limit: STRICTLY the driving-aid data (reported only when it differs
        // from the current limit; otherwise "--"). plain digits, no signal colouring
        dl->AddText( ui_layer::font_default, 13.0f, ImVec2( winpos.x + 44.0f, winpos.y + 10.0f ), IM_COL32( 255, 255, 255, 180 ), STR_C("Next signal limit") );
        if ( nextspeedlimit != speedlimit )
        {
            if ( nextspeedlimit > 0 )
                std::snprintf( buf, sizeof( buf ), "%d km/h", nextspeedlimit );
            else
                std::snprintf( buf, sizeof( buf ), "0" );
        }
        else
            std::snprintf( buf, sizeof( buf ), "--" );
        dl->AddText( ui_layer::font_default, 20.0f, ImVec2( winpos.x + 44.0f, winpos.y + 27.0f ), IM_COL32( 255, 255, 255, 235 ), buf );
    }
    if ( hudcfg::item_visible( "limit" ) )
    {
        // current limit: plain white digits, no signal colouring by design
        dl->AddText( ui_layer::font_default, 13.0f, ImVec2( winpos.x + 44.0f, winpos.y + 60.0f ), IM_COL32( 255, 255, 255, 180 ), STR_C("Current limit") );
        std::snprintf( buf, sizeof( buf ), "%d", speedlimit );
        dl->AddText( ui_layer::font_hud, 40.0f, ImVec2( winpos.x + 44.0f, winpos.y + 76.0f ), IM_COL32( 255, 255, 255, 245 ), buf );
    }

    // --- right column: other information (signal distance, passenger, doors) ------------
    // (the old "Next limit" text row is gone - the middle column shows the next signal limit)
    if ( hudcfg::item_visible( "sigdist" ) )
    {
    double const sigdist { owner->FirstSemaphorDist };
    if ( sigdist < 5000.0 )
    {
        std::snprintf( buf, sizeof( buf ), STR_C("Signal %.0f m"), sigdist );
        dl->AddText( ui_layer::font_default, 13.0f, ImVec2( winpos.x + cfg.sig_text_left, winpos.y + cfg.sig_text_top ), IM_COL32( 255, 255, 255, 200 ), buf );
    }
    } // item sigdist

    // persistent passenger stop / exchange reminder (same source as the old Driving Aid line,
    // reuses the game's own translated string)
    if ( hudcfg::item_visible( "passenger" ) && owner->ExchangeTime > 0.0 )
    {
        std::snprintf( buf, sizeof( buf ), STR_C(" Loading/unloading in progress (%d s left)"), static_cast<int>( std::ceil( owner->ExchangeTime ) ) );
        // keep the line inside the panel: if the translated text is wider than the remaining
        // column, shift its start left just enough so the full string stays visible
        float const textwidth { ui_layer::font_default->CalcTextSizeA( 13.0f, std::numeric_limits<float>::max(), 0.0f, buf ).x };
        float const textx { std::max( winpos.x + cfg.sig_text_left, winpos.x + cfg.sig_width - 12.0f - textwidth ) };
        dl->AddText( ui_layer::font_default, 13.0f, ImVec2( textx, winpos.y + cfg.sig_text_top + 18.0f ), IM_COL32( 140, 235, 160, 230 ), buf );
    }

    // door state line (EMU/DMU); only shown while any door is opening/open
    if ( hudcfg::item_visible( "doors" ) )
    {
        bool const doorl { mover->Doors.instances[ 1 ].is_open || mover->Doors.instances[ 1 ].is_opening };
        bool const doorr { mover->Doors.instances[ 0 ].is_open || mover->Doors.instances[ 0 ].is_opening };
        if ( doorl || doorr )
        {
            std::snprintf( buf, sizeof( buf ), STR_C("Doors: %s %s"), doorl ? "L" : "-", doorr ? "R" : "-" );
            dl->AddText( ui_layer::font_default, 13.0f, ImVec2( winpos.x + cfg.sig_text_left, winpos.y + cfg.sig_text_top + 36.0f ), IM_COL32( 140, 235, 160, 230 ), buf );
        }
    }

    // --- CA / SHP alert banner -------------------------------------------------
    // trigger conditions and strings matched to the game's own driving aid ("!ALERTER! " / "!SHP!"):
    // alerter = vigilance blinking while the blink timer runs; SHP = cab signal blinking
    // banner row sits below the content area: same height logic as update()
    int base { cfg.sig_height };
    if ( hudcfg::item_visible( "lamps" ) )
        base = std::max( base, 10 + 22 * 5 + 2 * 4 + 8 );
    if ( hudcfg::item_visible( "limit" ) || hudcfg::item_visible( "nextsiglimit" ) )
        base = std::max( base, 124 );

    auto const &sec { mover->SecuritySystem };
    bool const cabflash { sec.is_cabsignal_blinking() };
    bool const vflash { !cabflash && sec.is_vigilance_blinking() && ( train != nullptr ? train->fBlinkTimer > 0 : true ) };
    if ( hudcfg::item_visible( "alarm" ) && ( cabflash || vflash ) )
    {
        std::string alarmtext;
        if ( vflash )
            alarmtext += STR("!ALERTER! ");
        if ( cabflash )
            alarmtext += STR("!SHP!");
        int r, g, b;
        if ( cabflash ) { r = 240; g = 60; b = 45; }
        else { r = 250; g = 195; b = 40; }
        float const pulse { 0.5f + 0.5f * std::sin( static_cast<float>( ImGui::GetTime() ) * 10.0f ) };
        float const y { winpos.y + static_cast<float>( base ) + 4.0f };
        // pulse the container background, keep the text at full brightness
        dl->AddRectFilled( ImVec2( winpos.x + 8.0f, y ), ImVec2( winpos.x + 312.0f, y + 30.0f ), IM_COL32( 12, 12, 12, 150 ), 6.0f );
        dl->AddRectFilled( ImVec2( winpos.x + 8.0f, y ), ImVec2( winpos.x + 312.0f, y + 30.0f ), IM_COL32( r, g, b, static_cast<int>( 70.0f + 90.0f * pulse ) ), 6.0f );
        dl->AddText( ui_layer::font_default, 19.0f, ImVec2( winpos.x + 22.0f, y + 6.0f ), IM_COL32( r, g, b, 255 ), alarmtext.c_str() );
    }

    // wheel slip banner: same place as the CA/SHP alarm, right below it when both are active
    if ( hudcfg::item_visible( "slip" ) && mover->SlippingWheels )
    {
        float const y { winpos.y + static_cast<float>( base ) + 4.0f + ( ( cabflash || vflash ) ? 34.0f : 0.0f ) };
        float const pulse { 0.5f + 0.5f * std::sin( static_cast<float>( ImGui::GetTime() ) * 10.0f ) };
        dl->AddRectFilled( ImVec2( winpos.x + 8.0f, y ), ImVec2( winpos.x + 312.0f, y + 30.0f ), IM_COL32( 12, 12, 12, 150 ), 6.0f );
        dl->AddRectFilled( ImVec2( winpos.x + 8.0f, y ), ImVec2( winpos.x + 312.0f, y + 30.0f ), IM_COL32( 90, 190, 220, static_cast<int>( 70.0f + 90.0f * pulse ) ), 6.0f );
        dl->AddText( ui_layer::font_default, 19.0f, ImVec2( winpos.x + 22.0f, y + 6.0f ), IM_COL32( 90, 190, 220, 255 ), STR_C("Wheel slip!") );
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
        caps.shunt = ctrlv->ScndCtrlPosNo > 0 && ctrlv->EngineType == TEngineType::ElectricSeriesMotor;

    // current mode
    std::string const modestr { std::string( STR_C("Current mode: ") ) + STR_C( hudcfg::mode_name( hudcfg::mode() ) ) };
    ImGui::TextUnformatted( modestr.c_str() );
    ImGui::Separator();

    // group switches: independent on/off for the top strip, the main panel and the speed
    // panel (they apply only in Custom mode; Standard always shows all of them)
    if ( hudcfg::mode() == hudcfg::custom )
    {
        bool panelon { hudcfg::panel_group() };
        if ( ImGui::Checkbox( STR_C("Main panel"), &panelon ) )
            hudcfg::set_panel_group( panelon );
        ImGui::SameLine();
        bool stripon { hudcfg::strip_group() };
        if ( ImGui::Checkbox( STR_C("Signal strip"), &stripon ) )
            hudcfg::set_strip_group( stripon );
        ImGui::SameLine();
        bool speedon { hudcfg::speed_group() };
        if ( ImGui::Checkbox( STR_C("Speed panel"), &speedon ) )
            hudcfg::set_speed_group( speedon );
    }
    else
    {
        ImGui::TextDisabled( STR_C("Group switches apply in Custom mode") );
    }
    ImGui::Separator();

    // per-item checkboxes grouped by panel (top strip / speed panel / main panel);
    // items are driven by the hud_item registry (extension point); items the current
    // vehicle does not have are greyed out with a "-" marker
    int const groups[] = { 1, 2, 0 }; // strip, speed, main
    for ( int const group : groups )
    {
        ImGui::TextUnformatted( group == 1 ? STR_C("Top strip") : ( group == 2 ? STR_C("Speed panel") : STR_C("Main panel") ) );
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
    ImGui::Separator();

    // closing this window does NOT hide the HUD; the overlay keeps showing per the checks above
    ImGui::TextUnformatted( STR_C("HUD stays visible after closing this window; F1 cycles the modes") );
}
