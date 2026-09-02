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
#include "utilities/Logs.h"
#include "widgets/vehicleparams.h"
#include "utilities/U8.h"

#define DRIVER_HINT_CONTENT
#include "application/driverhints.h"

#ifdef WITH_UART
#include "utilities/uart.h"
#endif


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
// hudcfg: HUD layout configuration via the shared public config (hud.* keys in eu07.ini)
//---------------------------------------------------------------------------

namespace hudcfg {

static settings g;
static ui_panel *g_panel { nullptr };
static ui_panel *g_signalpanel { nullptr };
static bool g_visible { true };
static bool g_dragging { false };
static void apply_visibility();

void load()
{
    // settings arrive from the shared public configuration ("hud.*" keys in the
    // existing config file, parsed into global_settings); missing keys keep defaults
    settings s;
    s.enabled            = Global.hud_enabled;
    s.panel_width        = Global.hud_panel_width;
    s.panel_height       = Global.hud_panel_height;
    s.margin             = Global.hud_margin;
    s.speed_size         = Global.hud_speed_size;
    s.panel_x            = Global.hud_panel_x;
    s.panel_y            = Global.hud_panel_y;
    s.sig_width          = Global.hud_sig_width;
    s.sig_height         = Global.hud_sig_height;
    s.sig_top            = Global.hud_sig_top;
    s.sig_digit_size     = Global.hud_sig_digit_size;
    s.sig_digit_left     = Global.hud_sig_digit_left;
    s.sig_square_margin  = Global.hud_sig_square_margin;
    s.sig_square_size    = Global.hud_sig_square_size;
    s.sig_text_left      = Global.hud_sig_text_left;
    s.sig_text_top       = Global.hud_sig_text_top;
    s.sig_x              = Global.hud_sig_x;
    s.sig_y              = Global.hud_sig_y;
    g = s;
}

void save()
{
    // persist through the shared public configuration output (global_settings)
    Global.hud_enabled = g.enabled;
    Global.hud_panel_x = g.panel_x;
    Global.hud_panel_y = g.panel_y;
    Global.hud_sig_x = g.sig_x;
    Global.hud_sig_y = g.sig_y;
    Global.SaveIniFile();
}

settings const &get()
{
    return g;
}

void set_panels( ui_panel *Panel, ui_panel *SignalPanel )
{
    g_panel = Panel;
    g_signalpanel = SignalPanel;
    g_visible = g.enabled;
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
    if (g_panel != nullptr)
        g_panel->is_open = g_visible;
    if (g_signalpanel != nullptr)
        g_signalpanel->is_open = g_visible;
}

void set_panel_pos( int const X, int const Y )
{
    g.panel_x = X;
    g.panel_y = Y;
    save();
}

void set_signal_pos( int const X, int const Y )
{
    g.sig_x = X;
    g.sig_y = Y;
    save();
}

}

//---------------------------------------------------------------------------
// hud_panel: heads-up display overlay (borderless, freely movable window)
//---------------------------------------------------------------------------

void
hud_panel::update()
{
    auto const &fb { Global.fb_size };
    auto const &cfg { hudcfg::get() };
    size = { cfg.panel_width, cfg.panel_height };
    if ( hudcfg::dragging() )
    {
        pos = { -1, -1 }; // suspend the anchor while the user drags
    }
    else if (cfg.panel_x >= 0 && cfg.panel_y >= 0
          && cfg.panel_x + cfg.panel_width <= fb.x && cfg.panel_y + cfg.panel_height <= fb.y)
        pos = { cfg.panel_x, cfg.panel_y };
    else
        pos = { fb.x - size.x - cfg.margin, fb.y - size.y - cfg.margin };
}

void
hud_panel::render_contents()
{
    auto winpos { ImGui::GetWindowPos() };
    float const dt { ImGui::GetIO().DeltaTime };
    float const now { static_cast<float>( ImGui::GetTime() ) };
    auto *dl { ImGui::GetWindowDrawList() };

    // draggable module: the whole window is a drag zone (position persisted to the shared config)
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
        // subtle grip indicator at the bottom-right corner
        for ( int i = 0; i < 3; ++i )
            dl->AddCircleFilled( ImVec2( winpos.x + wsize.x - 12.0f - i * 8.0f, winpos.y + wsize.y - 8.0f ), 1.8f, IM_COL32( 255, 255, 255, 70 ) );
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
    auto const draw_bar = [&]( float const Y, char const *Label, float const Value, float const Vmax, char const *ValueText, ImU32 const ColOverride = 0 )
    {
        float const x0 { winpos.x + 8.0f };
        float const w { 230.0f };
        float const frac { Vmax > 0.0f ? std::clamp( Value / Vmax, 0.0f, 1.0f ) : 0.0f };
        dl->AddText( ui_layer::font_default, 15.0f, ImVec2( x0, Y ), IM_COL32( 255, 255, 255, 190 ), Label );
        dl->AddRectFilled( ImVec2( x0 + 70.0f, Y + 18.0f ), ImVec2( x0 + 70.0f + w, Y + 28.0f ), IM_COL32( 255, 255, 255, 30 ), 4.0f );
        if ( frac > 0.005f )
            dl->AddRectFilled( ImVec2( x0 + 70.0f, Y + 18.0f ), ImVec2( x0 + 70.0f + w * frac, Y + 28.0f ), ColOverride != 0 ? ColOverride : bar_col( frac ), 4.0f );
        dl->AddText( ui_layer::font_default, 15.0f, ImVec2( x0 + 70.0f + w + 10.0f, Y ), IM_COL32( 255, 255, 255, 240 ), ValueText );
    };

    float barY { winpos.y + 6.0f };
    if ( is_diesel )
    {
        // real diesel instrument: engine revolutions (RPM), see EngineRPMRatio/EngineMaxRPM
        double const rpmratio { std::clamp( mover->EngineRPMRatio(), 0.0, 1.0 ) };
        double const rpm { mover->EngineMaxRPM() * rpmratio };
        std::snprintf( buf, sizeof( buf ), "%.0f rpm", rpm );
        draw_bar( barY, STR_C("Engine RPM"), static_cast<float>( rpmratio ), 1.0f, buf );
        barY += 26.0f;
    }
    else if ( is_electric )
    {
        if ( two_groups )
        {
            std::snprintf( buf, sizeof( buf ), "%.0f A", dynbrake ? -std::abs( cur1 ) : std::abs( cur1 ) );
            draw_bar( barY, STR_C("Current 1"), std::abs( cur1 ), 500.0f, buf, current_col );
            barY += 26.0f;
            std::snprintf( buf, sizeof( buf ), "%.0f A", dynbrake ? -std::abs( cur2 ) : std::abs( cur2 ) );
            draw_bar( barY, STR_C("Current 2"), std::abs( cur2 ), 500.0f, buf, current_col );
            barY += 26.0f;
        }
        else
        {
            std::snprintf( buf, sizeof( buf ), "%.0f A", dynbrake ? -std::abs( train->fHCurrent[0] ) : std::abs( train->fHCurrent[0] ) );
            draw_bar( barY, STR_C("Current"), std::abs( train->fHCurrent[0] ), 800.0f, buf, current_col );
            barY += 26.0f;
        }
    }
    // brake pipe pressure (SPKS), labelled as the pipe itself
    float const pipefrac { std::clamp( static_cast<float>( mover->PipePress ) / 6.0f, 0.0f, 1.0f ) };
    std::snprintf( buf, sizeof( buf ), "%.1f bar", mover->PipePress );
    ImU32 const pipecol { mover->PipePress < 3.0 ? IM_COL32( 255, 70, 45, 240 ) : ( mover->PipePress < 4.5 ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 130, 220, 140, 240 ) ) };
    draw_bar( barY, STR_C("Brake pipe"), pipefrac, 1.0f, buf, pipecol );
    barY += 26.0f;
    // main brake handle position (same source as the game's driving aid "basicbraking" value);
    // vehicles with electro-dynamic brake show the ED brake position instead
    if ( dynbrake )
    {
        float const dbpos { static_cast<float>( mover->DynamicBrakeCtrlPos ) };
        float const dbno { mover->DynamicBrakeCtrlPosNo > 0 ? static_cast<float>( mover->DynamicBrakeCtrlPosNo ) : 10.0f };
        std::snprintf( buf, sizeof( buf ), "%.0f/%.0f", std::floor( dbpos * dbno + 0.5f ), dbno );
        draw_bar( barY, STR_C("Dynamic brake"), dbpos, 1.0f, buf );
    }
    else
    {
        float const brakcfrac { mover->BrakeCtrlPosNo > 0 ? std::clamp( static_cast<float>( std::max( 0.0, mover->fBrakeCtrlPos ) ) / mover->BrakeCtrlPosNo, 0.0f, 1.0f ) : 0.0f };
        std::snprintf( buf, sizeof( buf ), "%.1f", mover->fBrakeCtrlPos );
        draw_bar( barY, STR_C("train brake"), brakcfrac, 1.0f, buf );
    }
    barY += 26.0f;
    float const indfrac { static_cast<float>( std::clamp( mover->LocalBrakePosA, 0.0, 1.0 ) ) };
    std::snprintf( buf, sizeof( buf ), "%.0f/10", std::floor( indfrac * 10.0f ) );
    draw_bar( barY, STR_C("independent brake"), indfrac, 1.0f, buf );
    barY += 26.0f;
    // brake cylinder pressure (cab gauge "CYLINDER HAMULCOWY"), real SPKS value
    float const cylfrac { std::clamp( static_cast<float>( mover->BrakePress ) / 6.0f, 0.0f, 1.0f ) };
    std::snprintf( buf, sizeof( buf ), "%.2f bar", mover->BrakePress );
    ImU32 const cylcol { mover->BrakePress < 0.5f ? IM_COL32( 130, 220, 140, 240 ) : ( mover->BrakePress < 3.5f ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 255, 70, 45, 240 ) ) };
    draw_bar( barY, STR_C("Brake cylinder"), cylfrac, 1.0f, buf, cylcol );

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
    // --- notch / controller row (same layout style as the bars above) --------
    float const nstripy { winpos.y + 180.0f };
    float const sx0 { winpos.x + 78.0f };
    float const sw { 230.0f };
    dl->AddText( ui_layer::font_default, 15.0f, ImVec2( winpos.x + 8.0f, nstripy ), IM_COL32( 255, 255, 255, 190 ), STR_C("Notch") );
    // value text: position + zone name, e.g. "9/16 并联"
    char notchvalue[ 64 ];
    std::snprintf( notchvalue, sizeof( notchvalue ), "%s%s%s", notchtext.c_str(), notchtext.empty() ? "" : " ", zone );
    dl->AddText( ui_layer::font_default, 15.0f, ImVec2( sx0 + sw + 10.0f, nstripy ), IM_COL32( 255, 255, 255, 240 ), notchvalue );
    dl->AddRectFilled( ImVec2( sx0, nstripy + 18.0f ), ImVec2( sx0 + sw, nstripy + 28.0f ), IM_COL32( 255, 255, 255, 30 ), 4.0f );
    if ( eim_throttle )
    {
        // center-zero bidirectional gauge: right = drive (green), left = brake (amber)
        float const cx { sx0 + sw * 0.5f };
        dl->AddRectFilled( ImVec2( cx - 1.0f, nstripy + 17.0f ), ImVec2( cx + 1.0f, nstripy + 29.0f ), IM_COL32( 255, 255, 255, 70 ) );
        if ( notchfrac > 0.005f )
        {
            if ( ctrlv->eimic_real < 0.0 )
                dl->AddRectFilled( ImVec2( cx - sw * 0.5f * notchfrac, nstripy + 18.0f ), ImVec2( cx, nstripy + 28.0f ), IM_COL32( 255, 190, 40, 230 ), 4.0f );
            else
                dl->AddRectFilled( ImVec2( cx, nstripy + 18.0f ), ImVec2( cx + sw * 0.5f * notchfrac, nstripy + 28.0f ), IM_COL32( 90, 220, 120, 230 ), 4.0f );
        }
    }
    else if ( notchfrac > 0.005f )
        dl->AddRectFilled( ImVec2( sx0, nstripy + 18.0f ), ImVec2( sx0 + sw * notchfrac, nstripy + 28.0f ), IM_COL32( 90, 160, 255, 230 ), 4.0f );

    // field-weakening (shunt) controller row; only DC electrics have real field weakening
    // (on induction vehicles SCPN>0 means a power step switch, not a shunt controller)
    if ( ctrlv->ScndCtrlPosNo > 0 && dc_series )
    {
        float const nstripy2 { winpos.y + 210.0f };
        float const shfrac { std::clamp( static_cast<float>( ctrlv->ScndCtrlPos ) / ctrlv->ScndCtrlPosNo, 0.0f, 1.0f ) };
        dl->AddText( ui_layer::font_default, 15.0f, ImVec2( winpos.x + 8.0f, nstripy2 ), IM_COL32( 255, 255, 255, 190 ), STR_C("Shunt") );
        std::snprintf( notchvalue, sizeof( notchvalue ), "%d/%d", ctrlv->ScndCtrlPos, ctrlv->ScndCtrlPosNo );
        dl->AddText( ui_layer::font_default, 15.0f, ImVec2( sx0 + sw + 10.0f, nstripy2 ), IM_COL32( 255, 255, 255, 240 ), notchvalue );
        dl->AddRectFilled( ImVec2( sx0, nstripy2 + 18.0f ), ImVec2( sx0 + sw, nstripy2 + 28.0f ), IM_COL32( 255, 255, 255, 30 ), 4.0f );
        if ( shfrac > 0.005f )
            dl->AddRectFilled( ImVec2( sx0, nstripy2 + 18.0f ), ImVec2( sx0 + sw * shfrac, nstripy2 + 28.0f ), IM_COL32( 180, 130, 255, 230 ), 4.0f );
    }

    // --- draw: big speed digits ------------------------------------------------
    std::snprintf( buf, sizeof( buf ), "%03.0f", speed_kmh );
    ImFont const * const font { ui_layer::font_hud };
    float const fontsize { hudcfg::get().speed_size };
    ImVec2 const textsize { font->CalcTextSizeA( fontsize, std::numeric_limits<float>::max(), 0.0f, buf ) };
    // bottom-anchored: digits sit 12 px above the window's bottom edge
    ImVec2 const dpos { winpos.x + 52.0f + ( 258.0f - textsize.x ) * 0.5f, winpos.y + hudcfg::get().panel_height - 12.0f - textsize.y };
    ImU32 const col { ImGui::ColorConvertFloat4ToU32( ImVec4( m_speedcolor.x, m_speedcolor.y, m_speedcolor.z, alpha ) ) };
    // outline/shadow then fill
    dl->AddText( font, fontsize, ImVec2( dpos.x + 3.0f, dpos.y + 3.0f ), IM_COL32( 0, 0, 0, 150 ), buf );
    dl->AddText( font, fontsize, dpos, col, buf );
    // km/h caption
    dl->AddText( ui_layer::font_default, 15.0f, ImVec2( dpos.x + textsize.x + 5.0f, dpos.y + textsize.y - 6.0f ), IM_COL32( 255, 255, 255, 180 ), "km/h" );

    // --- draw: direction arrows (simple solid triangles) -------------------------
    auto const arrow_col { []( bool const Active ) { return Active ? IM_COL32( 255, 255, 255, 245 ) : IM_COL32( 200, 210, 220, 45 ); } };
    float const ax { winpos.x + 34.0f };
    // up triangle = forward
    bool const fwd { mover->DirActive > 0 };
    dl->AddTriangleFilled(
        ImVec2( ax - 17.0f, dpos.y + 46.0f ),
        ImVec2( ax + 17.0f, dpos.y + 46.0f ),
        ImVec2( ax, dpos.y + 4.0f ),
        arrow_col( fwd ) );
    // down triangle = backward
    bool const bwd { mover->DirActive < 0 };
    dl->AddTriangleFilled(
        ImVec2( ax - 17.0f, dpos.y + 58.0f ),
        ImVec2( ax + 17.0f, dpos.y + 58.0f ),
        ImVec2( ax, dpos.y + 104.0f ),
        arrow_col( bwd ) );

    // --- draw: gradient triangles with numeric value (hidden when flat) ----------------
    // value text reuses the game's own translated format "Grade: %.1f%%%%" (F1 driving aid source)
    if ( grade_pm > 2.5 || grade_pm < -2.5 )
    {
        ImU32 const slope_col { IM_COL32( 110, 235, 120, 240 ) };
        auto const slope_num_col = []( double const Grade ) -> int {
            double const a { std::abs( Grade ) };
            if ( a >= 35.0 ) return 0;   // steep: red
            if ( a >= 20.0 ) return 1;   // noticeable: amber
            return 2;                    // mild: green
        };
        int const ns { slope_num_col( grade_pm ) };
        ImU32 const snumcol { ns == 0 ? IM_COL32( 240, 80, 60, 240 ) : ( ns == 1 ? IM_COL32( 255, 200, 40, 240 ) : IM_COL32( 110, 235, 120, 240 ) ) };
        char gbuf[ 32 ];
        // use the game's exact translation key (leading space included), same call pattern as the F1 driving aid
        std::snprintf( gbuf, sizeof( gbuf ), STR_C(" Grade: %.1f%%%%"), std::abs( grade_pm ) * 0.1 );
        float const sx { winpos.x + 346.0f };
        if ( grade_pm > 2.5 ) // uphill
        {
            dl->AddTriangleFilled( ImVec2( sx - 16.0f, dpos.y + 40.0f ), ImVec2( sx + 16.0f, dpos.y + 40.0f ), ImVec2( sx, dpos.y + 4.0f ), slope_col );
            dl->AddText( ui_layer::font_default, 11.0f, ImVec2( winpos.x + 300.0f, dpos.y + 44.0f ), snumcol, gbuf );
        }
        else // downhill
        {
            dl->AddTriangleFilled( ImVec2( sx - 16.0f, dpos.y + 52.0f ), ImVec2( sx + 16.0f, dpos.y + 52.0f ), ImVec2( sx, dpos.y + 88.0f ), slope_col );
            dl->AddText( ui_layer::font_default, 11.0f, ImVec2( winpos.x + 300.0f, dpos.y + 94.0f ), snumcol, gbuf );
        }
    }
}

//---------------------------------------------------------------------------
// hud_signal_panel: top-of-screen signal preview + speed limit strip
//---------------------------------------------------------------------------

void
hud_signal_panel::update()
{
    auto const &fb { Global.fb_size };
    auto const &cfg { hudcfg::get() };
    auto const *train { simulation::Train };
    auto const *controlled { train ? train->Dynamic() : nullptr };
    auto const *mover { controlled ? controlled->MoverParameters : nullptr };
    bool const alarm { mover != nullptr && (
        mover->SecuritySystem.is_vigilance_blinking() || mover->SecuritySystem.is_beeping() ||
        mover->SecuritySystem.is_cabsignal_blinking() || mover->SecuritySystem.is_cabsignal_beeping() ||
        mover->SecuritySystem.is_braking() ) };
    size = { cfg.sig_width, alarm ? cfg.sig_height + 40 : cfg.sig_height };
    if ( hudcfg::dragging() )
    {
        pos = { -1, -1 }; // suspend the anchor while the user drags
    }
    else if (cfg.sig_x >= 0 && cfg.sig_y >= 0
          && cfg.sig_x + cfg.sig_width <= fb.x && cfg.sig_y + cfg.sig_height <= fb.y)
        pos = { cfg.sig_x, cfg.sig_y };
    else
        pos = { ( fb.x - size.x ) / 2, cfg.sig_top };
}

void
hud_signal_panel::render_contents()
{
    auto winpos { ImGui::GetWindowPos() };
    float const dt { ImGui::GetIO().DeltaTime };
    auto *dl { ImGui::GetWindowDrawList() };

    // draggable module: the whole window is a drag zone (position persisted to the shared config)
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
        // subtle grip indicator at the bottom-right corner
        for ( int i = 0; i < 3; ++i )
            dl->AddCircleFilled( ImVec2( winpos.x + wsize.x - 12.0f - i * 8.0f, winpos.y + wsize.y - 8.0f ), 1.8f, IM_COL32( 255, 255, 255, 70 ) );
    }

    auto const *train { simulation::Train };
    auto const *controlled { train ? train->Dynamic() : nullptr };
    auto const *owner { controlled ? ( controlled->ctOwner != nullptr ? controlled->ctOwner : controlled->Mechanik ) : nullptr };
    if ( controlled == nullptr || owner == nullptr )
        return;

    auto const *mover { controlled->MoverParameters };

    int const limit { static_cast<int>( owner->VelDesired ) };
    int const nextlimit { static_cast<int>( owner->VelNext ) };
    double const nextdist { owner->ActualProximityDist };
    // in the sim VelDesired < 0 means "no speed limit" (while 0 = stop); surface it as green "-"
    bool const nolimit { limit < 0 };

    if ( !nolimit && limit != m_prevlimit )
    {
        m_flash = 2.0f;
        m_prevlimit = limit;
    }
    if ( m_flash > 0.0f )
        m_flash -= dt;

    int dr, dg, db;
    // color shows the speed-limit intensity relative to the vehicle's maximum speed
    // (real quantity; the sim does not expose a raw lamp aspect)
    double const vmax { controlled->MoverParameters->Vmax };
    if ( nolimit ) { dr = 90; dg = 210; db = 100; }
    else if ( limit <= 0 ) { dr = 235; dg = 60; db = 50; }
    else if ( vmax > 0.0 && limit < 0.7 * vmax ) { dr = 240; dg = 190; db = 40; }
    else { dr = 90; dg = 210; db = 100; }
    float const brightalpha { m_flash > 0.0f ? ( 0.55f + 0.45f * std::sin( static_cast<float>( ImGui::GetTime() ) * 14.0f ) ) : 1.0f };

    // signal colour block, digit line centred on the block's centre line
    auto const &cfg { hudcfg::get() };
    float const sqm { cfg.sig_square_margin };
    float const sqs { cfg.sig_square_size };
    dl->AddRectFilled( ImVec2( winpos.x + sqm, winpos.y + sqm ), ImVec2( winpos.x + sqm + sqs, winpos.y + sqm + sqs ), IM_COL32( 255, 255, 255, 40 ), 6.0f );
    dl->AddRectFilled( ImVec2( winpos.x + sqm + 4.0f, winpos.y + sqm + 4.0f ), ImVec2( winpos.x + sqm + sqs - 4.0f, winpos.y + sqm + sqs - 4.0f ), IM_COL32( dr, dg, db, static_cast<int>( 255.0f * brightalpha ) ), 4.0f );

    // big limit number; vertical position follows the block's centre line
    char buf[ 96 ];
    std::snprintf( buf, sizeof( buf ), "%d", limit );
    ImU32 const digitcol { nolimit ? IM_COL32( 110, 235, 120, 215 ) : ( limit <= 0 ? IM_COL32( 235, 60, 50, static_cast<int>( 255.0f * brightalpha ) ) : IM_COL32( 255, 255, 255, static_cast<int>( 235.0f * brightalpha ) ) ) };
    float const sqcy { sqm + sqs * 0.5f };
    if ( nolimit )
        dl->AddText( ui_layer::font_default, cfg.sig_digit_size, ImVec2( winpos.x + cfg.sig_digit_left, winpos.y + sqcy - cfg.sig_digit_size * 0.55f ), digitcol, "--" );
    else
        dl->AddText( ui_layer::font_hud, cfg.sig_digit_size, ImVec2( winpos.x + cfg.sig_digit_left, winpos.y + sqcy - cfg.sig_digit_size * 0.55f ), digitcol, buf );

    // distance to the next signal (real data from the AI route scan)
    double const sigdist { owner->FirstSemaphorDist };
    if ( sigdist < 5000.0 )
    {
        std::snprintf( buf, sizeof( buf ), STR_C("Signal %.0f m"), sigdist );
        dl->AddText( ui_layer::font_default, 13.0f, ImVec2( winpos.x + cfg.sig_text_left, winpos.y + cfg.sig_text_top ), IM_COL32( 255, 255, 255, 200 ), buf );
    }
    // next limit preview
    if ( nextlimit != limit && nextdist < 5000.0 && nextdist > 0.0 )
    {
        std::snprintf( buf, sizeof( buf ), STR_C("Next limit %.1f km: %d"), nextdist * 0.001, nextlimit );
        dl->AddText( ui_layer::font_default, 13.0f, ImVec2( winpos.x + cfg.sig_text_left, winpos.y + cfg.sig_text_top + 18.0f ), IM_COL32( 255, 255, 255, 200 ), buf );
    }

    // persistent passenger stop / exchange reminder (same source as the old Driving Aid line,
    // reuses the game's own translated string)
    if ( owner->ExchangeTime > 0.0 )
    {
        std::snprintf( buf, sizeof( buf ), STR_C(" Loading/unloading in progress (%d s left)"), static_cast<int>( std::ceil( owner->ExchangeTime ) ) );
        dl->AddText( ui_layer::font_default, 13.0f, ImVec2( winpos.x + cfg.sig_text_left, winpos.y + cfg.sig_text_top + 36.0f ), IM_COL32( 140, 235, 160, 230 ), buf );
    }

    // --- CA / SHP alert banner -------------------------------------------------
    // trigger: real SecuritySystem state; text: read from the game's own active hints
    auto const &sec { mover->SecuritySystem };
    bool const cabflash { sec.is_cabsignal_blinking() || sec.is_cabsignal_beeping() || sec.is_braking() };
    bool const vflash { !cabflash && ( sec.is_vigilance_blinking() || sec.is_beeping() ) };
    if ( cabflash || vflash )
    {
        // read the prompt straight from the game's hint data (same source as the F1 "Hints" list)
        char const *alarmtext { nullptr };
        if ( owner != nullptr )
        {
            for ( auto const &hint : owner->m_hints )
            {
                auto const h { std::get<driver_hint>( hint ) };
                if ( h == driver_hint::shpsystemreset || h == driver_hint::securitysystemreset )
                {
                    alarmtext = Translations.lookup_c( driver_hints_texts[ static_cast<size_t>( h ) ], true );
                    break;
                }
            }
        }
        if ( alarmtext == nullptr )
            alarmtext = cabflash ? Translations.lookup_c( "Acknowledge SHP", true ) : Translations.lookup_c( "Acknowledge alerter", true );
        int r, g, b;
        if ( sec.is_braking() ) { r = 240; g = 60; b = 45; }
        else { r = 250; g = 195; b = 40; }
        float const pulse { 0.5f + 0.5f * std::sin( static_cast<float>( ImGui::GetTime() ) * 10.0f ) };
        float const y { winpos.y + hudcfg::get().sig_height + 4.0f };
        // pulse the container background, keep the text at full brightness
        dl->AddRectFilled( ImVec2( winpos.x + 8.0f, y ), ImVec2( winpos.x + 312.0f, y + 30.0f ), IM_COL32( 12, 12, 12, 150 ), 6.0f );
        dl->AddRectFilled( ImVec2( winpos.x + 8.0f, y ), ImVec2( winpos.x + 312.0f, y + 30.0f ), IM_COL32( r, g, b, static_cast<int>( 70.0f + 90.0f * pulse ) ), 6.0f );
        dl->AddText( ui_layer::font_default, 19.0f, ImVec2( winpos.x + 22.0f, y + 6.0f ), IM_COL32( r, g, b, 255 ), alarmtext );
    }
}
