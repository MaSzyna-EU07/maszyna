/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "input/drivermouseinput.h"

#include "utilities/Globals.h"
#include "application/application.h"
#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "simulation/simulation.h"
#include "vehicle/Train.h"
#include "model/AnimModel.h"
#include "rendering/renderer.h"
#include "application/uilayer.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

auto const EU07_CONTROLLER_MOUSESLIDERSIZE{ 0.6 };

void
mouse_slider::bind( user_command const &Command ) {

    m_command = Command;

    auto const *train { simulation::Train };
    TMoverParameters const *vehicle { nullptr };
    switch( m_command ) {
        using enum user_command;
        case jointcontrollerset:
        case mastercontrollerset:
        case secondcontrollerset: {
            vehicle = train ? train->Controlled() : nullptr;
            break;
        }
        case trainbrakeset:
        case independentbrakeset: {
            vehicle = train ? train->Occupied() : nullptr;
            break;
        }
        default: {
            break;
        }
    }
    if( vehicle == nullptr ) { return; }

    // calculate initial value and accepted range
    switch( m_command ) {
        case user_command::jointcontrollerset: {
            // NOTE: special case, merges two separate controls
            auto const *occupied { train ? train->Occupied() : nullptr };
            if( occupied == nullptr ) { return; }

            auto const powerrange { static_cast<double>(
                vehicle->CoupledCtrl ?
                    vehicle->MainCtrlPosNo + vehicle->ScndCtrlPosNo :
                    vehicle->MainCtrlPosNo ) };
            // for simplicity upper half of the range controls power, lower controls brakes

            m_valuerange = 1.0;
            m_value =
                0.5
                + 0.5 * ( vehicle->CoupledCtrl ?
                        vehicle->MainCtrlPos + vehicle->ScndCtrlPos :
                        vehicle->MainCtrlPos ) / powerrange
                - 0.5 * occupied->LocalBrakePosA;
            m_analogue = true;
            m_invertrange = false;
            break;
        }
        case user_command::mastercontrollerset: {
            m_valuerange = vehicle->CoupledCtrl ? vehicle->MainCtrlPosNo + vehicle->ScndCtrlPosNo : vehicle->MainCtrlPosNo;
            m_value = vehicle->CoupledCtrl ? vehicle->MainCtrlPos + vehicle->ScndCtrlPos : vehicle->MainCtrlPos;
            m_analogue = false;
            m_invertrange = false;
            break;
        }
        case user_command::secondcontrollerset: {
            m_valuerange = vehicle->ScndCtrlPosNo;
            m_value = vehicle->ScndCtrlPos;
            m_analogue = false;
            m_invertrange = true;
            break;
        }
        case user_command::trainbrakeset: {
            m_valuerange = 1.0;
            m_value = ( vehicle->fBrakeCtrlPos - vehicle->Handle->GetPos( bh_MIN ) ) / ( vehicle->Handle->GetPos( bh_MAX ) - vehicle->Handle->GetPos( bh_MIN ) );
            m_analogue = true;
            m_invertrange = true;
            break;
        }
        case user_command::independentbrakeset: {
            m_valuerange = 1.0;
            m_value = vehicle->LocalBrakePosA;
            m_analogue = true;
            m_invertrange = true;
            break;
        }
        default: {
            m_valuerange = 1;
            break;
        }
    }
    // hide the cursor and place it in accordance with current slider value
    m_cursorposition = glm::dvec2(Global.cursor_pos);
    Application.set_cursor( GLFW_CURSOR_DISABLED );

    auto const controlsize { Global.window_size.y * EU07_CONTROLLER_MOUSESLIDERSIZE };
    auto const controledge { Global.window_size.y * 0.5 + controlsize * 0.5 };
    auto const stepsize { controlsize / m_valuerange };

    if( m_invertrange ) {
        m_value = ( m_analogue ? 1.0 : m_valuerange ) - m_value;
    }

    Application.set_cursor_pos(
        Global.window_size.y,
        m_analogue ? controledge - m_value * controlsize : controledge - m_value * stepsize - 0.5 * stepsize );
}

void
mouse_slider::release() {

    m_command = user_command::none;
    Application.set_cursor_pos( m_cursorposition.x, m_cursorposition.y );
    Application.set_cursor( GLFW_CURSOR_NORMAL );
}

void
mouse_slider::on_move( double const /*Mousex*/, double const Mousey ) {

    auto const controlsize { Global.window_size.y * EU07_CONTROLLER_MOUSESLIDERSIZE };
    auto const controledge { Global.window_size.y * 0.5 + controlsize * 0.5 };
    auto const stepsize { controlsize / m_valuerange };

    auto mousey = std::clamp( Mousey, controledge - controlsize, controledge );
    m_value = m_analogue ? (controledge - mousey) / controlsize : std::floor((controledge - mousey) / stepsize);
    if( m_invertrange ) {
        m_value = ( m_analogue ? 1.0 : m_valuerange ) - m_value; }
}



bool
drivermouse_input::init() {

#ifdef _WIN32
    DWORD systemkeyboardspeed;
    ::SystemParametersInfo( SPI_GETKEYBOARDSPEED, 0, &systemkeyboardspeed, 0 );
    m_updaterate = std::lerp( 0.5, 0.04, systemkeyboardspeed / 31.0 );
    DWORD systemkeyboarddelay;
    ::SystemParametersInfo( SPI_GETKEYBOARDDELAY, 0, &systemkeyboarddelay, 0 );
    m_updatedelay = std::lerp( 0.25, 1.0, systemkeyboarddelay / 3.0 );
#endif

    default_bindings();
    recall_bindings();

    return true;
}
namespace fs = std::filesystem;

bool
drivermouse_input::recall_bindings() {

    std::string filePath = "eu07_input-mouse.ini";

	if (fs::path appPath = user_config_path("eu07_input-mouse.ini"); !appPath.empty() && fs::exists(appPath))
		filePath = appPath.string();

    cParser bindingparser(filePath.c_str(), cParser::buffer_FILE);

    if (false == bindingparser.ok()) {
        return false;
    }

    // build helper translation tables
    std::unordered_map<std::string, user_command> nametocommandmap;
    std::size_t commandid = 0;
    for( auto const &description : simulation::Commands_descriptions ) {
        nametocommandmap.try_emplace(
            description.name,
            static_cast<user_command>( commandid ) );
        ++commandid;
    }

    // NOTE: to simplify things we expect one entry per line, and whole entry in one line
    while( true == bindingparser.getTokens( 1, true, "\n\r" ) ) {

        std::string bindingentry;
        bindingparser >> bindingentry;
        cParser entryparser( bindingentry );

        if( true == entryparser.getTokens( 1, true, "\n\r\t " ) ) {

            std::string bindingpoint {};
            entryparser >> bindingpoint;

            std::vector< std::reference_wrapper<user_command> > bindingtargets;

            if( bindingpoint == "wheel" ) {
                bindingtargets.emplace_back( std::ref( m_wheelbindings.up ) );
                bindingtargets.emplace_back( std::ref( m_wheelbindings.down ) );
            }
            // TODO: binding targets for mouse buttons

            for( auto &bindingtarget : bindingtargets ) {
                // grab command(s) associated with the input pin
                auto const bindingcommandname{ entryparser.getToken<std::string>() };
                if( true == bindingcommandname.empty() ) {
                    // no tokens left, may as well complain then call it a day
                    WriteLog( "Mouse binding for " + bindingpoint + " didn't specify associated command(s)" );
                    break;
                }
                auto const commandlookup = nametocommandmap.find( bindingcommandname );
                if( commandlookup == nametocommandmap.end() ) {
                    WriteLog( "Mouse binding for " + bindingpoint + " specified unknown command, \"" + bindingcommandname + "\"" );
                }
                else {
                    bindingtarget.get() = commandlookup->second;
                }
            }
        }
    }

    return true;
}

void
drivermouse_input::move( double Mousex, double Mousey ) {

    if( false == Global.ControlPicking ) {
        // default control mode
        m_relay.post(
            user_command::viewturn,
            Mousex,
            Mousey,
            GLFW_PRESS,
		    0 );
    }
    else {
        // control picking mode
        if( m_slider.command() != user_command::none ) {
            m_slider.on_move( Mousex, Mousey );
            m_relay.post(
                m_slider.command(),
                m_slider.value(),
                0,
                GLFW_PRESS,
			    0 );
        }

        if( false == m_pickmodepanning ) {
            // even if the view panning isn't active we capture the cursor position in case it does get activated
            m_cursorposition.x = Mousex;
            m_cursorposition.y = Mousey;
            return;
        }
        glm::dvec2 cursorposition { Mousex, Mousey };
        auto const viewoffset = cursorposition - m_cursorposition;
        m_relay.post(
            user_command::viewturn,
            viewoffset.x,
            viewoffset.y,
            GLFW_PRESS,
		    0 );
        m_cursorposition = cursorposition;
    }
}

void
drivermouse_input::scroll( double const /*Xoffset*/, double const Yoffset ) {

    if( Global.ctrlState ) {
        // ctrl + scroll wheel adjusts fov
		Global.FieldOfView = std::clamp( static_cast<float>( Global.FieldOfView - Yoffset * 20.0 / Timer::subsystem.mainloop_total.average() ), 15.0f, 75.0f );
    }
    else {
        // scroll adjusts master controller
        // TODO: allow configurable scroll commands
        auto command {
            adjust_command(
                Yoffset > 0.0 ?
                    m_wheelbindings.up :
                    m_wheelbindings.down ) };

        m_relay.post(
            command,
            0,
            0,
            GLFW_PRESS,
            // TODO: pass correct entity id once the missing systems are in place
            0 );
    }
}

void
drivermouse_input::button( int const Button, int const Action ) {

    // store key state
    if( Button >= 0 ) {
        m_buttons[ Button ] = Action;
    }

    if( false == Global.ControlPicking ) { return; }

    if( true == FreeFlyModeFlag ) {
        // freefly mode
        // left mouse button launches on_click event associated with to the node
        if (Button == GLFW_MOUSE_BUTTON_LEFT && Action == GLFW_PRESS) {
            GfxRenderer->Pick_Node_Callback(
                [](scene::basic_node *node) {
                    if( node == nullptr
                     || typeid(*node) != typeid(TAnimModel) )
                        return;
                    simulation::Region->on_click( static_cast<TAnimModel const *>( node ) ); } );
        }
        // right button controls panning
        if( Button == GLFW_MOUSE_BUTTON_RIGHT ) {
            m_pickmodepanning = Action == GLFW_PRESS;
        }
    }
    else {
        // cab controls mode
        user_command &mousecommand = Button == GLFW_MOUSE_BUTTON_LEFT ? m_mousecommandleft : m_mousecommandright;

        if( Action == GLFW_RELEASE ) {
            if( mousecommand != user_command::none ) {
                // NOTE: basic keyboard controls don't have any parameters
                // as we haven't yet implemented either item id system or multiplayer, the 'local' controlled vehicle and entity have temporary ids of 0
                // TODO: pass correct entity id once the missing systems are in place
				m_relay.post( mousecommand, 0, 0, Action, 0 );
                mousecommand = user_command::none;
            }
            else {
                m_pickwaiting = false;
                if (Button == GLFW_MOUSE_BUTTON_LEFT && m_slider.command() != user_command::none) {
                    m_relay.post( m_slider.command(), 0, 0, Action, 0 );
                    m_slider.release();
                }
                // if it's the right mouse button that got released and we had no command active, we were potentially in view panning mode; stop it
                if( Button == GLFW_MOUSE_BUTTON_RIGHT ) {
                    m_pickmodepanning = false;
                }
            }
            // if we were in varying command repeat rate, we can stop that now. done without any conditions, to catch some unforeseen edge cases
            m_varyingpollrate = false;
        }
        else {
            // if not release then it's press
            m_pickwaiting = true;
            GfxRenderer->Pick_Control_Callback(
                [this, Button, Action, &mousecommand](TSubModel const *controlsubmodel, const glm::vec2 pos) {
                    using enum user_command;

                    bool pickwaiting = m_pickwaiting;
                    m_pickwaiting = false;

                    // click on python screen
                    if (Button == GLFW_MOUSE_BUTTON_LEFT
                            && controlsubmodel && controlsubmodel->screen_touch_list) {

                        controlsubmodel->screen_touch_list->emplace_back(pos);
                    }

                    auto const [leftbinding, rightbinding]{ bindings( simulation::Train->GetLabel( controlsubmodel ) ) };
                    // if the recognized element under the cursor has a command associated with the pressed button, notify the recipient
                    mousecommand = Button == GLFW_MOUSE_BUTTON_LEFT ? leftbinding : rightbinding;

                    if( mousecommand == none ) {
                        // if we don't have any recognized element under the cursor and the right button was pressed, enter view panning mode
                        if( Button == GLFW_MOUSE_BUTTON_RIGHT ) {
                            m_pickmodepanning = true;
                        }
                        return;
                    }
                    // check manually for commands which have 'fast' variants launched with shift modifier
                    if( Global.shiftState ) {
                        switch( mousecommand ) {
                            case mastercontrollerincrease: { mousecommand = mastercontrollerincreasefast; break; }
                            case mastercontrollerdecrease: { mousecommand = mastercontrollerdecreasefast; break; }
                            case secondcontrollerincrease: { mousecommand = secondcontrollerincreasefast; break; }
                            case secondcontrollerdecrease: { mousecommand = secondcontrollerdecreasefast; break; }
                            case independentbrakeincrease: { mousecommand = independentbrakeincreasefast; break; }
                            case independentbrakedecrease: { mousecommand = independentbrakedecreasefast; break; }
                            default: { break; }
                        }
                    }

					switch( mousecommand ) {
					    case mastercontrollerincrease:
					    case mastercontrollerdecrease:
					    case secondcontrollerincrease:
					    case secondcontrollerdecrease:
					    case trainbrakeincrease:
					    case trainbrakedecrease:
					    case independentbrakeincrease:
					    case independentbrakedecrease: {
						    // these commands trigger varying repeat rate mode,
						    // which scales the rate based on the distance of the cursor from its point when the command was first issued
						    m_varyingpollrateorigin = m_cursorposition;
							m_varyingpollrate = true;
							break;
					    }
					    case jointcontrollerset:
					    case mastercontrollerset:
					    case secondcontrollerset:
					    case trainbrakeset:
					    case independentbrakeset: {
						    m_slider.bind( mousecommand );
							mousecommand = none;
							return;
					    }
					    default: {
						    break;
					    }
					}
					// NOTE: basic keyboard controls don't have any parameters
					// NOTE: as we haven't yet implemented either item id system or multiplayer, the 'local' controlled vehicle and entity have temporary ids of 0
					// TODO: pass correct entity id once the missing systems are in place
					m_relay.post( mousecommand, 0, 0, Action, 0 );
					if (!pickwaiting) // already depressed
						m_relay.post( mousecommand, 0, 0, GLFW_RELEASE, 0 );
					m_updateaccumulator = -0.25; // prevent potential command repeat right after issuing one
				} );
        }
    }
}

int
drivermouse_input::button( int const Button ) const {

    return m_buttons[ Button ];
}

void
drivermouse_input::poll() {

    m_updateaccumulator += Timer::GetDeltaRenderTime();

    auto updaterate { m_updaterate };
    if( m_varyingpollrate ) {
        updaterate /= std::max( 0.15, 2.0 * glm::length( m_cursorposition - m_varyingpollrateorigin ) / std::max( 1, Global.window_size.y ) );
    }

    while( m_updateaccumulator > updaterate ) {

        if( m_mousecommandleft != user_command::none ) {
            // NOTE: basic keyboard controls don't have any parameters
            // as we haven't yet implemented either item id system or multiplayer, the 'local' controlled vehicle and entity have temporary ids of 0
            // TODO: pass correct entity id once the missing systems are in place
			m_relay.post( m_mousecommandleft, 0, 0, GLFW_REPEAT, 0 );
        }
        if( m_mousecommandright != user_command::none ) {
            // NOTE: basic keyboard controls don't have any parameters
            // as we haven't yet implemented either item id system or multiplayer, the 'local' controlled vehicle and entity have temporary ids of 0
            // TODO: pass correct entity id once the missing systems are in place
			m_relay.post( m_mousecommandright, 0, 0, GLFW_REPEAT, 0 );
        }
        m_updateaccumulator -= updaterate;
    }
}

user_command
drivermouse_input::command() const {

    return m_slider.command() != user_command::none ? m_slider.command() : m_mousecommandleft != user_command::none ? m_mousecommandleft : m_mousecommandright;
}

// returns pair of bindings associated with specified cab control
std::pair<user_command, user_command>
drivermouse_input::bindings( std::string const &Control ) const {

    auto const lookup{ m_buttonbindings.find( Control ) };

    if( lookup != m_buttonbindings.end() )
        return { lookup->second.left, lookup->second.right };
    else {
        return { user_command::none, user_command::none };
    }
}

void
drivermouse_input::default_bindings() {
    using enum user_command;
    // pierwsza komenda jest od zwiekszania a druga od zmniejszania - ewentualnie kolejno lewy i prawy przycisk
    m_buttonbindings = {
        { "jointctrl:", {
            jointcontrollerset,
            none } },
        { "mainctrl:", {
            mastercontrollerset,
            none } },
    	{ "dynamicbrakectrl:", {
    		dynamicbrakecontrollerset,
			none } },
        { "scndctrl:", {
            secondcontrollerset,
            none } },
        { "shuntmodepower:", {
            secondcontrollerincrease,
            secondcontrollerdecrease } },
        { "tempomat_sw:", {
            tempomattoggle,
            none } },
        { "tempomatoff_sw:", {
            tempomattoggle,
            none } },
        { "dirkey:", {
            reverserincrease,
            reverserdecrease } },
        { "dirforward_bt:", {
            reverserforward,
            none } },
        { "dirneutral_bt:", {
            reverserneutral,
            none } },
        { "dirbackward_bt:", {
            reverserbackward,
            none } },
        { "brakectrl:", {
            trainbrakeset,
            none } },
        { "localbrake:", {
            independentbrakeset,
            none } },
        { "manualbrake:", {
            manualbrakeincrease,
            manualbrakedecrease } },
        { "alarmchain:", {
            alarmchaintoggle,
            none } },
        { "alarmchainon:", {
            alarmchainenable,
            none} },
        { "alarmchainoff:", {
            alarmchainenable,
            none} },
        { "brakeprofile_sw:", {
            brakeactingspeedincrease,
            brakeactingspeeddecrease } },
        // TODO: dedicated methods for braking speed switches
        { "brakeprofileg_sw:", {
            brakeactingspeedsetcargo,
            brakeactingspeedsetpassenger } },
        { "brakeprofiler_sw:", {
            brakeactingspeedsetrapid,
            brakeactingspeedsetpassenger } },
        { "brakeopmode_sw:", {
            trainbrakeoperationmodeincrease,
            trainbrakeoperationmodedecrease } },
        { "maxcurrent_sw:", {
            motoroverloadrelaythresholdtoggle,
            none } },
        { "waterpumpbreaker_sw:", {
            waterpumpbreakertoggle,
            none } },
        { "waterpump_sw:", {
            waterpumptoggle,
            none } },
        { "waterheaterbreaker_sw:", {
            waterheaterbreakertoggle,
            none } },
        { "waterheater_sw:", {
            waterheatertoggle,
            none } },
        { "fuelpump_sw:", {
            fuelpumptoggle,
            none } },
        { "oilpump_sw:", {
            oilpumptoggle,
            none } },
        { "motorblowersfront_sw:", {
            motorblowerstogglefront,
            none } },
        { "motorblowersrear_sw:", {
            motorblowerstogglerear,
            none } },
        { "motorblowersalloff_sw:", {
            motorblowersdisableall,
            none } },
        { "coolingfans_sw:", {
            coolingfanstoggle,
            none } },
        { "main_off_bt:", {
            linebreakeropen,
            none } },
        { "main_on_bt:",{
            linebreakerclose,
            none } },
        { "security_reset_bt:", {
            alerteracknowledge,
            none } },
        { "shp_reset_bt:", {
            cabsignalacknowledge,
            none } },
        { "releaser_bt:", {
            independentbrakebailoff,
            none } },
		{ "springbraketoggle_bt:",{
			springbraketoggle,
			none } },
		{ "springbrakeon_bt:",{
			springbrakeenable,
			none } },
		{ "springbrakeoff_bt:",{
			springbrakedisable,
			none } },
		{ "universalbrake1_bt:",{
			universalbrakebutton1,
			none } },
		{ "universalbrake2_bt:",{
			universalbrakebutton2,
			none } },
		{ "universalbrake3_bt:",{
			universalbrakebutton3,
			none } },
		{ "epbrake_bt:",{
			epbrakecontroltoggle,
			none } },
		{ "epbrakeon_bt:",{
			epbrakecontrolenable,
			none } },
		{ "epbrakeoff_bt:",{
			epbrakecontroldisable,
			none } },
        { "sand_bt:", {
            sandboxactivate,
            none } },
        { "antislip_bt:", {
            wheelspinbrakeactivate,
            none } },
        { "horn_bt:", {
            hornhighactivate,
            hornlowactivate } },
        { "hornlow_bt:", {
            hornlowactivate,
            none } },
        { "hornhigh_bt:", {
            hornhighactivate,
            none } },
        { "whistle_bt:", {
            whistleactivate,
            none } },
        { "fuse_bt:", {
            motoroverloadrelayreset,
            none } },
        { "converterfuse_bt:", {
            converteroverloadrelayreset,
            none } },
        { "relayreset1_bt:", {
            universalrelayreset1,
            none } },
        { "relayreset2_bt:", {
            universalrelayreset2,
            none } },
        { "relayreset3_bt:", {
            universalrelayreset3,
            none } },
        { "stlinoff_bt:", {
            motorconnectorsopen,
            none } },
        { "doorleftpermit_sw:", {
            doorpermitleft,
            none } },
        { "doorrightpermit_sw:", {
            doorpermitright,
            none } },
        { "doorpermitpreset_sw:", {
            doorpermitpresetactivatenext,
            doorpermitpresetactivateprevious } },
        { "door_left_sw:", {
            doortoggleleft,
            none } },
        { "door_right_sw:", {
            doortoggleright,
            none } },
        { "doorlefton_sw:", {
            dooropenleft,
            none } },
        { "doorrighton_sw:", {
            dooropenright,
            none } },
        { "doorleftoff_sw:", {
            doorcloseleft,
            none } },
        { "doorrightoff_sw:", {
            doorcloseright,
            none } },
        { "doorallon_sw:", {
            dooropenall,
            none } },
        { "dooralloff_sw:", {
            doorcloseall,
            none } },
        { "doorstep_sw:", {
            doorsteptoggle,
            none } },
        { "doormode_sw:", {
            doormodetoggle,
            none } },
		{ "mirrors_sw:", {
			mirrorstoggle,
			none } },
        { "departure_signal_bt:", {
            departureannounce,
            none } },
        { "upperlight_sw:", {
            headlighttoggleupper,
            none } },
        { "leftlight_sw:", {
            headlighttoggleleft,
            none } },
        { "rightlight_sw:", {
            headlighttoggleright,
            none } },
        { "dimheadlights_sw:", {
            headlightsdimtoggle,
            none } },
	    {"moderndimmer_sw:", {
            modernlightdimmerincrease, 
            modernlightdimmerdecrease } },
        { "leftend_sw:", {
            redmarkertoggleleft,
            none } },
        { "rightend_sw:", {
            redmarkertoggleright,
            none } },
        { "lights_sw:", {
            lightspresetactivatenext,
            lightspresetactivateprevious } },
        { "rearupperlight_sw:", {
            headlighttogglerearupper,
            none } },
        { "rearleftlight_sw:", {
            headlighttogglerearleft,
            none } },
        { "rearrightlight_sw:", {
            headlighttogglerearright,
            none } },
        { "rearleftend_sw:", {
            redmarkertogglerearleft,
            none } },
        { "rearrightend_sw:", {
            redmarkertogglerearright,
            none } },
        { "compressor_sw:", {
            compressortoggle,
            none } },
        { "compressorlocal_sw:", {
            compressortogglelocal,
            none } },
		{ "compressorlist_sw:", {
			compressorpresetactivatenext,
			compressorpresetactivateprevious } },
        { "converter_sw:", {
            convertertoggle,
            none } },
        { "converterlocal_sw:", {
            convertertogglelocal,
            none } },
        { "converteroff_sw:", {
            convertertoggle,
            none } }, // TODO: dedicated converter shutdown command
        { "main_sw:", {
            linebreakertoggle,
            none } },
        { "radio_sw:", {
            radiotoggle,
            none } },
        { "radioon_sw:", {
            radioenable,
            none } },
        { "radiooff_sw:", {
            radiodisable,
            none } },
        { "radiochannel_sw:", {
            radiochannelincrease,
            radiochanneldecrease } },
        { "radiochannelprev_sw:", {
            radiochanneldecrease,
            none } },
        { "radiochannelnext_sw:", {
            radiochannelincrease,
            none } },
        { "radiostop_sw:", {
            radiostopsend,
            none } },
        { "radiostopon_sw:", {
            radiostopenable,
            none } },
        { "radiostopoff_sw:", {
            radiostopdisable,
            none } },
        { "radiotest_sw:", {
            radiostoptest,
            none } },
		{ "radiocall1_sw:", {
			radiocall1send,
			none } },
        { "radiocall3_sw:", {
            radiocall3send,
            none } },
		{ "radiovolume_sw:",{
			radiovolumeincrease,
			radiovolumedecrease } },
		{ "radiovolumeprev_sw:",{
			radiovolumedecrease,
			none } },
		{ "radiovolumenext_sw:",{
			radiovolumeincrease,
			none } },
        { "pantfront_sw:", {
            pantographtogglefront,
            none } },
        { "pantrear_sw:", {
            pantographtogglerear,
            none } },
        { "pantfrontoff_sw:", {
            pantographlowerfront,
            none } },
        { "pantrearoff_sw:", {
            pantographlowerrear,
            none } },
        { "pantalloff_sw:", {
            pantographlowerall,
            none } },
        { "pantselected_sw:", {
            pantographtoggleselected,
            none } }, // TBD: bind lowerselected in case of toggle switch
        { "pantselectedoff_sw:", {
            pantographlowerselected,
            none } },
        { "pantselect_sw:", {
            pantographselectnext,
            pantographselectprevious } },
        { "pantvalves_sw:", {
            pantographvalvesupdate,
            pantographvalvesoff } },
        { "pantvalvesupdate_bt:", {
	         pantographvalvesupdate, 
             none}},
        { "pantvalvesoff_bt:", {
	         pantographvalvesoff, 
             none}},
        { "pantcompressor_sw:", {
            pantographcompressoractivate,
            none } },
        { "pantcompressorvalve_sw:", {
            pantographcompressorvalvetoggle,
            none } },
        { "trainheating_sw:", {
            heatingtoggle,
            none } },
        { "signalling_sw:", {
            mubrakingindicatortoggle,
            none } },
        { "door_signalling_sw:", {
            doorlocktoggle,
            none } },
        { "nextcurrent_sw:", {
            mucurrentindicatorothersourceactivate,
            none } },
        { "distancecounter_sw:", {
            distancecounteractivate,
            none } },
        { "instrumentlight_sw:", {
            instrumentlighttoggle,
            none } },
        { "dashboardlight_sw:", {
            dashboardlighttoggle,
            none } },
        { "dashboardlighton_sw:", {
            dashboardlightenable,
            none } },
        { "dashboardlightoff_sw:", {
            dashboardlightdisable,
            none } },
        { "timetablelight_sw:", {
            timetablelighttoggle,
            none } },
        { "timetablelighton_sw:", {
            timetablelightenable,
            none } },
        { "timetablelightoff_sw:", {
            timetablelightdisable,
            none } },
        { "cablight_sw:", {
            interiorlighttoggle,
            none } },
        { "cablightdim_sw:", {
            interiorlightdimtoggle,
            none } },
        { "compartmentlights_sw:", {
            compartmentlightstoggle,
            none } },
        { "compartmentlightson_sw:", {
            compartmentlightsenable,
            none } },
        { "compartmentlightsoff_sw:", {
            compartmentlightsdisable,
            none } },
        { "battery_sw:", {
            batterytoggle,
            none } },
        { "batteryon_sw:", {
            batteryenable,
            none } },
        { "batteryoff_sw:", {
            batterydisable,
            none } },
		{ "cabactivation_sw:", {
			cabactivationtoggle,
			none } },
        { "couplingdisconnect_sw:",{
			occupiedcarcouplingdisconnect,
			none } },
		{ "couplingdisconnectback_sw:",{
			occupiedcarcouplingdisconnectback,
			none } },
	    {"universal0:", {generictoggle0, none}},
	    {"universal1:", {generictoggle1, none}},
	    {"universal2:", {generictoggle2, none}},
	    {"universal3:", {generictoggle3, none}},
	    {"universal4:", {generictoggle4, none}},
	    {"universal5:", {generictoggle5, none}},
	    {"universal6:", {generictoggle6, none}},
	    {"universal7:", {generictoggle7, none}},
	    {"universal8:", {generictoggle8, none}},
	    {"universal9:", {generictoggle9, none}},
	    {"universal10:", {generictoggle10, none}},
	    {"universal11:", {generictoggle11, none}},
	    {"universal12:", {generictoggle12, none}},
	    {"universal13:", {generictoggle13, none}},
	    {"universal14:", {generictoggle14, none}},
	    {"universal15:", {generictoggle15, none}},
	    {"universal16:", {generictoggle16, none}},
	    {"universal17:", {generictoggle17, none}},
	    {"universal18:", {generictoggle18, none}},
	    {"universal19:", {generictoggle19, none}},
	    {"universal20:", {generictoggle20, none}},
	    {"universal21:", {generictoggle21, none}},
	    {"universal22:", {generictoggle22, none}},
	    {"universal23:", {generictoggle23, none}},
	    {"universal24:", {generictoggle24, none}},
	    {"universal25:", {generictoggle25, none}},
	    {"universal26:", {generictoggle26, none}},
	    {"universal27:", {generictoggle27, none}},
	    {"universal28:", {generictoggle28, none}},
	    {"universal29:", {generictoggle29, none}},
		{ "speedinc_bt:",{
			speedcontrolincrease,
			none } },
		{ "speeddec_bt:",{
			speedcontroldecrease,
			none } },
		{ "speedctrlpowerinc_bt:",{
			speedcontrolpowerincrease,
			none } },
		{ "speedctrlpowerdec_bt:",{
			speedcontrolpowerdecrease,
			none } },
		{ "speedbutton0:",{
			speedcontrolbutton0,
			none } },
		{ "speedbutton1:",{
			speedcontrolbutton1,
			none } },
		{ "speedbutton2:",{
			speedcontrolbutton2,
			none } },
		{ "speedbutton3:",{
			speedcontrolbutton3,
			none } },
		{ "speedbutton4:",{
			speedcontrolbutton4,
			none } },
		{ "speedbutton5:",{
			speedcontrolbutton5,
			none } },
		{ "speedbutton6:",{
			speedcontrolbutton6,
			none } },
		{ "speedbutton7:",{
			speedcontrolbutton7,
			none } },
		{ "speedbutton8:",{
			speedcontrolbutton8,
			none } },
		{ "speedbutton9:",{
			speedcontrolbutton9,
			none } },
		{ "inverterenable1_bt:",{
			inverterenable1,
			none } },
		{ "inverterenable2_bt:",{
			inverterenable2,
			none } },
		{ "inverterenable3_bt:",{
			inverterenable3,
			none } },
		{ "inverterenable4_bt:",{
			inverterenable4,
			none } },
		{ "inverterenable5_bt:",{
			inverterenable5,
			none } },
		{ "inverterenable6_bt:",{
			inverterenable6,
			none } },
		{ "inverterenable7_bt:",{
			inverterenable7,
			none } },
		{ "inverterenable8_bt:",{
			inverterenable8,
			none } },
		{ "inverterenable9_bt:",{
			inverterenable9,
			none } },
		{ "inverterenable10_bt:",{
			inverterenable10,
			none } },
		{ "inverterenable11_bt:",{
			inverterenable11,
			none } },
		{ "inverterenable12_bt:",{
			inverterenable12,
			none } },
		{ "inverterdisable1_bt:",{
			inverterdisable1,
			none } },
		{ "inverterdisable2_bt:",{
			inverterdisable2,
			none } },
		{ "inverterdisable3_bt:",{
			inverterdisable3,
			none } },
		{ "inverterdisable4_bt:",{
			inverterdisable4,
			none } },
		{ "inverterdisable5_bt:",{
			inverterdisable5,
			none } },
		{ "inverterdisable6_bt:",{
			inverterdisable6,
			none } },
		{ "inverterdisable7_bt:",{
			inverterdisable7,
			none } },
		{ "inverterdisable8_bt:",{
			inverterdisable8,
			none } },
		{ "inverterdisable9_bt:",{
			inverterdisable9,
			none } },
		{ "inverterdisable10_bt:",{
			inverterdisable10,
			none } },
		{ "inverterdisable11_bt:",{
			inverterdisable11,
			none } },
		{ "inverterdisable12_bt:",{
			inverterdisable12,
			none } },
		{ "invertertoggle1_bt:",{
			invertertoggle1,
			none } },
		{ "invertertoggle2_bt:",{
			invertertoggle2,
			none } },
		{ "invertertoggle3_bt:",{
			invertertoggle3,
			none } },
		{ "invertertoggle4_bt:",{
			invertertoggle4,
			none } },
		{ "invertertoggle5_bt:",{
			invertertoggle5,
			none } },
		{ "invertertoggle6_bt:",{
			invertertoggle6,
			none } },
		{ "invertertoggle7_bt:",{
			invertertoggle7,
			none } },
		{ "invertertoggle8_bt:",{
			invertertoggle8,
			none } },
		{ "invertertoggle9_bt:",{
			invertertoggle9,
			none } },
		{ "invertertoggle10_bt:",{
			invertertoggle10,
			none } },
		{ "invertertoggle11_bt:",{
			invertertoggle11,
			none } },
		{ "invertertoggle12_bt:",{
			invertertoggle12,
			none } },
        { "wipers_sw:",{
			wiperswitchincrease,
			wiperswitchdecrease
         } },

    };
}

user_command
drivermouse_input::adjust_command( user_command Command ) const {
    using enum user_command;

    if( true == Global.shiftState
     && Command != none ) {
        switch( Command ) {
            case mastercontrollerincrease: { Command = mastercontrollerincreasefast; break; }
            case mastercontrollerdecrease: { Command = mastercontrollerdecreasefast; break; }
            case secondcontrollerincrease: { Command = secondcontrollerincreasefast; break; }
            case secondcontrollerdecrease: { Command = secondcontrollerdecreasefast; break; }
            case independentbrakeincrease: { Command = independentbrakeincreasefast; break; }
            case independentbrakedecrease: { Command = independentbrakedecreasefast; break; }
            default: { break; }
        }
    }

    return Command;
}

//---------------------------------------------------------------------------
