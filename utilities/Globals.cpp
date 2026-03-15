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
    Copyright (C) 2001-2004  Marcin Wozniak, Maciej Czapkiewicz and others

*/
#include "stdafx.h"
#include "utilities/Globals.h"

#include "simulation/simulation.h"
#include "simulation/simulationenvironment.h"
#include "vehicle/Driver.h"
#include "utilities/Logs.h"
#include "Console.h"
#include "scripting/PyInt.h"
#include "utilities/Timer.h"
#include "vao.h"

void global_settings::LoadIniFile(std::string asFileName)
{
	// initialize season data in case the main config file doesn't
	std::time_t timenow = std::time(nullptr);

	std::tm tm{};

#ifdef _WIN32
	localtime_s(&tm, &timenow);
#else
	localtime_r(&timenow, &tm);
#endif

	fMoveLight = tm.tm_yday + 1; // numer bieżącego dnia w roku
	simulation::Environment.compute_season(fMoveLight);

	cParser parser(asFileName, cParser::buffer_FILE);
	ConfigParse(parser);
}

template <typename T>
static void ParseOne(cParser& parser, T& out, int tokenCount = 1, bool convert = false)
{
    parser.getTokens(tokenCount, convert);
    parser >> out;
}

template <typename T>
static void ParseOneClamped(cParser& parser, T& out, T minValue, T maxValue, int tokenCount = 1, bool convert = false)
{
    parser.getTokens(tokenCount, convert);
    parser >> out;
    out = clamp(out, minValue, maxValue);
}

void global_settings::FinalizeConfig()
{
    if (!bLoadTraction)
    {
        bEnableTraction = false;
        bLiveTraction = false;
    }

    if (vr)
    {
        gfx_skippipeline = false;
        VSync = false;
    }

    if (iPause)
        iTextMode = GLFW_KEY_F1;

#ifndef WITH_PYTHON
    python_enabled = false;
#endif

#ifdef _WIN32
    Console::ModeSet(iFeedbackMode, iFeedbackPort);
#endif
}

bool global_settings::ConfigParseGeneral(cParser& Parser, const std::string& token)
{
    if (token == "sceneryfile")
    {
        ParseOne(Parser, SceneryFile);
        return true;
    }

    if (token == "humanctrlvehicle")
    {
        ParseOne(Parser, local_start_vehicle);
        return true;
    }

    if (token == "width")
    {
        ParseOne(Parser, window_size.x, 1, false);
        return true;
    }

    if (token == "height")
    {
        ParseOne(Parser, window_size.y, 1, false);
        return true;
    }

    if (token == "fullscreen")
    {
        ParseOne(Parser, bFullScreen);
        return true;
    }

    if (token == "fullscreenmonitor")
    {
        ParseOne(Parser, fullscreen_monitor, 1, false);
        return true;
    }

    if (token == "fullscreenwindowed")
    {
        ParseOne(Parser, fullscreen_windowed);
        return true;
    }

    if (token == "hideconsole")
    {
        ParseOne(Parser, bHideConsole);
        return true;
    }

    if (token == "lang")
    {
        ParseOne(Parser, asLang, 1, false);
        return true;
    }

    if (token == "screenshotsdir")
    {
        ParseOne(Parser, Global.screenshot_dir, 1);
        return true;
    }

    if (token == "execonexit")
    {
        ParseOne(Parser, exec_on_exit, 1);
        std::replace(std::begin(exec_on_exit), std::end(exec_on_exit), '_', ' ');
        return true;
    }

    if (token == "prepend_scn")
    {
        ParseOne(Parser, prepend_scn, 1);
        return true;
    }

    return false;
}

bool global_settings::ConfigParseAudio(cParser& Parser, const std::string& token)
{
    if (token == "soundenabled")
    {
        ParseOne(Parser, bSoundEnabled);
        return true;
    }

    if (token == "sound.openal.renderer")
    {
        AudioRenderer = Parser.getToken<std::string>(false);
        return true;
    }

    if (token == "sound.volume")
    {
        ParseOneClamped(Parser, AudioVolume, 0.f, 2.f);
        return true;
    }

    if (token == "sound.volume.radio")
    {
        ParseOneClamped(Parser, DefaultRadioVolume, 0.f, 1.f);
        return true;
    }

    if (token == "sound.maxsources")
    {
        ParseOne(Parser, audio_max_sources);
        return true;
    }

    if (token == "sound.volume.vehicle")
    {
        ParseOneClamped(Parser, VehicleVolume, 0.f, 1.f);
        return true;
    }

    if (token == "sound.volume.positional")
    {
        ParseOneClamped(Parser, EnvironmentPositionalVolume, 0.f, 1.f);
        return true;
    }

    if (token == "sound.volume.ambient")
    {
        ParseOneClamped(Parser, EnvironmentAmbientVolume, 0.f, 1.f);
        return true;
    }

    return false;
}

bool global_settings::ConfigParseGraphics(cParser& Parser, const std::string& token)
{
    if (token == "fieldofview")
    {
        ParseOneClamped(Parser, FieldOfView, 10.0f, 75.0f, 1, false);
        return true;
    }

    if (token == "heightbase")
    {
        ParseOne(Parser, fDistanceFactor, 1, false);
        return true;
    }

    if (token == "basedrawrange")
    {
        ParseOne(Parser, BaseDrawRange, 1);
        return true;
    }

    if (token == "vsync")
    {
        ParseOne(Parser, VSync);
        return true;
    }

    if (token == "wireframe")
    {
        ParseOne(Parser, bWireFrame);
        return true;
    }

    if (token == "skyenabled")
    {
        std::string value;
        ParseOne(Parser, value);
        asSky = (value == "yes" ? "1" : "0");
        return true;
    }

    if (token == "defaultext")
    {
        std::string value;
        ParseOne(Parser, value);

        if (value == "tga")
            szDefaultExt = szTexturesTGA;
        else
            szDefaultExt = (value[0] == '.' ? value : "." + value);

        return true;
    }

    if (token == "anisotropicfiltering")
    {
        ParseOne(Parser, AnisotropicFiltering, 1, false);
        if (AnisotropicFiltering < 1.0f)
            AnisotropicFiltering = 1.0f;
        return true;
    }

    if (token == "usevbo")
    {
        ParseOne(Parser, bUseVBO);
        return true;
    }

    if (token == "maxtexturesize")
    {
        int size = 0;
        ParseOne(Parser, size, 1, false);
        iMaxTextureSize = clamp_power_of_two(size, 64, 8192);
        return true;
    }

    if (token == "maxcabtexturesize")
    {
        int size = 0;
        ParseOne(Parser, size, 1, false);
        iMaxCabTextureSize = clamp_power_of_two(size, 512, 8192);
        return true;
    }

    if (token == "dynamiclights")
    {
        ParseOne(Parser, DynamicLightCount, 1, false);
        DynamicLightCount = clamp(DynamicLightCount, 0, 7);
        return true;
    }

    if (token == "gfxrenderer")
    {
        ParseOne(Parser, GfxRenderer);

        if (GfxRenderer == "full")
            GfxRenderer = "default";

        if (GfxRenderer == "experimental")
        {
            NvRenderer = true;
            GfxRenderer = "experimental";
        }

        BasicRenderer = (GfxRenderer == "simple");
        LegacyRenderer = !NvRenderer && (GfxRenderer != "default");
        return true;
    }

    if (token == "shadows")
    {
        ParseOne(Parser, RenderShadows);
        return true;
    }

    if (token == "shadowtune")
    {
        Parser.getTokens(4, false);

        float discard = 0.f;
        Parser >> shadowtune.map_size >> discard >> shadowtune.range >> discard;

        shadowtune.map_size = clamp_power_of_two<unsigned int>(shadowtune.map_size, 512, 8192);
        shadowtune.range =
            std::max((shadowtune.map_size <= 2048 ? 75.f : 75.f * shadowtune.map_size / 2048),
                     shadowtune.range);
        return true;
    }

    if (token == "scalespeculars")
    {
        ParseOne(Parser, ScaleSpecularValues);
        return true;
    }

    if (token == "rendercab")
    {
        ParseOne(Parser, render_cab);
        return true;
    }

    if (token == "multisampling")
    {
        ParseOne(Parser, iMultisampling, 1, false);
        iMultisampling = clamp(iMultisampling, 0, 4);
        return true;
    }

    if (token == "compresstex")
    {
        Parser.getTokens(1);
        if (!gfx_usegles)
            Parser >> compress_tex;
        return true;
    }

    if (token == "gfx.angleplatform")
    {
        ParseOne(Parser, gfx_angleplatform, 1);
        return true;
    }

    if (token == "gfx.gldebug")
    {
        ParseOne(Parser, gfx_gldebug, 1);
        return true;
    }

    if (token == "extraviewport")
    {
        Parser.getTokens(15, false);

        extraviewport_config conf;
        Parser >> conf.monitor >> conf.width >> conf.height
               >> conf.draw_range
               >> conf.projection.pa.x >> conf.projection.pa.y >> conf.projection.pa.z
               >> conf.projection.pb.x >> conf.projection.pb.y >> conf.projection.pb.z
               >> conf.projection.pc.x >> conf.projection.pc.y >> conf.projection.pc.z
               >> conf.projection.pe.x >> conf.projection.pe.y >> conf.projection.pe.z;

        extra_viewports.push_back(conf);

        if (gl::vao::use_vao && conf.monitor != "MAIN")
        {
            gl::vao::use_vao = false;
            WriteLog("using multiple windows, disabling vao!");
        }
        return true;
    }

    return false;
}

bool global_settings::ConfigParseInput(cParser& Parser, const std::string& token)
{
    if (token == "freefly")
    {
        ParseOne(Parser, FreeFlyModeFlag);
        Parser.getTokens(3, false);
        Parser >> FreeCameraInit[0].x >> FreeCameraInit[0].y >> FreeCameraInit[0].z;
        return true;
    }

    if (token == "mousescale")
    {
        Parser.getTokens(2, false);
        Parser >> fMouseXScale >> fMouseYScale;
        return true;
    }

    if (token == "mousecontrol")
    {
        ParseOne(Parser, InputMouse);
        return true;
    }

    if (token == "input.gamepad")
    {
        ParseOne(Parser, InputGamepad);
        return true;
    }

    if (token == "headtrack")
    {
        Parser.getTokens(14);
        Parser >> headtrack_conf.joy
               >> headtrack_conf.magic_window
               >> headtrack_conf.move_axes[0] >> headtrack_conf.move_axes[1] >> headtrack_conf.move_axes[2]
               >> headtrack_conf.move_mul[0] >> headtrack_conf.move_mul[1] >> headtrack_conf.move_mul[2]
               >> headtrack_conf.rot_axes[0] >> headtrack_conf.rot_axes[1] >> headtrack_conf.rot_axes[2]
               >> headtrack_conf.rot_mul[0] >> headtrack_conf.rot_mul[1] >> headtrack_conf.rot_mul[2];
        return true;
    }

    return false;
}

bool global_settings::ConfigParseSimulation(cParser& Parser, const std::string& token)
{
    if (token == "async.trainThreads")
    {
        ParseOne(Parser, trainThreads);
        return true;
    }

    if (token == "physicslog")
    {
        ParseOne(Parser, WriteLogFlag);
        return true;
    }

    if (token == "fullphysics")
    {
        ParseOne(Parser, FullPhysics);
        return true;
    }

    if (token == "enabletraction")
    {
        ParseOne(Parser, bEnableTraction);
        return true;
    }

    if (token == "loadtraction")
    {
        ParseOne(Parser, bLoadTraction);
        return true;
    }

    if (token == "friction")
    {
        ParseOne(Parser, fFriction, 1, false);
        return true;
    }

    if (token == "livetraction")
    {
        ParseOne(Parser, bLiveTraction);
        return true;
    }

    if (token == "newaircouplers")
    {
        ParseOne(Parser, bnewAirCouplers);
        return true;
    }

    if (token == "movelight")
    {
        ParseOne(Parser, fMoveLight, 1, false);

        if (fMoveLight == 0.f)
        {
            std::time_t timenow = std::time(nullptr);
            std::tm tm{};

#ifdef _WIN32
            localtime_s(&tm, &timenow);
#else
            localtime_r(&timenow, &tm);
#endif

            fMoveLight = tm.tm_yday + 1;
        }

        simulation::Environment.compute_season(fMoveLight);
        return true;
    }

    if (token == "scenario.time.override")
    {
        Parser.getTokens(1, false);

        std::string value;
        Parser >> value;

        std::istringstream stream(value);

        if (value.find(':') != std::string::npos)
        {
            float hours = 0.f;
            float minutes = 0.f;
            char sep = '\0';

            stream >> hours >> sep >> minutes;
            ScenarioTimeOverride = hours + minutes / 60.0f;
        }
        else
        {
            stream >> ScenarioTimeOverride;
        }

        ScenarioTimeOverride = clamp(ScenarioTimeOverride, 0.f, 24 * 1439 / 1440.f);
        return true;
    }

    if (token == "scenario.time.offset")
    {
        ParseOne(Parser, ScenarioTimeOffset, 1, false);
        return true;
    }

    if (token == "scenario.time.current")
    {
        ParseOne(Parser, ScenarioTimeCurrent, 1, false);
        return true;
    }

    if (token == "scenario.weather.temperature")
    {
        ParseOneClamped(Parser, AirTemperature, -15.f, 45.f);
        return true;
    }

    if (token == "smoothtraction")
    {
        ParseOne(Parser, bSmoothTraction);
        return true;
    }

    if (token == "splinefidelity")
    {
        float splinefidelity = 0.f;
        ParseOne(Parser, splinefidelity);
        SplineFidelity = clamp(splinefidelity, 1.f, 4.f);
        return true;
    }

    if (token == "createswitchtrackbeds")
    {
        ParseOne(Parser, CreateSwitchTrackbeds);
        return true;
    }

    if (token == "timespeed")
    {
        ParseOne(Parser, default_timespeed, 1, false);
        fTimeSpeed = default_timespeed;
        return true;
    }

    if (token == "deltaoverride")
    {
        float deltaoverride = 0.f;
        ParseOne(Parser, deltaoverride, 1, false);

        if (deltaoverride > 0.f)
            Timer::set_delta_override(1.0f / deltaoverride);
        return true;
    }

    if (token == "latitude")
    {
        ParseOne(Parser, fLatitudeDeg, 1, false);
        return true;
    }

    if (token == "convertmodels")
    {
        ParseOne(Parser, iConvertModels, 1, false);
        return true;
    }

    if (token == "convertindexrange")
    {
        ParseOne(Parser, iConvertIndexRange, 1, false);
        return true;
    }

    if (token == "file.binary.terrain")
    {
        ParseOne(Parser, file_binary_terrain, 1, false);
        return true;
    }

    if (token == "inactivepause")
    {
        ParseOne(Parser, bInactivePause);
        return true;
    }

    if (token == "slowmotion")
    {
        ParseOne(Parser, iSlowMotionMask, 1, false);
        return true;
    }

    if (token == "fpsaverage")
    {
        ParseOne(Parser, fFpsAverage, 1, false);
        return true;
    }

    if (token == "fpsdeviation")
    {
        ParseOne(Parser, fFpsDeviation, 1, false);
        return true;
    }

    if (token == "brakestep")
    {
        ParseOne(Parser, fBrakeStep, 1, false);
        return true;
    }

    if (token == "brakespeed")
    {
        ParseOne(Parser, brake_speed, 1, false);
        return true;
    }

    if (token == "joinduplicatedevents")
    {
        ParseOne(Parser, bJoinEvents);
        return true;
    }

    if (token == "hiddenevents")
    {
        ParseOne(Parser, iHiddenEvents, 1, false);
        return true;
    }

    if (token == "ai.trainman")
    {
        ParseOne(Parser, AITrainman, 1, false);
        return true;
    }

    if (token == "pause")
    {
        std::string value;
        ParseOne(Parser, value);
        iPause |= (value == "yes" ? 1 : 0);
        return true;
    }

    if (token == "priorityloadtext3d")
    {
        std::string value;
        ParseOne(Parser, value, 1);
        priorityLoadText3D = (value == "yes");
        return true;
    }

    if (token == "fpslimit")
    {
        float fpslimit = 0.f;
        ParseOne(Parser, fpslimit, 1);

        if (fpslimit > 0.f)
            minframetime = std::chrono::duration<float>(1.0f / fpslimit);
        return true;
    }

    if (token == "randomseed")
    {
        ParseOne(Parser, Global.random_seed, 1);
        return true;
    }

    if (token == "map.manualswitchcontrol")
    {
        ParseOne(Parser, map_manualswitchcontrol, 1);
        return true;
    }

    if (token == "map.highlightdistance")
    {
        ParseOne(Parser, map_highlight_distance, 1);
        return true;
    }

    if (token == "crashdamage")
    {
        ParseOne(Parser, crash_damage, 1);
        return true;
    }

    return false;
}

bool global_settings::ConfigParseUI(cParser& Parser, const std::string& token)
{
    if (token == "uitextcolor")
    {
        Parser.getTokens(3, false);
        Parser >> UITextColor.r >> UITextColor.g >> UITextColor.b;
        UITextColor = glm::clamp(UITextColor, 0.f, 255.f);
        UITextColor = UITextColor / 255.f;
        UITextColor.a = 1.f;
        return true;
    }

    if (token == "ui.bg.opacity")
    {
        ParseOneClamped(Parser, UIBgOpacity, 0.f, 1.f);
        return true;
    }

    if (token == "ui.fontsize")
    {
        ParseOne(Parser, ui_fontsize, 1);
        return true;
    }

    if (token == "ui.scale")
    {
        ParseOne(Parser, ui_scale, 1);
        return true;
    }

    if (token == "gui.defaultwindows")
    {
        ParseOne(Parser, gui_defaultwindows, 1);
        return true;
    }

    if (token == "gui.showtranscripts")
    {
        ParseOne(Parser, gui_showtranscripts, 1);
        return true;
    }

    if (token == "gui.trainingdefault")
    {
        ParseOne(Parser, gui_trainingdefault, 1);
        return true;
    }

    return false;
}

bool global_settings::ConfigParsePython(cParser& Parser, const std::string& token)
{
    if (token == "python.enabled")
    {
        ParseOne(Parser, python_enabled, 1);
        return true;
    }

    if (token == "python.updatetime")
    {
        ParseOne(Parser, PythonScreenUpdateRate);
        return true;
    }

    if (token == "python.displaywindows")
    {
        ParseOne(Parser, python_displaywindows, 1);
        return true;
    }

    if (token == "python.threadedupload")
    {
        ParseOne(Parser, python_threadedupload, 1);
        return true;
    }

    if (token == "python.sharectx")
    {
        ParseOne(Parser, python_sharectx, 1);
        return true;
    }

    if (token == "python.uploadmain")
    {
        ParseOne(Parser, python_uploadmain, 1);
        return true;
    }

    if (token == "python.fpslimit")
    {
        float fpslimit = 0.f;
        ParseOne(Parser, fpslimit, 1);

        if (fpslimit > 0.f)
            python_minframetime = std::chrono::duration<float>(1.0f / fpslimit);
        return true;
    }

    if (token == "python.vsync")
    {
        ParseOne(Parser, python_vsync, 1);
        return true;
    }

    if (token == "python.mipmaps")
    {
        ParseOne(Parser, python_mipmaps, 1);
        return true;
    }

    if (token == "python.viewport")
    {
        Parser.getTokens(8, false);

        pythonviewport_config conf;
        Parser >> conf.surface >> conf.monitor
               >> conf.size.x >> conf.size.y
               >> conf.offset.x >> conf.offset.y
               >> conf.scale.x >> conf.scale.y;

        python_viewports.push_back(conf);
        return true;
    }

    return false;
}

bool global_settings::ConfigParseNetwork(cParser& Parser, const std::string& token)
{
    if (token == "network.server")
    {
        Parser.getTokens(2);

        std::string backend;
        std::string conf;
        Parser >> backend >> conf;

        network_servers.emplace_back(backend, conf);
        return true;
    }

    if (token == "network.client")
    {
        Parser.getTokens(2);

        network_client.emplace();
        Parser >> network_client->first >> network_client->second;
        return true;
    }

    return false;
}

bool global_settings::ConfigParseHardware(cParser& Parser, const std::string& token)
{
    if (token == "feedbackmode")
    {
        ParseOne(Parser, iFeedbackMode, 1, false);
        return true;
    }

    if (token == "feedbackport")
    {
        ParseOne(Parser, iFeedbackPort, 1, false);
        return true;
    }

    if (token == "multiplayer")
    {
        ParseOne(Parser, iMultiplayer, 1, false);
        return true;
    }

    if (token == "isolatedtrainnumber")
    {
        ParseOne(Parser, bIsolatedTrainName, 1, false);
        return true;
    }

    if (token == "motiontelemetry")
    {
        Parser.getTokens(8);
        Global.motiontelemetry_conf.enable = true;
        Parser >> Global.motiontelemetry_conf.proto
               >> Global.motiontelemetry_conf.address
               >> Global.motiontelemetry_conf.port
               >> Global.motiontelemetry_conf.updatetime
               >> Global.motiontelemetry_conf.includegravity
               >> Global.motiontelemetry_conf.fwdposbased
               >> Global.motiontelemetry_conf.latposbased
               >> Global.motiontelemetry_conf.axlebumpscale;
        return true;
    }

    if (token == "vr.enabled")
    {
        ParseOne(Parser, vr, 1);
        return true;
    }

    if (token == "vr.backend")
    {
        ParseOne(Parser, vr_backend, 1);
        return true;
    }

#ifdef WITH_UART
    if (token == "uart")
    {
        uart_conf.enable = true;
        Parser.getTokens(3, false);
        Parser >> uart_conf.port
               >> uart_conf.baud
               >> uart_conf.updatetime;
        return true;
    }

    if (token == "uarttune")
    {
        Parser.getTokens(18);
        Parser >> uart_conf.mainbrakemin >> uart_conf.mainbrakemax
               >> uart_conf.localbrakemin >> uart_conf.localbrakemax
               >> uart_conf.tankmax >> uart_conf.tankuart
               >> uart_conf.pipemax >> uart_conf.pipeuart
               >> uart_conf.brakemax >> uart_conf.brakeuart
               >> uart_conf.pantographmax >> uart_conf.pantographuart
               >> uart_conf.hvmax >> uart_conf.hvuart
               >> uart_conf.currentmax >> uart_conf.currentuart
               >> uart_conf.lvmax >> uart_conf.lvuart;
        return true;
    }

    if (token == "uarttachoscale")
    {
        ParseOne(Parser, uart_conf.tachoscale, 1);
        return true;
    }

    if (token == "uartfeature")
    {
        Parser.getTokens(1);
        std::string firstToken = Parser.peek();

        if (firstToken.find('|') != std::string::npos || firstToken == "none" || uartfeatures_map.count(firstToken))
        {
            for (auto const& x : uartfeatures_map)
            {
                *(x.second) = false;
            }

            std::string key;
            std::stringstream firstTokenStream(firstToken);

            while (!firstTokenStream.eof())
            {
                std::getline(firstTokenStream, key, '|');

                if (uartfeatures_map.count(key))
                {
                    *(uartfeatures_map[key]) = true;
                }
            }
        }
        else
        {
            Parser >> uart_conf.mainenable;
            Parser.getTokens(3);
            Parser >> uart_conf.scndenable
                   >> uart_conf.trainenable
                   >> uart_conf.localenable;
        }

        return true;
    }

    if (token == "uartdebug")
    {
        ParseOne(Parser, uart_conf.debug, 1);
        return true;
    }

    if (token == "uartmainpercentage")
    {
        ParseOne(Parser, uart_conf.mainpercentage, 1);
        return true;
    }
#endif

#ifdef WITH_ZMQ
    if (token == "zmq.address")
    {
        ParseOne(Parser, zmq_address, 1);
        return true;
    }
#endif

#ifdef USE_EXTCAM_CAMERA
    if (token == "extcam.cmd")
    {
        ParseOne(Parser, extcam_cmd, 1);
        return true;
    }

    if (token == "extcam.rec")
    {
        ParseOne(Parser, extcam_rec, 1);
        return true;
    }

    if (token == "extcam.res")
    {
        Parser.getTokens(2);
        Parser >> extcam_res.x >> extcam_res.y;
        return true;
    }
#endif

    return false;
}

bool global_settings::ConfigParseDebug(cParser& Parser, const std::string& token)
{
    if (token == "debugmode")
    {
        ParseOne(Parser, DebugModeFlag);
        return true;
    }

    if (token == "debuglog")
    {
        std::string value;
        ParseOne(Parser, value);

        if (value == "yes")
            iWriteLogEnabled = 3;
        else if (value == "no")
            iWriteLogEnabled = 0;
        else
            iWriteLogEnabled = stol_def(value, 3);

        return true;
    }

    if (token == "multiplelogs")
    {
        ParseOne(Parser, MultipleLogs);
        return true;
    }

    if (token == "shakefactor")
    {
        ParseOne(Parser, ShakingMultiplierBF);
        ParseOne(Parser, ShakingMultiplierRL);
        ParseOne(Parser, ShakingMultiplierUD);
        return true;
    }

    if (token == "logs.filter")
    {
        ParseOne(Parser, DisabledLogTypes);
        return true;
    }

    if (token == "rollfix")
    {
        ParseOne(Parser, bRollFix);
        return true;
    }

    if (token == "loadinglog")
    {
        ParseOne(Parser, loading_log, 1);
        return true;
    }

    if (token == "captureonstart")
    {
        ParseOne(Parser, captureonstart, 1);
        return true;
    }

    if (token == "ddsupperorigin")
    {
        ParseOne(Parser, dds_upper_origin, 1);
        return true;
    }

    if (token == "calibratein")
    {
        Parser.getTokens(1, false);

        int in = 0;
        Parser >> in;

        if ((in < 0) || (in > 5))
            in = 5;

        Parser.getTokens(4, false);
        Parser >> fCalibrateIn[in][0]
               >> fCalibrateIn[in][1]
               >> fCalibrateIn[in][2]
               >> fCalibrateIn[in][3];

        fCalibrateIn[in][4] = 0.0;
        fCalibrateIn[in][5] = 0.0;
        return true;
    }

    if (token == "calibrate5din")
    {
        Parser.getTokens(1, false);

        int in = 0;
        Parser >> in;

        if ((in < 0) || (in > 5))
            in = 5;

        Parser.getTokens(6, false);
        Parser >> fCalibrateIn[in][0]
               >> fCalibrateIn[in][1]
               >> fCalibrateIn[in][2]
               >> fCalibrateIn[in][3]
               >> fCalibrateIn[in][4]
               >> fCalibrateIn[in][5];
        return true;
    }

    if (token == "calibrateout")
    {
        Parser.getTokens(1, false);

        int out = 0;
        Parser >> out;

        if ((out < 0) || (out > 6))
            out = 6;

        Parser.getTokens(4, false);
        Parser >> fCalibrateOut[out][0]
               >> fCalibrateOut[out][1]
               >> fCalibrateOut[out][2]
               >> fCalibrateOut[out][3];

        fCalibrateOut[out][4] = 0.0;
        fCalibrateOut[out][5] = 0.0;
        return true;
    }

    if (token == "calibrate5dout")
    {
        Parser.getTokens(1, false);

        int out = 0;
        Parser >> out;

        if ((out < 0) || (out > 6))
            out = 6;

        Parser.getTokens(6, false);
        Parser >> fCalibrateOut[out][0]
               >> fCalibrateOut[out][1]
               >> fCalibrateOut[out][2]
               >> fCalibrateOut[out][3]
               >> fCalibrateOut[out][4]
               >> fCalibrateOut[out][5];
        return true;
    }

    if (token == "calibrateoutmaxvalues")
    {
        Parser.getTokens(7, false);
        Parser >> fCalibrateOutMax[0]
               >> fCalibrateOutMax[1]
               >> fCalibrateOutMax[2]
               >> fCalibrateOutMax[3]
               >> fCalibrateOutMax[4]
               >> fCalibrateOutMax[5]
               >> fCalibrateOutMax[6];
        return true;
    }

    if (token == "calibrateoutdebuginfo")
    {
        ParseOne(Parser, iCalibrateOutDebugInfo, 1, false);
        return true;
    }

    if (token == "pwm")
    {
        Parser.getTokens(2, false);

        int pwm_out = 0;
        int pwm_no = 0;
        Parser >> pwm_out >> pwm_no;

        iPoKeysPWM[pwm_out] = pwm_no;
        return true;
    }

    return false;
}

void global_settings::ConfigParse(cParser& Parser)
{
    std::string token;

    do
    {
        token.clear();
        Parser.getTokens();
        Parser >> token;

        if (token.empty() || token == "endconfig")
            break;

        if (ConfigParse_gfx(Parser, token))
            continue;

        if (ConfigParseGeneral(Parser, token))
            continue;

        if (ConfigParseAudio(Parser, token))
            continue;

        if (ConfigParseGraphics(Parser, token))
            continue;

        if (ConfigParseInput(Parser, token))
            continue;

        if (ConfigParseSimulation(Parser, token))
            continue;

        if (ConfigParseUI(Parser, token))
            continue;

        if (ConfigParsePython(Parser, token))
            continue;

        if (ConfigParseNetwork(Parser, token))
            continue;

        if (ConfigParseHardware(Parser, token))
            continue;

        if (ConfigParseDebug(Parser, token))
            continue;

        // WriteLog(std::format("Unknown config token: {}", token), logtype::warning, false);

    } while (true);

    FinalizeConfig();
}

bool
global_settings::ConfigParse_gfx( cParser &Parser, std::string_view const Token ) {

    // TODO: move other graphics switches here
    auto tokenparsed { true };

    if (Token == "gfx.shadows.cab.range")
    {
        // shadow render toggle
        Parser.getTokens();
        Parser >> RenderCabShadowsRange;
    }
    else if (Token == "gfx.smoke")
    {
        // smoke visualization toggle
        Parser.getTokens();
        Parser >> Smoke;
    }
    else if (Token == "gfx.smoke.fidelity")
    {
        // smoke visualization fidelity
        float smokefidelity;
        Parser.getTokens();
        Parser >> smokefidelity;
        SmokeFidelity = clamp(smokefidelity, 1.f, 4.f);
    }
    else if (Token == "gfx.resource.sweep")
    {
        Parser.getTokens();
        Parser >> ResourceSweep;
    }
    else if (Token == "gfx.resource.move")
    {
        Parser.getTokens();
        Parser >> ResourceMove;
    }
    else if (Token == "gfx.reflections.framerate")
    {
        auto const updatespersecond{std::abs(Parser.getToken<double>())};
        reflectiontune.update_interval = 1.0 / updatespersecond;
    }
    else if (Token == "gfx.reflections.fidelity")
    {
        Parser.getTokens(1, false);
        Parser >> reflectiontune.fidelity;
        reflectiontune.fidelity = clamp(reflectiontune.fidelity, 0, 2);
    }
    else if (Token == "gfx.reflections.range_instances")
    {
        Parser.getTokens(1, false);
        Parser >> reflectiontune.range_instances;
    }
    else if (Token == "gfx.reflections.range_vehicles")
    {
        Parser.getTokens(1, false);
        Parser >> reflectiontune.range_vehicles;
    }
    else if (Token == "gfx.framebuffer.width")
    {
        Parser.getTokens(1, false);
        Parser >> gfx_framebuffer_width;
    }
    else if (Token == "gfx.framebuffer.height")
    {
        Parser.getTokens(1, false);
        Parser >> gfx_framebuffer_height;
    }
    else if (Token == "gfx.shadowmap.enabled")
    {
        Parser.getTokens(1);
        Parser >> gfx_shadowmap_enabled;
    }
    else if (Token == "gfx.envmap.enabled")
    {
        Parser.getTokens(1);
        Parser >> gfx_envmap_enabled;
    }
    else if (Token == "gfx.postfx.motionblur.enabled")
    {
        Parser.getTokens(1);
        Parser >> gfx_postfx_motionblur_enabled;
    }
    else if (Token == "gfx.postfx.motionblur.shutter")
    {
        Parser.getTokens(1);
        Parser >> gfx_postfx_motionblur_shutter;
    }
    else if (Token == "gfx.postfx.motionblur.format")
    {
        Parser.getTokens(1);
        std::string value;
        Parser >> value;
        if (value == "rg16f")
            gfx_postfx_motionblur_format = GL_RG16F;
        else if (value == "rg32f")
            gfx_postfx_motionblur_format = GL_RG32F;
    }
    else if (Token == "gfx.postfx.chromaticaberration.enabled")
    {
        Parser.getTokens(1);
        Parser >> gfx_postfx_chromaticaberration_enabled;
    }
    else if (Token == "gfx.format.color")
    {
        Parser.getTokens(1);
        std::string value;
        Parser >> value;
        if (value == "rgb8")
            gfx_format_color = GL_RGB8;
        else if (value == "rgb16f")
            gfx_format_color = GL_RGB16F;
        else if (value == "rgb32f")
            gfx_format_color = GL_RGB32F;
        else if (value == "r11f_g11f_b10f")
            gfx_format_color = GL_R11F_G11F_B10F;
    }
    else if (Token == "gfx.format.depth")
    {
        Parser.getTokens(1);
        std::string value;
        Parser >> value;
        if (value == "z16")
            gfx_format_depth = GL_DEPTH_COMPONENT16;
        else if (value == "z24")
            gfx_format_depth = GL_DEPTH_COMPONENT24;
        else if (value == "z32")
            gfx_format_depth = GL_DEPTH_COMPONENT32;
        else if (value == "z32f")
            gfx_format_depth = GL_DEPTH_COMPONENT32F;
    }
    else if (Token == "gfx.skiprendering")
    {
        Parser.getTokens(1);
        Parser >> gfx_skiprendering;
    }
    else if (Token == "gfx.skippipeline")
    {
        Parser.getTokens(1);
        Parser >> gfx_skippipeline;
    }
    else if (Token == "gfx.extraeffects")
    {
        Parser.getTokens(1);
        Parser >> gfx_extraeffects;
    }
    else if (Token == "gfx.usegles")
    {
        Parser.getTokens(1);
        Parser >> gfx_usegles;
    }
    else if (Token == "gfx.shadergamma")
    {
        Parser.getTokens(1);
        Parser >> gfx_shadergamma;
    }
    else if (Token == "gfx.drawrange.factor.max")
    {
        Parser.getTokens(1);
        Parser >> gfx_distance_factor_max;
    }
    else if (Token == "gfx.shadow.angle.min")
    {
        Parser.getTokens(1);
        Parser >> gfx_shadow_angle_min;
        // variable internally uses negative values, but is presented as positive in settings
        // so it's likely it'll be supplied as positive number by external launcher
        if( gfx_shadow_angle_min > 0 ) {
            gfx_shadow_angle_min *= -1;
        }
        gfx_shadow_angle_min = clamp(gfx_shadow_angle_min, -1.f, -0.2f);
    }
    else if (Token == "gfx.shadow.rank.cutoff")
    {
        Parser.getTokens(1);
        Parser >> gfx_shadow_rank_cutoff;
        gfx_shadow_rank_cutoff = clamp(gfx_shadow_rank_cutoff, 1, 3);
    }
    else
    {
        tokenparsed = false;
    }

    return tokenparsed;
}

void
global_settings::export_as_text( std::ostream &Output ) const {

    export_as_text( Output, "sceneryfile", SceneryFile );
    export_as_text( Output, "humanctrlvehicle", local_start_vehicle );
    export_as_text( Output, "fieldofview", FieldOfView );
    export_as_text( Output, "width", window_size.x );
    export_as_text( Output, "height", window_size.y );
    export_as_text( Output, "targetfps", targetfps );
    export_as_text( Output, "basedrawrange", BaseDrawRange );
    export_as_text( Output, "fullscreen", bFullScreen );
    export_as_text( Output, "fullscreenmonitor", fullscreen_monitor );
    export_as_text( Output, "fullscreenwindowed", fullscreen_windowed );
    export_as_text( Output, "vsync", VSync );
    // NOTE: values are changed dynamically during simulation. cache initial settings and export instead
    if( FreeFlyModeFlag ) {
        Output
            << "freefly yes "
            << FreeCameraInit[ 0 ].x << " "
            << FreeCameraInit[ 0 ].y << " "
            << FreeCameraInit[ 0 ].z << "\n";
    }
    else {
        export_as_text( Output, "freefly", FreeFlyModeFlag );
    }
    export_as_text( Output, "wireframe", bWireFrame );
    export_as_text( Output, "debugmode", DebugModeFlag );
    export_as_text( Output, "soundenabled", bSoundEnabled );
    export_as_text( Output, "sound.openal.renderer", AudioRenderer );
    export_as_text( Output, "sound.volume", AudioVolume );
    export_as_text( Output, "sound.volume.radio", DefaultRadioVolume );
    export_as_text( Output, "sound.volume.vehicle", VehicleVolume );
    export_as_text( Output, "sound.volume.positional", EnvironmentPositionalVolume );
    export_as_text( Output, "sound.volume.ambient", EnvironmentAmbientVolume );
    export_as_text( Output, "physicslog", WriteLogFlag );
    export_as_text( Output, "fullphysics", FullPhysics );
    export_as_text( Output, "debuglog", iWriteLogEnabled );
    export_as_text( Output, "multiplelogs", MultipleLogs );
    export_as_text( Output, "logs.filter", DisabledLogTypes );
    Output
        << "mousescale "
        << fMouseXScale << " "
        << fMouseYScale << "\n";
    export_as_text( Output, "mousecontrol", InputMouse );
    export_as_text( Output, "enabletraction", bEnableTraction );
    export_as_text( Output, "loadtraction", bLoadTraction );
    export_as_text( Output, "friction", fFriction );
    export_as_text( Output, "livetraction", bLiveTraction );
    export_as_text( Output, "skyenabled", asSky );
    export_as_text( Output, "defaultext", szDefaultExt );
    export_as_text( Output, "newaircouplers", bnewAirCouplers );
    export_as_text( Output, "anisotropicfiltering", AnisotropicFiltering );
    export_as_text( Output, "usevbo", bUseVBO );
    export_as_text( Output, "feedbackmode", iFeedbackMode );
    export_as_text( Output, "feedbackport", iFeedbackPort );
    export_as_text( Output, "multiplayer", iMultiplayer );
	export_as_text( Output, "isolatedtrainnumber", bIsolatedTrainName);
    export_as_text( Output, "maxtexturesize", iMaxTextureSize );
    export_as_text( Output, "maxcabtexturesize", iMaxCabTextureSize );
    export_as_text( Output, "movelight", fMoveLight );
    export_as_text( Output, "dynamiclights", DynamicLightCount );
    if( std::isnormal( ScenarioTimeOverride ) ) {
        export_as_text( Output, "scenario.time.override", ScenarioTimeOverride );
    }
    export_as_text( Output, "scenario.time.offset", ScenarioTimeOffset );
    export_as_text( Output, "scenario.time.current", ScenarioTimeCurrent );
    export_as_text( Output, "scenario.weather.temperature", AirTemperature );
    export_as_text( Output, "ai.trainman", AITrainman );
    export_as_text( Output, "scalespeculars", ScaleSpecularValues );
    export_as_text( Output, "gfxrenderer", GfxRenderer );
    export_as_text( Output, "shadows", RenderShadows );
    Output
        << "shadowtune "
        << shadowtune.map_size << " "
        << 0 << " "
        << shadowtune.range << " "
        << 0 << "\n";
    export_as_text( Output, "gfx.shadows.cab.range", RenderCabShadowsRange );
    export_as_text( Output, "gfx.smoke", Smoke );
    export_as_text( Output, "gfx.smoke.fidelity", SmokeFidelity );
    export_as_text( Output, "smoothtraction", bSmoothTraction );
    export_as_text( Output, "splinefidelity", SplineFidelity );
    export_as_text( Output, "rendercab", render_cab );
    export_as_text( Output, "createswitchtrackbeds", CreateSwitchTrackbeds );
    export_as_text( Output, "gfx.resource.sweep", ResourceSweep );
    export_as_text( Output, "gfx.resource.move", ResourceMove );
    export_as_text( Output, "gfx.reflections.framerate", 1.0 / reflectiontune.update_interval );
    export_as_text( Output, "gfx.reflections.fidelity", reflectiontune.fidelity );
    export_as_text( Output, "timespeed", fTimeSpeed );
    export_as_text( Output, "multisampling", iMultisampling );
    export_as_text( Output, "latitude", fLatitudeDeg );
    export_as_text( Output, "convertmodels", iConvertModels );
    export_as_text( Output, "file.binary.terrain", file_binary_terrain );
    export_as_text( Output, "inactivepause", bInactivePause );
    export_as_text( Output, "slowmotion", iSlowMotionMask );
    export_as_text( Output, "hideconsole", bHideConsole );
    export_as_text( Output, "rollfix", bRollFix );
    export_as_text( Output, "fpsaverage", fFpsAverage );
    export_as_text( Output, "fpsdeviation", fFpsDeviation );
    for( auto idx = 0; idx < 6; ++idx ) {
        Output
            << "calibrate5din "
            << idx << " "
            << fCalibrateIn[ idx ][ 0 ] << " "
            << fCalibrateIn[ idx ][ 1 ] << " "
            << fCalibrateIn[ idx ][ 2 ] << " "
            << fCalibrateIn[ idx ][ 3 ] << " "
            << fCalibrateIn[ idx ][ 4 ] << " "
            << fCalibrateIn[ idx ][ 5 ] << "\n";
    }
    for( auto idx = 0; idx < 6; ++idx ) {
        Output
            << "calibrate5dout "
            << idx << " "
            << fCalibrateOut[ idx ][ 0 ] << " "
            << fCalibrateOut[ idx ][ 1 ] << " "
            << fCalibrateOut[ idx ][ 2 ] << " "
            << fCalibrateOut[ idx ][ 3 ] << " "
            << fCalibrateOut[ idx ][ 4 ] << " "
            << fCalibrateOut[ idx ][ 5 ] << "\n";
    }
    Output
        << "calibrateoutmaxvalues "
        << fCalibrateOutMax[ 0 ] << " "
        << fCalibrateOutMax[ 1 ] << " "
        << fCalibrateOutMax[ 2 ] << " "
        << fCalibrateOutMax[ 3 ] << " "
        << fCalibrateOutMax[ 4 ] << " "
        << fCalibrateOutMax[ 5 ] << " "
        << fCalibrateOutMax[ 6 ] << "\n";
    export_as_text( Output, "calibrateoutdebuginfo", iCalibrateOutDebugInfo );
    for( auto idx = 0; idx < 7; ++idx ) {
        Output
            << "pwm "
            << idx << " "
            << iPoKeysPWM[ idx ] << "\n";
    }
    export_as_text( Output, "brakestep", fBrakeStep );
    export_as_text( Output, "joinduplicatedevents", bJoinEvents );
    export_as_text( Output, "hiddenevents", iHiddenEvents );
    export_as_text( Output, "pause", ( iPause & 1 ) != 0 );
	export_as_text(Output, "priorityLoadText3D", priorityLoadText3D);
    export_as_text( Output, "lang", asLang );
    export_as_text( Output, "python.updatetime", PythonScreenUpdateRate );
    Output
        << "uitextcolor "
        << UITextColor.r * 255 << " "
        << UITextColor.g * 255 << " "
        << UITextColor.b * 255 << "\n";
    export_as_text( Output, "ui.bg.opacity", UIBgOpacity );
    export_as_text( Output, "input.gamepad", InputGamepad );
#ifdef WITH_UART
    if( uart_conf.enable ) {
        Output
            << "uart "
            << uart_conf.port << " "
            << uart_conf.baud << " "
            << uart_conf.updatetime << "\n";
    }
    Output
        << "uarttune "
        << uart_conf.mainbrakemin << " "
        << uart_conf.mainbrakemax << " "
        << uart_conf.localbrakemin << " "
        << uart_conf.localbrakemax << " "
        << uart_conf.tankmax << " "
        << uart_conf.tankuart << " "
        << uart_conf.pipemax << " "
        << uart_conf.pipeuart << " "
        << uart_conf.brakemax << " "
        << uart_conf.brakeuart << " "
        << uart_conf.pantographmax << " "
        << uart_conf.pipemax << " "
        << uart_conf.hvmax << " "
        << uart_conf.hvuart << " "
        << uart_conf.currentmax << " "
        << uart_conf.currentuart << " "
        << uart_conf.lvmax << " "
        << uart_conf.lvuart << "\n";
    export_as_text( Output, "uarttachoscale", uart_conf.tachoscale );

    std::vector<std::string> enabled_uartfeatures;
    for(auto const &x : uartfeatures_map) {
        if(*(x.second)) {
            enabled_uartfeatures.push_back(x.first);
        }
    }
    Output << "uartfeature ";
    if(enabled_uartfeatures.empty()) {
        Output << "none\n";
    } else {
        for(auto const &feature : enabled_uartfeatures) {
            Output << feature;
            if(&feature != &enabled_uartfeatures.back()) {
                Output << "|";
            }
        }
        Output << "\n";
    }
    export_as_text( Output, "uartdebug", uart_conf.debug );
#endif

    export_as_text( Output, "extcam.cmd", extcam_cmd );
    export_as_text( Output, "extcam.rec", extcam_rec );
    Output
        << "extcam.res "
        << extcam_res.x << " "
        << extcam_res.y << "\n";

    export_as_text( Output, "compresstex", compress_tex );
    export_as_text( Output, "gfx.framebuffer.width", gfx_framebuffer_width );
    export_as_text( Output, "gfx.framebuffer.height", gfx_framebuffer_height );
    export_as_text( Output, "gfx.shadowmap.enabled", gfx_shadowmap_enabled );
    // TODO: export fpslimit
    export_as_text( Output, "randomseed", Global.random_seed );
    export_as_text( Output, "gfx.envmap.enabled", gfx_envmap_enabled );
    export_as_text( Output, "gfx.postfx.motionblur.enabled", gfx_postfx_motionblur_enabled );
    export_as_text( Output, "gfx.postfx.motionblur.shutter", gfx_postfx_motionblur_shutter );
    // TODO: export gfx_postfx_motionblur_format
    export_as_text( Output, "gfx.postfx.chromaticaberration.enabled", gfx_postfx_chromaticaberration_enabled );
    // TODO: export gfx_format_color
    // TODO: export gfx_format_depth
    export_as_text( Output, "gfx.skiprendering", gfx_skiprendering );
    export_as_text( Output, "gfx.skippipeline", gfx_skippipeline );
    export_as_text( Output, "gfx.extraeffects", gfx_extraeffects );
    export_as_text( Output, "gfx.shadergamma", gfx_shadergamma );
    export_as_text( Output, "gfx.shadow.angle.min", gfx_shadow_angle_min );
    export_as_text( Output, "gfx.shadow.rank.cutoff", gfx_shadow_rank_cutoff );
    export_as_text( Output, "python.enabled", python_enabled );
    export_as_text( Output, "python.threadedupload", python_threadedupload );
    export_as_text( Output, "python.uploadmain", python_uploadmain );
    export_as_text( Output, "python.mipmaps", python_mipmaps );
    export_as_text( Output, "async.trainThreads", trainThreads );
    for( auto const &server : network_servers ) {
        Output
            << "network.server "
            << server.first << " " << server.second << "\n";
    }
    if( network_client ) {
        Output
            << "network.client "
            << network_client->first << " " << network_client->second << "\n";
    }
    export_as_text( Output, "execonexit", exec_on_exit );
}

template <>
void
global_settings::export_as_text( std::ostream &Output, std::string const Key, std::string const &Value ) const {

    if( Value.empty() ) { return; }

    if( contains( Value, ' ' ) ) {
        Output << Key << " \"" << Value << "\"\n";
    }
    else {
        Output << Key << " " << Value << "\n";
    }
}

template <> void global_settings::export_as_text(std::ostream &Output, std::string const Key, bool const &Value) const
{

	Output << Key << " " << (Value ? "yes" : "no") << "\n";
}

global_settings &GetGlobalSettings()
{
	static global_settings global{};
	return global;
}
