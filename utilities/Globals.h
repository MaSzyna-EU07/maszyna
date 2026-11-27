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
#include "vehicle/Camera.h"
#include "utilities/Float3d.h"
#include "rendering/light.h"
#include "utilities/utilities.h"
#include "utilities/motiontelemetry.h"
#include <map>
#include <optional>

#ifdef WITH_UART
#include "utilities/uart.h"
#endif
#ifdef WITH_ZMQ
#include "input/zmq_input.h"
#endif

// where the keys global_settings::ConfigParse reads come from
enum class config_source {
	ini, // eu07.ini, and the file given with -ini
	scenery // a config block of a scenery (.scn, .inc), read as it loads
};

struct global_settings {
// members
    // data items
    // TODO: take these out of the settings

    std::chrono::steady_clock::time_point startTimestamp;

	/// <summary>
	/// Mapa z watkami w formacie <std::string nazwa, std::thread watek>
	/// </summary>
	std::map<std::string, std::thread> threads = {};
    bool shiftState{ false }; //m7todo: brzydko
    bool ctrlState{ false };
    bool altState{ false };
    // Simulation randomness, never used for anything security related: no keys, tokens or
    // credentials come out of these. Reproducibility is a requirement here, which is why both
    // are plain mt19937 -- the run's seed can be written down and passed on, and it travels in
    // server_hello so that every client in a session replays the same sequence. A generator
    // that cannot be seeded to a known state would break both of those.
    std::mt19937 random_engine; // NOSONAR shared run seed, kept in sync across the network
    std::mt19937 local_random_engine; // NOSONAR re-seeded from a true random source in init_modes,
                                      // drives purely local cosmetics (sound pitch, editor placement)
    bool ready_to_load{ false };
    std::time_t starting_timestamp = 0; // starting time, in local timezone
    uint32_t random_seed = 0;
    // whether random_seed was given (-seed, randomseed, the server of a network session) or is still to be drawn:
    // 0 is a seed like any other, so it can't stand for none
    bool random_seed_given{ false };
    TCamera pCamera; // parametry kamery
    TCamera pDebugCamera;
    std::array<glm::dvec3, 10> FreeCameraInit; // pozycje kamery
    std::array<glm::vec3, 10> FreeCameraInitAngle;
    int iCameraLast{ -1 };
    int iSlowMotion{ 0 }; // info o malym FPS: 0-OK, 1-wyłączyć multisampling, 3-promień 1.5km, 7-1km
    basic_light DayLight;
    float SunAngle{ 0.f }; // angle of the sun relative to horizon
	int trainThreads{0};
    double fLuminance{ 1.0 }; // jasność światła do automatycznego zapalania // TODO: Why double?
    double fTimeAngleDeg{ 0.0 }; // godzina w postaci kąta
    float fClockAngleDeg[ 6 ]; // kąty obrotu cylindrów dla zegara cyfrowego
    std::string LastGLError;
    float ZoomFactor{ 1.f }; // determines current camera zoom level. TODO: move it to the renderer
    bool CabWindowOpen{ false }; // controls sound attenuation between cab and outside
    bool ControlPicking{ true }; // indicates controls pick mode is active
    bool DLFont{ false }; // switch indicating presence of basic font
    bool bActive{ true }; // czy jest aktywnym oknem
    int iPause{ 0 }; // globalna pauza ruchu: b0=start,b1=klawisz,b2=tło,b3=lagi,b4=wczytywanie
    float AirTemperature{ 15.f };
    std::string asCurrentSceneryPath{ "scenery/" };
    std::string asCurrentTexturePath{ paths::textures };
    std::string asCurrentDynamicPath;
    int CurrentMaxTextureSize{ 4096 };
    bool UpdateMaterials{ true };
    // settings
    // filesystem
    bool bLoadTraction{ true };
    bool bSmoothTraction{ true };
    bool CatenaryProcedural{ true }; // przęsła z dopiskiem "type" rysowane wg typu sieci (lina nośna, wieszaki i Y liczone)
    bool CreateSwitchTrackbeds{ true };
    bool Frogs{ false }; // whether the rails are cut where they cross other rails into frogs, wing rails and check rails; off, they're laid whole as before
    std::string szTexturesTGA{ ".tga" }; // lista tekstur od TGA
    std::string szTexturesDDS{ ".dds" }; // lista tekstur od DDS
    std::string szDefaultExt{ szTexturesDDS };
	std::string SceneryFile;
    std::string local_start_vehicle{ "EU07-424" };
    std::string editor_enter_vehicle; // vehicle placed in the editor, to be entered when the driving goes on
    bool editor_session{ false }; // scenery was opened for editing (-edit): loader keeps track of scenery layers and hands over to the editor
    // integration test mode (--test, eu07/maszyna#541); the limits can be set in the scenery's config block (test.*)
    struct test_settings {
        bool enabled{ false };
        double timeout{ 3600.0 }; // [s] of simulation time; no result by then fails the test
        double standstill{ 300.0 }; // [s] the tested trains standing still this long ends the test; 0 = never
        bool pass_on_standstill{ false }; // the standstill end passes the test instead of failing it
        bool fail_if_stop_overran{ true }; // a train passing a signal at stop ends the test as failed; with false it's only logged
        bool no_movement_before_pass{ true }; // a pass waits until the tested trains stand still; with false it ends the test at once
        std::vector<std::string> vehicles; // test.vehicle: the trains under test are the consists of these; none = every vehicle
        bool timeout_from_command_line{ false }; // --test-timeout given, the scenery doesn't override it
        std::optional<double> wall_timeout; // [s] of real time since the start; none = test.timeout + time for the loading, 0 = no limit
        bool wall_timeout_from_command_line{ false }; // --test-wall-timeout given, the scenery doesn't override it
    };
    test_settings test;
    bool editor_tracks{ false }; // F3 track workspace: draw and pick tracks without the scenery draw-range cap
    int iConvertModels{ 0 }; // tworzenie plików binarnych
    int iConvertIndexRange{ 1000 }; // range of duplicate vertex scan
    bool file_binary_terrain{ true }; // enable binary terrain (de)serialization
    int file_binary_terrain_checksum{ 1 }; // check of a binary terrain file (.btf) against its text: 0 = none, 1 = length of the text, 2 = length and checksum
	bool file_binary_terrain_state{true};
	int file_binary_terrain_skipped{0}; // number of terrain includes left out by the parser during the current scenery load
    // logs
	bool priorityLoadText3D{false}; // ladowanie T3D priorytetowo
    int iWriteLogEnabled{ 3 }; // maska bitowa: 1-zapis do pliku, 2-okienko, 4-nazwy torów
    bool MultipleLogs{ false };
    bool crashdump_full{ false }; // crash dump with the whole memory of the process instead of the small one
    // the black box of the session (blackbox_doc.md): "blackbox yes" writes it whatever the physics logs, "blackbox no"
    // never, and with no entry it is written along with the physics logs
    std::optional<bool> BlackBox;
    bool ShowSystemConsole{ false }; // toggle visibility of the OS console window with log output
    unsigned int DisabledLogTypes{ 0 };
    bool ParserLogIncludes{ true };
    // simulation
    bool RealisticControlMode{ false }; // controls ability to steer the vehicle from outside views
    // odometer.expiry: the saved odometer state of a vehicle not run for longer than this many days is dropped from the
    // store (vehicle/odometer.h); 0 = never
    int odometer_expiry{ 365 };
    bool bEnableTraction{ true };
    float fFriction{ 1.f }; // mnożnik tarcia - KURS90
    float FrictionWeatherFactor{ 1.f };
    bool bLiveTraction{ true };
    float Overcast{ 0.1f }; // NOTE: all this weather stuff should be moved elsewhere
    glm::vec3 FogColor = { 0.6f, 0.7f, 0.8f };
    float fFogEnd{ 7500 };
    float fTurbidity{ 128 };
    std::string Season{}; // season of the year, based on simulation date
    std::string Weather{ "cloudy:" }; // current weather
    std::string Period{}; // time of the day, based on sun position
    bool FullPhysics{ true }; // full calculations performed for each simulation step
    bool physics_sleepheld{ true }; // vehicles standing still and held by their brakes let their physics sleep, see TMoverParameters::accelerating()
    bool physics_roadlod{ true }; // road vehicles far from the camera go by one step of the physics in a frame, see vehicle_table::update()
    float physics_pipewake{ 0.001f }; // bar: air flowing along a pipe wakes the physics of a vehicle when it changes its pressure by more than this. 0: any flow does
    bool performance_multithreading{ true }; // work split over the logical cores of the machine (worker_pool), as many as each job is worth
    bool debug_memorystats{ false }; // the memory of the parts of the program in the debug panel, with new and delete counted (start_heap_counting()). slows the loading down
    bool bnewAirCouplers{ true };
    float fMoveLight{ 0.f }; // numer dnia w roku albo -1
    bool FakeLight{ false }; // toggle between fixed and dynamic daylight
    double fTimeSpeed{ 1.0 }; // przyspieszenie czasu, zmienna do testów
	double default_timespeed { 1.0 }; // timescale loaded from config
    double fLatitudeDeg{ 52.0 }; // szerokość geograficzna
    float ScenarioTimeOverride{ std::numeric_limits<float>::quiet_NaN() }; // requested scenario start time
    float ScenarioTimeOffset{ 0.f }; // time shift (in hours) applied to train timetables
    bool ScenarioTimeCurrent{ false }; // automatic time shift to match scenario time with local clock
    bool bInactivePause{ true }; // automatyczna pauza, gdy okno nieaktywne
    int iSlowMotionMask{ -1 }; // maska wyłączanych właściwości
    bool bHideConsole{ false }; // hunter-271211: ukrywanie konsoli
    bool bRollFix{ true }; // czy wykonać przeliczanie przechyłki
    bool bJoinEvents{ false }; // czy grupować eventy o tych samych nazwach
    int iHiddenEvents{ 1 }; // czy łączyć eventy z torami poprzez nazwę toru
    bool AITrainman{ true }; // virtual assistant performing consist coupling/decoupling and other maintenance tasks
    bool AIBrakeSettingCw1{ true }; // ai.brakesetting.cw1: the AI wagon inspector sets the brakes (G/P/R/Mg) by the instruction Cw-1, no: as before eu07/maszyna#350
    // ui
    int PythonScreenUpdateRate{ 200 }; // delay between python-based screen updates, in milliseconds
    int iTextMode{ 0 }; // tryb pracy wyświetlacza tekstowego
    glm::vec4 UITextColor{ glm::vec4( 225.f / 255.f, 225.f / 255.f, 225.f / 255.f, 1.f ) }; // base color of UI text
    float UIBgOpacity{ 0.94f }; // opacity of ui windows
    std::string asLang{ "pl" }; // domyślny język - http://tools.ietf.org/html/bcp47
    // gfx
    glm::ivec2 window_size; // main window size in platform-specific virtual pixels
    glm::ivec2 cursor_pos; // cursor position in platform-specific virtual pixels
    glm::ivec2 fb_size; // main window framebuffer size
	float ShakingMultiplierBF {1.f}; // mnożnik bujania kamera przod/tyl
	float ShakingMultiplierRL {1.f}; // mnożnik bujania kamera lewo/prawo
	float ShakingMultiplierUD {1.f}; // mnożnik bujania kamera gora/dol
    float fDistanceFactor{ 1.f }; // baza do przeliczania odległości dla LoD
    float targetfps{ 0.0f };
    bool bFullScreen{ false };
    bool VSync{ false };
    bool bWireFrame{ false };
    bool bAdjustScreenFreq{ true };
    float BaseDrawRange{ 2500.f };
    int DynamicLightCount{ 7 };
    bool ScaleSpecularValues{ true };
    std::string GfxRenderer{ "default" };
    bool RenderShadows{ true };
	bool applicationQuitOrder{false};
    int RenderCabShadowsRange{ 0 };
    struct shadowtune_t {
        unsigned int map_size{ 2048 };
        float range{ 250.f };
    } shadowtune;
    struct reflectiontune_t {
        // how often the environment map (the sky and the clouds around the camera, the reflection of what the screen space
        // reflections and the ray tracing don't find) is drawn anew
        double update_interval{ 300.0 };
        // what the ray traced reflections of the vulkan renderer hold (gfx.raytracing): 0: sections, 1: +static models, 2: +vehicles
        int fidelity{ 2 };
        float range_instances{ 750.f };
        float range_vehicles{ 250.f };
    } reflectiontune;
    float AnisotropicFiltering{ 8.f }; // requested level of anisotropic filtering. TODO: move it to renderer object
    float FieldOfView{ 45.f }; // vertical field of view for the camera. TODO: move it to the renderer
    bool EditorOrtho{ false };
    float EditorOrthoExtent{ 150.f };
    int EditorOrthoTracks{ 0 }; // the railway in the top view of the editor: 0 whole, 1 without the ballast, 2 the rails alone
    // the free camera of the editor: its speed set in the View menu, the Ctrl multiplier of it, and in the top view whether it goes as far
    // as the view reaches, so the map moves across the screen as fast zoomed in as out
    bool EditorCameraOn{ false }; // the editor is on: its camera goes by the settings below
    double EditorCameraSpeed{ 1.0 };
    double EditorCameraFast{ 30.0 };
    bool EditorCameraZoomed{ false };
    GLint iMaxTextureSize{ 4096 }; // maksymalny rozmiar tekstury
    GLint iMaxCabTextureSize{ 4096 }; // largest allowed texture in vehicle cab
    int iMultisampling{ 2 }; // tryb antyaliasingu: 0=brak,1=2px,2=4px,3=8px,4=16px
    // gfx.antialiasing: off, msaax2/x4/x8 (the samples go to iMultisampling), taa (one sample, with the history of the last frames;
    // iMultisampling 0) or dlss (nvidia dlss of the vulkan renderer, one sample: the image of gfx.buffer.percent upscaled to the
    // window; dlaa is dlss at the full size of the window). the old key multisampling sets msaa or off. read at the start; where dlss
    // can't be had at the start (the opengl renderers, no nvidia rtx, a build without WITH_DLSS) the renderer takes taa instead, a
    // failure of dlss later leaves the scene stretched to the window without anti-aliasing
    enum class antialiasing_mode { off, msaa, taa, dlss };
    antialiasing_mode gfx_antialiasing{ antialiasing_mode::msaa };
    // the temporal anti-aliasing is on: asked for, and the post-processing which does it is there
    bool gfx_taa() const { return ( gfx_antialiasing == antialiasing_mode::taa ) && ( false == gfx_skippipeline ); }
    // dlss is on: asked for, with the post-processing (the renderer sets gfx_antialiasing to taa where it can't have it)
    bool gfx_dlss() const { return ( gfx_antialiasing == antialiasing_mode::dlss ) && ( false == gfx_skippipeline ); }
    // gfx.buffer.percent: the size of the image the scene of the main window is drawn to, as a part of the size of the window
    // (0.5: half its width and half its height). dlss upscales it to the window, the other modes stretch it. read at the start
    float gfx_buffer_scale{ 1.f };
    // gfx.antialiasing dlaa: dlss at the full size of the window whatever gfx.buffer.percent says
    bool gfx_dlss_native{ false };
    // gfx.dlss.preset: default, j, k, l or m, the value of NVSDK_NGX_DLSS_Hint_Render_Preset (0, 10, 11, 12, 13)
    int gfx_dlss_preset{ 0 };
    // gfx.dlss.autoexposure: dlss works the exposure out itself instead of taking the one of the tonemapping
    bool gfx_dlss_autoexposure{ false };
    // gfx.dlss.lodbias: added to the bias of the mipmaps of the textures of the scene dlss is given (log2(render / window) - 1)
    float gfx_dlss_lodbias{ 0.f };
    // gfx.dlss.developer: the development library of dlss (ngx_dev/ next to the program) with its indicator and debug keys
    bool gfx_dlss_developer{ false };
    // gfx.framegeneration off, x2, x3, x4: the frames shown for each frame drawn (dlss frame generation of the vulkan renderer, with
    // gfx.antialiasing dlss or dlaa; x3 and x4 where the gpu has multi frame generation, else x2). 1: off. read at the start
    int gfx_framegeneration{ 1 };
    // gfx.reflex off, on, boost: nvidia reflex of the vulkan renderer (VK_NV_low_latency2), the frame begun as late as the gpu
    // allows; on at least with the frame generation. read at the start
    int gfx_reflex{ 0 };
    float SplineFidelity{ 1.f }; // determines segment size during conversion of splines to geometry
    float SleeperDistance{ 250.f }; // max distance (in meters) at which per-track sleeper models are still drawn; 0 disables sleeper rendering entirely
    std::string Sleepers3D; // model of the sleepers and switch timbers laid where the simulator makes the ballast; empty: none
    std::string Sleepers3DPlain { "quark/podklad1" }; // model of the sleepers of a track running alone, those of Sleepers3D being the timbers of switches and crossings; empty: the timbers there too
    std::string Sleepers3DPlainSkin { "quark/podklad_betonowy1" };
    bool Sleepers3DBed { true }; // whether the ballast is laid under the tracks with 3D sleepers
    float Sleepers3DPlainSeat { 0.24f }; // how high the rails sit on the plain sleeper above the origin of its model; below 0: on its top
    bool Smoke{ true }; // toggles smoke simulation and visualization
    float SmokeFidelity{ 1.f }; // determines amount of generated smoke particles
    bool ResourceSweep{ true }; // gfx resource garbage collection
    bool ResourceMove{ false }; // gfx resources are moved between cpu and gpu side instead of sending a copy
    bool gfx_texture_streaming{ false }; // data of dds textures is read by worker threads, the rest of the loading doesn't wait for it
    float gfx_texture_releasedistance{ 0.f }; // gl textures of vehicles farther than this many draw ranges are released. 0 = off
    bool compress_tex{ true }; // all textures are compressed on gpu side
    std::string asSky{ "1" };
    float fFpsAverage{ 0.f }; // oczekiwana wartosć FPS
    float fFpsDeviation{ 5.f }; // odchylenie standardowe FPS
    double fFpsMin{ 30.0 }; // dolna granica FPS, przy której promień scenerii będzie zmniejszany
    double fFpsMax{ 65.0 }; // górna granica FPS, przy której promień scenerii będzie zwiększany
    // audio
    bool bSoundEnabled{ true };
    float AudioVolume{ 1.f };
    float DefaultRadioVolume{ 0.75f };
    float VehicleVolume{ 1.0f };
    float EnvironmentPositionalVolume{ 1.0f };
    float EnvironmentAmbientVolume{ 1.0f };
    int audio_max_sources = 30;
    std::string AudioRenderer;
    bool AudioAsyncLoad{ false }; // sound files are decoded by worker threads
    // broadcast radio receiver
    bool fmradio_enabled{ true }; // allows cab radio receivers to play internet radio streams
    bool fmradio_cleansound{ false }; // disables simulation of the receiver speaker
    std::string fmradio_stations{ "fmradio.txt" }; // list of stations available to the receivers
    // OpenAL Soft device and mixing options; the defaults leave the choice to the library, as before they existed
    std::string AudioOutputMode{ "auto" }; // sound.outputmode: auto, mono, stereo, stereo-basic, uhj, headphones, quad, 5.1, 6.1, 7.1
    std::string AudioHrtf{ "auto" }; // sound.hrtf: auto, on, off
    std::string AudioHrtfProfile; // sound.hrtf.profile: name of the HRTF data set, empty for the library's choice
    std::string AudioResampler; // sound.resampler: name of the resampler, empty for the library's default
    std::string AudioLimiter{ "auto" }; // sound.limiter: auto, on, off
    int AudioFrequency{ 0 }; // sound.frequency: mixing rate in Hz, 0 for the device's
    float AudioDoppler{ 0.25f }; // sound.doppler: strength of the Doppler effect
    bool AudioLowpass{ false }; // sound.efx.lowpass: soundproofing also muffles the high frequencies
    float AudioAirAbsorption{ 0.f }; // sound.efx.airabsorption: 0-10, high frequencies fade with distance
    // input
    float fMouseXScale{ 1.5f };
    float fMouseYScale{ 0.2f };
    int iFeedbackMode{ 1 }; // tryb pracy informacji zwrotnej
    int iFeedbackPort{ 0 }; // dodatkowy adres dla informacji zwrotnych
    bool InputGamepad{ true }; // whether gamepad support is enabled
    bool InputMouse{ true }; // whether control pick mode can be activated
    bool ConsistViews{ true }; // whether [Shift]+[F4] goes on to the orbit, shunting, coupling and trackside views of the train (eu07/maszyna#551)
    double fBrakeStep { 1.0 }; // krok zmiany hamulca innych niż FV4a dla klawiszy [Num3] i [Num9]
    double brake_speed { 3.0 }; // prędkość przesuwu hamulca dla FV4a
    // parametry kalibracyjne wejść z pulpitu
    double fCalibrateIn[ 6 ][ 6 ] = {
        { 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0 } };
    // parametry kalibracyjne wyjść dla pulpitu
    double fCalibrateOut[ 7 ][ 6 ] = {
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 } };
    // wartości maksymalne wyjść dla pulpitu
    double fCalibrateOutMax[ 7 ] = {
        0, 0, 0, 0, 0, 0, 0 };
    int iCalibrateOutDebugInfo{ -1 }; // numer wyjścia kalibrowanego dla którego wyświetlać informacje podczas kalibracji
    int iPoKeysPWM[ 7 ] = { 0, 1, 2, 3, 4, 5, 6 }; // numery wejść dla PWM
#ifdef WITH_UART
    uart_input::conf_t uart_conf;
    std::map<std::string, bool *> uartfeatures_map = {
        {"main", &uart_conf.mainenable},
        {"scnd", &uart_conf.scndenable},
        {"train", &uart_conf.trainenable},
        {"local", &uart_conf.localenable},
        {"radiovolume", &uart_conf.radiovolumeenable},
        {"radiochannel", &uart_conf.radiochannelenable},
        {"dynamicbrake", &uart_conf.dynamicenable},
    };
#endif
#ifdef WITH_ZMQ
    std::string zmq_address;
#endif
    // multiplayer
    int iMultiplayer{ 0 }; // blokada działania niektórych eventów na rzecz kominikacji
	bool bIsolatedTrainName{ false }; //wysyłanie zajęcia odcinka izolowanego z nazwą pociągu
    // other
    std::string AppName{ "MaSzyna" };
    std::string asVersion; // z opisem
	motiontelemetry::conf_t motiontelemetry_conf;
	std::string screenshot_dir;
	bool loading_log = false;
	bool dds_upper_origin = false;
    bool captureonstart = true;
	bool render_cab = true;
	bool crash_damage = true;
	// the buckling of welded track in the heat (physics.trackbuckling in eu07.ini, see world/TrackBuckle.h): off, none; scenery, the
	// buckles the scenery enters and its trackbuckle events; automatic, also by the temperature of the rails at the weak places
	enum class track_buckling_mode { off, scenery, automatic };
	track_buckling_mode track_buckling = track_buckling_mode::scenery;
	// the physical model of the couplings (McZapkie/couplerphysics.h): progressive buffers and draw gear with hysteresis,
	// the play of the coupling, its yield and breaking; the legacy model of TMoverParameters::CouplerForce() without it.
	// it covers the player's train and the trains it closes in on (vehicle_table::update_coupler_physics()), or every
	// train with coupler_physics_all
	bool coupler_physics = false;
	bool coupler_physics_all = false;
	// a vehicle too fast in a curve (physics.derailment in eu07.ini): new, it overturns or its flange climbs the rail
	// by the criteria of McZapkie/derailment.h held over 2 m, and slides on with the friction of the ballast; old, the
	// legacy draw of the lateral acceleration against the gauge to the height of the vehicle in every step, which stops
	// it; off, it stays on the rails at any speed; monitor, the legacy draw decides and the log tells where the criteria
	// would derail a vehicle (derailment monitor:, once for a vehicle on a track), for the run of a scenery as it was.
	// monitor is the default until the speed limits of the bundled sceneries are checked against the criteria (some
	// tracks allow more than the reference vehicle survives there): new is opt-in until then
	enum class derailment_model { off, old, physical, monitor };
	derailment_model derailment = derailment_model::monitor;
	// the collisions of rail vehicles past their buffers (physics.collisions in eu07.ini): new, the ends crush, a vehicle
	// may ride up onto the other one or be pushed off the track (McZapkie/collision.h); old, the legacy rule of the safe
	// closing speed weighed by the masses; monitor, the legacy rule decides and the log tells what the crush would be
	// (collision monitor:, once for a hit). the road vehicles of the traffic keep the legacy rule with any
	enum class collision_model { old, physical, monitor };
	collision_model collisions = collision_model::physical;
	// physics.trackcheck: the geometry of the rail tracks the criteria of derailment find suspicious, to the log once the
	// scenery loaded (trackcheck:, see world/TrackCheck.h)
	bool track_check = false;
	bool control_coupling_engine_check = true; // no multiple-unit control between electric and diesel traction vehicles (eu07/maszyna#520)
	bool gui_defaultwindows = true;
	bool gui_showtranscripts = true;
    bool gui_trainingdefault = false;

	std::string extcam_cmd;
	std::string extcam_rec;
	glm::ivec2 extcam_res{800, 600};

	std::chrono::duration<float> minframetime {0.0f};
    std::string fullscreen_monitor;
    bool fullscreen_windowed{ false };

    bool python_enabled = true;
	bool python_mipmaps = true;
	bool python_displaywindows = false;
	bool python_threadedupload = true;
	bool python_vsync = true;
	bool python_sharectx = true;
	bool python_uploadmain = true;
	std::chrono::duration<float> python_minframetime {0.01f};
	bool luascreens_enabled{true}; // screens drawn by Lua scripts (luascreen:, luadestinationsign: in the .mmd)
	bool lua_fulllibraries{false}; // the scenery scripts (lua in the .scn) get all of LuaJIT's libraries (io, os, ffi, require...), not only the sandboxed ones
	bool wasm_enabled{true}; // screens drawn by WebAssembly modules (wasmscreen:, wasmdestinationsign: in the .mmd)
	int screens_cabframes{4}; // the script screens in the cab are refreshed every so many rendered frames
	int screens_externalframes{30}; // the script screens outside the cab (destination signs) are refreshed every so many rendered frames
	bool screens_gpupost{true}; // the post-processing of the script screens (image:led_post, image:resize) on the gpu, when the renderer can

    bool gfx_skiprendering = false;
    // the size of the image the scene of the main window is drawn to: -1 until the renderer works it out from the window and
    // gfx.buffer.percent (the old keys gfx.framebuffer.width and .height aren't taken any more)
    int gfx_framebuffer_width = -1;
    int gfx_framebuffer_height = -1;
    // the size of the image of the main window drawn from the window and gfx.buffer.percent (Scale: the part of the window, 0: that
    // of the settings), at least a pixel
    glm::ivec2 gfx_buffer_size( float Scale = 0.f ) const;
    bool gfx_shadowmap_enabled = true;
    bool gfx_envmap_enabled = true;
    bool gfx_lights_clustered = true; // headlights from the light lists of the clusters (no limit of lit headlights) or the first dynamiclights of them
    bool gfx_lights_clusterheatmap = false; // debug view of the light clusters
    // culling of the model instances of the main view (opengl 4.6): 0 on the cpu, 1 in a compute pass, which the instances are
    // drawn from with indirect draws, 2 on the cpu, which draws, and in a compute pass too, for the results to be compared
    int gfx_gpucull = 0;
    bool gfx_gpucull_occlusion = true; // with gfx_gpucull 1: instances hidden behind the depth drawn before them are left out
    bool gfx_gpucull_occlusion_instances = true; // with gfx_gpucull_occlusion: in two phases, so that instances hide one another too
    bool gfx_depthprepass = true; // with gfx_gpucull 1: the depth of the instances is drawn before their colour
    // with gfx_gpucull 1: the draws of the instances which follow one another with the same material and geometry bank are
    // made with one multi draw, their model blocks read from a storage buffer by the number of the draw
    bool gfx_gpucull_multidraw = true;
    // with gfx_gpucull 1 or 2: the instances of the cascades of the sun shadows and of the faces of the environment map are
    // culled in compute passes as well (drawn from their result, or compared), not only those of the main view
    bool gfx_gpucull_shadows = true;
    bool gfx_gpucull_reflections = true;
    // with gfx_gpucull 1 or 2: the same for the maps of the headlights of vehicles and for the scenery parts of the maps of
    // scenery lamps, each map with a culling of its own
    bool gfx_gpucull_shadows_headlights = true;
    bool gfx_gpucull_shadows_lamps = true;
    // with gfx_gpucull 1 or 2: the sleepers of the tracks (sleepermodel) of the main view are culled with the instances as well,
    // and with 1 drawn from the result
    bool gfx_gpucull_sleepers = true;
    // with gfx_gpucull 1 or 2: the model instances drawn on their own (with lights, animations or skins of their own, which
    // keep them out of the instanced draws) are culled with the instances as well, each one a group of its own, in every pass
    // culled on the gpu, and with 1 drawn from the result. models with billboards stay on the cpu
    bool gfx_gpucull_models = true;
    // the buckets of instances of a scene cell lying wholly beyond their range (the largest range of their instances and of
    // the submodels of their model) are left out of a pass before the culling, the gpu one included, which would leave out
    // each of their instances anyway
    bool gfx_gpucull_bucketrange = true;
    // the walks of the submodel trees which record the draws of the instances culled on the gpu are kept from frame to frame and
    // made again only when the model changes: the trees of animated models, ones lit by the luminance and the models drawn on their
    // own are walked every time
    bool gfx_gpucull_walkcache = true;
    // (opengl 4.6) the geometry of the banks without user data (models, scenery) is kept in a few large buffers shared by them, so
    // the multi draws can take the chunks of many models at once. read when a bank sets its buffers up (load time)
    bool gfx_geometry_shared = true;
    // (opengl 4.6) the depth of the whole opaque scene of the main view is drawn before its colour, the instances included
    bool gfx_depthprepass_scene = false;
    // (opengl 4.6) the depth pass of the scene (gfx_depthprepass_scene, or the one of the contact shadows) walks the scene for
    // the colour pass as well: its opaque draws are recorded once, their depth drawn from the record (with the contact shadows
    // alone only within their reach) and their colour too. false: the scene is walked twice, as before
    bool gfx_depthprepass_replay = true;
    // (opengl 4.6) grass of the heightmap terrain, grown where it's painted: drawn, how far from the camera (metres), and how
    // many blades of the density painted (1: all)
    bool gfx_grass = true;
    float gfx_grass_distance = 60.f;
    // (opengl 4.6) the grass thins out from gfx_grass_distance to here, where it ends
    float gfx_grass_fadedistance = 70.f;
    float gfx_grass_density = 1.f;
    // (opengl 4.6) the smoke is simulated and drawn on the gpu (opengl46_gpu_particles), lit by the sun, the sky and the lamps
    // when gfx_smoke_lighting; false: simulated on the cpu and drawn as before
    bool gfx_smoke_gpu = true;
    bool gfx_smoke_lighting = true;
    // (opengl 4.6) the particles of the gpu fade where they near what is behind them, over this share of their size (smoke) or of
    // a few decimetres (rain, snow), instead of being cut by it; 0: cut, as before
    float gfx_particles_soft = 1.f;
    // (opengl 4.6) rain and snow as particles around the camera, lit like the smoke (opengl46_gpu_particles), at most this many of
    // them in the heaviest rain or snow; false: the textured cylinder, as before
    bool gfx_precipitation_gpu = true;
    int gfx_precipitation_particles = 65536;
    // (opengl 4.6) the model block of every draw and the matrices of instances are written to persistently mapped buffers,
    // one for each of the last frames, instead of with glBufferSubData. read at the start
    bool gfx_bufferstreaming = true;
    bool gfx_postfx_motionblur_enabled = true;
    // the colour pass writes the velocities of the pixels: for the motion blur, the temporal anti-aliasing or dlss
    // the velocity target is always drawn: the windscreens (mat_rain_windscreen, mat_bugs_windscreen) send the post-processing their
    // drops and haze through it, whatever the motion blur and the anti-aliasing (not without the pipeline: no post-processing there)
    bool gfx_motion_vectors() const { return ( false == gfx_skippipeline ) || gfx_postfx_motionblur_enabled; }
    float gfx_postfx_motionblur_shutter = 0.01f;
    GLenum gfx_postfx_motionblur_format = GL_RG16F;
    GLenum gfx_format_color = GL_RGB16F;
    GLenum gfx_format_depth = GL_DEPTH_COMPONENT32F;
    bool gfx_postfx_chromaticaberration_enabled = true;
    bool gfx_skippipeline = false;
    bool gfx_extraeffects = true;
    bool gfx_shadergamma = false;
    bool gfx_gldebug = false;
    // the shaders are loaded from the SPIR-V binaries of shaders/ogl46bin and shaders/vk14bin where there are up to date ones
    bool gfx_shader_binaries = true;
    // (vulkan) the gpu to run on: its index in the list of the log or a part of its name; empty: the best one
    std::string gfx_vulkan_device;
    // (vulkan) 0: no validation layer, 1: validation layer, 2: also gpu-assisted validation
    int gfx_vulkan_validation { 0 };
    // (vulkan) frames recorded while the gpu works on the earlier ones, 2 or 3
    int gfx_vulkan_framesinflight { 2 };
    // (vulkan) the command buffers of the frame recorded, submitted and presented by a thread of its own while the simulation goes on
    // with the next frame
    bool gfx_vulkan_renderthread { true };
    // threads recording the command buffers of the vulkan renderer; 0: the cores less two, 1: one thread records them all
    int gfx_threads { 0 };
    // (vulkan) the pipelines and the textures a draw needs first made by the background threads, the draw left out till then (the
    // cab waits for them); the pipelines of the last session made ahead at the start
    bool gfx_vulkan_asyncresources { true };
    // (vulkan) the cascades of the sun shadows and the shadow maps of the lamps and the headlights walked by threads of their own while
    // the main thread walks the colour pass
    bool gfx_vulkan_parallelpasses { true };
    // (vulkan) the submodel trees of the instances culled on the gpu walked by the threads of gfx.threads, model by model
    bool gfx_vulkan_parallelmodels { true };
    // (vulkan) the passes of the sky and the weather run on the compute queue, at the same time as the sun shadows, where the gpu has a
    // compute family of its own; off until a measurement on such a gpu shows what it gains
    bool gfx_vulkan_asynccompute { false };
    // (vulkan) the passes leave checkpoints where the gpu has them (VK_NV_device_diagnostic_checkpoints): the report of a lost device
    // in the log tells the pass the gpu got to
    bool gfx_vulkan_checkpoints { true };
    bool vr = false;
    std::string vr_backend;

    float gfx_distance_factor_max { 3.f };
    float gfx_shadow_angle_min { -0.2f };
    int gfx_shadow_rank_cutoff { 3 };
    // (opengl 4.6) the cascades of the sun shadows are fitted to the depth of the main view of a frame or two before (SDSM):
    // the distances they cover go from the nearest to the farthest pixel which isn't the sky, and the map of each covers only
    // the area of the pixels it takes. false: the fixed cascades of shadowtune
    bool gfx_shadow_sdsm { false };
    // with gfx_shadow_sdsm: how the distances between the nearest and the farthest pixel are split between the cascades,
    // 0: in equal lengths, 1: in equal ratios
    float gfx_shadow_sdsm_split { 0.9f };

    // sky light and exposure of the opengl 4.6 renderer. the switches can also be changed at run time in the debug window
    // "Sky and exposure"; the scales are multipliers of the calibrated defaults (1 = as calibrated)
    bool gfx_ambient_sh { true }; // the ambient light depends on where a surface faces: the sky above, the ground below
    float gfx_ambient_sh_strength { 1.f }; // 0 = uniform ambient light as before, 1 = full directional difference
    float gfx_ambient_groundalbedo { 0.1f }; // how much light the ground below the horizon gives back
    bool gfx_envmap_ggx { true }; // rough reflections from the environment map prefiltered with the GGX lobe
    bool gfx_atmosphere { true }; // sky, sun, ambient light and fog colour from the physical atmosphere (Hillaire 2020)
    float gfx_atmosphere_sky { 1.f }; // brightness of the drawn sky and of the fog
    float gfx_atmosphere_ambient { 1.f }; // brightness of the ambient light
    float gfx_atmosphere_sun { 1.f }; // brightness of the sunlight
    float gfx_atmosphere_haze { 1.f }; // amount of aerosols (mie scattering) on a clear day; grows with the overcast
    float gfx_atmosphere_altitude { 200.f }; // height of the scenery above the sea level, in metres
    bool gfx_postfx_autoexposure { true }; // the exposure follows the brightness of the image, within limits
    float gfx_postfx_autoexposure_key { 0.3f }; // average luminance of the image the exposure is set for
    float gfx_postfx_autoexposure_min { -1.f }; // lowest exposure correction, EV (darkening of bright images)
    float gfx_postfx_autoexposure_max { 0.5f }; // highest exposure correction, EV (brightening of dark images, tunnels)
    float gfx_postfx_autoexposure_max_cab { 2.f }; // the same seen from the cab by day: the eye takes to the light of the cab
    float gfx_postfx_autoexposure_speed_dark { 1.f }; // rate of adaptation to a darker image, 1/s
    float gfx_postfx_autoexposure_speed_bright { 3.f }; // rate of adaptation to a brighter image, 1/s
    float gfx_postfx_autoexposure_compensation { 0.f }; // fixed exposure correction, EV, applied with and without the adaptation
    bool gfx_postfx_bloom { true }; // glow around bright parts of the image
    float gfx_postfx_bloom_intensity { 0.04f }; // share of the blurred image in the result
    float gfx_postfx_bloom_threshold { 0.f }; // luminance below which nothing glows, 0 = everything contributes
    bool gfx_postfx_whitebalance { true }; // white balance of the camera, applied to the image before the tonemapping
    float gfx_postfx_whitebalance_temperature { 7500.f }; // colour temperature of the light taken as white, K; 6500 = none, more = warmer image
    float gfx_postfx_whitebalance_tint { 0.f }; // -1..1, shift off the daylight locus: + magenta image, - green image
    // screen space ambient occlusion of the opengl 4.6 renderer, applied to the image (debug window "Sky and exposure").
    // 0 off, 1 the older ssao, 2 ground truth ambient occlusion; when 0 at the start the targets are not made at all
    int gfx_postfx_ao { 2 };
    float gfx_postfx_ao_radius { 1.5f }; // reach of the occluders, metres
    float gfx_postfx_ao_strength { 1.f }; // power the occlusion is raised to, 1 = as computed, 0 = none
    float gfx_postfx_ao_bounce { 0.3f }; // albedo of the surfaces assumed for the light coming back after more bounces
    int gfx_postfx_ao_slices { 3 }; // directions searched around each pixel
    int gfx_postfx_ao_steps { 4 }; // samples on each side of a direction
    float gfx_postfx_ao_cab { 0.5f }; // share of the occlusion the cab and the low poly interiors take, 1 = all of it
    // with gfx_postfx_ao 2 (opengl 4.6 and vulkan): the occlusion is worked out from the depth of the scene drawn before its colour
    // (which it turns on, see gfx_depthprepass_scene) and darkens the ambient light and the reflection of the surroundings in the
    // shading, not the light of the sun and the lamps. false: it darkens the finished image, as before
    bool gfx_postfx_ao_lighting { true };
    // local lights of the opengl 4.6 renderer, besides the sun and the vehicle headlights (debug window "Light clusters").
    // they go through the light clusters, so they need gfx_lights_clustered
    bool gfx_lights_scenery { true }; // light points (freespot lights) of scenery models light their surroundings
    bool gfx_lights_vehicles { true }; // light points of vehicles and their lit compartments light their surroundings
    // the old lighting ahead of vehicles: the beam of their headlights (light_array, the projected texture of the beam), with its
    // shadows and its light in the fog. off: the vehicles light what is ahead of them only with the light points of their lamps
    bool gfx_lights_headlights { true };
    bool gfx_lights_cab { true }; // the lamp of the occupied cab is a point light instead of an addition to the ambient light
    float gfx_lights_freespot_intensity { 10.f }; // brightness of a light point of a model
    float gfx_lights_compartment_intensity { 1.f }; // brightness of the light of a lit vehicle compartment
    float gfx_lights_cab_intensity { 1.5f }; // brightness of the cab lamp
    float gfx_lights_cab_bounce { 0.3f }; // share of the former ambient light of the cab lamp kept, for the light which bounces around the cab
    // a light point of a scenery lamp shining downwards lights its surroundings also closer than its glare is seen from (the
    // minimal distance of its submodel, which many lamps have set for the glare seen from afar), so the lamp doesn't go out as the
    // camera comes near it. off: as before
    bool gfx_lights_points_near { false };
    // names of the smt_Stars submodels of scenery models (many street lamps have nothing else) whose glowing points light their
    // surroundings, each one downwards as the lamp of a street light, separated with |. other glowing points (lights of signals,
    // of chimneys, the stars of the sky) only glow. empty (as before): none. the lamps of many models are named "lamp"
    std::string gfx_lights_stars {};
    // the half angle of the cone of the light of a light point of a scenery lamp shining downwards is at least this, degrees.
    // 0: the cone of its glare, which many lamps have narrow. lights shining sideways (signals) keep their cones
    float gfx_lights_freespot_mincone { 0.f };
    // the brightness of the light the lamps of the scenery shining downwards (and their glowing points, gfx_lights_stars) light
    // their surroundings with is multiplied by this. lights shining sideways (signals) keep theirs
    float gfx_lights_lamps_gain { 1.f };
    // the lamps of the scenery are gathered from this far from the camera when it's above gfx_lights_range, metres, so the lit
    // stations are seen from afar; the vehicles keep gfx_lights_range. 0: from gfx_lights_range
    float gfx_lights_lamps_range { 0.f };
    // textures of the planes models fake the light of their lamps with (pools of light on the ground, glows), separated with |:
    // they aren't drawn while the light points of scenery models light their surroundings. empty: they're drawn
    std::string gfx_lights_hideglow;
    // the probe of the occupied cab (opengl 4.6 and vulkan, gfx::cab_probe): cube maps of the cab lit by its lamp (and what glows in
    // it) and of its albedo, drawn every few frames: the light of the lamp bounces around the cab from it (in place of
    // gfx_lights_cab_bounce) and the cab reflects itself in its glasses and glossy parts. 0 off, 1 low, 2 medium, 3 high (faces of
    // 64, 128, 256 pixels); the frames between its updates, the share of the reflected light which lights the cab again, and
    // whether the cab reflects itself (where the screen space reflections don't find it) instead of the sky
    int gfx_cabprobe { 2 };
    int gfx_cabprobe_interval { 4 };
    float gfx_cabprobe_bounce { 1.f };
    bool gfx_cabprobe_reflections { true };
    // the probe of the cab is lit by the sun (the moon) coming in through the windows too, so the light which bounces around the cab
    // is the light of the sun where it falls in it; the scale of that light, 0 for none
    float gfx_cabprobe_sun { 1.f };
    // the light of the sun coming in through the windows of the low poly interiors of the vehicles and of the cab without its probe,
    // bounced around them, as a share of the light of the sun on a surface facing it; 0 for none
    float gfx_interior_bounce { 0.1f };
    // the water of the scenery (the materials of the water shader, opengl 4.6 and vulkan, shaders/*/water_surface.glsl): rippled by
    // layers of its normal map, reflecting the scene and the sky with the fresnel of water, seen through as deep as it is and fading
    // out at the shore. off: the water drawn as before
    bool gfx_water_realistic { false };
    // the occlusion of the sky and of the sun by the scenery around the camera (opengl 4.6 and vulkan, gfx::sky_occlusion, the track T):
    // tunnels, the ground under bridges and roofs, cuttings and the foot of slopes are dark by their geometry, worked out in the
    // background from the scenery seen from above. 0 off (nothing is occluded), 1 low (256 m around the camera, texels of 1 m),
    // 2 medium (512 m, 1 m), 3 high (512 m, 0.5 m, searched farther); whether the sun (the moon) is held back too, and the light of
    // the volumetric fog
    int gfx_skyvisibility { 2 };
    bool gfx_skyvisibility_sun { true };
    bool gfx_skyvisibility_fog { true };
    // the occlusion only around the tracks with the tunnel flag (the geometry there makes the tunnel dark, the portals, the passage at
    // them); elsewhere the scenery is lit as in the open. off: everywhere
    bool gfx_skyvisibility_tunnels { true };
    // the environment flags of the tracks (tunnel, canyon) darken the tracks and the vehicles on them, as before the track T. off: the
    // renderers leave them alone and the light comes from the geometry (gfx_skyvisibility); the flags stay in the scenery files for
    // the sound and the driver
    bool gfx_tunnelflags { false };
    float gfx_lights_range { 300.f }; // distance from the camera the lights of models and vehicles are taken from, metres
    // moths flying about the lit lamps of the scenery (rendering/common/moths.cpp): a swarm (the model animals/cmy, animated by
    // textures) at a part of the lamps shining downwards, picked by their places, in the warm months, within the range of the camera
    bool moths { true };
    float moths_share { 0.4f }; // part of the lamps with a swarm, 0-1
    float moths_range { 80.f }; // distance from the camera the swarms are shown at, metres
    float moths_temperature { 8.f }; // the lowest temperature of the air moths fly at, degrees Celsius
    // deer at the tracks (world/deer.cpp): put on the path ahead of the train of the player, grazing, they run away from the horn
    // or the train coming close. off unless turned on
    bool deer { false };
    float deer_interval { 2000.0f }; // metres the train covers between the deer, 0 none
    float deer_distance { 2000.0f }; // metres ahead of the train a deer is put at
    float deer_chance { 1.0f }; // of a deer at each interval, 0-1
    float deer_hornrange { 600.0f }; // the horn sends away the deer within this many metres
    float deer_reactiontime { 3.5f }; // a deer runs away from the train this many seconds of its speed away, seconds
    float deer_reactiondistance { 20.0f }; // and from a slow or a standing train come this close, metres
    // shadows of the lamps (cube shadow maps), 0 none, 1 low, 2 medium, 3 high; see opengl46_local_shadows. for now the lamp of the
    // occupied cab, its shadows cast by the cab
    int gfx_lights_shadows { 2 };
    // with gfx_lights_shadows above 0: how many lights of the occupied cab (its lamps, the lights of its desk) at most have shadows
    // cast by the cab, the brightest ones (0-16); the others take the shadows of the map of the closest light with one
    int gfx_lights_shadows_cab { 8 };
    // shadows of the lamps of scenery models (cube maps of the scenery around them, kept; vehicles added as they pass), with
    // gfx_lights_shadows above 0: how many lamps at once, the size of a face of their maps, the distance from the camera within
    // which lamps get shadows and within which vehicles (and the cab) cast them
    int gfx_lights_shadows_lamps { 8 };
    int gfx_lights_shadows_lamps_size { 512 };
    float gfx_lights_shadows_lamps_range { 100.f };
    float gfx_lights_shadows_vehicles_range { 50.f };
    // shadows of the headlights of vehicles (a map drawn with the projector of the beam), with gfx_lights_shadows above 0: for how
    // many sets of headlights at once (the most important ones, 0-4), and the distance ahead of the vehicle within which things
    // cast them, metres
    int gfx_lights_shadows_headlights { 2 };
    float gfx_lights_shadows_headlights_range { 250.f };
    // (opengl 4.6) contact shadows: short rays through the depth of the scene drawn before its colour (which they turn on, see
    // gfx_depthprepass_scene) towards the sun and the nearest lamps. 0 none, 1 low (the sun), 2 medium (and 2 lamps), 3 high
    // (and 4 lamps, more steps); the length of the rays and the distance from the camera within which they are traced, metres
    int gfx_contactshadows { 0 };
    float gfx_contactshadows_length { 0.5f };
    float gfx_contactshadows_range { 50.f };
    // screen space reflections of the opengl 4.6 and vulkan renderers, in place of the scenery in the environment map: the
    // reflection of a surface traced through the depth of the scene drawn before its colour (which they turn on, see
    // gfx_depthprepass_scene) and taken from the image of the last frame; where the screen doesn't hold it, the environment map
    // (the sky) or the ray tracing. 0 none, 1 low, 2 medium, 3 high (steps of the rays); the longest ray, metres, and the
    // roughest surface they're traced for (perceptual roughness, rougher ones reflect the environment map)
    int gfx_ssr { 2 };
    float gfx_ssr_range { 1000.f };
    float gfx_ssr_roughness { 0.6f };
    // (vulkan) ray traced reflections where the screen space ones find nothing, on a gpu with ray queries: the scenery within
    // the range, metres, as gfx.reflections.fidelity says. taken at the start
    bool gfx_raytracing { false };
    float gfx_raytracing_range { 500.f };
    // weather of the opengl 4.6 renderer (debug window "Weather"); the weather itself comes from the scenery, see
    // environment/weather.h. fog: 1 volumetric (a volume over the view with the shafts of the sun, the shadows of the clouds and
    // the light of lamps and headlights, worked out along the ray past it), 0 analytic (the same fog along the ray only, the way
    // the old one was drawn); the quality of the volume 1-4 (low, medium, high, ultra), its depth (metres), the brightness of the
    // lamps in it (0: none), whether the sun and the headlights cast their shadows into it, blending over frames and the share of
    // a new frame in the blend (lower: smoother, but the light of moving lamps lags more behind them)
    int gfx_fog { 1 };
    int gfx_fog_quality { 2 };
    float gfx_fog_range { 1500.f };
    float gfx_fog_lights { 1.f };
    bool gfx_fog_shadows { true };
    bool gfx_fog_temporal { true };
    float gfx_fog_temporal_share { 0.2f };
    // volumetric clouds, 0 none, 1-4 the quality (low, medium, high, ultra); the farthest they are drawn (metres); the shadows
    // they cast (a map around the camera: its size in texels and in metres)
    int gfx_clouds { 2 };
    float gfx_clouds_distance { 40000.f };
    bool gfx_clouds_shadows { true };
    int gfx_clouds_shadows_size { 512 };
    float gfx_clouds_shadows_range { 16000.f };
    // the clouds and the cells of the fog volume the scenery hid in the last frame aren't worked out anew (the vulkan renderer)
    bool gfx_weather_occlusion { true };
    // a quarter of the pixels of the clouds of the main view is marched a frame, the image made of the last ones (the vulkan renderer)
    bool gfx_clouds_interleave { true };
    // stars from the catalogue, and their brightness
    bool gfx_stars { true };
    float gfx_stars_brightness { 1.f };
    // brightness of the sun disc against the radiance its sunlight gives it (1: physical, held within the range of the image),
    // and of the moon disc
    float gfx_sun_disc { 1.f };
    float gfx_moon_disc { 1.f };
    // how much the moon lights the scenery at night (its direct light, the night light that comes with it and the moonlit sky)
    float gfx_moonlight { 1.f };

    float ui_fontsize = 13.0f;
    float ui_scale = 1.0f;
    bool ui_viewports = true; // panels of the ui can be dragged out of the simulator window, into windows of their own

	float map_highlight_distance = 3000.0f;

	std::string exec_on_exit;
    std::string prepend_scn;

	struct extraviewport_config {
		std::string monitor;

		int width, height;
		float draw_range;

        viewport_proj_config projection;
	};
	std::vector<extraviewport_config> extra_viewports;

	struct pythonviewport_config {
		std::string surface;

		std::string monitor;
		glm::ivec2 size;

		glm::vec2 offset;
		glm::vec2 scale;
	};
	std::vector<pythonviewport_config> python_viewports;

    struct headtrack_config {
        std::string joy;

        bool magic_window = false;

        glm::ivec3 move_axes;
        glm::vec3 move_mul;

        glm::ivec3 rot_axes;
        glm::vec3 rot_mul;
    };
    headtrack_config headtrack_conf;

    glm::vec3 viewport_move;
    glm::mat3 viewport_rotate;

    bool map_manualswitchcontrol = false;;

	std::vector<std::pair<std::string, std::string>> network_servers;
	std::optional<std::pair<std::string, std::string>> network_client;
	float desync = 0.0f;

	std::unordered_map<int, std::string> trainset_overrides;

    float m_skysaturationcorrection{ 1.65f };
    float m_skyhuecorrection{ 0.5f };

	// methods
	void LoadIniFile( std::string asFileName );
	void FinalizeConfig();
	// the keys of eu07.ini. Source: where they come from, eu07.ini or a config block of a scenery, which doesn't change
	// the settings only eu07.ini sets (utilities/inionlysettings.h)
	void ConfigParse(cParser &parser, config_source Source = config_source::ini);
	bool ConfigParseGeneral(cParser& Parser, const std::string& token);
	bool ConfigParseAudio(cParser& Parser, const std::string& token);
	bool ConfigParseAudioDevice(cParser& Parser, const std::string& token);
	bool ConfigParseGraphics(cParser& Parser, const std::string& token);
	bool ConfigParseInput(cParser& Parser, const std::string& token);
	bool ConfigParseSimulation(cParser& Parser, const std::string& token);
	bool ConfigParseUI(cParser& Parser, const std::string& token);
	bool ConfigParsePython(cParser& Parser, const std::string& token);
	bool ConfigParseScriptScreens(cParser& Parser, std::string_view token);
	bool ConfigParseNetwork(cParser& Parser, const std::string& token);
	bool ConfigParseHardware(cParser& Parser, const std::string& token);
	bool ConfigParseDebug(cParser& Parser, const std::string& token);
	bool ConfigParseTest(cParser& Parser, std::string_view Token);
	bool ConfigParse_gfx( cParser &parser, std::string_view const Token );
	// sky light and exposure switches of the opengl 4.6 renderer (gfx.ambient.*, gfx.envmap.ggx, gfx.atmosphere*, gfx.postfx.autoexposure*, gfx.postfx.bloom*, gfx.postfx.whitebalance*, gfx.postfx.ao*)
	bool ConfigParse_gfx_sky( cParser &parser, std::string_view const Token );
	bool ConfigParse_gfx_lamps( cParser &parser, std::string_view const Token );
    // sends basic content of the class in legacy (text) format to provided stream
    void
        export_as_text( std::ostream &Output ) const;
    template <typename Type_>
    void
        export_as_text( std::ostream &Output, std::string const Key, Type_ const &Value ) const;
};

template <typename Type_>
void
global_settings::export_as_text( std::ostream &Output, std::string const Key, Type_ const &Value ) const {

    Output << Key << " " << Value << "\n";
}

template <>
void
global_settings::export_as_text( std::ostream &Output, std::string const Key, std::string const &Value ) const;

template <>
void
global_settings::export_as_text( std::ostream &Output, std::string const Key, bool const &Value ) const;

// the models of physics.collisions by their names in eu07.ini: new (the default, also for a name it doesn't know), old,
// monitor
inline global_settings::collision_model collision_model_of( std::string_view const Name ) {
    if( Name == "old" ) { return global_settings::collision_model::old; }
    if( Name == "monitor" ) { return global_settings::collision_model::monitor; }
    return global_settings::collision_model::physical;
}
inline std::string_view name_of( global_settings::collision_model const Model ) {
    switch( Model ) {
        case global_settings::collision_model::old: { return "old"; }
        case global_settings::collision_model::monitor: { return "monitor"; }
        default: { return "new"; }
    }
}
// the models of physics.derailment by their names in eu07.ini: new, old, off or no, monitor (the default, also for a
// name it doesn't know)
inline global_settings::derailment_model derailment_model_of(std::string_view const Name) {
    if( Name == "new" ) { return global_settings::derailment_model::physical; }
    if( Name == "old" ) { return global_settings::derailment_model::old; }
    if( Name == "off" || Name == "no" ) { return global_settings::derailment_model::off; }
    return global_settings::derailment_model::monitor;
}
inline std::string_view name_of( global_settings::derailment_model const Model ) {
    switch( Model ) {
        case global_settings::derailment_model::old: { return "old"; }
        case global_settings::derailment_model::off: { return "off"; }
        case global_settings::derailment_model::monitor: { return "monitor"; }
        default: { return "new"; }
    }
}

extern global_settings& GetGlobalSettings();

#define Global (GetGlobalSettings())
