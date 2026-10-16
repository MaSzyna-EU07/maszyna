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

#ifdef WITH_UART
#include "utilities/uart.h"
#endif
#ifdef WITH_ZMQ
#include "input/zmq_input.h"
#endif

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
    bool CreateSwitchTrackbeds{ true };
    std::string szTexturesTGA{ ".tga" }; // lista tekstur od TGA
    std::string szTexturesDDS{ ".dds" }; // lista tekstur od DDS
    std::string szDefaultExt{ szTexturesDDS };
	std::string SceneryFile;
    std::string local_start_vehicle{ "EU07-424" };
    std::string editor_enter_vehicle; // vehicle placed in the editor, to be entered when the driving goes on
    bool editor_session{ false }; // scenery was opened for editing (-edit): loader keeps track of scenery layers and hands over to the editor
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
    bool ShowSystemConsole{ false }; // toggle visibility of the OS console window with log output
    unsigned int DisabledLogTypes{ 0 };
    bool ParserLogIncludes{ true };
    // simulation
    bool RealisticControlMode{ false }; // controls ability to steer the vehicle from outside views
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
    bool NvRenderer{ false };
    bool RenderShadows{ true };
	bool applicationQuitOrder{false};
    int RenderCabShadowsRange{ 0 };
    struct shadowtune_t {
        unsigned int map_size{ 2048 };
        float range{ 250.f };
    } shadowtune;
    struct reflectiontune_t {
        double update_interval{ 300.0 };
        int fidelity{ 0 }; // 0: sections, 1: +static models, 2: +vehicles
        float range_instances{ 750.f };
        float range_vehicles{ 250.f };
    } reflectiontune;
    float AnisotropicFiltering{ 8.f }; // requested level of anisotropic filtering. TODO: move it to renderer object
    float FieldOfView{ 45.f }; // vertical field of view for the camera. TODO: move it to the renderer
    bool EditorOrtho{ false };
    float EditorOrthoExtent{ 150.f };
    GLint iMaxTextureSize{ 4096 }; // maksymalny rozmiar tekstury
    GLint iMaxCabTextureSize{ 4096 }; // largest allowed texture in vehicle cab
    int iMultisampling{ 2 }; // tryb antyaliasingu: 0=brak,1=2px,2=4px,3=8px,4=16px
    float SplineFidelity{ 1.f }; // determines segment size during conversion of splines to geometry
    float SleeperDistance{ 250.f }; // max distance (in meters) at which per-track sleeper models are still drawn; 0 disables sleeper rendering entirely
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
    // input
    float fMouseXScale{ 1.5f };
    float fMouseYScale{ 0.2f };
    int iFeedbackMode{ 1 }; // tryb pracy informacji zwrotnej
    int iFeedbackPort{ 0 }; // dodatkowy adres dla informacji zwrotnych
    bool InputGamepad{ true }; // whether gamepad support is enabled
    bool InputMouse{ true }; // whether control pick mode can be activated
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

    bool gfx_skiprendering = false;
    int gfx_framebuffer_width = -1;
    int gfx_framebuffer_height = -1;
    bool gfx_shadowmap_enabled = true;
    bool gfx_envmap_enabled = true;
    bool gfx_lights_clustered = true; // headlights from the light lists of the clusters (no limit of lit headlights) or the first dynamiclights of them
    bool gfx_lights_clusterheatmap = false; // debug view of the light clusters
    // culling of the model instances of the main view (opengl 4.6): 0 on the cpu, 1 in a compute pass, which the instances are
    // drawn from with indirect draws, 2 on the cpu, which draws, and in a compute pass too, for the results to be compared
    int gfx_gpucull = 0;
    bool gfx_gpucull_occlusion = true; // with gfx_gpucull 1: instances hidden behind the depth drawn before them are left out
    bool gfx_depthprepass = true; // with gfx_gpucull 1: the depth of the instances is drawn before their colour
    // (opengl 4.6) the model block of every draw and the matrices of instances are written to persistently mapped buffers,
    // one for each of the last frames, instead of with glBufferSubData. read at the start
    bool gfx_bufferstreaming = true;
    bool gfx_postfx_motionblur_enabled = true;
    float gfx_postfx_motionblur_shutter = 0.01f;
    GLenum gfx_postfx_motionblur_format = GL_RG16F;
    GLenum gfx_format_color = GL_RGB16F;
    GLenum gfx_format_depth = GL_DEPTH_COMPONENT32F;
    bool gfx_postfx_chromaticaberration_enabled = true;
    bool gfx_skippipeline = false;
    bool gfx_extraeffects = true;
    bool gfx_shadergamma = false;
    bool gfx_gldebug = false;
    bool vr = false;
    std::string vr_backend;

    float gfx_distance_factor_max { 3.f };
    float gfx_shadow_angle_min { -0.2f };
    int gfx_shadow_rank_cutoff { 3 };

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
    float gfx_postfx_autoexposure_speed_dark { 1.f }; // rate of adaptation to a darker image, 1/s
    float gfx_postfx_autoexposure_speed_bright { 3.f }; // rate of adaptation to a brighter image, 1/s
    float gfx_postfx_autoexposure_compensation { 0.f }; // fixed exposure correction, EV, applied with and without the adaptation
    bool gfx_postfx_bloom { true }; // glow around bright parts of the image
    float gfx_postfx_bloom_intensity { 0.04f }; // share of the blurred image in the result
    float gfx_postfx_bloom_threshold { 0.f }; // luminance below which nothing glows, 0 = everything contributes
    // screen space ambient occlusion of the opengl 4.6 renderer, applied to the image (debug window "Sky and exposure").
    // 0 off, 1 the older ssao, 2 ground truth ambient occlusion; when 0 at the start the targets are not made at all
    int gfx_postfx_ao { 2 };
    float gfx_postfx_ao_radius { 1.5f }; // reach of the occluders, metres
    float gfx_postfx_ao_strength { 1.f }; // power the occlusion is raised to, 1 = as computed, 0 = none
    float gfx_postfx_ao_bounce { 0.3f }; // albedo of the surfaces assumed for the light coming back after more bounces
    int gfx_postfx_ao_slices { 3 }; // directions searched around each pixel
    int gfx_postfx_ao_steps { 4 }; // samples on each side of a direction
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
    float gfx_lights_range { 300.f }; // distance from the camera the lights of models and vehicles are taken from, metres
    // shadows of the lamps (cube shadow maps), 0 none, 1 low, 2 medium, 3 high; see opengl46_local_shadows. for now the lamp of the
    // occupied cab, its shadows cast by the cab
    int gfx_lights_shadows { 2 };
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
    // stars from the catalogue, and their brightness
    bool gfx_stars { true };
    float gfx_stars_brightness { 1.f };

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
	void ConfigParse(cParser &parser);
	bool ConfigParseGeneral(cParser& Parser, const std::string& token);
	bool ConfigParseAudio(cParser& Parser, const std::string& token);
	bool ConfigParseGraphics(cParser& Parser, const std::string& token);
	bool ConfigParseInput(cParser& Parser, const std::string& token);
	bool ConfigParseSimulation(cParser& Parser, const std::string& token);
	bool ConfigParseUI(cParser& Parser, const std::string& token);
	bool ConfigParsePython(cParser& Parser, const std::string& token);
	bool ConfigParseNetwork(cParser& Parser, const std::string& token);
	bool ConfigParseHardware(cParser& Parser, const std::string& token);
	bool ConfigParseDebug(cParser& Parser, const std::string& token);
	bool ConfigParse_gfx( cParser &parser, std::string_view const Token );
	// sky light and exposure switches of the opengl 4.6 renderer (gfx.ambient.*, gfx.envmap.ggx, gfx.atmosphere*, gfx.postfx.autoexposure*, gfx.postfx.bloom*, gfx.postfx.ao*)
	bool ConfigParse_gfx_sky( cParser &parser, std::string_view const Token );
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

extern global_settings& GetGlobalSettings();

#define Global (GetGlobalSettings())
