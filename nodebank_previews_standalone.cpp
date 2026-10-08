#include "stdafx.h"

#include "model/AnimModel.h"
#include "rendering/renderer.h"
#include "scene/scenenode.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"
#include "editor/editorPreviews.hpp"
#include <png.h>
#include <cstring>

// standalone generation of preview images of the models of the node bank of the scenery editor:
// -generate-nodebank-previews [-entries "node ... endmodel" ...] [-resolution <pixels>] [-margin <pixels>] [-no-shadows] [-software]
// each image is a square png on white background, saved in textures/previews/nodebank under the path of the model

namespace
{

struct preview_settings
{
	std::vector<std::string> entries; // node definitions to draw. the whole node bank if empty
	int resolution{1024};
	int margin{64};
	bool shadows{true};
	bool software{false};
};

// prints a line on the console the generator was started from, and puts it in the log
void report(std::string const &Text, bool const Error = false)
{
	std::fprintf((Error ? stderr : stdout), "%s\n", Text.c_str());
	std::fflush(Error ? stderr : stdout);
	if (Error)
	{
		// goes to the log as well
		ErrorLog(Text);
	}
	else
	{
		WriteLog(Text);
	}
}

bool parse_number(char const *Text, int &Value)
{
	try
	{
		std::size_t used{0};
		auto const value{std::stoi(Text, &used)};
		if (Text[used] != '\0')
		{
			return false;
		}
		Value = value;
		return true;
	}
	catch (std::exception const &)
	{
		return false;
	}
}

bool parse_arguments(int const Argc, char *Argv[], preview_settings &Settings)
{
	for (int i = 2; i < Argc; ++i)
	{
		std::string const token{Argv[i]};
		if (token == "-entries")
		{
			while ((i + 1 < Argc) && (Argv[i + 1][0] != '-'))
			{
				Settings.entries.emplace_back(Argv[++i]);
			}
		}
		else if (token == "-resolution")
		{
			if ((i + 1 >= Argc) || (false == parse_number(Argv[++i], Settings.resolution)) || (Settings.resolution < 16))
			{
				report("-resolution needs a number of pixels, 16 or more", true);
				return false;
			}
		}
		else if (token == "-margin")
		{
			if ((i + 1 >= Argc) || (false == parse_number(Argv[++i], Settings.margin)) || (Settings.margin < 0))
			{
				report("-margin needs a number of pixels, 0 or more", true);
				return false;
			}
		}
		else if (token == "-no-shadows")
		{
			Settings.shadows = false;
		}
		else if (token == "-software")
		{
			Settings.software = true;
		}
		else
		{
			report("unknown option " + token, true);
			return false;
		}
	}
	if (Settings.margin * 2 >= Settings.resolution)
	{
		report("the margin leaves no room for the model at this resolution", true);
		return false;
	}
	return true;
}

// node definitions of the node bank, read the way the node bank panel of the editor reads them
std::vector<std::string> nodebank_entries()
{
	std::vector<std::string> entries;
	std::ifstream file("nodebank.txt", std::ios_base::in | std::ios_base::binary);
	std::string line;
	while (std::getline(file, line))
	{
		if (line.size() < 4)
		{
			continue;
		}
		auto const nodestart{line.find("node")};
		if (nodestart == std::string::npos)
		{
			// group header
			continue;
		}
		auto entry{line.substr(nodestart)};
		while ((false == entry.empty()) && ((entry.back() == '\r') || (entry.back() == '\n')))
		{
			entry.pop_back();
		}
		entries.emplace_back(entry);
	}
	return entries;
}

// creates the hidden window with the gl context for the renderer. on a computer without a graphics driver able to provide
// gl 3.3 the software rasterizer of mesa can take over: on linux mesa picks it by itself, on windows its opengl32.dll
// (with the libraries it needs) is looked for in the mesa folder
GLFWwindow *create_context(bool const Software)
{
	if (Software)
	{
#ifdef _WIN32
		auto const folder{std::filesystem::absolute("mesa")};
		::SetDllDirectoryW(folder.wstring().c_str());
#else
		::setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
#endif
	}
	if (glfwInit() == GLFW_FALSE)
	{
		return nullptr;
	}
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	if (Global.gfx_gldebug)
		glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
	auto *window{glfwCreateWindow(64, 64, "eu07 previews", nullptr, nullptr)};
	if (window == nullptr)
	{
		glfwTerminate();
		return nullptr;
	}
	glfwMakeContextCurrent(window);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		glfwDestroyWindow(window);
		glfwTerminate();
		return nullptr;
	}
	return window;
}

bool save_png(std::string const &Path, std::vector<std::uint8_t> const &Image, int const Size)
{
	std::error_code error;
	std::filesystem::create_directories(std::filesystem::path(Path).parent_path(), error);
	png_image png;
	std::memset(&png, 0, sizeof(png_image));
	png.version = PNG_IMAGE_VERSION;
	png.width = Size;
	png.height = Size;
	png.format = PNG_FORMAT_RGB;
	return (png_image_write_to_file(&png, Path.c_str(), 0, Image.data(), Size * 3, nullptr) != 0);
}

// draws the preview of a single node bank entry. returns: true on success
bool generate_preview(std::string const &Definition, preview_settings const &Settings, std::set<std::string> &Saved, std::string &Outcome)
{
	// a line of the node bank can be given as it is, with the label in front of the definition
	auto const nodestart{Definition.find("node")};
	if (nodestart == std::string::npos)
	{
		Outcome = "not a node definition";
		return false;
	}
	auto const entry{Definition.substr(nodestart)};
	cParser parser(entry, cParser::buffer_TEXT);
	parser.getTokens(); // node
	scene::node_data nodedata;
	parser.getTokens(4);
	parser >> nodedata.range_max >> nodedata.range_min >> nodedata.name >> nodedata.type;
	if (nodedata.name == "none")
	{
		nodedata.name.clear();
	}
	if (nodedata.type != "model")
	{
		Outcome = "skipped, node of type " + nodedata.type + " isn't a model";
		return false;
	}
	// placement in the scenery is of no use here, the rotation however is a part of the look of the node
	glm::dvec3 location;
	glm::vec3 rotation{0.f};
	parser.getTokens(4);
	parser >> location.x >> location.y >> location.z >> rotation.y;
	// NOTE: instances are kept, as the rest of the simulator assumes scenery nodes live until the exit
	auto *instance = new TAnimModel(nodedata);
	instance->Angles(rotation);
	instance->Load(&parser, false);
	if (instance->pModel == nullptr)
	{
		Outcome = "model not found";
		return false;
	}
	instance->location(glm::dvec3{0.0});

	// the model and skin, which give the name of the image, are the first two tokens after the rotation
	cParser names(entry, cParser::buffer_TEXT);
	names.getTokens(9, false);
	auto const model{names.getToken<std::string>()};
	auto const skin{names.getToken<std::string>(false)};
	// another entry with the same model and skin, but other lights or angles, gets a number; the node bank of the editor
	// counts the entries the same way to find the image
	auto path{editor_previews::variant(editor_previews::preview_path(model, skin), Saved)};
	path += ".png";

	std::vector<std::uint8_t> image;
	if (false == GfxRenderer->Render_Preview(instance, Settings.resolution, Settings.margin, Settings.shadows, image))
	{
		Outcome = "nothing to draw";
		return false;
	}
	if (false == save_png(path, image, Settings.resolution))
	{
		Outcome = "failed to save " + path;
		return false;
	}
	Saved.emplace(path.substr(0, path.size() - 4));
	Outcome = path;
	return true;
}

} // namespace

int generate_nodebank_previews(int Argc, char *Argv[])
{
#ifdef _WIN32
	// the simulator is a windows application, so the console it was started from has to be joined to report progress
	if (::AttachConsole(ATTACH_PARENT_PROCESS))
	{
		FILE *stream{nullptr};
		freopen_s(&stream, "CONOUT$", "w", stdout);
		freopen_s(&stream, "CONOUT$", "w", stderr);
	}
#endif
	// settings of the simulator, for the paths and the graphics options which aren't set for the previews below
	auto const inipath{user_config_path("eu07.ini")};
	if (false == inipath.empty() && std::filesystem::exists(inipath))
	{
		Global.LoadIniFile(inipath.string());
	}
	else
	{
		Global.LoadIniFile("eu07.ini");
	}
	// the progress goes to the console, the log keeps the details
	Global.iWriteLogEnabled &= ~2;
	std::thread logservice(LogService);

	auto const finish = [&](int const Result) {
		// give the log service time to write out what's left
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		Global.applicationQuitOrder = true;
		logservice.join();
		return Result;
	};

	preview_settings settings;
	if (false == parse_arguments(Argc, Argv, settings))
	{
		report("usage: " + std::string(Argv[0]) + " -generate-nodebank-previews [-entries \"node ... endmodel\" ...] [-resolution <pixels>] [-margin <pixels>] [-no-shadows] [-software]", true);
		return finish(1);
	}

	// the full renderer, drawing offscreen
	Global.GfxRenderer = "default";
	Global.LegacyRenderer = false;
	Global.NvRenderer = false;
	Global.BasicRenderer = false;
	Global.gfx_usegles = false;
	Global.gfx_skippipeline = false;
	Global.gfx_postfx_motionblur_enabled = false;
	Global.gfx_postfx_chromaticaberration_enabled = false;
	Global.gfx_postfx_ssao_enabled = false;
	Global.gfx_envmap_enabled = false;
	Global.gfx_shadowmap_enabled = settings.shadows;
	Global.shadowtune.map_size = std::max(Global.shadowtune.map_size, 2048u);
	Global.iMultisampling = 0;
	Global.vr = false;
	Global.extra_viewports.clear();
	Global.VSync = false;
	Global.fb_size = {64, 64};
	Global.window_size = {64, 64};
	Global.gfx_framebuffer_width = 64;
	Global.gfx_framebuffer_height = 64;
	Global.gfx_texture_streaming = false;
	Global.gfx_texture_releasedistance = 0.f;
	// summer daylight, which picks the seasonal variants of models and materials and keeps the lamps off
	Global.Season = "summer:";
	Global.Overcast = 0.f;
	Global.fLuminance = 1.0;

	auto *window{create_context(settings.software)};
	if ((window == nullptr) && (false == settings.software))
	{
		report("no gl 3.3 context from the graphics driver, trying the software renderer");
		settings.software = true;
		window = create_context(true);
	}
	if (window == nullptr)
	{
#ifdef _WIN32
		report("failed to create gl 3.3 context. for computers without a capable graphics driver put opengl32.dll of mesa (with the libraries it comes with) in the mesa folder", true);
#else
		report("failed to create gl 3.3 context. without a display run it through xvfb-run", true);
#endif
		return finish(1);
	}
	GfxRenderer = gfx_renderer_factory::get_instance()->create("modern");
	if ((!GfxRenderer) || (false == GfxRenderer->Init(window)))
	{
		report("failed to initialize the renderer", true);
		return finish(1);
	}
	report(std::string("renderer: ") + reinterpret_cast<char const *>(glGetString(GL_RENDERER)));

	auto const entries{settings.entries.empty() ? nodebank_entries() : settings.entries};
	if (entries.empty())
	{
		report("no entries to draw, nodebank.txt is missing or empty", true);
		return finish(1);
	}

	std::set<std::string> saved;
	auto failures{0};
	for (std::size_t idx = 0; idx < entries.size(); ++idx)
	{
		std::string outcome;
		auto success{false};
		try
		{
			success = generate_preview(entries[idx], settings, saved, outcome);
		}
		catch (std::exception const &Error)
		{
			// a broken entry shouldn't stop the others
			outcome = Error.what();
		}
		if (false == success)
		{
			++failures;
		}
		report("[" + std::to_string(idx + 1) + "/" + std::to_string(entries.size()) + "] " + entries[idx] + (success ? " -> " : ": ") + outcome, (false == success));
	}
	report(std::to_string(saved.size()) + " previews saved in " + editor_previews::folder + (failures > 0 ? ", " + std::to_string(failures) + " entries failed" : ""));

	return finish(failures > 0 ? 1 : 0);
}
