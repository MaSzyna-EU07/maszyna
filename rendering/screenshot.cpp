#include "stdafx.h"
#include "rendering/screenshot.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include <png.h>

void screenshot_manager::screenshot_save_thread( char *img, int w, int h )
{
	png_image png;
	memset(&png, 0, sizeof(png_image));
	png.version = PNG_IMAGE_VERSION;
    png.width = w;
    png.height = h;

    png.format = PNG_FORMAT_RGB;
    int stride = -w * 3;

	std::array<char, 64> datetime{};
	std::tm const tm_info = local_tm(std::time(nullptr));
	strftime(datetime.data(), datetime.size(), "%Y-%m-%d_%H-%M-%S", &tm_info);

	uint64_t perf;
#ifdef _WIN32
	QueryPerformanceCounter((LARGE_INTEGER*)&perf);
#elif __unix__
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	perf = ts.tv_nsec;
#endif

	std::string filename = Global.screenshot_dir + "/" + std::string(datetime.data()) +
	                       "_" + std::to_string(perf) + ".png";

    if (png_image_write_to_file(&png, filename.c_str(), 0, img, stride, nullptr) == 1)
		WriteLog("saved " + filename);
	else
		WriteLog("failed to save " + filename);

	delete[] img;
}

void screenshot_manager::save(char *Image, int Width, int Height)
{
    std::thread t(screenshot_save_thread, Image, Width, Height);
	t.detach();
}

