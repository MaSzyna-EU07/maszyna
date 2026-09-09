module;
#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

export module eu07.rendering.screenshot;

export {
class screenshot_manager
{
    static void screenshot_save_thread(char *img , int w, int h);

public:
	static void make_screenshot();
};

}  // export
