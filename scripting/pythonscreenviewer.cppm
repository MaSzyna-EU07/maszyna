module;
#include <array>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <vector>
#include "scripting/PyInt_macros.h"
#include <GLFW/glfw3.h>

export module eu07.scripting.pythonscreenviewer;
import eu07.glm;
import eu07.gl.shader;
import eu07.gl.ubo;
import eu07.gl.bindable;
import eu07.rendering.renderer;
import eu07.scripting.pyint;

export {

class python_screen_viewer
{
	struct window_state {
		GLFWwindow *window = nullptr;

        glm::ivec2 window_size;
        glm::ivec2 fb_size;
        glm::ivec2 cursor_pos;

		glm::vec2 offset;
		glm::vec2 scale;

		std::unique_ptr<gl::ubo> ubo;
		std::unique_ptr<gl::program> shader;

		std::chrono::high_resolution_clock::time_point timestamp;

		window_state() = default;
		~window_state();
	};

	std::vector<std::unique_ptr<window_state>> m_windows;

	std::shared_ptr<python_rt> m_rt;
    std::shared_ptr<std::vector<glm::vec2>> m_touchlist;
	std::shared_ptr<std::thread> m_renderthread;
	gl::scene_ubs m_ubs;

	std::atomic_bool m_exit = false;

	void threadfunc();

public:
    python_screen_viewer(std::shared_ptr<python_rt> rt, std::shared_ptr<std::vector<glm::vec2>> touchlist, std::string name);
	~python_screen_viewer();

	void notify_window_size(GLFWwindow *window, int w, int h);
    void notify_window_fb_size(GLFWwindow *window, int w, int h);
    void notify_cursor_pos(GLFWwindow *window, double x, double y);
    void notify_click(GLFWwindow *window, int button, int action);
};

}  // export
