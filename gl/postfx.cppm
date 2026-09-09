module;
#include <array>
#include <chrono>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <cstddef>
#include <string>
#include <vector>
#include <memory>

export module eu07.gl.postfx;
import eu07.gl.shader;
import eu07.gl.vao;
import eu07.gl.framebuffer;
import eu07.model.texture;

export {


namespace gl
{
    class postfx
    {
    private:
        gl::program program;
        static std::shared_ptr<gl::shader> vertex;
        static std::shared_ptr<gl::vao> vao;

    public:
        postfx(const std::string &s);
        postfx(const shader &s);

        void attach();
        void apply(opengl_texture &src, framebuffer *dst);
        void apply(std::vector<opengl_texture*> src, framebuffer *dst);
    };
}

}  // export
