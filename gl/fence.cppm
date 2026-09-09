module;
#include "glad/glad.h"

export module eu07.gl.fence;
import eu07.gl.object;

export {


namespace gl
{
    class fence
    {
        GLsync sync;

    public:
        fence();
        ~fence();

        bool is_signalled();

        fence(const fence&) = delete;
        fence& operator=(const fence&) = delete;
    };
}

}  // export
