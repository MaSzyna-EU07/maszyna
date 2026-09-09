module;
#include "glad/glad.h"

export module eu07.gl.cubemap;
import eu07.gl.object;

export {


namespace gl
{
    // cubemap texture rendertarget
    // todo: integrate with texture system
    class cubemap : public object
    {
    public:
        cubemap();
        ~cubemap();

        void alloc(GLint format, int width, int height, GLenum components, GLenum type);
        void bind(int unit);
        void generate_mipmaps();
    };
}

}  // export
