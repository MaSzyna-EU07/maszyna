module;
#include "glad/glad.h"

export module eu07.gl.renderbuffer;
import eu07.gl.object;
import eu07.gl.bindable;

export {


namespace gl
{
    class renderbuffer : public object, public bindable<renderbuffer>
    {
    public:
        renderbuffer();
        ~renderbuffer();

        void alloc(GLuint format, int width, int height, int samples = 1);

        static void bind(GLuint id);
        using bindable::bind;
    };
}

}  // export
