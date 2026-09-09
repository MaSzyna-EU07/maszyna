#pragma once

// Macros cannot cross a module boundary, so they live in a plain header
// that consumers pull into their global module fragment. The bodies are
// expanded at the use site, where the imported names are in scope, so this
// header deliberately includes nothing.

#define glColor3f OpenGLColor.color3
#define glColor3fv OpenGLColor.color3
#define glColor4f OpenGLColor.color4
#define glColor4fv OpenGLColor.color4
