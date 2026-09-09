#pragma once

// Macros cannot cross a module boundary, so they live in a plain header
// that consumers pull into their global module fragment. The bodies are
// expanded at the use site, where the imported names are in scope, so this
// header deliberately includes nothing.

#define glMatrixMode OpenGLMatrices.mode
#define glPushMatrix OpenGLMatrices.push_matrix
#define glPopMatrix OpenGLMatrices.pop_matrix
#define glLoadIdentity OpenGLMatrices.load_identity
#define glLoadMatrixf OpenGLMatrices.load_matrix
#define glRotated OpenGLMatrices.rotate
#define glRotatef OpenGLMatrices.rotate
#define glTranslated OpenGLMatrices.translate
#define glTranslatef OpenGLMatrices.translate
#define glScalef OpenGLMatrices.scale
#define glMultMatrixd OpenGLMatrices.multiply
#define glMultMatrixf OpenGLMatrices.multiply
