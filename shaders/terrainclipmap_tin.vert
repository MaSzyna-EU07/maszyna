// TIN Terrain Vertex Shader
// Replaces texture-based heightfield with vertex attributes

#version 330 core

// Vertex attributes from VBO
layout(location = 0) in vec3 aPosition;    // Relative to tile origin
layout(location = 1) in vec2 aUV;
layout(location = 2) in uint aMaterial;

// Uniforms
uniform vec3 tile_offset;     // Tile origin relative to camera
uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;

// Output to fragment shader
out vec2 vUV;
flat out uint vMaterial;

void main() {
    // Position is tile-relative, add offset to get camera-relative
    vec3 worldPos = aPosition + tile_offset;
    
    // Transform to clip space
    gl_Position = projectionMatrix * modelViewMatrix * vec4(worldPos, 1.0);
    
    // Pass through to fragment shader
    vUV = aUV;
    vMaterial = aMaterial;
}
