// TIN Terrain Fragment Shader
// Minimal material lookup, no blending for now

#version 330 core

// Input from vertex shader
in vec2 vUV;
flat in uint vMaterial;

// Uniforms
uniform sampler2DArray ground;   // Material texture array

// Output
out vec4 fragColor;

void main() {
    // Sample material texture
    // TODO: Add blending between materials for smooth transitions
    vec3 texCoord = vec3(vUV, float(vMaterial));
    vec4 color = texture(ground, texCoord);
    
    fragColor = color;
}
