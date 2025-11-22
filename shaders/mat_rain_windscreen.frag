// Windshield rain effects
// by @lcddisplay; wiper movement @MichauSto

in vec3 f_normal;
in vec2 f_coord;
in vec4 f_pos;
#include <common>

layout(location = 0) out vec4 out_color;

#if MOTIONBLUR_ENABLED
layout(location = 1) out vec4 out_motion;
#endif

#texture(diffuse,         0, sRGB_A)
#texture(raindropsatlas,  1, sRGB_A)
#texture(wipermask,       2, sRGB_A)
#param (color, 0, 0, 4, diffuse)
#param (diffuse, 1, 0, 1, diffuse)
#param (specular, 1, 1, 1, specular)
#param (reflection, 1, 2, 1, zero)
#param (glossiness, 1, 3, 1, glossiness)
#param (raindrop_grid_size, 2, 0, 1, one)

#include <light_common.glsl>
#include <apply_fog.glsl>
#include <tonemapping.glsl>
#include <random.glsl>

uniform sampler2D diffuse;
uniform sampler2D raindropsatlas;
uniform sampler2D wipermask;

uniform float specular_intensity = 1.0;
uniform float wobble_strength    = 0.002;
uniform float wobble_speed       = 30.0;

vec4 getDropTex(float choice, vec2 uv) {
    vec2 offset;
    if      (choice < 0.25) offset = vec2(0.0, 0.0);
    else if (choice < 0.5)  offset = vec2(0.5, 0.0);
    else if (choice < 0.75) offset = vec2(0.0, 0.5);
    else                    offset = vec2(0.5, 0.5);
    return texture(raindropsatlas, offset + uv * 0.5);
}

float GetMixFactor(in vec2 co, out float side);

void main() {
    vec4 tex_color = texture(diffuse, f_coord);
    if (tex_color.a < 0.01) discard;

    vec2 rainCoord = f_coord;
    float gridSize = ceil(param[2].x);

    const float numDrops      = 20000.0;
    const float cycleDuration = 4.0;
    
    float squareMin     = 0.5 / gridSize;
    float squareMax     = 1.2 / gridSize;

    vec2  cell     = floor(rainCoord * gridSize);

    vec3 dropLayer = vec3(0.0);
    float dropMaskSum = 0.0;

    // Grid of 9 droplets in immediate neighbourhood
    for (int oy = -1; oy <= 1; oy++) {
        for (int ox = -1; ox <= 1; ox++) {

            vec2 neighborCell = cell + vec2(ox, oy);
            vec2 neighborCenter = (neighborCell + .5) / gridSize;
            
            float side;
            float mixFactor = GetMixFactor(neighborCenter, side);
            
            uint seed = Hash(uvec3(neighborCell, side));
            
            if(mixFactor < RandF(seed)) {
              continue;
            }

            // Show a percentage of droplets given by rain intensity param
            float activationSeed = RandF(seed);
            if (activationSeed > rain_params.x)
                continue; // kropla nieaktywna

            // Randomly modulate droplet center & size
            vec2 dropCenter = (neighborCell + vec2(RandF(seed), RandF(seed))) / gridSize;
            float squareSize = mix(squareMin, squareMax, RandF(seed));

            float lifeTime = time + RandF(seed) * cycleDuration;
            float phase    = fract(lifeTime / cycleDuration);
            float active   = clamp(1.0 - phase, 0.0, 1.0);

            // Gravity influence (TODO add vehicle speed & wind here!)
            float gravityStart = 0.5;
            float gravityPhase = smoothstep(gravityStart, 1.0, phase);
            float dropMass     = mix(0.3, 1.2, RandF(seed));
            float gravitySpeed = 0.15 * dropMass;
            vec2  gravityOffset = vec2(0.0, gravityPhase * gravitySpeed * phase);

            // Random wobble
            bool hasWobble = (RandF(seed) < 0.10);
            vec2 wobbleOffset = vec2(0.0);
            if (hasWobble && gravityPhase > 0.0) {
                float intensity = sin(time * wobble_speed + RandF(seed) * 100.) * wobble_strength * gravityPhase;
                wobbleOffset = vec2(intensity, 0.0);
            }

            vec2 slideOffset = gravityOffset + wobbleOffset;

            // Flatten droplets influenced by gravity
            float flattenAmount = smoothstep(0.1, 0.5, gravityPhase);
            float flattenX = mix(1.0, 0.4, flattenAmount);
            float stretchY = mix(1.0, 1.6, flattenAmount);

            // Droplet local position & mask
            vec2 diff = (rainCoord + slideOffset) - dropCenter;
            diff.x *= 1.0 / flattenX;
            diff.y *= 1.0 / stretchY;
            float mask = smoothstep(squareSize * 0.5, squareSize * 0.45, max(abs(diff.x), abs(diff.y)));

            if (mask > 0.001) {
                vec2 localUV = (diff + squareSize * 0.5) / squareSize;
                float choice = RandF(seed);
                vec4 dropTex = getDropTex(choice, localUV);
                float sharpAlpha = smoothstep(0.3, 0.9, dropTex.a);

                float ambLum = clamp(dot(ambient, vec3(0.2126, 0.7152, 0.0722)), 0.0, 1.0);
                float sunFactor = pow(clamp(ambLum * 6.0, 0.0, 1.0), 0.5);
                float dynBright = mix(0.0, 1.0, sunFactor);

                float colorLuma  = length(dropTex.rgb);
                float alphaRange = smoothstep(0.1, 0.3, colorLuma);
                float blackAlpha = mix(0.25, 0.85, alphaRange);

                float sparkle = 0.0;
                if (hasWobble) {
                    sparkle = pow(abs(sin(f_coord.x * 8.0 + time * 2.0 + RandF(seed) * 6.2831)), 40.0)
                              * mix(0.2, 1.0, sunFactor);
                }

                vec3 specularColor = vec3(1.0) * specular_intensity * sparkle;
                vec3 dropLit = dropTex.rgb * dynBright + specularColor * sharpAlpha;

                dropLayer   += dropLit * sharpAlpha * active * blackAlpha * mask;
                dropMaskSum += sharpAlpha * active * blackAlpha * mask;
            }
        }
    }
    vec3  finalMix = dropLayer;
    float alphaOut = clamp(dropMaskSum, 0.0, 1.0);
    out_color = vec4(finalMix, alphaOut);
    
    { // Overlay windshield texture with alpha
      vec3 fragcolor = ambient;
      vec3 fragnormal = normalize(f_normal);
      float reflectivity = param[1].z;
      float specularity = (tex_color.r + tex_color.g + tex_color.b) * 0.5;
      glossiness = abs(param[1].w);
      
      fragcolor = apply_lights(fragcolor, fragnormal, tex_color.rgb, reflectivity, specularity, shadow_tone);
      vec4 color = vec4(fragcolor, tex_color.a * alpha_mult);
      
      out_color.xyz = apply_fog(mix(out_color.xyz, color.xyz, color.a));
      out_color.a = mix(out_color.a, 1., color.a);
    }

#if MOTIONBLUR_ENABLED
    out_motion = vec4(0.0);
#endif
}

float GetMixFactor(in vec2 co, out float side) {
    vec4 movePhase  = wiper_pos;
    bvec4 is_out = lessThanEqual(movePhase, vec4(1.));
    movePhase = mix(vec4(2.) - movePhase, movePhase, is_out);

    vec4 mask = texture(wipermask, co);
    
    vec4 areaMask = step(.001, mask);

    vec4 maskVal   = mix(1. - mask, mask, is_out);
    vec4 wipeWidth = smoothstep(1., .9, movePhase) * .25;
    vec4 cleaned   = smoothstep(movePhase - wipeWidth, movePhase, maskVal) * areaMask;
    vec4 side_v = step(maskVal, movePhase);
    cleaned *= side_v;
    side_v = mix(side_v, vec4(1.) - side_v, is_out);

    // "regeneration", raindrops gradually returning after wiper pass:
    vec4 regenPhase = clamp((vec4(time) - mix(wiper_timer_out, wiper_timer_return, side_v) - vec4(.2)) / vec4(rain_params.y), vec4(0.), vec4(1.));
    
    side_v = mix(vec4(0.), vec4(1.) - side_v, areaMask);
    
    vec4 factor_v = mix(vec4(1.), regenPhase * (vec4(1.) - cleaned), areaMask);
    
    side = 0.;
    float out_factor = 1.;
    
    // Find out the wiper blade that influences given grid cell the most
    for(int i = 0; i < 4; ++i)
    {
      bool is_candidate = factor_v[i] < out_factor;
      out_factor = mix(out_factor, factor_v[i], is_candidate);
      side = mix(side, side_v[i], is_candidate);
    }
    
    return out_factor;
}
