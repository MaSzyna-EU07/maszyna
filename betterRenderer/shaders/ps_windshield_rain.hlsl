#include "manul/math.hlsli"
#include "manul/material.hlsli"
#include "manul/color_transform.hlsli"
#include "manul/random.hlsli"

sampler diffuse_sampler : register(s0);
sampler raindrop_sampler : register(s1);
sampler wipermask_sampler : register(s2);
Texture2D<float4> diffuse : register(t0);
Texture2D<float4> raindropsatlas : register(t1);
Texture2D<float4> wipermask : register(t2);

float4 getDropTex(float choice, float2 uv) {
    float2 offset;
    if      (choice < .25) offset = float2(0.0, 0.0);
    else if (choice < .5)  offset = float2(0.5, 0.0);
    else if (choice < .75) offset = float2(0.0, 0.5);
    else                    offset = float2(0.5, 0.5);
    return raindropsatlas.Sample(raindrop_sampler, offset + uv * 0.5);
}

float GetMixFactor(in float2 co, out float side);

void MaterialPass(inout MaterialData material) {
#if PASS & FORWARD_LIGHTING
    const float specular_intensity = 1.;
    const float wobble_strength    = .002;
    const float wobble_speed       = 30.;

    float4 tex_color = diffuse.Sample(diffuse_sampler, material.m_TexCoord);
    if (tex_color.a < .01) discard;

    float2 rainCoord = material.m_TexCoord;
    float gridSize = ceil(200.);

    const float numDrops      = 20000.;
    const float cycleDuration = 4.;

    float squareMin     = .5 / gridSize;
    float squareMax     = 1.2 / gridSize;

    float2  cell     = floor(rainCoord * gridSize);

    float3 dropLayer = 0.;
    float dropMaskSum = 0.;

    // Grid of 9 droplets in immediate neighbourhood
    [unroll]
    for (int oy = -1; oy <= 1; ++oy) {

        [unroll]
        for (int ox = -1; ox <= 1; ++ox) {

            float2 neighborCell = cell + float2(ox, oy);
            float2 neighborCenter = (neighborCell + .5) / gridSize;

            float side;
            float mixFactor = GetMixFactor(neighborCenter, side);

            uint seed = Hash(uint3(neighborCell, side));

            if(mixFactor < RandF(seed)) {
              continue;
            }

            // Show a percentage of droplets given by rain intensity param
            float activationSeed = RandF(seed);
            if (activationSeed > g_RainParams.x)
                continue; // kropla nieaktywna

            // Randomly modulate droplet center & size
            float2 dropCenter = (neighborCell + float2(RandF(seed), RandF(seed))) / gridSize;
            float squareSize = lerp(squareMin, squareMax, RandF(seed));

            float lifeTime = g_Time + RandF(seed) * cycleDuration;
            float phase    = frac(lifeTime / cycleDuration);
            float active   = saturate(1. - phase);

            // Gravity influence (TODO add vehicle speed & wind here!)
            float gravityStart = .5;
            float gravityPhase = smoothstep(gravityStart, 1., phase);
            float dropMass     = lerp(.3, 1.2, RandF(seed));
            float gravitySpeed = .15 * dropMass;
            float2  gravityOffset = float2(0., gravityPhase * gravitySpeed * phase);

            // Random wobble
            bool hasWobble = (RandF(seed) < .10);
            float2 wobbleOffset = 0.;
            if (hasWobble && gravityPhase > 0.) {
                float intensity = sin(g_Time * wobble_speed + RandF(seed) * 100.) * wobble_strength * gravityPhase;
                wobbleOffset = float2(intensity, 0.);
            }

            float2 slideOffset = gravityOffset + wobbleOffset;

            // Flatten droplets influenced by gravity
            float flattenAmount = smoothstep(0.1, 0.5, gravityPhase);
            float flattenX = lerp(1.0, 0.4, flattenAmount);
            float stretchY = lerp(1.0, 1.6, flattenAmount);

            // Droplet local position & mask
            float2 diff = (rainCoord + slideOffset) - dropCenter;
            diff.x *= 1.0 / flattenX;
            diff.y *= 1.0 / stretchY;
            float mask = smoothstep(squareSize * 0.5, squareSize * 0.45, max(abs(diff.x), abs(diff.y)));

            if (mask > .001) {
                float2 localUV = (diff + squareSize * 0.5) / squareSize;
                float choice = RandF(seed);
                float4 dropTex = getDropTex(choice, localUV);
                float sharpAlpha = smoothstep(0.3, 0.9, dropTex.a);

                float colorLuma  = length(dropTex.rgb);
                float alphaRange = smoothstep(0.1, 0.3, colorLuma);
                float blackAlpha = lerp(0.25, 0.85, alphaRange);

                dropLayer   += dropTex.rgb * sharpAlpha * active * blackAlpha * mask;
                dropMaskSum += sharpAlpha * active * blackAlpha * mask;
            }
        }
    }
    float3 finalMix = dropLayer;
    float alphaOut = clamp(dropMaskSum, 0.0, 1.0);
    material.m_MaterialAlbedoAlpha = float4(finalMix, alphaOut);

    { // Overlay windshield texture with alpha
        material.m_MaterialAlbedoAlpha.xyz = lerp(material.m_MaterialAlbedoAlpha.xyz, tex_color.xyz, tex_color.a);
        material.m_MaterialAlbedoAlpha.a = lerp(material.m_MaterialAlbedoAlpha.a, 1., tex_color.a);
    }
#endif
}

#if PASS & FORWARD_LIGHTING
float GetMixFactor(in float2 co, out float side) {
    float4 movePhase  = g_WiperPos;
    bool4 is_out = movePhase <= 1.;
    movePhase = select(is_out, movePhase, 2. - movePhase);

    float4 mask = wipermask.Sample(wipermask_sampler, co);

    float4 areaMask = step(.001, mask);

    float4 maskVal   = select(is_out, mask, 1. - mask);
    float4 wipeWidth = smoothstep(1., .9, movePhase) * .25;
    float4 cleaned   = smoothstep(movePhase - wipeWidth, movePhase, maskVal) * areaMask;
    float4 side_v = step(maskVal, movePhase);
    cleaned *= side_v;
    side_v = select(is_out, 1. - side_v, side_v);

    // "regeneration", raindrops gradually returning after wiper pass:
    float4 regenPhase = saturate((g_Time - lerp(g_WiperTimerOut, g_WiperTimerReturn, side_v) - .2) / g_RainParams.y);

    side_v = lerp(0., 1. - side_v, areaMask);

    float4 factor_v = lerp(1., regenPhase * (1. - cleaned), areaMask);

    side = 0.;
    float out_factor = 1.;

    // Find out the wiper blade that influences given grid cell the most
    [unroll]
    for(int i = 0; i < 4; ++i)
    {
      bool is_candidate = factor_v[i] < out_factor;
      out_factor = select(is_candidate, factor_v[i], out_factor);
      side = select(is_candidate, side_v[i], side);
    }

    return out_factor;
}
#endif
