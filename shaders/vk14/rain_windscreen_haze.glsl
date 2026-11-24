// the haze of the wet windscreen (mat_rain_windscreen.frag), drawn by the post-processing: the glass asks for it in the velocity
// target (x = WS_ENC * strength, y = 0, see mat_rain_windscreen.frag), and the picture there is blurred over the hazy glass only.
// postfx_taa.frag does it with the temporal anti-aliasing (every frame, also standing still), postfx_motionblur.frag without it
// (it runs only while something moves). the picture blurred by the taa is marked in its alpha (WS_HAZED), so the motion blur
// after it leaves it as it is.
//
// real motion vectors are tiny (<< 0.25 per frame), so the large values can only come from the windscreen
#ifndef RAIN_WINDSCREEN_HAZE_GLSL
#define RAIN_WINDSCREEN_HAZE_GLSL

const float WS_ENC    = 64.0;    // must match mat_rain_windscreen.frag
const float WS_FLOOR  = 0.004;   // = 0.256 in raw velocity units: everything below is real motion
const float WS_HAZED  = -4.0;    // alpha of the picture the taa already blurred: below anything a scene writes (additive
                                 // particles took the alpha of the image above 1.5, and the motion blur then left the haze of a
                                 // bug or of the glass out on some frames: a flicker). negative, so the lock of the taa takes 0

// whether the picture was blurred by the taa already (WS_HAZED in its alpha)
bool wsWasHazed(float alpha)
{
    return alpha < -3.0;
}

// blur of hazy glass
const float WS_RADIUS = 0.014;   // max blur radius, fraction of screen height
const int   WS_TAPS   = 16;
const float WS_HAZE_LIGHT_CAP = 12.0;   // the most a light adds to the haze around it (brightness, scene units)

// strength of the blur request, 0 for pixels that are not blur pixels
float wsBlur(vec2 raw)
{
    // a blended edge with something else: not a blur pixel. the haze is asked with y = 0; with msaa the samples of a pixel are
    // averaged, and the edge of a drop (y half of a drop's) came out as a full haze request: a ring blurred around the drop which
    // flickered as the edges moved over the samples
    if (raw.x > 100.0 || raw.y > 1.0)
        return 0.0;
    float m = clamp(raw.x / WS_ENC, 0.0, 1.0);
    return clamp((m - WS_FLOOR) / (1.0 - WS_FLOOR), 0.0, 1.0);
}

// a colour of the picture made safe: a very bright light can be an infinity in the half float image (and a broken pixel a nan);
// in an average that gave nan for the whole result, which the taa turns black: black dotted blobs around the lights
vec3 wsSafe(vec3 c)
{
    return any(isnan(c)) ? vec3(0.0) : clamp(c, vec3(0.0), vec3(65000.0));
}

vec3 wsFetch(sampler2D tex, vec2 uv)
{
    return wsSafe(textureLod(tex, uv, 0.0).rgb);
}

// the weight of a colour in an average that a single very bright pixel must not take over
float wsKaris(vec3 c)
{
    return 1.0 / (1.0 + dot(c, vec3(0.2126, 0.7152, 0.0722)));
}

// the picture at Uv blurred over the hazy glass, for a request of BlurAmt (> 0)
vec3 wsHaze(sampler2D colortex, sampler2D velocitytex, vec2 uv, float blurAmt)
{
    vec2 texelSize = 1.0 / vec2(textureSize(colortex, 0));
    float aspect = texelSize.x / texelSize.y;   // height / width
    vec3 sharp = wsFetch(colortex, uv);

    float radius = WS_RADIUS * blurAmt;

    // interleaved gradient noise rotates the disc per pixel -> no banding
    float rot = 6.2831853 * fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));

    float wsum = wsKaris(sharp);
    vec3  sum  = sharp * wsum;
    float psum_w = 1.0;              // the plain average beside it
    vec3  psum   = sharp;

    for (int i = 0; i < WS_TAPS; ++i)
    {
        float r = sqrt((float(i) + 0.5) / float(WS_TAPS));
        float a = float(i) * 2.3999632 + rot;     // golden angle
        vec2  o = vec2(cos(a), sin(a)) * r * radius;
        o.x *= aspect;

        vec2 tap = uv + o;

        // take only hazy-glass pixels (not the cab frame, not clear glass, not drops)
        // weighed by 1 / (1 + brightness) (karis average): an ordinary picture blurs the same, but a very bright light
        // (a lamp close by, headlights) is not smeared over the whole disc at full strength, which the bloom then turned
        // into a large flat blob shaped like a clover
        float w = step(0.01, wsBlur(textureLod(velocitytex, tap, 0.0).xy));
        vec3  c = wsFetch(colortex, tap);
        psum   += c * w;
        psum_w += w;
        w   *= wsKaris(c);
        sum  += c * w;
        wsum += w;
    }

    // the karis average alone almost wipes small bright lights out of the haze (signals at night went faint): the light the
    // plain average has above it comes back, up to WS_HAZE_LIGHT_CAP, so a light stays a soft glow of its colour and only a
    // very bright one (a lamp close by) isn't smeared at full strength
    vec3  blurred = sum / wsum;
    vec3  extra   = max(psum / psum_w - blurred, vec3(0.0));
    float extraL  = dot(extra, vec3(0.2126, 0.7152, 0.0722));
    blurred += extra * min(1.0, WS_HAZE_LIGHT_CAP / max(extraL, 1e-4));
    return mix(sharp, blurred, min(blurAmt * 4.0, 1.0));
}

// ---------------------------------------------------------------------------------------------------------------------------
// refraction in the drops: the glass sends the picture shift of the drop (x, y = WS_DROP_BIAS + WS_BEND_ENC * bend, bend -1..1
// = WS_REFR_MAXPX pixels), and the picture of this frame is bent here. the depth of the whole frame (with the cab) is known here,
// so the dashboard, the frame and the roof of the cab are never pulled into a drop (the image of the last frame used by the
// glass shader can't tell them apart: white drops at night at the edges and under the wipers)
// ---------------------------------------------------------------------------------------------------------------------------
const float WS_BEND_ENC   = 60.0;    // must match mat_rain_windscreen.frag
const float WS_DROP_BIAS  = 192.0;   // must match mat_rain_windscreen.frag
const float WS_REFR_MAXPX = 96.0;    // must match mat_rain_windscreen.frag
const float WS_REFR_MIX   = 0.85;    // 0 = no refraction, 1 = only the refracted picture
const float WS_LENS_AA    = 0.75;    // above this spread of the picture between neighbouring pixels, a 4x4 footprint is averaged

// the lights in the drops: a drop is a wide-angle lens and lights up near a bright light (signals, lamps, headlights). taken from
// the mipmaps of the image the screen reflections keep (ssr_history: the last frame, or this one in the motion blur), so the glow
// is smooth: point samples of the full image caught a headlight of one pixel only at fixed offsets and drew it as grids of dots
const float WS_LIGHT_GAIN     = 1.0;   // brightness of the lights in the drops (0 = off)
const float WS_LIGHT_CAP      = 8.0;   // the most the glow adds to a drop (brightness, scene units)
const float WS_LIGHT_MIN_LUM  = 1.0;   // a light: the fine level at least this bright (scene units) above the place around it ...
const float WS_LIGHT_CONTRAST = 3.0;   // ... and this many times brighter than it
const float WS_LIGHT_FINE     = 3.5;   // mip levels: the drop's own neighbourhood (~11 px), a wider one (~32 px), the place around
const float WS_LIGHT_MID      = 5.0;
const float WS_LIGHT_WIDE     = 7.0;
const float WS_LIGHT_MID_SHARE = 0.5;  // how much the lights a little further away count

// the vulkan renderer gives the post-processing only its own images (LAYOUT_POST): no history of the reflections, no glow there
#ifndef LAYOUT_POST
layout(binding = SSR_HISTORY_TEX) uniform sampler2D ws_light_history;
#endif

// reverse z, infinite far: depth = znear / distance. a sample is "the cab" when it is much nearer than the centre and closer than ~5 m
const float WS_NEAR_RATIO = 1.6;
const float WS_NEAR_DEPTH = 0.02;

bool wsSameLayer(float d, float d0)
{
    return d <= max(d0 * WS_NEAR_RATIO, WS_NEAR_DEPTH);
}

float wsLum(vec3 c)
{
    return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

// a drop pixel (drop, rivulet): both values in 132..252
bool wsIsDrop(vec2 raw)
{
    // 132 .. 252 exactly: with msaa the edge of a drop averaged with the scene behind (half the values, up to 126) isn't a drop,
    // else its decoded bend was nonsense, pulling the picture from far away for a frame (a drop flashing)
    return raw.x > 131.5 && raw.y > 131.5;
}

vec2 wsBend(vec2 raw)
{
    return clamp((raw - WS_DROP_BIAS) / WS_BEND_ENC, -1.0, 1.0);
}

#ifndef LAYOUT_POST
// the image of the reflections around Uv as blurred as its mip level Lod, but round: a level of the mipmaps is a grid of squares
// (2^Lod pixels each), and a light read from it alone lit the drops in a square around it. a ring of taps of the finer level
// around the place, weighed as a soft disc, gives the same blur without the squares
vec3 wsHistoryRound(vec2 uv, float lod)
{
    float level = min(lod, ssr_params2.x);
    float finer = max(level - 1.0, 0.0);
    vec2  pixel = 1.0 / vec2(textureSize(ws_light_history, 0));
    float radius = 0.6 * exp2(level);                 // pixels of the image
    vec3  sum = wsSafe(textureLod(ws_light_history, uv, finer).rgb) * 0.2;
    for (int i = 0; i < 8; ++i)
    {
        float a = (float(i) + 0.5) * 0.7853982;
        sum += wsSafe(textureLod(ws_light_history, uv + vec2(cos(a), sin(a)) * radius * pixel, finer).rgb) * 0.06;
        sum += wsSafe(textureLod(ws_light_history, uv + vec2(cos(a + 0.3927), sin(a + 0.3927)) * (2.0 * radius) * pixel, finer).rgb) * 0.04;
    }
    return sum;
}

vec3 wsLightExcess(vec2 uv, float lod, vec3 wide)
{
    vec3 c = wsHistoryRound(uv, lod);
    vec3 excess = max(c - wide * WS_LIGHT_CONTRAST, vec3(0.0));
    // no threshold with an edge: a light dims away from it (the drops right by a lamp the brightest, then less and less); full
    // from twice WS_LIGHT_MIN_LUM above the place around, as much as before there
    return excess * clamp(wsLum(excess) / (2.0 * WS_LIGHT_MIN_LUM), 0.0, 1.0);
}

vec3 wsLightGlow(vec2 uv)
{
    if ((ssr_params.x < 1.0) || (textureSize(ws_light_history, 0).x < 16))
        return vec3(0.0);
    vec3 wide = wsHistoryRound(uv, WS_LIGHT_WIDE);
    // the near light in full, the one a little further away fading with it squared: the farther the drop, the less
    vec3  mid  = wsLightExcess(uv, WS_LIGHT_MID, wide);
    float midF = clamp(wsLum(mid) / (2.0 * WS_LIGHT_MIN_LUM), 0.0, 1.0);
    vec3 glow = wsLightExcess(uv, WS_LIGHT_FINE, wide) + WS_LIGHT_MID_SHARE * mid * midF;
    glow *= WS_LIGHT_GAIN;
    // a light close by is kept to the brightness of one light
    return glow * min(1.0, WS_LIGHT_CAP / max(wsLum(glow), 1e-4));
}
#else
vec3 wsLightGlow(vec2 uv)
{
    return vec3(0.0);
}
#endif

// the picture at Uv seen through the drop, for a drop pixel with the request Raw
vec3 wsDropRefraction(sampler2D colortex, sampler2D velocitytex, sampler2D depthtex, vec2 uv, vec2 raw)
{
    vec2 texelSize = 1.0 / vec2(textureSize(colortex, 0));
    float aspect = texelSize.x / texelSize.y;   // height / width
    vec3 center = wsFetch(colortex, uv);

    vec2 bend = wsBend(raw);
    vec2 uv2  = uv - bend * WS_REFR_MAXPX * texelSize;

    // the spread of the picture between neighbouring pixels (the jacobian of the shift): a drop shows a minified, flipped copy
    // of the scene, single samples would alias into a hatch there
    vec2 rx = textureLod(velocitytex, uv + vec2(texelSize.x, 0.0), 0.0).xy;
    vec2 ry = textureLod(velocitytex, uv + vec2(0.0, texelSize.y), 0.0).xy;
    vec2 jx = (wsIsDrop(rx) ? wsBend(rx) - bend : vec2(0.0)) * WS_REFR_MAXPX;
    vec2 jy = (wsIsDrop(ry) ? wsBend(ry) - bend : vec2(0.0)) * WS_REFR_MAXPX;
    float lens = length(jx) + length(jy);

    // never pull in what is much nearer than the scene behind the glass (the cab)
    vec3 refr = center;
    float d0 = textureLod(depthtex, uv, 0.0).x;
    float d1 = textureLod(depthtex, uv2, 0.0).x;
    if (wsSameLayer(d1, d0))
    {
        if (lens < WS_LENS_AA)
        {
            refr = wsFetch(colortex, uv2);
        }
        else
        {
            vec2 fx = clamp(vec2(1.0, 0.0) - jx, vec2(-10.0), vec2(10.0));
            vec2 fy = clamp(vec2(0.0, 1.0) - jy, vec2(-10.0), vec2(10.0));
            vec3  acc  = vec3(0.0);
            float wacc = 0.0;
            for (int j = 0; j < 4; ++j)
            {
                for (int i = 0; i < 4; ++i)
                {
                    vec2  t = (vec2(float(i), float(j)) + 0.5) / 4.0 - 0.5;
                    vec3  c = wsFetch(colortex, uv2 + (fx * t.x + fy * t.y) * texelSize);
                    float w = wsKaris(c);
                    acc  += c * w;
                    wacc += w;
                }
            }
            refr = acc / wacc;
        }
    }

    vec3 outc = mix(center, refr, WS_REFR_MIX);
    // the glow only in the drops themselves, strongest in their middle, where the lens bends most: the water film and the
    // rivulets between them (no bend) would show it as flat squares around the headlights of an oncoming train
    float glowW = smoothstep(0.03, 0.2, length(bend));
    if ((WS_LIGHT_GAIN > 0.0) && (glowW > 0.0))
        outc += wsLightGlow(uv) * glowW;
    return outc;
}

#endif
