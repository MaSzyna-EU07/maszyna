const float pureWhite = 1.0;

vec3 reinhard(vec3 x)
{
//	return x / (x + vec3(1.0));

	// Reinhard tonemapping operator.
	// see: "Photographic Tone Reproduction for Digital Images", eq. 4
	float luminance = dot(x, vec3(0.2126, 0.7152, 0.0722));
	float mappedLuminance = (luminance * (1.0 + luminance/(pureWhite*pureWhite))) / (1.0 + luminance);
	// Scale color by ratio of average luminances.
	return (mappedLuminance / luminance) * x;
}

// https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
vec3 ACESFilm(vec3 x)
{
    float a = 2.51f;
    float b = 0.03f;
    float c = 2.43f;
    float d = 0.59f;
    float e = 0.14f;
    return (x*(a*x+b))/(x*(c*x+d)+e);
}

// https://www.slideshare.net/ozlael/hable-john-uncharted2-hdr-lighting
vec3 filmicF(vec3 x)
{
	float A = 0.22f;
	float B = 0.30f;
	float C = 0.10f;
	float D = 0.20f;
	float E = 0.01f;
	float F = 0.30f;
	return ((x*(A*x+C*B)+D*E)/(x*(A*x+B)+D*F)) - E/F;
}

vec3 filmic(vec3 x)
{
	return filmicF(x) / filmicF(vec3(11.2f));
}


// AgX tonemapping based on nxrighthere / Missing Deadlines implementation.
// 0: Default, 1: Golden, 2: Punchy
#ifndef AGX_LOOK
#define AGX_LOOK 2
#endif

vec3 AgxDefaultContrastApprox(vec3 x)
{
    vec3 x2 = x * x;
    vec3 x4 = x2 * x2;

    return 15.5 * x4 * x2
        - 40.14 * x4 * x
        + 31.96 * x4
        - 6.868 * x2 * x
        + 0.4298 * x2
        + 0.1191 * x
        - 0.00232;
}

vec3 Agx(vec3 val)
{
    mat3 agx_mat = mat3(
        0.842479062253094, 0.0784335999999992, 0.0792237451477643,
        0.0423282422610123, 0.878468636469772, 0.0791661274605434,
        0.0423756549057051, 0.0784336, 0.879142973793104
    );

    // DEFAULT_LOG2_MIN = -10.0
    // DEFAULT_LOG2_MAX = +6.5
    // MIDDLE_GRAY = 0.18
    // log2(pow(2, VALUE) * MIDDLE_GRAY)
    const float min_ev = -12.47393;
    const float max_ev = 0.526069;
    const float agx_eps = 1e-6;

    // Input transform (inset)
    val = agx_mat * val;

    // Log2 space encoding. max() avoids -INF/NaN for zero/negative inputs.
    val = clamp(log2(max(val, vec3(agx_eps))), min_ev, max_ev);
    val = (val - min_ev) / (max_ev - min_ev);

    // Apply sigmoid function approximation.
    return AgxDefaultContrastApprox(val);
}

vec3 AgxEotf(vec3 val)
{
    mat3 agx_mat_inv = mat3(
        1.19687900512017, -0.0980208811401368, -0.0990297440797205,
        -0.0528968517574562, 1.15190312990417, -0.0989611768448433,
        -0.0529716355144438, -0.0980434501171241, 1.15107367264116
    );

    // Inverse input transform (outset)
    val = agx_mat_inv * val;

    // sRGB IEC 61966-2-1 2.2 Exponent Reference EOTF Display.
    // If your render target already applies sRGB conversion, replace this with:
    // return max(val, vec3(0.0));
    return pow(max(val, vec3(0.0)), vec3(2.2));
}

vec3 AgxLook(vec3 val)
{
    vec3 lw = vec3(0.2126, 0.7152, 0.0722);
    float luma = dot(val, lw);

    vec3 offset = vec3(0.0);
    vec3 slope = vec3(1.0);
    vec3 power = vec3(1.0);
    float sat = 1.0;

#if AGX_LOOK == 1
    // Golden
    slope = vec3(1.0, 0.9, 0.5);
    power = vec3(0.8);
    sat = 0.8;
#elif AGX_LOOK == 2
    // Punchy
    slope = vec3(1.0);
    power = vec3(1.35);
    sat = 1.4;
#endif

    // ASC CDL
    val = pow(max(val * slope + offset, vec3(0.0)), power);
    return vec3(luma) + sat * (val - vec3(luma));
}

vec3 ApplyAgX(vec3 linearColorRec709)
{
    linearColorRec709 = Agx(linearColorRec709);
    linearColorRec709 = AgxLook(linearColorRec709);
    linearColorRec709 = AgxEotf(linearColorRec709);
    return linearColorRec709;
}

vec4 tonemap(vec4 x)
{
	// Use AgX by default. Reinhard and ACES above are kept for reference.

	// Last-line-of-defense sanitize so NaN/Inf/negative HDR values do not
	// escape into log2()/pow() and produce black flashes or invalid output.
	vec3 hdr = x.rgb;
	hdr = mix(hdr, vec3(0.0), vec3(any(isnan(hdr)) || any(isinf(hdr))));
	hdr = max(hdr, vec3(0.0));
	return FBOUT(vec4(ApplyAgX(hdr), x.a));
	//return FBOUT(vec4(ACESFilm(hdr), x.a));
	//return FBOUT(vec4(reinhard(x.rgb), x.a));
}
