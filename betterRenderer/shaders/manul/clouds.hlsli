#ifndef CLOUDS_HLSLI
#define CLOUDS_HLSLI

#include "sky.hlsli"

Texture2D<float4> g_Clouds : register(t15);
Texture2D<float> g_HighClouds : register(t16);
SamplerState g_CloudsSampler : register(s15);

float3 desaturate(float3 col, float amount) {
  return lerp(col, dot(col, float3(.2126, .7152, .0722)), amount);
}

// https://iquilezles.org/articles/smin/
// sigmoid
float smin( float a, float b, float k )
{
  k *= log(2.0);
  float x = b-a;
  return a + x/(1.0-exp2(x/k));
}

float ComputeTopDown(float value) {
  value = -0.9501426 * value * value +  2.09511187 * value + -0.16186117;
  return -smin(-value, 0., .045);
}

void CalcClouds(inout float3 color, in float3 viewDir, in float3 sunDir) {
  float3 emissive_top = 1.e-7;
  float3 emissive_sun = linear_srgb_from_spectral_samples(sun_spectral_irradiance) * exp(-4.);
  float3 emissive_view = 1.e-7;
  CalcAtmosphere(emissive_top, float3(0., 1., 0.), sunDir);
  CalcAtmosphere(emissive_sun, sunDir, sunDir);
  CalcAtmosphere(emissive_view, viewDir, sunDir);
  float3 cloud_dir = viewDir;
  cloud_dir.y = 4. * abs(cloud_dir.y);
  cloud_dir = normalize(cloud_dir);
  float4 cloud_mask = g_Clouds.SampleLevel(g_CloudsSampler, cloud_dir.xz * .5 + .5, 0.);
  float high_cloud_mask = g_HighClouds.SampleLevel(g_CloudsSampler, cloud_dir.xz * .5 + .5, 0.) * .5;

  float selector = atan2(sunDir.z, sunDir.x) / TWO_PI;
  selector -= floor(selector);
  selector *= 3.;
  int idx = floor(selector);
  float cloud_lit = lerp(cloud_mask[idx], cloud_mask[(idx + 1) % 3], frac(selector));

  float topdown = ComputeTopDown(saturate(viewDir.y));

  float3 ndotl = saturate(dot(viewDir, sunDir) * .5 + .5);
  float shine = pow(ndotl, 17.);

  float3 shadow_color = desaturate(lerp(emissive_view, emissive_top, .5), .5);// * lerp(1., .1, shine);
  float3 lit_color = lerp(emissive_view, emissive_sun, .5) * lerp(1., 4., shine);



  cloud_lit = pow(cloud_lit, lerp(lerp(1.6, 1.2, topdown), 3., shine));

  float3 cloud_color = lerp( shadow_color, lit_color, cloud_lit);
  cloud_color = lerp(cloud_color, emissive_view, smoothstep(.05, 0., topdown));

  float3 high_cloud_color = lit_color;
  high_cloud_color = lerp(high_cloud_color, emissive_view, smoothstep(.05, 0., topdown));

  color = lerp(color, high_cloud_color, high_cloud_mask * smoothstep(-.025, .025, viewDir.y));
  color = lerp(color, cloud_color, cloud_mask.a * smoothstep(-.025, .025, viewDir.y));
}

#endif
