#include "manul/math.hlsli"
#include "manul/material.hlsli"
#include "manul/color_transform.hlsli"

sampler diffuse_sampler : register(s0);
sampler raindrop_sampler : register(s1);
sampler wipermask_sampler : register(s2);
Texture2D<float4> diffuse : register(t0);
Texture2D<float4> raindropsatlas : register(t1);
Texture2D<float4> wipermask : register(t2);
Texture2D<float> rain : register(t3);

// Project the surface gradient (dhdx, dhdy) onto the surface (n, dpdx, dpdy)
float3 CalculateSurfaceGradient(float3 n, float3 dpdx, float3 dpdy, float dhdx, float dhdy)
{
  float3 r1 = cross(dpdy, n);
  float3 r2 = cross(n, dpdx);

  return (r1 * dhdx + r2 * dhdy) / dot(dpdx, r1);
}

// Move the normal away from the surface normal in the opposite surface gradient direction
float3 PerturbNormal(float3 n, float3 dpdx, float3 dpdy, float dhdx, float dhdy)
{
  return normalize(n - CalculateSurfaceGradient(n, dpdx, dpdy, dhdx, dhdy));
}

// Calculate the surface normal using screen-space partial derivatives of the height field
float3 CalculateSurfaceNormal(float3 position, float3 normal, float2 gradient)
{
  float3 dpdx = ddx(position);
  float3 dpdy = ddy(position);

  float dhdx = gradient.x;
  float dhdy = gradient.y;

  return PerturbNormal(normal, dpdx, dpdy, dhdx, dhdy);
}

void MaterialPass(inout MaterialData material) {
#if PASS & FORWARD_LIGHTING

    MaterialData material_glass = material;
    float4 tex_color = diffuse.Sample(diffuse_sampler, material.m_TexCoord);
    uint2 size;
    rain.GetDimensions(size.x, size.y);
    float droplet_distance = rain.Sample(raindrop_sampler, material.m_ScreenCoord);
    float droplet_distance_x = rain.Sample(raindrop_sampler, material.m_ScreenCoord, int2(1, 0));
    float droplet_distance_y = rain.Sample(raindrop_sampler, material.m_ScreenCoord, int2(0, 1));
    float2 gradient = float2(droplet_distance_x - droplet_distance, droplet_distance_y - droplet_distance);
    material_glass.m_MaterialAlbedoAlpha.xyz = 0.;
    material_glass.m_MaterialNormal = material.m_Normal;
    material_glass.m_MaterialParams.g = .2;
    float3 normal = CalculateSurfaceNormal(material_glass.m_Position, material_glass.m_Normal, gradient * -.0075);
    material_glass.m_MaterialNormal = normal;
    float cosTheta = saturate(dot(-normalize(material_glass.m_Position), normal));
    material.m_MaterialAlbedoAlpha.a = lerp(.1, FresnelSchlickRoughness(cosTheta, .04, 0.), smoothstep(0., .15, droplet_distance));

    float3 normal_world = mul((float3x3)g_InverseModelView, material_glass.m_MaterialNormal);

    float4 glass_lit;
    ApplyMaterialLighting(glass_lit, material_glass, material_glass.m_PixelCoord);

    material.m_MaterialEmission = glass_lit * smoothstep(0., .15, droplet_distance) * smoothstep(-1., 0., normal_world.y);

    material.m_MaterialAlbedoAlpha.xyz = 0.;
    material.m_RefractionOffset = normal.xy * (.005 / (length(material.m_Position) * tan(.5 * g_VerticalFov))) * smoothstep(0., .15, droplet_distance);

    float glass_opacity = FresnelSchlickRoughness(saturate(dot(-normalize(material.m_Position), material.m_Normal)), .2, 0.);
    material.m_MaterialEmission = lerp(material.m_MaterialEmission, 0., glass_opacity);
    material.m_MaterialAlbedoAlpha.a = lerp(material.m_MaterialAlbedoAlpha.a, 1., glass_opacity);
    material.m_MaterialParams.g = .05;
    material.m_MaterialNormal = material.m_Normal;


    { // Overlay windshield texture with alpha
        material.m_MaterialAlbedoAlpha.xyz = lerp(material.m_MaterialAlbedoAlpha.xyz, tex_color.xyz, tex_color.a);
        material.m_MaterialAlbedoAlpha.a = lerp(material.m_MaterialAlbedoAlpha.a, 1., tex_color.a);
        material.m_MaterialEmission.xyz = lerp(material.m_MaterialEmission.xyz, 0., tex_color.a);
        material.m_MaterialParams.g = lerp(material.m_MaterialParams.g, float4(0., .5, 1., .5), tex_color.a);
    }
#endif
}
