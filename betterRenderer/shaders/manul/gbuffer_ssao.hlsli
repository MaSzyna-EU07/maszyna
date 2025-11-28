#ifndef GBUFFER_SSAO_HLSLI
#define GBUFFER_SSAO_HLSLI
Texture2D<float4> g_SSAO : register(t5);

float4 R8G8B8A8_UNORM_to_FLOAT4( uint packedInput )
{
    float4 unpackedOutput;
    unpackedOutput.x = (float)( packedInput & 0x000000ff ) / 255;
    unpackedOutput.y = (float)( ( ( packedInput >> 8 ) & 0x000000ff ) ) / 255;
    unpackedOutput.z = (float)( ( ( packedInput >> 16 ) & 0x000000ff ) ) / 255;
    unpackedOutput.w = (float)( packedInput >> 24 ) / 255;
    return unpackedOutput;
}

void DecodeVisibilityBentNormal( const uint packedValue, out float visibility, out float3 bentNormal )
{
    float4 decoded = R8G8B8A8_UNORM_to_FLOAT4( packedValue );
    bentNormal = decoded.xyz * 2.0.xxx - 1.0.xxx;   // could normalize - don't want to since it's done so many times, better to do it at the final step only
    visibility = decoded.w;
}

float4 GetBentNormal(in uint2 pixel_position) {
  float4 bent_normal = g_SSAO[pixel_position];
  bent_normal.xyz = 2. * bent_normal.xyz - 1.;
  bent_normal.z *= -1.;
  return bent_normal;
}

#endif

