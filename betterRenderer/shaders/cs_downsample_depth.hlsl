Texture2D<float> g_DepthTexture : register(t0);
RWTexture2D<float> g_Output : register(u0);
sampler depth_sampler : register(s0);

[numthreads(8, 8, 1)]
void main(uint3 PixCoord : SV_DispatchThreadID) {
  uint2 dimensions;
  g_Output.GetDimensions(dimensions.x, dimensions.y);

  float2 co = float2(PixCoord.xy) / float2(dimensions);

  float4 depths = g_DepthTexture.GatherRed(depth_sampler, co);
  g_Output[PixCoord.xy] = min(min(depths.x, depths.y), min(depths.z, depths.w));
}

