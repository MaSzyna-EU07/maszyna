#ifndef ALPHA_MASK_HLSLI
#define ALPHA_MASK_HLSLI

void AlphaMask(in float alpha) {
#if (PASS & FORWARD_LIGHTING) || defined (SHADOW)
  if(g_DrawConstants.m_AlphaThreshold >= 0. ? (alpha < g_DrawConstants.m_AlphaThreshold) : (alpha >= -g_DrawConstants.m_AlphaThreshold)) discard;
#endif
}

#endif
