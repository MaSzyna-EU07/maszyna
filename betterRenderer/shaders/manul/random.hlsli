
uint Hash(uint s);
uint Hash(uint2 s);
uint Hash(uint3 s);
uint Hash(float s);
uint HashCombine(uint seed, uint value);

// Evolving Sub-Grid Turbulence for Smoke Animation
// H. Schechter and R. Bridson
uint Hash(uint s) {
  s ^= 2747636419u;
  s *= 2654435769u;
  s ^= s >> 16u;
  s *= 2654435769u;
  s ^= s >> 16u;
  s *= 2654435769u;
  return s;
}

uint Hash(uint2 s) {
  return HashCombine(Hash(s.x), s.y);
}

uint Hash(uint3 s) {
  return HashCombine(Hash(s.xy), s.z);
}

uint HashCombine(uint seed, uint value) {
  return seed ^ (Hash(value) + 0x9e3779b9u + (seed<<6u) + (seed>>2u));
}

uint Rand(inout uint seed) {
  seed = Hash(seed);
  return seed;
}

float RandF(inout uint seed) {
  return float(Rand(seed)) * 2.3283064365386963e-10;
}
