# TIN Renderer Migration - Full Replacement

## Status: Loader Complete, Renderer Next

### ✅ Complete

1. **Format** (`quantizedmeshformat.h`) - Cesium Quantized Mesh spec
2. **Triangulator** (`terraincooker_tin.{h,cpp}`) - CDT + LOD generation  
3. **Cooker** (`tools/terraincook/main.cpp`) - `-cook` now generates TIN by default
4. **Loader** (`terraintileloader.{cppm,cpp}`) - Reads `.qm` files, dequantizes vertices

### 🚧 Next: Renderer (`rendering/terrainclipmap.{cppm,cpp}`)

#### Current State (Heightfield)
```cpp
// Texture arrays per LOD
terrain_tile_pool (GL_TEXTURE_2D_ARRAY)
  - heights: R16
  - materials: R8

// Shader samples texture
layout(binding = 0) uniform sampler2DArray heights;
vec4 h = texelFetch(heights, ivec3(x, y, layer));
```

#### Target State (TIN)
```cpp
// Vertex buffer per tile
struct tile_vertex {
    glm::vec3 position;   // 12 bytes
    glm::vec2 uv;         // 8 bytes  
    std::uint8_t material;// 1 byte
    std::uint8_t pad[3];  // alignment
};  // 24 bytes per vertex

// Index buffer per tile
std::vector<std::uint32_t> indices;

// Shader gets attributes
layout(location = 0) in vec3 position;
layout(location = 1) in vec2 uv;
layout(location = 2) in uint material_idx;
```

### Implementation Plan

#### Phase 1: Replace `terrain_tile_pool`

**Current** (`rendering/terraintilepool.cppm`):
- Manages texture array layers
- Upload = `glTexSubImage3D` to layer

**New** (`rendering/terraintilepool_tin.cppm`):
```cpp
class terrain_tile_pool {
public:
    struct tile_handle {
        gl::buffer vbo;  // vertex buffer
        gl::buffer ibo;  // index buffer
        std::uint32_t vertex_count;
        std::uint32_t index_count;
    };
    
    tile_handle upload(
        std::vector<double> const &positions_x,
        std::vector<double> const &positions_y,
        std::vector<double> const &positions_z,
        std::vector<float> const &uvs_u,
        std::vector<float> const &uvs_v,
        std::vector<std::uint8_t> const &materials,
        std::vector<std::uint32_t> const &indices,
        glm::dvec3 const &origin  // for relative positioning
    );
    
    void free( tile_handle const &Handle );
};
```

#### Phase 2: Adapt `terrain_clipmap::render()`

**Current flow**:
1. Frustum cull tiles → instance list
2. One instanced draw per LOD
3. Shader computes vertex from `gl_VertexID`
4. Shader samples heightmap texture

**New flow**:
1. Frustum cull tiles → draw list
2. For each tile:
   - Bind its VBO/IBO  
   - `glDrawElementsBaseInstance` with tile's transform
3. Shader receives vertex attributes directly

**Key change**: From massive instanced draw to per-tile indexed draws.

#### Phase 3: Update Shaders

**terrainclipmap.vert** (before):
```glsl
// No vertex input - compute from ID
layout(location = 0) out vec2 vUV;

void main() {
    int x = gl_VertexID % sideLength;
    int y = gl_VertexID / sideLength;
    
    // Fetch height from texture
    float h = texelFetch(heights, ivec3(x, y, layer), 0).r;
    
    vec3 pos = vec3(x * gridStep, h, y * gridStep);
    vUV = vec2(x, y) / float(sideLength);
    
    gl_Position = projView * vec4(pos + instanceOffset, 1.0);
}
```

**terrainclipmap.vert** (after):
```glsl
// Vertex attributes
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec2 aUV;
layout(location = 2) in uint aMaterial;

layout(location = 0) out vec2 vUV;
layout(location = 1) flat out uint vMaterial;

uniform vec3 uTileOffset;  // from instance data

void main() {
    vUV = aUV;
    vMaterial = aMaterial;
    
    // Position is already world-space, just offset to camera-relative
    vec3 pos = aPosition + uTileOffset;
    gl_Position = projView * vec4(pos, 1.0);
}
```

**terrainclipmap.frag** (minimal change):
```glsl
// Material lookup stays the same
layout(binding = 1) uniform sampler2DArray groundTextures;

layout(location = 0) in vec2 vUV;
layout(location = 1) flat in uint vMaterial;

void main() {
    vec4 color = texture(groundTextures, vec3(vUV, vMaterial));
    fragColor = color;
}
```

### Critical Details

#### 1. **Camera-Relative Rendering**

Heightfield stored absolute positions in camera-relative space per frame.  
TIN vertices are **world-absolute** - subtract camera pos in shader:

```cpp
// In clipmap::render()
glm::dvec3 camera_pos = Viewpoint;

// Per tile:
glm::vec3 tile_offset = glm::vec3(
    tile_center - camera_pos  // double → float is safe at this range
);
shader.uniform("uTileOffset", tile_offset);
```

#### 2. **LOD Morphing**

Heightfield morphed in shader between LOD levels.  
TIN has **separate geometry per LOD** - just render appropriate one.

```cpp
// Select LOD based on distance (existing logic)
std::uint32_t lod = level_for(distance);

// Fetch tile at that LOD
auto tile = m_tiles[make_key(x, z, lod)];
```

#### 3. **Edge Stitching** (Phase 2)

Edge cracks between LOD levels need special handling:

```cpp
// Along tile borders, sample neighbor's edge vertices
// Use edge indices from payload

if (on_north_edge && neighbor_lod > my_lod) {
    // Fetch position from neighbor's south_edge
    position = neighbor_south_edge[edge_index];
}
```

This can wait - without it, there might be thin cracks but rendering works.

### Files to Modify

```
rendering/terrainclipmap.cppm     - Main renderer
rendering/terrainclipmap.cpp      - Implementation
rendering/terraintilepool.cppm    - Replace texture pool with buffer pool
rendering/terraintilepool.cpp     - Implementation
shaders/terrainclipmap.vert       - Vertex shader
shaders/terrainclipmap.frag       - Fragment shader (minimal)
```

### Files to Remove (Later)

```
scene/heightfieldformat.h         - Old format
scene/heightfieldreader.h         - Old reader
scene/terraincooker.h             - Old cooker
rendering/terrainground.cppm      - May be reusable for material textures
```

### Testing Strategy

1. **Render single tile**
   - Load one `.qm` file
   - Upload to VBO/IBO
   - Draw with simple shader
   - Verify vertices appear at correct world positions

2. **Render LOD 0 only**
   - Load all tiles at finest LOD
   - No morphing, no stitching
   - Check coverage, no gaps

3. **Add LOD selection**
   - Distance-based LOD (reuse existing formula)
   - Verify coarser LODs appear at distance

4. **Add edge stitching**
   - Use edge indices
   - Eliminate cracks

5. **Performance comparison**
   - Frame time vs old heightfield
   - Memory usage
   - Load time

### Next Session: Implement Phase 1

1. Create `terrain_tile_pool_tin.{cppm,cpp}`
2. Modify `terrain_clipmap::upload()` to use new pool
3. Modify `terrain_clipmap::render()` for per-tile draws
4. Update shaders

**Estimate**: 4-6 hours for working render, another 2-3 for edge stitching.

---

**Ready to start renderer implementation now.**
