# TIN (Triangulated Irregular Network) Terrain Format

Implementation of Cesium Quantized Mesh format for adaptive terrain triangulation as an alternative to regular heightfield.

## What's Implemented

### 1. Format Definition (`scene/quantizedmeshformat.h`)

Cesium Quantized Mesh format adapted for MaSzyna:
- **Header**: 128 bytes with tile bounds, vertex/triangle counts, LOD error metrics
- **Vertex data**: Quantized uint16[3] positions (XYZ) - 6 bytes per vertex
- **UV data**: Quantized uint16[2] texture coords - 4 bytes per vertex  
- **Indices**: uint16 or uint32 depending on vertex count
- **Edge indices**: For seamless LOD stitching (north/south/east/west)
- **Extensions**: Material splat, metadata, meshlets

**Compression**: 
- Positions quantized to tile bounding box (2mm precision for 256m tiles)
- UVs quantized 0-1 range
- Optional per-tile zstd compression (same as heightfield)

### 2. Triangulator (`scene/terraincooker_tin.{h,cpp}`)

Constrained Delaunay triangulation using [CDT library](https://github.com/artem-ogre/CDT):

**Features**:
- Input: Same triangle soup as heightfield cooker (compatible interface)
- Constraint edges: Track corridors, roads, water boundaries
- Multi-LOD generation: 4 levels by default, configurable
- Edge-collapse simplification per LOD
- Automatic edge extraction for seamless tile stitching

**LOD Strategy**:
- Level 0 (finest): Full triangulation with all constraints
- Level 1-3: Progressive simplification, removing low-error vertices
- Geometric error tracked per LOD (for distance-based selection)
- Target: 30% vertex reduction per LOD level

**Statistics** (from `binary-scene-format-plan.md` measurements):
```
l204 terrain: 8.8M triangles
Expected TIN output: ~4.4M triangles (50% reduction)
Heightfield cost: 186 MB (2m grid)
TIN cost estimate: ~90-120 MB (better compression, variable density)
```

### 3. Integration (`tools/terraincook/main.cpp`)

New mode: `-cooktin` alongside existing `-cook`

```bash
terraincook <terrain.scm files> -cooktin -tilesize 256 -lodlevels 4 -out cooked/
```

**Output**: 
- `terrain_X_Z_lodN.qm` per tile per LOD
- Binary Quantized Mesh format
- Compatible with existing tile coordinate system

**Parameters**:
- `-tilesize <metres>`: Tile size (default: 256m)
- `-lodlevels <n>`: Number of LOD levels (default: 4)
- `-raw`: Disable compression (for debugging)

### 4. Key Design Decisions

**Why TIN over Heightfield?**

From measurements (`binary-scene-format-plan.md`):
- Track corridor (< 25m): 91.8% triangles have edges < 4m  
- Open field (> 1600m): Only 25.7% need fine detail
- Uniform 2m grid wastes resolution where not needed
- TIN adapts: dense along infrastructure, coarse elsewhere

**Why Cesium Quantized Mesh?**

- Industry standard, proven at scale (Cesium terrain streaming)
- Efficient: quantization + index compression
- Edge constraints solve T-junction problem (no cracks between LODs)
- Extensible: material splat, normals, metadata
- Open format, not tied to any vendor

**CDT vs Triangle vs Delaunator**:

- **CDT** (chosen): MPL-2.0, header-only, C++, constraint edges
- **Triangle**: Unclear license, C, harder to integrate
- **Delaunator**: Fast but no constraints (can't preserve track edges)

## What's NOT Yet Implemented

### 1. Loader (`rendering/terraintileloader.cppm`)

Current loader reads heightfield `.ehf` tiles. Needs:
- Parallel reader for `.qm` format
- Dequantization of vertices
- Upload to GPU (vertex buffer instead of texture array)

### 2. Renderer (`rendering/terrainclipmap.cppm`)

Current clipmap renders from texture arrays. Needs:
- Vertex buffer pool (replace `terrain_tile_pool` texture arrays)
- Index buffer per LOD
- Vertex input state (position, UV, material index)
- Edge stitching shader (use edge indices to eliminate cracks)

### 3. Material Splat

TIN vertices carry material indices. Needs:
- Per-vertex material blending in fragment shader
- Material texture array (same as heightfield)
- Possibly: bake splat into vertex colors for simpler shader

### 4. Constraint Generation

Current implementation takes constraints as input. Needs:
- Automatic extraction from track geometry
- Road boundaries from scenery
- Water edges from material boundaries

## Build Setup

### Dependencies

Add to CMake or vcpkg:
```
find_package(CDT CONFIG REQUIRED)
target_link_libraries(target PRIVATE CDT::CDT)
```

CDT is available in vcpkg:
```json
{
  "name": "cdt",
  "version": "1.4.4",
  "description": "Constrained Delaunay Triangulation"
}
```

### Files Added

```
scene/quantizedmeshformat.h         - Format specification
scene/terraincooker_tin.h           - Triangulator interface
scene/terraincooker_tin.cpp         - Triangulator implementation
tools/terraincook/main.cpp          - Integration (modified)
tools/terraincook/CMakeLists.txt    - Build config (modified)
tools/terraincook/test_tin.cpp      - Simple test program
```

## Testing

### 1. Cook Test Data

```bash
# Build terraincook with CDT dependency
cd tools/terraincook && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

# Run on small scenery subset
./terraincook ../../test_data/teren -cooktin -out cooked_tin/
```

Expected output:
```
Triangulating tiles: 100/100
Triangulation complete:
  Input triangles:  8979
  Output triangles: 4512 (49.7% reduction)
  Output vertices:  2389
Wrote 400 quantized mesh files to cooked_tin/
```

### 2. Verify Tile Format

```bash
hexdump -C cooked_tin/terrain_0_0_lod0.qm | head -20
```

Should see:
```
00000000  45 55 30 37 51 4d 53 48  01 00 00 00 ...  |EU07QMSH|
```

### 3. Compare to Heightfield

```bash
du -sh cooked/terrain.ehf cooked_tin/*.qm
```

Expected: TIN ~30-40% smaller for l204 terrain.

## Next Steps (Priority Order)

1. **Verify triangulation quality** 
   - Export preview images (height, overlay, triangle density)
   - Check edge constraints preserved
   - Measure actual reduction vs. theory

2. **Implement loader**
   - Read `.qm` files on background thread
   - Dequantize and upload to GPU
   - Integrate with existing `terrain_tile_loader` thread pool

3. **Adapt renderer**
   - Replace texture fetch with vertex attributes
   - Add edge stitching (sample coarser LOD along shared edges)
   - Preserve existing CDLOD distance logic

4. **Constraint extraction**
   - Parse track geometry from `.scm`
   - Generate corridor outline (track width + margin)
   - Feed to triangulator as constraints

5. **Performance validation**
   - Measure vs. heightfield: memory, load time, frame time
   - Confirm 50% vertex reduction translates to visible win
   - Check no regressions in visual quality

## References

- [Cesium Quantized Mesh Spec](https://github.com/CesiumGS/quantized-mesh)
- [CDT Library](https://github.com/artem-ogre/CDT)
- `binary-scene-format-plan.md` - Original measurements and rationale
- `scene/heightfieldformat.h` - Current heightfield format for comparison

## Notes

- **Hybrid approach viable**: Keep heightfield for flat areas, TIN for corridors
- **Format extensible**: Meshlets for mesh shading, normals, etc.
- **Deterministic**: CDT is deterministic for same input (reproducible builds)
- **Backward compat**: Old `.ehf` files still work, TIN is opt-in via `-cooktin`
