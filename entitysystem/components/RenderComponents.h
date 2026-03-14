#ifndef EU07_RENDERCOMPONENTS_H
#define EU07_RENDERCOMPONENTS_H

namespace ECSComponent
{
/// <summary>
/// Component for entities that can be rendered.
/// </summary>
/// <remarks>Currently empty
/// TODO: Add component members
/// </remarks>
struct MeshRenderer
{
};

/// <summary>
/// Component for entities that can cast shadows.
/// </summary>
/// <remarks>Currently empty
/// TODO: Add component members
/// </remarks>
struct SpotLight{};

/// <summary>
/// Component for entities that can be rendered with LOD
/// </summary>
struct LODController
{
	double RangeMin;
	double RangeMax;
};

/// <summary>
/// Component for entities that can be rendered with LOD
/// </summary>
/// <remarks>
/// Has higher priority as it's for node objects
/// </remarks>
struct NodeLODController
{
	double RangeMin;
	double RangeMax;
};

} // namespace ECSComponent
#endif // EU07_RENDERCOMPONENTS_H
