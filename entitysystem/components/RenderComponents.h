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
/// Component containing data for LOD controller
/// </summary>
struct LODController
{
	double RangeMin;
	double RangeMax;
};

} // namespace ECSComponent
#endif // EU07_RENDERCOMPONENTS_H
