#include "stdafx.h"
#include "MeshRenderSystem.h"

#include "entitysystem/ECWorld.h"
#include "entitysystem/components/BasicComponents.h"
#include "entitysystem/components/RenderComponents.h"

// gfx_renderer does not expose per-handle draw calls — geometry is drawn
// by the legacy pipeline (simulation::Region → GfxRenderer->Render()).
// MeshRenderer.visible is the authoritative visibility flag; LODSystem writes
// it and the legacy scene node reads it when the renderer traverses the scene.
// When a custom ECS draw path is wired into the renderer this system will
// issue the actual draw calls.
void MeshRenderSystem::Render(ECWorld& world)
{
}
