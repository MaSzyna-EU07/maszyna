# Local changes of the bundled ImGui

`imgui/` holds Dear ImGui **v1.91.9b-docking** (`IMGUI_VERSION "1.91.9b"`, the docking branch: upstream tag
[`v1.91.9b-docking`](https://github.com/ocornut/imgui/tree/v1.91.9b-docking)), the GLFW, OpenGL 2 and OpenGL 3 backends
of the same tag, and ImGuizmo. The changes below are ours: an update of ImGui has to carry them over, or drop them on
purpose. Most of them are marked in the code with a `MaSzyna` comment (`grep -n MaSzyna imgui/*`), but not every
changed line is.

Compared with the upstream tag, these files are unchanged: `imgui_demo.cpp`, `imgui_draw.cpp`, `imgui_internal.h`,
`imgui_tables.cpp`, `imstb_rectpack.h`, `imstb_textedit.h`, `imstb_truetype.h`, `LICENSE.txt` and the backend headers
(`imgui_impl_glfw.h`, `imgui_impl_opengl2.h`, `imgui_impl_opengl3.h`). The backends are in `backends/` upstream, here
next to the rest.

## ImGui itself

| Change | Files | Since | Used by |
|--------|-------|-------|---------|
| `io.ConfigViewportsNoAutoDetach` (default `false`): a floating window stays inside its host viewport. A resize stops at the viewport's edge, a window left past the edge or too big for it is moved back and shrunk, and it gets a viewport (an OS window) of its own only when dragged with the cursor outside the host viewport. Tooltips and popups over the main viewport, or begun outside any window, stay in the main viewport. Upstream has no such option ([#236](https://github.com/MaSzyna-EU07/maszyna/pull/236); the closest upstream report is [ocornut/imgui#9594](https://github.com/ocornut/imgui/issues/9594)) | `imgui.h` (`ImGuiIO` field), `imgui.cpp`: its default in `ImGuiIO::ImGuiIO()`; `WindowStaysInHostViewport()`, `IsWindowBeingMoved()`, `KeepWindowInsideRect()`; the resize limit in `UpdateWindowManualResize()`; two places in `Begin()` (no late viewport, window kept inside the work rect); the fallback and `ViewportAllowPlatformMonitorExtend` in `UpdateSelectWindowViewport()` | 5fa03304e (#236), be4000880, d0ca44cd6 | `application/uilayer.cpp`, with the viewports on |
| `io.ConfigTooltipsWrapWidth` (default `0`, no wrapping): the text of `SetTooltip()` / `SetItemTooltip()` wraps at that width. `BeginTooltip()` is not affected | `imgui.h` (`ImGuiIO` field), `imgui.cpp` (default, `SetTooltipV()`) | be4000880 | `application/uilayer.cpp` (40 font sizes) |
| `ImGui::BufferingBar()` and `ImGui::Spinner()`, by zfedoran from [ocornut/imgui#1901](https://github.com/ocornut/imgui/issues/1901) | `imgui.h`, end of `imgui_widgets.cpp` | ae3cecfa4 (2019, for the progress bar of the loading screen of the time), carried over in 344034ec7 | nothing at present: can be dropped |
| `#define IMGUI_IMPL_OPENGL_LOADER_CUSTOM`: the OpenGL 3 backend uses glad, loaded by the simulator, instead of its own loader | `imconfig.h` | 344034ec7 | the OpenGL 3 backend |

## Backends

| Change | File | Since |
|--------|------|-------|
| With multi-viewports, ImGui positions are relative to the simulator's main window instead of the desktop, so the main viewport stays at (0,0) and the overlays, picking and editor gizmo work in window coordinates as without viewports. `MainWindowOrigin` in the backend data, `ImGui_ImplGlfw_UpdateMainWindowOrigin()` called from `NewFrame()`; the origin is subtracted from the mouse position (callback and `UpdateMouseData()`), the monitors' `MainPos` / `WorkPos` and `GetWindowPos()`, and added back in `CreateWindow()` and `SetWindowPos()` | `imgui_impl_glfw.cpp` | 476f3bf1b |
| GL functions from glad (`<glad/glad.h>`) instead of the system's `GL/gl.h` | `imgui_impl_opengl2.cpp` | 344034ec7 |
| GL functions from glad, with `stdafx.h` and `utilities/Globals.h` included for the renderer's settings | `imgui_impl_opengl3.cpp` | 344034ec7 |
| `GlslVersionString` is 128 bytes instead of 32, for the longer GLES version string the simulator passes | `imgui_impl_opengl3.cpp` | 344034ec7 |
| Gamma of the UI follows the renderer: the vertex shaders linearise the vertex colour (`pow(Color.rgb, 2.2)`); each fragment shader's output goes through `FBOUT()`, which applies `1/2.2` when `Global.gfx_shadergamma` is set and passes it through otherwise; with desktop GL and no shader gamma `GL_FRAMEBUFFER_SRGB` is enabled while the UI is drawn and restored after | `imgui_impl_opengl3.cpp` (`RenderDrawData()`, the shader sources, `CreateDeviceObjects()`) | 344034ec7 |

## ImGuizmo

From [CedricGuillemet/ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo), added in 5a39ed519; the upstream commit
it was taken from isn't recorded. Changed in 344034ec7 for the ImGui API of 1.91:

- `IMGUI_DEFINE_MATH_OPERATORS` is defined before the first `#include "imgui.h"` (ImGui 1.86 and later want it so).
- `ImGui::CaptureMouseFromApp()` is replaced by `ImGui::SetNextFrameWantCaptureMouse(true)` (six places).

## Checking an update

Download the files of the new tag (`imgui*.cpp`, `imgui*.h`, `imstb_*.h`, `imconfig.h`, and from `backends/`
`imgui_impl_glfw.*`, `imgui_impl_opengl2.*`, `imgui_impl_opengl3.*`), diff the files here against the tag listed at the
top, and apply that diff to the new files. The diff of the files here against `v1.91.9b-docking` is exactly the list
above.
