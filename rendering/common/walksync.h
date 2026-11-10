/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the walks of the scene made by several threads at once (the stage M2 of claude/vulkan14-renderer-plan.md). only the vulkan renderer
// opens the window in which they are (gfx.vulkan.parallelpasses); outside of it everything here does nothing, and the opengl renderer
// never opens it. while it's open:
// - every thread walking the scene holds the walk lock shared (a pass of a walker from the start to the end, the main thread from the
//   opening to the closing). what the walks make the first time they need it, and what's shared by them (the geometry of the
//   sections, the animated tracks, the records of the instances in the cells and in the gpu culling, the sleepers laid, the
//   resources a lazily loaded model brings: geometry banks, materials, textures) is made with the lock held exclusive (change): the
//   thread lets go of its shared hold meanwhile and takes it again after. the walks let go of it at the points where they hold
//   nothing which such a change could move (yield()), so a change waits only a little for them
// - the shared state of a model (the submodels the instances and the vehicles set up just before they're drawn: animations,
//   lights, the variants of the seasons) is held by the thread drawing it: a scenery model by a lock of its own (a stripe of
//   them), the vehicles all at once, as their models may be those of the scenery as well. nothing is changed while it's held
// - the static fields of TSubModel and the matrix stack of OpenGLMatrices are per thread

#pragma once

#include <cstddef>

class TModel3d;

namespace gfx
{
namespace walk_sync
{

// the window is open
bool parallel();
// the main thread opens the window (it holds the walk lock shared from now on), lets go of its hold before it waits for the walkers
// (leave()) and closes the window once they're done
void open();
void leave();
void close();

// the shared hold of a thread walking a pass (a walker); none outside of the window
class walk_scope
{
  public:
	walk_scope();
	~walk_scope();
	walk_scope(walk_scope const &) = delete;
	walk_scope &operator=(walk_scope const &) = delete;

  private:
	bool m_held{false};
};

// a change of what the walks share: made with the walk lock held exclusive, while the window is open. Wanted: false when nothing
// is to be changed after all (the change isn't waited for)
class change
{
  public:
	explicit change(bool Wanted = true);
	~change();
	change(change const &) = delete;
	change &operator=(change const &) = delete;

  private:
	bool m_exclusive{false};
	bool m_shared{false}; // the thread held the lock shared before
};

// lets a change waiting for the walk lock go first; the caller holds nothing it could move
void yield();

// a thread working for a walk which holds the walk lock and waits for it (the walks of the groups of a culling split among helpers,
// the stage M3): it's covered by the hold of that walk, takes no hold of its own and makes no changes
class borrowed_scope
{
  public:
	borrowed_scope();
	~borrowed_scope();
	borrowed_scope(borrowed_scope const &) = delete;
	borrowed_scope &operator=(borrowed_scope const &) = delete;
};

// the shared state of a scenery model held by the thread, while the window is open
class model_hold
{
  public:
	explicit model_hold(TModel3d const *Model);
	~model_hold();
	model_hold(model_hold const &) = delete;
	model_hold &operator=(model_hold const &) = delete;

  private:
	std::size_t m_stripe{0};
	bool m_held{false};
};

// the shared state of the vehicles held by the thread (all of them, with the scenery models)
class vehicle_hold
{
  public:
	vehicle_hold();
	~vehicle_hold();
	vehicle_hold(vehicle_hold const &) = delete;
	vehicle_hold &operator=(vehicle_hold const &) = delete;

  private:
	bool m_held{false};
};

} // namespace walk_sync
} // namespace gfx
