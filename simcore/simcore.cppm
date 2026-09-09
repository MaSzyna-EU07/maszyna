/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Primary interface unit for the simulation core. It only re-exports the
// partitions; every declaration still lives in the file it always lived in.
// These pieces reference each other in both directions, so C++20 modules require
// them to share a module -- see :fwd for why.

export module eu07.simcore;

export import :fwd;
export import :mover;
export import :model3d;
export import :animmodel;
export import :scenenode;
export import :scene;
export import :event;
export import :memcell;
export import :evlaunch;
export import :segment;
export import :trkfoll;
export import :track;
export import :traction;
export import :driver;
export import :aircoupler;
export import :button;
export import :tractionpower;
export import :dynobj;
export import :editoruipanels;
export import :editoruilayer;
