/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editortool.h"
#include "application/editormodebuildservices.h"
#include "application/editormodebuildshared.h"
#include "imgui/imgui.h"

#include <cstddef>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;
class infra_binder;

// the objects along the tracks bound to their paths: read from the scenery and stored in it, recognised, bound and unbound, and
// carried along when their tracks change
class infra_binder final : public editor_tool
{
  public:
	infra_binder(editor_services &Services, editor_build_services &Build, build_shared &Shared) : m_services{Services}, m_build{Build}, m_profile{Build.vertical_profile()}, m_infra{Build.infra_state()}, m_infra_carried{Shared.infra_carried} {}
	// the bindings, the lines of the scenery kept as they are, and whether the bindings were read from the scenery
	std::vector<infra::binding> &bindings()
	{
		return m_bindings;
	}
	std::vector<std::pair<scene::layer_handle, std::string>> const &unresolved_bindings() const
	{
		return m_bindings_unresolved;
	}
	bool bindings_loaded() const
	{
		return m_bindings_loaded;
	}
	// the bindings of a track captured before it changes, carried along with the tracks committed, moved to the part created by a split
	void bindings_captured(TTrack const &Track);
	void bindings_committed(std::vector<TTrack *> const &Tracks);
	void bindings_split(TTrack &Original, TTrack &Created, segment_data const &First);
	void infra_load();
	void infra_forget(scene::basic_node const *Node);
	void infra_store();
	std::vector<TTrack *> infra_scope() const;
	infra::binding *infra_find(infra::binding const &Binding);
	std::string infra_carried_note(); // a line of the status telling how many, empty for none; clears them
	int infra_bound_count() const; // by the infra tool, not the ones a change of their track took along
	void infra_recognize();
	void infra_bind_chosen();
	void infra_unbind(std::vector<std::size_t> Indices);
	infra::object_state infra_state_of(infra::binding const &Binding) const;
	void infra_restore_state(infra::object_state const &State);
	void infra_buffer(infra::binding const &Binding, bool const Refresh);
	void infra_adopt(TTrack &Track);
	void infra_move(infra::binding &Binding);
	void infra_rebase();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	profile_state &m_profile;
	infra_window &m_infra;
	std::set<std::pair<scene::basic_node const *, scene::instance_handle>> &m_infra_carried;
	// the bindings read from the scenery, its lines whose objects or paths weren't found, kept as they are, and whether they were read
	std::vector<infra::binding> m_bindings;
	std::vector<std::pair<scene::layer_handle, std::string>> m_bindings_unresolved; // lines kept as they are
	bool m_bindings_loaded{false};
	// the point objects of the scenery gathered, for the revision of the scenery and the count of its nodes they were gathered at
	std::vector<point_object> m_point_objects;
	std::size_t m_point_objects_revision{static_cast<std::size_t>(-1)};
	std::size_t m_point_objects_count{0};
	// its steps: the point objects gathered again, one of them taken by the track
	void point_objects_gather();
	void infra_adopt_object(TTrack &Track, point_object const &Object);
};

// the panel of the objects along the tracks: the search, the objects recognised and the ones bound, and the marks of them in the view
class infra_panel final : public editor_tool
{
  public:
	infra_panel(editor_services &Services, editor_build_services &Build, infra_binder &Binder) : m_services{Services}, m_binder{Binder}, m_infra{Build.infra_state()} {}
	void render_infra_body();
	void draw_infra_overlay() const;

  private:
	editor_services &m_services;
	infra_binder &m_binder;
	infra_window &m_infra;
	void render_infra_search();
	void render_infra_candidates();
	void render_infra_bound();
};
