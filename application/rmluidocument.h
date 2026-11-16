/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <RmlUi/Core/DataModelHandle.h>

namespace Rml
{
class ElementDocument;
}

namespace rmlui
{

// a text of the game data for the documents: in UTF-8 (the timetables and the scenery are in Windows-1250), the characters below
// the space as spaces, and with Underscores the underscores as well, as Bezogonkow() takes them
std::string game_text(std::string Text, bool Underscores);

// a document of RmlUi with the data model it's bound to: ui/<name>.rml of the game folder, or the copy of it built into the
// program when the folder has none. closed, its model removed, when it goes
class document
{
  public:
	using binder = std::function<void(Rml::DataModelConstructor &)>;

	document() = default;
	~document();
	document(document const &) = delete;
	document &operator=(document const &) = delete;

	// the data model Model made by Bind, then the document File (of the game folder) or Builtin loaded. false: RmlUi doesn't
	// run, or the document couldn't be loaded (the reason is in the log)
	bool open(std::string_view Model, std::string_view File, std::span<unsigned char const> Builtin, binder const &Bind);
	bool is_open() const
	{
		return m_element != nullptr;
	}
	// the document itself, nullptr until it's open
	Rml::ElementDocument *element() const
	{
		return m_element;
	}
	// shown; Focus: it takes the focus from the documents shown before, else the keys go on to the simulation as they did
	void show(bool Focus);
	void hide();
	// shown in this frame only: hidden in the first frame this isn't called in (a screen of a mode)
	void show_in_frame();
	bool visible() const;
	// the values of the data model read again
	void dirty();
	Rml::DataModelHandle &model()
	{
		return m_handle;
	}

  private:
	std::string m_model;
	Rml::DataModelHandle m_handle;
	Rml::ElementDocument *m_element{nullptr};
};

// the values a data model is bound to, kept apart from those worked out anew each frame: a value which changed is copied and its
// variable marked changed, the others are left alone, so RmlUi updates (and lays out again) only what changed. the members bound
// are compared with ==
template <typename Values> class bound_values
{
  public:
	Values &current()
	{
		return m_values;
	}

	// the member bound to the variable of the name
	template <typename Type> void bind(Rml::DataModelConstructor &Constructor, Rml::String const &Name, Type Values::*Member)
	{
		Constructor.Bind(Name, &(m_values.*Member));
		m_updates.emplace_back([Name, Member](Values &Current, Values const &Fresh, Rml::DataModelHandle &Handle) {
			if (Current.*Member == Fresh.*Member)
				return;
			Current.*Member = Fresh.*Member;
			Handle.DirtyVariable(Name);
		});
	}

	// the values worked out anew, those which changed taken
	void update(Values const &Fresh, document &Document)
	{
		for (auto const &take : m_updates)
			take(m_values, Fresh, Document.model());
	}

  private:
	Values m_values;
	std::vector<std::function<void(Values &, Values const &, Rml::DataModelHandle &)>> m_updates;
};

} // namespace rmlui
