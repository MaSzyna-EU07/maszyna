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

#include <RmlUi/Core/DataModelHandle.h>

namespace Rml
{
class ElementDocument;
}

namespace rmlui
{

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
	// shown; Focus: it takes the focus from the documents shown before, else the keys go on to the simulation as they did
	void show(bool Focus);
	void hide();
	bool visible() const;
	// the values of the data model read again
	void dirty();

  private:
	std::string m_model;
	Rml::DataModelHandle m_handle;
	Rml::ElementDocument *m_element{nullptr};
};

} // namespace rmlui
