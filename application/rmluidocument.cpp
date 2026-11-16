/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/rmluidocument.h"

#include <RmlUi/Core.h>

#include "application/inisettings.h"
#include "application/rmluilayer.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

namespace rmlui
{

std::string game_text(std::string Text, bool const Underscores)
{
	for (auto &character : Text)
	{
		if (static_cast<unsigned char>(character) < 0x20U || (Underscores && character == '_'))
			character = ' ';
	}
	return ini_settings::is_utf8(Text) ? Text : win1250_to_utf8(Text);
}

document::~document()
{
	// with RmlUi ended, the document went with it
	auto *context{rmlui::context()};
	if (m_element == nullptr || context == nullptr)
		return;
	rmlui::forget(m_element);
	m_element->Close();
	// the document is unloaded with the update, the data model can go then
	context->Update();
	context->RemoveDataModel(m_model);
}

bool document::open(std::string_view const Model, std::string_view const File, std::span<unsigned char const> const Builtin, binder const &Bind)
{
	auto *context{rmlui::context()};
	if (context == nullptr || m_element != nullptr)
		return m_element != nullptr;
	m_model = Model;
	auto constructor{context->CreateDataModel(m_model)};
	if (!constructor)
	{
		ErrorLog("rmlui: can't make the data model " + m_model);
		return false;
	}
	Bind(constructor);
	m_handle = constructor.GetModelHandle();
	Rml::String const file{File};
	if (FileExists(file))
	{
		WriteLog("rmlui: " + file + " of the game folder in place of the one built in");
		m_element = context->LoadDocument(file);
	}
	else
	{
		// relative paths in the document lead to the folder of the file in the game folder, as they would from the file
		m_element = context->LoadDocumentFromMemory(Rml::String(Builtin.begin(), Builtin.end()), file);
	}
	if (m_element == nullptr)
	{
		ErrorLog("rmlui: can't load " + file);
		context->RemoveDataModel(m_model);
		return false;
	}
	return true;
}

void document::show(bool const Focus)
{
	if (m_element != nullptr && false == m_element->IsVisible())
		m_element->Show(Rml::ModalFlag::None, Focus ? Rml::FocusFlag::Document : Rml::FocusFlag::None);
}

void document::hide()
{
	if (m_element != nullptr && m_element->IsVisible())
		m_element->Hide();
}

void document::show_in_frame()
{
	rmlui::show_in_frame(m_element);
}

bool document::visible() const
{
	return m_element != nullptr && m_element->IsVisible();
}

void document::dirty()
{
	if (m_element != nullptr)
		m_handle.DirtyAllVariables();
}

} // namespace rmlui
