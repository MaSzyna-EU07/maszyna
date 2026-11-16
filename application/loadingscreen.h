/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <memory>
#include <string>

// the screen of the loading of a scenery made with RmlUi, from ui/loading.rml in the game folder or the copy built into the
// program: the name of the scenery, what's loaded, the bars of the progress, and a piece of trivia. shown in place of the one of
// imgui while RmlUi runs (ui.rmlui); without it the one of imgui stays as it was. the background is drawn under it as before
class loading_screen
{
  public:
	// what the screen tells
	struct progress
	{
		float done{0.f}; // 0 to 1
		float subtask{0.f}; // of the part loaded now, 0 to 1
		std::string text; // what's loaded
		std::string title; // the name of the scenery, empty: none
		std::string trivia; // empty: none
	};

	loading_screen();
	~loading_screen();
	loading_screen(loading_screen const &) = delete;
	loading_screen &operator=(loading_screen const &) = delete;

	// the screen brought up to date and shown in this frame, to be drawn with the documents of RmlUi.
	// returns: whether RmlUi shows it, in place of the screen of imgui
	bool update(progress const &Progress);

  private:
	struct state;
	bool open();

	std::unique_ptr<state> m_state;
	bool m_failed{false}; // the document couldn't be loaded, it isn't tried again
};
