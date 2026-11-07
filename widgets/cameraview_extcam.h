/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#pragma once

#include "stdafx.h"
#include "application/uilayer.h"

namespace ui
{
class cameraview_panel : public ui_panel
{
private:

	enum thread_state {
		STARTING,
		RUNNING,
		STOPPING,
		STOPPING_DONE,
		STOPPED,
	};

	std::atomic<thread_state> capture_state = STOPPED;
	std::atomic<thread_state> record_state = STOPPED;
	std::thread capture_workthread;
	std::thread record_workthread;

	std::condition_variable notify_var;
	uint32_t frame_cnt = 0;

	uint8_t* image_ptr = nullptr;
	std::mutex mutex;

	bool recording = false;

	// the image shown, made by the renderer of the user interface (imgui_renderer), and the frame turned to rgba for it
	std::uint64_t m_image{0};
	std::vector<std::uint8_t> m_rgba;

	void capture_func();
	void record_func();

  public:
	cameraview_panel();
	~cameraview_panel();

	void render() override;
	void render_contents() override;
	bool set_state(bool rec);

	std::string rec_name;
};
} // namespace ui
