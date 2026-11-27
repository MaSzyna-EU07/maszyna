/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "stdafx.h"
#include "widgets/cameraview_extcam.h"
#include "stb/stb_image.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"
#include "extras/piped_proc.h"
#include "rendering/renderer.h"

ui::cameraview_panel::cameraview_panel()
    : ui_panel(STR_C("Camera preview"), false)
{
	size_min = { -2, -2 };
}

void cameraview_window_callback(ImGuiSizeCallbackData *data) {
	data->DesiredSize.y = data->DesiredSize.x * (float)Global.extcam_res.y / (float)Global.extcam_res.x;
}

ui::cameraview_panel::~cameraview_panel() {
	record_state = STOPPING;
	if (record_workthread.joinable())
		record_workthread.join();

	capture_state = STOPPING;
	if (capture_workthread.joinable())
		capture_workthread.join();

	if ((m_image != 0) && (GfxRenderer != nullptr) && (GfxRenderer->GetImguiRenderer() != nullptr))
		GfxRenderer->GetImguiRenderer()->Release_Image(m_image);
}

void ui::cameraview_panel::render()
{
	if (Global.extcam_cmd.empty())
		return;

	if (record_state == STOPPING_DONE) {
		record_workthread.join();
		record_state = STOPPED;
	}

	if (capture_state == STOPPING_DONE) {
		capture_workthread.join();
		capture_state = STOPPED;
	}

	if (capture_state == STOPPED) {
		capture_state = STARTING;
		capture_workthread = std::thread(&cameraview_panel::capture_func, this);
	}

	if (recording && !(record_state == RUNNING || record_state == STARTING)) {
		record_state = STARTING;
		record_workthread = std::thread(&cameraview_panel::record_func, this);
	}
	if (!recording && !(record_state == STOPPED || record_state == STOPPING_DONE)) {
		record_state = STOPPING;
	}

	if (!is_open)
		return;

	ImGui::SetNextWindowSizeConstraints(ImVec2(200, 200), ImVec2(2500, 2500), cameraview_window_callback);

	if (auto const panelname{(title.empty() ? m_name : title) + "###" + m_name}; ImGui::Begin(panelname.c_str(), &is_open)) {
		render_contents();
	}

	ImGui::End();
}

bool ui::cameraview_panel::set_state(bool rec)
{
	if (recording == rec)
		return false;

	recording = rec;

	return true;
}

void ui::cameraview_panel::render_contents()
{
	auto *renderer{GfxRenderer->GetImguiRenderer()};
	imgui_renderer::image_options options;
	options.mipmaps = Global.python_mipmaps;
	if ((m_image == 0) && (renderer != nullptr)) {
		// grey until the first frame comes
		std::uint8_t const stub[2 * 2 * 4] = {0xc0, 0xc0, 0xc0, 0xff, 0xc0, 0xc0, 0xc0, 0xff, 0xc0, 0xc0, 0xc0, 0xff, 0xc0, 0xc0, 0xc0, 0xff};
		m_image = renderer->Create_Image(stub, 2, 2, options);
	}

	{
		std::scoped_lock lock(mutex);
		if (image_ptr && (renderer != nullptr)) {
			// the frames come as rgb
			auto const pixels{static_cast<std::size_t>(Global.extcam_res.x) * Global.extcam_res.y};
			m_rgba.resize(pixels * 4);
			for (std::size_t i = 0; i < pixels; ++i) {
				m_rgba[i * 4 + 0] = image_ptr[i * 3 + 0];
				m_rgba[i * 4 + 1] = image_ptr[i * 3 + 1];
				m_rgba[i * 4 + 2] = image_ptr[i * 3 + 2];
				m_rgba[i * 4 + 3] = 0xff;
			}
			image_ptr = nullptr;
			m_image = renderer->Update_Image(m_image, m_rgba.data(), Global.extcam_res.x, Global.extcam_res.y, options);
		}
	}

	ImVec2 surface_size = ImGui::GetContentRegionAvail();
	ImGui::Image((ImTextureID)(intptr_t)(m_image), surface_size);
}

void ui::cameraview_panel::capture_func()
{
	std::string cmdline = Global.extcam_cmd;

	piped_proc proc(cmdline);

	size_t frame_size = Global.extcam_res.x * Global.extcam_res.y * 3;
	auto read_buffer = new uint8_t[frame_size];
	auto active_buffer = new uint8_t[frame_size];

	size_t bufpos = 0;

	capture_state = RUNNING;
	while (capture_state == RUNNING) {
		size_t read = 0;
		read = proc.read(read_buffer + bufpos, frame_size - bufpos);

		if (!read)
			break;
		bufpos += read;

		if (bufpos != frame_size)
			continue;

		bufpos = 0;

		std::scoped_lock lock(mutex);
		image_ptr = read_buffer;
		read_buffer = active_buffer;
		active_buffer = image_ptr;

		frame_cnt++;
		notify_var.notify_one();
	}

	std::scoped_lock lock(mutex);
	image_ptr = nullptr;
	delete[] read_buffer;
	delete[] active_buffer;

	capture_state = STOPPING_DONE;
}

void ui::cameraview_panel::record_func()
{
	std::string cmdline = Global.extcam_rec;

	if (!rec_name.empty()) {
		const std::string magic{"{RECORD}"};
		size_t magicpos = cmdline.find(magic);
		if (magicpos != -1)
			cmdline.replace(magicpos, magic.size(), rec_name);
	}

	piped_proc proc(cmdline, true);

	size_t frame_size = Global.extcam_res.x * Global.extcam_res.y * 3;
	auto read_buffer = new uint8_t[frame_size];
	uint32_t last_cnt = 0;
	size_t bufpos = frame_size;

	record_state = RUNNING;
	while (record_state == RUNNING) {
		if (bufpos == frame_size)
		{
			std::unique_lock lock(mutex);
			auto r = notify_var.wait_for(lock, std::chrono::milliseconds(50), [this, last_cnt]{return last_cnt != frame_cnt;});
			last_cnt = frame_cnt;
			if (!image_ptr || !r)
				continue;

			bufpos = 0;
			memcpy(read_buffer, image_ptr, frame_size);
		}
		bufpos += proc.write(read_buffer + bufpos, frame_size - bufpos);
	}

	delete[] read_buffer;

	record_state = STOPPING_DONE;
}
