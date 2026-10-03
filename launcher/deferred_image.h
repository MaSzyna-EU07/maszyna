#pragma once

#include <limits>
#include <mutex>

#include "model/Texture.h"
#include "rendering/renderer.h"

class deferred_image {
public:
	// value returned by get() when the image isn't (yet) available
	static constexpr GLuint invalid_id { std::numeric_limits<GLuint>::max() };

	deferred_image() = default;
	deferred_image(const std::string &p) : path(p) { }
	deferred_image(const deferred_image&) = delete;
	deferred_image(deferred_image &&other) noexcept
	{
		std::lock_guard<std::mutex> lock(other.mutex);
		path = std::move(other.path);
		image = other.image;
	}
	deferred_image &operator=(deferred_image &&other) noexcept
	{
		if (this != &other) {
			std::scoped_lock lock(mutex, other.mutex);
			path = std::move(other.path);
			image = other.image;
		}
		return *this;
	}
	operator bool() const
	{
		std::lock_guard<std::mutex> lock(mutex);
		return image != null_handle || !path.empty();
	}

	GLuint get() const
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (!path.empty()) {
            image = GfxRenderer->Fetch_Texture(path, true);
			path.clear();
		}

		if (image != null_handle) {
            auto &tex = GfxRenderer->Texture(image);
			tex.create();

			if (tex.get_is_ready())
				return tex.get_id();
		}

		return invalid_id;
	}

	glm::ivec2 size() const
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (image != null_handle) {
            auto &tex = GfxRenderer->Texture(image);
			return glm::ivec2(tex.get_width(), tex.get_height());
		}
		return glm::ivec2();
	}

private:
	// guards lazy loading performed by the const accessors
	mutable std::mutex mutex;
	mutable std::string path;
	mutable texture_handle image = 0;
};
