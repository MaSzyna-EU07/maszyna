#pragma once

#include <limits>

#include "model/Texture.h"
#include "rendering/renderer.h"

class deferred_image {
public:
	// value returned by get() when the image isn't (yet) available
	static constexpr GLuint invalid_id { std::numeric_limits<GLuint>::max() };

	deferred_image() = default;
	explicit deferred_image(const std::string &p) : path(p) { }
	deferred_image(const deferred_image&) = delete;
	deferred_image(deferred_image&&) = default;
	deferred_image &operator=(deferred_image&&) = default;
	explicit operator bool() const
	{
		return image != null_handle || !path.empty();
	}

	// loads the texture on first use
	GLuint get()
	{
		if (!path.empty()) {
            image = GfxRenderer->Fetch_Texture(path, true);
			path.clear();
		}

		if (image != null_handle) {
            auto &tex = GfxRenderer->Texture(image);
			tex.create();

			if (tex.get_is_ready())
				return static_cast<GLuint>(tex.get_id());
		}

		return invalid_id;
	}

	glm::ivec2 size() const
	{
		if (image != null_handle) {
            auto const &tex = GfxRenderer->Texture(image);
			return glm::ivec2(tex.get_width(), tex.get_height());
		}
		return glm::ivec2();
	}

private:
	std::string path;
	texture_handle image = 0;
};
