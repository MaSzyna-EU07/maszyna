#pragma once

#include <glm/glm.hpp>
#include <string>
#include <optional>

#include "ITexture.h"

struct IMaterial
{
	virtual void finalize(bool Loadnow) = 0;
	virtual bool update() = 0;
	virtual float get_or_guess_opacity() const = 0;
	virtual bool is_translucent() const = 0;
	virtual glm::vec2 GetSize() const = 0;
	virtual std::string GetName() const = 0;
	virtual std::optional<float> GetSelfillum() const = 0;
	virtual int GetShadowRank() const = 0;
	virtual texture_handle GetTexture(int slot) const = 0;
	static IMaterial *null_material()
	{
		static struct NullMaterial : public IMaterial
		{
			void finalize(bool Loadnow) override { /* null implementation, nothing to do */ }
			bool update() override
			{
				return false;
			}
			float get_or_guess_opacity() const override
			{
				return 1.f;
			}
			bool is_translucent() const override
			{
				return false;
			}
			glm::vec2 GetSize() const override
			{
				return {-1.f, -1.f};
			}
			std::string GetName() const override
			{
				return "";
			}
			std::optional<float> GetSelfillum() const override
			{
				return std::nullopt;
			}
			int GetShadowRank() const override
			{
				return 0;
			}
			texture_handle GetTexture(int slot) const override
			{
				return 0;
			}
		} null_material{};
		return &null_material;
	}
	virtual ~IMaterial() = default;
};