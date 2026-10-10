#pragma once

using texture_handle = int;

#define null_handle (0)

struct ITexture
{
	virtual bool create(bool Static = false) = 0;
	virtual int get_width() const = 0;
	virtual int get_height() const = 0;
	virtual size_t get_id() const = 0;
	virtual void release() = 0;
	virtual void make_stub() = 0;
	virtual std::string_view get_traits() const = 0;
	virtual std::string_view get_name() const = 0;
	virtual std::string_view get_type() const = 0;
	virtual bool is_stub() const = 0;
	virtual bool get_has_alpha() const = 0;
	virtual bool get_is_ready() const = 0;
	virtual void set_components_hint(int hint) = 0;
	virtual void make_from_memory(size_t width, size_t height, const uint8_t *data) = 0;
	virtual void update_from_memory(size_t width, size_t height, const uint8_t *data) = 0;
	static ITexture *null_texture()
	{
		static struct NullTexture : public ITexture
		{
			bool create(bool Static = false) override
			{
				return false;
			}
			bool is_stub() const override
			{
				return false;
			}
			int get_width() const override
			{
				return 1;
			}
			int get_height() const override
			{
				return 1;
			}
			size_t get_id() const override
			{
				return 0;
			}
			void release() override {}
			void make_stub() override {}
			std::string_view get_traits() const override
			{
				return "";
			}
			std::string_view get_name() const override
			{
				return "";
			}
			std::string_view get_type() const override
			{
				return "";
			}
			bool get_has_alpha() const override
			{
				return false;
			}
			bool get_is_ready() const override
			{
				return false;
			}
			void set_components_hint(int hint) override {}
			void make_from_memory(size_t width, size_t height, const uint8_t *data) override {}
			void update_from_memory(size_t width, size_t height, const uint8_t *data) override {}
		} null_texture{};
		return &null_texture;
	}
};