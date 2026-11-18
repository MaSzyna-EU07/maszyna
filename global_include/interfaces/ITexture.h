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
	// uploads an image from a thread with a gl context of its own, sharing the objects with the drawing one. a texture that
	// can keeps the image apart until commit_thread_upload(), called on the drawing thread, puts it in place
	virtual void upload_from_thread(size_t width, size_t height, const uint8_t *data)
	{
		update_from_memory(width, height, data);
	}
	// puts in place the image of the last upload_from_thread(), once the gpu has it. called on the drawing thread
	virtual void commit_thread_upload() { /* the image went in place right away */ }
	static ITexture *null_texture()
	{
		struct NullTexture : public ITexture
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
			void release() override { /* null implementation, nothing to do */ }
			void make_stub() override { /* null implementation, nothing to do */ }
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
			void set_components_hint(int hint) override { /* null implementation, nothing to do */ }
			void make_from_memory(size_t width, size_t height, const uint8_t *data) override { /* null implementation, nothing to do */ }
			void update_from_memory(size_t width, size_t height, const uint8_t *data) override { /* null implementation, nothing to do */ }
		};
		static NullTexture null_texture{};
		return &null_texture;
	}
	virtual ~ITexture() = default;
};