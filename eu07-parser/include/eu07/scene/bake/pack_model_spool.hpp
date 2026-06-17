#pragma once

// Append-only spool: modele spłukane z PACK compose (mniejszy peak RAM).

#include <eu07/scene/binary/runtime_module.hpp>
#include <eu07/scene/runtime/nodes.hpp>

#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace eu07::scene::bake {

inline constexpr std::size_t kSpoolStreamBufferBytes = 1024u * 1024u;

class PackModelSpoolFile {
public:
    explicit PackModelSpoolFile(std::filesystem::path path)
        : path_(std::move(path)),
          out_(path_, std::ios::binary | std::ios::trunc) {
        if (!out_) {
            throw std::runtime_error("EU7 PACK spool: nie mozna utworzyc " + path_.string());
        }
        out_.rdbuf()->pubsetbuf(write_buffer_.data(), write_buffer_.size());
    }

    void append(std::vector<runtime::RuntimeModelInstance>&& models) {
        if (models.empty()) {
            return;
        }
        std::lock_guard lock(mutex_);
        for (runtime::RuntimeModelInstance& model : models) {
            binary::detail::collectModelStrings(table_, model);
            binary::detail::writeRuntimeModel(out_, table_, model);
            ++model_count_;
        }
    }

    template <typename Fn>
    void for_each_model(Fn&& fn) const {
        {
            std::lock_guard lock(mutex_);
            out_.flush();
        }
        std::ifstream in(path_, std::ios::binary);
        if (!in) {
            return;
        }
        while (in.peek() != EOF) {
            fn(binary::detail::readRuntimeModel(in, table_));
        }
    }

    [[nodiscard]] std::size_t model_count() const noexcept {
        return model_count_;
    }

    [[nodiscard]] const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    std::filesystem::path path_;
    binary::StringTable table_;
    std::size_t model_count_ = 0;
    std::array<char, kSpoolStreamBufferBytes> write_buffer_ {};
    mutable std::ofstream out_;
    mutable std::mutex mutex_;
};

class ShapeSpoolFile {
public:
    explicit ShapeSpoolFile(std::filesystem::path path)
        : path_(std::move(path)),
          out_(path_, std::ios::binary | std::ios::trunc) {
        if (!out_) {
            throw std::runtime_error("EU7 shape spool: nie mozna utworzyc " + path_.string());
        }
        out_.rdbuf()->pubsetbuf(write_buffer_.data(), write_buffer_.size());
    }

    void append(runtime::RuntimeShapeNode&& shape) {
        std::lock_guard lock(mutex_);
        binary::detail::collectShapeStrings(table_, shape);
        binary::detail::writeRuntimeShape(out_, table_, shape);
        ++shape_count_;
    }

    void append_batch(std::vector<runtime::RuntimeShapeNode>&& shapes) {
        if (shapes.empty()) {
            return;
        }
        std::lock_guard lock(mutex_);
        for (runtime::RuntimeShapeNode& shape : shapes) {
            binary::detail::collectShapeStrings(table_, shape);
            binary::detail::writeRuntimeShape(out_, table_, shape);
            ++shape_count_;
        }
    }

    template <typename Fn>
    void for_each_shape(Fn&& fn) const {
        {
            std::lock_guard lock(mutex_);
            out_.flush();
        }
        std::ifstream in(path_, std::ios::binary);
        if (!in) {
            return;
        }
        while (in.peek() != EOF) {
            fn(binary::detail::readRuntimeShape(in, table_));
        }
    }

    [[nodiscard]] std::size_t shape_count() const noexcept {
        return shape_count_;
    }

    [[nodiscard]] const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    std::filesystem::path path_;
    binary::StringTable table_;
    std::size_t shape_count_ = 0;
    std::array<char, kSpoolStreamBufferBytes> write_buffer_ {};
    mutable std::ofstream out_;
    mutable std::mutex mutex_;
};

} // namespace eu07::scene::bake
