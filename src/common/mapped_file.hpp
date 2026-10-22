#pragma once

#include <cstdint>
#include <filesystem>
#include <span>

namespace atmos {

class MappedFile final {
public:
    explicit MappedFile(const std::filesystem::path& path);
    ~MappedFile();
    MappedFile(const MappedFile&) = delete;
    MappedFile& operator=(const MappedFile&) = delete;
    [[nodiscard]] std::span<const std::uint8_t> bytes() const noexcept { return {view_, size_}; }

private:
    void close() noexcept;
    std::intptr_t file_ = -1;
    void* mapping_ = nullptr;
    const std::uint8_t* view_ = nullptr;
    std::size_t size_ = 0;
};

} // namespace atmos
