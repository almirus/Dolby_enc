#include "mapped_file.hpp"
#include <limits>
#include <stdexcept>
#include <system_error>

#ifdef _WIN32
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace atmos {

MappedFile::MappedFile(const std::filesystem::path& path) {
    try {
#ifdef _WIN32
        const auto file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                                      OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (file == INVALID_HANDLE_VALUE)
            throw std::system_error(GetLastError(), std::system_category(), "cannot open input");
        file_ = reinterpret_cast<std::intptr_t>(file);
        LARGE_INTEGER length{};
        if (!GetFileSizeEx(file, &length))
            throw std::system_error(GetLastError(), std::system_category(), "cannot size input");
        if (length.QuadPart < 0 || static_cast<std::uint64_t>(length.QuadPart) > std::numeric_limits<std::size_t>::max())
            throw std::runtime_error("input is too large to map");
        size_ = static_cast<std::size_t>(length.QuadPart);
        if (!size_) return;
        mapping_ = CreateFileMappingW(file, nullptr, PAGE_READONLY, 0, 0, nullptr);
        if (!mapping_)
            throw std::system_error(GetLastError(), std::system_category(), "cannot map input");
        view_ = static_cast<const std::uint8_t*>(MapViewOfFile(mapping_, FILE_MAP_READ, 0, 0, 0));
        if (!view_)
            throw std::system_error(GetLastError(), std::system_category(), "cannot view input");
#else
        const auto file = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
        if (file < 0) throw std::system_error(errno, std::generic_category(), "cannot open input");
        file_ = file;
        struct stat info{};
        if (fstat(file, &info) != 0)
            throw std::system_error(errno, std::generic_category(), "cannot size input");
        if (info.st_size < 0 || static_cast<std::uint64_t>(info.st_size) > std::numeric_limits<std::size_t>::max())
            throw std::runtime_error("input is too large to map");
        size_ = static_cast<std::size_t>(info.st_size);
        if (!size_) return;
        auto view = mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, file, 0);
        if (view == MAP_FAILED)
            throw std::system_error(errno, std::generic_category(), "cannot map input");
        view_ = static_cast<const std::uint8_t*>(view);
#endif
    } catch (...) {
        close();
        throw;
    }
}

MappedFile::~MappedFile() { close(); }

void MappedFile::close() noexcept {
#ifdef _WIN32
    if (view_) UnmapViewOfFile(view_);
    if (mapping_) CloseHandle(mapping_);
    if (file_ != -1) CloseHandle(reinterpret_cast<HANDLE>(file_));
#else
    if (view_) munmap(const_cast<std::uint8_t*>(view_), size_);
    if (file_ != -1) ::close(static_cast<int>(file_));
#endif
    view_ = nullptr;
    mapping_ = nullptr;
    file_ = -1;
    size_ = 0;
}

} // namespace atmos
