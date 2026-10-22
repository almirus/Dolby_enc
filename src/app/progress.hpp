#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <io.h>
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace atmos {
namespace console_style {

inline constexpr const char* kReset = "\033[0m";
inline constexpr const char* bold = "\033[1m";
inline constexpr const char* dim = "\033[2m";
inline constexpr const char* cyan = "\033[36m";
inline constexpr const char* bright_cyan = "\033[96m";
inline constexpr const char* green = "\033[32m";
inline constexpr const char* bright_green = "\033[92m";
inline constexpr const char* bright_yellow = "\033[93m";
inline constexpr const char* bright_red = "\033[91m";
inline constexpr const char* white = "\033[97m";
inline constexpr const char* bright_magenta = "\033[95m";
inline constexpr const char* hide_cursor = "\033[?25l";
inline constexpr const char* show_cursor = "\033[?25h";

inline bool stream_is_tty(FILE* stream) {
#ifdef _WIN32
    return _isatty(_fileno(stream)) != 0;
#else
    return isatty(fileno(stream)) != 0;
#endif
}

inline bool color_enabled(FILE* stream) {
    if (std::getenv("NO_COLOR") != nullptr) {
        return false;
    }
    if (const char* force = std::getenv("FORCE_COLOR")) {
        if (force[0] != '\0' && std::strcmp(force, "0") != 0) {
            return true;
        }
    }
    return stream_is_tty(stream);
}

inline void enable_virtual_terminal() {
#ifdef _WIN32
    for (const DWORD id : {STD_OUTPUT_HANDLE, STD_ERROR_HANDLE}) {
        const HANDLE handle = GetStdHandle(id);
        if (handle == INVALID_HANDLE_VALUE || handle == nullptr) {
            continue;
        }
        DWORD mode = 0;
        if (GetConsoleMode(handle, &mode)) {
            SetConsoleMode(
                handle, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
        }
    }
#endif
}

inline std::ostream& paint(
    std::ostream& output,
    bool enabled,
    const char* color) {
    if (enabled && color != nullptr && *color != '\0') {
        output << color;
    }
    return output;
}

inline std::ostream& reset(std::ostream& output, bool enabled) {
    if (enabled) {
        output << kReset;
    }
    return output;
}

} // namespace console_style

class ProgressReporter {
public:
    ProgressReporter()
        : color_(console_style::color_enabled(stderr)),
          tty_(console_style::stream_is_tty(stderr)) {}

    ProgressReporter(const ProgressReporter&) = delete;
    ProgressReporter& operator=(const ProgressReporter&) = delete;

    ~ProgressReporter() {
        finish();
    }

    void update(const char* stage, int percent = -1) {
        if (stage == nullptr) {
            stage = "";
        }
        std::unique_lock<std::mutex> lock(mutex_);
        const bool stage_changed = current_ != stage;
        if (stage_changed) {
            stop_spinner(lock);
            if (line_open_) {
                finish_line(lock, last_percent_ >= 100);
            }
            current_ = stage;
            last_percent_ = -2;
            spinner_index_ = 0;
        }

        if (percent < 0) {
            last_percent_ = -1;
            line_open_ = true;
            render_indeterminate();
            start_spinner(lock);
            return;
        }

        percent = (std::min)(percent, 100);
        if (percent >= 100 && completed_stage_ == current_
            && !line_open_) {
            return;
        }
        if (!stage_changed && percent == last_percent_) {
            return;
        }
        last_percent_ = percent;
        line_open_ = true;
        if (percent >= 100) {
            stop_spinner(lock);
            render_done();
            finish_line(lock, true);
            completed_stage_ = current_;
        } else {
            render_percent(percent);
            start_spinner(lock);
        }
    }

    void done(const char* stage) {
        update(stage, 100);
    }

    void finish() {
        std::unique_lock<std::mutex> lock(mutex_);
        stop_spinner(lock);
        finish_line(lock, true);
        show_cursor();
    }

private:
    static constexpr const char* kSpinnerFrames[] = {
        "⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"};
    static constexpr int kSpinnerFrameCount =
        static_cast<int>(
            sizeof(kSpinnerFrames) / sizeof(kSpinnerFrames[0]));
    static constexpr int kBarWidth = 24;
    static constexpr int kStageWidth = 16;

    void start_spinner(std::unique_lock<std::mutex>&) {
        if (!tty_ || spinner_running_) {
            return;
        }
        spinner_stop_.store(false, std::memory_order_release);
        spinner_running_ = true;
        spinner_thread_ = std::thread([this] { spinner_loop(); });
    }

    void stop_spinner(std::unique_lock<std::mutex>& lock) {
        if (!spinner_running_) {
            return;
        }
        spinner_stop_.store(true, std::memory_order_release);
        lock.unlock();
        if (spinner_thread_.joinable()) {
            spinner_thread_.join();
        }
        lock.lock();
        spinner_running_ = false;
    }

    void spinner_loop() {
        using namespace std::chrono_literals;
        while (!spinner_stop_.load(std::memory_order_acquire)) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (line_open_ && last_percent_ >= -1
                    && last_percent_ < 100) {
                    ++spinner_index_;
                    if (last_percent_ < 0) {
                        render_indeterminate();
                    } else {
                        render_percent(last_percent_);
                    }
                }
            }
            std::this_thread::sleep_for(100ms);
        }
    }

    void hide_cursor() {
        if (!tty_ || cursor_hidden_) {
            return;
        }
        std::cerr << console_style::hide_cursor;
        cursor_hidden_ = true;
    }

    void show_cursor() {
        if (!cursor_hidden_) {
            return;
        }
        std::cerr << console_style::show_cursor << std::flush;
        cursor_hidden_ = false;
    }

    void emit_line(const std::string& line) {
        if (!tty_ && last_percent_ > 0 && last_percent_ < 100) {
            return;
        }
        hide_cursor();
        if (tty_) {
            std::cerr << '\r' << line << "\033[K" << std::flush;
        } else {
            std::cerr << line << '\n' << std::flush;
        }
        line_open_ = true;
    }

    void append_paint(
        std::ostream& output,
        const char* color) const {
        console_style::paint(output, color_, color);
    }

    void append_reset(std::ostream& output) const {
        console_style::reset(output, color_);
    }

    void append_stage(std::ostream& output) const {
        append_paint(output, console_style::bold);
        append_paint(output, console_style::white);
        output << current_;
        append_reset(output);
        if (static_cast<int>(current_.size()) < kStageWidth) {
            output << std::string(
                static_cast<std::size_t>(kStageWidth)
                    - current_.size(),
                ' ');
        }
    }

    void render_indeterminate() {
        std::ostringstream output;
        append_paint(output, console_style::bright_cyan);
        output << kSpinnerFrames[
            spinner_index_ % kSpinnerFrameCount];
        append_reset(output);
        output << ' ';
        append_stage(output);
        output << ' ';
        append_paint(output, console_style::dim);
        output << "working";
        append_reset(output);
        emit_line(output.str());
    }

    void render_percent(int percent) {
        std::ostringstream output;
        append_paint(output, console_style::bright_cyan);
        output << kSpinnerFrames[
            spinner_index_++ % kSpinnerFrameCount];
        append_reset(output);
        output << ' ';
        append_stage(output);
        output << ' ';
        append_paint(output, console_style::cyan);
        output << make_bar(percent);
        append_reset(output);
        output << ' ';
        append_paint(output, console_style::bold);
        append_paint(output, console_style::bright_cyan);
        output << percent << '%';
        append_reset(output);
        emit_line(output.str());
    }

    void render_done() {
        std::ostringstream output;
        append_paint(output, console_style::bright_green);
        output << "✔";
        append_reset(output);
        output << ' ';
        append_stage(output);
        output << ' ';
        append_paint(output, console_style::green);
        output << make_bar(100);
        append_reset(output);
        output << ' ';
        append_paint(output, console_style::bright_green);
        output << "100%";
        append_reset(output);
        emit_line(output.str());
    }

    static std::string make_bar(int percent) {
        percent = (std::max)(0, (std::min)(100, percent));
        const int filled = (percent * kBarWidth + 50) / 100;
        std::string bar;
        bar.reserve(static_cast<std::size_t>(kBarWidth) * 3U + 2U);
        bar.push_back('[');
        for (int i = 0; i < kBarWidth; ++i) {
            bar += i < filled ? "█" : "░";
        }
        bar.push_back(']');
        return bar;
    }

    void finish_line(
        std::unique_lock<std::mutex>&,
        bool keep_content) {
        if (!line_open_) {
            return;
        }
        if (tty_) {
            if (!keep_content) {
                std::cerr << '\r' << "\033[K";
            }
            std::cerr << '\n' << std::flush;
        }
        line_open_ = false;
        last_percent_ = -2;
    }

    bool color_ = false;
    bool tty_ = false;
    bool cursor_hidden_ = false;
    std::string current_;
    std::string completed_stage_;
    int last_percent_ = -2;
    int spinner_index_ = 0;
    bool line_open_ = false;
    bool spinner_running_ = false;
    std::atomic<bool> spinner_stop_{false};
    std::thread spinner_thread_;
    std::mutex mutex_;
};

} // namespace atmos
