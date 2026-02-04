#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <mutex>
#include <chrono>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

class Logger {
public:
    enum class Level {
        DEBUG = 0,
        TIME  = 1,
        INFO  = 2,
        WARN  = 3,
        ERROR_ = 4,
        NONE  = 5
    };

    // ---------- 配置 ----------
    static void setLevel(Level level) { instance().level_ = level; }
    static void enableColor(bool enable) {
        instance().color_enabled_ = enable;
#ifdef _WIN32
        if (enable) enableANSI();
#endif
    }

    // ---------- 流式日志 ----------
    class LogStream {
    public:
        LogStream(Level level, const char* tag, const char* color)
            : level_(level), tag_(tag), color_(color) {}

        ~LogStream() {
            Logger& inst = Logger::instance();
            if (level_ < inst.level_) return;

            std::lock_guard<std::mutex> lock(inst.mutex_);

            if (inst.color_enabled_) std::cerr << color_;
            std::string time_str = Logger::getFormattedTime();
            std::cerr << oss_.str();
            if (inst.color_enabled_) std::cerr << Logger::Color::RESET;
            std::cerr << '\n';
        }

        template<typename T>
        LogStream& operator<<(const T& value) {
            oss_ << value;
            return *this;
        }

        // 支持 manipulators，如 std::fixed, std::setprecision, std::endl
        LogStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
            oss_ << manip;
            return *this;
        }

    private:
        std::ostringstream oss_;
        Level level_;
        const char* tag_;
        const char* color_;
    };

    // ---------- 静态接口 ----------
    static LogStream debug() { return LogStream(Level::DEBUG, "DEBUG", Color::WHITE); }
    static LogStream info()  { return LogStream(Level::INFO,  "INFO",  Color::CYAN); }
    static LogStream time()  { return LogStream(Level::TIME,  "TIME",  Color::PINK); }
    static LogStream warn()  { return LogStream(Level::WARN,  "WARN",  Color::YELLOW); }
    static LogStream error() { return LogStream(Level::ERROR_, "ERROR_", Color::RED); }

    // ---------- RAII ScopedTimer with lap ----------
    class ScopedTimer {
    public:
        explicit ScopedTimer(const std::string& label)
            : label_(label)
        {
            start_ = std::chrono::high_resolution_clock::now();
            last_lap_ = start_;
        }

        // 打印到现在为止的耗时
        void lap(const std::string& lap_label) {
            auto now = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(now - last_lap_).count();
            last_lap_ = now;
            Logger::time() << lap_label << " = " << ms << " ms";
        }

        ~ScopedTimer() {
            auto end = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(end - start_).count();
            Logger::time() << label_ << " total = " << ms << " ms";
        }

    private:
        std::string label_;
        std::chrono::high_resolution_clock::time_point start_;
        std::chrono::high_resolution_clock::time_point last_lap_;
    };

private:
    struct Color {
        static constexpr const char* RESET   = "\033[0m";
        static constexpr const char* RED     = "\033[31m";
        static constexpr const char* GREEN   = "\033[32m";
        static constexpr const char* YELLOW  = "\033[33m";
        static constexpr const char* BLUE    = "\033[34m";
        static constexpr const char* MAGENTA = "\033[35m";
        static constexpr const char* CYAN    = "\033[36m";
        static constexpr const char* WHITE   = "\033[37m";
        static constexpr const char* PINK    = "\033[35;1m";
    };

    Logger() : level_(Level::INFO), color_enabled_(true) {}

    static Logger& instance() {
        static Logger inst;
        return inst;
    }

    static std::string getFormattedTime() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        char time_str[64];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&time_t));
        return time_str;
    }

#ifdef _WIN32
    static void enableANSI() {
        static bool enabled = false;
        if (enabled) return;
        HANDLE hOut = GetStdHandle(STD_ERROR_HANDLE);
        if (hOut == INVALID_HANDLE_VALUE) return;
        DWORD mode = 0;
        if (!GetConsoleMode(hOut, &mode)) return;
        SetConsoleMode(hOut, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
        enabled = true;
    }
#endif

private:
    Level level_;
    bool color_enabled_;
    std::mutex mutex_;
};
