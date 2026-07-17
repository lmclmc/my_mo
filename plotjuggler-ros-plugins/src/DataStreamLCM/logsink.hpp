#pragma once
#ifndef ROTATING_FILE_SINK_H
#define ROTATING_FILE_SINK_H

#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstdint>
#include <thread>
#include <queue>
#include <memory>
#include <vector>
#include <sys/time.h>
#include <lcm/lcm-cpp.hpp>
#ifdef _WIN32
#include <windows.h>
#include <direct.h>
#define mkdir(dir) _mkdir(dir)
#else
#include <unistd.h>
#include <sys/stat.h>
#endif

#include "spinmutex.hpp"

// LCM消息结构体
struct LcmMessage {
    std::string channel;
    int64_t timestamp;
    std::vector<uint8_t> data;
};

// 线程安全的消息队列
template<typename T>
class SafeQueue {
public:
    void push(const T& item) {
        std::lock_guard<lmc::SpinMutex> lock(mutex_);
        queue_.push(item);
    }

    T pop() {
        std::lock_guard<lmc::SpinMutex> lock(mutex_);
        T item = queue_.front();
        queue_.pop();
        return item;
    }

    bool empty() {
        std::lock_guard<lmc::SpinMutex> lock(mutex_);
        return queue_.empty();
    }

    int size() {
        std::lock_guard<lmc::SpinMutex> lock(mutex_);
        return queue_.size();
    }

private:
    std::queue<T> queue_;
    lmc::SpinMutex mutex_;
};

// 复刻 spdlog rotating_file_sink 核心轮转逻辑，无spdlog依赖
class rotating_file_sink
{
public:
    /**
     * @param base_filename 基础文件名，不带后缀
     * @param max_file_size 单文件最大字节
     * @param max_files 最大归档文件数量
     * @param truncate 新建文件是否清空原有内容，默认false追加
     */
    rotating_file_sink(std::string base_filename, size_t max_file_size, size_t max_files)
        : base_filename_(std::move(base_filename)),
          max_size_(max_file_size),
          max_files_(max_files) {
        is_thread_running = true;
        write_thread_ = std::thread(&rotating_file_sink::write_thread_func, this);
        open_file();
        // 启动时清理超出数量的老旧归档文件
        cleanup_excess_files();
    }

    ~rotating_file_sink() {
        is_thread_running = false;
        if (write_thread_.joinable()) {
            write_thread_.join();
        }
        lf_ptr_.reset();
    }

    void write_thread_func() {
        uint64_t eventnum = 0;
        while (is_thread_running) {
            usleep(10000);
            if (g_message_queue.empty())
                continue;

            int queue_size = g_message_queue.size();
            for (size_t i =0; i < queue_size; i++) {
                LcmMessage msg = g_message_queue.pop();

                lcm::LogEvent le;
                le.channel = msg.channel;
                le.datalen = msg.data.size();
                le.data = msg.data.data();
                le.eventnum = eventnum++;
                le.timestamp = msg.timestamp;
                lf_ptr_->writeEvent(&le);
                if (get_file_size_by_lcmptr(lf_ptr_) > max_size_) {
                    rotate();
                }
            }
        }
    }

    // 写入lcmlog数据
    void write(const std::string& channel, const lcm::ReceiveBuffer* rbuf) {
        LcmMessage msg;
        msg.channel = channel;
        msg.timestamp = rbuf->recv_utime;
        msg.data.assign(reinterpret_cast<const uint8_t*>(rbuf->data), reinterpret_cast<const uint8_t*>(rbuf->data) + rbuf->data_size);
        g_message_queue.push(msg);
    }

private:
    std::thread write_thread_;
    SafeQueue<LcmMessage> g_message_queue;

    std::shared_ptr<lcm::LogFile> lf_ptr_;

    bool is_thread_running = false;

    std::string base_filename_;
    size_t max_size_;
    size_t max_files_;

    size_t current_size_ = 0;

    // 拼接带序号文件名: name.1.txt / name.txt
    std::string calc_filename(size_t idx) const {
        std::ostringstream oss;
        oss << base_filename_;
        if (idx > 0) {
            oss << "." << idx;
        }
        oss << ".lcmlog";
        return oss.str();
    }

    // 打开当前主日志文件
    void open_file() {
        std::string fname = calc_filename(0);
        lf_ptr_ = std::make_shared<lcm::LogFile>(fname, "a");
        current_size_ = get_file_size_by_lcmptr(lf_ptr_);
    }

    // 根据lcm描述符获取文件大小
    static size_t get_file_size_by_lcmptr(std::shared_ptr<lcm::LogFile> lf_) {
        return ftell(lf_->getFilePtr());
    }

    // 根据文件名获取文件大小
    static size_t get_file_size_by_name(const std::string& path)
    {
#ifdef _WIN32
        WIN32_FILE_ATTRIBUTE_DATA fad;
        if (!GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &fad))
            return 0;
        LARGE_INTEGER size;
        size.HighPart = fad.nFileSizeHigh;
        size.LowPart = fad.nFileSizeLow;
        return static_cast<size_t>(size.QuadPart);
#else
        struct stat stat_buf;
        int rc = stat(path.c_str(), &stat_buf);
        return rc == 0 ? static_cast<size_t>(stat_buf.st_size) : 0;
#endif
    }

    // 删除单个文件
    static bool remove_file(const std::string& path)
    {
#ifdef _WIN32
        return DeleteFileA(path.c_str()) != 0;
#else
        return unlink(path.c_str()) == 0;
#endif
    }

    // 文件轮转，完全复刻spdlog逻辑
    void rotate() {
        lf_ptr_.reset();

        // 从后往前重命名，避免覆盖
        for (int i = static_cast<int>(max_files_) - 1; i >= 1; --i) {
            std::string src = calc_filename(i);
            std::string dst = calc_filename(i + 1);
            if (get_file_size_by_name(src) > 0) {
                remove_file(dst);
                rename(src.c_str(), dst.c_str());
            }
        }

        // 主文件改名 .1
        std::string src = calc_filename(0);
        std::string dst = calc_filename(1);
        remove_file(dst);
        rename(src.c_str(), dst.c_str());

        open_file();
    }

    // 初始化删除超出max_files的老旧文件
    void cleanup_excess_files() {
        for (size_t i = max_files_ + 1;; ++i) {
            std::string f = calc_filename(i);
            if (get_file_size_by_name(f) == 0)
                break;
            remove_file(f);
        }
    }
};

#endif

