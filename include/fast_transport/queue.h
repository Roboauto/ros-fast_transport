#pragma once

#include <queue>
#include <string>

class Queue {
    const std::size_t maxSize_;
    std::queue<std::string> queue_;
    bool full_ = false;
public:
    Queue(std::size_t maxSize) : maxSize_(maxSize) {
    }

    std::string push(std::string&& element) {
        queue_.push(std::move(element));
        if (full_) {
            std::string tmp{queue_.front()};
            queue_.pop();
            return tmp;
        }
        full_ = queue_.size() == maxSize_;
        return "";
    }

    std::string push(const std::string& element) {
        queue_.push(element);
        if (full_) {
            std::string tmp{queue_.front()};
            queue_.pop();
            return tmp;
        }
        full_ = queue_.size() == maxSize_;
        return "";
    }
};