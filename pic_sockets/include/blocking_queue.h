/*
 * Copyright (C) 2024 Coder.AN
 * Email: an.hongjun@foxmail.com
 * Page: www.anhongjun.top
 */
#ifndef BLOCKING_QUEUE_H
#define BLOCKING_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>

template<typename T>
class BlockingQueue
{
private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable not_full_cond_;
    std::condition_variable not_empty_cond_;
    size_t capacity_;

public:
    explicit BlockingQueue(size_t capacity) : capacity_(capacity) {}

    void push(const T& item)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        while (queue_.size() >= capacity_)
        {
            not_full_cond_.wait(lock);
        }

        queue_.push(item);
        not_empty_cond_.notify_all();
    }

    T pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);

        while (queue_.empty())
        {
            not_empty_cond_.wait(lock);
        }

        T item = queue_.front();
        queue_.pop();
        not_full_cond_.notify_all();
        return item;
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
};

#endif // BLOCKING_QUEUE_H

