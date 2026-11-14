#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <stdexcept>

namespace tools
{
// 固定容量的线程安全队列：PopWhenFull=true 时队满丢弃最旧元素（用于只保最新图像帧）；
// 注意 front() 等待非空且不响应 close()，退出同步应使用 pop() / try_pop_for()。
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
  }

  void push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (closed_) return;

    if (queue_.size() >= max_size_) {
      if (PopWhenFull) {
        queue_.pop();
      } else {
        full_handler_();
        return;
      }
    }

    queue_.push(value);
    not_empty_condition_.notify_all();
  }

  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return closed_ || !queue_.empty(); });

    if (queue_.empty()) {
      throw std::runtime_error("Queue closed");
    }

    value = queue_.front();
    queue_.pop();
  }

  template <typename Rep, typename Period>
  bool try_pop_for(T & value, const std::chrono::duration<Rep, Period> & timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!not_empty_condition_.wait_for(lock, timeout, [this] { return closed_ || !queue_.empty(); }) || queue_.empty())
      return false;
    value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  T pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return closed_ || !queue_.empty(); });
    if (queue_.empty()) throw std::runtime_error("Queue closed");

    T value = std::move(queue_.front());
    queue_.pop();
    return std::move(value);
  }

  T front()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    return queue_.front();
  }

  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();
  }

  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
    not_empty_condition_.notify_all();
  }

  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    not_empty_condition_.notify_all();
  }

private:
  bool closed_ = false;
  std::queue<T> queue_;
  size_t max_size_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP
