#include "robot_communication/utils/thread_pool.hpp"
#include <algorithm>

#ifdef __linux__
#include <pthread.h>
#include <unistd.h>
#endif

namespace robot_communication {

ThreadPool::ThreadPool(size_t num_threads) {
    if (num_threads == 0) {
        num_threads = std::max(1u, std::thread::hardware_concurrency());
    }
    
    workers_.reserve(num_threads);
    
    for (size_t i = 0; i < num_threads; ++i) {
        workers_.emplace_back(&ThreadPool::workerLoop, this);
        setThreadAffinity(workers_.back(), i);
    }
}

ThreadPool::~ThreadPool() {
    stop_.store(true);
    condition_.notify_all();
    
    for (auto& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void ThreadPool::workerLoop() {
    while (true) {
        std::function<void()> task;
        
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            
            condition_.wait(lock, [this] { 
                return stop_.load() || !tasks_.empty(); 
            });
            
            if (stop_.load() && tasks_.empty()) {
                break;
            }
            
            task = std::move(tasks_.front());
            tasks_.pop();
        }
        
        task();
    }
}

void ThreadPool::setThreadAffinity(std::thread& thread, size_t thread_id) {
#ifdef __linux__
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    
    // Simple round-robin CPU affinity
    size_t cpu_count = std::thread::hardware_concurrency();
    if (cpu_count > 0) {
        CPU_SET(thread_id % cpu_count, &cpuset);
        
        int result = pthread_setaffinity_np(thread.native_handle(), 
                                          sizeof(cpu_set_t), &cpuset);
        if (result != 0) {
            // Silently ignore affinity errors - not critical
        }
    }
#else
    // Thread affinity not supported on non-Linux platforms
    (void)thread;
    (void)thread_id;
#endif
}

} // namespace robot_communication