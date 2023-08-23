#ifndef THREAD_POOL_H
#define THREAD_POOL_H

// mark extra functionality wrt orig code
#define TP_EXT

#ifdef TP_EXT
#  include <map>
#endif
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <vector>

#ifdef TP_EXT
struct ThreadData
{
  ThreadData() : tmpCounter(0), traceIndent(0) {}
  unsigned long long tmpCounter;
  unsigned long long traceIndent;
};
#endif

// std::result_of is deprecated since C++17 and we should use std::invoke_result instead of it
#if true  //__cplusplus >= 201704L
#  include <type_traits>
#  define TP_RESULT_OF_INVOKE(Func, Args) std::invoke_result_t<Func, Args...>
#else
#  define TP_RESULT_OF_INVOKE(Func, Args) typename std::result_of<Func(Args...)>::type
#endif

class ThreadPool
{
public:
  ThreadPool(size_t);
  template <class F, class... Args>
  auto enqueue(F &&f, Args &&...args) -> std::future<TP_RESULT_OF_INVOKE(F, Args)>;
  ~ThreadPool();
#ifdef TP_EXT
  size_t getNumThreads() const;
  bool isWorkerThread() const;
  // size_t getThreadIndex() const;
  // ThreadDatas getThreadData();
#endif
private:
  // need to keep track of threads so we can join them
  std ::vector<std::thread> workers;
  // the task queue
  std::queue<std::function<void()> > tasks;

  // synchronization
  std::mutex queue_mutex;
  std::condition_variable condition;
  bool stop;
#ifdef TP_EXT
  std::map<std::thread::id, size_t> ids;
  ThreadData *data;
#endif
};

// the constructor just launches some amount of workers
inline ThreadPool::ThreadPool(size_t threads) : stop(false)
{
#ifdef TP_EXT
  data                            = new ThreadData[threads + 1];
  ids[std::this_thread::get_id()] = 0;
#endif
  for (size_t i = 0; i < threads; ++i)
  {
    workers.emplace_back([this] {
      for (;;)
      {
        std::function<void()> task;

        {
          std::unique_lock<std::mutex> lock(this->queue_mutex);
          this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });
          if (this->stop && this->tasks.empty())
            return;
          task = std::move(this->tasks.front());
          this->tasks.pop();
        }

        task();
      }
    });
#ifdef TP_EXT
    const std::thread &wt = workers.back();
    ids[wt.get_id()]      = i + 1;
#endif
  }
}

#ifdef TP_EXT
inline size_t ThreadPool::getNumThreads() const
{
  return workers.size();
}

inline bool ThreadPool::isWorkerThread() const
{
  auto thisId = std::this_thread::get_id();
  for (const std::thread &worker : workers)
    if (worker.get_id() == thisId)
      return true;
  return false;
}

#  if 0
inline size_t ThreadPool::getThreadIndex() const
{
	return ids.at(std::this_thread::get_id());
}
		
inline ThreadData& ThreadPool::getThreadData()
{
	return data[getThreadIndex()];
}
#  endif
#endif

// add new work item to the pool
template <class F, class... Args>
auto ThreadPool::enqueue(F &&f, Args &&...args) -> std::future<TP_RESULT_OF_INVOKE(F, Args)>
{
  using return_type = TP_RESULT_OF_INVOKE(F, Args);

  auto task = std::make_shared<std::packaged_task<return_type()> >(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

  std::future<return_type> res = task->get_future();
  {
    std::unique_lock<std::mutex> lock(queue_mutex);

    // don't allow enqueueing after stopping the pool
    if (stop)
      throw std::runtime_error("enqueue on stopped ThreadPool");

    tasks.emplace([task]() { (*task)(); });
  }
  condition.notify_one();
  return res;
}

// the destructor joins all threads
inline ThreadPool ::~ThreadPool()
{
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    stop = true;
  }
  condition.notify_all();
  for (std::thread &worker : workers)
    worker.join();
#ifdef TP_EXT
  delete[] data;
#endif
}

#endif
