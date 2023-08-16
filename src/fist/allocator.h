#pragma once

#include <cassert>
#include <cstdlib>
#include <queue>

template <class T, bool CheckExtension = false>
class RecyclableAllocator
{
public:
  inline RecyclableAllocator(unsigned int n = 0)
  {
    if (!n)
      return;

    _data = (T *)std::malloc(sizeof(T) * n);
    _next = _data;
    _top  = _data + n;

    _recycled.reserve(n / 3);
  }
  RecyclableAllocator(RecyclableAllocator &&target) = delete;
  RecyclableAllocator &operator=(RecyclableAllocator &&target)
  {
    _data     = target._data;
    _next     = target._next;
    _top      = target._top;
    _recycled = std::move(target._recycled);

    target._data = target._next = target._top = nullptr;
    return *this;
  }
  RecyclableAllocator &operator=(const RecyclableAllocator &) = delete;
  inline ~RecyclableAllocator() { free(_data); }

  template <class... Args>
  inline T *alloc(Args... args)
  {
    if (_recycled.empty())
    {
      T *res = new (_next) T(args...);
      _next++;
      return res;
    }
    unsigned int loc = _recycled.back();
    T *res           = new (_data + loc) T(args...);
    _recycled.pop_back();
    return res;
  }

  inline void remove(T *ptr) { _recycled.push_back(static_cast<unsigned int>(ptr - _data)); }

  inline unsigned int size() const { return static_cast<unsigned int>(_next - _data - _recycled.size()); }

  inline unsigned int capacity() const { return static_cast<unsigned int>(_top - _data); }

  inline void reserve(unsigned int n)
  {
    if (capacity() > n)
      return;
    auto oldNext     = _next - _data;
    auto reallocated = (T *)std::realloc(_data, sizeof(T) * n);
    if (reallocated)
    {
      _data = reallocated;
      _next = oldNext + _data;
      _top  = _data + n;
    }
    else
    {
      assert(false);
    }
  }

  inline unsigned int getID(T *ptr) const { return ptr - _data; }

  inline void clear()
  {
    _next = _data;
    _recycled.clear();
  }

  std::vector<unsigned int> _recycled;

  T *_data = nullptr, *_next = nullptr, *_top = nullptr;
};