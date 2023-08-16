#pragma once

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

    _data = (T *)malloc(sizeof(T) * n);
    _next = _data;
    _top  = _data + n;

    _recycled.reserve(n / 3);
  }
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

  inline void remove(T *ptr) { _recycled.push_back(ptr - _data); }

  inline unsigned int size() const { return _next - _data - _recycled.size(); }

  inline unsigned int capacity() const { return _top - _data; }

  inline void reserve(unsigned int n)
  {
    if (capacity() > n)
      return;
    auto oldNext = _next - _data;
    _data        = (T *)realloc(_data, sizeof(T) * n);
    _next        = oldNext + _data;
    _top         = _data + n;
  }

  inline unsigned int getID(T *ptr) const { return ptr - _data; }

  inline void clear()
  {
    _next = _data;
    _recycled.clear();
  }

  std::vector<unsigned int> _recycled;

  T *_data, *_next, *_top;
};