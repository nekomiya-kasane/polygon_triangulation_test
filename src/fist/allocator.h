#pragma once

#include <cstdlib>
#include <queue>

template <class T, bool CheckExtension = false>
class RecyclableAllocator
{
  inline RecyclableAllocator(unsigned int n)
  {
    _data = (T *)malloc(sizeof(T) * n);
    _next = _data;
    _top  = _data + n;
  }

  template <class... Args>
  inline T *alloc(Args... args)
  {
    if (_recycled.empty())
    {
      T *res = new (_next) T(args);
      _next++;
      return res;
    }
    unsigned int loc = _recycled.back();
    T *res           = new (_data + loc) T(args);
    _recycled.pop_back();
    return res;
  }

  inline void remove(T *ptr) { _recycled.push_back(ptr - _data); }

  inline unsigned int size() const { return _next - _data - _recycled.size(); }

  inline unsigned int capacity() const { return _top - _data; }

  inline unsigned int getID(T *ptr) const { return _ptr - _data; }

  std::vector<unsigned int> _recycled;

  T *_data, _next, _top;
};