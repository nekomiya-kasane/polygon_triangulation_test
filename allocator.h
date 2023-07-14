#pragma once

template <class T>
class Allocator
{
public:
  template <class... Args>
  T &New(Args... args)
  {
    T *res = new (_next) T(args);
    _next++;
    return *res;
  }

  inline unsigned int Size() const { return _next - _data; }

  inline T &operator[](size_t index) { return *(_data + index); }
  inline const T &operator[](size_t index) const { return *(_data + index); }

public:
  T *_data = nullptr, *_next = nullptr, *_top = nullptr;
};