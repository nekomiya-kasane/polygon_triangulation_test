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

  inline void Reserve(size_t count)
  {
    size_t size = Size();
    if (size >= count)
      return;
    _data = (T *)realloc(_data, sizeof(T) * count);
    _top  = _data + count;
    _next = _data + size;
  }
  inline void Extend(size_t count) { Reserve(count + Size()); }
  inline void Sqeeze()
  {
    auto size = Size();
    _data     = (T *)realloc(_data, sizeof(T) * size);
    _top = _next = _data + size;
  }

  inline unsigned int Size() const { return _next - _data; }
  inline unsigned int Capability() const { return _top - _data; }

  inline unsigned int Pushback(const T *begin, size_t size)
  {
    if (Capability() < Size() + size)
      Reserve(size + Size());
    memcpy(_next, begin, size * sizeof(T));
    _next += size;
  }
  inline unsigned int Pushback(const T &element)
  {
    if (Capability() <= Size())
      Reserve(1 + Size());
    memcpy(_next, &element, sizeof(T));
    ++_next;
  }

  inline T &operator[](size_t index) { return *(_data + index); }
  inline const T &operator[](size_t index) const { return *(_data + index); }

public:
  T *_data = nullptr, *_next = nullptr, *_top = nullptr;
};