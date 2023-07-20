#pragma once

template <class T, class SizeType = unsigned int>
class Allocator
{
  ~Allocator() { free(_data); }

public:
  using size_type = SizeType;

  template <class... Args>
  T &New(Args... args)
  {
    T *res = new (_next) T(args);
    _next++;
    return *res;
  }

  template <class... Args>
  unsigned int New2(Args... args)
  {
    new (_next) T(args);
    _next++;
    return Size() - 1;
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

  inline size_type Size() const { return _next - _data; }
  inline size_type Capability() const { return _top - _data; }

  inline size_type Pushback(const T *begin, size_t size)
  {
    if (Capability() < Size() + size)
      Reserve(size + Size());
    memcpy(_next, begin, size * sizeof(T));
    _next += size;
  }
  inline size_type Pushback(const T &element)
  {
    if (Capability() <= Size())
      Reserve(1 + Size());
    memcpy(_next, &element, sizeof(T));
    ++_next;
  }

  inline size_type GetIndex(T *elementPtr) const
  {
    assert((elementPtr < _top) && (elementPtr >= _data));
    assert(((char *)elementPtr - (char *)_data) % sizeof(T))
  }

  inline T &operator[](size_t index) { return *(_data + index); }
  inline const T &operator[](size_t index) const { return *(_data + index); }

  inline T *begin() { return _data; }
  inline const T *cbegin() const { return _data; }
  inline T *end() { return _next; }
  inline const T *cend() const { return _next; }

public:
  T *_data = nullptr, *_next = nullptr, *_top = nullptr;
};