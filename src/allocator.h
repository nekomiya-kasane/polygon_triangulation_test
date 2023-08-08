#pragma once

#ifndef SEIDEL_ALLOCATOR_H
#  define SEIDEL_ALLOCATOR_H

#  include <cassert>
#  include <cstdlib>
#  include <cstring>

template <class T, class SizeType = unsigned int>
class Allocator
{
public:
  ~Allocator() { free(_data); }

public:
  using size_type = SizeType;

  class AllocatorIterator
  {
  public:
    AllocatorIterator(T *ptr) : ptr(ptr) {}
    AllocatorIterator operator++()
    {
      ++ptr;
      return *this;
    }
    bool operator!=(const AllocatorIterator &other) const { return ptr != other.ptr; }
    const T &operator*() const { return *ptr; }

  private:
    T *ptr;
  };

  template <class... Args>
  T &New(Args... args)
  {
    // todo: expand memory
    T *res = new (_next) T(args...);
    _next++;
    return *res;
  }

  template <class... Args>
  size_type New2(Args... args)
  {
    // todo: expand memory
    new (_next) T(args);
    _next++;
    return Size() - 1;
  }

  inline void Reserve(size_t count)
  {
    size_t size = Size();
    if (size >= count)
      return;
    _data = (T *)std::realloc(_data, sizeof(T) * count);
    _top  = _data + count;
    _next = _data + size;
  }
  inline void ResizeRaw(size_t count)
  {
    Reserve(count);
    _next = _data + count;
  }
  inline void Extend(size_t count) { Reserve(count + Size()); }
  inline void Sqeeze()
  {
    auto size = Size();
    _data     = (T *)std::realloc(_data, sizeof(T) * size);
    _top = _next = _data + size;
  }

  inline size_type Size() const { return static_cast<size_type>(_next - _data); }
  inline size_type Capability() const { return static_cast<size_type>(_top - _data); }

  inline T *Back() const
  {
    assert(Size() > 0);
    return _next - 1;
  }

  inline size_type Pushback(const T *begin, size_t size)
  {
    if (Capability() < Size() + size)
      Reserve(size + Size());
    std::memcpy(_next, begin, size * sizeof(T));
    _next += size;
    return Size() - 1;
  }
  inline size_type Pushback(const T &element)
  {
    if (Capability() <= Size())
      Reserve(_expansion + Size());  // todo: improve this
    std::memcpy(_next, &element, sizeof(T));
    ++_next;
    return Size() - 1;
  }

  inline void Reset() { _next = _data; }

  inline size_type GetIndex(const T *const elementPtr) const
  {
    assert((elementPtr < _top) && (elementPtr >= _data));
    assert(((char *)elementPtr - (char *)_data) % sizeof(T) == 0);  // integrity
    return static_cast<size_type>(elementPtr - _data);
  }

  inline T &operator[](size_t index) { return *(_data + index); }
  inline const T &operator[](size_t index) const { return *(_data + index); }

  inline AllocatorIterator begin() const { return _data; }
  inline AllocatorIterator end() const { return _next; }

public:
  T *_data = nullptr, *_next = nullptr, *_top = nullptr;
  unsigned int _expansion = 10;
};

#endif SEIDEL_ALLOCATOR_H