//
// Created by nova on 7/6/25.
//

#ifndef TELEOP_MODULAR_INPUT_SOURCE_SPAN_HPP
#define TELEOP_MODULAR_INPUT_SOURCE_SPAN_HPP

#include <vector>
#include <cstddef>

namespace input_source
{

/**
 * Substitute for std::span to support more C++ versions
 */
template <typename T>
struct span
{
  using iterator = T*;

  T* data_;
  std::size_t size_;

  constexpr span() noexcept : data_(nullptr), size_(0)
  {
  }
  constexpr span(T* data, std::size_t size) noexcept : data_(data), size_(size)
  {
  }
  constexpr explicit span(std::vector<T>& vector) noexcept : data_(vector.data()), size_(vector.size())
  {
  }
  constexpr explicit span(const std::vector<T>& vector) noexcept : data_(vector.data()), size_(vector.size())
  {
  }

  [[nodiscard]] constexpr std::size_t size() const noexcept
  {
    return size_;
  }
  [[nodiscard]] constexpr bool empty() const noexcept
  {
    return size_ == 0;
  }

  constexpr T& operator[](std::size_t idx) const noexcept
  {
    assert(idx < size_);
    return data_[idx];
  }

  constexpr T& front() const noexcept
  {
    assert(size_ > 0);
    return data_[0];
  }

  constexpr T& back() const noexcept
  {
    assert(size_ > 0);
    return data_[size_ - 1];
  }

  constexpr iterator begin() const noexcept
  {
    return data_;
  }
  constexpr iterator end() const noexcept
  {
    return data_ + size_;
  }
};

}  // namespace input_source

#endif  // TELEOP_MODULAR_INPUT_SOURCE_SPAN_HPP
