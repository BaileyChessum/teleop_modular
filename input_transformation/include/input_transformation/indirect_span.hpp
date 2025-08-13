// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
//
// Created by Bailey Chessum on 13/8/25.
//

#ifndef INPUT_TRANSFORMATION_INDIRECT_SPAN_HPP
#define INPUT_TRANSFORMATION_INDIRECT_SPAN_HPP
#include <cstddef>
#include <iterator>
#include <type_traits>
#include <utility>
#include <vector>
#include "input_source/utilities/span.hpp"
#include "input_transformation/visibility_control.h"

namespace input_transformation {

using input_source::span;

template <class T>
class INPUT_TRANSFORMATION_PUBLIC indirect_span {
  static_assert(!std::is_pointer_v<T>, "T should be the pointee type (e.g., Foo, const Foo).");
public:
  using element_type   = T;
  using value_type     = std::remove_cv_t<T>;
  using size_type      = std::size_t;
  using difference_type= std::ptrdiff_t;
  using pointer        = T*;              // pointer to pointee
  using reference      = T&;              // reference to pointee
  using ptrptr_type    = std::add_pointer_t<pointer>; // T** (or const T** when T is const)

  // Random-access iterator that dereferences through the inner pointer
  class INPUT_TRANSFORMATION_PUBLIC_TYPE iterator {
  private:
    ptrptr_type p_;
  public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type        = std::remove_cv_t<T>;
    using difference_type   = std::ptrdiff_t;
    using pointer           = std::add_pointer_t<T>;
    using reference         = T&;

    constexpr iterator() noexcept : p_(nullptr) {}
    constexpr explicit iterator(ptrptr_type p) noexcept : p_(p) {}

    // Core access
    constexpr reference operator*()  const noexcept { return **p_; }
    constexpr pointer   operator->() const noexcept { return  *p_; }
    constexpr reference operator[](difference_type n) const noexcept { return *p_[n]; }

    // Navigation
    constexpr iterator& operator++()    noexcept { ++p_; return *this; }
    constexpr iterator  operator++(int) noexcept { iterator t=*this; ++*this; return t; }
    constexpr iterator& operator--()    noexcept { --p_; return *this; }
    constexpr iterator  operator--(int) noexcept { iterator t=*this; --*this; return t; }
    constexpr iterator& operator+=(difference_type n) noexcept { p_ += n; return *this; }
    constexpr iterator& operator-=(difference_type n) noexcept { p_ -= n; return *this; }
    friend constexpr iterator operator+(iterator it, difference_type n) noexcept { it+=n; return it; }
    friend constexpr iterator operator+(difference_type n, iterator it) noexcept { it+=n; return it; }
    friend constexpr iterator operator-(iterator it, difference_type n) noexcept { it-=n; return it; }
    friend constexpr difference_type operator-(iterator a, iterator b) noexcept { return a.p_ - b.p_; }

    // Comparison
    friend constexpr bool operator==(const iterator& a, const iterator& b) noexcept { return a.p_ == b.p_; }
    // friend constexpr auto operator<=>(const iterator& a, const iterator& b) noexcept { return a.p_ <=> b.p_; }

    // Expose underlying pointer-to-pointer if needed
    constexpr ptrptr_type base() const noexcept { return p_; }
  };

  using const_iterator = iterator; // same behavior: yields T& / const T& depending on T

  // Constructors
  constexpr indirect_span() noexcept : first_(nullptr), count_(0) {}

  // From raw T** and size
  constexpr indirect_span(ptrptr_type first, size_type count) noexcept
    : first_(first), count_(count) {}

  // From begin/end of T**
  constexpr indirect_span(ptrptr_type first, ptrptr_type last) noexcept
    : first_(first), count_(static_cast<size_type>(last - first)) {}

  // From std::span<T*>
  constexpr explicit indirect_span(span<pointer> s) noexcept
    : first_(s.data()), count_(s.size()) {}

  // Observers
  [[nodiscard]] constexpr size_type size()  const noexcept { return count_; }
  [[nodiscard]] constexpr bool      empty() const noexcept { return count_ == 0; }

  // Random access to pointees
  constexpr reference operator[](size_type i) const noexcept { return *first_[i]; }
  constexpr reference front()            const noexcept { return **first_; }
  constexpr reference back()             const noexcept { return *first_[count_-1]; }

  // Iteration
  constexpr iterator begin()  const noexcept { return iterator(first_); }
  constexpr iterator end()    const noexcept { return iterator(first_ + count_); }
  constexpr const_iterator cbegin() const noexcept { return begin(); }
  constexpr const_iterator cend()   const noexcept { return end(); }

  // Subviews (like std::span)
  constexpr indirect_span first(size_type n)  const noexcept { return {first_, n}; }
  constexpr indirect_span last(size_type n)   const noexcept { return {first_ + (count_ - n), n}; }
  constexpr indirect_span subspan(size_type off, size_type n = size_type(-1)) const noexcept {
    const auto len = (n == size_type(-1)) ? (count_ - off) : n;
    return {first_ + off, len};
  }

  // Underlying pointer-to-pointer (if the caller needs it)
  constexpr ptrptr_type data() const noexcept { return first_; }

private:
  ptrptr_type first_;
  size_type   count_;
};

} // namespace input_transformation

#endif  // INPUT_TRANSFORMATION_INDIRECT_SPAN_HPP
