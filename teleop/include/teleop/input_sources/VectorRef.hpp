//
// Created by nova on 7/6/25.
//

#ifndef TELEOP_VECTORREF_HPP
#define TELEOP_VECTORREF_HPP

namespace teleop
{

/**
 * A pointer-like type that provides a reference to an item in a vector, remaining valid across vector resizes. Does not
 * track rearrangement of vector elements.
 *
 * This was added to allow button and axis values from input sources to be placed together in the same vector, without
 * requiring implementers to index directly into the values array during on_update(), by holding onto a VectorRef in
 * export_buttons() or export_axes().
 */
template <typename T>
class VectorRef
{
public:
  inline VectorRef(std::vector<T>& container, std::size_t index) noexcept : container_(&container), index_(index)
  {
  }

  [[nodiscard]] inline T& get() const noexcept
  {
    assert(index_ < container_->size());
    return (*container_)[index_];
  }

  inline operator T&() const noexcept
  {
    return get();
  }

  inline T* operator->() const noexcept
  {
    return &get();
  }

  inline T& operator*() const noexcept
  {
    return get();
  }

  // Delete assignment from other wrappers
  VectorRef& operator=(const VectorRef&) = delete;
  VectorRef& operator=(VectorRef&&) = delete;

  // Allow assignment to modify the held value
  inline VectorRef& operator=(const T& value) noexcept
  {
    get() = value;
    return *this;
  }
  inline VectorRef& operator=(T&& value) noexcept
  {
    get() = std::move(value);
    return *this;
  }

  // Allow the index to be accessed
  inline std::size_t index() const noexcept
  {
    return index_;
  }

private:
  std::vector<T>* container_;
  std::size_t index_;
};

}  // namespace teleop

#endif  // TELEOP_VECTORREF_HPP
