//
// Created by nova on 7/25/25.
//

#ifndef CONTROL_MODE_REMAPPER_REDUCER_HPP
#define CONTROL_MODE_REMAPPER_REDUCER_HPP

#include <vector>
#include <cstdint>
#include <memory>

namespace teleop::internal::remapping {

template<typename T>
struct Transformer
{
  virtual void accumulate(T & result) noexcept = 0;
};

/**
 * Either something that reduces multiple original inputs into a single result, or a transform
 */
template<typename Type, typename From>
struct Reducer : Transformer<Type>
{
  std::vector<From *> values;

  [[nodiscard]] bool empty() const noexcept
  {
    return values.empty();
  }

  [[nodiscard]] size_t size() const noexcept
  {
    return values.size();
  }
};

template<typename T>
struct DirectReducer final : public Reducer<T, T>
{
  std::vector<T *> values;

  inline void accumulate(uint8_t & result) noexcept final
  {
    for (size_t i = 0; i < values.size(); ++i) {
      result += *values[i];
    }
  }

  /// Special case for DirectReducers to allow direct mapping
  [[nodiscard]] T * first() const noexcept
  {
    assert(this->size() == 1);
    return values[0];
  }
};


/// What we actually store in memory for transformed things
template<typename T>
struct TransformedValue final
{
  T value;
  const std::vector<std::shared_ptr<Transformer<T>>> transformers;

  inline void update() noexcept
  {
    value = 0;
    for (const auto & transformer : transformers) {
      transformer->accumulate(value);
    }
  }

  explicit TransformedValue(std::vector<std::shared_ptr<Transformer<T>>> transformers)
      : transformers(std::move(transformers))
  {
    value = 0;
  }
};

}  // namespace teleop::internal::remapper

#endif  // CONTROL_MODE_REMAPPER_REDUCER_HPP
