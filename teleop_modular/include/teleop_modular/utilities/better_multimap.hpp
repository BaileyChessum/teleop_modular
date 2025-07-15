//
// Created by nova on 7/7/25.
//

#ifndef BETTER_MULTIMAP_HPP
#define BETTER_MULTIMAP_HPP
#include <map>
#include <vector>

namespace teleop::utils
{
template <typename KeyT, typename ValueT>
class better_multimap
{
public:
  using iterator = typename std::map<KeyT, std::vector<ValueT> >::iterator;
  using const_iterator = typename std::map<KeyT, std::vector<ValueT> >::const_iterator;

  std::vector<ValueT>& operator[](const KeyT& key)
  {
    return items_[key];
  }

  iterator find(const KeyT& key)
  {
    return items_.find(key);
  }

  void insert(const KeyT& key, ValueT value)
  {
    auto it = items_.find(key);

    if (it != items_.end())
    {
      it->second.emplace_back(value);
      return;
    }

    items_.insert({ key, { value } });
  }

  constexpr iterator begin() noexcept
  {
    return items_.begin();
  }

  constexpr iterator end() noexcept
  {
    return items_.end();
  }

  constexpr const_iterator begin() const noexcept
  {
    return items_.begin();
  }

  constexpr const_iterator end() const noexcept
  {
    return items_.end();
  }

  [[nodiscard]] constexpr std::size_t size() const noexcept
  {
    return items_.size();
  }

private:
  std::map<KeyT, std::vector<ValueT> > items_{};
};
}  // namespace teleop::utils

#endif  // BETTER_MULTIMAP_HPP
