//
// Created by nova on 7/6/25.
//

#ifndef TELEOP_INPUTDECLARATIONLIST_HPP
#define TELEOP_INPUTDECLARATIONLIST_HPP

#include "VectorRef.hpp"

namespace teleop
{

/**
 * A class to be given to InputSource implementation to declare their buttons and axes, in a way that allows button
 * values to be placed next to each other
 */
template <typename T>
class InputDeclarationList
{
public:
  InputDeclarationList(std::vector<std::string>& names, std::vector<T>& values) : names_(names), values_(values)
  {
  }

  /**
   * @brief Declares a button/axis to exist with the given name, at an index equal to the number of declarations before
   * calling this method.
   * @param[in] name    The name of the button/axis being declared.
   * @returns   A reference to the value of the declared button/axis. Assign the value stored in this VectorRef with
   * `operator=` during on_update() to change the value of the declared input. Alternatively, you can set the value
   * directly in your InputSource by assigning `this->buttons_[i]` or `this->axes_[i]` to avoid a layer of indirection.
   */
  VectorRef<T> add(const std::string& name)
  {
    names_.emplace_back(name);
  }

  void reserve(size_t n)
  {
    names_.reserve(n);
    values_.reserve(n);
  }

private:
  std::vector<std::string>& names_;
  std::vector<T>& values_;
};

}  // namespace teleop

#endif  // TELEOP_INPUTDECLARATIONLIST_HPP
