//
// Created by nova on 9/13/25.
//

#ifndef CONTROLLERORDERING_HPP
#define CONTROLLERORDERING_HPP

#include <vector>
#include <set>
#include <string>
#include <map>
#include <math.h>
#include <optional>

namespace teleop {

class ControllerOrdering {
public:
  /**
   * Add an ordered set of controller dependencies
   * \param controllers controller names in the order they need to be activated
   */
  void add(const std::vector<std::string>& controllers)
  {
    if (controllers.size() == 0)
      return;

    auto previous_id = find_or_create_handle(controllers[0]);

    // Make handles for each controller depend on the previous controller in controllers
    for (size_t i = 1; i < controllers.size(); ++i)
    {
      const auto id = find_or_create_handle(controllers[i]);
      handles_[id].dependencies.insert(previous_id);
      handles_[previous_id].children.insert(id);

      previous_id = id;
    }
  }

  const std::vector<size_t>& get_ordering()
  {
    if (!ordering_.has_value())
      order();

    return ordering_.value();
  }

private:
  struct Handle
  {
    std::string name;
    std::set<size_t> dependencies{};
    std::set<size_t> children{};
  };

  size_t find_or_create_handle(const std::string& controller)
  {
    const auto it = name_to_id_.find(controller);
    if (it != name_to_id_.end())
      return it->second;

    const auto new_id = handles_.size();
    name_to_id_[controller] = new_id;
    handles_.emplace_back(Handle{controller, {}, {}});
    return new_id;
  }

  /**
   * Create value for ordering_
   */
  void order()
  {
    std::vector<size_t> in_degrees{};   //< stores for each handle the number of dependencies of the handle
    in_degrees.reserve(handles_.size());

    std::vector<size_t> ordering{};
    ordering.reserve(handles_.size());

    size_t root_count = 0;

    // Get the in_degree for every handle, and keep track of any root handles
    for (size_t i = 0; i < handles_.size(); ++i)
    {
      in_degrees.emplace_back(handles_[i].dependencies.size());
      if (in_degrees[i] == 0)
      {
        // This element is a root, so we can safely add it to the ordering
        ordering.emplace_back(i);
        ++root_count;
      }
    }

    // Iterate over every element in the ordering to try add its children to the ordering
    for (size_t i = 0; i < ordering.size(); ++i)
    {
      const auto id = ordering[i];
      const auto& children = handles_[id].children;

      for (const auto& child : children)
      {
        // decrement the in_degree
        const auto child_in_degree = --in_degrees[child];

        //
        if (child_in_degree == 0)
          ordering.emplace_back(child);
      }

      if (ordering.size() != handles_.size())
      {
        // Uh oh! The graph is actually not acyclic!
      }




    }



  }

  std::vector<Handle> handles_{};
  std::map<std::string, size_t> name_to_id_{};

  std::optional<std::vector<size_t>> ordering_{};



};

} // teleop

#endif //CONTROLLERORDERING_HPP
