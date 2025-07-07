//
// Created by nova on 7/6/25.
//

#ifndef TELEOP_INPUTSOURCEHANDLE_HPP
#define TELEOP_INPUTSOURCEHANDLE_HPP

#include <memory>
#include <vector>
#include "InputSource.hpp"

namespace teleop::internal
{

/// For each InputSource we store, we associate some extra metadata.
class InputSourceHandle
{
  using ParameterInterface = rclcpp::node_interfaces::NodeParametersInterface;

public:
  explicit InputSourceHandle(const ParameterInterface::SharedPtr& parameters, InputManager& inputs,
                             const std::shared_ptr<InputSource>& source);

  explicit InputSourceHandle(InputManager& inputs, const std::shared_ptr<InputSource>& source);

  void update(const rclcpp::Time& now) const;
  void declare_and_link_inputs();

private:
  struct RemapButtonParams
  {
    std::string name;
    std::string from;
  };
  struct RemapAxisParams
  {
    std::string name;
    std::string from;
  };
  struct RemapParams
  {
    std::vector<RemapButtonParams> buttons;
    std::vector<RemapAxisParams> axes;
  };
  template <typename T>
  struct InputRemappings
  {
    std::vector<std::string> remapped_names;
    std::vector<std::reference_wrapper<T>> references;
  };

  struct Remapping
  {
    InputRemappings<uint8_t> buttons;
    InputRemappings<float> axes;
  };

  template <typename T, typename InputT>
  struct Definition
  {
    static_assert(std::is_base_of_v<InputCommon<T>, InputT>,
                  "InputT must be an input type inheriting from InputCommon (Button, Axis).");

    /// We hold a shared pointer to keep any exported input alive
    std::shared_ptr<InputT> input;
    std::reference_wrapper<T> reference;

    Definition(const std::shared_ptr<InputT> input, const std::reference_wrapper<T>& reference)
      : input(input), reference(reference)
    {
    }
  };

  Remapping remap(InputSource::InputDeclarationSpans declarations, RemapParams remap_params);
  RemapParams get_remap_params();
  std::optional<RemapButtonParams> get_remap_button_params(const std::string& name);
  std::optional<RemapAxisParams> get_remap_axis_params(const std::string& name);

  std::vector<Definition<uint8_t, Button>> button_definitions_;
  std::vector<Definition<float, Axis>> axis_definitions_;

  std::shared_ptr<InputSource> source_;
  std::reference_wrapper<InputManager> inputs_;
  ParameterInterface::SharedPtr parameters_;
};

}  // namespace teleop::internal

#endif  // TELEOP_INPUTSOURCEHANDLE_HPP
