//
// Created by nova on 7/6/25.
//

#ifndef TELEOP_INPUTSOURCEHANDLE_HPP
#define TELEOP_INPUTSOURCEHANDLE_HPP

#include <memory>
#include <vector>
#include "InputSource.hpp"
#include "teleop/utilities/better_multimap.hpp"

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

  void update(const rclcpp::Time& now);

  void add_definitions_to_inputs() const;

  void declare_and_link_inputs();

private:
  struct ButtonTransformParams
  {
    bool invert = false;
  };
  struct AxisTransformParams
  {
    bool invert = false;
  };

  struct ButtonFromAxisParams
  {
    std::string name;
    float threshold = 0.0;
  };
  struct AxisFromButtonsParams
  {
    std::optional<std::string> positive;
    std::optional<std::string> negative;
  };

  template <typename transformT, typename fromOtherParamsT>
  struct RemapRenameParams
  {
    std::string name;
    std::optional<std::string> from;
    std::optional<transformT> transform;
    std::optional<fromOtherParamsT> from_other;
  };

  using RemapButtonParams = RemapRenameParams<ButtonTransformParams, ButtonFromAxisParams>;
  using RemapAxisParams = RemapRenameParams<AxisTransformParams, AxisFromButtonsParams>;

  struct RemapParams
  {
    std::vector<RemapButtonParams> buttons;
    std::vector<RemapAxisParams> axes;
  };

  /// The thing that actually holds the result of remap transformations in memory
  template <typename T, typename fromOtherT, typename transformT>
  struct TransformedRemapValue
  {
    T value;
    std::optional<std::reference_wrapper<T>> from;
    std::optional<fromOtherT> from_other;
    std::optional<transformT> transform;

    TransformedRemapValue(T value, std::optional<std::reference_wrapper<T>> from, std::optional<fromOtherT> from_other)
      : value(value), from(from), from_other(from_other)
    {
    }

    inline void update(const rclcpp::Time& now);
  };

  struct TransformedRemapButtonFromAxis
  {
    std::reference_wrapper<float> axis;
    float threshold = 0.0f;
  };
  struct TransformedRemapAxisFromButtons
  {
    std::optional<std::reference_wrapper<uint8_t>> negative;
    std::optional<std::reference_wrapper<uint8_t>> positive;
  };

  using TransformedRemapButton = TransformedRemapValue<uint8_t, TransformedRemapButtonFromAxis, ButtonTransformParams>;
  using TransformedRemapAxis = TransformedRemapValue<float, TransformedRemapAxisFromButtons, AxisTransformParams>;

  /// This is an actual connection of an input to a defining value
  template <typename T, typename InputT>
  struct Definition
  {
    static_assert(std::is_base_of_v<InputCommon<T>, InputT>,
                  "InputT must be an input type inheriting from InputCommon (Button, Axis).");

    /// We hold a shared pointer to keep any exported input alive
    std::shared_ptr<InputT> input;
    std::vector<std::reference_wrapper<T>> references;

    Definition(const std::shared_ptr<InputT> input, const std::vector<std::reference_wrapper<T>>& references)
      : input(input), references(references)
    {
    }
  };

  void remap(InputSource::InputDeclarationSpans declarations, RemapParams remap_params);
  RemapParams get_remap_params();

  std::optional<RemapButtonParams> get_remap_button_params(const std::string& name);
  std::optional<ButtonTransformParams> get_button_transform_params(const std::string& name);

  std::optional<RemapAxisParams> get_remap_axis_params(const std::string& name);
  std::optional<AxisTransformParams> get_axis_transform_params(const std::string& name);

  std::vector<Definition<uint8_t, Button>> button_definitions_;
  std::vector<Definition<float, Axis>> axis_definitions_;

  std::vector<TransformedRemapButton> transformed_buttons_;
  std::vector<TransformedRemapAxis> transformed_axes_;

  std::shared_ptr<InputSource> source_;
  std::reference_wrapper<InputManager> inputs_;
  ParameterInterface::SharedPtr parameters_;
};

}  // namespace teleop::internal

#endif  // TELEOP_INPUTSOURCEHANDLE_HPP
