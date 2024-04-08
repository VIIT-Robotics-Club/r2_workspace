// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from r2_interfaces:msg/LedStateArray.idl
// generated code does not contain a copyright notice

#ifndef R2_INTERFACES__MSG__DETAIL__LED_STATE_ARRAY__BUILDER_HPP_
#define R2_INTERFACES__MSG__DETAIL__LED_STATE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "r2_interfaces/msg/detail/led_state_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace r2_interfaces
{

namespace msg
{

namespace builder
{

class Init_LedStateArray_led_states
{
public:
  Init_LedStateArray_led_states()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::r2_interfaces::msg::LedStateArray led_states(::r2_interfaces::msg::LedStateArray::_led_states_type arg)
  {
    msg_.led_states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::r2_interfaces::msg::LedStateArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::r2_interfaces::msg::LedStateArray>()
{
  return r2_interfaces::msg::builder::Init_LedStateArray_led_states();
}

}  // namespace r2_interfaces

#endif  // R2_INTERFACES__MSG__DETAIL__LED_STATE_ARRAY__BUILDER_HPP_
