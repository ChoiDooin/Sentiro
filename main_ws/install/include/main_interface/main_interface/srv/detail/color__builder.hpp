// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from main_interface:srv/Color.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__COLOR__BUILDER_HPP_
#define MAIN_INTERFACE__SRV__DETAIL__COLOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "main_interface/srv/detail/color__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace main_interface
{

namespace srv
{

namespace builder
{

class Init_Color_Request_blue
{
public:
  explicit Init_Color_Request_blue(::main_interface::srv::Color_Request & msg)
  : msg_(msg)
  {}
  ::main_interface::srv::Color_Request blue(::main_interface::srv::Color_Request::_blue_type arg)
  {
    msg_.blue = std::move(arg);
    return std::move(msg_);
  }

private:
  ::main_interface::srv::Color_Request msg_;
};

class Init_Color_Request_green
{
public:
  explicit Init_Color_Request_green(::main_interface::srv::Color_Request & msg)
  : msg_(msg)
  {}
  Init_Color_Request_blue green(::main_interface::srv::Color_Request::_green_type arg)
  {
    msg_.green = std::move(arg);
    return Init_Color_Request_blue(msg_);
  }

private:
  ::main_interface::srv::Color_Request msg_;
};

class Init_Color_Request_red
{
public:
  Init_Color_Request_red()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Color_Request_green red(::main_interface::srv::Color_Request::_red_type arg)
  {
    msg_.red = std::move(arg);
    return Init_Color_Request_green(msg_);
  }

private:
  ::main_interface::srv::Color_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::main_interface::srv::Color_Request>()
{
  return main_interface::srv::builder::Init_Color_Request_red();
}

}  // namespace main_interface


namespace main_interface
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::main_interface::srv::Color_Response>()
{
  return ::main_interface::srv::Color_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace main_interface

#endif  // MAIN_INTERFACE__SRV__DETAIL__COLOR__BUILDER_HPP_
