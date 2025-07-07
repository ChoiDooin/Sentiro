// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from main_interface:srv/ResetOdom.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__RESET_ODOM__BUILDER_HPP_
#define MAIN_INTERFACE__SRV__DETAIL__RESET_ODOM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "main_interface/srv/detail/reset_odom__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace main_interface
{

namespace srv
{

namespace builder
{

class Init_ResetOdom_Request_theta
{
public:
  explicit Init_ResetOdom_Request_theta(::main_interface::srv::ResetOdom_Request & msg)
  : msg_(msg)
  {}
  ::main_interface::srv::ResetOdom_Request theta(::main_interface::srv::ResetOdom_Request::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::main_interface::srv::ResetOdom_Request msg_;
};

class Init_ResetOdom_Request_y
{
public:
  explicit Init_ResetOdom_Request_y(::main_interface::srv::ResetOdom_Request & msg)
  : msg_(msg)
  {}
  Init_ResetOdom_Request_theta y(::main_interface::srv::ResetOdom_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ResetOdom_Request_theta(msg_);
  }

private:
  ::main_interface::srv::ResetOdom_Request msg_;
};

class Init_ResetOdom_Request_x
{
public:
  Init_ResetOdom_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ResetOdom_Request_y x(::main_interface::srv::ResetOdom_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ResetOdom_Request_y(msg_);
  }

private:
  ::main_interface::srv::ResetOdom_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::main_interface::srv::ResetOdom_Request>()
{
  return main_interface::srv::builder::Init_ResetOdom_Request_x();
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
auto build<::main_interface::srv::ResetOdom_Response>()
{
  return ::main_interface::srv::ResetOdom_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace main_interface

#endif  // MAIN_INTERFACE__SRV__DETAIL__RESET_ODOM__BUILDER_HPP_
