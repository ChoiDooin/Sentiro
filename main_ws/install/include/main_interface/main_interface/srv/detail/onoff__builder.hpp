// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from main_interface:srv/Onoff.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__ONOFF__BUILDER_HPP_
#define MAIN_INTERFACE__SRV__DETAIL__ONOFF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "main_interface/srv/detail/onoff__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace main_interface
{

namespace srv
{

namespace builder
{

class Init_Onoff_Request_set
{
public:
  Init_Onoff_Request_set()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::main_interface::srv::Onoff_Request set(::main_interface::srv::Onoff_Request::_set_type arg)
  {
    msg_.set = std::move(arg);
    return std::move(msg_);
  }

private:
  ::main_interface::srv::Onoff_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::main_interface::srv::Onoff_Request>()
{
  return main_interface::srv::builder::Init_Onoff_Request_set();
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
auto build<::main_interface::srv::Onoff_Response>()
{
  return ::main_interface::srv::Onoff_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace main_interface

#endif  // MAIN_INTERFACE__SRV__DETAIL__ONOFF__BUILDER_HPP_
