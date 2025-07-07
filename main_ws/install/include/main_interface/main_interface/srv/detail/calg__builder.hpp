// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from main_interface:srv/Calg.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__CALG__BUILDER_HPP_
#define MAIN_INTERFACE__SRV__DETAIL__CALG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "main_interface/srv/detail/calg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace main_interface
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::main_interface::srv::Calg_Request>()
{
  return ::main_interface::srv::Calg_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
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
auto build<::main_interface::srv::Calg_Response>()
{
  return ::main_interface::srv::Calg_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace main_interface

#endif  // MAIN_INTERFACE__SRV__DETAIL__CALG__BUILDER_HPP_
