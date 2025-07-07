// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from main_interface:srv/Battery.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__BATTERY__BUILDER_HPP_
#define MAIN_INTERFACE__SRV__DETAIL__BATTERY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "main_interface/srv/detail/battery__struct.hpp"
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
auto build<::main_interface::srv::Battery_Request>()
{
  return ::main_interface::srv::Battery_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace main_interface


namespace main_interface
{

namespace srv
{

namespace builder
{

class Init_Battery_Response_current
{
public:
  explicit Init_Battery_Response_current(::main_interface::srv::Battery_Response & msg)
  : msg_(msg)
  {}
  ::main_interface::srv::Battery_Response current(::main_interface::srv::Battery_Response::_current_type arg)
  {
    msg_.current = std::move(arg);
    return std::move(msg_);
  }

private:
  ::main_interface::srv::Battery_Response msg_;
};

class Init_Battery_Response_soc
{
public:
  explicit Init_Battery_Response_soc(::main_interface::srv::Battery_Response & msg)
  : msg_(msg)
  {}
  Init_Battery_Response_current soc(::main_interface::srv::Battery_Response::_soc_type arg)
  {
    msg_.soc = std::move(arg);
    return Init_Battery_Response_current(msg_);
  }

private:
  ::main_interface::srv::Battery_Response msg_;
};

class Init_Battery_Response_volt
{
public:
  Init_Battery_Response_volt()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Battery_Response_soc volt(::main_interface::srv::Battery_Response::_volt_type arg)
  {
    msg_.volt = std::move(arg);
    return Init_Battery_Response_soc(msg_);
  }

private:
  ::main_interface::srv::Battery_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::main_interface::srv::Battery_Response>()
{
  return main_interface::srv::builder::Init_Battery_Response_volt();
}

}  // namespace main_interface

#endif  // MAIN_INTERFACE__SRV__DETAIL__BATTERY__BUILDER_HPP_
