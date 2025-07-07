// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from main_interface:srv/ResetOdom.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__RESET_ODOM__TRAITS_HPP_
#define MAIN_INTERFACE__SRV__DETAIL__RESET_ODOM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "main_interface/srv/detail/reset_odom__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace main_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const ResetOdom_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ResetOdom_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ResetOdom_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace main_interface

namespace rosidl_generator_traits
{

[[deprecated("use main_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const main_interface::srv::ResetOdom_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  main_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use main_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const main_interface::srv::ResetOdom_Request & msg)
{
  return main_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<main_interface::srv::ResetOdom_Request>()
{
  return "main_interface::srv::ResetOdom_Request";
}

template<>
inline const char * name<main_interface::srv::ResetOdom_Request>()
{
  return "main_interface/srv/ResetOdom_Request";
}

template<>
struct has_fixed_size<main_interface::srv::ResetOdom_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<main_interface::srv::ResetOdom_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<main_interface::srv::ResetOdom_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace main_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const ResetOdom_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ResetOdom_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ResetOdom_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace main_interface

namespace rosidl_generator_traits
{

[[deprecated("use main_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const main_interface::srv::ResetOdom_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  main_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use main_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const main_interface::srv::ResetOdom_Response & msg)
{
  return main_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<main_interface::srv::ResetOdom_Response>()
{
  return "main_interface::srv::ResetOdom_Response";
}

template<>
inline const char * name<main_interface::srv::ResetOdom_Response>()
{
  return "main_interface/srv/ResetOdom_Response";
}

template<>
struct has_fixed_size<main_interface::srv::ResetOdom_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<main_interface::srv::ResetOdom_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<main_interface::srv::ResetOdom_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<main_interface::srv::ResetOdom>()
{
  return "main_interface::srv::ResetOdom";
}

template<>
inline const char * name<main_interface::srv::ResetOdom>()
{
  return "main_interface/srv/ResetOdom";
}

template<>
struct has_fixed_size<main_interface::srv::ResetOdom>
  : std::integral_constant<
    bool,
    has_fixed_size<main_interface::srv::ResetOdom_Request>::value &&
    has_fixed_size<main_interface::srv::ResetOdom_Response>::value
  >
{
};

template<>
struct has_bounded_size<main_interface::srv::ResetOdom>
  : std::integral_constant<
    bool,
    has_bounded_size<main_interface::srv::ResetOdom_Request>::value &&
    has_bounded_size<main_interface::srv::ResetOdom_Response>::value
  >
{
};

template<>
struct is_service<main_interface::srv::ResetOdom>
  : std::true_type
{
};

template<>
struct is_service_request<main_interface::srv::ResetOdom_Request>
  : std::true_type
{
};

template<>
struct is_service_response<main_interface::srv::ResetOdom_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAIN_INTERFACE__SRV__DETAIL__RESET_ODOM__TRAITS_HPP_
