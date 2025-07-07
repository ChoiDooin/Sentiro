// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from main_interface:srv/Battery.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__BATTERY__TRAITS_HPP_
#define MAIN_INTERFACE__SRV__DETAIL__BATTERY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "main_interface/srv/detail/battery__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace main_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Battery_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Battery_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Battery_Request & msg, bool use_flow_style = false)
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
  const main_interface::srv::Battery_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  main_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use main_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const main_interface::srv::Battery_Request & msg)
{
  return main_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<main_interface::srv::Battery_Request>()
{
  return "main_interface::srv::Battery_Request";
}

template<>
inline const char * name<main_interface::srv::Battery_Request>()
{
  return "main_interface/srv/Battery_Request";
}

template<>
struct has_fixed_size<main_interface::srv::Battery_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<main_interface::srv::Battery_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<main_interface::srv::Battery_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace main_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Battery_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: volt
  {
    out << "volt: ";
    rosidl_generator_traits::value_to_yaml(msg.volt, out);
    out << ", ";
  }

  // member: soc
  {
    out << "soc: ";
    rosidl_generator_traits::value_to_yaml(msg.soc, out);
    out << ", ";
  }

  // member: current
  {
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Battery_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: volt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "volt: ";
    rosidl_generator_traits::value_to_yaml(msg.volt, out);
    out << "\n";
  }

  // member: soc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "soc: ";
    rosidl_generator_traits::value_to_yaml(msg.soc, out);
    out << "\n";
  }

  // member: current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Battery_Response & msg, bool use_flow_style = false)
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
  const main_interface::srv::Battery_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  main_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use main_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const main_interface::srv::Battery_Response & msg)
{
  return main_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<main_interface::srv::Battery_Response>()
{
  return "main_interface::srv::Battery_Response";
}

template<>
inline const char * name<main_interface::srv::Battery_Response>()
{
  return "main_interface/srv/Battery_Response";
}

template<>
struct has_fixed_size<main_interface::srv::Battery_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<main_interface::srv::Battery_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<main_interface::srv::Battery_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<main_interface::srv::Battery>()
{
  return "main_interface::srv::Battery";
}

template<>
inline const char * name<main_interface::srv::Battery>()
{
  return "main_interface/srv/Battery";
}

template<>
struct has_fixed_size<main_interface::srv::Battery>
  : std::integral_constant<
    bool,
    has_fixed_size<main_interface::srv::Battery_Request>::value &&
    has_fixed_size<main_interface::srv::Battery_Response>::value
  >
{
};

template<>
struct has_bounded_size<main_interface::srv::Battery>
  : std::integral_constant<
    bool,
    has_bounded_size<main_interface::srv::Battery_Request>::value &&
    has_bounded_size<main_interface::srv::Battery_Response>::value
  >
{
};

template<>
struct is_service<main_interface::srv::Battery>
  : std::true_type
{
};

template<>
struct is_service_request<main_interface::srv::Battery_Request>
  : std::true_type
{
};

template<>
struct is_service_response<main_interface::srv::Battery_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAIN_INTERFACE__SRV__DETAIL__BATTERY__TRAITS_HPP_
