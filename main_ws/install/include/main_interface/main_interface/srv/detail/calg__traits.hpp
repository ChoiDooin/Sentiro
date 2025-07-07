// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from main_interface:srv/Calg.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__CALG__TRAITS_HPP_
#define MAIN_INTERFACE__SRV__DETAIL__CALG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "main_interface/srv/detail/calg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace main_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Calg_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Calg_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Calg_Request & msg, bool use_flow_style = false)
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
  const main_interface::srv::Calg_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  main_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use main_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const main_interface::srv::Calg_Request & msg)
{
  return main_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<main_interface::srv::Calg_Request>()
{
  return "main_interface::srv::Calg_Request";
}

template<>
inline const char * name<main_interface::srv::Calg_Request>()
{
  return "main_interface/srv/Calg_Request";
}

template<>
struct has_fixed_size<main_interface::srv::Calg_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<main_interface::srv::Calg_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<main_interface::srv::Calg_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace main_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Calg_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Calg_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Calg_Response & msg, bool use_flow_style = false)
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
  const main_interface::srv::Calg_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  main_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use main_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const main_interface::srv::Calg_Response & msg)
{
  return main_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<main_interface::srv::Calg_Response>()
{
  return "main_interface::srv::Calg_Response";
}

template<>
inline const char * name<main_interface::srv::Calg_Response>()
{
  return "main_interface/srv/Calg_Response";
}

template<>
struct has_fixed_size<main_interface::srv::Calg_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<main_interface::srv::Calg_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<main_interface::srv::Calg_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<main_interface::srv::Calg>()
{
  return "main_interface::srv::Calg";
}

template<>
inline const char * name<main_interface::srv::Calg>()
{
  return "main_interface/srv/Calg";
}

template<>
struct has_fixed_size<main_interface::srv::Calg>
  : std::integral_constant<
    bool,
    has_fixed_size<main_interface::srv::Calg_Request>::value &&
    has_fixed_size<main_interface::srv::Calg_Response>::value
  >
{
};

template<>
struct has_bounded_size<main_interface::srv::Calg>
  : std::integral_constant<
    bool,
    has_bounded_size<main_interface::srv::Calg_Request>::value &&
    has_bounded_size<main_interface::srv::Calg_Response>::value
  >
{
};

template<>
struct is_service<main_interface::srv::Calg>
  : std::true_type
{
};

template<>
struct is_service_request<main_interface::srv::Calg_Request>
  : std::true_type
{
};

template<>
struct is_service_response<main_interface::srv::Calg_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAIN_INTERFACE__SRV__DETAIL__CALG__TRAITS_HPP_
