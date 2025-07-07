// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from main_interface:srv/ResetOdom.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__RESET_ODOM__STRUCT_H_
#define MAIN_INTERFACE__SRV__DETAIL__RESET_ODOM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ResetOdom in the package main_interface.
typedef struct main_interface__srv__ResetOdom_Request
{
  double x;
  double y;
  double theta;
} main_interface__srv__ResetOdom_Request;

// Struct for a sequence of main_interface__srv__ResetOdom_Request.
typedef struct main_interface__srv__ResetOdom_Request__Sequence
{
  main_interface__srv__ResetOdom_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} main_interface__srv__ResetOdom_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ResetOdom in the package main_interface.
typedef struct main_interface__srv__ResetOdom_Response
{
  uint8_t structure_needs_at_least_one_member;
} main_interface__srv__ResetOdom_Response;

// Struct for a sequence of main_interface__srv__ResetOdom_Response.
typedef struct main_interface__srv__ResetOdom_Response__Sequence
{
  main_interface__srv__ResetOdom_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} main_interface__srv__ResetOdom_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAIN_INTERFACE__SRV__DETAIL__RESET_ODOM__STRUCT_H_
