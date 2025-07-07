// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from main_interface:srv/Battery.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__BATTERY__STRUCT_H_
#define MAIN_INTERFACE__SRV__DETAIL__BATTERY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Battery in the package main_interface.
typedef struct main_interface__srv__Battery_Request
{
  uint8_t structure_needs_at_least_one_member;
} main_interface__srv__Battery_Request;

// Struct for a sequence of main_interface__srv__Battery_Request.
typedef struct main_interface__srv__Battery_Request__Sequence
{
  main_interface__srv__Battery_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} main_interface__srv__Battery_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Battery in the package main_interface.
typedef struct main_interface__srv__Battery_Response
{
  double volt;
  double soc;
  double current;
} main_interface__srv__Battery_Response;

// Struct for a sequence of main_interface__srv__Battery_Response.
typedef struct main_interface__srv__Battery_Response__Sequence
{
  main_interface__srv__Battery_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} main_interface__srv__Battery_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAIN_INTERFACE__SRV__DETAIL__BATTERY__STRUCT_H_
