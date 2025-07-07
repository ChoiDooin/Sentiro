// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from main_interface:srv/Calg.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__CALG__STRUCT_H_
#define MAIN_INTERFACE__SRV__DETAIL__CALG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Calg in the package main_interface.
typedef struct main_interface__srv__Calg_Request
{
  uint8_t structure_needs_at_least_one_member;
} main_interface__srv__Calg_Request;

// Struct for a sequence of main_interface__srv__Calg_Request.
typedef struct main_interface__srv__Calg_Request__Sequence
{
  main_interface__srv__Calg_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} main_interface__srv__Calg_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Calg in the package main_interface.
typedef struct main_interface__srv__Calg_Response
{
  uint8_t structure_needs_at_least_one_member;
} main_interface__srv__Calg_Response;

// Struct for a sequence of main_interface__srv__Calg_Response.
typedef struct main_interface__srv__Calg_Response__Sequence
{
  main_interface__srv__Calg_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} main_interface__srv__Calg_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAIN_INTERFACE__SRV__DETAIL__CALG__STRUCT_H_
