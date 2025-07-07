// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from main_interface:srv/SaveColor.idl
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__SRV__DETAIL__SAVE_COLOR__FUNCTIONS_H_
#define MAIN_INTERFACE__SRV__DETAIL__SAVE_COLOR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "main_interface/msg/rosidl_generator_c__visibility_control.h"

#include "main_interface/srv/detail/save_color__struct.h"

/// Initialize srv/SaveColor message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * main_interface__srv__SaveColor_Request
 * )) before or use
 * main_interface__srv__SaveColor_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Request__init(main_interface__srv__SaveColor_Request * msg);

/// Finalize srv/SaveColor message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
void
main_interface__srv__SaveColor_Request__fini(main_interface__srv__SaveColor_Request * msg);

/// Create srv/SaveColor message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * main_interface__srv__SaveColor_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
main_interface__srv__SaveColor_Request *
main_interface__srv__SaveColor_Request__create();

/// Destroy srv/SaveColor message.
/**
 * It calls
 * main_interface__srv__SaveColor_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
void
main_interface__srv__SaveColor_Request__destroy(main_interface__srv__SaveColor_Request * msg);

/// Check for srv/SaveColor message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Request__are_equal(const main_interface__srv__SaveColor_Request * lhs, const main_interface__srv__SaveColor_Request * rhs);

/// Copy a srv/SaveColor message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Request__copy(
  const main_interface__srv__SaveColor_Request * input,
  main_interface__srv__SaveColor_Request * output);

/// Initialize array of srv/SaveColor messages.
/**
 * It allocates the memory for the number of elements and calls
 * main_interface__srv__SaveColor_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Request__Sequence__init(main_interface__srv__SaveColor_Request__Sequence * array, size_t size);

/// Finalize array of srv/SaveColor messages.
/**
 * It calls
 * main_interface__srv__SaveColor_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
void
main_interface__srv__SaveColor_Request__Sequence__fini(main_interface__srv__SaveColor_Request__Sequence * array);

/// Create array of srv/SaveColor messages.
/**
 * It allocates the memory for the array and calls
 * main_interface__srv__SaveColor_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
main_interface__srv__SaveColor_Request__Sequence *
main_interface__srv__SaveColor_Request__Sequence__create(size_t size);

/// Destroy array of srv/SaveColor messages.
/**
 * It calls
 * main_interface__srv__SaveColor_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
void
main_interface__srv__SaveColor_Request__Sequence__destroy(main_interface__srv__SaveColor_Request__Sequence * array);

/// Check for srv/SaveColor message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Request__Sequence__are_equal(const main_interface__srv__SaveColor_Request__Sequence * lhs, const main_interface__srv__SaveColor_Request__Sequence * rhs);

/// Copy an array of srv/SaveColor messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Request__Sequence__copy(
  const main_interface__srv__SaveColor_Request__Sequence * input,
  main_interface__srv__SaveColor_Request__Sequence * output);

/// Initialize srv/SaveColor message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * main_interface__srv__SaveColor_Response
 * )) before or use
 * main_interface__srv__SaveColor_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Response__init(main_interface__srv__SaveColor_Response * msg);

/// Finalize srv/SaveColor message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
void
main_interface__srv__SaveColor_Response__fini(main_interface__srv__SaveColor_Response * msg);

/// Create srv/SaveColor message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * main_interface__srv__SaveColor_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
main_interface__srv__SaveColor_Response *
main_interface__srv__SaveColor_Response__create();

/// Destroy srv/SaveColor message.
/**
 * It calls
 * main_interface__srv__SaveColor_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
void
main_interface__srv__SaveColor_Response__destroy(main_interface__srv__SaveColor_Response * msg);

/// Check for srv/SaveColor message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Response__are_equal(const main_interface__srv__SaveColor_Response * lhs, const main_interface__srv__SaveColor_Response * rhs);

/// Copy a srv/SaveColor message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Response__copy(
  const main_interface__srv__SaveColor_Response * input,
  main_interface__srv__SaveColor_Response * output);

/// Initialize array of srv/SaveColor messages.
/**
 * It allocates the memory for the number of elements and calls
 * main_interface__srv__SaveColor_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Response__Sequence__init(main_interface__srv__SaveColor_Response__Sequence * array, size_t size);

/// Finalize array of srv/SaveColor messages.
/**
 * It calls
 * main_interface__srv__SaveColor_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
void
main_interface__srv__SaveColor_Response__Sequence__fini(main_interface__srv__SaveColor_Response__Sequence * array);

/// Create array of srv/SaveColor messages.
/**
 * It allocates the memory for the array and calls
 * main_interface__srv__SaveColor_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
main_interface__srv__SaveColor_Response__Sequence *
main_interface__srv__SaveColor_Response__Sequence__create(size_t size);

/// Destroy array of srv/SaveColor messages.
/**
 * It calls
 * main_interface__srv__SaveColor_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
void
main_interface__srv__SaveColor_Response__Sequence__destroy(main_interface__srv__SaveColor_Response__Sequence * array);

/// Check for srv/SaveColor message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Response__Sequence__are_equal(const main_interface__srv__SaveColor_Response__Sequence * lhs, const main_interface__srv__SaveColor_Response__Sequence * rhs);

/// Copy an array of srv/SaveColor messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_main_interface
bool
main_interface__srv__SaveColor_Response__Sequence__copy(
  const main_interface__srv__SaveColor_Response__Sequence * input,
  main_interface__srv__SaveColor_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MAIN_INTERFACE__SRV__DETAIL__SAVE_COLOR__FUNCTIONS_H_
