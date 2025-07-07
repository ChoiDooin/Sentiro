// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef MAIN_INTERFACE__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define MAIN_INTERFACE__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_main_interface __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_main_interface __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_main_interface __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_main_interface __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_main_interface
    #define ROSIDL_GENERATOR_C_PUBLIC_main_interface ROSIDL_GENERATOR_C_EXPORT_main_interface
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_main_interface ROSIDL_GENERATOR_C_IMPORT_main_interface
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_main_interface __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_main_interface
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_main_interface __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_main_interface
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // MAIN_INTERFACE__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
