// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from r2_interfaces:srv/SetLed.idl
// generated code does not contain a copyright notice

#ifndef R2_INTERFACES__SRV__DETAIL__SET_LED__STRUCT_H_
#define R2_INTERFACES__SRV__DETAIL__SET_LED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetLed in the package r2_interfaces.
typedef struct r2_interfaces__srv__SetLed_Request
{
  int64_t led_number;
  int64_t state;
} r2_interfaces__srv__SetLed_Request;

// Struct for a sequence of r2_interfaces__srv__SetLed_Request.
typedef struct r2_interfaces__srv__SetLed_Request__Sequence
{
  r2_interfaces__srv__SetLed_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} r2_interfaces__srv__SetLed_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetLed in the package r2_interfaces.
typedef struct r2_interfaces__srv__SetLed_Response
{
  bool success;
} r2_interfaces__srv__SetLed_Response;

// Struct for a sequence of r2_interfaces__srv__SetLed_Response.
typedef struct r2_interfaces__srv__SetLed_Response__Sequence
{
  r2_interfaces__srv__SetLed_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} r2_interfaces__srv__SetLed_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // R2_INTERFACES__SRV__DETAIL__SET_LED__STRUCT_H_
