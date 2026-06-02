// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mi_interfaces:srv/Saludo.idl
// generated code does not contain a copyright notice

#ifndef MI_INTERFACES__SRV__DETAIL__SALUDO__STRUCT_H_
#define MI_INTERFACES__SRV__DETAIL__SALUDO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'nombre'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Saludo in the package mi_interfaces.
typedef struct mi_interfaces__srv__Saludo_Request
{
  rosidl_runtime_c__String nombre;
} mi_interfaces__srv__Saludo_Request;

// Struct for a sequence of mi_interfaces__srv__Saludo_Request.
typedef struct mi_interfaces__srv__Saludo_Request__Sequence
{
  mi_interfaces__srv__Saludo_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mi_interfaces__srv__Saludo_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'mensaje'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Saludo in the package mi_interfaces.
typedef struct mi_interfaces__srv__Saludo_Response
{
  rosidl_runtime_c__String mensaje;
} mi_interfaces__srv__Saludo_Response;

// Struct for a sequence of mi_interfaces__srv__Saludo_Response.
typedef struct mi_interfaces__srv__Saludo_Response__Sequence
{
  mi_interfaces__srv__Saludo_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mi_interfaces__srv__Saludo_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MI_INTERFACES__SRV__DETAIL__SALUDO__STRUCT_H_
