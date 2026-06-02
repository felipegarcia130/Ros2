// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mi_interfaces:srv/Saludo.idl
// generated code does not contain a copyright notice
#include "mi_interfaces/srv/detail/saludo__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mi_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mi_interfaces/srv/detail/saludo__struct.h"
#include "mi_interfaces/srv/detail/saludo__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // nombre
#include "rosidl_runtime_c/string_functions.h"  // nombre

// forward declare type support functions


using _Saludo_Request__ros_msg_type = mi_interfaces__srv__Saludo_Request;

static bool _Saludo_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Saludo_Request__ros_msg_type * ros_message = static_cast<const _Saludo_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: nombre
  {
    const rosidl_runtime_c__String * str = &ros_message->nombre;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _Saludo_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Saludo_Request__ros_msg_type * ros_message = static_cast<_Saludo_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: nombre
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->nombre.data) {
      rosidl_runtime_c__String__init(&ros_message->nombre);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->nombre,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'nombre'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mi_interfaces
size_t get_serialized_size_mi_interfaces__srv__Saludo_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Saludo_Request__ros_msg_type * ros_message = static_cast<const _Saludo_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name nombre
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->nombre.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _Saludo_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mi_interfaces__srv__Saludo_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mi_interfaces
size_t max_serialized_size_mi_interfaces__srv__Saludo_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: nombre
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mi_interfaces__srv__Saludo_Request;
    is_plain =
      (
      offsetof(DataType, nombre) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Saludo_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mi_interfaces__srv__Saludo_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Saludo_Request = {
  "mi_interfaces::srv",
  "Saludo_Request",
  _Saludo_Request__cdr_serialize,
  _Saludo_Request__cdr_deserialize,
  _Saludo_Request__get_serialized_size,
  _Saludo_Request__max_serialized_size
};

static rosidl_message_type_support_t _Saludo_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Saludo_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mi_interfaces, srv, Saludo_Request)() {
  return &_Saludo_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "mi_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "mi_interfaces/srv/detail/saludo__struct.h"
// already included above
// #include "mi_interfaces/srv/detail/saludo__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/string.h"  // mensaje
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // mensaje

// forward declare type support functions


using _Saludo_Response__ros_msg_type = mi_interfaces__srv__Saludo_Response;

static bool _Saludo_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Saludo_Response__ros_msg_type * ros_message = static_cast<const _Saludo_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: mensaje
  {
    const rosidl_runtime_c__String * str = &ros_message->mensaje;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _Saludo_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Saludo_Response__ros_msg_type * ros_message = static_cast<_Saludo_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: mensaje
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->mensaje.data) {
      rosidl_runtime_c__String__init(&ros_message->mensaje);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->mensaje,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'mensaje'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mi_interfaces
size_t get_serialized_size_mi_interfaces__srv__Saludo_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Saludo_Response__ros_msg_type * ros_message = static_cast<const _Saludo_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name mensaje
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->mensaje.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _Saludo_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mi_interfaces__srv__Saludo_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mi_interfaces
size_t max_serialized_size_mi_interfaces__srv__Saludo_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: mensaje
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mi_interfaces__srv__Saludo_Response;
    is_plain =
      (
      offsetof(DataType, mensaje) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Saludo_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mi_interfaces__srv__Saludo_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Saludo_Response = {
  "mi_interfaces::srv",
  "Saludo_Response",
  _Saludo_Response__cdr_serialize,
  _Saludo_Response__cdr_deserialize,
  _Saludo_Response__get_serialized_size,
  _Saludo_Response__max_serialized_size
};

static rosidl_message_type_support_t _Saludo_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Saludo_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mi_interfaces, srv, Saludo_Response)() {
  return &_Saludo_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "mi_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mi_interfaces/srv/saludo.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t Saludo__callbacks = {
  "mi_interfaces::srv",
  "Saludo",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mi_interfaces, srv, Saludo_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mi_interfaces, srv, Saludo_Response)(),
};

static rosidl_service_type_support_t Saludo__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &Saludo__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mi_interfaces, srv, Saludo)() {
  return &Saludo__handle;
}

#if defined(__cplusplus)
}
#endif
