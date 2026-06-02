// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mi_interfaces:srv/Saludo.idl
// generated code does not contain a copyright notice

#ifndef MI_INTERFACES__SRV__DETAIL__SALUDO__TRAITS_HPP_
#define MI_INTERFACES__SRV__DETAIL__SALUDO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mi_interfaces/srv/detail/saludo__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mi_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Saludo_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: nombre
  {
    out << "nombre: ";
    rosidl_generator_traits::value_to_yaml(msg.nombre, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Saludo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: nombre
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nombre: ";
    rosidl_generator_traits::value_to_yaml(msg.nombre, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Saludo_Request & msg, bool use_flow_style = false)
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

}  // namespace mi_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mi_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mi_interfaces::srv::Saludo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mi_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mi_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const mi_interfaces::srv::Saludo_Request & msg)
{
  return mi_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mi_interfaces::srv::Saludo_Request>()
{
  return "mi_interfaces::srv::Saludo_Request";
}

template<>
inline const char * name<mi_interfaces::srv::Saludo_Request>()
{
  return "mi_interfaces/srv/Saludo_Request";
}

template<>
struct has_fixed_size<mi_interfaces::srv::Saludo_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mi_interfaces::srv::Saludo_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mi_interfaces::srv::Saludo_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mi_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Saludo_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: mensaje
  {
    out << "mensaje: ";
    rosidl_generator_traits::value_to_yaml(msg.mensaje, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Saludo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mensaje
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mensaje: ";
    rosidl_generator_traits::value_to_yaml(msg.mensaje, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Saludo_Response & msg, bool use_flow_style = false)
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

}  // namespace mi_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mi_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mi_interfaces::srv::Saludo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mi_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mi_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const mi_interfaces::srv::Saludo_Response & msg)
{
  return mi_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mi_interfaces::srv::Saludo_Response>()
{
  return "mi_interfaces::srv::Saludo_Response";
}

template<>
inline const char * name<mi_interfaces::srv::Saludo_Response>()
{
  return "mi_interfaces/srv/Saludo_Response";
}

template<>
struct has_fixed_size<mi_interfaces::srv::Saludo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mi_interfaces::srv::Saludo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mi_interfaces::srv::Saludo_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mi_interfaces::srv::Saludo>()
{
  return "mi_interfaces::srv::Saludo";
}

template<>
inline const char * name<mi_interfaces::srv::Saludo>()
{
  return "mi_interfaces/srv/Saludo";
}

template<>
struct has_fixed_size<mi_interfaces::srv::Saludo>
  : std::integral_constant<
    bool,
    has_fixed_size<mi_interfaces::srv::Saludo_Request>::value &&
    has_fixed_size<mi_interfaces::srv::Saludo_Response>::value
  >
{
};

template<>
struct has_bounded_size<mi_interfaces::srv::Saludo>
  : std::integral_constant<
    bool,
    has_bounded_size<mi_interfaces::srv::Saludo_Request>::value &&
    has_bounded_size<mi_interfaces::srv::Saludo_Response>::value
  >
{
};

template<>
struct is_service<mi_interfaces::srv::Saludo>
  : std::true_type
{
};

template<>
struct is_service_request<mi_interfaces::srv::Saludo_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mi_interfaces::srv::Saludo_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MI_INTERFACES__SRV__DETAIL__SALUDO__TRAITS_HPP_
