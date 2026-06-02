// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mi_interfaces:srv/Saludo.idl
// generated code does not contain a copyright notice

#ifndef MI_INTERFACES__SRV__DETAIL__SALUDO__BUILDER_HPP_
#define MI_INTERFACES__SRV__DETAIL__SALUDO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mi_interfaces/srv/detail/saludo__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mi_interfaces
{

namespace srv
{

namespace builder
{

class Init_Saludo_Request_nombre
{
public:
  Init_Saludo_Request_nombre()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mi_interfaces::srv::Saludo_Request nombre(::mi_interfaces::srv::Saludo_Request::_nombre_type arg)
  {
    msg_.nombre = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mi_interfaces::srv::Saludo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mi_interfaces::srv::Saludo_Request>()
{
  return mi_interfaces::srv::builder::Init_Saludo_Request_nombre();
}

}  // namespace mi_interfaces


namespace mi_interfaces
{

namespace srv
{

namespace builder
{

class Init_Saludo_Response_mensaje
{
public:
  Init_Saludo_Response_mensaje()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mi_interfaces::srv::Saludo_Response mensaje(::mi_interfaces::srv::Saludo_Response::_mensaje_type arg)
  {
    msg_.mensaje = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mi_interfaces::srv::Saludo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mi_interfaces::srv::Saludo_Response>()
{
  return mi_interfaces::srv::builder::Init_Saludo_Response_mensaje();
}

}  // namespace mi_interfaces

#endif  // MI_INTERFACES__SRV__DETAIL__SALUDO__BUILDER_HPP_
