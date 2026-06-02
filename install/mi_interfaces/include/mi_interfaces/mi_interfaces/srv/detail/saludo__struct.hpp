// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mi_interfaces:srv/Saludo.idl
// generated code does not contain a copyright notice

#ifndef MI_INTERFACES__SRV__DETAIL__SALUDO__STRUCT_HPP_
#define MI_INTERFACES__SRV__DETAIL__SALUDO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mi_interfaces__srv__Saludo_Request __attribute__((deprecated))
#else
# define DEPRECATED__mi_interfaces__srv__Saludo_Request __declspec(deprecated)
#endif

namespace mi_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Saludo_Request_
{
  using Type = Saludo_Request_<ContainerAllocator>;

  explicit Saludo_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->nombre = "";
    }
  }

  explicit Saludo_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : nombre(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->nombre = "";
    }
  }

  // field types and members
  using _nombre_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _nombre_type nombre;

  // setters for named parameter idiom
  Type & set__nombre(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->nombre = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mi_interfaces::srv::Saludo_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mi_interfaces::srv::Saludo_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mi_interfaces::srv::Saludo_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mi_interfaces::srv::Saludo_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mi_interfaces::srv::Saludo_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mi_interfaces::srv::Saludo_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mi_interfaces::srv::Saludo_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mi_interfaces::srv::Saludo_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mi_interfaces::srv::Saludo_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mi_interfaces::srv::Saludo_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mi_interfaces__srv__Saludo_Request
    std::shared_ptr<mi_interfaces::srv::Saludo_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mi_interfaces__srv__Saludo_Request
    std::shared_ptr<mi_interfaces::srv::Saludo_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Saludo_Request_ & other) const
  {
    if (this->nombre != other.nombre) {
      return false;
    }
    return true;
  }
  bool operator!=(const Saludo_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Saludo_Request_

// alias to use template instance with default allocator
using Saludo_Request =
  mi_interfaces::srv::Saludo_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mi_interfaces


#ifndef _WIN32
# define DEPRECATED__mi_interfaces__srv__Saludo_Response __attribute__((deprecated))
#else
# define DEPRECATED__mi_interfaces__srv__Saludo_Response __declspec(deprecated)
#endif

namespace mi_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Saludo_Response_
{
  using Type = Saludo_Response_<ContainerAllocator>;

  explicit Saludo_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mensaje = "";
    }
  }

  explicit Saludo_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mensaje(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mensaje = "";
    }
  }

  // field types and members
  using _mensaje_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mensaje_type mensaje;

  // setters for named parameter idiom
  Type & set__mensaje(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mensaje = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mi_interfaces::srv::Saludo_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mi_interfaces::srv::Saludo_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mi_interfaces::srv::Saludo_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mi_interfaces::srv::Saludo_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mi_interfaces::srv::Saludo_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mi_interfaces::srv::Saludo_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mi_interfaces::srv::Saludo_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mi_interfaces::srv::Saludo_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mi_interfaces::srv::Saludo_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mi_interfaces::srv::Saludo_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mi_interfaces__srv__Saludo_Response
    std::shared_ptr<mi_interfaces::srv::Saludo_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mi_interfaces__srv__Saludo_Response
    std::shared_ptr<mi_interfaces::srv::Saludo_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Saludo_Response_ & other) const
  {
    if (this->mensaje != other.mensaje) {
      return false;
    }
    return true;
  }
  bool operator!=(const Saludo_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Saludo_Response_

// alias to use template instance with default allocator
using Saludo_Response =
  mi_interfaces::srv::Saludo_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mi_interfaces

namespace mi_interfaces
{

namespace srv
{

struct Saludo
{
  using Request = mi_interfaces::srv::Saludo_Request;
  using Response = mi_interfaces::srv::Saludo_Response;
};

}  // namespace srv

}  // namespace mi_interfaces

#endif  // MI_INTERFACES__SRV__DETAIL__SALUDO__STRUCT_HPP_
