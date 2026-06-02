// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mi_interfaces:srv/Saludo.idl
// generated code does not contain a copyright notice
#include "mi_interfaces/srv/detail/saludo__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `nombre`
#include "rosidl_runtime_c/string_functions.h"

bool
mi_interfaces__srv__Saludo_Request__init(mi_interfaces__srv__Saludo_Request * msg)
{
  if (!msg) {
    return false;
  }
  // nombre
  if (!rosidl_runtime_c__String__init(&msg->nombre)) {
    mi_interfaces__srv__Saludo_Request__fini(msg);
    return false;
  }
  return true;
}

void
mi_interfaces__srv__Saludo_Request__fini(mi_interfaces__srv__Saludo_Request * msg)
{
  if (!msg) {
    return;
  }
  // nombre
  rosidl_runtime_c__String__fini(&msg->nombre);
}

bool
mi_interfaces__srv__Saludo_Request__are_equal(const mi_interfaces__srv__Saludo_Request * lhs, const mi_interfaces__srv__Saludo_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // nombre
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->nombre), &(rhs->nombre)))
  {
    return false;
  }
  return true;
}

bool
mi_interfaces__srv__Saludo_Request__copy(
  const mi_interfaces__srv__Saludo_Request * input,
  mi_interfaces__srv__Saludo_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // nombre
  if (!rosidl_runtime_c__String__copy(
      &(input->nombre), &(output->nombre)))
  {
    return false;
  }
  return true;
}

mi_interfaces__srv__Saludo_Request *
mi_interfaces__srv__Saludo_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mi_interfaces__srv__Saludo_Request * msg = (mi_interfaces__srv__Saludo_Request *)allocator.allocate(sizeof(mi_interfaces__srv__Saludo_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mi_interfaces__srv__Saludo_Request));
  bool success = mi_interfaces__srv__Saludo_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mi_interfaces__srv__Saludo_Request__destroy(mi_interfaces__srv__Saludo_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mi_interfaces__srv__Saludo_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mi_interfaces__srv__Saludo_Request__Sequence__init(mi_interfaces__srv__Saludo_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mi_interfaces__srv__Saludo_Request * data = NULL;

  if (size) {
    data = (mi_interfaces__srv__Saludo_Request *)allocator.zero_allocate(size, sizeof(mi_interfaces__srv__Saludo_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mi_interfaces__srv__Saludo_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mi_interfaces__srv__Saludo_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mi_interfaces__srv__Saludo_Request__Sequence__fini(mi_interfaces__srv__Saludo_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mi_interfaces__srv__Saludo_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mi_interfaces__srv__Saludo_Request__Sequence *
mi_interfaces__srv__Saludo_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mi_interfaces__srv__Saludo_Request__Sequence * array = (mi_interfaces__srv__Saludo_Request__Sequence *)allocator.allocate(sizeof(mi_interfaces__srv__Saludo_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mi_interfaces__srv__Saludo_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mi_interfaces__srv__Saludo_Request__Sequence__destroy(mi_interfaces__srv__Saludo_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mi_interfaces__srv__Saludo_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mi_interfaces__srv__Saludo_Request__Sequence__are_equal(const mi_interfaces__srv__Saludo_Request__Sequence * lhs, const mi_interfaces__srv__Saludo_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mi_interfaces__srv__Saludo_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mi_interfaces__srv__Saludo_Request__Sequence__copy(
  const mi_interfaces__srv__Saludo_Request__Sequence * input,
  mi_interfaces__srv__Saludo_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mi_interfaces__srv__Saludo_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mi_interfaces__srv__Saludo_Request * data =
      (mi_interfaces__srv__Saludo_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mi_interfaces__srv__Saludo_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mi_interfaces__srv__Saludo_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mi_interfaces__srv__Saludo_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `mensaje`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
mi_interfaces__srv__Saludo_Response__init(mi_interfaces__srv__Saludo_Response * msg)
{
  if (!msg) {
    return false;
  }
  // mensaje
  if (!rosidl_runtime_c__String__init(&msg->mensaje)) {
    mi_interfaces__srv__Saludo_Response__fini(msg);
    return false;
  }
  return true;
}

void
mi_interfaces__srv__Saludo_Response__fini(mi_interfaces__srv__Saludo_Response * msg)
{
  if (!msg) {
    return;
  }
  // mensaje
  rosidl_runtime_c__String__fini(&msg->mensaje);
}

bool
mi_interfaces__srv__Saludo_Response__are_equal(const mi_interfaces__srv__Saludo_Response * lhs, const mi_interfaces__srv__Saludo_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mensaje
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mensaje), &(rhs->mensaje)))
  {
    return false;
  }
  return true;
}

bool
mi_interfaces__srv__Saludo_Response__copy(
  const mi_interfaces__srv__Saludo_Response * input,
  mi_interfaces__srv__Saludo_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // mensaje
  if (!rosidl_runtime_c__String__copy(
      &(input->mensaje), &(output->mensaje)))
  {
    return false;
  }
  return true;
}

mi_interfaces__srv__Saludo_Response *
mi_interfaces__srv__Saludo_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mi_interfaces__srv__Saludo_Response * msg = (mi_interfaces__srv__Saludo_Response *)allocator.allocate(sizeof(mi_interfaces__srv__Saludo_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mi_interfaces__srv__Saludo_Response));
  bool success = mi_interfaces__srv__Saludo_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mi_interfaces__srv__Saludo_Response__destroy(mi_interfaces__srv__Saludo_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mi_interfaces__srv__Saludo_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mi_interfaces__srv__Saludo_Response__Sequence__init(mi_interfaces__srv__Saludo_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mi_interfaces__srv__Saludo_Response * data = NULL;

  if (size) {
    data = (mi_interfaces__srv__Saludo_Response *)allocator.zero_allocate(size, sizeof(mi_interfaces__srv__Saludo_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mi_interfaces__srv__Saludo_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mi_interfaces__srv__Saludo_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mi_interfaces__srv__Saludo_Response__Sequence__fini(mi_interfaces__srv__Saludo_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mi_interfaces__srv__Saludo_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mi_interfaces__srv__Saludo_Response__Sequence *
mi_interfaces__srv__Saludo_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mi_interfaces__srv__Saludo_Response__Sequence * array = (mi_interfaces__srv__Saludo_Response__Sequence *)allocator.allocate(sizeof(mi_interfaces__srv__Saludo_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mi_interfaces__srv__Saludo_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mi_interfaces__srv__Saludo_Response__Sequence__destroy(mi_interfaces__srv__Saludo_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mi_interfaces__srv__Saludo_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mi_interfaces__srv__Saludo_Response__Sequence__are_equal(const mi_interfaces__srv__Saludo_Response__Sequence * lhs, const mi_interfaces__srv__Saludo_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mi_interfaces__srv__Saludo_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mi_interfaces__srv__Saludo_Response__Sequence__copy(
  const mi_interfaces__srv__Saludo_Response__Sequence * input,
  mi_interfaces__srv__Saludo_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mi_interfaces__srv__Saludo_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mi_interfaces__srv__Saludo_Response * data =
      (mi_interfaces__srv__Saludo_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mi_interfaces__srv__Saludo_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mi_interfaces__srv__Saludo_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mi_interfaces__srv__Saludo_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
