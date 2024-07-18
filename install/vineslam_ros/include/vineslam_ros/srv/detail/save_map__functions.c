// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vineslam_ros:srv/SaveMap.idl
// generated code does not contain a copyright notice
#include "vineslam_ros/srv/detail/save_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
vineslam_ros__srv__SaveMap_Request__init(vineslam_ros__srv__SaveMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
vineslam_ros__srv__SaveMap_Request__fini(vineslam_ros__srv__SaveMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
vineslam_ros__srv__SaveMap_Request__are_equal(const vineslam_ros__srv__SaveMap_Request * lhs, const vineslam_ros__srv__SaveMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
vineslam_ros__srv__SaveMap_Request__copy(
  const vineslam_ros__srv__SaveMap_Request * input,
  vineslam_ros__srv__SaveMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

vineslam_ros__srv__SaveMap_Request *
vineslam_ros__srv__SaveMap_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vineslam_ros__srv__SaveMap_Request * msg = (vineslam_ros__srv__SaveMap_Request *)allocator.allocate(sizeof(vineslam_ros__srv__SaveMap_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vineslam_ros__srv__SaveMap_Request));
  bool success = vineslam_ros__srv__SaveMap_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vineslam_ros__srv__SaveMap_Request__destroy(vineslam_ros__srv__SaveMap_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vineslam_ros__srv__SaveMap_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vineslam_ros__srv__SaveMap_Request__Sequence__init(vineslam_ros__srv__SaveMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vineslam_ros__srv__SaveMap_Request * data = NULL;

  if (size) {
    data = (vineslam_ros__srv__SaveMap_Request *)allocator.zero_allocate(size, sizeof(vineslam_ros__srv__SaveMap_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vineslam_ros__srv__SaveMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vineslam_ros__srv__SaveMap_Request__fini(&data[i - 1]);
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
vineslam_ros__srv__SaveMap_Request__Sequence__fini(vineslam_ros__srv__SaveMap_Request__Sequence * array)
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
      vineslam_ros__srv__SaveMap_Request__fini(&array->data[i]);
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

vineslam_ros__srv__SaveMap_Request__Sequence *
vineslam_ros__srv__SaveMap_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vineslam_ros__srv__SaveMap_Request__Sequence * array = (vineslam_ros__srv__SaveMap_Request__Sequence *)allocator.allocate(sizeof(vineslam_ros__srv__SaveMap_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vineslam_ros__srv__SaveMap_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vineslam_ros__srv__SaveMap_Request__Sequence__destroy(vineslam_ros__srv__SaveMap_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vineslam_ros__srv__SaveMap_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vineslam_ros__srv__SaveMap_Request__Sequence__are_equal(const vineslam_ros__srv__SaveMap_Request__Sequence * lhs, const vineslam_ros__srv__SaveMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vineslam_ros__srv__SaveMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vineslam_ros__srv__SaveMap_Request__Sequence__copy(
  const vineslam_ros__srv__SaveMap_Request__Sequence * input,
  vineslam_ros__srv__SaveMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vineslam_ros__srv__SaveMap_Request);
    vineslam_ros__srv__SaveMap_Request * data =
      (vineslam_ros__srv__SaveMap_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vineslam_ros__srv__SaveMap_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          vineslam_ros__srv__SaveMap_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vineslam_ros__srv__SaveMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `save_map`
#include "std_msgs/msg/detail/bool__functions.h"

bool
vineslam_ros__srv__SaveMap_Response__init(vineslam_ros__srv__SaveMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // save_map
  if (!std_msgs__msg__Bool__init(&msg->save_map)) {
    vineslam_ros__srv__SaveMap_Response__fini(msg);
    return false;
  }
  return true;
}

void
vineslam_ros__srv__SaveMap_Response__fini(vineslam_ros__srv__SaveMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // save_map
  std_msgs__msg__Bool__fini(&msg->save_map);
}

bool
vineslam_ros__srv__SaveMap_Response__are_equal(const vineslam_ros__srv__SaveMap_Response * lhs, const vineslam_ros__srv__SaveMap_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // save_map
  if (!std_msgs__msg__Bool__are_equal(
      &(lhs->save_map), &(rhs->save_map)))
  {
    return false;
  }
  return true;
}

bool
vineslam_ros__srv__SaveMap_Response__copy(
  const vineslam_ros__srv__SaveMap_Response * input,
  vineslam_ros__srv__SaveMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // save_map
  if (!std_msgs__msg__Bool__copy(
      &(input->save_map), &(output->save_map)))
  {
    return false;
  }
  return true;
}

vineslam_ros__srv__SaveMap_Response *
vineslam_ros__srv__SaveMap_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vineslam_ros__srv__SaveMap_Response * msg = (vineslam_ros__srv__SaveMap_Response *)allocator.allocate(sizeof(vineslam_ros__srv__SaveMap_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vineslam_ros__srv__SaveMap_Response));
  bool success = vineslam_ros__srv__SaveMap_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vineslam_ros__srv__SaveMap_Response__destroy(vineslam_ros__srv__SaveMap_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vineslam_ros__srv__SaveMap_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vineslam_ros__srv__SaveMap_Response__Sequence__init(vineslam_ros__srv__SaveMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vineslam_ros__srv__SaveMap_Response * data = NULL;

  if (size) {
    data = (vineslam_ros__srv__SaveMap_Response *)allocator.zero_allocate(size, sizeof(vineslam_ros__srv__SaveMap_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vineslam_ros__srv__SaveMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vineslam_ros__srv__SaveMap_Response__fini(&data[i - 1]);
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
vineslam_ros__srv__SaveMap_Response__Sequence__fini(vineslam_ros__srv__SaveMap_Response__Sequence * array)
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
      vineslam_ros__srv__SaveMap_Response__fini(&array->data[i]);
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

vineslam_ros__srv__SaveMap_Response__Sequence *
vineslam_ros__srv__SaveMap_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vineslam_ros__srv__SaveMap_Response__Sequence * array = (vineslam_ros__srv__SaveMap_Response__Sequence *)allocator.allocate(sizeof(vineslam_ros__srv__SaveMap_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vineslam_ros__srv__SaveMap_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vineslam_ros__srv__SaveMap_Response__Sequence__destroy(vineslam_ros__srv__SaveMap_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vineslam_ros__srv__SaveMap_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vineslam_ros__srv__SaveMap_Response__Sequence__are_equal(const vineslam_ros__srv__SaveMap_Response__Sequence * lhs, const vineslam_ros__srv__SaveMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vineslam_ros__srv__SaveMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vineslam_ros__srv__SaveMap_Response__Sequence__copy(
  const vineslam_ros__srv__SaveMap_Response__Sequence * input,
  vineslam_ros__srv__SaveMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vineslam_ros__srv__SaveMap_Response);
    vineslam_ros__srv__SaveMap_Response * data =
      (vineslam_ros__srv__SaveMap_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vineslam_ros__srv__SaveMap_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          vineslam_ros__srv__SaveMap_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vineslam_ros__srv__SaveMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
