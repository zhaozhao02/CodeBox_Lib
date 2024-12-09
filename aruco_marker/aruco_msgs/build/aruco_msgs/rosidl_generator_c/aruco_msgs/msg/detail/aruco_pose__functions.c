// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aruco_msgs:msg/ArucoPose.idl
// generated code does not contain a copyright notice
#include "aruco_msgs/msg/detail/aruco_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
aruco_msgs__msg__ArucoPose__init(aruco_msgs__msg__ArucoPose * msg)
{
  if (!msg) {
    return false;
  }
  // mark_id
  // px
  // py
  // pz
  // ox
  // oy
  // oz
  // ow
  return true;
}

void
aruco_msgs__msg__ArucoPose__fini(aruco_msgs__msg__ArucoPose * msg)
{
  if (!msg) {
    return;
  }
  // mark_id
  // px
  // py
  // pz
  // ox
  // oy
  // oz
  // ow
}

bool
aruco_msgs__msg__ArucoPose__are_equal(const aruco_msgs__msg__ArucoPose * lhs, const aruco_msgs__msg__ArucoPose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mark_id
  if (lhs->mark_id != rhs->mark_id) {
    return false;
  }
  // px
  if (lhs->px != rhs->px) {
    return false;
  }
  // py
  if (lhs->py != rhs->py) {
    return false;
  }
  // pz
  if (lhs->pz != rhs->pz) {
    return false;
  }
  // ox
  if (lhs->ox != rhs->ox) {
    return false;
  }
  // oy
  if (lhs->oy != rhs->oy) {
    return false;
  }
  // oz
  if (lhs->oz != rhs->oz) {
    return false;
  }
  // ow
  if (lhs->ow != rhs->ow) {
    return false;
  }
  return true;
}

bool
aruco_msgs__msg__ArucoPose__copy(
  const aruco_msgs__msg__ArucoPose * input,
  aruco_msgs__msg__ArucoPose * output)
{
  if (!input || !output) {
    return false;
  }
  // mark_id
  output->mark_id = input->mark_id;
  // px
  output->px = input->px;
  // py
  output->py = input->py;
  // pz
  output->pz = input->pz;
  // ox
  output->ox = input->ox;
  // oy
  output->oy = input->oy;
  // oz
  output->oz = input->oz;
  // ow
  output->ow = input->ow;
  return true;
}

aruco_msgs__msg__ArucoPose *
aruco_msgs__msg__ArucoPose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aruco_msgs__msg__ArucoPose * msg = (aruco_msgs__msg__ArucoPose *)allocator.allocate(sizeof(aruco_msgs__msg__ArucoPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aruco_msgs__msg__ArucoPose));
  bool success = aruco_msgs__msg__ArucoPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aruco_msgs__msg__ArucoPose__destroy(aruco_msgs__msg__ArucoPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aruco_msgs__msg__ArucoPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aruco_msgs__msg__ArucoPose__Sequence__init(aruco_msgs__msg__ArucoPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aruco_msgs__msg__ArucoPose * data = NULL;

  if (size) {
    data = (aruco_msgs__msg__ArucoPose *)allocator.zero_allocate(size, sizeof(aruco_msgs__msg__ArucoPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aruco_msgs__msg__ArucoPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aruco_msgs__msg__ArucoPose__fini(&data[i - 1]);
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
aruco_msgs__msg__ArucoPose__Sequence__fini(aruco_msgs__msg__ArucoPose__Sequence * array)
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
      aruco_msgs__msg__ArucoPose__fini(&array->data[i]);
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

aruco_msgs__msg__ArucoPose__Sequence *
aruco_msgs__msg__ArucoPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aruco_msgs__msg__ArucoPose__Sequence * array = (aruco_msgs__msg__ArucoPose__Sequence *)allocator.allocate(sizeof(aruco_msgs__msg__ArucoPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aruco_msgs__msg__ArucoPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aruco_msgs__msg__ArucoPose__Sequence__destroy(aruco_msgs__msg__ArucoPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aruco_msgs__msg__ArucoPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aruco_msgs__msg__ArucoPose__Sequence__are_equal(const aruco_msgs__msg__ArucoPose__Sequence * lhs, const aruco_msgs__msg__ArucoPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aruco_msgs__msg__ArucoPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aruco_msgs__msg__ArucoPose__Sequence__copy(
  const aruco_msgs__msg__ArucoPose__Sequence * input,
  aruco_msgs__msg__ArucoPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aruco_msgs__msg__ArucoPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    aruco_msgs__msg__ArucoPose * data =
      (aruco_msgs__msg__ArucoPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aruco_msgs__msg__ArucoPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          aruco_msgs__msg__ArucoPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!aruco_msgs__msg__ArucoPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
