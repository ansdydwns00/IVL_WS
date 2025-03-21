// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vision_msgs:msg/Classification2D.idl
// generated code does not contain a copyright notice
#include "vision_msgs/msg/detail/classification2_d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `results`
#include "vision_msgs/msg/detail/object_hypothesis__functions.h"
// Member `source_img`
#include "sensor_msgs/msg/detail/image__functions.h"

bool
vision_msgs__msg__Classification2D__init(vision_msgs__msg__Classification2D * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    vision_msgs__msg__Classification2D__fini(msg);
    return false;
  }
  // results
  if (!vision_msgs__msg__ObjectHypothesis__Sequence__init(&msg->results, 0)) {
    vision_msgs__msg__Classification2D__fini(msg);
    return false;
  }
  // source_img
  if (!sensor_msgs__msg__Image__init(&msg->source_img)) {
    vision_msgs__msg__Classification2D__fini(msg);
    return false;
  }
  return true;
}

void
vision_msgs__msg__Classification2D__fini(vision_msgs__msg__Classification2D * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // results
  vision_msgs__msg__ObjectHypothesis__Sequence__fini(&msg->results);
  // source_img
  sensor_msgs__msg__Image__fini(&msg->source_img);
}

bool
vision_msgs__msg__Classification2D__are_equal(const vision_msgs__msg__Classification2D * lhs, const vision_msgs__msg__Classification2D * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // results
  if (!vision_msgs__msg__ObjectHypothesis__Sequence__are_equal(
      &(lhs->results), &(rhs->results)))
  {
    return false;
  }
  // source_img
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->source_img), &(rhs->source_img)))
  {
    return false;
  }
  return true;
}

bool
vision_msgs__msg__Classification2D__copy(
  const vision_msgs__msg__Classification2D * input,
  vision_msgs__msg__Classification2D * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // results
  if (!vision_msgs__msg__ObjectHypothesis__Sequence__copy(
      &(input->results), &(output->results)))
  {
    return false;
  }
  // source_img
  if (!sensor_msgs__msg__Image__copy(
      &(input->source_img), &(output->source_img)))
  {
    return false;
  }
  return true;
}

vision_msgs__msg__Classification2D *
vision_msgs__msg__Classification2D__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_msgs__msg__Classification2D * msg = (vision_msgs__msg__Classification2D *)allocator.allocate(sizeof(vision_msgs__msg__Classification2D), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vision_msgs__msg__Classification2D));
  bool success = vision_msgs__msg__Classification2D__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vision_msgs__msg__Classification2D__destroy(vision_msgs__msg__Classification2D * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vision_msgs__msg__Classification2D__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vision_msgs__msg__Classification2D__Sequence__init(vision_msgs__msg__Classification2D__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_msgs__msg__Classification2D * data = NULL;

  if (size) {
    data = (vision_msgs__msg__Classification2D *)allocator.zero_allocate(size, sizeof(vision_msgs__msg__Classification2D), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vision_msgs__msg__Classification2D__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vision_msgs__msg__Classification2D__fini(&data[i - 1]);
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
vision_msgs__msg__Classification2D__Sequence__fini(vision_msgs__msg__Classification2D__Sequence * array)
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
      vision_msgs__msg__Classification2D__fini(&array->data[i]);
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

vision_msgs__msg__Classification2D__Sequence *
vision_msgs__msg__Classification2D__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_msgs__msg__Classification2D__Sequence * array = (vision_msgs__msg__Classification2D__Sequence *)allocator.allocate(sizeof(vision_msgs__msg__Classification2D__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vision_msgs__msg__Classification2D__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vision_msgs__msg__Classification2D__Sequence__destroy(vision_msgs__msg__Classification2D__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vision_msgs__msg__Classification2D__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vision_msgs__msg__Classification2D__Sequence__are_equal(const vision_msgs__msg__Classification2D__Sequence * lhs, const vision_msgs__msg__Classification2D__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vision_msgs__msg__Classification2D__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vision_msgs__msg__Classification2D__Sequence__copy(
  const vision_msgs__msg__Classification2D__Sequence * input,
  vision_msgs__msg__Classification2D__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vision_msgs__msg__Classification2D);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vision_msgs__msg__Classification2D * data =
      (vision_msgs__msg__Classification2D *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vision_msgs__msg__Classification2D__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vision_msgs__msg__Classification2D__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vision_msgs__msg__Classification2D__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
