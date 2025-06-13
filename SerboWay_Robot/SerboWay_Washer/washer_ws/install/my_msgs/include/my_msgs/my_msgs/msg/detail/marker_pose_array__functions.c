// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_msgs:msg/MarkerPoseArray.idl
// generated code does not contain a copyright notice
#include "my_msgs/msg/detail/marker_pose_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `markers`
#include "my_msgs/msg/detail/marker_pose__functions.h"

bool
my_msgs__msg__MarkerPoseArray__init(my_msgs__msg__MarkerPoseArray * msg)
{
  if (!msg) {
    return false;
  }
  // markers
  if (!my_msgs__msg__MarkerPose__Sequence__init(&msg->markers, 0)) {
    my_msgs__msg__MarkerPoseArray__fini(msg);
    return false;
  }
  return true;
}

void
my_msgs__msg__MarkerPoseArray__fini(my_msgs__msg__MarkerPoseArray * msg)
{
  if (!msg) {
    return;
  }
  // markers
  my_msgs__msg__MarkerPose__Sequence__fini(&msg->markers);
}

bool
my_msgs__msg__MarkerPoseArray__are_equal(const my_msgs__msg__MarkerPoseArray * lhs, const my_msgs__msg__MarkerPoseArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // markers
  if (!my_msgs__msg__MarkerPose__Sequence__are_equal(
      &(lhs->markers), &(rhs->markers)))
  {
    return false;
  }
  return true;
}

bool
my_msgs__msg__MarkerPoseArray__copy(
  const my_msgs__msg__MarkerPoseArray * input,
  my_msgs__msg__MarkerPoseArray * output)
{
  if (!input || !output) {
    return false;
  }
  // markers
  if (!my_msgs__msg__MarkerPose__Sequence__copy(
      &(input->markers), &(output->markers)))
  {
    return false;
  }
  return true;
}

my_msgs__msg__MarkerPoseArray *
my_msgs__msg__MarkerPoseArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__MarkerPoseArray * msg = (my_msgs__msg__MarkerPoseArray *)allocator.allocate(sizeof(my_msgs__msg__MarkerPoseArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_msgs__msg__MarkerPoseArray));
  bool success = my_msgs__msg__MarkerPoseArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_msgs__msg__MarkerPoseArray__destroy(my_msgs__msg__MarkerPoseArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_msgs__msg__MarkerPoseArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_msgs__msg__MarkerPoseArray__Sequence__init(my_msgs__msg__MarkerPoseArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__MarkerPoseArray * data = NULL;

  if (size) {
    data = (my_msgs__msg__MarkerPoseArray *)allocator.zero_allocate(size, sizeof(my_msgs__msg__MarkerPoseArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_msgs__msg__MarkerPoseArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_msgs__msg__MarkerPoseArray__fini(&data[i - 1]);
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
my_msgs__msg__MarkerPoseArray__Sequence__fini(my_msgs__msg__MarkerPoseArray__Sequence * array)
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
      my_msgs__msg__MarkerPoseArray__fini(&array->data[i]);
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

my_msgs__msg__MarkerPoseArray__Sequence *
my_msgs__msg__MarkerPoseArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__MarkerPoseArray__Sequence * array = (my_msgs__msg__MarkerPoseArray__Sequence *)allocator.allocate(sizeof(my_msgs__msg__MarkerPoseArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_msgs__msg__MarkerPoseArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_msgs__msg__MarkerPoseArray__Sequence__destroy(my_msgs__msg__MarkerPoseArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_msgs__msg__MarkerPoseArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_msgs__msg__MarkerPoseArray__Sequence__are_equal(const my_msgs__msg__MarkerPoseArray__Sequence * lhs, const my_msgs__msg__MarkerPoseArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_msgs__msg__MarkerPoseArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_msgs__msg__MarkerPoseArray__Sequence__copy(
  const my_msgs__msg__MarkerPoseArray__Sequence * input,
  my_msgs__msg__MarkerPoseArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_msgs__msg__MarkerPoseArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_msgs__msg__MarkerPoseArray * data =
      (my_msgs__msg__MarkerPoseArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_msgs__msg__MarkerPoseArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_msgs__msg__MarkerPoseArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_msgs__msg__MarkerPoseArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
