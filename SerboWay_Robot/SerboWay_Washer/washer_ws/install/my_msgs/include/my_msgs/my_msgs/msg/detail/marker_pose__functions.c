// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_msgs:msg/MarkerPose.idl
// generated code does not contain a copyright notice
#include "my_msgs/msg/detail/marker_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
my_msgs__msg__MarkerPose__init(my_msgs__msg__MarkerPose * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // pose_of_6d
  return true;
}

void
my_msgs__msg__MarkerPose__fini(my_msgs__msg__MarkerPose * msg)
{
  if (!msg) {
    return;
  }
  // id
  // pose_of_6d
}

bool
my_msgs__msg__MarkerPose__are_equal(const my_msgs__msg__MarkerPose * lhs, const my_msgs__msg__MarkerPose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // pose_of_6d
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->pose_of_6d[i] != rhs->pose_of_6d[i]) {
      return false;
    }
  }
  return true;
}

bool
my_msgs__msg__MarkerPose__copy(
  const my_msgs__msg__MarkerPose * input,
  my_msgs__msg__MarkerPose * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // pose_of_6d
  for (size_t i = 0; i < 6; ++i) {
    output->pose_of_6d[i] = input->pose_of_6d[i];
  }
  return true;
}

my_msgs__msg__MarkerPose *
my_msgs__msg__MarkerPose__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__MarkerPose * msg = (my_msgs__msg__MarkerPose *)allocator.allocate(sizeof(my_msgs__msg__MarkerPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_msgs__msg__MarkerPose));
  bool success = my_msgs__msg__MarkerPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_msgs__msg__MarkerPose__destroy(my_msgs__msg__MarkerPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_msgs__msg__MarkerPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_msgs__msg__MarkerPose__Sequence__init(my_msgs__msg__MarkerPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__MarkerPose * data = NULL;

  if (size) {
    data = (my_msgs__msg__MarkerPose *)allocator.zero_allocate(size, sizeof(my_msgs__msg__MarkerPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_msgs__msg__MarkerPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_msgs__msg__MarkerPose__fini(&data[i - 1]);
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
my_msgs__msg__MarkerPose__Sequence__fini(my_msgs__msg__MarkerPose__Sequence * array)
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
      my_msgs__msg__MarkerPose__fini(&array->data[i]);
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

my_msgs__msg__MarkerPose__Sequence *
my_msgs__msg__MarkerPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__MarkerPose__Sequence * array = (my_msgs__msg__MarkerPose__Sequence *)allocator.allocate(sizeof(my_msgs__msg__MarkerPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_msgs__msg__MarkerPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_msgs__msg__MarkerPose__Sequence__destroy(my_msgs__msg__MarkerPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_msgs__msg__MarkerPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_msgs__msg__MarkerPose__Sequence__are_equal(const my_msgs__msg__MarkerPose__Sequence * lhs, const my_msgs__msg__MarkerPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_msgs__msg__MarkerPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_msgs__msg__MarkerPose__Sequence__copy(
  const my_msgs__msg__MarkerPose__Sequence * input,
  my_msgs__msg__MarkerPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_msgs__msg__MarkerPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_msgs__msg__MarkerPose * data =
      (my_msgs__msg__MarkerPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_msgs__msg__MarkerPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_msgs__msg__MarkerPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_msgs__msg__MarkerPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
