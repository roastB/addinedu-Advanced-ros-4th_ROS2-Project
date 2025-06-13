// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_msgs:msg/OrderInformation.idl
// generated code does not contain a copyright notice
#include "my_msgs/msg/detail/order_information__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ingredients`
#include "rosidl_runtime_c/string_functions.h"

bool
my_msgs__msg__OrderInformation__init(my_msgs__msg__OrderInformation * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // ingredients
  for (size_t i = 0; i < 4; ++i) {
    if (!rosidl_runtime_c__String__init(&msg->ingredients[i])) {
      my_msgs__msg__OrderInformation__fini(msg);
      return false;
    }
  }
  return true;
}

void
my_msgs__msg__OrderInformation__fini(my_msgs__msg__OrderInformation * msg)
{
  if (!msg) {
    return;
  }
  // id
  // ingredients
  for (size_t i = 0; i < 4; ++i) {
    rosidl_runtime_c__String__fini(&msg->ingredients[i]);
  }
}

bool
my_msgs__msg__OrderInformation__are_equal(const my_msgs__msg__OrderInformation * lhs, const my_msgs__msg__OrderInformation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // ingredients
  for (size_t i = 0; i < 4; ++i) {
    if (!rosidl_runtime_c__String__are_equal(
        &(lhs->ingredients[i]), &(rhs->ingredients[i])))
    {
      return false;
    }
  }
  return true;
}

bool
my_msgs__msg__OrderInformation__copy(
  const my_msgs__msg__OrderInformation * input,
  my_msgs__msg__OrderInformation * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // ingredients
  for (size_t i = 0; i < 4; ++i) {
    if (!rosidl_runtime_c__String__copy(
        &(input->ingredients[i]), &(output->ingredients[i])))
    {
      return false;
    }
  }
  return true;
}

my_msgs__msg__OrderInformation *
my_msgs__msg__OrderInformation__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__OrderInformation * msg = (my_msgs__msg__OrderInformation *)allocator.allocate(sizeof(my_msgs__msg__OrderInformation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_msgs__msg__OrderInformation));
  bool success = my_msgs__msg__OrderInformation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_msgs__msg__OrderInformation__destroy(my_msgs__msg__OrderInformation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_msgs__msg__OrderInformation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_msgs__msg__OrderInformation__Sequence__init(my_msgs__msg__OrderInformation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__OrderInformation * data = NULL;

  if (size) {
    data = (my_msgs__msg__OrderInformation *)allocator.zero_allocate(size, sizeof(my_msgs__msg__OrderInformation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_msgs__msg__OrderInformation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_msgs__msg__OrderInformation__fini(&data[i - 1]);
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
my_msgs__msg__OrderInformation__Sequence__fini(my_msgs__msg__OrderInformation__Sequence * array)
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
      my_msgs__msg__OrderInformation__fini(&array->data[i]);
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

my_msgs__msg__OrderInformation__Sequence *
my_msgs__msg__OrderInformation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__OrderInformation__Sequence * array = (my_msgs__msg__OrderInformation__Sequence *)allocator.allocate(sizeof(my_msgs__msg__OrderInformation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_msgs__msg__OrderInformation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_msgs__msg__OrderInformation__Sequence__destroy(my_msgs__msg__OrderInformation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_msgs__msg__OrderInformation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_msgs__msg__OrderInformation__Sequence__are_equal(const my_msgs__msg__OrderInformation__Sequence * lhs, const my_msgs__msg__OrderInformation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_msgs__msg__OrderInformation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_msgs__msg__OrderInformation__Sequence__copy(
  const my_msgs__msg__OrderInformation__Sequence * input,
  my_msgs__msg__OrderInformation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_msgs__msg__OrderInformation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_msgs__msg__OrderInformation * data =
      (my_msgs__msg__OrderInformation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_msgs__msg__OrderInformation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_msgs__msg__OrderInformation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_msgs__msg__OrderInformation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
