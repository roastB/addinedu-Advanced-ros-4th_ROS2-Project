// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jetcobot_interfaces:action/OrderJetcobot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "jetcobot_interfaces/action/order_jetcobot.h"


#ifndef JETCOBOT_INTERFACES__ACTION__DETAIL__ORDER_JETCOBOT__STRUCT_H_
#define JETCOBOT_INTERFACES__ACTION__DETAIL__ORDER_JETCOBOT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'order'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/OrderJetcobot in the package jetcobot_interfaces.
typedef struct jetcobot_interfaces__action__OrderJetcobot_Goal
{
  rosidl_runtime_c__String order;
} jetcobot_interfaces__action__OrderJetcobot_Goal;

// Struct for a sequence of jetcobot_interfaces__action__OrderJetcobot_Goal.
typedef struct jetcobot_interfaces__action__OrderJetcobot_Goal__Sequence
{
  jetcobot_interfaces__action__OrderJetcobot_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetcobot_interfaces__action__OrderJetcobot_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/OrderJetcobot in the package jetcobot_interfaces.
typedef struct jetcobot_interfaces__action__OrderJetcobot_Result
{
  rosidl_runtime_c__String status;
} jetcobot_interfaces__action__OrderJetcobot_Result;

// Struct for a sequence of jetcobot_interfaces__action__OrderJetcobot_Result.
typedef struct jetcobot_interfaces__action__OrderJetcobot_Result__Sequence
{
  jetcobot_interfaces__action__OrderJetcobot_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetcobot_interfaces__action__OrderJetcobot_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/OrderJetcobot in the package jetcobot_interfaces.
typedef struct jetcobot_interfaces__action__OrderJetcobot_Feedback
{
  int32_t progress;
} jetcobot_interfaces__action__OrderJetcobot_Feedback;

// Struct for a sequence of jetcobot_interfaces__action__OrderJetcobot_Feedback.
typedef struct jetcobot_interfaces__action__OrderJetcobot_Feedback__Sequence
{
  jetcobot_interfaces__action__OrderJetcobot_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetcobot_interfaces__action__OrderJetcobot_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "jetcobot_interfaces/action/detail/order_jetcobot__struct.h"

/// Struct defined in action/OrderJetcobot in the package jetcobot_interfaces.
typedef struct jetcobot_interfaces__action__OrderJetcobot_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  jetcobot_interfaces__action__OrderJetcobot_Goal goal;
} jetcobot_interfaces__action__OrderJetcobot_SendGoal_Request;

// Struct for a sequence of jetcobot_interfaces__action__OrderJetcobot_SendGoal_Request.
typedef struct jetcobot_interfaces__action__OrderJetcobot_SendGoal_Request__Sequence
{
  jetcobot_interfaces__action__OrderJetcobot_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetcobot_interfaces__action__OrderJetcobot_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/OrderJetcobot in the package jetcobot_interfaces.
typedef struct jetcobot_interfaces__action__OrderJetcobot_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} jetcobot_interfaces__action__OrderJetcobot_SendGoal_Response;

// Struct for a sequence of jetcobot_interfaces__action__OrderJetcobot_SendGoal_Response.
typedef struct jetcobot_interfaces__action__OrderJetcobot_SendGoal_Response__Sequence
{
  jetcobot_interfaces__action__OrderJetcobot_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetcobot_interfaces__action__OrderJetcobot_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  jetcobot_interfaces__action__OrderJetcobot_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  jetcobot_interfaces__action__OrderJetcobot_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/OrderJetcobot in the package jetcobot_interfaces.
typedef struct jetcobot_interfaces__action__OrderJetcobot_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  jetcobot_interfaces__action__OrderJetcobot_SendGoal_Request__Sequence request;
  jetcobot_interfaces__action__OrderJetcobot_SendGoal_Response__Sequence response;
} jetcobot_interfaces__action__OrderJetcobot_SendGoal_Event;

// Struct for a sequence of jetcobot_interfaces__action__OrderJetcobot_SendGoal_Event.
typedef struct jetcobot_interfaces__action__OrderJetcobot_SendGoal_Event__Sequence
{
  jetcobot_interfaces__action__OrderJetcobot_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetcobot_interfaces__action__OrderJetcobot_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/OrderJetcobot in the package jetcobot_interfaces.
typedef struct jetcobot_interfaces__action__OrderJetcobot_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} jetcobot_interfaces__action__OrderJetcobot_GetResult_Request;

// Struct for a sequence of jetcobot_interfaces__action__OrderJetcobot_GetResult_Request.
typedef struct jetcobot_interfaces__action__OrderJetcobot_GetResult_Request__Sequence
{
  jetcobot_interfaces__action__OrderJetcobot_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetcobot_interfaces__action__OrderJetcobot_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "jetcobot_interfaces/action/detail/order_jetcobot__struct.h"

/// Struct defined in action/OrderJetcobot in the package jetcobot_interfaces.
typedef struct jetcobot_interfaces__action__OrderJetcobot_GetResult_Response
{
  int8_t status;
  jetcobot_interfaces__action__OrderJetcobot_Result result;
} jetcobot_interfaces__action__OrderJetcobot_GetResult_Response;

// Struct for a sequence of jetcobot_interfaces__action__OrderJetcobot_GetResult_Response.
typedef struct jetcobot_interfaces__action__OrderJetcobot_GetResult_Response__Sequence
{
  jetcobot_interfaces__action__OrderJetcobot_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetcobot_interfaces__action__OrderJetcobot_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  jetcobot_interfaces__action__OrderJetcobot_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  jetcobot_interfaces__action__OrderJetcobot_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/OrderJetcobot in the package jetcobot_interfaces.
typedef struct jetcobot_interfaces__action__OrderJetcobot_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  jetcobot_interfaces__action__OrderJetcobot_GetResult_Request__Sequence request;
  jetcobot_interfaces__action__OrderJetcobot_GetResult_Response__Sequence response;
} jetcobot_interfaces__action__OrderJetcobot_GetResult_Event;

// Struct for a sequence of jetcobot_interfaces__action__OrderJetcobot_GetResult_Event.
typedef struct jetcobot_interfaces__action__OrderJetcobot_GetResult_Event__Sequence
{
  jetcobot_interfaces__action__OrderJetcobot_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetcobot_interfaces__action__OrderJetcobot_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "jetcobot_interfaces/action/detail/order_jetcobot__struct.h"

/// Struct defined in action/OrderJetcobot in the package jetcobot_interfaces.
typedef struct jetcobot_interfaces__action__OrderJetcobot_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  jetcobot_interfaces__action__OrderJetcobot_Feedback feedback;
} jetcobot_interfaces__action__OrderJetcobot_FeedbackMessage;

// Struct for a sequence of jetcobot_interfaces__action__OrderJetcobot_FeedbackMessage.
typedef struct jetcobot_interfaces__action__OrderJetcobot_FeedbackMessage__Sequence
{
  jetcobot_interfaces__action__OrderJetcobot_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetcobot_interfaces__action__OrderJetcobot_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JETCOBOT_INTERFACES__ACTION__DETAIL__ORDER_JETCOBOT__STRUCT_H_
