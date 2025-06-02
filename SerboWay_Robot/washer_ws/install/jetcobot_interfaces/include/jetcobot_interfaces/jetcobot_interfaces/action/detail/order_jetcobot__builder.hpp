// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jetcobot_interfaces:action/OrderJetcobot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "jetcobot_interfaces/action/order_jetcobot.hpp"


#ifndef JETCOBOT_INTERFACES__ACTION__DETAIL__ORDER_JETCOBOT__BUILDER_HPP_
#define JETCOBOT_INTERFACES__ACTION__DETAIL__ORDER_JETCOBOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jetcobot_interfaces/action/detail/order_jetcobot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jetcobot_interfaces
{

namespace action
{

namespace builder
{

class Init_OrderJetcobot_Goal_order
{
public:
  Init_OrderJetcobot_Goal_order()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jetcobot_interfaces::action::OrderJetcobot_Goal order(::jetcobot_interfaces::action::OrderJetcobot_Goal::_order_type arg)
  {
    msg_.order = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetcobot_interfaces::action::OrderJetcobot_Goal>()
{
  return jetcobot_interfaces::action::builder::Init_OrderJetcobot_Goal_order();
}

}  // namespace jetcobot_interfaces


namespace jetcobot_interfaces
{

namespace action
{

namespace builder
{

class Init_OrderJetcobot_Result_status
{
public:
  Init_OrderJetcobot_Result_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jetcobot_interfaces::action::OrderJetcobot_Result status(::jetcobot_interfaces::action::OrderJetcobot_Result::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetcobot_interfaces::action::OrderJetcobot_Result>()
{
  return jetcobot_interfaces::action::builder::Init_OrderJetcobot_Result_status();
}

}  // namespace jetcobot_interfaces


namespace jetcobot_interfaces
{

namespace action
{

namespace builder
{

class Init_OrderJetcobot_Feedback_progress
{
public:
  Init_OrderJetcobot_Feedback_progress()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jetcobot_interfaces::action::OrderJetcobot_Feedback progress(::jetcobot_interfaces::action::OrderJetcobot_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetcobot_interfaces::action::OrderJetcobot_Feedback>()
{
  return jetcobot_interfaces::action::builder::Init_OrderJetcobot_Feedback_progress();
}

}  // namespace jetcobot_interfaces


namespace jetcobot_interfaces
{

namespace action
{

namespace builder
{

class Init_OrderJetcobot_SendGoal_Request_goal
{
public:
  explicit Init_OrderJetcobot_SendGoal_Request_goal(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Request goal(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Request msg_;
};

class Init_OrderJetcobot_SendGoal_Request_goal_id
{
public:
  Init_OrderJetcobot_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderJetcobot_SendGoal_Request_goal goal_id(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_OrderJetcobot_SendGoal_Request_goal(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Request>()
{
  return jetcobot_interfaces::action::builder::Init_OrderJetcobot_SendGoal_Request_goal_id();
}

}  // namespace jetcobot_interfaces


namespace jetcobot_interfaces
{

namespace action
{

namespace builder
{

class Init_OrderJetcobot_SendGoal_Response_stamp
{
public:
  explicit Init_OrderJetcobot_SendGoal_Response_stamp(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Response stamp(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Response msg_;
};

class Init_OrderJetcobot_SendGoal_Response_accepted
{
public:
  Init_OrderJetcobot_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderJetcobot_SendGoal_Response_stamp accepted(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_OrderJetcobot_SendGoal_Response_stamp(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Response>()
{
  return jetcobot_interfaces::action::builder::Init_OrderJetcobot_SendGoal_Response_accepted();
}

}  // namespace jetcobot_interfaces


namespace jetcobot_interfaces
{

namespace action
{

namespace builder
{

class Init_OrderJetcobot_SendGoal_Event_response
{
public:
  explicit Init_OrderJetcobot_SendGoal_Event_response(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Event response(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Event msg_;
};

class Init_OrderJetcobot_SendGoal_Event_request
{
public:
  explicit Init_OrderJetcobot_SendGoal_Event_request(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_OrderJetcobot_SendGoal_Event_response request(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_OrderJetcobot_SendGoal_Event_response(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Event msg_;
};

class Init_OrderJetcobot_SendGoal_Event_info
{
public:
  Init_OrderJetcobot_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderJetcobot_SendGoal_Event_request info(::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_OrderJetcobot_SendGoal_Event_request(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetcobot_interfaces::action::OrderJetcobot_SendGoal_Event>()
{
  return jetcobot_interfaces::action::builder::Init_OrderJetcobot_SendGoal_Event_info();
}

}  // namespace jetcobot_interfaces


namespace jetcobot_interfaces
{

namespace action
{

namespace builder
{

class Init_OrderJetcobot_GetResult_Request_goal_id
{
public:
  Init_OrderJetcobot_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jetcobot_interfaces::action::OrderJetcobot_GetResult_Request goal_id(::jetcobot_interfaces::action::OrderJetcobot_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetcobot_interfaces::action::OrderJetcobot_GetResult_Request>()
{
  return jetcobot_interfaces::action::builder::Init_OrderJetcobot_GetResult_Request_goal_id();
}

}  // namespace jetcobot_interfaces


namespace jetcobot_interfaces
{

namespace action
{

namespace builder
{

class Init_OrderJetcobot_GetResult_Response_result
{
public:
  explicit Init_OrderJetcobot_GetResult_Response_result(::jetcobot_interfaces::action::OrderJetcobot_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::jetcobot_interfaces::action::OrderJetcobot_GetResult_Response result(::jetcobot_interfaces::action::OrderJetcobot_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_GetResult_Response msg_;
};

class Init_OrderJetcobot_GetResult_Response_status
{
public:
  Init_OrderJetcobot_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderJetcobot_GetResult_Response_result status(::jetcobot_interfaces::action::OrderJetcobot_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_OrderJetcobot_GetResult_Response_result(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetcobot_interfaces::action::OrderJetcobot_GetResult_Response>()
{
  return jetcobot_interfaces::action::builder::Init_OrderJetcobot_GetResult_Response_status();
}

}  // namespace jetcobot_interfaces


namespace jetcobot_interfaces
{

namespace action
{

namespace builder
{

class Init_OrderJetcobot_GetResult_Event_response
{
public:
  explicit Init_OrderJetcobot_GetResult_Event_response(::jetcobot_interfaces::action::OrderJetcobot_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::jetcobot_interfaces::action::OrderJetcobot_GetResult_Event response(::jetcobot_interfaces::action::OrderJetcobot_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_GetResult_Event msg_;
};

class Init_OrderJetcobot_GetResult_Event_request
{
public:
  explicit Init_OrderJetcobot_GetResult_Event_request(::jetcobot_interfaces::action::OrderJetcobot_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_OrderJetcobot_GetResult_Event_response request(::jetcobot_interfaces::action::OrderJetcobot_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_OrderJetcobot_GetResult_Event_response(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_GetResult_Event msg_;
};

class Init_OrderJetcobot_GetResult_Event_info
{
public:
  Init_OrderJetcobot_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderJetcobot_GetResult_Event_request info(::jetcobot_interfaces::action::OrderJetcobot_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_OrderJetcobot_GetResult_Event_request(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetcobot_interfaces::action::OrderJetcobot_GetResult_Event>()
{
  return jetcobot_interfaces::action::builder::Init_OrderJetcobot_GetResult_Event_info();
}

}  // namespace jetcobot_interfaces


namespace jetcobot_interfaces
{

namespace action
{

namespace builder
{

class Init_OrderJetcobot_FeedbackMessage_feedback
{
public:
  explicit Init_OrderJetcobot_FeedbackMessage_feedback(::jetcobot_interfaces::action::OrderJetcobot_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::jetcobot_interfaces::action::OrderJetcobot_FeedbackMessage feedback(::jetcobot_interfaces::action::OrderJetcobot_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_FeedbackMessage msg_;
};

class Init_OrderJetcobot_FeedbackMessage_goal_id
{
public:
  Init_OrderJetcobot_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderJetcobot_FeedbackMessage_feedback goal_id(::jetcobot_interfaces::action::OrderJetcobot_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_OrderJetcobot_FeedbackMessage_feedback(msg_);
  }

private:
  ::jetcobot_interfaces::action::OrderJetcobot_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetcobot_interfaces::action::OrderJetcobot_FeedbackMessage>()
{
  return jetcobot_interfaces::action::builder::Init_OrderJetcobot_FeedbackMessage_goal_id();
}

}  // namespace jetcobot_interfaces

#endif  // JETCOBOT_INTERFACES__ACTION__DETAIL__ORDER_JETCOBOT__BUILDER_HPP_
