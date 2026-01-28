// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from go2_msgs:action/NavigateToEntrance.idl
// generated code does not contain a copyright notice

#ifndef GO2_MSGS__ACTION__DETAIL__NAVIGATE_TO_ENTRANCE__BUILDER_HPP_
#define GO2_MSGS__ACTION__DETAIL__NAVIGATE_TO_ENTRANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "go2_msgs/action/detail/navigate_to_entrance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateToEntrance_Goal_entrance_height
{
public:
  explicit Init_NavigateToEntrance_Goal_entrance_height(::go2_msgs::action::NavigateToEntrance_Goal & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::NavigateToEntrance_Goal entrance_height(::go2_msgs::action::NavigateToEntrance_Goal::_entrance_height_type arg)
  {
    msg_.entrance_height = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_Goal msg_;
};

class Init_NavigateToEntrance_Goal_entrance_width
{
public:
  explicit Init_NavigateToEntrance_Goal_entrance_width(::go2_msgs::action::NavigateToEntrance_Goal & msg)
  : msg_(msg)
  {}
  Init_NavigateToEntrance_Goal_entrance_height entrance_width(::go2_msgs::action::NavigateToEntrance_Goal::_entrance_width_type arg)
  {
    msg_.entrance_width = std::move(arg);
    return Init_NavigateToEntrance_Goal_entrance_height(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_Goal msg_;
};

class Init_NavigateToEntrance_Goal_target_position
{
public:
  Init_NavigateToEntrance_Goal_target_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToEntrance_Goal_entrance_width target_position(::go2_msgs::action::NavigateToEntrance_Goal::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_NavigateToEntrance_Goal_entrance_width(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::NavigateToEntrance_Goal>()
{
  return go2_msgs::action::builder::Init_NavigateToEntrance_Goal_target_position();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateToEntrance_Result_message
{
public:
  explicit Init_NavigateToEntrance_Result_message(::go2_msgs::action::NavigateToEntrance_Result & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::NavigateToEntrance_Result message(::go2_msgs::action::NavigateToEntrance_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_Result msg_;
};

class Init_NavigateToEntrance_Result_success
{
public:
  Init_NavigateToEntrance_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToEntrance_Result_message success(::go2_msgs::action::NavigateToEntrance_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_NavigateToEntrance_Result_message(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::NavigateToEntrance_Result>()
{
  return go2_msgs::action::builder::Init_NavigateToEntrance_Result_success();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateToEntrance_Feedback_progress_percentage
{
public:
  explicit Init_NavigateToEntrance_Feedback_progress_percentage(::go2_msgs::action::NavigateToEntrance_Feedback & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::NavigateToEntrance_Feedback progress_percentage(::go2_msgs::action::NavigateToEntrance_Feedback::_progress_percentage_type arg)
  {
    msg_.progress_percentage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_Feedback msg_;
};

class Init_NavigateToEntrance_Feedback_distance_remaining
{
public:
  explicit Init_NavigateToEntrance_Feedback_distance_remaining(::go2_msgs::action::NavigateToEntrance_Feedback & msg)
  : msg_(msg)
  {}
  Init_NavigateToEntrance_Feedback_progress_percentage distance_remaining(::go2_msgs::action::NavigateToEntrance_Feedback::_distance_remaining_type arg)
  {
    msg_.distance_remaining = std::move(arg);
    return Init_NavigateToEntrance_Feedback_progress_percentage(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_Feedback msg_;
};

class Init_NavigateToEntrance_Feedback_status
{
public:
  Init_NavigateToEntrance_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToEntrance_Feedback_distance_remaining status(::go2_msgs::action::NavigateToEntrance_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_NavigateToEntrance_Feedback_distance_remaining(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::NavigateToEntrance_Feedback>()
{
  return go2_msgs::action::builder::Init_NavigateToEntrance_Feedback_status();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateToEntrance_SendGoal_Request_goal
{
public:
  explicit Init_NavigateToEntrance_SendGoal_Request_goal(::go2_msgs::action::NavigateToEntrance_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::NavigateToEntrance_SendGoal_Request goal(::go2_msgs::action::NavigateToEntrance_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_SendGoal_Request msg_;
};

class Init_NavigateToEntrance_SendGoal_Request_goal_id
{
public:
  Init_NavigateToEntrance_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToEntrance_SendGoal_Request_goal goal_id(::go2_msgs::action::NavigateToEntrance_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavigateToEntrance_SendGoal_Request_goal(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::NavigateToEntrance_SendGoal_Request>()
{
  return go2_msgs::action::builder::Init_NavigateToEntrance_SendGoal_Request_goal_id();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateToEntrance_SendGoal_Response_stamp
{
public:
  explicit Init_NavigateToEntrance_SendGoal_Response_stamp(::go2_msgs::action::NavigateToEntrance_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::NavigateToEntrance_SendGoal_Response stamp(::go2_msgs::action::NavigateToEntrance_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_SendGoal_Response msg_;
};

class Init_NavigateToEntrance_SendGoal_Response_accepted
{
public:
  Init_NavigateToEntrance_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToEntrance_SendGoal_Response_stamp accepted(::go2_msgs::action::NavigateToEntrance_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_NavigateToEntrance_SendGoal_Response_stamp(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::NavigateToEntrance_SendGoal_Response>()
{
  return go2_msgs::action::builder::Init_NavigateToEntrance_SendGoal_Response_accepted();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateToEntrance_GetResult_Request_goal_id
{
public:
  Init_NavigateToEntrance_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::go2_msgs::action::NavigateToEntrance_GetResult_Request goal_id(::go2_msgs::action::NavigateToEntrance_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::NavigateToEntrance_GetResult_Request>()
{
  return go2_msgs::action::builder::Init_NavigateToEntrance_GetResult_Request_goal_id();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateToEntrance_GetResult_Response_result
{
public:
  explicit Init_NavigateToEntrance_GetResult_Response_result(::go2_msgs::action::NavigateToEntrance_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::NavigateToEntrance_GetResult_Response result(::go2_msgs::action::NavigateToEntrance_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_GetResult_Response msg_;
};

class Init_NavigateToEntrance_GetResult_Response_status
{
public:
  Init_NavigateToEntrance_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToEntrance_GetResult_Response_result status(::go2_msgs::action::NavigateToEntrance_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_NavigateToEntrance_GetResult_Response_result(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::NavigateToEntrance_GetResult_Response>()
{
  return go2_msgs::action::builder::Init_NavigateToEntrance_GetResult_Response_status();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateToEntrance_FeedbackMessage_feedback
{
public:
  explicit Init_NavigateToEntrance_FeedbackMessage_feedback(::go2_msgs::action::NavigateToEntrance_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::NavigateToEntrance_FeedbackMessage feedback(::go2_msgs::action::NavigateToEntrance_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_FeedbackMessage msg_;
};

class Init_NavigateToEntrance_FeedbackMessage_goal_id
{
public:
  Init_NavigateToEntrance_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToEntrance_FeedbackMessage_feedback goal_id(::go2_msgs::action::NavigateToEntrance_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavigateToEntrance_FeedbackMessage_feedback(msg_);
  }

private:
  ::go2_msgs::action::NavigateToEntrance_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::NavigateToEntrance_FeedbackMessage>()
{
  return go2_msgs::action::builder::Init_NavigateToEntrance_FeedbackMessage_goal_id();
}

}  // namespace go2_msgs

#endif  // GO2_MSGS__ACTION__DETAIL__NAVIGATE_TO_ENTRANCE__BUILDER_HPP_
