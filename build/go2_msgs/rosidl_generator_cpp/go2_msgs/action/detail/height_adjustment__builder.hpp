// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from go2_msgs:action/HeightAdjustment.idl
// generated code does not contain a copyright notice

#ifndef GO2_MSGS__ACTION__DETAIL__HEIGHT_ADJUSTMENT__BUILDER_HPP_
#define GO2_MSGS__ACTION__DETAIL__HEIGHT_ADJUSTMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "go2_msgs/action/detail/height_adjustment__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_HeightAdjustment_Goal_required_height_adjustment
{
public:
  explicit Init_HeightAdjustment_Goal_required_height_adjustment(::go2_msgs::action::HeightAdjustment_Goal & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::HeightAdjustment_Goal required_height_adjustment(::go2_msgs::action::HeightAdjustment_Goal::_required_height_adjustment_type arg)
  {
    msg_.required_height_adjustment = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_Goal msg_;
};

class Init_HeightAdjustment_Goal_passable
{
public:
  Init_HeightAdjustment_Goal_passable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeightAdjustment_Goal_required_height_adjustment passable(::go2_msgs::action::HeightAdjustment_Goal::_passable_type arg)
  {
    msg_.passable = std::move(arg);
    return Init_HeightAdjustment_Goal_required_height_adjustment(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::HeightAdjustment_Goal>()
{
  return go2_msgs::action::builder::Init_HeightAdjustment_Goal_passable();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_HeightAdjustment_Result_message
{
public:
  explicit Init_HeightAdjustment_Result_message(::go2_msgs::action::HeightAdjustment_Result & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::HeightAdjustment_Result message(::go2_msgs::action::HeightAdjustment_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_Result msg_;
};

class Init_HeightAdjustment_Result_success
{
public:
  Init_HeightAdjustment_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeightAdjustment_Result_message success(::go2_msgs::action::HeightAdjustment_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_HeightAdjustment_Result_message(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::HeightAdjustment_Result>()
{
  return go2_msgs::action::builder::Init_HeightAdjustment_Result_success();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_HeightAdjustment_Feedback_status
{
public:
  Init_HeightAdjustment_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::go2_msgs::action::HeightAdjustment_Feedback status(::go2_msgs::action::HeightAdjustment_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::HeightAdjustment_Feedback>()
{
  return go2_msgs::action::builder::Init_HeightAdjustment_Feedback_status();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_HeightAdjustment_SendGoal_Request_goal
{
public:
  explicit Init_HeightAdjustment_SendGoal_Request_goal(::go2_msgs::action::HeightAdjustment_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::HeightAdjustment_SendGoal_Request goal(::go2_msgs::action::HeightAdjustment_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_SendGoal_Request msg_;
};

class Init_HeightAdjustment_SendGoal_Request_goal_id
{
public:
  Init_HeightAdjustment_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeightAdjustment_SendGoal_Request_goal goal_id(::go2_msgs::action::HeightAdjustment_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_HeightAdjustment_SendGoal_Request_goal(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::HeightAdjustment_SendGoal_Request>()
{
  return go2_msgs::action::builder::Init_HeightAdjustment_SendGoal_Request_goal_id();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_HeightAdjustment_SendGoal_Response_stamp
{
public:
  explicit Init_HeightAdjustment_SendGoal_Response_stamp(::go2_msgs::action::HeightAdjustment_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::HeightAdjustment_SendGoal_Response stamp(::go2_msgs::action::HeightAdjustment_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_SendGoal_Response msg_;
};

class Init_HeightAdjustment_SendGoal_Response_accepted
{
public:
  Init_HeightAdjustment_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeightAdjustment_SendGoal_Response_stamp accepted(::go2_msgs::action::HeightAdjustment_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_HeightAdjustment_SendGoal_Response_stamp(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::HeightAdjustment_SendGoal_Response>()
{
  return go2_msgs::action::builder::Init_HeightAdjustment_SendGoal_Response_accepted();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_HeightAdjustment_GetResult_Request_goal_id
{
public:
  Init_HeightAdjustment_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::go2_msgs::action::HeightAdjustment_GetResult_Request goal_id(::go2_msgs::action::HeightAdjustment_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::HeightAdjustment_GetResult_Request>()
{
  return go2_msgs::action::builder::Init_HeightAdjustment_GetResult_Request_goal_id();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_HeightAdjustment_GetResult_Response_result
{
public:
  explicit Init_HeightAdjustment_GetResult_Response_result(::go2_msgs::action::HeightAdjustment_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::HeightAdjustment_GetResult_Response result(::go2_msgs::action::HeightAdjustment_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_GetResult_Response msg_;
};

class Init_HeightAdjustment_GetResult_Response_status
{
public:
  Init_HeightAdjustment_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeightAdjustment_GetResult_Response_result status(::go2_msgs::action::HeightAdjustment_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_HeightAdjustment_GetResult_Response_result(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::HeightAdjustment_GetResult_Response>()
{
  return go2_msgs::action::builder::Init_HeightAdjustment_GetResult_Response_status();
}

}  // namespace go2_msgs


namespace go2_msgs
{

namespace action
{

namespace builder
{

class Init_HeightAdjustment_FeedbackMessage_feedback
{
public:
  explicit Init_HeightAdjustment_FeedbackMessage_feedback(::go2_msgs::action::HeightAdjustment_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::go2_msgs::action::HeightAdjustment_FeedbackMessage feedback(::go2_msgs::action::HeightAdjustment_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_FeedbackMessage msg_;
};

class Init_HeightAdjustment_FeedbackMessage_goal_id
{
public:
  Init_HeightAdjustment_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeightAdjustment_FeedbackMessage_feedback goal_id(::go2_msgs::action::HeightAdjustment_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_HeightAdjustment_FeedbackMessage_feedback(msg_);
  }

private:
  ::go2_msgs::action::HeightAdjustment_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::action::HeightAdjustment_FeedbackMessage>()
{
  return go2_msgs::action::builder::Init_HeightAdjustment_FeedbackMessage_goal_id();
}

}  // namespace go2_msgs

#endif  // GO2_MSGS__ACTION__DETAIL__HEIGHT_ADJUSTMENT__BUILDER_HPP_
