// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vision_msgs:msg/Classification2D.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vision_msgs/msg/detail/classification2_d__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace vision_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Classification2D_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vision_msgs::msg::Classification2D(_init);
}

void Classification2D_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vision_msgs::msg::Classification2D *>(message_memory);
  typed_message->~Classification2D();
}

size_t size_function__Classification2D__results(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<vision_msgs::msg::ObjectHypothesis> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Classification2D__results(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<vision_msgs::msg::ObjectHypothesis> *>(untyped_member);
  return &member[index];
}

void * get_function__Classification2D__results(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<vision_msgs::msg::ObjectHypothesis> *>(untyped_member);
  return &member[index];
}

void fetch_function__Classification2D__results(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const vision_msgs::msg::ObjectHypothesis *>(
    get_const_function__Classification2D__results(untyped_member, index));
  auto & value = *reinterpret_cast<vision_msgs::msg::ObjectHypothesis *>(untyped_value);
  value = item;
}

void assign_function__Classification2D__results(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<vision_msgs::msg::ObjectHypothesis *>(
    get_function__Classification2D__results(untyped_member, index));
  const auto & value = *reinterpret_cast<const vision_msgs::msg::ObjectHypothesis *>(untyped_value);
  item = value;
}

void resize_function__Classification2D__results(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<vision_msgs::msg::ObjectHypothesis> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Classification2D_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs::msg::Classification2D, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "results",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<vision_msgs::msg::ObjectHypothesis>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs::msg::Classification2D, results),  // bytes offset in struct
    nullptr,  // default value
    size_function__Classification2D__results,  // size() function pointer
    get_const_function__Classification2D__results,  // get_const(index) function pointer
    get_function__Classification2D__results,  // get(index) function pointer
    fetch_function__Classification2D__results,  // fetch(index, &value) function pointer
    assign_function__Classification2D__results,  // assign(index, value) function pointer
    resize_function__Classification2D__results  // resize(index) function pointer
  },
  {
    "source_img",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Image>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs::msg::Classification2D, source_img),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Classification2D_message_members = {
  "vision_msgs::msg",  // message namespace
  "Classification2D",  // message name
  3,  // number of fields
  sizeof(vision_msgs::msg::Classification2D),
  Classification2D_message_member_array,  // message members
  Classification2D_init_function,  // function to initialize message memory (memory has to be allocated)
  Classification2D_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Classification2D_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Classification2D_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace vision_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vision_msgs::msg::Classification2D>()
{
  return &::vision_msgs::msg::rosidl_typesupport_introspection_cpp::Classification2D_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vision_msgs, msg, Classification2D)() {
  return &::vision_msgs::msg::rosidl_typesupport_introspection_cpp::Classification2D_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
