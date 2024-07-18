// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vineslam_ros:srv/SaveMap.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vineslam_ros/srv/detail/save_map__rosidl_typesupport_introspection_c.h"
#include "vineslam_ros/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vineslam_ros/srv/detail/save_map__functions.h"
#include "vineslam_ros/srv/detail/save_map__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vineslam_ros__srv__SaveMap_Request__init(message_memory);
}

void SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_fini_function(void * message_memory)
{
  vineslam_ros__srv__SaveMap_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vineslam_ros__srv__SaveMap_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_message_members = {
  "vineslam_ros__srv",  // message namespace
  "SaveMap_Request",  // message name
  1,  // number of fields
  sizeof(vineslam_ros__srv__SaveMap_Request),
  SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_message_member_array,  // message members
  SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_message_type_support_handle = {
  0,
  &SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vineslam_ros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vineslam_ros, srv, SaveMap_Request)() {
  if (!SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_message_type_support_handle.typesupport_identifier) {
    SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SaveMap_Request__rosidl_typesupport_introspection_c__SaveMap_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "vineslam_ros/srv/detail/save_map__rosidl_typesupport_introspection_c.h"
// already included above
// #include "vineslam_ros/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "vineslam_ros/srv/detail/save_map__functions.h"
// already included above
// #include "vineslam_ros/srv/detail/save_map__struct.h"


// Include directives for member types
// Member `save_map`
#include "std_msgs/msg/bool.h"
// Member `save_map`
#include "std_msgs/msg/detail/bool__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vineslam_ros__srv__SaveMap_Response__init(message_memory);
}

void SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_fini_function(void * message_memory)
{
  vineslam_ros__srv__SaveMap_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_message_member_array[1] = {
  {
    "save_map",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vineslam_ros__srv__SaveMap_Response, save_map),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_message_members = {
  "vineslam_ros__srv",  // message namespace
  "SaveMap_Response",  // message name
  1,  // number of fields
  sizeof(vineslam_ros__srv__SaveMap_Response),
  SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_message_member_array,  // message members
  SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_message_type_support_handle = {
  0,
  &SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vineslam_ros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vineslam_ros, srv, SaveMap_Response)() {
  SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Bool)();
  if (!SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_message_type_support_handle.typesupport_identifier) {
    SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SaveMap_Response__rosidl_typesupport_introspection_c__SaveMap_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "vineslam_ros/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "vineslam_ros/srv/detail/save_map__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers vineslam_ros__srv__detail__save_map__rosidl_typesupport_introspection_c__SaveMap_service_members = {
  "vineslam_ros__srv",  // service namespace
  "SaveMap",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // vineslam_ros__srv__detail__save_map__rosidl_typesupport_introspection_c__SaveMap_Request_message_type_support_handle,
  NULL  // response message
  // vineslam_ros__srv__detail__save_map__rosidl_typesupport_introspection_c__SaveMap_Response_message_type_support_handle
};

static rosidl_service_type_support_t vineslam_ros__srv__detail__save_map__rosidl_typesupport_introspection_c__SaveMap_service_type_support_handle = {
  0,
  &vineslam_ros__srv__detail__save_map__rosidl_typesupport_introspection_c__SaveMap_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vineslam_ros, srv, SaveMap_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vineslam_ros, srv, SaveMap_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vineslam_ros
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vineslam_ros, srv, SaveMap)() {
  if (!vineslam_ros__srv__detail__save_map__rosidl_typesupport_introspection_c__SaveMap_service_type_support_handle.typesupport_identifier) {
    vineslam_ros__srv__detail__save_map__rosidl_typesupport_introspection_c__SaveMap_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)vineslam_ros__srv__detail__save_map__rosidl_typesupport_introspection_c__SaveMap_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vineslam_ros, srv, SaveMap_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vineslam_ros, srv, SaveMap_Response)()->data;
  }

  return &vineslam_ros__srv__detail__save_map__rosidl_typesupport_introspection_c__SaveMap_service_type_support_handle;
}
