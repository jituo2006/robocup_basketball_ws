// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rb_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "rb_msgs/msg/detail/robot_state__struct.h"
#include "rb_msgs/msg/detail/robot_state__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rb_msgs__msg__robot_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[36];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("rb_msgs.msg._robot_state.RobotState", full_classname_dest, 35) == 0);
  }
  rb_msgs__msg__RobotState * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // phase
    PyObject * field = PyObject_GetAttrString(_pymsg, "phase");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->phase, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // has_ball
    PyObject * field = PyObject_GetAttrString(_pymsg, "has_ball");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->has_ball = (Py_True == field);
    Py_DECREF(field);
  }
  {  // ball_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "ball_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ball_count = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // ball_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "ball_type");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->ball_type, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // in_pass_zone
    PyObject * field = PyObject_GetAttrString(_pymsg, "in_pass_zone");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->in_pass_zone = (Py_True == field);
    Py_DECREF(field);
  }
  {  // in_shoot_zone_outside
    PyObject * field = PyObject_GetAttrString(_pymsg, "in_shoot_zone_outside");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->in_shoot_zone_outside = (Py_True == field);
    Py_DECREF(field);
  }
  {  // estop
    PyObject * field = PyObject_GetAttrString(_pymsg, "estop");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->estop = (Py_True == field);
    Py_DECREF(field);
  }
  {  // localization_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "localization_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->localization_ok = (Py_True == field);
    Py_DECREF(field);
  }
  {  // launcher_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "launcher_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->launcher_ok = (Py_True == field);
    Py_DECREF(field);
  }
  {  // perception_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "perception_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->perception_ok = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rb_msgs__msg__robot_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RobotState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rb_msgs.msg._robot_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RobotState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rb_msgs__msg__RobotState * ros_message = (rb_msgs__msg__RobotState *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // phase
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->phase.data,
      strlen(ros_message->phase.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "phase", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // has_ball
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->has_ball ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "has_ball", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ball_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->ball_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ball_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ball_type
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->ball_type.data,
      strlen(ros_message->ball_type.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "ball_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // in_pass_zone
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->in_pass_zone ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "in_pass_zone", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // in_shoot_zone_outside
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->in_shoot_zone_outside ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "in_shoot_zone_outside", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // estop
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->estop ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "estop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // localization_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->localization_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "localization_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // launcher_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->launcher_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "launcher_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // perception_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->perception_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "perception_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
