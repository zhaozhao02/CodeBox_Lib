// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from aruco_msgs:msg/ArucoPose.idl
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
#include "aruco_msgs/msg/detail/aruco_pose__struct.h"
#include "aruco_msgs/msg/detail/aruco_pose__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool aruco_msgs__msg__aruco_pose__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[37];
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
    assert(strncmp("aruco_msgs.msg._aruco_pose.ArucoPose", full_classname_dest, 36) == 0);
  }
  aruco_msgs__msg__ArucoPose * ros_message = _ros_message;
  {  // mark_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "mark_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mark_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // px
    PyObject * field = PyObject_GetAttrString(_pymsg, "px");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->px = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // py
    PyObject * field = PyObject_GetAttrString(_pymsg, "py");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->py = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pz
    PyObject * field = PyObject_GetAttrString(_pymsg, "pz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pz = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ox
    PyObject * field = PyObject_GetAttrString(_pymsg, "ox");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ox = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // oy
    PyObject * field = PyObject_GetAttrString(_pymsg, "oy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->oy = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // oz
    PyObject * field = PyObject_GetAttrString(_pymsg, "oz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->oz = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ow
    PyObject * field = PyObject_GetAttrString(_pymsg, "ow");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ow = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * aruco_msgs__msg__aruco_pose__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ArucoPose */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("aruco_msgs.msg._aruco_pose");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ArucoPose");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  aruco_msgs__msg__ArucoPose * ros_message = (aruco_msgs__msg__ArucoPose *)raw_ros_message;
  {  // mark_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->mark_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mark_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // px
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->px);
    {
      int rc = PyObject_SetAttrString(_pymessage, "px", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // py
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->py);
    {
      int rc = PyObject_SetAttrString(_pymessage, "py", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ox
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ox);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ox", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // oy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->oy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "oy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // oz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->oz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "oz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ow
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
