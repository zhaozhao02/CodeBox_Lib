# generated from rosidl_generator_py/resource/_idl.py.em
# with input from aruco_msgs:msg/ArucoPose.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ArucoPose(type):
    """Metaclass of message 'ArucoPose'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('aruco_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'aruco_msgs.msg.ArucoPose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__aruco_pose
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__aruco_pose
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__aruco_pose
            cls._TYPE_SUPPORT = module.type_support_msg__msg__aruco_pose
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__aruco_pose

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ArucoPose(metaclass=Metaclass_ArucoPose):
    """Message class 'ArucoPose'."""

    __slots__ = [
        '_mark_id',
        '_px',
        '_py',
        '_pz',
        '_ox',
        '_oy',
        '_oz',
        '_ow',
    ]

    _fields_and_field_types = {
        'mark_id': 'int32',
        'px': 'double',
        'py': 'double',
        'pz': 'double',
        'ox': 'double',
        'oy': 'double',
        'oz': 'double',
        'ow': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mark_id = kwargs.get('mark_id', int())
        self.px = kwargs.get('px', float())
        self.py = kwargs.get('py', float())
        self.pz = kwargs.get('pz', float())
        self.ox = kwargs.get('ox', float())
        self.oy = kwargs.get('oy', float())
        self.oz = kwargs.get('oz', float())
        self.ow = kwargs.get('ow', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.mark_id != other.mark_id:
            return False
        if self.px != other.px:
            return False
        if self.py != other.py:
            return False
        if self.pz != other.pz:
            return False
        if self.ox != other.ox:
            return False
        if self.oy != other.oy:
            return False
        if self.oz != other.oz:
            return False
        if self.ow != other.ow:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def mark_id(self):
        """Message field 'mark_id'."""
        return self._mark_id

    @mark_id.setter
    def mark_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mark_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'mark_id' field must be an integer in [-2147483648, 2147483647]"
        self._mark_id = value

    @builtins.property
    def px(self):
        """Message field 'px'."""
        return self._px

    @px.setter
    def px(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'px' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'px' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._px = value

    @builtins.property
    def py(self):
        """Message field 'py'."""
        return self._py

    @py.setter
    def py(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'py' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'py' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._py = value

    @builtins.property
    def pz(self):
        """Message field 'pz'."""
        return self._pz

    @pz.setter
    def pz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pz' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pz' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pz = value

    @builtins.property
    def ox(self):
        """Message field 'ox'."""
        return self._ox

    @ox.setter
    def ox(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ox' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ox' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ox = value

    @builtins.property
    def oy(self):
        """Message field 'oy'."""
        return self._oy

    @oy.setter
    def oy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'oy' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'oy' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._oy = value

    @builtins.property
    def oz(self):
        """Message field 'oz'."""
        return self._oz

    @oz.setter
    def oz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'oz' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'oz' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._oz = value

    @builtins.property
    def ow(self):
        """Message field 'ow'."""
        return self._ow

    @ow.setter
    def ow(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ow' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ow' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ow = value
