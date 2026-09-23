# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rb_msgs:msg/DetectionArray.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DetectionArray(type):
    """Metaclass of message 'DetectionArray'."""

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
            module = import_type_support('rb_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rb_msgs.msg.DetectionArray')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__detection_array
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__detection_array
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__detection_array
            cls._TYPE_SUPPORT = module.type_support_msg__msg__detection_array
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__detection_array

            from rb_msgs.msg import Detection
            if Detection.__class__._TYPE_SUPPORT is None:
                Detection.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DetectionArray(metaclass=Metaclass_DetectionArray):
    """Message class 'DetectionArray'."""

    __slots__ = [
        '_header',
        '_frame_id_seq',
        '_fx',
        '_fy',
        '_cx_cam',
        '_cy_cam',
        '_img_width',
        '_img_height',
        '_detections',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'frame_id_seq': 'uint32',
        'fx': 'float',
        'fy': 'float',
        'cx_cam': 'float',
        'cy_cam': 'float',
        'img_width': 'uint32',
        'img_height': 'uint32',
        'detections': 'sequence<rb_msgs/Detection>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rb_msgs', 'msg'], 'Detection')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.frame_id_seq = kwargs.get('frame_id_seq', int())
        self.fx = kwargs.get('fx', float())
        self.fy = kwargs.get('fy', float())
        self.cx_cam = kwargs.get('cx_cam', float())
        self.cy_cam = kwargs.get('cy_cam', float())
        self.img_width = kwargs.get('img_width', int())
        self.img_height = kwargs.get('img_height', int())
        self.detections = kwargs.get('detections', [])

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
        if self.header != other.header:
            return False
        if self.frame_id_seq != other.frame_id_seq:
            return False
        if self.fx != other.fx:
            return False
        if self.fy != other.fy:
            return False
        if self.cx_cam != other.cx_cam:
            return False
        if self.cy_cam != other.cy_cam:
            return False
        if self.img_width != other.img_width:
            return False
        if self.img_height != other.img_height:
            return False
        if self.detections != other.detections:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def frame_id_seq(self):
        """Message field 'frame_id_seq'."""
        return self._frame_id_seq

    @frame_id_seq.setter
    def frame_id_seq(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'frame_id_seq' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'frame_id_seq' field must be an unsigned integer in [0, 4294967295]"
        self._frame_id_seq = value

    @builtins.property
    def fx(self):
        """Message field 'fx'."""
        return self._fx

    @fx.setter
    def fx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fx = value

    @builtins.property
    def fy(self):
        """Message field 'fy'."""
        return self._fy

    @fy.setter
    def fy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fy = value

    @builtins.property
    def cx_cam(self):
        """Message field 'cx_cam'."""
        return self._cx_cam

    @cx_cam.setter
    def cx_cam(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cx_cam' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cx_cam' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cx_cam = value

    @builtins.property
    def cy_cam(self):
        """Message field 'cy_cam'."""
        return self._cy_cam

    @cy_cam.setter
    def cy_cam(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cy_cam' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cy_cam' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cy_cam = value

    @builtins.property
    def img_width(self):
        """Message field 'img_width'."""
        return self._img_width

    @img_width.setter
    def img_width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'img_width' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'img_width' field must be an unsigned integer in [0, 4294967295]"
        self._img_width = value

    @builtins.property
    def img_height(self):
        """Message field 'img_height'."""
        return self._img_height

    @img_height.setter
    def img_height(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'img_height' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'img_height' field must be an unsigned integer in [0, 4294967295]"
        self._img_height = value

    @builtins.property
    def detections(self):
        """Message field 'detections'."""
        return self._detections

    @detections.setter
    def detections(self, value):
        if __debug__:
            from rb_msgs.msg import Detection
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Detection) for v in value) and
                 True), \
                "The 'detections' field must be a set or sequence and each value of type 'Detection'"
        self._detections = value
