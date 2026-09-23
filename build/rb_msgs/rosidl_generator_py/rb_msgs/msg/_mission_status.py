# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rb_msgs:msg/MissionStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MissionStatus(type):
    """Metaclass of message 'MissionStatus'."""

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
                'rb_msgs.msg.MissionStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mission_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mission_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mission_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mission_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mission_status

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


class MissionStatus(metaclass=Metaclass_MissionStatus):
    """Message class 'MissionStatus'."""

    __slots__ = [
        '_header',
        '_mission',
        '_phase',
        '_detail',
        '_elapsed_s',
        '_score_estimate',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'mission': 'string',
        'phase': 'string',
        'detail': 'string',
        'elapsed_s': 'float',
        'score_estimate': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.mission = kwargs.get('mission', str())
        self.phase = kwargs.get('phase', str())
        self.detail = kwargs.get('detail', str())
        self.elapsed_s = kwargs.get('elapsed_s', float())
        self.score_estimate = kwargs.get('score_estimate', int())

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
        if self.mission != other.mission:
            return False
        if self.phase != other.phase:
            return False
        if self.detail != other.detail:
            return False
        if self.elapsed_s != other.elapsed_s:
            return False
        if self.score_estimate != other.score_estimate:
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
    def mission(self):
        """Message field 'mission'."""
        return self._mission

    @mission.setter
    def mission(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'mission' field must be of type 'str'"
        self._mission = value

    @builtins.property
    def phase(self):
        """Message field 'phase'."""
        return self._phase

    @phase.setter
    def phase(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'phase' field must be of type 'str'"
        self._phase = value

    @builtins.property
    def detail(self):
        """Message field 'detail'."""
        return self._detail

    @detail.setter
    def detail(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'detail' field must be of type 'str'"
        self._detail = value

    @builtins.property
    def elapsed_s(self):
        """Message field 'elapsed_s'."""
        return self._elapsed_s

    @elapsed_s.setter
    def elapsed_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'elapsed_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'elapsed_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._elapsed_s = value

    @builtins.property
    def score_estimate(self):
        """Message field 'score_estimate'."""
        return self._score_estimate

    @score_estimate.setter
    def score_estimate(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'score_estimate' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'score_estimate' field must be an integer in [-2147483648, 2147483647]"
        self._score_estimate = value
