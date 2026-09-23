# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rb_msgs:msg/RobotState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobotState(type):
    """Metaclass of message 'RobotState'."""

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
                'rb_msgs.msg.RobotState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot_state

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


class RobotState(metaclass=Metaclass_RobotState):
    """Message class 'RobotState'."""

    __slots__ = [
        '_header',
        '_phase',
        '_has_ball',
        '_ball_count',
        '_ball_type',
        '_in_pass_zone',
        '_in_shoot_zone_outside',
        '_estop',
        '_localization_ok',
        '_launcher_ok',
        '_perception_ok',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'phase': 'string',
        'has_ball': 'boolean',
        'ball_count': 'uint8',
        'ball_type': 'string',
        'in_pass_zone': 'boolean',
        'in_shoot_zone_outside': 'boolean',
        'estop': 'boolean',
        'localization_ok': 'boolean',
        'launcher_ok': 'boolean',
        'perception_ok': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.phase = kwargs.get('phase', str())
        self.has_ball = kwargs.get('has_ball', bool())
        self.ball_count = kwargs.get('ball_count', int())
        self.ball_type = kwargs.get('ball_type', str())
        self.in_pass_zone = kwargs.get('in_pass_zone', bool())
        self.in_shoot_zone_outside = kwargs.get('in_shoot_zone_outside', bool())
        self.estop = kwargs.get('estop', bool())
        self.localization_ok = kwargs.get('localization_ok', bool())
        self.launcher_ok = kwargs.get('launcher_ok', bool())
        self.perception_ok = kwargs.get('perception_ok', bool())

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
        if self.phase != other.phase:
            return False
        if self.has_ball != other.has_ball:
            return False
        if self.ball_count != other.ball_count:
            return False
        if self.ball_type != other.ball_type:
            return False
        if self.in_pass_zone != other.in_pass_zone:
            return False
        if self.in_shoot_zone_outside != other.in_shoot_zone_outside:
            return False
        if self.estop != other.estop:
            return False
        if self.localization_ok != other.localization_ok:
            return False
        if self.launcher_ok != other.launcher_ok:
            return False
        if self.perception_ok != other.perception_ok:
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
    def has_ball(self):
        """Message field 'has_ball'."""
        return self._has_ball

    @has_ball.setter
    def has_ball(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'has_ball' field must be of type 'bool'"
        self._has_ball = value

    @builtins.property
    def ball_count(self):
        """Message field 'ball_count'."""
        return self._ball_count

    @ball_count.setter
    def ball_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ball_count' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'ball_count' field must be an unsigned integer in [0, 255]"
        self._ball_count = value

    @builtins.property
    def ball_type(self):
        """Message field 'ball_type'."""
        return self._ball_type

    @ball_type.setter
    def ball_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'ball_type' field must be of type 'str'"
        self._ball_type = value

    @builtins.property
    def in_pass_zone(self):
        """Message field 'in_pass_zone'."""
        return self._in_pass_zone

    @in_pass_zone.setter
    def in_pass_zone(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'in_pass_zone' field must be of type 'bool'"
        self._in_pass_zone = value

    @builtins.property
    def in_shoot_zone_outside(self):
        """Message field 'in_shoot_zone_outside'."""
        return self._in_shoot_zone_outside

    @in_shoot_zone_outside.setter
    def in_shoot_zone_outside(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'in_shoot_zone_outside' field must be of type 'bool'"
        self._in_shoot_zone_outside = value

    @builtins.property
    def estop(self):
        """Message field 'estop'."""
        return self._estop

    @estop.setter
    def estop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'estop' field must be of type 'bool'"
        self._estop = value

    @builtins.property
    def localization_ok(self):
        """Message field 'localization_ok'."""
        return self._localization_ok

    @localization_ok.setter
    def localization_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'localization_ok' field must be of type 'bool'"
        self._localization_ok = value

    @builtins.property
    def launcher_ok(self):
        """Message field 'launcher_ok'."""
        return self._launcher_ok

    @launcher_ok.setter
    def launcher_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'launcher_ok' field must be of type 'bool'"
        self._launcher_ok = value

    @builtins.property
    def perception_ok(self):
        """Message field 'perception_ok'."""
        return self._perception_ok

    @perception_ok.setter
    def perception_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'perception_ok' field must be of type 'bool'"
        self._perception_ok = value
