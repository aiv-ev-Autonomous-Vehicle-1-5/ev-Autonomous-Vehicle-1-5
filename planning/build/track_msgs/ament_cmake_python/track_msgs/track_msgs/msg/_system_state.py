# generated from rosidl_generator_py/resource/_idl.py.em
# with input from track_msgs:msg/SystemState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SystemState(type):
    """Metaclass of message 'SystemState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'OFFLINE': 0,
        'MANUAL': 1,
        'AUTO_STANDBY': 2,
        'AUTO_ACTIVE': 3,
        'AUTO_HOLD': 4,
        'INFEASIBLE': 5,
        'EMERGENCY_STOP': 6,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('track_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'track_msgs.msg.SystemState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__system_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__system_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__system_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__system_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__system_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'OFFLINE': cls.__constants['OFFLINE'],
            'MANUAL': cls.__constants['MANUAL'],
            'AUTO_STANDBY': cls.__constants['AUTO_STANDBY'],
            'AUTO_ACTIVE': cls.__constants['AUTO_ACTIVE'],
            'AUTO_HOLD': cls.__constants['AUTO_HOLD'],
            'INFEASIBLE': cls.__constants['INFEASIBLE'],
            'EMERGENCY_STOP': cls.__constants['EMERGENCY_STOP'],
        }

    @property
    def OFFLINE(self):
        """Message constant 'OFFLINE'."""
        return Metaclass_SystemState.__constants['OFFLINE']

    @property
    def MANUAL(self):
        """Message constant 'MANUAL'."""
        return Metaclass_SystemState.__constants['MANUAL']

    @property
    def AUTO_STANDBY(self):
        """Message constant 'AUTO_STANDBY'."""
        return Metaclass_SystemState.__constants['AUTO_STANDBY']

    @property
    def AUTO_ACTIVE(self):
        """Message constant 'AUTO_ACTIVE'."""
        return Metaclass_SystemState.__constants['AUTO_ACTIVE']

    @property
    def AUTO_HOLD(self):
        """Message constant 'AUTO_HOLD'."""
        return Metaclass_SystemState.__constants['AUTO_HOLD']

    @property
    def INFEASIBLE(self):
        """Message constant 'INFEASIBLE'."""
        return Metaclass_SystemState.__constants['INFEASIBLE']

    @property
    def EMERGENCY_STOP(self):
        """Message constant 'EMERGENCY_STOP'."""
        return Metaclass_SystemState.__constants['EMERGENCY_STOP']


class SystemState(metaclass=Metaclass_SystemState):
    """
    Message class 'SystemState'.

    Constants:
      OFFLINE
      MANUAL
      AUTO_STANDBY
      AUTO_ACTIVE
      AUTO_HOLD
      INFEASIBLE
      EMERGENCY_STOP
    """

    __slots__ = [
        '_header',
        '_state',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'state': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.state = kwargs.get('state', int())

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
        if self.state != other.state:
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
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'state' field must be an unsigned integer in [0, 255]"
        self._state = value
