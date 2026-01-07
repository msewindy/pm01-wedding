# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interface_protocol:msg/GamepadKeys.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'digital_states'
# Member 'analog_states'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GamepadKeys(type):
    """Metaclass of message 'GamepadKeys'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'LB': 0,
        'RB': 1,
        'A': 2,
        'B': 3,
        'X': 4,
        'Y': 5,
        'BACK': 6,
        'START': 7,
        'CROSS_X_UP': 8,
        'CROSS_X_DOWN': 9,
        'CROSS_Y_LEFT': 10,
        'CROSS_Y_RIGHT': 11,
        'LT': 0,
        'RT': 1,
        'LEFT_STICK_X': 2,
        'LEFT_STICK_Y': 3,
        'RIGHT_STICK_X': 4,
        'RIGHT_STICK_Y': 5,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interface_protocol')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interface_protocol.msg.GamepadKeys')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gamepad_keys
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gamepad_keys
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gamepad_keys
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gamepad_keys
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gamepad_keys

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'LB': cls.__constants['LB'],
            'RB': cls.__constants['RB'],
            'A': cls.__constants['A'],
            'B': cls.__constants['B'],
            'X': cls.__constants['X'],
            'Y': cls.__constants['Y'],
            'BACK': cls.__constants['BACK'],
            'START': cls.__constants['START'],
            'CROSS_X_UP': cls.__constants['CROSS_X_UP'],
            'CROSS_X_DOWN': cls.__constants['CROSS_X_DOWN'],
            'CROSS_Y_LEFT': cls.__constants['CROSS_Y_LEFT'],
            'CROSS_Y_RIGHT': cls.__constants['CROSS_Y_RIGHT'],
            'LT': cls.__constants['LT'],
            'RT': cls.__constants['RT'],
            'LEFT_STICK_X': cls.__constants['LEFT_STICK_X'],
            'LEFT_STICK_Y': cls.__constants['LEFT_STICK_Y'],
            'RIGHT_STICK_X': cls.__constants['RIGHT_STICK_X'],
            'RIGHT_STICK_Y': cls.__constants['RIGHT_STICK_Y'],
        }

    @property
    def LB(self):
        """Message constant 'LB'."""
        return Metaclass_GamepadKeys.__constants['LB']

    @property
    def RB(self):
        """Message constant 'RB'."""
        return Metaclass_GamepadKeys.__constants['RB']

    @property
    def A(self):
        """Message constant 'A'."""
        return Metaclass_GamepadKeys.__constants['A']

    @property
    def B(self):
        """Message constant 'B'."""
        return Metaclass_GamepadKeys.__constants['B']

    @property
    def X(self):
        """Message constant 'X'."""
        return Metaclass_GamepadKeys.__constants['X']

    @property
    def Y(self):
        """Message constant 'Y'."""
        return Metaclass_GamepadKeys.__constants['Y']

    @property
    def BACK(self):
        """Message constant 'BACK'."""
        return Metaclass_GamepadKeys.__constants['BACK']

    @property
    def START(self):
        """Message constant 'START'."""
        return Metaclass_GamepadKeys.__constants['START']

    @property
    def CROSS_X_UP(self):
        """Message constant 'CROSS_X_UP'."""
        return Metaclass_GamepadKeys.__constants['CROSS_X_UP']

    @property
    def CROSS_X_DOWN(self):
        """Message constant 'CROSS_X_DOWN'."""
        return Metaclass_GamepadKeys.__constants['CROSS_X_DOWN']

    @property
    def CROSS_Y_LEFT(self):
        """Message constant 'CROSS_Y_LEFT'."""
        return Metaclass_GamepadKeys.__constants['CROSS_Y_LEFT']

    @property
    def CROSS_Y_RIGHT(self):
        """Message constant 'CROSS_Y_RIGHT'."""
        return Metaclass_GamepadKeys.__constants['CROSS_Y_RIGHT']

    @property
    def LT(self):
        """Message constant 'LT'."""
        return Metaclass_GamepadKeys.__constants['LT']

    @property
    def RT(self):
        """Message constant 'RT'."""
        return Metaclass_GamepadKeys.__constants['RT']

    @property
    def LEFT_STICK_X(self):
        """Message constant 'LEFT_STICK_X'."""
        return Metaclass_GamepadKeys.__constants['LEFT_STICK_X']

    @property
    def LEFT_STICK_Y(self):
        """Message constant 'LEFT_STICK_Y'."""
        return Metaclass_GamepadKeys.__constants['LEFT_STICK_Y']

    @property
    def RIGHT_STICK_X(self):
        """Message constant 'RIGHT_STICK_X'."""
        return Metaclass_GamepadKeys.__constants['RIGHT_STICK_X']

    @property
    def RIGHT_STICK_Y(self):
        """Message constant 'RIGHT_STICK_Y'."""
        return Metaclass_GamepadKeys.__constants['RIGHT_STICK_Y']


class GamepadKeys(metaclass=Metaclass_GamepadKeys):
    """
    Message class 'GamepadKeys'.

    Constants:
      LB
      RB
      A
      B
      X
      Y
      BACK
      START
      CROSS_X_UP
      CROSS_X_DOWN
      CROSS_Y_LEFT
      CROSS_Y_RIGHT
      LT
      RT
      LEFT_STICK_X
      LEFT_STICK_Y
      RIGHT_STICK_X
      RIGHT_STICK_Y
    """

    __slots__ = [
        '_header',
        '_digital_states',
        '_analog_states',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'digital_states': 'int32[12]',
        'analog_states': 'double[6]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 12),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 6),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        if 'digital_states' not in kwargs:
            self.digital_states = numpy.zeros(12, dtype=numpy.int32)
        else:
            self.digital_states = kwargs.get('digital_states')
        if 'analog_states' not in kwargs:
            self.analog_states = numpy.zeros(6, dtype=numpy.float64)
        else:
            self.analog_states = kwargs.get('analog_states')

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
        if any(self.digital_states != other.digital_states):
            return False
        if any(self.analog_states != other.analog_states):
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
    def digital_states(self):
        """Message field 'digital_states'."""
        return self._digital_states

    @digital_states.setter
    def digital_states(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'digital_states' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 12, \
                "The 'digital_states' numpy.ndarray() must have a size of 12"
            self._digital_states = value
            return
        if __debug__:
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
                 len(value) == 12 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'digital_states' field must be a set or sequence with length 12 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._digital_states = numpy.array(value, dtype=numpy.int32)

    @builtins.property
    def analog_states(self):
        """Message field 'analog_states'."""
        return self._analog_states

    @analog_states.setter
    def analog_states(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'analog_states' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 6, \
                "The 'analog_states' numpy.ndarray() must have a size of 6"
            self._analog_states = value
            return
        if __debug__:
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
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'analog_states' field must be a set or sequence with length 6 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._analog_states = numpy.array(value, dtype=numpy.float64)
