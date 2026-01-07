# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interface_protocol:msg/Heartbeat.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Heartbeat(type):
    """Metaclass of message 'Heartbeat'."""

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
            module = import_type_support('interface_protocol')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interface_protocol.msg.Heartbeat')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__heartbeat
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__heartbeat
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__heartbeat
            cls._TYPE_SUPPORT = module.type_support_msg__msg__heartbeat
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__heartbeat

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


class Heartbeat(metaclass=Metaclass_Heartbeat):
    """Message class 'Heartbeat'."""

    __slots__ = [
        '_header',
        '_node_name',
        '_startup_timestamp',
        '_node_status',
        '_error_code',
        '_error_message',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'node_name': 'string',
        'startup_timestamp': 'int64',
        'node_status': 'string',
        'error_code': 'int64',
        'error_message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.node_name = kwargs.get('node_name', str())
        self.startup_timestamp = kwargs.get('startup_timestamp', int())
        self.node_status = kwargs.get('node_status', str())
        self.error_code = kwargs.get('error_code', int())
        self.error_message = kwargs.get('error_message', str())

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
        if self.node_name != other.node_name:
            return False
        if self.startup_timestamp != other.startup_timestamp:
            return False
        if self.node_status != other.node_status:
            return False
        if self.error_code != other.error_code:
            return False
        if self.error_message != other.error_message:
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
    def node_name(self):
        """Message field 'node_name'."""
        return self._node_name

    @node_name.setter
    def node_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'node_name' field must be of type 'str'"
        self._node_name = value

    @builtins.property
    def startup_timestamp(self):
        """Message field 'startup_timestamp'."""
        return self._startup_timestamp

    @startup_timestamp.setter
    def startup_timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'startup_timestamp' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'startup_timestamp' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._startup_timestamp = value

    @builtins.property
    def node_status(self):
        """Message field 'node_status'."""
        return self._node_status

    @node_status.setter
    def node_status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'node_status' field must be of type 'str'"
        self._node_status = value

    @builtins.property
    def error_code(self):
        """Message field 'error_code'."""
        return self._error_code

    @error_code.setter
    def error_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_code' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'error_code' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._error_code = value

    @builtins.property
    def error_message(self):
        """Message field 'error_message'."""
        return self._error_message

    @error_message.setter
    def error_message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'error_message' field must be of type 'str'"
        self._error_message = value
