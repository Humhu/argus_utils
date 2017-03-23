"""This module provides an interface for runtime-settable parameters.
"""

import rospy
from threading import Lock
from paraset.msg import RuntimeParameter
from paraset.srv import GetParameterInfo, GetParameterInfoRequest, GetParameterInfoResponse
from paraset.srv import SetRuntimeParameter

_rtparam_to_type = {RuntimeParameter.PARAM_INVALID: None,
                    RuntimeParameter.PARAM_NUMERIC: float,
                    RuntimeParameter.PARAM_STRING: str,
                    RuntimeParameter.PARAM_BOOLEAN: bool}
_type_to_rtparam = {v: k for k, v in _rtparam_to_type.iteritems()}


def _generate_rtparam_msg(val):
    msg = RuntimeParameter()
    msg.type = _type_to_rtparam[type(val)]
    if val is None:
        pass
    elif isinstance(val, float):
        msg.numeric_value = val
    elif isinstance(val, str):
        msg.string_value = val
    elif isinstance(val, bool):
        msg.boolean_value = val
    return msg


def _retrieve_rtparam_value(msg):
    try:
        rt_type = _rtparam_to_type[msg.type]
    except KeyError:
        return None

    if rt_type is float:
        return float(msg.numeric_value)
    elif rt_type is str:
        return str(msg.string_value)
    elif rt_type is bool:
        return bool(msg.boolean_value)
    else:
        raise RuntimeError('Unknown type: %s' % str(rt_type))


class RuntimeParamSetter:
    """Provides an interface for setting runtime parameters.

    Parameters
    ----------
    param_type : class (float, str, or bool)
        The parameter type
    topic_path : string
        The fully-resolved parameter base ROS path
    """

    def __init__(self, param_type, name, base_topic):
        if param_type not in [float, str, bool]:
            raise ValueError('Invalid parameter type: %s' % str(param_type))
        self._type = param_type
        self._name = name

        if base_topic[-1] == '/':
            base_topic = base_topic[:-1]

        set_topic = '%s/set_%s' % (base_topic, name)
        rospy.loginfo('Waiting for %s...', set_topic)
        rospy.wait_for_service(set_topic)
        self._set_proxy = rospy.ServiceProxy(set_topic, SetRuntimeParameter)
        rospy.loginfo('Connected to %s', set_topic)

        get_topic = '%s/get_%s_info' % (base_topic, name)
        rospy.loginfo('Waiting for %s...', get_topic)
        rospy.wait_for_service(get_topic)
        self._info_proxy = rospy.ServiceProxy(get_topic, GetParameterInfo)
        rospy.loginfo('Connected to %s', get_topic)

    def set_value(self, v):
        """Set the runtime parameter.
        """
        val = self._type(v)
        rt = _generate_rtparam_msg(val)
        try:
            res = self._set_proxy(rt)
            return _retrieve_rtparam_value(res.actual)
        except rospy.ServiceException as e:
            rospy.logerr('Could not set value: %s', str(e))
            return None

    def get_info(self):
        """Retrieve the runtime parameter description.
        """
        try:
            return self._info_proxy()
        except rospy.ServiceException as e:
            rospy.logerr('Could not retrieve info: %s', str(e))
            return None


class RuntimeParamGetter:
    """Provides an interface for declaring and getting runtime parameters.

    Parameters
    ----------
    param_type  : class (float, str, or bool)
        The parameter type
    name        : string
        The registered name of the parameter
    init_val    : class of type 'type'
        The initial value this parameter should take
    description : string
        A human-readable description of the parameter
    """

    def __init__(self, param_type, name, init_val, description):
        if param_type not in [float, str, bool]:
            raise ValueError('Invalid parameter type: %s' % str(param_type))
        self._type = param_type
        self._name = name
        self._value = self._type(init_val)
        self._description = description
        self._checks = []
        self._mutex = Lock()
        os = 'Initializing runtime parameter: %s\n' % name
        os += '\tDescription: %s' % description
        os += '\tInitial value: %s' % str(init_val)
        rospy.loginfo(os)
        self._set_server = rospy.Service('~set_%s' % name,
                                         SetRuntimeParameter,
                                         self.__set_callback)
        self._info_server = rospy.Service('~get_%s_info' % name,
                                          GetParameterInfo,
                                          self.__info_callback)

    def add_check(self, c):
        self._mutex.acquire()
        self._checks.append(c)
        rospy.loginfo('Runtime parameter: %s added check: %s',
                      self._name, str(c))
        self._mutex.release()

    def __set_callback(self, req):
        """Handles SetRuntimeParameter service calls.
        """
        recv_val = _retrieve_rtparam_value(req.param)

        if recv_val is None:
            rospy.logerr('Unknown parameter type: %d', req.param.type)
            return None

        if not isinstance(recv_val, self._type):
            rospy.logerr('Parameter %s is type %s but got %s',
                         self._name, str(self._type), str(recv_val))

        self.__set_value(recv_val)
        return _generate_rtparam_msg(self._value)

    def __info_callback(self, req):
        out = 'Description: %s\n' % self._description
        out += 'Type: %s\n' % str(self._type)
        out += 'Constraints:\n'
        for c in self._checks:
            out += ' %s' % str(c)
        return out

    def __set_value(self, v):
        self._mutex.acquire()
        for c in self._checks:
            v = c(v)
        self._value = v
        self._mutex.release()

    @property
    def value(self):
        self._mutex.acquire()
        val = self._value
        self._mutex.release()
        return val

    @property
    def description(self):
        return self._description

    @property
    def param_type(self):
        return self._type

    @property
    def name(self):
        return self._name
