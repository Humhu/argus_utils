#!/usr/bin/env python

from lookup.LookupInterface import register_lookup_target, retrieve_lookup_target
from broadcast.msg import FloatVectorStamped
from broadcast.srv import QueryFeatures, QueryFeaturesRequest, QueryFeaturesResponse
from broadcast.utils import TimeSeries, ros_time_diff

import rospy

feature_size_key = 'feature_size'
description_key = 'descriptions'
topic_key = 'topic'
mode_key = 'mode'


def register_broadcast_info(name, feature_size, description, topic, mode):
    """Registers broadcast stream information.

    Parameters
    ----------
    name         : string
        The stream's lookup-registered name
    feature_size : integer
        The dimensionality of the stream
    description  : string
        A human-readable description of the stream, feature elements
    mode         : string (push or pull)
        Whether this stream is operating in push or pull mode
    topic        : string
        For push mode, the fully-resolved ROS topic the stream is published to
        For pull mode, the fully-resolved ROS service to call for retrieving
    """
    namespace = retrieve_lookup_target(name)
    rospy.set_param(namespace + '/' + feature_size_key, feature_size)
    rospy.set_param(namespace + '/' + description_key, description)
    rospy.set_param(namespace + '/' + topic_key, topic)
    rospy.set_param(namespace + '/' + mode_key, mode)


def retrieve_broadcast_info(name, num_retries=5):
    """Retrieves a broadcast stream's information.

    Parameters
    ----------
    name        : string
        The stream's lookup-registered name
    num_retries : integer
        Number of times to wait 1 second and retry
    Returns
    -------
    info        : dict
        A dictionary populated with the stream's information
        See register_broadcast_info for information fields
    """
    for i in range(num_retries+1):
        try:
            namespace = retrieve_lookup_target(name)
            info = {}
            info[feature_size_key] = rospy.get_param(namespace + '/' + feature_size_key)
            info[description_key] = rospy.get_param(namespace + '/' + description_key)
            info[topic_key] = rospy.get_param(namespace + '/' + topic_key)
            info[mode_key] = rospy.get_param(namespace + '/' + mode_key)
            return info
        except (LookupError, KeyError):
            rospy.sleep(rospy.Duration(1.0))

pull_mode_dict = {QueryFeaturesRequest.CLOSEST_BEFORE: 'closest_before',
                  QueryFeaturesRequest.CLOSEST_AFTER: 'closest_after',
                  QueryFeaturesRequest.CLOSEST_EITHER: 'closest_either'}
inv_pull_dict = dict(zip(pull_mode_dict.values(), pull_mode_dict.keys()))

def from_pull_mode(m):
    """Converts from QueryFeaturesRequest's mode enum to a string
    """
    return pull_mode_dict[m]

def to_pull_mode(s):
    """Converts a string to QueryFeatureRequest's mode enum.
    """
    return inv_pull_dict[s]

class Transmitter(object):
    """Sets up a broadcast stream and allows publishing to it.

    Parameters
    ----------
    stream_name  : string
        Unique name for this broadcast
    feature_size : string
        Dimensionality of the broadcast feature vector
    description  : string (default '')
        Description of each feature dimension
    mode         : string (default push)
        Either 'push' or 'pull'
    namespace    : string (default ~)
        Namespace for the transmitter
    topic        : string
        The broadcast topic/service name. If push mode, defaults to 'stream_raw'.
        If pull mode, defaults to 'pull_stream'. It is recommended to use defaults.

    Push mode parameters
    --------------------
    queue_size:   Publish queue size (10)

    Pull mode parameters
    --------------------
    cache_time:   Length of time to keep data
    """

    def __init__(self, stream_name, feature_size, description='', mode='push',
                 namespace='~', topic=None, **kwargs):

        if len(namespace) > 0 and namespace != '~' and namespace[-1] != '/':
            namespace += '/'

        # Register in the lookup directory
        register_lookup_target(target_name=stream_name,
                               target_namespace=namespace)

        if topic is None:
            if mode == 'push':
                topic = 'stream_raw'
            elif mode == 'pull':
                topic = 'pull_stream'

        # Populate field info
        topic_path = rospy.resolve_name(namespace + topic)
        rospy.loginfo('Registering broadcast (' + stream_name + ') to topic ('
                      + topic_path + ')')

        self.stream_name = stream_name
        self.mode = mode
        if mode == 'push':
            self.publisher = rospy.Publisher(topic_path,
                                             FloatVectorStamped,
                                             queue_size=kwargs['queue_size'])
        elif mode == 'pull':
            self.cache = TimeSeries(diff=ros_time_diff)
            self.cache_time = kwargs['cache_time']
            self.server = rospy.Service(topic_path,
                                        QueryFeatures,
                                        self.__query_callback)
        else:
            raise ValueError('Invalid mode: ' + mode)

        register_broadcast_info(name=stream_name,
                                feature_size=feature_size,
                                description=description,
                                topic=topic_path,
                                mode=mode)

    def publish(self, time, feats):
        """Publish a message to the broadcast topic.

        Parameters
        ----------
        time  : rospy.Time object
            The corresponding feature timestamp
        feats : iterable of floats
            The feature values
        """

        if self.mode == 'push':
            msg = FloatVectorStamped()
            msg.header.stamp = time
            msg.header.frame_id = self.stream_name
            msg.values = feats
            self.publisher.publish(msg)

        elif self.mode == 'pull':
            item = FloatVectorStamped()
            item.header.stamp = time
            item.header.frame_id = self.stream_name
            item.values = feats
            self.cache.insert(time, item)

    def __query_callback(self, req):
        if self.mode == 'push':
            raise RuntimeError('Query callback called in push mode!')
        res = QueryFeaturesResponse()

        mode = from_pull_mode(req.time_mode)
        res.features = self.cache.get_data(req.query_time, mode).data
        return res


class Receiver(object):
    """Provides an interface to receive broadcast streams.

    Parameters
    ----------
    stream_name  : string
        Unique name for this broadcast
    """

    def __init__(self, stream_name):
        info = retrieve_broadcast_info(stream_name)

        self._dim = info[feature_size_key]
        self._mode = info[mode_key]
        self._description = info[description_key]
        if self._mode == 'pull':
            self.proxy = rospy.ServiceProxy(info[topic_key], QueryFeatures)
        elif self._mode == 'push':
            self.subscriber = rospy.Subscriber(info[topic_key],
                                               FloatVectorStamped,
                                               self.__push_callback)
            self.cache = TimeSeries(diff=ros_time_diff)
        else:
            raise RuntimeError('Unknown stream mode: %s' % self._mode)

    def read_stream(self, time, mode):
        """Retrieves or reads the stream at the specified time.

        Parameters
        ----------
        time  : rospy.Time
            The time value to retrieve from
        mode  : string (closest_before, closest_after, closest_either)
            The time nearest-neighbor mode
        Returns
        -------
        stamp : rospy.Time
            The retrieved stream data timestamp. None if no data was found
        data  : vector of floats
            The retrieved stream data values. None if no data was found
        """
        if self._mode == 'pull':
            req = QueryFeaturesRequest()
            req.query_time = time
            req.time_mode = to_pull_mode(mode)
            try:
                res = self.proxy(req)
                return res.features.header.stamp, res.features.values
            except rospy.ServiceException as e:
                rospy.loginfo('Could not read stream: %s', str(e))
                return None, None

        elif self._mode == 'push':
            item = self.cache.get_data(time, mode)
            if item is None:
                return None, None
            else:
                return item.time, item.data

        else:
            raise ValueError('Unknown stream mode %s' % self._mode)

    def __push_callback(self, msg):
        self.cache.insert(msg.header.stamp, msg.values)

    @property
    def stream_feature_size(self):
        return self._dim

    @property
    def stream_description(self):
        return self._description

    @property
    def stream_mode(self):
        return self._mode
