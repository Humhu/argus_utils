"""This module contains functions for accessing the lookup system.
"""

import rospy

class LookupError(Exception):
    """Represents an error retrieving information from the lookup system.
    """
    def __init__(self, msg=''):
        super(LookupError, self).__init__(self, msg)

lookup_namespace='/lookup'

def register_lookup_target(target_name, target_namespace):
    """Writes a lookup entry in the lookup system, storing the fully resolved
    target namespace to a global ROS parameter.

    Parameters
    ----------
    target_name      : string
        The lookup name to register
    target_namespace : string
        The ROS namespace to fully resolve in this current context and store
    """
    # NOTE Strip the trailing / character
    resolved_namespace = rospy.resolve_name(target_namespace)[:-1]
    rospy.loginfo('Registering (' + target_name + ') with namespace ('
                  + resolved_namespace + ') to registry (' + lookup_namespace + ')')
    key = lookup_namespace + '/' + target_name
    if rospy.has_param(key):
        raise LookupError('Lookup name %s already registered!' % target_name)
    rospy.set_param(key, resolved_namespace)

def retrieve_lookup_target(target_name, num_retries=5):
    """Reads a lookup entry in the lookup system, retrieving a fully resolved
    namespace if it exists or raising a LookupException if it does not.

    Parameters
    ----------
    target_name      : string
        The lookup name to register
    num_retries      : integer (default 5)
        Number of times to wait 1 second and retry

    Returns
    -------
    target_namespace : string
        The registered fully resolved namespace
    """
    for i in range(num_retries+1):
        try:
            key = lookup_namespace + '/' + target_name
            return rospy.get_param(key)
        except KeyError:
            rospy.sleep(rospy.Duration(1.0))
    raise LookupError('Lookup name %s not registered!' % target_name)


