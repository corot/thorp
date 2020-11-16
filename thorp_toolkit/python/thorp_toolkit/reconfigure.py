import rospy

import dynamic_reconfigure.client

from singleton import Singleton


class Reconfigure:
    """ Singleton that simplifies calling dynamic reconfigure services """
    __metaclass__ = Singleton

    def __init__(self):
        self.srv_clients = {}
        self.prev_values = {}

    def update_config(self, ns, config):
        """
        Update configuration with the given parameter-value pairs.
        :param ns: Namespace to which the parameters belong
        :param config: Dictionary containing parameter-value pairs
        :return: True on success, False otherwise
        """
        try:
            srv = self.__get_srv(ns)
            # Keep previous values to restore if requested
            for param in config.keys():
                self.prev_values[param] = rospy.get_param(ns + '/' + param)
            # Set new values
            srv.update_configuration(config)
            return True
        except rospy.exceptions.ROSException as err:
            rospy.logerr("'%s' dynamic reconfigure service not available: %s", ns, str(err))
            return False
        except dynamic_reconfigure.DynamicReconfigureCallbackException as err:
            rospy.logerr("Call to '%s' dynamic reconfigure service failed: %s", ns, str(err))
            return False

    def restore_config(self, ns, params):
        """
        Restore previous values for the given parameters. This is expected to be called after update_config
        with the list of parameters previously changed.
        :param ns: Namespace to which the parameters belong
        :param params: List of parameters to restore
        :return: True on success, False otherwise
        """
        try:
            srv = self.__get_srv(ns)
            prev_config = {}
            for param in params:
                if param in self.prev_values:
                    prev_config[param] = self.prev_values[param]
                    del self.prev_values[param]
                else:
                    rospy.logwarn("No previous value stored for parameter '%s'", param)
            if prev_config:
                srv.update_configuration(prev_config)
            return True
        except rospy.exceptions.ROSException as err:
            rospy.logerr("'%s' dynamic reconfigure service not available: %s", ns, str(err))
            return False
        except dynamic_reconfigure.DynamicReconfigureCallbackException as err:
            rospy.logerr("Call to '%s' dynamic reconfigure service failed: %s", ns, str(err))
            return False

    def __get_srv(self, ns):
        if ns in self.srv_clients:
            return self.srv_clients[ns]

        # dynamic reconfigure client not available, so we must create one
        try:
            rospy.logdebug("Creating '%s' dynamic reconfiguration", ns)
            rospy.loginfo("Waiting for '%s' dynamic reconfigure service...", ns)
            srv = dynamic_reconfigure.client.Client(ns, timeout=5)
            rospy.loginfo("Dynamic reconfigure service '%s' alive!", ns)
            self.srv_clients[ns] = srv
            return srv
        except rospy.exceptions.ROSInterruptException:
            pass
