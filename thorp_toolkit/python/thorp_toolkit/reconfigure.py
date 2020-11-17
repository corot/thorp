import os
import yaml

import rospy

import dynamic_reconfigure.client

from singleton import Singleton


class Reconfigure:
    """ Singleton that simplifies calling dynamic reconfigure services """
    __metaclass__ = Singleton

    def __init__(self):
        self.srv_clients = {}
        self.prev_values = {}
        self.named_configs = {}

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

    def use_named_config(self, config_name):
        """
        Use one of the named configurations loaded with load_named_configs
        :param config_name: configuration name
        :return: True on success, False otherwise
        """
        if config_name not in self.named_configs:
            rospy.logerr("Named configuration '%s' not loaded", config_name)
            return False
        named_config = self.named_configs[config_name]
        for ns, config in named_config.items():
            if not self.update_config(ns, config):
                return False
        rospy.loginfo("Using named configuration '%s' with %d namespace(s)", config_name, len(named_config))
        return True

    def dismiss_named_config(self, config_name):
        """
        Undo one of the named configurations currently in use
        :param config_name: configuration name
        :return: True on success, False otherwise
        """
        if config_name not in self.named_configs:
            rospy.logerr("Named configuration '%s' not loaded", config_name)
            return False
        named_config = self.named_configs[config_name]
        for ns, config in named_config.items():
            if not self.restore_config(ns, config.keys()):
                return False
        rospy.loginfo("Named configuration '%s' successfully dismissed", config_name)
        return True

    def load_named_configs(self, path=None):
        """
        Load all named configurations (yaml files) from a given path
        :param path: Target path (defaults to parameter 'named_configs_path' value)
        :return: True on success, False otherwise
        """
        if not path:
            path = rospy.get_param('named_configs_path')
        if not os.path.isdir(path):
            rospy.logerr("Named configurations path '%s' doesn't exist", path)
            return False
        count = 0
        for file_name in os.listdir(path):
            if file_name.endswith(".yaml"):
                with open(os.path.join(path, file_name), 'r') as yaml_file:
                    try:
                        config_name = file_name[:-5]
                        self.named_configs[config_name] = yaml.safe_load(yaml_file)
                        count += 1
                        rospy.loginfo("Named configuration '%s' successfully loaded", config_name)
                    except yaml.YAMLError as err:
                        rospy.logerr("Load named configuration from file '%s' failed: %s", file_name, str(err))
                        return False
        rospy.loginfo("%d named configurations loaded from '%s'", count, path)
        return True

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
