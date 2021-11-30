import smach_ros

import thorp_msgs.msg as thorp_msgs
import thorp_msgs.srv as thorp_srvs


class CannonCmd(smach_ros.ServiceState):
    def __init__(self):
        super(CannonCmd, self).__init__('cannon_command',
                                        thorp_srvs.CannonCmd,
                                        request_cb=self.request_cb,
                                        response_cb=self.response_cb)
        self.error_code = None

    def response_cb(self, ud, response):
        self.error_code = response.error

    def execute(self, ud):
        outcome = super(CannonCmd, self).execute(ud)
        if outcome == 'succeeded':
            return 'succeeded' if self.error_code.code == thorp_msgs.ThorpError.SUCCESS else 'aborted'
        return outcome


class AimCannon(CannonCmd):
    def request_cb(self, ud, request):
        request.action = thorp_srvs.CannonCmdRequest.AIM


class TiltCannon(CannonCmd):
    def __init__(self, angle):
        super(TiltCannon, self).__init__()
        self.angle = angle

    def request_cb(self, ud, request):
        request.action = thorp_srvs.CannonCmdRequest.TILT
        request.angle = self.angle


class FireCannon(CannonCmd):
    def __init__(self, shots):
        super(FireCannon, self).__init__()
        self.shots = shots

    def request_cb(self, ud, request):
        request.action = thorp_srvs.CannonCmdRequest.FIRE
        request.shots = self.shots
