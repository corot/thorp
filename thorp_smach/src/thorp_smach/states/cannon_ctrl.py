import smach_ros

import thorp_msgs.msg as thorp_msgs
import thorp_msgs.srv as thorp_srvs


class CannonCommand(smach_ros.ServiceState):
    def __init__(self):
        super(CannonCommand, self).__init__('cannon_command',
                                            thorp_srvs.CannonCommand,
                                            request_cb=self.request_cb,
                                            response_cb=self.response_cb)
        self.error_code = None

    def response_cb(self, ud, response):
        self.error_code = response.error

    def execute(self, ud):
        outcome = super(CannonCommand, self).execute(ud)
        if outcome == 'succeeded':
            return 'succeeded' if self.error_code.code == thorp_msgs.ThorpError.SUCCESS else 'aborted'
        return outcome


class AimCannon(CannonCommand):
    def request_cb(self, ud, request):
        request.action = thorp_srvs.CannonCommandRequest.AIM


class TiltCannon(CannonCommand):
    def __init__(self, angle):
        super(TiltCannon, self).__init__()
        self.angle = angle

    def request_cb(self, ud, request):
        request.action = thorp_srvs.CannonCommandRequest.TILT
        request.angle = self.angle


class FireCannon(CannonCommand):
    def __init__(self, shots):
        super(FireCannon, self).__init__()
        self.shots = shots

    def request_cb(self, ud, request):
        request.action = thorp_srvs.CannonCommandRequest.FIRE
        request.shots = self.shots
