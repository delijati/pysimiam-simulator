import robots.ultrabase
from scripts.simobject import Cloud
from scripts.pose import Pose
from scripts.xbotcomm import XBotComm


def sign(x):
    if x == 0:
        return 0
    if x > 0:
        return 1
    return -1


def v2pwm(vl, vr):
    """Convert from angular velocity to PWM"""
    vl = min(vl, robots.ultrabase.MAX_VELOCITY)
    vr = min(vr, robots.ultrabase.MAX_VELOCITY)
    pwm_min_l = robots.ultrabase.MIN_PWM[0]
    pwm_min_r = robots.ultrabase.MIN_PWM[1]
    pwm_diff_l = 100.0 - pwm_min_l
    pwm_diff_r = 100.0 - pwm_min_r
    fac_l = abs(vl / robots.ultrabase.MAX_VELOCITY)
    fac_r = abs(vr / robots.ultrabase.MAX_VELOCITY)

    pwm_l = sign(vl) * ((fac_l * pwm_diff_l) + pwm_min_l)
    pwm_r = sign(vr) * ((fac_r * pwm_diff_r) + pwm_min_r)
    # print "v2pwm vl's ", vl, vr
    # print "v2pwm pwm's ", pwm_l, pwm_r
    return pwm_l, pwm_r


def pwm2v(pwm_l, pwm_r):
    """Convert from PWM to angular velocity"""
    pwm_min_l = robots.ultrabase.MIN_PWM[0]
    pwm_min_r = robots.ultrabase.MIN_PWM[1]
    pwm_l = 0.0 if abs(pwm_l) < pwm_min_l else pwm_l
    pwm_r = 0.0 if abs(pwm_r) < pwm_min_r else pwm_r
    pwm_l = min(pwm_l, 100.0)
    pwm_r = min(pwm_r, 100.0)
    pwm_diff_l = 100.0 - pwm_min_l
    pwm_diff_r = 100.0 - pwm_min_r
    vdiff = robots.ultrabase.MAX_VELOCITY - robots.ultrabase.MIN_VELOCITY
    fac_l = (abs(pwm_l) - pwm_min_l) / pwm_diff_l
    fac_r = (abs(pwm_r) - pwm_min_r) / pwm_diff_r

    vl = sign(pwm_l) * ((fac_l * vdiff) + robots.ultrabase.MIN_VELOCITY)
    vr = sign(pwm_r) * ((fac_r * vdiff) + robots.ultrabase.MIN_VELOCITY)

    # print "pwm2v vl's ", vl, vr
    # print "pwm2v pwm's ", pwm_l, pwm_r
    return vl, vr


class UltraBot(robots.ultrabase.BaseBot):

    """Communication with a UltraBot"""

    REALTIME = True

    def __init__(self, pose, color=0xFFFFFF, options=None):
        # create shape
        self._shapes = self._create_shapes()

        # create ultra sensors
        self.ultra_sensors = self._create_ultras(real=True)

        # Black, no readings
        self.walls = Cloud(0)

        # base constructor
        super(UltraBot, self).__init__(pose, color)

        # create info
        self.info = self._create_info(self.ultra_sensors)

        # Connect to bot...
        # This code will raise an exception if not able to connect

        if options is None:
            self.log("No IP/port supplied to connect to the robot")
            self.xbotcomm = XBotComm("localhost", "localhost", 5005)
        else:
            try:
                self.xbotcomm = XBotComm(options.baseIP, options.robotIP,
                                         options.port)
            except AttributeError:
                self.log(
                    "No IP/port supplied, unable to connect to the robot")
                self.xbotcomm = XBotComm("localhost", "localhost", 5005)

        self.xbotcomm.ping()  # Check if the robot is there
        self.pause()  # Initialize self.__paused

    def set_pose(self, rpose):
        super(UltraBot, self).set_pose(rpose)

        # We have to update walls here. WHY?
        for pose, dst in zip(
                [x.get_pose() for x in self.ultra_sensors],
                [x.distance() for x in self.ultra_sensors]):
            # if dst < robots.ultrabase.MIN_DIST_WALL:
            self.walls.add_point(Pose(dst) >> pose >> self.get_pose())

    def v2pwm(self, vl, vr):
        """Convert from angular velocity to PWM"""
        return v2pwm(vl, vr)

    def pwm2v(self, pwm_l, pwm_r):
        """Convert from PWM to angular velocity"""
        return pwm2v(pwm_l, pwm_r)

    def set_inputs(self, inputs, supervisor=None):
        pwm_l, pwm_r = self.v2pwm(*inputs)

        with self.xbotcomm.connect() as connection:
            self.xbotcomm.set_pwm(pwm_l, pwm_r, connection)
            # print "PWM ", pwm_l, pwm_r
            speeds = self.xbotcomm.get_pwm(connection)
            # print "speeds ", speeds
            if speeds is None:
                raise RuntimeError("Communication with UltraBot failed")

        if speeds is not None:
            self.info.wheels.vel_left, self.info.wheels.vel_right = self.pwm2v(
                *speeds)
        # useless??
        if supervisor:
            self.set_pose(supervisor.pose_est)

    def update_external_info(self):
        """Communicate with the robot"""

        with self.xbotcomm.connect() as connection:
            ticks = self.xbotcomm.get_encoder_ticks(connection)
            if ticks is None:
                raise RuntimeError("Communication with UltraBot failed")

            self.info.wheels.left_ticks, self.info.wheels.right_ticks = ticks

            # XXX we get date in this ordering
            # fm 2, br 4, fl 1, fr 3, bl 0
            _tmp = self.xbotcomm.get_ultra_raw_values(connection)
            ultras = (_tmp[4], _tmp[2], _tmp[0], _tmp[3], _tmp[1])
            print ultras
            if ultras is None:
                raise RuntimeError("Communication with UltraBot failed")

            # update simobject ultras
            for i in range(len(ultras)):
                self.ultra_sensors[i].update_distance_raw(ultras[i])

    def reset(self):
        # print "reset"
        self.__paused = True
        self.info.wheels.vel_left, self.info.wheels.vel_right = 0, 0
        self.xbotcomm.send_reset()

    def pause(self):
        # print "pause"
        self.__paused = True
        self.xbotcomm.send_halt()

    def resume(self):
        # print "resume"
        self.__paused = False
        self.set_inputs(
            (self.info.wheels.vel_left, self.info.wheels.vel_right))

    def move(self, dt):
        # print "move"
        self.__paused = False
        self.update_external_info()
