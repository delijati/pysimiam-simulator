from scripts.pose import Pose
from math import sin, cos, pi
import robots.ultrabase


class UltraBot(robots.ultrabase.BaseBot):

    """Communication with a UltraBot"""

    REALTIME = False

    def __init__(self, pose, color=0xFFFFFF, options=None):
        # base constructor
        super(UltraBot, self).__init__(pose, color)

        # create shape
        self._shapes = self._create_shapes()

        # create ultra sensors
        self.ultra_sensors = self._create_ultras()

        # create info
        self.info = self._create_info(self.ultra_sensors)

        # initialize motion
        self.ang_velocity = (0.0, 0.0)

        self.left_revolutions = 0.0
        self.right_revolutions = 0.0

    def move(self, dt):
        # There's no need to use the integrator - these equations have a
        # solution
        (vl, vr) = self.get_wheel_speeds()
        (v, w) = self.diff2uni((vl, vr))
        x, y, theta = self.get_pose()
        if w == 0:
            x += v * cos(theta) * dt
            y += v * sin(theta) * dt
        else:
            dtheta = w * dt
            x += 2 * v / w * cos(theta + dtheta / 2) * sin(dtheta / 2)
            y += 2 * v / w * sin(theta + dtheta / 2) * sin(dtheta / 2)
            theta += dtheta

        self.set_pose(Pose(x, y, (theta + pi) % (2 * pi) - pi))

        self.left_revolutions += vl * dt / 2 / pi
        self.right_revolutions += vr * dt / 2 / pi
        self.info.wheels.left_ticks = int(
            self.left_revolutions * self.info.wheels.ticks_per_rev)
        self.info.wheels.right_ticks = int(
            self.right_revolutions * self.info.wheels.ticks_per_rev)

    def set_inputs(self, inputs, supervisor=None):
        self.set_wheel_speeds(inputs)

    def diff2uni(self, diff):
        (vl, vr) = diff
        v = (vl + vr) * self.info.wheels.radius / 2
        w = (vr - vl) * self.info.wheels.radius / self.info.wheels.base_length
        return (v, w)

    def get_wheel_speeds(self):
        return self.ang_velocity

    def set_wheel_speeds(self, *args):
        if len(args) == 2:
            (vl, vr) = args
        else:
            (vl, vr) = args[0]

        left_ms = max(-self.info.wheels.max_velocity,
                      min(self.info.wheels.max_velocity, vl))
        right_ms = max(-self.info.wheels.max_velocity,
                       min(self.info.wheels.max_velocity, vr))

        self.ang_velocity = (left_ms, right_ms)

    def get_external_sensors(self):
        return self.ultra_sensors

    def update_sensors(self):
        for sensor in self.ultra_sensors:
            sensor.update_distance()


if __name__ == "__main__":
    # JP limits
    #v = max(min(v,0.314),-0.3148);
    #w = max(min(w,2.276),-2.2763);
    # Real limits
    k = UltraBot(Pose(0, 0, 0))
    k.set_wheel_speeds(1000, 1000)
    print(k.diff2uni(k.get_wheel_speeds()))
    k.set_wheel_speeds(1000, -1000)
    print(k.diff2uni(k.get_wheel_speeds()))
    # 0.341 and 7.7
