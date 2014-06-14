import numpy as np
import math
import scripts.robot
from scripts.pose import Pose
from scripts.helpers import Struct
from scripts.sensor import ProximitySensor

MAX_DIST = 4.0
MIN_DIST = 0.02
MIN_DIST_WALL = 0.3
MIN_PWM = [65, 70]  # left, right
MAX_VELOCITY = 2 * math.pi * 130 / 60  # 130 RPM
MIN_VELOCITY = 2 * math.pi * 30 / 60  # 30 RPM


class UltraBot_IRSensor(ProximitySensor):

    """Inherits from the proximity sensor class. Performs calculations
    specific to the HC-RS04 ultrasonic sensors"""

    def __init__(self, pose, robot, real=False):
        # values copied from SimIAm
        self.real = real
        ProximitySensor.__init__(self, pose, robot,
                                 (0.04, MAX_DIST, np.radians(30)))

    def update_distance(self, sim_object=None):
        if not self.real:
            super(UltraBot_IRSensor, self).update_distance(sim_object)

    def distance_to_value(self, dst):
        """Returns the distance of the proximity sensors"""

        if dst < self.rmin:
            return MIN_DIST  # min in m
        elif dst > self.rmax:
            return MAX_DIST  # max in m
        else:
            return dst

    def draw(self, r):
        pose = self.get_pose()
        r.set_pose(pose)

        dst = self.distance()
        # print "UltraBot_IRSensor dst %s" % dst

        if dst > MIN_DIST_WALL:
            r.set_brush(0x33FF5566)
        else:
            r.set_brush(0xCCFF5566)

        r.draw_ellipse(0, 0, min(1, self.rmin / 2), min(1, self.rmin / 2))
        r.draw_polygon(self.pts)
        r.reset_pose()
        #r.draw_text(-14, -14, "%.2f" % dst)
        #r.draw_rectangle(-14, -14, 8, 2)


class BaseBot(scripts.robot.Robot):

    """Communication with a UltraBot"""

    def _create_shapes(self):
        # create shape
        shapes = Struct()
        shapes.base_plate = np.array([[0.0335, 0.0534, 1],
                                      [0.0429, 0.0534, 1],
                                      [0.0639, 0.0334, 1],
                                      [0.0686, 0.0000, 1],
                                      [0.0639, -0.0334, 1],
                                      [0.0429, -0.0534, 1],
                                      [0.0335, -0.0534, 1],
                                      [-0.0465, -0.0534, 1],
                                      [-0.0815, -0.0534, 1],
                                      [-0.1112, -0.0387, 1],
                                      [-0.1112, 0.0387, 1],
                                      [-0.0815, 0.0534, 1],
                                      [-0.0465, 0.0534, 1]])

        shapes.bbb = np.array([[-0.0914, -0.0406, 1],
                               [-0.0944, -0.0376, 1],
                               [-0.0944, 0.0376, 1],
                               [-0.0914, 0.0406, 1],
                               [-0.0429, 0.0406, 1],
                               [-0.0399, 0.0376, 1],
                               [-0.0399, -0.0376, 1],
                               [-0.0429, -0.0406, 1]])

        shapes.bbb_rail_l = np.array([[-0.0429, -0.0356, 1],
                                      [-0.0429, 0.0233, 1],
                                      [-0.0479, 0.0233, 1],
                                      [-0.0479, -0.0356, 1]])

        shapes.bbb_rail_r = np.array([[-0.0914, -0.0356, 1],
                                      [-0.0914, 0.0233, 1],
                                      [-0.0864, 0.0233, 1],
                                      [-0.0864, -0.0356, 1]])

        shapes.bbb_eth = np.array([[-0.0579, 0.0436, 1],
                                   [-0.0579, 0.0226, 1],
                                   [-0.0739, 0.0226, 1],
                                   [-0.0739, 0.0436, 1]])

        shapes.left_wheel = np.array([[0.0254, 0.0595, 1],
                                      [0.0254, 0.0335, 1],
                                      [-0.0384, 0.0335, 1],
                                      [-0.0384, 0.0595, 1]])

        shapes.left_wheel_ol = np.array([[0.0254, 0.0595, 1],
                                         [0.0254, 0.0335, 1],
                                         [-0.0384, 0.0335, 1],
                                         [-0.0384, 0.0595, 1]])

        shapes.right_wheel_ol = np.array([[0.0254, -0.0595, 1],
                                          [0.0254, -0.0335, 1],
                                          [-0.0384, -0.0335, 1],
                                          [-0.0384, -0.0595, 1]])

        shapes.right_wheel = np.array([[0.0254, -0.0595, 1],
                                       [0.0254, -0.0335, 1],
                                       [-0.0384, -0.0335, 1],
                                       [-0.0384, -0.0595, 1]])

        shapes.ultra_bl = np.array([[-0.0732, 0.0534, 1],
                                    [-0.0732, 0.0634, 1],
                                    [-0.0432, 0.0634, 1],
                                    [-0.0432, 0.0534, 1]])

        shapes.ultra_fl = np.array([[0.0643, 0.0214, 1],
                                    [0.0714, 0.0285, 1],
                                    [0.0502, 0.0497, 1],
                                    [0.0431, 0.0426, 1]])

        shapes.ultra_fm = np.array([[0.0636, -0.0158, 1],
                                    [0.0636, 0.0158, 1],
                                    [0.0736, 0.0158, 1],
                                    [0.0736, -0.0158, 1]])

        shapes.ultra_fr = np.array([[0.0643, -0.0214, 1],
                                    [0.0714, -0.0285, 1],
                                    [0.0502, -0.0497, 1],
                                    [0.0431, -0.0426, 1]])

        shapes.ultra_br = np.array([[-0.0732, -0.0534, 1],
                                    [-0.0732, -0.0634, 1],
                                    [-0.0432, -0.0634, 1],
                                    [-0.0432, -0.0534, 1]])

        shapes.bbb_usb = np.array([[-0.0824, -0.0418, 1],
                                   [-0.0694, -0.0418, 1],
                                   [-0.0694, -0.0278, 1],
                                   [-0.0824, -0.0278, 1]])
        return shapes

    def _create_ultras(self, real=False):
        ultras = []

        ultra_sensor_poses = [
            # bl 0
            Pose(-0.0474, 0.0534, np.radians(90)),
            # fl 1
            Pose(0.0461, 0.0244, np.radians(45)),
            # fm 2
            Pose(0.0636, 0.0, np.radians(0)),
            # fr 3
            Pose(0.0461, -0.0244, np.radians(-45)),
            # br 4
            Pose(-0.0474, -0.0534, np.radians(-90)),
        ]

        for pose in ultra_sensor_poses:
            ultras.append(UltraBot_IRSensor(pose, self, real=real))
        return ultras

    def _create_info(self, ultras):
        info = Struct()
        info.wheels = Struct()
        info.wheels.radius = 0.012
        info.wheels.base_length = 0.09925
        info.wheels.ticks_per_rev = 20.0
        info.wheels.left_ticks = 0
        info.wheels.right_ticks = 0

        info.wheels.max_velocity = MAX_VELOCITY
        info.wheels.min_velocity = MIN_VELOCITY

        info.wheels.vel_left = 0
        info.wheels.vel_right = 0

        info.ultra_sensors = Struct()
        info.ultra_sensors.poses = [x.get_pose() for x in ultras]
        info.ultra_sensors.readings = None
        info.ultra_sensors.rmax = MAX_DIST
        info.ultra_sensors.rmin = MIN_DIST
        return info

    def draw(self, r):
        if self.REALTIME:
            self.walls.draw(r, thickness=.01)

        r.set_pose(self.get_pose())
        r.set_pen(0)
        r.set_brush(0)
        r.draw_polygon(self._shapes.ultra_bl)
        r.draw_polygon(self._shapes.ultra_fl)
        r.draw_polygon(self._shapes.ultra_fm)
        r.draw_polygon(self._shapes.ultra_fr)
        r.draw_polygon(self._shapes.ultra_br)

        r.draw_polygon(self._shapes.left_wheel)
        r.draw_polygon(self._shapes.right_wheel)

        r.set_pen(0x01000000)
        r.set_brush(self.get_color())
        r.draw_polygon(self._shapes.base_plate)

        r.set_pen(0x10000000)
        r.set_brush(None)
        r.draw_polygon(self._shapes.left_wheel)
        r.draw_polygon(self._shapes.right_wheel)

        r.set_pen(None)
        r.set_brush(0x333333)
        r.draw_polygon(self._shapes.bbb)
        r.set_brush(0)
        r.draw_polygon(self._shapes.bbb_rail_l)
        r.draw_polygon(self._shapes.bbb_rail_r)
        r.set_brush(0xb2b2b2)
        r.draw_polygon(self._shapes.bbb_eth)
        r.draw_polygon(self._shapes.bbb_usb)

    def get_envelope(self):
        return self._shapes.base_plate

    def draw_sensors(self, renderer):
        """Draw the sensors that this robot has"""
        for sensor in self.ultra_sensors:
            sensor.draw(renderer)

    def get_info(self):
        self.info.ultra_sensors.readings = [
            sensor.reading() for sensor in self.ultra_sensors]
        return self.info

    def get_external_sensors(self):
        return self.ultra_sensors
