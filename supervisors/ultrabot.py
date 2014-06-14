from supervisors.ultrabase import UltraBotSupervisor
from scripts.ui import uiFloat
from math import sqrt, sin, cos
import numpy


class UltraFullSupervisor(UltraBotSupervisor):

    """
    QBFull supervisor implements the full switching behaviour for
    navigating labyrinths.
    """

    def __init__(self, robot_pose, robot_info, options=None):
        """Create controllers and the state transitions"""
        UltraBotSupervisor.__init__(self, robot_pose, robot_info)

        self.extgoal = False
        self.distance_from_goal_closest = 9999

        if options is not None:
            try:
                self.parameters.goal.x = options.x
                self.parameters.goal.y = options.y
                self.extgoal = True
            except Exception:
                pass

        # Fill in some parameters
        self.parameters.sensor_poses = robot_info.ultra_sensors.poses[:]
        self.parameters.ultra_max = robot_info.ultra_sensors.rmax
        self.parameters.direction = 'left'
        # min distance to obstacle
        self.parameters.distance = 0.2

        self.robot = robot_info

        # Add controllers
        self.avoidobstacles = self.create_controller(
            'AvoidObstacles', self.parameters)
        self.gtg = self.create_controller('GoToGoal', self.parameters)
        self.wall = self.create_controller('FollowWall', self.parameters)
        self.hold = self.create_controller('Hold', None)
        self.goback = self.create_controller('GoBack', self.parameters)

        # Define transitions
        self.add_controller(
            self.hold,
            (lambda: not self.at_goal(), self.gtg)
        )
        self.add_controller(
            self.gtg,
            (self.at_goal, self.hold),
            (self.at_obstacle, self.avoidobstacles),
            # (self.need_back, self.goback),  # new controll to move backward
        )

        self.add_controller(
            self.goback,
            (self.at_goal, self.hold),
            (self.free, self.gtg),
            (lambda: not self.progress_made() and self.at_obstacle(),
             self.wall),
        )

        self.add_controller(
            self.avoidobstacles,
            (self.at_goal, self.hold),
            (self.free, self.gtg),
            (lambda: not self.progress_made() and self.at_obstacle(),
             self.wall),
            # (self.need_back, self.goback),  # new controll to move backward
        )
        self.add_controller(
            self.wall,
            (lambda: self.progress_made() and self.can_detach(),
             self.gtg),
            (self.at_goal, self.hold),
            # (self.need_back, self.goback),  # new controll to move backward
        )

        # Start in the 'go-to-goal' state
        self.current = self.gtg
        #self.current = self.avoidobstacles
        print self.current

    def set_parameters(self, params):
        """Set parameters for itself and the controllers"""
        UltraBotSupervisor.set_parameters(self, params)
        self.gtg.set_parameters(self.parameters)
        self.avoidobstacles.set_parameters(self.parameters)
        self.wall.set_parameters(self.parameters)

    def unsafe(self):
        return self.distmin < self.parameters.distance * 1.5

    def at_goal(self):
        """Check if the distance to goal is small"""
        return self.distance_from_goal < 0.15

    def need_back(self):
        """Check if the distance is too small and go back"""
        ret = self.distmin < self.parameters.distance * 0.4
        return ret

    def at_obstacle(self):
        """Check if the distance to obstacle is small"""
        ret = self.distmin < self.parameters.distance * 0.8
        if ret:
            if self.distance_from_goal < self.distance_from_goal_closest:
                self.distance_from_goal_closest = self.distance_from_goal
        return ret

    def free(self):
        """Check if the distance to obstacle is large"""
        return self.distmin > self.parameters.distance * 1.1

    def progress_made(self):
        e = 0.1
        ret = self.distance_from_goal < self.distance_from_goal_closest - e
        # print "progress_made goal (%s) closest (%s)" % (
        #   self.distance_from_goal, self.distance_from_goal_closest)
        return ret

    def can_detach(self):
        u_gtg = self.gtg.get_heading(self.parameters)
        u_fw = self.wall.get_heading(self.parameters)

        left = numpy.dot(u_gtg, u_fw) > 0
        right = numpy.dot(u_gtg, u_fw) < 0
        ret = False
        if self.parameters.direction == 'left' and left:
            ret = True
        if self.parameters.direction == 'right' and right:
            ret = True
        # print "can_detach (%s)" % ret
        return ret

    def process_state_info(self, state):
        """Update state parameters for the controllers and self"""

        UltraBotSupervisor.process_state_info(self, state)

        # The pose for controllers
        self.parameters.pose = self.pose_est

        # Distance to the goal
        self.distance_from_goal = sqrt(
            (self.pose_est.x - self.parameters.goal.x) ** 2 + (
                self.pose_est.y - self.parameters.goal.y) ** 2)

        # Sensor readings
        self.parameters.sensor_distances = self.robot.ultra_sensors.readings

        # Distance to the closest obstacle
        self.distmin = min(self.robot.ultra_sensors.readings)

    def draw_foreground(self, renderer):
        """Draw controller info"""
        UltraBotSupervisor.draw_foreground(self, renderer)

        # Make sure to have all headings:
        renderer.set_pose(self.pose_est)
        arrow_length = self.robot_size * 5

        # Ensure the headings are calculated

        # Draw arrow to goal
        if self.current == self.gtg:
            goal_angle = self.gtg.get_heading_angle(self.parameters)
            renderer.set_pen(0x00FF00)
            renderer.draw_arrow(0, 0,
                                arrow_length * cos(goal_angle),
                                arrow_length * sin(goal_angle))

        # Draw arrow away from obstacles
        elif self.current == self.avoidobstacles:
            away_angle = self.avoidobstacles.get_heading_angle(self.parameters)
            renderer.set_pen(0xCC3311)
            renderer.draw_arrow(0, 0,
                                arrow_length * cos(away_angle),
                                arrow_length * sin(away_angle))

        # Draw vector to wall
        elif self.current == self.wall:
            renderer.set_pen(0x0000FF)
            renderer.draw_arrow(0, 0,
                                self.wall.to_wall_vector[0],
                                self.wall.to_wall_vector[1])
            # Draw
            renderer.set_pen(0xFF00FF)
            renderer.push_state()
            renderer.translate(
                self.wall.to_wall_vector[0], self.wall.to_wall_vector[1])
            renderer.draw_arrow(0, 0,
                                self.wall.along_wall_vector[0],
                                self.wall.along_wall_vector[1])
            renderer.pop_state()

            # Draw heading (who knows, it might not be along_wall)
            renderer.set_pen(0xFF00FF)
            renderer.draw_arrow(0, 0,
                                arrow_length * cos(self.wall.heading_angle),
                                arrow_length * sin(self.wall.heading_angle))

    def get_ui_description(self, p=None):
        """Returns the UI description for the docker"""
        if p is None:
            p = self.parameters

        ui = [('goal', [('x', uiFloat(p.goal.x, 0.1)),
                        ('y', uiFloat(p.goal.y, 0.1))]),
              ('velocity', [('v', uiFloat(p.velocity.v, 0.1))]),
              (('gains', "PID gains"), [
                  (('kp', 'Proportional gain'), uiFloat(p.gains.kp, 0.1)),
                  (('ki', 'Integral gain'), uiFloat(p.gains.ki, 0.1)),
                  (('kd', 'Differential gain'), uiFloat(p.gains.kd, 0.1))])]

        if self.extgoal:
            return ui[1:]
        else:
            return ui
