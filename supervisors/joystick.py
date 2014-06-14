from supervisors.ultrabase import UltraBotSupervisor
from scripts.helpers import Struct
from scripts.ui import uiInt, uiBool


class JoystickSupervisor(UltraBotSupervisor):

    """Control the robot with a joystick!!!

       The supervisor will choose two first axes, and assign one to
       v and one to w control.
    """

    def __init__(self, robot_pose, robot_info):
        """Create the controller"""

        UltraBotSupervisor.__init__(self, robot_pose, robot_info)

        self.jcontroller = self.create_controller(
            'joystick.JoystickController',
            (self.parameters.joystick,
             robot_info.wheels.max_velocity *
             robot_info.wheels.radius,
             robot_info.wheels.max_velocity *
             robot_info.wheels.radius /
             robot_info.wheels.base_length))

        self.current = self.jcontroller

    def set_parameters(self, params):
        """Set parameters for itself and the controllers"""
        self.jcontroller.set_parameters(params.joystick)
        self.parameters.joystick = params.joystick

    def init_default_parameters(self):
        """Sets the default PID parameters, goal, and velocity"""
        self.parameters = Struct(
            {"joystick": {"i_j": 0, "i_x": 0, "i_y": 1, "inv_x": True,
                          "inv_y": True}})

    def get_ui_description(self, p=None):
        """Returns the UI description for the docker"""
        if p is None:
            p = self.parameters

        return [('joystick', [
            (('i_j', "Joystick index"),
             uiInt(p.joystick.i_j, 0, self.jcontroller.joystick_count() - 1)),
            (('i_x', "Omega axis"), uiInt(p.joystick.i_x, 0, 100)),
            (('inv_x', "Invert axis"), uiBool(p.joystick.inv_x)),
            (('i_y', "Velocity axis"), uiInt(p.joystick.i_y, 0, 100)),
            (('inv_y', "Invert axis"), uiBool(p.joystick.inv_y))])]

    def draw_background(self, renderer):
        pass

    def draw_foreground(self, renderer):
        pass
