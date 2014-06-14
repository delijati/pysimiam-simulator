from scripts.controller import Controller


class Hold(Controller):

    """This controller halts the robot"""

    def __init__(self, params):
        Controller.__init__(self, params)

    def set_parameters(self, params):
        """This controller has no parameters"""
        pass

    def execute(self, state, dt):
        """Stop the robot"""
        return [0., 0.]
