from controllers.pid_controller import PIDController
import numpy


class GoBack(PIDController):

    """
    Go in oposit direction of obstcle
    """

    def __init__(self, params):
        """Initialize internal variables"""
        PIDController.__init__(self, params)
        self._timer = 0
        self.done = False

    def set_parameters(self, params):
        """Set PID values and sensor poses.

        The params structure is expected to have sensor poses in the robot's
        reference frame as ``params.sensor_poses``.
        """
        PIDController.set_parameters(self, params)

        self.sensor_poses = params.sensor_poses

    def get_heading(self, state):
        """Get the direction away from the obstacles as a vector."""

        # oposit direction of obstacle
        d, i = min(zip(state.sensor_distances,
                       range(len(state.sensor_distances))))
        pose = self.sensor_poses[i]
        return numpy.array([pose.x, pose.y, pose.theta])

    def execute(self, state, dt):
        self._timer += 1
        v, w = PIDController.execute(self, state, dt)
        d, i = min(zip(state.sensor_distances,
                       range(len(state.sensor_distances))))
        if i in (1, 2, 3):
            ret = -v, w
        else:
            ret = v, w
        print self._timer

        if d > self.params.distance:
            # if self._timer >= 23:
            self._timer = 0
            self.done = True
        return ret

    def restart(self):
        # super(GoBack, self).restart()
        PIDController.restart(self)
        self.done = False
