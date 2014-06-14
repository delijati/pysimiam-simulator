from __future__ import print_function
import sys
from PyQt4 import QtGui

from gui.qt_mainwindow import SimulationWidget

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    simWidget = SimulationWidget()
    simWidget.superv_action.trigger()
    simWidget.show()
    simWidget.load_world("ultrabot_simulation.xml")
    simWidget.add_graph([
        [("Robot theta", "robot.get_pose().theta", 'red'),
         ("Angle to goal", "math.atan2(supervisor.parameters.goal.y - robot.get_pose().y,supervisor.parameters.goal.x - robot.get_pose().x)", 'blue')],
        [("Robot X", "robot.get_pose().x", 'red'),
         ("Goal X",
          "supervisor.parameters.goal.x", 'blue')],
    ])
    app.exec_()
