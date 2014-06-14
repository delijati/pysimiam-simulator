#!/usr/bin/env python
# QtSimiam main executable
# Author: Tim Fuchs <typograph@elec.ru>
# Description: This is the top-level application for QtSimiam for
# controlling real robots.

import sys
from PyQt4 import QtGui
from gui.qt_mainwindow import SimulationWidget


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    simWidget = SimulationWidget()
    simWidget.step_action.setVisible(False)
    simWidget.speed_slider.setVisible(False)
    simWidget.show()
    simWidget.load_world("ultrabot_realtime.xml")
    app.exec_()
