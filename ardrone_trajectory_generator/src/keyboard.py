#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
In this module a simple GUI is created to control the AR.Drone
"""

from PyQt4 import QtGui, QtCore
import sys

KEYMAP = {
    QtCore.Qt.Key_Up: 'forward',
    QtCore.Qt.Key_Down: 'backward',
    QtCore.Qt.Key_Right: 'right',
    QtCore.Qt.Key_Left: 'left',
    QtCore.Qt.Key_W: 'up',
    QtCore.Qt.Key_S: 'down',
    QtCore.Qt.Key_D: 'clockwise',
    QtCore.Qt.Key_A: 'anti_clockwise',
    QtCore.Qt.Key_Return: 'take_off',
    QtCore.Qt.Key_Backspace: 'land',
    QtCore.Qt.Key_Space: 'stop',
    QtCore.Qt.Key_C: 'change_way_point',
    QtCore.Qt.Key_T: 'control_on',
    QtCore.Qt.Key_G: 'control_off',
}

class GUI(QtGui.QMainWindow, object):
    """docstring for GUI"""
    def __init__(self):
        super(GUI, self).__init__()
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('AR.Drone KeyboardController')
        self.show()

    def keyPressEvent(self, event):
        """
        key Press Parser
        """
        try:
            if not event.isAutoRepeat():
                cmd = KEYMAP[event.key()]
                print 'pressed', cmd
        except KeyError:
            pass

    def keyReleaseEvent(self, event):
        """
        key Press Parser
        """
        try:
            if not event.isAutoRepeat():
                cmd = KEYMAP[event.key()]
                print 'released', cmd
        except KeyError:
            pass


def run_GUI():
    """
    run created GUI
    """
    app = QtGui.QApplication(sys.argv)
    keyb = GUI()

    # executes the QT application
    status = app.exec_()

    #sys.exit(status)

if __name__ == '__main__':
    run_GUI()
