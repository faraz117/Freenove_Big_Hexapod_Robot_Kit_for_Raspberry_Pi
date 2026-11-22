# -*- coding: utf-8 -*-
import os
import sys,getopt
import time
import threading
import psutil
import Thread
from ui_server import Ui_server
from PyQt5.QtCore import QCoreApplication, QTimer, Qt
from PyQt5.QtGui import QPixmap, QColor
from PyQt5.QtWidgets import QApplication, QMainWindow
#from PyQt5.QtGui import *
from server import Server

class MyWindow(QMainWindow,Ui_server):
    def __init__(self):
        self.user_ui=True
        self.start_tcp=False
        self.server=Server()
        self.parseOpt()
        if self.user_ui:
            self.app = QApplication(sys.argv)
            super(MyWindow,self).__init__()
            self.setupUi(self)
            # The On/Off button pauses/resumes the robot's own movement - it no longer touches
            # the network server, which stays up regardless so a client can always connect.
            self.pushButton_On_And_Off.clicked.connect(self.toggle_pause)
            self.latest_frame = None
            self.camera_thread = threading.Thread(target=self.camera_preview_loop, daemon=True)
            self.camera_thread.start()
            self.status_timer = QTimer()
            self.status_timer.timeout.connect(self.update_status)
            self.status_timer.start(300)
            self.camera_timer = QTimer()
            self.camera_timer.timeout.connect(self.update_camera_preview)
            self.camera_timer.start(66)  # ~15fps, independent of the slower obstacle/stats refresh
            self.update_hardware_status()  # reflect a startup I2C failure immediately, not after the first tick
            self.start_network()
        elif self.start_tcp:
            self.start_network()

    def start_network(self):
        self.server.start_server()
        self.server.tcp_flag=True
        self.video=threading.Thread(target=self.server.transmit_video)
        self.video.start()
        self.instruction=threading.Thread(target=self.server.receive_commands)
        self.instruction.start()

    def camera_preview_loop(self):
        # Keeps the camera stream alive for the local preview regardless of whether a
        # remote video client is connected, and caches the latest JPEG frame for the
        # (non-blocking) Qt timer tick to paint - get_frame() blocks until a frame is
        # ready, so it must not be called directly from the GUI thread.
        while True:
            try:
                if not self.server.camera_device.streaming:
                    self.server.camera_device.start_stream()
                self.latest_frame = self.server.camera_device.get_frame()
            except Exception as e:
                print("camera_preview_loop:", e)
                time.sleep(0.5)

    def update_status(self):
        self.update_hardware_status()
        self.update_obstacle_label()
        self.update_system_stats()
        self.update_detection_table()

    def update_hardware_status(self):
        # Reflects Control.hardware_ok regardless of what caused it to change (the button,
        # a fresh startup failure, or a mid-operation I2C drop caught by SupervisedServo) -
        # polled every tick rather than only updated at click-time.
        hardware_ok = self.server.control_system.hardware_ok
        if hardware_ok and self.pushButton_On_And_Off.text() == 'Recheck I2C':
            self.pushButton_On_And_Off.setText('Pause')
            self.states.setText('Running')
        elif not hardware_ok:
            self.pushButton_On_And_Off.setText('Recheck I2C')
            self.states.setText('I2C ERROR')

    def update_obstacle_label(self):
        obstacle = self.server.control_system.last_obstacle
        if obstacle is None:
            self.obstacle_label.setText("No obstacle detected yet")
        else:
            self.obstacle_label.setText(
                "[{}] obstacle detected\npan {}°, tilt {}°, {} cm".format(
                    obstacle['source'].upper(), obstacle['pan'], obstacle['tilt'], obstacle['distance']))

    def update_camera_preview(self):
        if self.latest_frame:
            pixmap = QPixmap()
            if pixmap.loadFromData(self.latest_frame, "JPG"):
                self.camera_label.setPixmap(pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio))

    def voltage_to_percent(self, voltage, min_voltage=6.0, max_voltage=8.4):
        # Rough approximation for a 2S Li-ion pack (nominal 7.4V, ~8.4V full, ~6.0V empty).
        # Li-ion discharge isn't actually linear, so treat this as an estimate, not a gauge.
        percent = (voltage - min_voltage) / (max_voltage - min_voltage) * 100
        return max(0, min(100, round(percent)))

    def update_system_stats(self):
        cpu = psutil.cpu_percent()
        ram = psutil.virtual_memory().percent
        try:
            load_voltage, pi_voltage = self.server.adc_sensor.read_battery_voltage()
            battery_text = "Batt(load): {}%   Batt(Pi): {}%".format(
                self.voltage_to_percent(load_voltage), self.voltage_to_percent(pi_voltage))
        except Exception as e:
            battery_text = "Batt: --"
            print("update_system_stats: battery read failed:", e)
        self.stats_label.setText("CPU: {:.0f}%   RAM: {:.0f}%\n{}".format(cpu, ram, battery_text))

    def update_detection_table(self):
        grid = self.server.control_system.detection_grid
        pan_points = self.server.control_system.pan_points
        tilt_points = self.server.control_system.tilt_points
        colors = {'0': QColor('#2e7d32'), 'U': QColor('#f9a825'),
                  'C': QColor('#1565c0'), 'B': QColor('#c62828')}
        for row, tilt_angle in enumerate(tilt_points):
            for col, pan_angle in enumerate(pan_points):
                status = grid.get((pan_angle, tilt_angle), '0')
                item = self.detection_table.item(row, col)
                item.setText(status)
                item.setBackground(colors.get(status, colors['0']))

    def parseOpt(self):
        self.opts,self.args = getopt.getopt(sys.argv[1:],"tn")
        for o,a in self.opts:
            if o in ('-t'):
                print ("Open TCP")
                self.start_tcp=True
            elif o in ('-n'):
                self.user_ui=False
                
    def toggle_pause(self):
        control = self.server.control_system
        if not control.hardware_ok:
            print("GUI: rechecking I2C...")
            self.server.recheck_i2c()
            # update_hardware_status (next status_timer tick) reflects whatever this found.
            return
        if self.pushButton_On_And_Off.text() == 'Pause':
            control.paused = True
            self.pushButton_On_And_Off.setText('Resume')
            self.states.setText('Paused')
        else:
            control.paused = False
            self.pushButton_On_And_Off.setText('Pause')
            self.states.setText('Running')
    def closeEvent(self,event):
        try:
            Thread.stop_thread(self.video)
            Thread.stop_thread(self.instruction)
        except:
            pass
        try:
            self.server.server_socket.shutdown(2)
            self.server.server_socket1.shutdown(2)
            self.server.stop_server()
        except:
            pass
        if self.user_ui:
            QCoreApplication.instance().quit()
        os._exit(0)
        
if __name__ == '__main__':
    try:
        myshow=MyWindow()
        if myshow.user_ui==True:
            myshow.show();   
            sys.exit(myshow.app.exec_())
        else:
            try:
                pass
            except KeyboardInterrupt:
                myshow.close()
        while True:
            pass
    except KeyboardInterrupt:
            myshow.close()
