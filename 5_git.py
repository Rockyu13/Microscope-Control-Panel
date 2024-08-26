import sys
import usb.core
import usb.util
import toupcam
import numpy as np
import cv2
import time
import serial
from PyQt5.QtCore import pyqtSignal, QTimer, Qt, QSignalBlocker, QThread
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QLabel, QApplication, QWidget, QCheckBox, QMessageBox, QPushButton, QComboBox, QSlider, QGroupBox, QGridLayout, QHBoxLayout, QVBoxLayout, QMenu, QAction

ser = serial.Serial('COM4', 115200, timeout=1, write_timeout=5)

class AutoFocus:
    def __init__(self):
        pass

    def rgb_to_gray(self, image):
        gray_image = np.dot(image[..., :3], [38, 75, 15]) >> 7
        return gray_image.astype(np.uint8)

    def max_dfd(self, gray_image1, gray_image2, delta_u):
        V1 = gray_image1.astype(np.float32)
        V2 = gray_image2.astype(np.float32)
        ratio = np.sqrt(V2 / (V1 + 1e-10)) 

        u_real_max = delta_u * ratio / (1 - ratio)
        u_real_max_min = np.min(u_real_max)

        print(f"Min of Max DFD: {u_real_max_min:.6f} m")
        return u_real_max_min

    def calculate_FV(self, gray_image):
        alpha = 0.5
        _, otsu_threshold = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        T = alpha * otsu_threshold

        kernel = np.array([[1, -2, 1], [-2, 4, -2], [1, -2, 1]])
        variance = cv2.filter2D(gray_image.astype(np.float32), -1, kernel)

        max_gradient = self.calculate_max_gradient(gray_image)

        mask = variance > T
        weighted_gradients = max_gradient[mask] ** 2
        FV = np.sum(weighted_gradients)

        return FV

    def calculate_max_gradient(self, gray_image):
        W1 = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
        W2 = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])
        W3 = np.array([[0, 1, 2], [-1, 0, 1], [-2, -1, 0]])
        W4 = np.array([[-2, -1, 0], [-1, 0, 1], [0, 1, 2]])

        G1 = np.abs(cv2.filter2D(gray_image, -1, W1))
        G2 = np.abs(cv2.filter2D(gray_image, -1, W2))
        G3 = np.abs(cv2.filter2D(gray_image, -1, W3))
        G4 = np.abs(cv2.filter2D(gray_image, -1, W4))

        max_gradient = np.maximum(np.maximum(G1, G2), np.maximum(G3, G4))
        return max_gradient

class FocusThread(QThread):
    finished = pyqtSignal()
    update_image_signal = pyqtSignal(np.ndarray)

    def __init__(self, main_widget):
        super().__init__()
        self.main_widget = main_widget
        self.is_running = True

    def run(self):
        self.main_widget.rough_approach(100, 50, 10)  # 添加 rough_approach 调用
        self.main_widget.fine_approach(5, 20)
        self.main_widget.fine_approach(1, 10)
        self.finished.emit()

    def stop(self):
        self.is_running = False

class MainWidget(QWidget):
    evtCallback = pyqtSignal(int)

    @staticmethod
    def makeLayout(lbl1, sli1, val1, lbl2, sli2, val2):
        hlyt1 = QHBoxLayout()
        hlyt1.addWidget(lbl1)
        hlyt1.addStretch()
        hlyt1.addWidget(val1)
        hlyt2 = QHBoxLayout()
        hlyt2.addWidget(lbl2)
        hlyt2.addStretch()
        hlyt2.addWidget(val2)
        vlyt = QVBoxLayout()
        vlyt.addLayout(hlyt1)
        vlyt.addWidget(sli1)
        vlyt.addLayout(hlyt2)
        vlyt.addWidget(sli2)
        return vlyt

    def __init__(self):
        super().__init__()
        self.setMinimumSize(1024, 768)
        self.hcam = None
        self.timer = QTimer(self)
        self.imgWidth = 0
        self.imgHeight = 0
        self.pData = None
        self.res = 0
        self.temp = toupcam.TOUPCAM_TEMP_DEF
        self.tint = toupcam.TOUPCAM_TINT_DEF
        self.count = 0
        self.last_image = None
        self.last_image_flag = 0 #Flag to monitor whether last_image is active
        self.focus_thread = None  # Initialize focus_thread attribute

        gboxres = QGroupBox("Resolution")
        self.cmb_res = QComboBox()
        self.cmb_res.setEnabled(False)
        vlytres = QVBoxLayout()
        vlytres.addWidget(self.cmb_res)
        gboxres.setLayout(vlytres)
        self.cmb_res.currentIndexChanged.connect(self.onResolutionChanged)

        gboxexp = QGroupBox("Exposure")
        self.cbox_auto = QCheckBox("Auto exposure")
        self.cbox_auto.setEnabled(False)
        self.lbl_expoTime = QLabel("0")
        self.lbl_expoGain = QLabel("0")
        self.slider_expoTime = QSlider(Qt.Horizontal)
        self.slider_expoGain = QSlider(Qt.Horizontal)
        self.slider_expoTime.setEnabled(False)
        self.slider_expoGain.setEnabled(False)
        self.cbox_auto.stateChanged.connect(self.onAutoExpo)
        self.slider_expoTime.valueChanged.connect(self.onExpoTime)
        self.slider_expoGain.valueChanged.connect(self.onExpoGain)
        vlytexp = QVBoxLayout()
        vlytexp.addWidget(self.cbox_auto)
        vlytexp.addLayout(self.makeLayout(QLabel("Time(us):"), self.slider_expoTime, self.lbl_expoTime, QLabel("Gain(%):"), self.slider_expoGain, self.lbl_expoGain))
        gboxexp.setLayout(vlytexp)

        gboxwb = QGroupBox("White balance")
        self.btn_autoWB = QPushButton("White balance")
        self.btn_autoWB.setEnabled(False)
        self.btn_autoWB.clicked.connect(self.onAutoWB)
        self.lbl_temp = QLabel(str(toupcam.TOUPCAM_TEMP_DEF))
        self.lbl_tint = QLabel(str(toupcam.TOUPCAM_TINT_DEF))
        self.slider_temp = QSlider(Qt.Horizontal)
        self.slider_tint = QSlider(Qt.Horizontal)
        self.slider_temp.setRange(toupcam.TOUPCAM_TEMP_MIN, toupcam.TOUPCAM_TEMP_MAX)
        self.slider_temp.setValue(toupcam.TOUPCAM_TEMP_DEF)
        self.slider_tint.setRange(toupcam.TOUPCAM_TINT_MIN, toupcam.TOUPCAM_TINT_MAX)
        self.slider_tint.setValue(toupcam.TOUPCAM_TINT_DEF)
        self.slider_temp.setEnabled(False)
        self.slider_tint.setEnabled(False)
        self.slider_temp.valueChanged.connect(self.onWBTemp)
        self.slider_tint.valueChanged.connect(self.onWBTint)
        vlytwb = QVBoxLayout()
        vlytwb.addLayout(self.makeLayout(QLabel("Temperature:"), self.slider_temp, self.lbl_temp, QLabel("Tint:"), self.slider_tint, self.lbl_tint))
        vlytwb.addWidget(self.btn_autoWB)
        gboxwb.setLayout(vlytwb)

        self.btn_open = QPushButton("Open")
        self.btn_open.clicked.connect(self.onBtnOpen)
        self.btn_snap = QPushButton("Snap")
        self.btn_snap.setEnabled(False)
        self.btn_snap.clicked.connect(self.onBtnSnap)
        self.btn_focus = QPushButton("Focus")
        self.btn_focus.clicked.connect(self.onBtnFocus)
        vlytctrl = QVBoxLayout()
        vlytctrl.addWidget(gboxres)
        vlytctrl.addWidget(gboxexp)
        vlytctrl.addWidget(gboxwb)
        vlytctrl.addWidget(self.btn_open)
        vlytctrl.addWidget(self.btn_snap)
        vlytctrl.addWidget(self.btn_focus)
        vlytctrl.addStretch()
        wgctrl = QWidget()
        wgctrl.setLayout(vlytctrl)

        self.lbl_frame = QLabel()
        self.lbl_video = QLabel()
        vlytshow = QVBoxLayout()
        vlytshow.addWidget(self.lbl_video, 1)
        vlytshow.addWidget(self.lbl_frame)
        wgshow = QWidget()
        wgshow.setLayout(vlytshow)

        gmain = QGridLayout()
        gmain.setColumnStretch(0, 1)
        gmain.setColumnStretch(1, 4)
        gmain.addWidget(wgctrl)
        gmain.addWidget(wgshow)
        self.setLayout(gmain)

        self.timer.timeout.connect(self.onTimer)
        self.evtCallback.connect(self.onevtCallback)

    def onTimer(self):
        if self.hcam:
            nFrame, nTime, nTotalFrame = self.hcam.get_FrameRate()
            self.lbl_frame.setText("{}, fps = {:.1f}".format(nTotalFrame, nFrame * 1000.0 / nTime))

    def closeCamera(self):
        if self.hcam:
            self.hcam.Close()
        self.hcam = None
        self.pData = None

        self.btn_open.setText("Open")
        self.timer.stop()
        self.lbl_frame.clear()
        self.cbox_auto.setEnabled(False)
        self.slider_expoGain.setEnabled(False)
        self.slider_expoTime.setEnabled(False)
        self.btn_autoWB.setEnabled(False)
        self.slider_temp.setEnabled(False)
        self.slider_tint.setEnabled(False)
        self.btn_snap.setEnabled(False)
        self.btn_focus.setEnabled(False)
        self.cmb_res.setEnabled(False)
        self.cmb_res.clear()

    def closeEvent(self, event):
        if self.focus_thread is not None and self.focus_thread.isRunning():
            self.focus_thread.stop()
            self.focus_thread.wait()
        self.closeCamera()
        event.accept()

    def onResolutionChanged(self, index):
        if self.hcam: #step 1: stop camera
            self.hcam.Stop()

        self.res = index
        self.imgWidth = self.cur.model.res[index].width
        self.imgHeight = self.cur.model.res[index].height

        if self.hcam: #step 2: restart camera
            self.hcam.put_eSize(self.res)
            self.startCamera()

    def onAutoExpo(self, state):
        if self.hcam:
            self.hcam.put_AutoExpoEnable(1 if state else 0)
            self.slider_expoTime.setEnabled(not state)
            self.slider_expoGain.setEnabled(not state)

    def onExpoTime(self, value):
        if self.hcam:
            self.lbl_expoTime.setText(str(value))
            if not self.cbox_auto.isChecked():
                self.hcam.put_ExpoTime(value)

    def onExpoGain(self, value):
        if self.hcam:
            self.lbl_expoGain.setText(str(value))
            if not self.cbox_auto.isChecked():
                self.hcam.put_ExpoAGain(value)

    def onAutoWB(self):
        if self.hcam:
            self.hcam.AwbOnce()

    def wbCallback(nTemp, nTint, self):
        self.slider_temp.setValue(nTemp)
        self.slider_tint.setValue(nTint)

    def onWBTemp(self, value):
        if self.hcam:
            self.temp = value
            self.hcam.put_TempTint(self.temp, self.tint)
            self.lbl_temp.setText(str(value))

    def onWBTint(self, value):
        if self.hcam:
            self.tint = value
            self.hcam.put_TempTint(self.temp, self.tint)
            self.lbl_tint.setText(str(value))

    def startCamera(self):
        self.pData = bytes(toupcam.TDIBWIDTHBYTES(self.imgWidth * 24) * self.imgHeight)
        uimin, uimax, uidef = self.hcam.get_ExpTimeRange()
        self.slider_expoTime.setRange(uimin, uimax)
        self.slider_expoTime.setValue(uidef)
        usmin, usmax, usdef = self.hcam.get_ExpoAGainRange()
        self.slider_expoGain.setRange(usmin, usmax)
        self.slider_expoGain.setValue(usdef)
        self.handleExpoEvent()
        if self.cur.model.flag & toupcam.TOUPCAM_FLAG_MONO == 0:
            self.handleTempTintEvent()
        try:
            self.hcam.StartPullModeWithCallback(self.eventCallBack, self)
        except toupcam.HRESULTException:
            self.closeCamera()
            QMessageBox.warning(self, "Warning", "Failed to start camera.")
        else:
            self.cmb_res.setEnabled(True)
            self.cbox_auto.setEnabled(True)
            self.btn_autoWB.setEnabled(self.cur.model.flag & toupcam.TOUPCAM_FLAG_MONO == 0)
            self.slider_temp.setEnabled(self.cur.model.flag & toupcam.TOUPCAM_FLAG_MONO == 0)
            self.slider_tint.setEnabled(self.cur.model.flag & toupcam.TOUPCAM_FLAG_MONO == 0)
            self.btn_open.setText("Close")
            self.btn_snap.setEnabled(True)
            self.btn_focus.setEnabled(True)
            bAuto = self.hcam.get_AutoExpoEnable()
            self.cbox_auto.setChecked(1 == bAuto)
            self.timer.start(1000)

    def openCamera(self):
        self.hcam = toupcam.Toupcam.Open(self.cur.id)
        if self.hcam:
            self.res = self.hcam.get_eSize()
            self.imgWidth = self.cur.model.res[self.res].width
            self.imgHeight = self.cur.model.res[self.res].height
            with QSignalBlocker(self.cmb_res):
                self.cmb_res.clear()
                for i in range(0, self.cur.model.preview):
                    self.cmb_res.addItem("{}*{}".format(self.cur.model.res[i].width, self.cur.model.res[i].height))
                self.cmb_res.setCurrentIndex(self.res)
                self.cmb_res.setEnabled(True)
            self.hcam.put_Option(toupcam.TOUPCAM_OPTION_BYTEORDER, 0) #Qimage use RGB byte order
            self.hcam.put_AutoExpoEnable(1)
            self.startCamera()

    def onBtnOpen(self):
        if self.hcam:
            self.closeCamera()
        else:
            arr = toupcam.Toupcam.EnumV2()
            if 0 == len(arr):
                QMessageBox.warning(self, "Warning", "No camera found.")
            elif 1 == len(arr):
                self.cur = arr[0]
                self.openCamera()
            else:
                menu = QMenu()
                for i in range(0, len(arr)):
                    action = QAction(arr[i].displayname, self)
                    action.setData(i)
                    menu.addAction(action)
                action = menu.exec(self.mapToGlobal(self.btn_open.pos()))
                if action:
                    self.cur = arr[action.data()]
                    self.openCamera()

    def onBtnSnap(self):
        if self.hcam:
            if 0 == self.cur.model.still:    # not support still image capture
                if self.pData is not None:
                    image = QImage(self.pData, self.imgWidth, self.imgHeight, QImage.Format_RGB888)
                    self.count += 1
                    image.save("pyqt{}.jpg".format(self.count))
            else:
                menu = QMenu()
                for i in range(0, self.cur.model.still):
                    action = QAction("{}*{}".format(self.cur.model.res[i].width, self.cur.model.res[i].height), self)
                    action.setData(i)
                    menu.addAction(action)
                action = menu.exec(self.mapToGlobal(self.btn_snap.pos()))
                self.hcam.Snap(action.data())
    
    def onBtnFocus(self):
        if self.hcam:
            # 如果当前有运行中的线程，先停止它
            if self.focus_thread is not None and self.focus_thread.isRunning():
                self.focus_thread.stop()
                self.focus_thread.wait()

            # 创建并启动新的Focus线程
            self.focus_thread = FocusThread(self)
            self.focus_thread.finished.connect(self.onFocusFinished)
            self.focus_thread.start()

    def onFocusFinished(self):
        print("Focus operation completed.")
        self.focus_thread = None  # Reset the thread variable

    def rough_approach(self, first_step, threshold, n):
        auto_focus = AutoFocus()
        delta_u = first_step
        u_real_max_min = float('inf')
        rough_iteration_count = 0

        while u_real_max_min >= threshold:
            image1 = self.last_image.copy()
            gray_image1 = auto_focus.rgb_to_gray(image1)

            self.send_gcommand(f'$J=G91Z{delta_u}F30000\n')
            print(f"Moving Z axis {delta_u} um")
            time.sleep(0.5)

            image2 = self.last_image.copy()
            gray_image2 = auto_focus.rgb_to_gray(image2)

            u_real_max_min = auto_focus.max_dfd(gray_image1, gray_image2, delta_u)
            delta_u = u_real_max_min / n

            rough_iteration_count += 1
            print(f"Rough Iteration: {rough_iteration_count}, Delta U: {delta_u}")
        
        print(f"Rough Approach Finished, current Max DFD: {u_real_max_min:.6f} m")

    def fine_approach(self, step_size, max_iteration):
        auto_focus = AutoFocus()
        FV_queue = np.zeros((3,), dtype=np.float32)
        direction = 1
        flag = 0

        image = self.last_image.copy()
        gray_image = auto_focus.rgb_to_gray(image)
        FV = auto_focus.calculate_FV(gray_image)
        FV_queue[-1] = FV

        for i in range(max_iteration):
            if not self.focus_thread.is_running:
                break  # If the thread is stopped, break out of the loop

            self.send_gcommand(f"$J=G91Z{step_size * direction}F31000\n")
            time.sleep(0.5)
            image = self.last_image.copy()
            gray_image = auto_focus.rgb_to_gray(image)
            FV = auto_focus.calculate_FV(gray_image)
            if FV > FV_queue[-1] and FV_queue[-1] != 0:
                direction = -direction
            FV_queue[:-1] = FV_queue[1:]
            FV_queue[-1] = FV
            if FV_queue[0] > FV_queue[1] and FV_queue[1] < FV_queue[2] and FV_queue[1] != 0 and FV_queue[0] != 0:
                flag = 1
                break
        
        if flag == 1:
            print(f"Fine Approach Finished")
        else:
            print(f"Fine Approach Failed, please change the parameters")

    @staticmethod
    def eventCallBack(nEvent, self):
        '''callbacks come from toupcam.dll/so internal threads, so we use qt signal to post this event to the UI thread'''
        self.evtCallback.emit(nEvent)

    def onevtCallback(self, nEvent):
        '''this run in the UI thread'''
        if self.hcam:
            if toupcam.TOUPCAM_EVENT_IMAGE == nEvent:
                self.handleImageEvent()
            elif toupcam.TOUPCAM_EVENT_EXPOSURE == nEvent:
                self.handleExpoEvent()
            elif toupcam.TOUPCAM_EVENT_TEMPTINT == nEvent:
                self.handleTempTintEvent()
            elif toupcam.TOUPCAM_EVENT_STILLIMAGE == nEvent:
                self.handleStillImageEvent()
            elif toupcam.TOUPCAM_EVENT_ERROR == nEvent:
                self.closeCamera()
                QMessageBox.warning(self, "Warning", "Generic Error.")
            elif toupcam.TOUPCAM_EVENT_STILLIMAGE == nEvent:
                self.closeCamera()
                QMessageBox.warning(self, "Warning", "Camera disconnect.")

    def handleImageEvent(self):
        try:
            self.hcam.PullImageV3(self.pData, 0, 24, 0, None)
        except toupcam.HRESULTException:
            pass
        else:
            image_copy = np.frombuffer(self.pData, dtype=np.uint8).reshape(self.imgWidth, self.imgHeight, 3).copy()
            self.display_image(image_copy)
            self.last_image = image_copy
            if (self.last_image is not None) and (self.last_image_flag == 0): #Check if it is activated
                print('Last image activated')
                self.last_image_flag = 1
            
    def display_image(self, image):
        qimage = QImage(image, self.imgWidth, self.imgHeight, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        self.lbl_video.setPixmap(pixmap)

    def handleExpoEvent(self):
        time = self.hcam.get_ExpoTime()
        gain = self.hcam.get_ExpoAGain()
        with QSignalBlocker(self.slider_expoTime):
            self.slider_expoTime.setValue(time)
        with QSignalBlocker(self.slider_expoGain):
            self.slider_expoGain.setValue(gain)
        self.lbl_expoTime.setText(str(time))
        self.lbl_expoGain.setText(str(gain))

    def handleTempTintEvent(self):
        nTemp, nTint = self.hcam.get_TempTint()
        with QSignalBlocker(self.slider_temp):
            self.slider_temp.setValue(nTemp)
        with QSignalBlocker(self.slider_tint):
            self.slider_tint.setValue(nTint)
        self.lbl_temp.setText(str(nTemp))
        self.lbl_tint.setText(str(nTint))

    def handleStillImageEvent(self):
        info = toupcam.ToupcamFrameInfoV3()
        try:
            self.hcam.PullImageV3(None, 1, 24, 0, info) # peek
        except toupcam.HRESULTException:
            pass
        else:
            if info.width > 0 and info.height > 0:
                buf = bytes(toupcam.TDIBWIDTHBYTES(info.width * 24) * info.height)
                try:
                    self.hcam.PullImageV3(buf, 1, 24, 0, info)
                except toupcam.HRESULTException:
                    pass
                else:
                    image = QImage(buf, info.width, info.height, QImage.Format_RGB888)
                    self.count += 1
                    image.save("pyqt{}.jpg".format(self.count))

    def send_gcommand(self, command, need_ret=True):
        if need_ret:
            ser.read_all() # Make sure there is nothing to read

        ser.write(command.encode())
        
        if need_ret:
            ret = ser.readline()
            print('command=',command, 'ret=', ret)
            return ret


if __name__ == '__main__':
    toupcam.Toupcam.GigeEnable(None, None)
    app = QApplication(sys.argv)
    mw = MainWidget()
    mw.show()
    sys.exit(app.exec_())
