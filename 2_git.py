import sys
import toupcam
import numpy as np
import cv2
import time
import serial
from PyQt5.QtCore import pyqtSignal, QTimer, Qt, QThread, QSignalBlocker
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QLabel, QApplication, QWidget, QCheckBox, QMessageBox, QPushButton, QComboBox, QSlider, QGroupBox, QGridLayout, QHBoxLayout, QVBoxLayout, QMenu, QAction

ser = serial.Serial('COM4', 115200, timeout=1, write_timeout=5)

class AutoFocus:
    def __init__(self):
        pass

    def rgb_to_gray(self, image):
        return (image[:, :, 0] * 38 + image[:, :, 1] * 75 + image[:, :, 2] * 15) >> 7

    def max_dfd(self, gray_image1, gray_image2, delta_u):
        V1 = gray_image1.astype(np.float32)
        V2 = gray_image2.astype(np.float32)
        ratio = np.sqrt(V2 / (V1 + 1e-10))  # 防止除零

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
        self.realtime = 0
        self.still_image_ready = False

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
        self.btn_focus.setEnabled(False)
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
        self.cmb_res.setEnabled(False)
        self.cmb_res.clear()

    def closeEvent(self, event):
        self.closeCamera()

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
            self.hcam.put_RealTime(1)
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
            auto_focus = AutoFocus()
            gray_images = []

            # 粗逼近步骤
            delta_u = 1000  # 初始步长
            threshold = 100  # 进入细调的阈值
            u_real_max_min = float('inf')

            while u_real_max_min > threshold:
                hresult = self.hcam.Snap(1)
                if hresult != 0:
                    raise RuntimeError(f"Snap failed with HRESULT: {hresult} at getting figure 1")
                hresult = 0
                
                start_time = time.time()
                while not self.still_image_ready:
                    if time.time() - start_time > 1:
                        raise RuntimeError("Timeout waiting for still image")
                    time.sleep(0.01)
                
                self.still_image_ready = False
                image1 = np.frombuffer(self.snap_buf, dtype=np.uint8).reshape((self.hcam.get_Size()[1], self.hcam.get_Size()[0], 3))
                gray_image1 = auto_focus.rgb_to_gray(image1)

                self.send_gcommand(f"G91 G1 Z{delta_u} F31000")
                time.sleep(0.1)

                hresult = self.hcam.Snap(1)
                if hresult != 0:
                    raise RuntimeError(f"Snap failed with HRESULT: {hresult} at getting figure 2")
                hresult = 0
                
                start_time = time.time()
                while not self.still_image_ready:
                    if time.time() - start_time > 1:
                        raise RuntimeError("Timeout waiting for still image")
                    time.sleep(0.01)

                self.still_image_ready = False
                image2 = np.frombuffer(self.snap_buf, dtype=np.uint8).reshape((self.hcam.get_Size()[1], self.hcam.get_Size()[0], 3))
                gray_image2 = auto_focus.rgb_to_gray(image2)

                u_real_max_min = auto_focus.max_dfd(gray_image1, gray_image2, delta_u)
                delta_u = u_real_max_min / 10  

            # 细调步骤
            gray_images.append(gray_image1)
            gray_images.append(gray_image2)
            self.fine_approach(auto_focus, gray_images, direction=-1)

    def fine_approach(self, auto_focus, gray_images, direction):
        max_iterations = 50
        step_size = 10  # 设定步长

        FV_values = [auto_focus.calculate_FV(gray_images[0]), auto_focus.calculate_FV(gray_images[1])]
        positions = [0, -step_size]

        for i in range(max_iterations):
            if FV_values[0] > FV_values[1]:
                direction *= -1

            # 移动并获取新图像
            self.send_gcommand(f"G91 G1 Z{direction * step_size} F31000")
            time.sleep(0.05)

            self.hcam.Snap(1)
            start_time = time.time()
            while not self.still_image_ready:
                if time.time() - start_time > 5:
                    raise RuntimeError("Timeout waiting for still image")
                time.sleep(0.05)

            self.still_image_ready = False
            image = np.frombuffer(self.snap_buf, dtype=np.uint8).reshape((self.hcam.get_Size()[1], self.hcam.get_Size()[0], 3))
            gray_image = auto_focus.rgb_to_gray(image)
            new_FV = auto_focus.calculate_FV(gray_image)

            FV_values = [FV_values[1], new_FV]
            positions = [positions[1], positions[1] + direction * step_size]

            if FV_values[0] < FV_values[1]:
                break

        # 二分法确定极小值
        low_pos, high_pos = positions[0], positions[1]
        for _ in range(int(np.log2(high_pos - low_pos)) + 1):
            mid_pos = (low_pos + high_pos) / 2
            self.send_gcommand(f"G90 G1 Z{mid_pos} F31000")
            time.sleep(0.05)

            self.hcam.Snap(1)
            start_time = time.time()
            while not self.still_image_ready:
                if time.time() - start_time > 5:
                    raise RuntimeError("Timeout waiting for still image")
                time.sleep(0.05)

            self.still_image_ready = False
            image = np.frombuffer(self.buf, dtype=np.uint8).reshape((self.hcam.get_Size()[1], self.hcam.get_Size()[0], 3))
            gray_image = auto_focus.rgb_to_gray(image)
            mid_FV = auto_focus.calculate_FV(gray_image)

            if mid_FV < FV_values[1]:
                high_pos = mid_pos
            else:
                low_pos = mid_pos

        print("Fine approach completed.")

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
            image = QImage(self.pData, self.imgWidth, self.imgHeight, QImage.Format_RGB888)
            newimage = image.scaled(self.lbl_video.width(), self.lbl_video.height(), Qt.KeepAspectRatio, Qt.FastTransformation)
            self.lbl_video.setPixmap(QPixmap.fromImage(newimage))

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
            self.hcam.PullStillImageV2(self.snap_buf, 24, None, info) # 获取静态图像
        except toupcam.HRESULTException:
            pass
        else:
            print("Snap data accepted")

    def send_gcommand(self, command):
        ser.write(command.encode())
        ser.flush()


if __name__ == '__main__':
    toupcam.Toupcam.GigeEnable(None, None)
    app = QApplication(sys.argv)
    mw = MainWidget()
    mw.show()
    sys.exit(app.exec_())
