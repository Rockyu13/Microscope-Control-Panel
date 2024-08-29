import sys
import usb.core
import usb.util
import serial
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QLabel, QWidget, QGridLayout, QTextEdit
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QColor, QPalette
from math import *

# USB HID 设备的供应商ID和产品ID
VENDOR_ID = 0x054C  # 替换为你的设备供应商ID
PRODUCT_ID = 0x0061  # 替换为你的设备产品ID

# 初始化串口通信
ser = serial.Serial('COM4', 115200, timeout=1, write_timeout=5)

class Worker(QThread):
    update_position = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self.running = False
        self.device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        if self.device is None:
            raise ValueError('Device not found')
        usb.util.claim_interface(self.device, 0)
        self.device.set_configuration()
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0
        self.flag = 0  # 用于控制键盘输入的状态
        self.home = 0
        self.alarmed = 0  # True if the system becomes in alarm status. Need to press button to reset
        self.cancel = 0
        self.move = 0
        self.x_min = 2
        self.x_max = 982
        self.y_min = 46
        self.y_max = 1023
        self.z_min = 63
        self.z_max = 1023

    def run(self):
        self.running = True
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)

        # Set reporting mask to 1
        self.send_gcode_command('$10=1\n')
        
        # 8/11/2024 Yuan: make sure it stays ON at initialization
        self.send_keep_on()
        self.flag = 1
        time_step = 0.04 # Yuan 8/11
        last_time = time.time()
        time_started = last_time
        while self.running:
            data = self.read_hid_data()
            if data:
                keyboard_input = self.check_keyboard(data)
                if keyboard_input == 1 and self.flag != 1:
                    self.send_keep_on()
                    self.flag = 1
                    print('on1')
                elif keyboard_input == 2 and self.flag != 2:
                    self.send_keep_off()
                    self.flag = 2
                    print('off')
                
                if keyboard_input == 5 and self.home == 0:
                    self.send_homing_command()
                    self.home = 1

                if keyboard_input == 7 and self.alarmed == 1:
                    self.send_reset_alarm()
                    self.alarmed = 0

                x = self.parse_axis_data(data[0], data[1])
                y = self.parse_axis_data(data[2], data[3])
                z = self.parse_axis_data(data[4], data[5])

                x = self.x_map(x)
                y = self.y_map(y)
                z = self.z_map(z)

                # Yuan edit 8/11/2024: Check of cancel flag is moved inside. Otherwise the program keeps sending zero jogs even when joystick not moving
                # If in alarmed status, no more moving
                if (abs(x - 0x0200) < 0x0020 and abs(y - 0x0200) < 0x0020 and abs(z - 0x0200) < 0x0010) or self.alarmed:
                    if self.cancel == 0:
                        ser.reset_input_buffer()
                        self.send_jog_cancel()
                        self.update_position_from_device() # Yuan 8/11/2024: update once jog has stopped
                        self.cancel = 1
                        time.sleep(0.1) # Yuan 8/11/2024: simply add a wait. This could be done better by checking "Idle" status
                    #print('no move')
                else:  # Yuan 8/11/2024 changed to if/else so that there will be 20ms delay even if no move
                    print('move')
                    gcode_command = self.process_data(x, y, z)
                    if gcode_command:
                        # 8/11/2024 Yuan: make sure it stays ON once a move has been issued
                        if self.flag != 1:
                            self.send_keep_on()
                            self.flag = 1

                        self.send_gcode_command(gcode_command, False)
                        self.cancel = 0
                        self.move = 1
                        self.home = 0

                        status = self.send_status_request()
                        
                        if status.find('Pn:') != -1:
                            self.alarmed = 1
                        #print(status)

                    self.update_position.emit(self.x_pos, self.y_pos, self.z_pos)

            # Yuan 8/11/2024: Use sleep_until logic to make sure step size is consistent
            time_now = time.time()
            time_elapsed = (time_now - last_time)
            time.sleep(max(0, time_step - time_elapsed))
            last_time = time.time()
            print((last_time-time_started)*1000)

    def stop(self):
        self.running = False


    # Yuan: 8/11 read until nothing can be read
    def read_hid_data(self):
        data = None
        while True:
            try:
                data = self.device.read(0x81, 10, 1) # Added a timeout of 1ms. If no data is available within 1ms, then the queue is empty, quit
            except usb.core.USBError as e:
                if data == None:
                    continue
                else:
                    break
        #print(data)
        return data
    '''
    def write_hid_data(self, data):
        try:
            self.device.write(0x81, data)
        except usb.core.USBError as e:
            raise
    
    def set_keyboard_led(self, led1, led2, led3):
        D1 = 0
        if led1:
            D1 |= 1
        if led2:
            D1 |= 2
        if led3:
            D1 |= 4
        data = [0xF5, D1, 0, 0, 0, 0, 0, 0]
        self.write_hid_data(data)
    '''
    def parse_axis_data(self, low_byte, high_byte):
        return (high_byte << 8) | low_byte

    def map_value(self, value, min_input, max_input, range):
        return 8 * range * ((value - (min_input + max_input) / 2) ** 3) / ((max_input - min_input) ** 3)

    def map_value_z(self, value, min_input, max_input, range):
        return 16 * range * ((value - (min_input + max_input) / 2) ** 4) / ((max_input - min_input) ** 4) * ((value - (min_input + max_input) / 2) / abs(value - (min_input + max_input) / 2))

    def process_data(self, x, y, z):
        x_speed = -self.map_value(x, 0x0000, 0x03FF, 4000)
        y_speed = self.map_value(y, 0x0000, 0x03FF, 4000)
        z_speed = self.map_value_z(z, 0x0000, 0x03FF, 31000)

        tot_speed = (x_speed ** 2 + y_speed ** 2 + z_speed ** 2) ** 0.5

        # Yuan: 8/11/2024: use consistent step size
        dt = 0.055 / 60
        dtz = 0.055 / 60

        x_disp = x_speed * dt
        y_disp = y_speed * dt
        z_disp = z_speed * dtz

        self.x_pos += x_disp
        self.y_pos += y_disp
        self.z_pos += z_disp

        gcode_command = f'$J=G21G91X{x_disp:.3f}Y{y_disp:.3f}Z{z_disp:.3f}F{tot_speed:.1f}\n'
        return gcode_command

    def send_gcode_command(self, command, need_ret=True):
        if need_ret:
            ser.read_all() # Make sure there is nothing to read

        ser.write(command.encode())
        
        if need_ret:
            ret = ser.readline()
            print('command=',command, 'ret=', ret)
            return ret

    def send_jog_cancel(self):
        ser.write(b'\x85')
        ser.flush()

    def send_keep_on(self):
        self.send_gcode_command('$1=255\n')
        self.send_gcode_command('G0G91X0.01\n') # Yuan 8/11/2024: Dummy move to turn on motor
        self.send_gcode_command('G0G91X-0.01\n') # Yuan 8/11/2024: Dummy move to turn on motor
        # self.set_keyboard_led(True, False, False)

    def send_keep_off(self):
        self.send_gcode_command('$1=100\n')
        self.send_gcode_command('G0G91X0.01\n') # Yuan 8/11/2024: Dummy move to turn on motor
        self.send_gcode_command('G0G91X-0.01\n') # Yuan 8/11/2024: Dummy move to turn on motor
        # self.set_keyboard_led(False, False, False)

    def send_homing_command(self):
        retries = 1
        for _ in range(retries):
            try:
                ser.write(f'$H\n'.encode())
                ser.flush()
                time.sleep(3)  # 等待归零完成
                self.update_position_from_device()
                self.update_position_from_device()
                break
            except serial.SerialTimeoutException:
                continue

    def send_reset_alarm(self):
        self.send_gcode_command('$X\n')
        print('Reset alarm')

    def send_status_request(self):
        return self.send_gcode_command('?\n').decode().strip()

    def check_keyboard(self, data):
        if data[6] == 0x01:
            return 1
        elif data[6] == 0x02:
            return 2
        elif data[6] == 0x04:
            return 3
        elif data[6] == 0x08:
            return 4
        elif data[6] == 0x10:
            return 5
        elif data[6] == 0x20:
            return 6
        elif data[6] == 0x40:
            return 7
        elif data[6] == 0x80:
            return 8

    def x_map(self, x):
        if x < 512:
            return 512 - 512 * (512 - x) / (512 - self.x_min)
        if x > 512:
            return 512 + 512 * (512 - x) / (512 - self.x_max)
        return 512

    def y_map(self, y):
        if y < 512:
            return 512 - 512 * (512 - y) / (512 - self.y_min)
        if y > 512:
            return 512 + 512 * (512 - y) / (512 - self.y_max)
        return 512
    
    def z_map(self, z):
        if z < 512:
            return 512 - 512 * (512 - z) / (512 - self.z_min)
        if z > 512:
            return 512 + 512 * (512 - z) / (512 - self.z_max)
        return 512

    def update_position_from_device(self):
        response = self.send_status_request()
        if response.startswith('<'):
            positions = self.parse_position_response(response)
            self.update_position.emit(*positions)

    def parse_position_response(self, response):
        # 假设响应格式为 '<Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000>'
        mpos_start = response.find('MPos:') + len('MPos:')
        mpos_end = response.find('|', mpos_start)
        mpos_str = response[mpos_start:mpos_end]
        mpos = list(map(float, mpos_str.split(',')))
        self.x_pos, self.y_pos, self.z_pos = mpos
        return self.x_pos, self.y_pos, self.z_pos


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("CNC Control")
        self.setGeometry(100, 100, 800, 400)

        self.worker = Worker()
        self.worker.update_position.connect(self.update_display)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        self.start_button = QPushButton("Start", self)
        self.start_button.clicked.connect(self.start_worker)
        layout.addWidget(self.start_button)

        self.close_button = QPushButton("Close", self)
        self.close_button.clicked.connect(self.close_application)
        layout.addWidget(self.close_button)

        self.kill_alarm_button = QPushButton("Kill Alarm", self)
        self.kill_alarm_button.clicked.connect(self.kill_alarm)
        self.kill_alarm_button.setEnabled(False)
        layout.addWidget(self.kill_alarm_button)

        self.homing_button = QPushButton("Homing", self)
        self.homing_button.clicked.connect(self.homing)
        self.homing_button.setEnabled(False)
        layout.addWidget(self.homing_button)

        self.lcd_layout = QGridLayout()

        self.x_label, self.x_lcd = self.create_lcd_display("X")
        self.y_label, self.y_lcd = self.create_lcd_display("Y")
        self.z_label, self.z_lcd = self.create_lcd_display("Z")

        self.lcd_layout.addWidget(self.x_label, 0, 0)
        self.lcd_layout.addWidget(self.x_lcd, 0, 1)
        self.lcd_layout.addWidget(self.y_label, 1, 0)
        self.lcd_layout.addWidget(self.y_lcd, 1, 1)
        self.lcd_layout.addWidget(self.z_label, 2, 0)
        self.lcd_layout.addWidget(self.z_lcd, 2, 1)

        layout.addLayout(self.lcd_layout)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def create_lcd_display(self, axis):
        label = QLabel(f"{axis}\u2080", self)
        label.setAlignment(Qt.AlignCenter)
        label.setFont(QFont("Courier", 24))
        label.setAutoFillBackground(True)
        label.setFixedWidth(100)
        palette = label.palette()
        palette.setColor(QPalette.Window, QColor("gray"))
        palette.setColor(QPalette.WindowText, QColor("blue"))
        label.setPalette(palette)

        lcd = QLabel("0.000", self)
        lcd.setAlignment(Qt.AlignCenter)
        lcd.setFont(QFont("Courier", 24))
        lcd.setAutoFillBackground(True)
        palette = lcd.palette()
        palette.setColor(QPalette.Window, QColor("gray"))
        palette.setColor(QPalette.WindowText, QColor("blue"))
        lcd.setPalette(palette)

        return label, lcd

    def start_worker(self):
        self.worker.start()
        self.worker.update_position_from_device()
        self.start_button.setEnabled(False)
        self.kill_alarm_button.setEnabled(True)
        self.homing_button.setEnabled(True)

    def close_application(self):
        self.worker.stop()
        self.worker.wait()
        self.close()

    def kill_alarm(self):
        self.worker.send_reset_alarm()

    def homing(self):
        self.worker.send_homing_command()

    def update_display(self, x, y, z):
        self.x_lcd.setText(f"{x:07.3f}")
        self.y_lcd.setText(f"{y:07.3f}")
        self.z_lcd.setText(f"{z:08.3f}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
