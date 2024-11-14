import sys
import usb.core
import usb.util
import numpy as np
import time
import serial

ser = serial.Serial('COM4', 115200, timeout=1, write_timeout=5)

class PositionWorker():

    def __init__(self):
        super().__init__()
        self.running = False
        self.device = usb.core.find(idVendor=0x054C, idProduct=0x0061)
        if self.device is None:
            raise ValueError('Device not found')
        usb.util.claim_interface(self.device, 0)
        self.device.set_configuration()
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0

    def run(self):
        self.running = True
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)

        while self.running:
            try:
                z_dis = float(input("请输入Z轴移动距离 (输入非数字以停止): "))
                self.move_z_axis(z_dis)
            except ValueError:
                print("输入无效，程序即将停止...")
                self.stop()
            except KeyboardInterrupt:
                print("\n用户中止程序。")
                self.stop()

    def stop(self):
        self.running = False

    def send_gcode_command(self, command, need_ret=True):
        if need_ret:
            ser.read_all()

        ser.write(command.encode())
        
        if need_ret:
            ret = ser.readline()
            print('command=', command, 'ret=', ret)
            return ret

    def move_z_axis(self, z_dis):
        command = f'G0G91Z{z_dis}\n'
        self.send_gcode_command(command)

def main():
    worker = PositionWorker()
    
    try:
        worker.run()
    except KeyboardInterrupt:
        print("\n程序被用户终止")
    finally:
        worker.stop()

if __name__ == "__main__":
    main()
