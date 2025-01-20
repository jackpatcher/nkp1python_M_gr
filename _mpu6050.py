from machine import I2C, Pin
import time
from math import atan2, sqrt, degrees
from _ssd1306 import SSD1306_I2C

class KalmanFilter:
    def __init__(self, q_angle=0.001, q_bias=0.003, r_measure=0.01):
        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_measure = r_measure
        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0
        self.p = [[0.0, 0.0], [0.0, 0.0]]

    def update(self, new_rate, new_angle, dt):
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate

        self.p[0][0] += dt * (dt * self.p[1][1] - self.p[0][1] - self.p[1][0] + self.q_angle)
        self.p[0][1] -= dt * self.p[1][1]
        self.p[1][0] -= dt * self.p[1][1]
        self.p[1][1] += self.q_bias * dt

        s = self.p[0][0] + self.r_measure
        k = [self.p[0][0] / s, self.p[1][0] / s]

        y = new_angle - self.angle
        self.angle += k[0] * y
        self.bias += k[1] * y

        p00_temp = self.p[0][0]
        p01_temp = self.p[0][1]

        self.p[0][0] -= k[0] * p00_temp
        self.p[0][1] -= k[0] * p01_temp
        self.p[1][0] -= k[1] * p00_temp
        self.p[1][1] -= k[1] * p01_temp

        return self.angle

class MPU6050:
    def __init__(self, i2c, address=0x68):
        self.i2c = i2c
        self.address = address
        
        # เพิ่มตัวแปรสำหรับการรีเซ็ตมุม
        self.zero_reference = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        self.gyro_offsets = {'x': 0, 'y': 0, 'z': 0}
        self.kalman_filters = {
            'x': KalmanFilter(q_angle=0.001, q_bias=0.003, r_measure=0.01),
            'y': KalmanFilter(q_angle=0.001, q_bias=0.003, r_measure=0.01),
            'z': KalmanFilter(q_angle=0.001, q_bias=0.003, r_measure=0.01)
        }
        self.angles = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.time_prev = time.ticks_ms()

        # กระบวนการเริ่มต้น
        self.reset()
        time.sleep(0.2)
        self.configure()
        time.sleep(0.2)
        self.calibrate_gyro()
        self.zero_position()

    def reset(self):
        try:
#             self.i2c.writeto_mem(self.address, 0x6B, b'\x00')
            self.angles = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            print("รีเซ็ต MPU6050 สำเร็จ")
        except Exception as e:
            print(f"เกิดข้อผิดพลาดขณะรีเซ็ต: {e}")

    def configure(self):
        try:
            self.i2c.writeto_mem(self.address, 0x19, b'\x07')
            self.i2c.writeto_mem(self.address, 0x1A, b'\x06')
            self.i2c.writeto_mem(self.address, 0x1B, b'\x08')
            self.i2c.writeto_mem(self.address, 0x1C, b'\x10')
            print("ตั้งค่า MPU6050 เรียบร้อย")
        except Exception as e:
            print(f"เกิดข้อผิดพลาดขณะตั้งค่า: {e}")

    def calibrate_gyro(self):
        samples = 500
        self.gyro_offsets = {'x': 0, 'y': 0, 'z': 0}
        print("เริ่มปรับเทียบไจโร...")

        for _ in range(samples):
            self.gyro_offsets['x'] += self.read_raw_data(0x43)
            self.gyro_offsets['y'] += self.read_raw_data(0x45)
            self.gyro_offsets['z'] += self.read_raw_data(0x47)
            time.sleep(0.002)

        self.gyro_offsets = {k: v / samples for k, v in self.gyro_offsets.items()}
        print("ค่าชดเชยไจโร:", self.gyro_offsets)

    def zero_position(self):
        """ฟังก์ชันปรับตำแหน่งเริ่มต้นเป็นศูนย์"""
        print("กำลังปรับตำแหน่งเริ่มต้นเป็นศูนย์...")
        time.sleep(1)  # รอให้เซ็นเซอร์นิ่ง
        
        # เก็บค่ามุมปัจจุบันเป็นจุดอ้างอิง
        current_angles = self.read_current_angles()
        self.zero_reference = {
            'x': current_angles['x'],
            'y': current_angles['y'],
            'z': current_angles['z']
        }
        print("ปรับตำแหน่งเป็นศูนย์เรียบร้อย")

    def read_raw_data(self, addr):
        high = self.i2c.readfrom_mem(self.address, addr, 1)
        low = self.i2c.readfrom_mem(self.address, addr + 1, 1)
        value = (high[0] << 8) | low[0]
        return value - 65536 if value > 32768 else value

    def read_current_angles(self):
        """อ่านมุมปัจจุบันโดยตรง"""
        acc_x = self.read_raw_data(0x3B) / 16384.0
        acc_y = self.read_raw_data(0x3D) / 16384.0
        acc_z = self.read_raw_data(0x3F) / 16384.0

        gyro_x = (self.read_raw_data(0x43) - self.gyro_offsets['x']) / 131.0
        gyro_y = (self.read_raw_data(0x45) - self.gyro_offsets['y']) / 131.0
        gyro_z = (self.read_raw_data(0x47) - self.gyro_offsets['z']) / 131.0

        accel_angle_x = degrees(atan2(acc_y, acc_z))
        accel_angle_y = degrees(atan2(-acc_x, sqrt(acc_y**2 + acc_z**2)))

        return {
            'x': accel_angle_x, 
            'y': accel_angle_y, 
            'z': 0.0
        }

    def is_stationary(self, acc_x, acc_y, acc_z, threshold=0.05):
        magnitude = sqrt(acc_x**2 + acc_y**2 + acc_z**2)
        return abs(magnitude - 1.0) < threshold

    def update_angles(self):
        acc_x = self.read_raw_data(0x3B) / 16384.0
        acc_y = self.read_raw_data(0x3D) / 16384.0
        acc_z = self.read_raw_data(0x3F) / 16384.0

        gyro_x = (self.read_raw_data(0x43) - self.gyro_offsets['x']) / 131.0
        gyro_y = (self.read_raw_data(0x45) - self.gyro_offsets['y']) / 131.0
        gyro_z = (self.read_raw_data(0x47) - self.gyro_offsets['z']) / 131.0

        accel_angle_x = degrees(atan2(acc_y, acc_z))
        accel_angle_y = degrees(atan2(-acc_x, sqrt(acc_y**2 + acc_z**2)))

        time_now = time.ticks_ms()
        delta_time = (time_now - self.time_prev) / 1000.0
        self.time_prev = time_now

        angle_x = self.kalman_filters['x'].update(gyro_x, accel_angle_x, delta_time)
        angle_y = self.kalman_filters['y'].update(gyro_y, accel_angle_y, delta_time)
        angle_z = self.angles['z'] + gyro_z * delta_time

        # ใช้ค่าจากการคำนวณรวมกับการชดเชย
        self.angles['x'] = (angle_x - self.zero_reference['x'])
        self.angles['y'] = angle_y - self.zero_reference['y']
        self.angles['z'] = angle_z - self.zero_reference['z']

        return self.angles

    



