from machine import I2C, Pin
import time

class TCS3472:
    def __init__(self, i2c, address=0x29):
        self.i2c = i2c
        self.address = address
        self.enable()
    
    def enable(self):
        # เปิดใช้งานเซนเซอร์
        self._write_byte(0x80 | 0x00, 0x03)  # เปิด Power ON
        self._write_byte(0x80 | 0x01, 0x2B)  # เปิด ADC Integration
    
    def _write_byte(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value]))
    
    def _read_word(self, reg):
        data = self.i2c.readfrom_mem(self.address, reg, 2)
        return int.from_bytes(data, 'little')
    
    def get_raw_data(self):
        # อ่านค่า Raw Data จาก RGBC
        red = self._read_word(0x80 | 0x16)
        green = self._read_word(0x80 | 0x18)
        blue = self._read_word(0x80 | 0x1A)
        clear = self._read_word(0x80 | 0x14)
        return red, green, blue, clear
    
    def get_color(self):
        # แปลงค่าเป็นสัดส่วน
        red, green, blue, clear = self.get_raw_data()
        if clear == 0:
            return (0, 0, 0)  # ป้องกันการหารด้วย 0
        return (
            int((red / clear) * 255),
            int((green / clear) * 255),
            int((blue / clear) * 255),
        )
