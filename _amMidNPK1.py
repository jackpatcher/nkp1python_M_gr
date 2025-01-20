from _ssd1306 import SSD1306_I2C
from _MOTOR import Motor
from _Servo import Servo
#from _tcs3472 import TCS3472
import _mpu6050
from machine import Pin, PWM,ADC,I2C
from time import sleep,sleep_ms
import os,json
import _thread
import test_gyro



    

'''
--------------------------------------------------------------------------
            CLASS ห้ามแก้ไข
--------------------------------------------------------------------------
'''
class AmMid_NKP1:
    def __init__(self):
        print('..ready iam.geradt@gmail.com..')
        
    i2c = I2C(0, scl=Pin(22), sda=Pin(21))
    oled = SSD1306_I2C(128, 64)
    m = Motor(2,4,16,17)
    gyro = _mpu6050.MPU6050(i2c)
    
    sv1 = Servo(pin=23) ; sv2 = Servo(pin=19) ;sv3 = Servo(pin=18);sv4 = Servo(pin=5)

    #color = TCS3472(i2c) 




    
    # ตัวแปร sensor
    sensor = {
        "a0k": 0, "a1k": 0, "a2k": 0, "a3k": 0, "a4k": 0, "a5k": 0,
        "a0w": 0, "a1w": 0, "a2w": 0, "a3w": 0, "a4w": 0, "a5w": 0
    }

    # ตัวแปร mean
    mean = {
        "mA0": 0, "mA1": 0, "mA2": 0, "mA3": 0, "mA4": 0, "mA5": 0
    }
    
    MEAN_FILE = "mean_values.json"
    
    Plus_speedL=0;Plus_speedR=0;S=0
    
    # menu
    menu_options = ["0. start", "1. SetSensor", "2. testGyro", "3. readS", "4. xx"]
    current_menu = 0
    dice_count = 1
    flag_startGyro = False
    
    #motion
    kp =0.5 ; ki = 0;kd =0;integral =0 ;preError =0
    
    
    
    

    
    #gyro

    
    # analog sensor
    _A0  =   39 ; _A1  =   34 ;    _A2  =   32;    _A3  =   35;    _A4  =   25;    _A5  =   33
    

    # sound
    _BZ = 12
 
    # Knop Switch
    _KNOP = 36
    _SW = 15
    
    knop = ADC(Pin(_KNOP))
    knop.atten(ADC.ATTN_11DB)
    knop.width(ADC.WIDTH_12BIT)
    sw = Pin(_SW, Pin.IN, Pin.PULL_UP)

    # [[[[[[[[[[[    COLOR      ]]]]]]]]]]]
    def getColor(self):
        red, green, blue, clear = self.color.get_raw_data()
        cl= self.color.get_color()
        print(f"Raw Data - Red: {red}, Green: {green}, Blue: {blue}, Clear: {clear}")
        print(f"RGB Color - {cl}")
        self.wait(500)
    # [[[[[[[[[[[    THREAD      ]]]]]]]]]]]
    def start_gyro(self):
        _thread.start_new_thread(self.gyro_thread, ())
    def stop_gyro(self):
        self.flag_startGyro = False
    
    def reset_gyro(self):
        self.gyro.reset()
#         self.gyro.zero_position()
        self.wait(100)
        
        
        
    def gyro_thread(self):
        print("gyro thread")
        self.flag_startGyro = True
#         self.gyro.start()

        try:
            while self.flag_startGyro:
                angles = self.gyro.update_angles()
#                 print("มุม X: {:.2f}, มุม Y: {:.2f}, มุม Z: {:.2f}".format(-angles['x'], angles['y'], 2*angles['z']))
                # แสดงผลบน OLED
#                 self.oled.fill(0)
#                 self.oled.text("X: {:.2f}".format(-angles['x']), 0, 0)
#                 self.oled.text("Y: {:.2f}".format(angles['y']), 0, 10)
#                 self.oled.text("Z: {:.2f}".format(2*angles['z']), 0, 20)
#                 self.oled.show()
                sleep(0.1)

        except KeyboardInterrupt:
            self.flag_startGyro = False
            print("หยุดการทำงาน")
            

            
            
    def getYaw(self):
        return self.gyro.angles['z']*-2
    def getPitch(self):
        return self.gyro.angles['x']*-1
    def getRoll(self):
        return self.gyro.angles['y']


    
    # [[[[[[[[[[[    GYRO      ]]]]]]]]]]]
    
     
    
    # [[[[[[[[[[[    MENU      ]]]]]]]]]]]  
    # Function to display a menu
    def display_menu(self):
        self.oled.fill(0)
        for i, option in enumerate(self.menu_options):
            prefix = "> " if i == self.current_menu else "  "
            self.oled.text(prefix + option, 0, i * 10)
        self.oled.show()
        
    # Function to read knop value and calculate menu position
    def read_knop(self):
        
        value = self.knop.read()
        self.current_menu =  int(value /  1000) 
        sleep(0.1)
    # Set Sensor
    # [[[[[[[[[[[    MENU      ]]]]]]]]]]]
    def set_sensor_w(self):
        self.sensor["a0w"] = self.ra(self._A0)
        self.sensor["a1w"] = self.ra(self._A1)
        self.sensor["a2w"] = self.ra(self._A2)
        self.sensor["a3w"] = self.ra(self._A3)
        self.sensor["a4w"] = self.ra(self._A4)
        self.sensor["a5w"] = self.ra(self._A5)
    def set_sensor_k(self):
        self.sensor["a0k"] = self.ra(self._A0)
        self.sensor["a1k"] = self.ra(self._A1)
        self.sensor["a2k"] = self.ra(self._A2)
        self.sensor["a3k"] = self.ra(self._A3)
        self.sensor["a4k"] = self.ra(self._A4)
        self.sensor["a5k"] = self.ra(self._A5)    
    def set_sensor_get_mean(self,sensor):
        return {
            "mA0": (sensor["a0k"] + sensor["a0w"]) / 2,
            "mA1": (sensor["a1k"] + sensor["a1w"]) / 2,
            "mA2": (sensor["a2k"] + sensor["a2w"]) / 2,
            "mA3": (sensor["a3k"] + sensor["a3w"]) / 2,
            "mA4": (sensor["a4k"] + sensor["a4w"]) / 2,
            "mA5": (sensor["a5k"] + sensor["a5w"]) / 2
        }
    # ฟังก์ชันสำหรับบันทึก mean ลงไฟล์
    def save_mean_to_file(self):
        with open(self.MEAN_FILE, "w") as file:
            json.dump(self.mean, file)
            
    # ฟังก์ชันสำหรับโหลด mean จากไฟล์
    def load_mean_from_file(self):
        try:
            file = open(self.MEAN_FILE, "r")
        except:
            print("error read file")
            self.beep(1000);self.beep(1000);
            
        return json.load(file)
    def set_sensor_mean(self):
        self.beep(500);
        ### BEGIN WHITE
        print("set white")
        self.clear();self.show("setting... WHITE")
        sleep(1);self.clear()
        while not self.sw.value()==0: 
            self.show_ss()
        self.beep(500);self.clear()
        self.set_sensor_w()
        self.show("White Saved !!!");sleep(2)
        
            
        ### BEGIN BLACK 
        print("set black")
        self.clear();self.show("setting... BLACK")
        sleep(1);self.clear()
        while not self.sw.value()==0: 
            self.show_ss()
        self.beep(500);self.clear()
        self.set_sensor_k()
        self.show("Black Saved !!!");sleep(2)
        ### Cale Mean 
        print("cale mean")
        self.mean = self.set_sensor_get_mean(self.sensor)
        print(self.mean)
        self.clear()
        self.show("SAVE MEAN");sleep(2)
        print("save to file")
        self.save_mean_to_file()
        print("success return to start menu")
        self.start_menu()

        
    def set_sensor(self):
        print("Set sensor")
        menu_sensors = ["0. BACK", "1. setMean", "2. x", "3. x"]
        current_menu_sensor = 0
        while True:
            #display
            self.clear()
            for i, option in enumerate(menu_sensors):
                prefix = "> " if i == current_menu_sensor else "  "
                self.oled.text(prefix + option, 0, i * 10)
            self.oled.show()
            #read knop
            value = self.knop.read()
            current_menu_sensor =  int(value /  1000)
            print(current_menu_sensor)
            sleep(0.1)
            #check switch
            if self.sw.value() == 0:
                if current_menu_sensor == 0:
                    print("Back")
                    self.start_menu();break
                elif current_menu_sensor == 1:
                    print("setMean")
                    self.set_sensor_mean();break
                elif current_menu_sensor == 2:
                    print("set...")

                    
        
        


 


    def set_dice_count(self):
         
        oled.fill(0)
        oled.text("Set Dice Count:", 0, 0)
        oled.text("Count: " + str(self.dice_count), 0, 10)
        oled.show()
        while True:
            read_knop()
            if sw.value() == 0:
                break
            if knop.read() > 3000:
                dice_count += 1
            elif knop.read() < 1000:
                self.dice_count = max(1, self.dice_count - 1)
            oled.fill(0)
            oled.text("Set Dice Count:", 0, 0)
            oled.text("Count: " + str(dice_count), 0, 10)
            oled.show()
            sleep(0.2)
        
    def start_program(self):
        oled.fill(0)
        oled.text("Starting Program", 0, 0)
        oled.show()
        sleep(2)
    
    def start_menu(self):
        self.beep(200);  
        while True:
            self.display_menu()
            self.read_knop()
            if self.sw.value() == 0:
                if self.current_menu == 0:
                    #0 start
                    self.start();break
                elif self.current_menu == 1:
                    #1 setSensor 
                    self.set_sensor() ;break
                elif self.current_menu == 2:
                    #2 testGyro 
                    test_gyro.start_test_gyro();break
                elif self.current_menu == 3:
                    self.set_dice_count() ;break
                elif self.current_menu == 4:
                    self.start_program() ;break
                sleep(0.5)

        self.beep(500);
        
            

#     def setW(self,a0:int,a1:int,a2:int,a3:int,a4:int,a5:int):
#         self.a0w = a0; self.a1w = a2 ; self.a2w = a3
#         self.a3w = a3; self.a4w = a4 ; self.a5w = a5
#     def setB(self,a0:int,a1:int,a2:int,a3:int,a4:int,a5:int):
#         self.a0k = a0; self.a1k = a2 ; self.a2k = a3
#         self.a3k = a3; self.a4k = a4 ; self.a5k = a5
#     def cal_mean(self):
#         self.mA0 = (self.a0k+self.a0w)/2	;	self.mA1 = (self.a1k+self.a1w)/2
#         self.mA2 = (self.a2k+self.a2w)/2	;	self.mA3 = (self.a3k+self.a3w)/2
#         self.mA4 = (self.a4k+self.a4w)/2	;	self.mA5 = (self.a5k+self.a5w)/2
    def set_motion(self,spL,spR):
        self.Plus_speedL =spL; self.Plus_speedR = spR
    def set_distance(self,V,T):
        self.S=V*T
        print(f"set distanc s = {self.S}")
    def get_S(self):
        return self.S
    def beep(self,f):
        self.buzzerWrite(self._BZ, freq=f, stop=0.1)
        
    def show_ss(self):
            self.oled.fill(0)
            self.oled.text('A0 ' + str( self.ra(self._A0) ),0,0);  self.oled.text('A1 ' + str( self.ra(self._A1) ),0,10) ; self.oled.text('A2 ' + str( self.ra(self._A2) ),0,20)
            self.oled.text('A3 ' + str( self.ra(self._A3) ),0,30)
            self.oled.text('A4 ' + str( self.ra(self._A4) ),0,40)
            self.oled.text('A5 ' + str( self.ra(self._A5) ),0,50)
            self.oled.show() ; sleep(0.01)
        
    def start(self):
        # ตรวจสอบ mean จากไฟล์เมื่อเริ่มโปรแกรม
        foundMeanflag  = False
        checkfile = {}
        checkfile = self.load_mean_from_file()
        if checkfile == {}:
            self.show("ERROR READ FILE")
        else:
            self.mean = self.load_mean_from_file()
            foundMeanflag = True

            
        if(foundMeanflag == True):
            self.start_gyro()
            print(self.mean)
            self.oled.fill(0);self.beep(500)
            self.wait(500)
            while True:
                self.show_gyro()
                if self.sw.value() == 0:
                    break
            self.beep(1000)
            self.oled.fill(0);
            
         

    def test(self, a:str):
        print(a)
    
 
    # [[[ scan I2C]]]
    def scan_i2c(self):
        i2c = I2C(0, scl=Pin(22), sda=Pin(21))
        print('Scan i2c bus...')
        devices = i2c.scan()
        #address 60  0x3c
        #address 104 0x68

        if len(devices) == 0:
          print("No i2c device !")
        else:
          print('i2c devices found:',len(devices))

          for device in devices:  
            print("Decimal address: ",device," | Hexa address: ",hex(device))
    # [[[[[[[[[[[    SOUND      ]]]]]]]]]]]
    def buzzerWrite(self,pin, freq=1000, duty=50, stop=0):
        pwm = PWM(Pin(pin))
        pwm.freq(freq)
        pwm.duty(int(duty / 100 * 1023))
        if stop > 0:
            sleep(stop)
            pwm.duty(0)
    

  # [[[[[[[[[[[    SENSOR      ]]]]]]]]]]]  
    def ra(self,analog_pin):  # read analog
        adc = ADC(Pin(analog_pin))
        adc.atten( ADC.ATTN_11DB)
        adc.width( ADC.WIDTH_12BIT)
        return adc.read()

    def iB(self,p:int):
        rt = False
        if   p == 0:
            rt = self.ra(self._A0) < self.mean['mA0']
        elif p == 1 :
            rt = self.ra(self._A1) < self.mean['mA1']
        elif p == 2 :
            rt = self.ra(self._A2) < self.mean['mA2']
        elif p == 3 :
            rt = self.ra(self._A3) < self.mean['mA3']
        elif p == 4 :
            rt = self.ra(self._A4) < self.mean['mA4']
        elif p == 5 :
            rt = self.ra(self._A5) < self.mean['mA5']
        else :
            print("error input sensor port")
        return rt
    def iW(self,p:int):
        rt = False
        if   p == 0:
            rt = self.ra(self._A0) > self.mean['mA0']
        elif p == 1 :
            rt = self.ra(self._A1) > self.mean['mA1']
        elif p == 2 :
            rt = self.ra(self._A2) > self.mean['mA2']
        elif p == 3 :
            rt = self.ra(self._A3) > self.mean['mA3']
        elif p == 4 :
            rt = self.ra(self._A4) > self.mean['mA4']
        elif p == 5 :
            rt = self.ra(self._A5) > self.mean['mA5']
        else :
            print("error input sensor port")
        return rt
    
    
    def show_ss(self):
        self.oled.fill(0)
        self.oled.text('A0 ' + str( self.ra(self._A0) ),0,0);  self.oled.text('A1 ' + str( self.ra(self._A1) ),0,10) ; self.oled.text('A2 ' + str( self.ra(self._A2) ),0,20)
        self.oled.text('A3 ' + str( self.ra(self._A3) ),0,30);  self.oled.text('A4 ' + str( self.ra(self._A4) ),0,40); self.oled.text('A5 ' + str( self.ra(self._A5) ),0,50)
        self.oled.show() ; sleep(0.01)
    def show_gyro(self):
       
        self.oled.fill(0)
        self.oled.text("X: {:.2f}".format(self.getPitch()), 0, 0)
        self.oled.text("Y: {:.2f}".format(self.getRoll()), 0, 10)
        self.oled.text("Z: {:.2f}".format(self.getYaw()), 0, 20)
        self.oled.text("READY....", 0, 30)
        self.oled.show() ; sleep(0.01)
    
    def show(self,txt):
        
        self.clear()
        self.oled.text(txt,0,0)
        self.oled.show()
    def clear(self):
        self.oled.fill(0)
    

 

 
    # [[[[[[[[[[[     MOTION      ]]]]]]]]]]]  
    def start_move(self,_spL:float, _spR:float):
        
        self.m.Motor( _spL + self.Plus_speedL , _spR + self.Plus_speedR )
        
    def zStart_F(self,pSpeed:float):
        target = 0
        power = pSpeed
        yaw = self.getYaw()
#         print("yaw =",yaw)
        error =target - yaw
        self.integral = self.integral + error
        derivative = error - self.preError
        result = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.start_move(1*(power+result),1*(power-result))
        
    def zStart_B(self,pSpeed:float):
        target = 0
        power = pSpeed
        yaw = self.getYaw()
        error =target - yaw
        self.integral = self.integral + error
        derivative = error - self.preError
        result = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.start_move(-1*(power-result),-1*(power+result))
        
    def stop(self):
         
        self.m.stop();sleep(0.1)
    
    def wait(self,t:int):
        sleep_ms(t)
        
    # [[[[[[[[[[[     SERVO     ]]]]]]]]]]]
    
    def testSv(self):
        print("Testing servo....")
        servo = Servo(pin=23)   
        servo.write_angle(180)   
        sleep_ms(1000)
        

    def sv(self,svid:int,degree:int):
        if svid==1:
            self.sv1.write_angle(degree)
        if svid==2:
            self.sv2.write_angle(degree)
        if svid==3:
            self.sv3.write_angle(degree)
        if svid==4:
            self.sv4.write_angle(degree)

'''
--------------------------------------------------------------------------
           END CLASS
--------------------------------------------------------------------------
'''

