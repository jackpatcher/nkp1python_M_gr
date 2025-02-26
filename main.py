import _amMidNPK1
from time import sleep_ms
import asyncio

am = _amMidNPK1.AmMid_NKP1()


'''
v68.1.3
--------------------------------------------------------------------------
           CHECK LIST สิ่งที่ต้องทำก่อนแข่ง มีอยู่ในคำสั่ง test()
           1. สั่ง FP(n=1,v=50) ก่อนเพื่อหาระยะ 1 แผ่น  แก้ไขที่ตัวแปร gV gT
           2. สั่ง FP(n=4,v=30) เคลื่อนที่ตรง โอเคมั๊ย ถ้าไม่ไปปรับค่าตัวแปร gPlus_vL  gPlus_vR
           3. เลี้ยวซ้าย เลี้ยวขวา โอเคมั๊ย  เลือกความเร็วและเวลาการหมุนให้เหมาะสม
--------------------------------------------------------------------------
'''
 
'''
--------------------------------------------------------------------------
           ตัวแปร Global
--------------------------------------------------------------------------
'''

# ตั้งค่า ว่าวิ่งตรงดี หรือไม่ ถ้าไม่ให้ปรับเพิ่มล้อบางล้อ
gPlus_vL = 0    #ปรับล้อซ้าย เพิ่ม หากสั่งตรงแล้วยังวิ่งไม่ตรง
gPlus_vR = 1    #ปรับล้อขวา เพิ่ม หากสั่งตรงแล้วยังวิ่งไม่ตรง

gSpinVslow = 20

gTuneFast = 30
gTuneSlow = -20
# ทดสอบวิ่งตรง 1 แผ่น แล้วกรอกความเร็ว V และ เวลา T  โดยใช้คำสั่ง FS เพื่อทดสอบ
gV = 50             #ความเร็ว 1 แผ่น  เป็นจำนวนเต็ม
gT = 65             #เวลาที่ใช้ 1 แผ่น  เป็นจำนวนเต็ม

kp = 0.7           #ปรับค่าการเหวี่ยง เพื่อรักษาสมดุลย์การเคลื่อนที่

'''
--------------------------------------------------------------------------
           โปรแกรม main
--------------------------------------------------------------------------
'''

def main():
    setup()


 
    
    # test()

    Drop(sv=1,d=90,t=800)

#     box1()
#     box2()
#     box3()
#     box4()
#     finish()
    
def box1():
    print("box1")
    B2K(v=20,n=0)
    FP(n=2.1,v=50)
    L90(v=50,d=40)  
    B2K(v=20,n=3.5)
    
    R90(v=50,d=40)
    FP(n=0.2,v=-50)
#     Drop(sv=1,d=90,t=800)
    
    
    
    
    
    
    
    
    
    
def box2():
    print("box2")
    
    B2K(v=30,n=0.3)
    R90(v=50,d=40)
    FP(n=1,v=50)
    L90(v=50,d=40)
    F2K(v=30,n=0.3) 
    R90(v=50,d=40)
    FP(n=0.6,v=-50)
    
    
    
def box3():
    print("box3")

    
def box4():
    print("box4")
    
def finish():
    print("finish")
    


def test():
    print("test")      #เปิดทีละคำสั่ง เพื่อทดสอบ  ctrl+3 หรือ ctrl+/ 
#     FP(n=1,v=50)	 #หา 1 แผ่น หากไม่ตรง แก้ไขที่ตัวแปร gT gV
    # FP(n=4,v=30)       #ทดสอบการเดินตรง  หากไม่ตรง แก้ไขทีตัวแปร gPlus_vL R
    # L90(v=50,d=40) 
    # R90(v=50,d=40) 
    # L180(v=50,d=140) 
	
'''
--------------------------------------------------------------------------
            ชุดคำสั่ง
--------------------------------------------------------------------------
'''

# [[[[[[[[[[[  SETUP    ]]]]]]]]]]]  
def setup():
    global kp
    am.kp = kp
    am.sv(svid=1,degree=180)
    print(am.kp)
    am.start_menu()
    print("setup..")
    setMotion()
    set_distance()
    
def setMotion():
    global gPlus_vL,gPlus_vR
    print("set motion..")
    am.set_motion(gPlus_vL,gPlus_vR)
def set_distance():
    global gV,gT
    am.set_distance(gV,gT)
    
# [[[[[[[[[[[    SOUND      ]]]]]]]]]]] 
def beep():
    am.beep()

# [[[[[[[[[[[    MOTION     ]]]]]]]]]]]
def R90(v:int,d:int):
    am.show("R90")
    am.reset_gyro();am.wait(40)
    while am.getYaw() < d:
        am.start_move(v, -v)
    while am.getYaw() < 90:
        am.start_move(gSpinVslow, -gSpinVslow)
    am.stop()
    

def L90(v:int,d:int):
    am.show("L90")
    am.reset_gyro();am.wait(40)
    while am.getYaw() > -d:
        am.start_move(-v, v)
    while am.getYaw() > -90:
        am.start_move(-gSpinVslow, gSpinVslow)
    am.stop()
def L180(v:int,d:int):
    am.show("L180")
    am.reset_gyro();am.wait(40)
    while am.getYaw() > -d:
        am.start_move(-v, v)
    while am.getYaw() > -180:
        am.start_move(-gSpinVslow, gSpinVslow)
    am.stop()
    
def myDrop():
    Drop(sv=1,d=90,t=800)

    
    
def FS(v:float,t:int):
    print("FS...") 
    am.start_move(v, v);am.wait(t) 
    am.stop()
def FS2(v1:float,t1:int,v2:float,t2:int):
    print("FS2...") 
    am.start_move(v1, v1);am.wait(t1)
    am.start_move(v2, v2);am.wait(t2)
    am.stop();  
def FP(n:float,v:float):
    am.show("FP")
    am.reset_gyro()
    S = am.get_S() 
    t = abs(int(n*(S/v)))
    for i in range(t):
        am.zStart_F(v)
        am.wait(1)
    am.stop();
def FP2(n1:float,v1:float , n2:float,v2:float):
    print("FP2...")
    S = am.get_S() 
    t1 = abs(int(n1*S/v1)) ; am.start_move(v1, v1); am.wait(t1)
    t2 = abs(int(n2*S/v2)) ; am.start_move(v2, v2); am.wait(t2)
    am.stop();        

def F2K(v:int,n:float):
    global gTuneFast,gTuneSlow
    rt = "00"
    am.show("F2K...");
    while True:
        print(am.iW(1))
        if (am.iW(5)  and  am.iW(0) ) and (am.iW(1)  and  am.iW(2) ) :
            am.start_move(v,v)
        if (am.iB(5)  and  am.iB(0) ) and (am.iW(1)  and  am.iW(2) ):
            rt = "10";break
        if (am.iW(5)  and  am.iW(0) ) and (am.iB(1)  and  am.iB(2) ):
            rt = "01";break

    
    am.stop();
    
    am.show(rt)
    
    if rt=="10" :
        while am.iW(2):
            am.start_move(gTuneSlow,gTuneFast)
        
    if rt=="01" :
        while am.iW(0):
            am.start_move(gTuneFast,gTuneSlow)
         
    am.stop()
    sleep_ms(40)
    FP(n,-v)
    
    
    
    print(f"found blacK {rt}")
    return rt


def Run(n:float,v:int,d:int):
    print("RunLane...")
    global gTuneFast,gTuneSlow
    ching = n-1
    
    am.reset_gyro()
    S = am.get_S() 
    t = abs(int(ching*(S/v)))
    for i in range(t):
        am.zStart_F(v)
        am.wait(1)
        if am.iB(0):
            am.reset_gyro()
            while am.getYaw() < d:
                am.start_move(20, -20)
            am.reset_gyro()
        if am.iB(2):
            while am.getYaw() > -d:
                am.start_move(-20,20)
            am.reset_gyro()
            
    FP(n=1,v=v)
        

def B2K(v:int,n:float):
    global gTuneFast,gTuneSlow
    rt = "00"
    am.show("B2K...");
    while True:
        if am.iB(4) and am.iB(3) :
            rt = "11";break
        if am.iB(4) and am.iW(3):
            rt = "10";break
        if am.iW(4) and am.iB(3):
            rt = "01";break
        if rt == "00":
            am.start_move(-v,-v)
    am.stop();
    #tune
    if rt == "10":  
        while not am.iB(3):
            am.start_move(-gTuneSlow,-gTuneFast)
    if rt == "01":  
        while not am.iB(4):
            am.start_move(-gTuneFast,-gTuneSlow)

    am.stop()
    sleep_ms(40)
    FP(n,v)
#     am.start_move(v, v)
#     sleep_ms(50);  am.stop();   
    
def TurnL(vL:int,vR:int,t:int):
    am.show("TurnL..")
    if vL>vR:
        vR,vL = vL,vR
    am.start_move(vL, vR)
    sleep_ms(time);  am.stop()

def TurnR(vL:int,vR:int,t:int):
    print("TurnR...");
    if vR>vL:
        vR,vL = vL,vR
    am.start_move(vL, vR)
    sleep_ms(time);  am.stop()
    
def gTurnL(vL:int,vR:int,d:int):
    am.show("gTurnL..")
    am.reset_gyro();am.wait(40)
    if vL>vR:
        vR,vL = vL,vR
    while am.getYaw() > -d:
        am.start_move(vL, vR)
    am.stop()
    
def gTurnR(vL:int,vR:int,d:int):
    print("gTurnR...")
    am.reset_gyro();am.wait(40)
    if vR>vL:
        vR,vL = vL,vR
    while am.getYaw() < d:
        am.start_move(vL, vR)
    am.stop()

# [[[[[[[[[[[   Servo    ]]]]]]]]]]]  
def Drop(sv:int,d:int,t:int):
    am.show("Drop..")

    am.sv(svid=sv,degree=d)
    am.wait(t)
    am.sv(svid=sv,degree=180)
    am.wait(t)
    print("DROPPP")
 
 
def start_main():
    try:
        main()
    except KeyboardInterrupt:
        am.stop_gyro()
        print("catching keyboard interrupt")
    finally:
        print("Exiting the program")
        am.show("End Program")
'''
--------------------------------------------------------------------------
           RUN ห้ามแก้ไข
--------------------------------------------------------------------------
'''

start_main()

#     while 1:
#         am.getColor()
#         am.wait(200)

'''
--------------------------------------------------------------------------
           คู่มือ
--------------------------------------------------------------------------
'''
# [[[[[[[[[[[   คีย์ลัด     ]]]]]]]]]]] 
#   CTRL+Z ย้อน.  CTRL+C คัดลอก   CTRL+V วาง    CTRL+/  คอมเม้น

# [[[[[[[[[[[    ผังการต่อ      ]]]]]]]]]]] 
#     A5            A1
# A0                    A2
#
# A4                    A3

# ==== ตัวแปรที่ใช้ ในการเขียนโปรแกรมย่อย
# v = ความเร็ว velocity vFast=เร็วมาก vSlow=เร็วน้อย,vL= เร็วล้อซ้าย ,vR=เร็วล้อขวา
# d = องศา ดีกรี degree  
# n = จำนวนแผ่น
# t = time เวลา


# [[[[[[[[[[[   คำสั่งเคลื่อนที่      ]]]]]]]]]]] 



# =======FP วิ่งไปหน้า หรือถอยหลัง ตามจำนวนแผ่น พร้อมใช้ gyro ในการกำหนดทิศทาง
# FP(n=1,v=50)      					# เดินหน้า   1 แผ่น ด้วยความเร็ว 50 เวลา จะใช้ตามตัวแปรที่ตั้งค่าไว้ 1 แผ่น  
# FP(n=1,v=-50)     					# ถอยหลัง  1 แผ่น ด้วยความเร็ว 50 เวลา จะใช้ตามตัวแปรที่ตั้งค่าไว้ 1 แผ่น  
# FP2(n1=1,v1=50 , n2=0.5,v2=20)

# =======RUN วิ่งไปหน้า พร้อมชิ่ง
# Run(n=3,v=30,d=10) 		# วิ่งชิ่ง 2 แผ่น ด้วยความเร็ว 30  ชิ่ง 10 องศา จากนั้นวิ่งไปอีก 1 แผ่น

# =======B2K F2K  วิ่งหาเส้นดำ
# B2K(v=30,n=0.2)                       # ถอยด้วยความเร็ว 30 จนเจอเส้นดำ พร้อมจูน  จากนั้นไปหน้า 0.2 แผ่น
# F2K(v=30,n=0.2)                       # ไปหน้าด้วยความเร็ว 30 จนเจอเส้นดำ พร้อมจูน  จากนั้นไปถอยหลัง 0.2 แผ่น

# =======L90 R90 L180  หมุนกับที่ด้วย Gyro  เข็มทิศ
# L90(v=50,d=40)                        # หมุนกับที่ซ้าย ด้วยความเร็ว 50 ไปที่ 40 องศา จากนั้นหมุนช้าจนครบ 90 องศา
# R90(v=50,d=40)                        # หมุนกับที่ขวา ด้วยความเร็ว 50 ไปที่ 40 องศา จากนั้นหมุนช้าจนครบ 90 องศา
# L180(v=50,d=140)                      # หมุนกับที่ซ้าย กลับหลัง ด้วยความเร็ว 50 ไปที่ 140 องศา จากนั้นหมุนช้าจนครบ 180 องศา


# =======Turn gTurn  หมุนด้วยเวลา  หมุนด้วยเข็มทิศ
# gTurnL(vL=50,vR=30,d=90)              # หมุนซ้ายด้วย Gyro ด้วยความเร็วซ้าย 50 ขวา 30  หมุนจะกว่า จะถึง 90 องศา  หมุนซัดโค้ง
# gTurnR( vL=30,vR=50,d=90)  			# หมุนขวาด้วย Gyro ด้วยความเร็วซ้าย 30 ขวา 50  หมุนจะกว่า จะถึง 90 องศา  หมุนซัดโค้ง
# TurnL( vL=50,vR=-50,t=220)  			# หมุนขวา ด้วยความเร็วซ้าย -50 ขวา 50  ในเวลา 0.22 วินาที
# TurnR( vL=-50,vR=50,t=220)  			# หมุนขวา ด้วยความเร็วซ้าย -50 ขวา 50  ในเวลา 0.22 วินาที

# =======Drop  ปล่อยลูกเต๋า
# myDrop()                              # ปล่อยลูกเต๋า ช่อง servo1  ไปที่ 90 องศา เวลา 0.8 วินาที
# Drop(sv=1,d=90,t=800)                # ปล่อยลูกเต๋า ช่อง servo1  ไปที่ 90 องศา เวลา 0.8 วินาที

# =======FS  ไม่ค่อยใช้ วิ่ง ตามระยะทาง ด้วย ความเร็วและเวลา
# FS(v=50,t=500)                        # เดินหน้า  ด้วยความเร็ว 50 เวลา 0.5 วิ  ใช้สำหรับวิเคราะห์หาแผ่น 1 แผ่น เมื่อได้ค่า 1 แผ่นแล้ว ต้องไปปรับตัวแปร V และ T ด้วย
# FS(v=-50,t=500)                   	# ถอยหลัง  ด้วยความเร็ว 50 เวลา 0.5 วิ
# FS2(v1=50,t1=500 , v2=20,t2=200)    	# เดินหน้า 2ช่วง  ด้วยความเร็ว 50 เวลา 0.5 วิ  วิ่งต่อทันทีด้วยความเร็ว 20 เวลา 0.2 วิ แล้วหยุด
# FS2(v1=-50,t1=500 , v2=-20,t2=200)  	# ถอยหลัง 2ช่วง  ด้วยความเร็ว 50 เวลา 0.5 วิ  วิ่งถอยต่อทันทีด้วยความเร็ว 20 เวลา 0.2 วิ แล้วหยุด
