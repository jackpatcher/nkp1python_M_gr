import _amMidNPK1
from time import sleep_ms
import asyncio
am = _amMidNPK1.AmMid_NKP1()


'''
--------------------------------------------------------------------------
           CHECK LIST สิ่งที่ต้องทำก่อนแข่ง
           1. หาค่าเซนเซอร์ ดำ ขาว
           2. เคลื่อนที่ตรง โอเคมั๊ย ถ้าไม่ไปปรับค่าตัวแปร gPlus_speedL R
           3. เลี้ยวซ้าย เลี้ยวขวา โอเคมั๊ย  เลือกความเร็วและเวลาการหมุนให้เหมาะสม
--------------------------------------------------------------------------
'''

'''
--------------------------------------------------------------------------
           ตัวแปร Global
--------------------------------------------------------------------------
'''
#ตัวแปรแก้ไขได้

# ตั้งค่าเซนเซอร์ วางบนสีขาว
# a0w=3300	;	a1w=3400	;	a2w=3200	;	a3w=3500	;	a4w=3400	;	a5w=3990
# # ตั้งค่าเซนเซอร์ วางบนสีดำ
# a0k=600		;	a1k=845		;	a2k=673		;	a3k=987		;	a4k=820		;	a5k=1400

    
# ตั้งค่า ว่าวิ่งตรงดี หรือไม่ ถ้าไม่ให้ปรับเพิ่มล้อบางล้อ
Plus_speedL = 0    #ปรับล้อซ้าย เพิ่ม หากสั่งตรงแล้วยังวิ่งไม่ตรง
Plus_speedR = 1    #ปรับล้อขวา เพิ่ม หากสั่งตรงแล้วยังวิ่งไม่ตรง
MinSpeed = 20
spinSlowSpeed = 20
gTuneFast = 30
gTuneSlow = -20
# ทดสอบวิ่งตรง 1 แผ่น แล้วกรอกความเร็ว V และ เวลา T  โดยใช้คำสั่ง FS เพื่อทดสอบ
V = 50             #ความเร็ว 1 แผ่น  เป็นจำนวนเต็ม
T = 160            #เวลาที่ใช้ 1 แผ่น  เป็นจำนวนเต็ม

kp = 0.7

   

    






'''
--------------------------------------------------------------------------
           เขียนโปรแกรม main
--------------------------------------------------------------------------
'''

def main():
    global V,T
    setup()  
    
#     while 1:
#         am.getColor()
#         am.wait(200)
    box1()
    box2()
    box3()
    box4()
    finish()
    

    
    
    
 
      
    
def box1():
    print("box1")
#     TurnL(30,68,752)
#     FP(n=1.2,v=50)
#     TurnR(-50,50,300)
#     B2K(30,0.2)
#     F2K(30,0.2)
#     R90(50,40)
#     L90(50,40)
#     gTurnR(50,30,90)
#     gTurnR(30,-30,180)
    L180(50,140)  
#     gTurnL(30,30,90)
      


    


    
def box2():
    print("box2")
    

    

    
def box3():
    print("box3")

    
def box4():
    print("box4")
    
def finish():
    print("finish")
    



'''
--------------------------------------------------------------------------
            ชุดคำสั่ง
--------------------------------------------------------------------------
'''

# [[[[[[[[[[[  SETUP    ]]]]]]]]]]]  
def setup():
    global kp
    am.kp = kp
    print(am.kp)
    am.start_menu()
    print("setup..")
    setMotion()
    set_distance()
    
def setMotion():
    global Plus_speedL,Plus_speedR
    print("set motion..")
    am.set_motion(Plus_speedL,Plus_speedR)
def set_distance():
    global V,T
    am.set_distance(V,T)
    
# [[[[[[[[[[[    SOUND      ]]]]]]]]]]] 
def beep():
    am.beep()

# [[[[[[[[[[[    MOTION     ]]]]]]]]]]]
def R90(speed:int,degree:int):
    am.show("R90")
    am.reset_gyro();am.wait(40)
    while am.getYaw() < degree:
        am.start_move(speed, -speed)
    while am.getYaw() < 90:
        am.start_move(spinSlowSpeed, -spinSlowSpeed)
    am.stop()
    

def L90(speed:int,degree:int):
    am.show("L90")
    am.reset_gyro();am.wait(40)
    while am.getYaw() > -degree:
        am.start_move(-speed, speed)
    while am.getYaw() > -90:
        am.start_move(-spinSlowSpeed, spinSlowSpeed)
    am.stop()
def L180(speed:int,degree:int):
    am.show("L180")
    am.reset_gyro();am.wait(40)
    while am.getYaw() > -degree:
        am.start_move(-speed, speed)
    while am.getYaw() > -180:
        am.start_move(-spinSlowSpeed, spinSlowSpeed)
    am.stop()
    
def myDrop():
    Drop(sv=1,deg=180,t=800)


def TL():
    TurnL( 18,58,600)  					# โค้งซ้าย
def TR():
    TurnR( 56,18,580)  					# โค้งขวา
    
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

def F2K(speed:int,nback:float):
    global gTuneFast,gTuneSlow
    rt = "00"
    am.show("F2K...");
    while True:
        print(am.iW(1))
        if (am.iW(5)  and  am.iW(0) ) and (am.iW(1)  and  am.iW(2) ) :
            am.start_move(speed,speed)
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
    FP(nback,-speed)
    
    
    
    print(f"found blacK {rt}")
    return rt


def Run(speed:int,tuneFast:int,tuneSlow:int,timeReflect:int):
    print("RunLane...");
    flagTune = "00"
    while True:
        if   am.iB(1):
            am.stop(); break
        elif am.iB(0):
            am.start_move(speed,-speed);am.wait(timeReflect)
        elif am.iB(2):
            am.start_move(-speed,speed);am.wait(timeReflect)
        elif am.iW(1): 
            am.start_move(speed,speed)  
    am.stop() 
    flagTune = F2K(speed)
    if flagTune=="01" :
        while am.iW(5):
            am.start_move(tuneFast,tuneSlow)
    elif flagTune=="10":
        while am.iW(1):
            am.start_move(tuneSlow,tuneFast)
            
    am.start_move(-speed,-speed)
    am.wait(200); am.stop()
        

def B2K(speed:int,nf:float):
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
            am.start_move(-speed,-speed)
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
    FP(nf,speed)
#     am.start_move(speed, speed)
#     sleep_ms(50);  am.stop();   
    
def TurnL(speedL:int,speedR:int,time:int):
    am.show("TurnL..")
    if speedL>speedR:
        speedR,speedL = speedL,speedR
    am.start_move(speedL, speedR)
    sleep_ms(time);  am.stop()

def TurnR(speedL:int,speedR:int,time:int):
    print("TurnR...");
    if speedR>speedL:
        speedR,speedL = speedL,speedR
    am.start_move(speedL, speedR)
    sleep_ms(time);  am.stop()
    
def gTurnL(speedL:int,speedR:int,degree:int):
    am.show("gTurnL..")
    am.reset_gyro();am.wait(40)
    if speedL>speedR:
        speedR,speedL = speedL,speedR
    while am.getYaw() > -degree:
        am.start_move(speedL, speedR)
    am.stop()
    
def gTurnR(speedL:int,speedR:int,degree:int):
    print("gTurnR...")
    am.reset_gyro();am.wait(40)
    if speedR>speedL:
        speedR,speedL = speedL,speedR
    while am.getYaw() < degree:
        am.start_move(speedL, speedR)
    am.stop()

# [[[[[[[[[[[   Servo    ]]]]]]]]]]]  
def Drop(sv:int,deg:int,t:int):
    am.show("Drop..")
    am.sv(svid=sv,degree=0)
    am.wait(t)
    am.sv(svid=sv,degree=deg)
    am.wait(t)
 
 
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


# [[[[[[[[[[[   คำสั่งเคลื่อนที่      ]]]]]]]]]]] 
# FS(v=50,t=500)                       # เดินหน้า  ด้วยความเร็ว 50 เวลา 0.5 วิ
# FS(v=-50,t=500)                   	# ถอยหลัง  ด้วยความเร็ว 50 เวลา 0.5 วิ
# FS2(v1=50,t1=500 , v2=20,t2=200)    	# เดินหน้า 2ช่วง  ด้วยความเร็ว 50 เวลา 0.5 วิ  วิ่งต่อทันทีด้วยความเร็ว 20 เวลา 0.2 วิ แล้วหยุด
# FS2(v1=-50,t1=500 , v2=-20,t2=200)  	# ถอยหลัง 2ช่วง  ด้วยความเร็ว 50 เวลา 0.5 วิ  วิ่งถอยต่อทันทีด้วยความเร็ว 20 เวลา 0.2 วิ แล้วหยุด
# FP(n=1,v=50)      					# เดินหน้า   1 แผ่น ด้วยความเร็ว 50 เวลา จะใช้ตามตัวแปรที่ตั้งค่าไว้ 1 แผ่น gT
# FP(n=1,v=-50)     					# ถอยหลัง  1 แผ่น ด้วยความเร็ว 50 เวลา จะใช้ตามตัวแปรที่ตั้งค่าไว้ 1 แผ่น gT
# FP2(n1=1,v1=50 , n2=0.5,v2=20)

# B2K(20,25,-20)  						# ถอยหลังพร้อมจูน      ถอยด้วยความเร็ว 20 จากนั้นจูนเร็ว 25  จูนช้า -20 (-คือกลับทิศด้วย)
# TurnL( 50,-50,220)  					# หมุนขวา ด้วยความเร็วซ้าย -50 ขวา 50  ในเวลา 0.22 วินาที
# TurnR(-50,50,220)  					# หมุนขวา ด้วยความเร็วซ้าย -50 ขวา 50  ในเวลา 0.22 วินาที
