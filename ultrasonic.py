from machine import Pin
import utime

#======================
#untuk ultrasonik depan
#Ultra 1 Trigger (0) dan echo (1)
#Ultra 2 Trigger (29) dan echo (23)
#======================

#======================
#untuk ultrasonik kanan
#ultra 6 Trigger (22) dan echo (26)
#ultra 8 Trigger (18) dan echo (19)
#======================

#======================
#untuk ultrasonik kiri
#ultra 5 Trigger (4) dan echo (5)
#ultra 7 Trigger (6) dan echo (7)
#======================

#======================
#untuk ultrasonik belakang
#ultra 11 Trigger (14) dan echo (15)
#ultra 12 Trigger (6) dan echo (7)
#======================

#======================
#untuk ultrasonik serong kanan depan
#ultra 4 Trigger (27) dan echo (28)
#======================

#======================
#untuk ultrasonik serong kiri depan
#ultra 3 Trigger (2) dan echo (3)
#======================

#======================
#untuk ultrasonik serong kanan belakang
#ultra 10 Trigger (16) dan echo (17)
#======================

#======================
#untuk ultrasonik serong kiri belakang
#ultra 6 Trigger (8) dan echo (9)
#======================


#Ultrasonik depan
trigger1 = Pin(0, Pin.OUT)
echo1 = Pin(1, Pin.IN)
trigger2 = Pin(29, Pin.OUT)
echo2 = Pin(23, Pin.IN)

#ultrasonik kanan
trigger6 = Pin(22, Pin.OUT)
echo6 = Pin(26, Pin.IN)
trigger8 = Pin(18, Pin.OUT)
echo8 = Pin(19, Pin.IN)

#ultrasonik kiri
trigger5 = Pin(4, Pin.OUT)
echo5 = Pin(5, Pin.IN)
trigger7 = Pin(6, Pin.OUT)
echo7 = Pin(7, Pin.IN)

#ultrasonik belakang
trigger11 = Pin(14, Pin.OUT)
echo11 = Pin(15, Pin.IN)
trigger12 = Pin(12, Pin.OUT)
echo12 = Pin(13, Pin.IN)

#ultrasonik serong kanan depan
trigger4 = Pin(27, Pin.OUT)
echo4 = Pin(28, Pin.IN)

#ultrasonik serong kiri depan
trigger3 = Pin(2, Pin.OUT)
echo3 = Pin(3, Pin.IN)

#ultrasonik serong kanan belakang
trigger10 = Pin(16, Pin.OUT)
echo10 = Pin(17, Pin.IN)

#ultrasonik serong kiri belakang
trigger9 = Pin(8, Pin.OUT)
echo9 = Pin(9, Pin.IN)

def measure_distance(trigger, echo):
    trigger.low()
    utime.sleep_us(2)
    trigger.high()
    utime.sleep_us(5)
    trigger.low()
    
    while echo.value() == 0:
        signaloff = utime.ticks_us()
    while echo.value() == 1:
        signalon = utime.ticks_us()
        
    timepassed = signalon - signaloff
    distance = (timepassed * 0.0343) / 2
    return distance

def ultra_depan():
    distance1 = measure_distance(trigger1, echo1)
    distance2 = measure_distance(trigger2, echo2)
    
    print("Ultrasonic 1:", distance1, "cm")
    print("Ultrasonic 2:", distance2, "cm")
    
def ultra_kanan():
    distance6 = measure_distance(trigger6, echo6)
    distance8 = measure_distance(trigger8, echo8)
    
    print("Ultrasonic 6:", distance6, "cm")
    print("Ultrasonic 8:", distance8, "cm") 

def ultra_kiri():
    distance5 = measure_distance(trigger5, echo5)
    distance7 = measure_distance(trigger7, echo7)
    
    print("Ultrasonic 5:", distance5, "cm")
    print("Ultrasonic 7:", distance7, "cm")
    
def ultra_belakang():
    distance11 = measure_distance(trigger11, echo11)
    distance12 = measure_distance(trigger12, echo12)
    
    print("Ultrasonic 11:", distance11, "cm")
    print("Ultrasonic 12:", distance12, "cm")
    
def ultra_serong_kanan_depan():
    distance4 = measure_distance(trigger4, echo4)

    print("Ultrasonic 4:", distance4, "cm")

def ultra_serong_kiri_depan():
    distance3 = measure_distance(trigger3, echo3)

    print("Ultrasonic 3:", distance3, "cm")

def ultra_serong_kanan_belakang():
    distance10 = measure_distance(trigger10, echo10)

    print("Ultrasonic 3:", distance10, "cm")
    
def ultra_serong_kiri_belakang():
    distance6 = measure_distance(trigger6, echo6)

    print("Ultrasonic 6:", distance6, "cm")
    
# Main loop
while True:
    ultra_serong_kanan_belakang()
    utime.sleep(1)
