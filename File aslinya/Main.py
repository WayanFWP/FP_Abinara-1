import serial
import math
import time

ser = serial.Serial('COM8', 9600)

def sleep_ms(milliseconds):
    time.sleep(milliseconds / 1000)

def Transfer(command):
    commandString = ';'.join(map(str, command)) + '\n'
    ser.write(commandString.encode())
    dataRaw = ser.readline().strip()
    
    if dataRaw:
        data = dataRaw.split(b'-')
        if len(data) >= 5:
            try:
                servo_data = [int(d) for d in data]
                print('{0}-{1}-{2}-{3}-{4}cm'.format(*servo_data))
                Distance = servo_data[4]
                print("Distance = ", Distance, "cm")
                return Distance
            except ValueError as e:
                print("Error converting data to integers:", e)
                return None
        else:
            print("Invalid data format:", dataRaw)
            return None
    else:
        print("No data received from serial port")
        return None


def Robo_stato():
    global servo1, servo2, servo3, gripper
    servo1 = 0
    servo2 = 0
    servo3 = 0
    gripper = 0

    robot = [servo1, servo2, servo3, gripper]
    Transfer(robot)

move_forward = True

Delay_servo1 = 30
Servo1_Kembali = 10
Delay_servo = 10

Robo_stato()
time.sleep(1)

while True:
    if move_forward:
        for i in range(0, 181):
            distance = Transfer([i, 0, 0, 0])
            if 12 <= distance < 14:
                for k in range(0, 80):
                    Transfer([i, k, 0, 0])
                    sleep_ms(Delay_servo)

                time.sleep(1)

                for j in range(-1, 80):
                    Transfer([i, k, j, 0])
                    sleep_ms(Delay_servo)

                time.sleep(1)

                Transfer([i, k, j, 90])

                time.sleep(1)

                for j in range(80, -1, -1):
                    Transfer([i, k, j, 90])
                    sleep_ms(Delay_servo)

                time.sleep(1)
                for k in range(80, -1, -1):
                    Transfer([i, k, j, 90])
                    sleep_ms(Delay_servo)

                time.sleep(1)

                for l in range(i, 220):
                    Transfer([l, k, j, 90])
                    sleep_ms(Servo1_Kembali)
                    
                time.sleep(1)
                
                if distance < 14:
                    Transfer([l,k,j,90])

                    for j in range(-1, 45):
                        Transfer([l, k, j, 90])
                        sleep_ms(Delay_servo)

                    time.sleep(1)

                    for k in range(-1, 40):
                        Transfer([l, k, j, 90])
                        sleep_ms(Delay_servo)

                    time.sleep(1)
                    Transfer([l, j, k, 0])
                    time.sleep(1)
                    
                    for k in range(40, -1, -1):
                        Transfer([l, k, j, 0])
                        sleep_ms(Delay_servo)
                    
                    time.sleep(1)

                    for j in range(45, -1, -1):
                        Transfer([l, k, j, 0])
                        sleep_ms(Delay_servo)
                    
                    time.sleep(1)
                        
                    for l in range(l,180,-1):
                        Transfer([l,k,j,0])
                        sleep_ms(Servo1_Kembali)
                    break
                
                elif 8 < distance:
                    Transfer([l,k,j,90])

                    for k in range(-1, 80):
                        Transfer([l, k, j, 90])
                        sleep_ms(Delay_servo)

                    time.sleep(1)

                    for j in range(-1, 80):
                        Transfer([l, k, j, 90])
                        sleep_ms(Delay_servo)

                    time.sleep(1)
                    Transfer([l, j, k, 0])
                    time.sleep(1)
                    
                    for j in range(80, -1, -1):
                        Transfer([l, k, j, 0])
                        sleep_ms(Delay_servo)

                    time.sleep(1)
                    for k in range(80, -1, -1):
                        Transfer([l, k, j, 0])
                        sleep_ms(Delay_servo)
                    
                    time.sleep(1)
                    
                    for l in range(l,180,-1):
                        Transfer([l,k,j,0])
                        sleep_ms(Servo1_Kembali)
                    break
            sleep_ms(Delay_servo1)
        move_forward = False
    else:
        for i in range(180, -1, -1):
            distance = Transfer([i, 0, 0, 0])
            if 12 <= distance < 14:
                for k in range(-1, 80):
                    Transfer([i, k, 0, 0])
                    sleep_ms(Delay_servo)

                time.sleep(1)

                for j in range(-1, 80):
                    Transfer([i, k, j, 0])
                    sleep_ms(Delay_servo)

                time.sleep(1)

                Transfer([i, k, j, 90])

                time.sleep(1)

                for j in range(80, -1, -1):
                    Transfer([i, k, j, 90])
                    sleep_ms(Delay_servo)

                time.sleep(1)
                for k in range(80, -1, -1):
                    Transfer([i, k, j, 90])
                    sleep_ms(Delay_servo)

                time.sleep(1)

                for l in range(i, 220):
                    pembalik = Transfer([l, k, j, 90])
                    sleep_ms(Servo1_Kembali)
                    
                time.sleep(1)
                
                if distance < 14:
                    Transfer([l,k,j,90])

                    for j in range(-1, 45):
                        Transfer([l, k, j, 90])
                        sleep_ms(Delay_servo)

                    time.sleep(1)

                    for k in range(-1, 40):
                        Transfer([l, k, j, 90])
                        sleep_ms(Delay_servo)

                    time.sleep(1)
                    Transfer([l, j, k, 0])
                    time.sleep(1)
                    
                    for k in range(40, -1, -1):
                        Transfer([l, k, j, 0])
                        sleep_ms(Delay_servo)
                    
                    time.sleep(1)

                    for j in range(45, -1, -1):
                        Transfer([l, k, j, 0])
                        sleep_ms(Delay_servo)
                    
                    time.sleep(1)
                        
                    for l in range(l,0,-1):
                        Transfer([l,k,j,0])
                        sleep_ms(Servo1_Kembali)
                    break
                
                elif 8 < distance:
                    Transfer([l,k,j,90])

                    for k in range(-1, 80):
                        Transfer([l, k, j, 90])
                        sleep_ms(Delay_servo)

                    time.sleep(1)

                    for j in range(-1, 80):
                        Transfer([l, k, j, 90])
                        sleep_ms(Delay_servo)

                    time.sleep(1)
                    Transfer([l, j, k, 0])
                    time.sleep(1)
                    
                    for j in range(80, -1, -1):
                        Transfer([l, k, j, 0])
                        sleep_ms(Delay_servo)

                    time.sleep(1)
                    for k in range(80, -1, -1):
                        Transfer([l, k, j, 0])
                        sleep_ms(Delay_servo)
                    
                    time.sleep(1)
                    
                    for l in range(l,0,-1):
                        Transfer([l,k,j,0])
                        sleep_ms(Servo1_Kembali)
                    break

            sleep_ms(Delay_servo1)
        move_forward = True

    time.sleep(1)

ser.close()
