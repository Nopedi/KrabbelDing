import numpy as np
import pygame
import serial
import time

pygame.init()
pygame.joystick.init()

joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
joystick = joysticks[0]

old_angle_0Lv, old_angle_0Lh, old_angle_0Rv, old_angle_0Rh,  = 90, 90, 90, 90
old_angle_1Lv, old_angle_1Lh, old_angle_1Rv, old_angle_1Rh,  = 90, 90, 90, 90
old_angle_2Lv, old_angle_2Lh, old_angle_2Rv, old_angle_2Rh,  = 90, 90, 90, 90
MAX_OFFSET_V, MAX_OFFSET_H = 60, 60
OLD_ANGLE_WEIGHT = 1

with serial.Serial() as ser:
    ser.baudrate = 115200
    ser.port = '/dev/ttyACM0'
    ser.open()
    time.sleep(2)
    #ser.write('10 20'.encode("utf-8"))
    # time.sleep(3)
    
    
    while 1:
        pygame.event.get()
        x,y = round(joystick.get_axis(1),1), round(joystick.get_axis(0),1) 
        angle_0Lv = (90 - MAX_OFFSET_V * x + OLD_ANGLE_WEIGHT * old_angle_0Lv)/(OLD_ANGLE_WEIGHT+1)
        angle_0Lh = (90 + MAX_OFFSET_H * y + OLD_ANGLE_WEIGHT * old_angle_0Lh)/(OLD_ANGLE_WEIGHT+1)
        angle_0Rv = (90 - MAX_OFFSET_V * x + OLD_ANGLE_WEIGHT * old_angle_0Rv)/(OLD_ANGLE_WEIGHT+1)
        angle_0Rh = (90 + MAX_OFFSET_H * y + OLD_ANGLE_WEIGHT * old_angle_0Rh)/(OLD_ANGLE_WEIGHT+1)
        
        angle_1Lv = (90 + MAX_OFFSET_V * x + OLD_ANGLE_WEIGHT * old_angle_1Lv)/(OLD_ANGLE_WEIGHT+1)
        angle_1Lh = (90 - MAX_OFFSET_H * y + OLD_ANGLE_WEIGHT * old_angle_1Lh)/(OLD_ANGLE_WEIGHT+1)
        angle_1Rv = (90 + MAX_OFFSET_V * x + OLD_ANGLE_WEIGHT * old_angle_1Rv)/(OLD_ANGLE_WEIGHT+1)
        angle_1Rh = (90 - MAX_OFFSET_H * y + OLD_ANGLE_WEIGHT * old_angle_1Rh)/(OLD_ANGLE_WEIGHT+1)
        
        angle_2Lv = (90 - MAX_OFFSET_V * x + OLD_ANGLE_WEIGHT * old_angle_2Lv)/(OLD_ANGLE_WEIGHT+1)
        angle_2Lh = (90 + MAX_OFFSET_H * y + OLD_ANGLE_WEIGHT * old_angle_2Lh)/(OLD_ANGLE_WEIGHT+1)
        angle_2Rv = (90 - MAX_OFFSET_V * x + OLD_ANGLE_WEIGHT * old_angle_2Rv)/(OLD_ANGLE_WEIGHT+1)
        angle_2Rh = (90 + MAX_OFFSET_H * y + OLD_ANGLE_WEIGHT * old_angle_2Rh)/(OLD_ANGLE_WEIGHT+1)
        
        old_angle_0Rv, old_angle_0Rh = angle_0Rv, angle_0Rh
        old_angle_0Lv, old_angle_0Lh = angle_0Lv, angle_0Lh
        
        old_angle_1Rv, old_angle_1Rh = angle_1Rv, angle_1Rh
        old_angle_1Lv, old_angle_1Lh = angle_1Lv, angle_1Lh
        
        old_angle_2Rv, old_angle_2Rh = angle_2Rv, angle_2Rh
        old_angle_2Lv, old_angle_2Lh = angle_2Lv, angle_2Lh
        
        if joystick.get_button(3):
            print("Bye Bye")
            break
        
        out_0Lv = f"{round(angle_0Lv)}".ljust(3)
        out_0Lh = f"{round(angle_0Lh)}".ljust(3)
        out_0Rv = f"{round(angle_0Rv)}".ljust(3)
        out_0Rh = f"{round(angle_0Rh)}".ljust(3)
        
        out_1Lv = f"{round(angle_1Lv)}".ljust(3)
        out_1Lh = f"{round(angle_1Lh)}".ljust(3)
        out_1Rv = f"{round(angle_1Rv)}".ljust(3)
        out_1Rh = f"{round(angle_1Rh)}".ljust(3)
        
        out_2Lv = f"{round(angle_2Lv)}".ljust(3)
        out_2Lh = f"{round(angle_2Lh)}".ljust(3)
        out_2Rv = f"{round(angle_2Rv)}".ljust(3)
        out_2Rh = f"{round(angle_2Rh)}".ljust(3)
        
        out_final = f'{out_0Lh}{out_0Lv}{out_0Rh}{out_0Rv}{out_1Lh}{out_1Lv}{out_1Rh}{out_1Rv}{out_2Lh}{out_2Lv}{out_2Rh}{out_2Rv}'
        print(out_final)
        
        ser.write(out_final.encode("utf-8"))
        print(ser.readline())
        
        # time.sleep(0.1)
        