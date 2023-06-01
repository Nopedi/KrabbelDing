import pygame

pygame.init()
pygame.joystick.init()

joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
joystick = joysticks[0]

while 1:
    pygame.event.get()
    axis = joystick.get_axis(0)
    print(axis)