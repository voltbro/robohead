# Пример работы с тачскрином

import pygame, evdev, select, time

surfaceSize = (1080, 1080)

pygame.init()

lcd = pygame.Surface(surfaceSize)

def refresh():
    f = open("/dev/fb0","wb")
    f.write(lcd.convert(24,0).get_buffer())
    f.close()


pygame.font.init()
defaultFont = pygame.font.SysFont(None,30)

lcd.fill((255,0,0))
lcd.blit(defaultFont.render("Hello World!", False, (0, 0, 0)),(500, 500))
refresh()
time.sleep(0.5)

lcd.fill((0, 255, 0))
lcd.blit(defaultFont.render("Hello World!", False, (0, 0, 0)),(500, 500))
refresh()
time.sleep(0.5)

lcd.fill((0,0,255))
lcd.blit(defaultFont.render("Hello World!", False, (0, 0, 0)),(500, 500))
refresh()
time.sleep(0.5)

lcd.fill((128, 128, 128))
lcd.blit(defaultFont.render("Hello World!", False, (0, 0, 0)),(500, 500))
refresh()
time.sleep(0.5)


for i in [0,1]:
    touch = evdev.InputDevice(f"/dev/input/event{i}")
    print(touch.name, f"/dev/input/event{i}")
    if ("waveshare" in str(touch.name).lower()):
        break

touch.grab()

def getPixelsFromCoordinates(coords):
    x = coords[0]/4095*1079
    y = coords[1]/4095*1079

    return (int(x), int(y))

def printEvent(event):
    print(evdev.categorize(event))
    print("Value: {0}".format(event.value))
    print("Type: {0}".format(event.type))
    print("Code: {0}".format(event.code))

X = 0
Y = 0
flag = 0
while True:
    r,w,x = select.select([touch], [], [])
    for event in touch.read():
        # printEvent(event)
        # print('---------')
        if event.type == evdev.ecodes.EV_ABS:

            if event.code == 0:
                X = event.value
            elif event.code == 1:
                Y = event.value
                flag = 1
        if flag==1:
            flag = 0

            p = getPixelsFromCoordinates((X, Y))
            lcd.fill((50, 50, 50))
            pygame.draw.circle(lcd, (255, 255, 255), p, 50, 10)
            refresh()