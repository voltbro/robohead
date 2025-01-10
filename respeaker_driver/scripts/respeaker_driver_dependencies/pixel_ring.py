import usb.core
import usb.util

class PixelRing():
    # forked from https://github.com/respeaker/pixel_ring
    def __init__(self, dev, timeout:int=8000):
        self.dev = dev
        self.timeout = timeout

    def trace(self):
        # режим "слежения" за голосом
        # когда есть звук (голос) светодиоды зажигаются следующим образом:
        # цветом b из color_palette заливаются все свотодиоды
        # цветом a из color_palette зажигается один светодиод, указывающий направление, откуда идет звук
        # если звука нет, через пару секунд светодиоды тухнут
        self._write(0)        
    
    def set_color_all(self, r=0, g=0, b=0):
        # устанавливает все светодиоды в один цвет
        # rgb - задать цвет в формате одного 3-байтового числа
        # r,g,b - задать цвет тремя 1-байтовыми числами 0..255
        self._write(1, [r, g, b, 0])

    def off(self):
        self.set_color_all(0,0,0)

    def listen(self):
        # тоже самое, что trace, только светодиоды горят всегда, независимо от наличия голоса
        self._write(2)

    def wait(self):
        # берет из color_palette цвета a и b
        # окрашивает все светодиоды сначала одним цветом, потом плавно меняет на другой и так далее
        # эффект "ожидания"
        self._write(3)

    def speak(self):
        # берет их color_palette цвета a и b и ими светодиоды чередованием
        # переключает попеременно цвета - эффект "говорения"
        self._write(4)

    def spin(self):
        # берет из color_palette цвет a, заливает все светодиоды этим цветом
        # начинаем плавно переливаться по кругу, понижая яркость свтодиодов
        self._write(5)

    def set_color_manual(self, data:list):
        # data=[r,g,b,0] - устанавливает каждый из 12 светодиодов в свой цвет
        # r,g,b - цвета в формета 1-байта 0...255
        # нумерация светодиодов: расположить respeaker так, чтобы usb выход был снизу
        # тогда если представить, что светодиоды - это циферблат, то
        # 1й светодиод будет там, где 1-й час, 2й светодиод на 2 часа и т.д.
        # последний, 12й светодиод, будет там, куда указывала бы часовая стрелка на 12 часов
        # порядок нумерации, соотвественно, по часовой стрелке
        self._write(6, data)
        
    def set_brightness(self, brightness:int):
        # Устанавливает яркость всех светодиодов
        # 0<=brightness<=31 - 32 уровня яркости
        self._write(0x20, [brightness])
    
    def set_color_palette(self, a:int, b:int):
        # Изменяет цветовую палитру для режима trace и listen
        # a - цвет, указывающий направление, откуда идет звук
        # b - цвет, заполняющий остальные светодиоды
        # Для режима spin цвет a задает общую заливку, b не используется
        # цвет задаётся в виде 3-байтового числа, удобно задавать в 16-ричном формате, например
        # 0xFFFFFF - белый
        # 0xFF0000 - красный
        # 0x00FF00 - зеленый
        # 0x0000FF - синий
        self._write(0x21, [(a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF, 0, (b >> 16) & 0xFF, (b >> 8) & 0xFF, b & 0xFF, 0])

    def set_vad_led(self, state:int):
        # Управление VAD светодиодом (Voice Activity Detection)
        # state=0 - off LED
        # state=1 - on LED
        # state={любое другое значение} - VAD LED будет загораться в зависимости от наличия звуков
        # ! VAD LED находится в центре модуля, из-за корпуса его может быть не видно, имеет красный цвет
        self._write(0x22, [state])

    def set_volume(self, volume:int):
        # ????
        # что делает эта функция? 
        self._write(0x23, [volume])

    def _write(self, cmd:int, data=[0]):
        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, 0x1C, data, self.timeout)

    def __del__(self):
        usb.util.dispose_resources(self.dev)


# def find(vid=0x2886, pid=0x0018):
#     dev = usb.core.find(idVendor=vid, idProduct=pid)
#     if not dev:
#         return

#     # configuration = dev.get_active_configuration()

#     # interface_number = None
#     # for interface in configuration:
#     #     interface_number = interface.bInterfaceNumber

#     #     if dev.is_kernel_driver_active(interface_number):
#     #         dev.detach_kernel_driver(interface_number)

#     return PixelRing(dev)



# if __name__ == '__main__':
#     import time

#     pixel_ring = find()

#     while True:
#         try:
#             pixel_ring.wakeup(180)
#             time.sleep(3)
#             pixel_ring.listen()
#             time.sleep(3)
#             pixel_ring.think()
#             time.sleep(3)
#             pixel_ring.set_volume(8)
#             time.sleep(3)
#             pixel_ring.off()
#             time.sleep(3)
#         except KeyboardInterrupt:
#             break

#     pixel_ring.off()