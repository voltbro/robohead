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
    
    def set_color_palette(self, A_r:int, A_g:int, A_b:int,
                          B_r:int, B_g:int, B_b:int):
        # Изменяет цветовую палитру для режима trace и listen
        # A - цвет, указывающий направление, откуда идет звук 
        # B - цвет, заполняющий остальные светодиоды
        # Для режима spin цвет a задает общую заливку, b не используется
        # цвет задаётся в виде трёх 1-байтовых чисел
        # цвет A: A_r, A_g, A_b
        # цвет B: B_r, B_g, B_b
        self._write(0x21, [A_r, A_g, A_b, 0, B_r, B_g, B_b, 0])

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