# ========================================================= #
#                                                           #
#   Пример работы с пакетом speakers_driver на ЯП python    #
#                                                           #
# ========================================================= #

from audio_common_msgs.msg import AudioData # Подключаем тип сообщения для топиков с аудио-сигналом

import rospy # библиотека для работы с ROS
import os # библиотека для работы с ОС
import pyaudio # библиотека для работы со звуком (нужна, чтоб получить значение формата аудио (PaInt16 в байтах = 2))
import wave # библиотека для работы с .wav файлами

# Класс "записывателя" (рекордера)
class Recorder():
    def __init__(self, filename, duration):
        script_path = os.path.dirname(os.path.abspath(__file__)) + '/' # Получаем путь текущей директории
        self.filename = filename
        self.path_to_file = script_path+filename # формируем абсолютный путь до файла, в который будем сохранять аудио
        self.CHUNK = 1024 # количество frame в буффере, то же самое что: period, framebuffer и т.д. Должно совпадать со значением конфиг-файла
        self.RATE = 16000 # Частота дискретизации микрофона. Должно совпадать со значением конфиг-файла
        self.RECORD_SECONDS = duration # Длительность записи
        self.FORMAT = pyaudio.get_sample_size(pyaudio.paInt16) # Получаем формат PaInt16 в байтах для .wav файла

        self.frames = [] # Массив, куда складываются все аудио-фреймы
        self.count_of_frames = 0 # Количество записанных фреймов
        self.is_end = 0

    def callback(self, msg:AudioData):
        if self.is_end: # Если запись завершена, то игнорируем все поступающие сообщения из топика
            return

        print(self.filename, ":", self.count_of_frames, '/', int(self.RATE / self.CHUNK * self.RECORD_SECONDS)) # печатает сколько записано frame`ов

        self.frames.append(msg.data) # Добавляем в массив пришедший аудио-фрейм
        self.count_of_frames += 1 # увеличиваем счетчик фреймов в массиве

        if (self.count_of_frames > int(self.RATE / self.CHUNK * self.RECORD_SECONDS)): # Если записано нужное количество frame`ов (вычисляется на основе необходимой длительности записи)
            # То записываем self.frames в файл .wav:

            self.is_end = 1 # Устанавливаем значения флага 1, сигнализируя, что запись завершена
            wf = wave.open(self.path_to_file, 'wb') # Открываем файл self.path_to_file в режиме записи байтов 'wb'
            wf.setnchannels(1) # Устанавливаем количество каналов 1 - монозвук
            wf.setsampwidth(self.FORMAT) # Устанавливаем формат аудио-фреймов (в байтах PaInt16)
            wf.setframerate(self.RATE) # Устанавливаем частоту дискретизации
            wf.writeframes(b''.join(self.frames)) # Записываем массив self.frames в файл
            wf.close() # Сохраням и закрываем .wav файл
            print(self.filename, ": Recorded to:", self.path_to_file)

duration = 5 # Длительность записи аудио
rec_0 = Recorder("rec_0.wav", duration) # channel 0 - обработанный звук самим respeaker
rec_1 = Recorder("rec_1.wav", duration) # channel 1 - raw звук с микрофона 1
rec_2 = Recorder("rec_2.wav", duration) # channel 2 - raw звук с микрофона 2
rec_3 = Recorder("rec_3.wav", duration) # channel 3 - raw звук с микрофона 3
rec_4 = Recorder("rec_4.wav", duration) # channel 4 - raw звук с микрофона 4
rec_5 = Recorder("rec_5.wav", duration) # channel 5 - звук, воспроизводимый на динамиках
rec_main = Recorder("rec_main.wav", duration) # main_channel - звук с главного канала (по умолчанию дублирует значения с канала 0)

rospy.init_node("example_recording") # Инициализируем ROS-ноду
print("Start recording!")
rospy.Subscriber("/respeaker_driver/audio/channel_0", AudioData, rec_0.callback) # Подписываемся на топик
rospy.Subscriber("/respeaker_driver/audio/channel_1", AudioData, rec_1.callback) # Подписываемся на топик
rospy.Subscriber("/respeaker_driver/audio/channel_2", AudioData, rec_2.callback) # Подписываемся на топик
rospy.Subscriber("/respeaker_driver/audio/channel_3", AudioData, rec_3.callback) # Подписываемся на топик
rospy.Subscriber("/respeaker_driver/audio/channel_4", AudioData, rec_4.callback) # Подписываемся на топик
rospy.Subscriber("/respeaker_driver/audio/channel_5", AudioData, rec_5.callback) # Подписываемся на топик
rospy.Subscriber("/respeaker_driver/audio/main", AudioData, rec_main.callback) # Подписываемся на топик

rospy.Timer(rospy.Duration(duration+1), callback=lambda e: rospy.signal_shutdown("end record")) # Таймер для завершения ROS.
rospy.spin() # Зацикливаем ROS, для обновления топиков