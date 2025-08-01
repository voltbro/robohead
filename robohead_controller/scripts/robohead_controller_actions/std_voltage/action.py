from robohead_controller_actions.main import *
import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import BatteryState
import rospy  # Убедимся, что rospy импортирован

def run(robohead_controller: RoboheadController, cmds: str):
    """
    Отображает напряжение батареи на экране робота на черном фоне.

    Args:
        robohead_controller: Экземпляр контроллера робоhead.
        cmds: Строка команд (в данном скрипте не используется).
                      Если нужно задать время, можно передать как аргумент,
                      но в данном примере N зашито в коде.
    """
    # --- Конфигурация ---
    DISPLAY_DURATION = 20.0  # Время отображения в секундах (N)
    SCREEN_WIDTH = 1080
    SCREEN_HEIGHT = 1080
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 10
    FONT_COLOR = (255, 255, 255)  # Белый
    THICKNESS = 10
    LINE_TYPE = cv2.LINE_AA
    # --------------------
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = SetModeLEDRequest()
    msg.mode = robohead_controller.respeaker_driver_default_led_mode
    robohead_controller.respeaker_driver_srv_SetModeLED(msg)

    msg = SetBrightnessLEDRequest()
    msg.brightness = robohead_controller.respeaker_driver_default_led_brightness
    robohead_controller.respeaker_driver_srv_SetBrightnessLED(msg)

    msg = SetColorPaletteLEDRequest()
    msg.colorA = robohead_controller.respeaker_driver_default_led_A_color
    msg.colorB = robohead_controller.respeaker_driver_default_led_B_color
    robohead_controller.respeaker_driver_srv_SetColorPaletteLED(msg)

    msg = PlayMediaRequest()
    msg.is_blocking = 0
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'wait.png'
    robohead_controller.display_driver_srv_PlayMedia(msg)

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = 0
    msg.right_ear_angle = 0
    robohead_controller.ears_driver_srv_EarsSetAngle(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 20
    msg.duration = 1
    msg.is_blocking = 1
    robohead_controller.neck_driver_srv_NeckSetAngle(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = ""
    msg.is_blocking = 0
    msg.is_cycled = 0
    robohead_controller.speakers_driver_srv_PlayAudio(msg)

    # Инициализируем CvBridge
    cv_bridge = CvBridge()

    # Создаем черный фон
    black_image = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH, 3), dtype=np.uint8)

    # Переменная для хранения последнего значения напряжения
    last_voltage = 0.0

    # Callback для обновления напряжения
    def battery_callback(data: BatteryState):
        nonlocal last_voltage
        last_voltage = data.voltage

    # Подписываемся на топик с напряжением
    rospy.Subscriber("/robohead_controller/sensor_driver/bat", BatteryState, battery_callback)

    # Небольшая пауза, чтобы получить первое сообщение
    rospy.sleep(0.5)

    # Засекаем время начала
    start_time = rospy.get_time()

    rospy.loginfo(f"Начинаю отображение напряжения на экране на {DISPLAY_DURATION} секунд.")

    # Цикл отображения
    rate = rospy.Rate(30) # ~30 FPS
    while (rospy.get_time() - start_time) < DISPLAY_DURATION and not rospy.is_shutdown():
        # Создаем копию фона для нового кадра
        display_image = black_image.copy()

        # Формируем строку напряжения
        voltage_str = f"{last_voltage:.2f} V"

        # Вычисляем размер текста для центрирования
        text_size, _ = cv2.getTextSize(voltage_str, FONT, FONT_SCALE, THICKNESS)
        text_x = (SCREEN_WIDTH - text_size[0]) // 2
        text_y = (SCREEN_HEIGHT + text_size[1]) // 2

        # Наносим текст на изображение
        cv2.putText(display_image, voltage_str, (text_x, text_y), FONT, FONT_SCALE, FONT_COLOR, THICKNESS, LINE_TYPE)

        # Публикуем изображение в топик для отображения на экране
        try:
            img_msg = cv_bridge.cv2_to_imgmsg(display_image, encoding="bgr8")
            robohead_controller.display_driver_pub_PlayMedia.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Ошибка при публикации изображения: {e}")

        rate.sleep()
    
    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'finish_voice.mp3'
    msg.is_blocking = 1
    msg.is_cycled = 0
    robohead_controller.speakers_driver_srv_PlayAudio(msg)

    rospy.loginfo("Завершено отображение напряжения.")