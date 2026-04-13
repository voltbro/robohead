from robohead_controller_actions.main import *
import rospy
import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np

latest_image_msg = None

def image_callback(msg):
    global latest_image_msg
    latest_image_msg = msg

def run(robohead_controller: RoboheadController, cmds: str):
    global latest_image_msg
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    # Воспроизведение аудио
    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'look_around.mp3'
    msg.is_blocking = 0
    msg.is_cycled = 0
    robohead_controller.speakers_driver_srv_PlayAudio(msg)

    # Определяем последовательность движений шеи
    neck_movements = [
        {"horizontal_angle": -30, "vertical_angle": 0, "duration": 2},
        {"horizontal_angle": 30, "vertical_angle": 0, "duration": 2},
        {"horizontal_angle": -30, "vertical_angle": 0, "duration": 2},
        {"horizontal_angle": 30, "vertical_angle": 0, "duration": 2},
        {"horizontal_angle": 0, "vertical_angle": 0, "duration": 2},
    ]

    # Подписка на видеопоток
    rospy.Subscriber('/front_camera/image_raw/compressed', CompressedImage, image_callback)
    rospy.sleep(0.5)  # Даем время для получения первого кадра

    cvBridge = CvBridge()
    start_time = rospy.get_time()

    movement_index = 0  # Индекс текущего движения
    next_movement_time = start_time  # Время начала следующего движения

    while rospy.get_time() - start_time < 8.0:
        # Управление движением шеи
        current_time = rospy.get_time()
        if movement_index < len(neck_movements) and current_time >= next_movement_time:
            movement = neck_movements[movement_index]
            msg = NeckSetAngleRequest()
            msg.horizontal_angle = movement["horizontal_angle"]
            msg.vertical_angle = movement["vertical_angle"]
            msg.duration = movement["duration"]
            msg.is_blocking = 0  # Неблокирующий вызов
            robohead_controller.neck_driver_srv_NeckSetAngle(msg)

            # Обновляем индекс и время следующего движения
            movement_index += 1
            next_movement_time = current_time + movement["duration"]

        # Отображение видео
        if latest_image_msg:
            # Если есть данные с топика /front_camera/image_raw/compressed
            np_arr = np.frombuffer(latest_image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            # Если данных с топика нет, используем usb_cam_image_raw
            try:
                cv_image = cvBridge.imgmsg_to_cv2(robohead_controller.usb_cam_image_raw, "bgr8")
            except Exception as e:
                rospy.logwarn(f"Ошибка при получении изображения из usb_cam_image_raw: {e}")
                cv_image = np.zeros((1080, 1080, 3), dtype=np.uint8)  # Черный экран в случае ошибки

        # Изменяем размер изображения до 1080x1080
        cv_image = cv2.resize(cv_image, (1080, 1080))

        # Отображаем изображение на экране
        robohead_controller.display_driver_pub_PlayMedia.publish(
            cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        rospy.sleep(0.05)

    rospy.sleep(2)