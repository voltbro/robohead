import struct
import rospy
import pyaudio
import numpy as np
import usb.core
import usb.util
import math

from std_msgs.msg import Bool, Int16, ColorRGBA
from audio_common_msgs.msg import AudioData

from respeaker_driver_dependencies.pixel_ring import PixelRing
from respeaker_driver_dependencies.utils import *

class RespeakerDriver():        
    def __init__(self):
        # Get rosparams
        vendor_id = rospy.get_param("~usb/vendor_id", 0x2886)
        product_id = rospy.get_param("~usb/product_id", 0x0018)
        reset_time_sleep = rospy.get_param("~usb/reset_time_sleep", 10)
        self._timeout = rospy.get_param("~usb/timeout", 8000)

        rate = rospy.get_param("~audio/rate", 16000)
        chunk = rospy.get_param("~audio/chunk", 1024)

        default_brightness = rospy.get_param("~led/brightness", 10)
        A_color = rospy.get_param("~led/A_color", [0,0,255])
        B_color = rospy.get_param("~led/B_color", [0,255,0])
        
        topic_audio_main_name = rospy.get_param("~ros/topic_audio_main_name", "~audio/main")
        topic_audio_channel_names = {
            0 : rospy.get_param("~ros/topic_audio_channel_0_name", "~audio/channel_0"),
            1 : rospy.get_param("~ros/topic_audio_channel_1_name", "~audio/channel_1"),
            2 : rospy.get_param("~ros/topic_audio_channel_2_name", "~audio/channel_2"),
            3 : rospy.get_param("~ros/topic_audio_channel_3_name", "~audio/channel_3"),
            4 : rospy.get_param("~ros/topic_audio_channel_4_name", "~audio/channel_4"),
            5 : rospy.get_param("~ros/topic_audio_channel_5_name", "~audio/channel_5"),
        }
        self._main_channel = rospy.get_param("~ros/main_channel", 0)
        topic_doa_angle_name = rospy.get_param("~ros/topic_doa_angle_name", "~doa_angle")

        self._doa_yaw_offset = rospy.get_param("~doa_yaw_offset", 0)

        # INIT dev to get DOA, set params, control PixelRing
        self.dev = usb.core.find(idVendor=vendor_id,
                                 idProduct=product_id)
        if not self.dev:
            rospy.logerr("can`t find dev=respeaker by vendor_id and product_id")

        self.pixel_ring = PixelRing(self.dev)

        try:
            self.pixel_ring.set_vad_led(1)
        except usb.USBError as err:
            rospy.logerr(f"error in dev: {err}\nReset usb respeaker...")
            self.dev.reset()
            rospy.sleep(reset_time_sleep)
        self.pixel_ring.set_vad_led(2)

        self.pixel_ring.set_brightness(default_brightness)
        self.pixel_ring.set_color_palette(A_color[0],A_color[1],A_color[2],B_color[0],B_color[1],B_color[2])
        self.pixel_ring.trace()

        # INIT respeaker as audio input
        self.available_channels = None
        device_index = None
        with ignore_stderr(enable=True):
            self.pyaudio = pyaudio.PyAudio()

        count = self.pyaudio.get_device_count()
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            name = info["name"]
            maxInputChannels = info["maxInputChannels"]

            if name.lower().find("respeaker") >= 0:
                self.available_channels = maxInputChannels
                device_index = i
                break

        if device_index is None:
            rospy.logerr("respeaker_driver: Failed to find audio respeaker device by name")
            
        if self.available_channels != 6:
            rospy.loginfo(f"{self.available_channels} channel is found for audio respeaker device\nYou may have to update firmware of respeaker.")

        try:
            self.stream = self.pyaudio.open(
                input=True, start=False,
                format=pyaudio.paInt16,
                channels=self.available_channels,
                rate=rate,
                frames_per_buffer=chunk,
                stream_callback=self._stream_callback,
                input_device_index=device_index,
            )
        except BaseException as err:
            rospy.logerr(f"error in stream: {err}\nReset usb respeaker...")
            self.dev.reset()
            rospy.sleep(reset_time_sleep)

        # INIT ROS subjects
        self.pub_audio_main = rospy.Publisher(topic_audio_main_name, AudioData, queue_size=10)
        self.pub_audio_channels = {c:rospy.Publisher(topic_audio_channel_names[c], AudioData, queue_size=10) for c in range(self.available_channels)}
        self.pub_doa = rospy.Publisher(topic_doa_angle_name, Int16, queue_size=10)

        self.prev_doa = 400 # просто стартовое значение
        rospy.loginfo("respeaker_driver INITED")

    def __del__(self):
        try:
            usb.util.dispose_resources(self.dev)
            self.dev.close()
        except:
            pass
        finally:
            self.dev = None
        try:
            self.stream.close()
        except:
            pass
        finally:
            self.stream = None
        try:
            self.pyaudio.terminate()
        except:
            pass

    def start_stream(self):
        if self.stream.is_stopped():
            self.stream.start_stream()

    def stop_stream(self):
        if self.stream.is_active():
            self.stream.stop_stream()
    
    def get_doa_angle(self):
        doa = self._read('DOAANGLE')
        rad = math.radians(doa)
        x = math.cos(rad)
        y = math.sin(rad)
        rotate = math.radians(self._doa_yaw_offset)
        x_ = x*math.cos(rotate)+y*math.sin(rotate)
        y_ = -x*math.sin(rotate)+y*math.cos(rotate)
        doa_angle = int(math.degrees(math.atan2(y_, x_)))
        return -doa_angle

    def _stream_callback(self, in_data, frame_count, time_info, status):
        doa = self.get_doa_angle()
        if self.prev_doa != doa:
            self.prev_doa = doa
            self.pub_doa.publish(doa)

        data = np.frombuffer(buffer=in_data, dtype=np.int16)
        chunk_per_channel = len(data) / self.available_channels
        data = np.reshape(data, (int(chunk_per_channel), self.available_channels))

        for channel in range(self.available_channels):
            channel_data = data[:, channel]

            msg = AudioData()
            msg.data = channel_data.tostring()
            self.pub_audio_channels[channel].publish(msg)
            if channel == self._main_channel:
                self.pub_audio_main.publish(msg)

        return None, pyaudio.paContinue

    def _write(self, name:str, value:float):
        try:
            data = RESPEAKER_PARAMETERS[name]
        except KeyError:
            rospy.logerr(f"Parameter {name} was not found")
            return

        if data[5] == 'ro':
            rospy.loginfo(f'Parameter {name} is read-only')

        id = data[0]

        # 4 bytes offset, 4 bytes value, 4 bytes type
        if data[2] == 'int':
            payload = struct.pack(b'iii', data[1], int(value), 1)
        else:
            payload = struct.pack(b'ifi', data[1], float(value), 0)

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0, id, payload, self._timeout)

    def _read(self, name:str):
        try:
            data = RESPEAKER_PARAMETERS[name]
        except KeyError:
            rospy.logerr(f"Parameter {name} was not found")
            return

        id = data[0]

        cmd = 0x80 | data[1]
        if data[2] == 'int':
            cmd |= 0x40

        length = 8

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, id, length, self._timeout)

        response = struct.unpack(b'ii', response.tobytes())

        if data[2] == 'int':
            result = response[0]
        else:
            result = response[0] * (2.**response[1])

        return result

if __name__ == '__main__':
    rospy.init_node("respeaker_node")
    obj = RespeakerDriver()
    obj.start_stream()
    rospy.spin()