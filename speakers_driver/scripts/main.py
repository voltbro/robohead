import rospy
import mpd
import os
import urllib.request

from speakers_driver.srv import PlayAudio, PlayAudioRequest, PlayAudioResponse
from speakers_driver.srv import GetVolume, GetVolumeRequest, GetVolumeResponse
from speakers_driver.srv import SetVolume, SetVolumeRequest, SetVolumeResponse

class SpeakersDriver():
    def __init__(self):        
        service_PlayAudio_name = rospy.get_param("~service_PlayAudio_name", "~PlayAudio")
        service_GetVolume_name = rospy.get_param("~service_GetVolume_name", "~GetVolume")
        service_SetVolume_name = rospy.get_param("~service_SetVolume_name", "~SetVolume")

        mpd_host = rospy.get_param("~mpd_host", "/run/mpd/socket")
        mpd_port = rospy.get_param("~mpd_port", 6600)

        self._update_hz = rospy.get_param("~update_hz", 10)
        default_volume = rospy.get_param("~default_volume", 50)

        rospy.Service(service_PlayAudio_name, PlayAudio, self._requester_PlayAudio)
        rospy.Service(service_GetVolume_name, GetVolume, self._requester_GetVolume)
        rospy.Service(service_SetVolume_name, SetVolume, self._requester_SetVolume)

        self._mpd_client = mpd.MPDClient()
        self._mpd_client.connect(mpd_host, mpd_port)

        self._mpd_client.timeout = 10               
        self._mpd_client.idletimeout = None

        self._mpd_client.repeat(0)
        self._mpd_client.random(0)
        self._mpd_client.single(0)
        self._mpd_client.consume(0)
        self._mpd_client.setvol(default_volume)
        rospy.Timer(period=rospy.Duration(secs=10.0), callback=self._ping_mpd, oneshot=False)

        rospy.loginfo("speakers_driver INITED")
    def _ping_mpd(self, event):
        self._mpd_client.ping()

    def __del__(self):
        self._mpd_client.close()                     # send the close command
        self._mpd_client.disconnect()                # disconnect from the server
    
    def _requester_GetVolume(self, request:GetVolumeRequest):
        response = GetVolumeResponse()
        response.value = int(self._mpd_client.status()['volume'])
        return response

    def _requester_SetVolume(self, request:SetVolumeRequest):
        response = SetVolumeResponse()
        if (0<=request.volume<=100):
            self._mpd_client.setvol(request.volume)
            response.value = 0
        else:
            response.value = -1
        return response

    def _requester_PlayAudio(self, request:PlayAudioRequest):
        response = PlayAudioResponse()
        if (request.path_to_file==''):
            self._mpd_client.stop()
            response = 0

        else:
            if os.path.exists(request.path_to_file):
                self._mpd_client.clear()
                res = self._mpd_client.add(request.path_to_file)

                if request.is_cycled!=0:
                    self._mpd_client.repeat(1)
                else:
                    self._mpd_client.repeat(0)
                
                self._mpd_client.play()


                rate = rospy.Rate(self._update_hz)
                while (request.is_blocking!=0) and (self._mpd_client.status()['state']=='play'):
                    rate.sleep()

                response.value = 0
            else:
                try:
                    urllib.request.urlopen(request.path_to_file)
                    self._mpd_client.clear()
                    res = self._mpd_client.add(request.path_to_file)

                    if request.is_cycled!=0:
                        self._mpd_client.repeat(1)
                    else:
                        self._mpd_client.repeat(0)
                    
                    self._mpd_client.play()


                    rate = rospy.Rate(self._update_hz)
                    while (request.is_blocking!=0) and (self._mpd_client.status()['state']=='play'):
                        rate.sleep()

                    response.value = 0
                except ValueError:
                    response.value = -1
        return response

if __name__ == "__main__":
    rospy.init_node("speakers_driver")
    obj = SpeakersDriver()
    rospy.spin()