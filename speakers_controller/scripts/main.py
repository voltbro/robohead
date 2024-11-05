#!/usr/bin/python3

import rospy
from speakers_controller.srv import playSound, playSoundResponse, setVolume, setVolumeResponse, isPlaying, isPlayingResponse, stopPlaying, stopPlayingResponse
import time
import vlc
class SpeakersController():
    def __init__(self, std_volume:int=50, node_name:str="speakers_controller_node", srv_play_name:str="playSound", srv_volume_name:str="setVolume")->None:
        # std_volume: 0...100
        self.__vlc_obj = vlc.Instance() 

        # creating a media player 
        self.__vlc_player = self.__vlc_obj.media_player_new() 
        if (0<=std_volume<=100):
            self.__vlc_player.audio_set_volume(std_volume)
            self.volume = std_volume
        else:
            print("error volume")

        rospy.init_node(node_name)
        srv_playSound = rospy.Service(srv_play_name, playSound, self.requester_playSound)
        srv_setVolume = rospy.Service(srv_volume_name, setVolume, self.requester_setVolume)
        srv_isPlaying = rospy.Service("isPlaying", isPlaying, self.requester_isPlaying)
        srv_stopPlaying = rospy.Service("stopPlaying", stopPlaying, self.requester_stopPlaying)
        print("Ready")

        rospy.spin()

    def requester_isPlaying(self, request:isPlaying):
        return isPlayingResponse(self.is_playing_sound())

    def requester_stopPlaying(self, request:stopPlaying):
        self.stop_sound()
        return stopPlayingResponse(0)

    def requester_playSound(self, request:playSound):
        self.play_sound(src=request.FileName, breakable=request.IsBreakable)
        return playSoundResponse(0)

    def requester_setVolume(self, request:setVolume):
        self.set_volume(volume=request.Volume)
        return setVolumeResponse(0)
    
    def play_sound(self, src:str, breakable:bool=True)->None:
        # creating a media 
        vlcmedia = self.__vlc_obj.media_new(src)
        
        # setting media to the player
        self.__vlc_player.set_media(vlcmedia)
        self.__vlc_player.audio_set_volume(self.volume)
        self.__vlc_player.play()
        time.sleep(0.1)
        if breakable:
            while self.__vlc_player.is_playing():
                pass
    
    def stop_sound(self)->None:
        self.__vlc_player.stop()

    def is_playing_sound(self)->bool:
        return self.__vlc_player.is_playing()
    
    def set_volume(self, volume:int)->None:
        if (0<=volume<=100):
            self.volume = volume
            self.__vlc_player.audio_set_volume(volume)

if __name__ == "__main__":
    SpeakersController()