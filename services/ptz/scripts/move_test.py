# -*- coding: utf-8 -*-
import sys
from onvif import ONVIFCamera
from time import sleep
import time

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

import pygame

#import keyboard

class ptzControl(object):
    def __init__(self,IP,PORT,USER,PASS):
        """Initialize the PTZ control with camera connection parameters."""
        if not all([IP, PORT, USER, PASS]):
            raise ValueError("All parameters (IP, PORT, USER, PASS) must be provided.")
        if not isinstance(IP, str) or not isinstance(USER, str) or not isinstance(PASS, str):
            raise TypeError("IP, USER, and PASS must be strings.")
        if not isinstance(PORT, int) or PORT <= 0:
            raise ValueError("PORT must be a positive integer.")    
        
        super(ptzControl, self).__init__()
        self.mycam = ONVIFCamera(IP,PORT,USER,PASS)
        # create media service object
        self.media = self.mycam.create_media_service()
        # Get target profile
        self.media_profile = self.media.GetProfiles()[0]
        # Use the first profile and Profiles have at least one
        token = self.media_profile.token
        # PTZ controls  -------------------------------------------------------------
        self.ptz = self.mycam.create_ptz_service()
        # Get available PTZ services
        request = self.ptz.create_type('GetServiceCapabilities')
        Service_Capabilities = self.ptz.GetServiceCapabilities(request)
        print(Service_Capabilities)
        # Get PTZ status
        status = self.ptz.GetStatus({'ProfileToken': token})
        # Get PTZ configuration options for getting option ranges
        request = self.ptz.create_type('GetConfigurationOptions')
        request.ConfigurationToken = self.media_profile.PTZConfiguration.token
        ptz_configuration_options = self.ptz.GetConfigurationOptions(request)
        print(ptz_configuration_options)
        self.ptz_configuration_options = ptz_configuration_options
        # get continuousMove request -- requestc
        self.requestc = self.ptz.create_type('ContinuousMove')
        self.requestc.ProfileToken = self.media_profile.token
        if self.requestc.Velocity is None:
            self.requestc.Velocity = self.ptz.GetStatus({'ProfileToken': self.media_profile.token}).Position
            self.requestc.Velocity.PanTilt.space = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].URI
            self.requestc.Velocity.Zoom.space = ptz_configuration_options.Spaces.ContinuousZoomVelocitySpace[0].URI

        # get absoluteMove request -- requesta
        self.requesta = self.ptz.create_type('AbsoluteMove')
        self.requesta.ProfileToken = self.media_profile.token
        if self.requesta.Position is None:
            self.requesta.Position = self.ptz.GetStatus({'ProfileToken': self.media_profile.token}).Position
            self.requesta.Position.PanTilt.space = self.ptz_configuration_options.Spaces.AbsolutePanTiltPositionSpace[0].URI
            self.requesta.Position.Zoom.space = self.ptz_configuration_options.Spaces.AbsoluteZoomPositionSpace[0].URI
        if self.requesta.Speed is None:
            self.requesta.Speed = self.ptz.GetStatus({'ProfileToken': self.media_profile.token}).Position
            self.requesta.Speed.PanTilt.space = self.ptz_configuration_options.Spaces.PanTiltSpeedSpace[0].URI

        # get relativeMove request -- requestr
        self.requestr = self.ptz.create_type('RelativeMove')
        self.requestr.ProfileToken = self.media_profile.token
        if self.requestr.Translation is None:
            self.requestr.Translation = self.ptz.GetStatus(
                {'ProfileToken': self.media_profile.token}).Position
            self.requestr.Translation.PanTilt.space = ptz_configuration_options.Spaces.RelativePanTiltTranslationSpace[0].URI
            self.requestr.Translation.Zoom.space = ptz_configuration_options.Spaces.RelativeZoomTranslationSpace[0].URI
        if self.requestr.Speed is None:
            self.requestr.Speed = self.ptz.GetStatus(
                {'ProfileToken': self.media_profile.token}).Position

        self.requests = self.ptz.create_type('Stop')
        self.requests.ProfileToken = self.media_profile.token
        self.requestp = self.ptz.create_type('SetPreset')
        self.requestp.ProfileToken = self.media_profile.token
        self.requestg = self.ptz.create_type('GotoPreset')
        self.requestg.ProfileToken = self.media_profile.token
        self.stop()

        self.gps_position = NavSatFix()
        self.gps_position.latitude = 0.0
        self.gps_position.longitude = 0.0
        self.gps_position.altitude = 0.0

        self.target_position = NavSatFix()
        self.target_position.latitude = 0.0
        self.target_position.longitude = 0.0
        self.target_position.altitude = 0.0 

    def get_state(self):
        """Return the current state of the PTZ control."""
        status = self.ptz.GetStatus({'ProfileToken': self.media_profile.token})   
        self.pan = status.Position.PanTilt.x
        self.tilt = status.Position.PanTilt.y
        self.zoom = status.Position.Zoom.x
        return {'pan': self.pan, 'tilt': self.tilt, 'zoom': self.zoom}

    # Stop pan, tilt and zoom
    def stop(self):
        self.requests.PanTilt = True
        self.requests.Zoom = True
        print(f"self.request:{self.requests}")
        self.ptz.Stop(self.requests)

    # Continuous move functions
    def perform_move(self, requestc):
        # Start continuous move
        ret = self.ptz.ContinuousMove(requestc)

    def move_tilt(self, velocity):
        self.requestc.Velocity.PanTilt.x = 0.0
        self.requestc.Velocity.PanTilt.y = velocity
        self.perform_move(self.requestc)

    def move_pan(self, velocity):
        self.requestc.Velocity.PanTilt.x = velocity
        self.requestc.Velocity.PanTilt.y = 0.0
        self.perform_move(self.requestc)

    def move_continuous(self, pan, tilt):
        self.requestc.Velocity.PanTilt.x = pan
        self.requestc.Velocity.PanTilt.y = tilt
        self.perform_move(self.requestc)

    def move_to_absolute(self, pan, tilt, zoom):
        self.requesta.Position.PanTilt.x = pan
        self.requesta.Position.PanTilt.y = tilt
        self.requesta.Position.Zoom.x = zoom
        self.requesta.Speed.PanTilt.x = 1.0
        self.requesta.Speed.PanTilt.y = 1.0
        self.ptz.Stop({'ProfileToken': self.media_profile.token})  # Stop any ongoing movement
        self.ptz.AbsoluteMove(self.requesta)
        time.sleep(1)  # Wait for the move to complete

    def zoom(self, velocity):
        self.requestc.Velocity.Zoom.x = velocity
        self.perform_move(self.requestc)


    # Absolute move functions --NO ERRORS BUT CAMERA DOES NOT MOVE
    def move_abspantilt(self, pan, tilt, velocity):
        self.requesta.Position.PanTilt.x = pan
        self.requesta.Position.PanTilt.y = tilt
        print(self.requesta)
        #self.requesta.Speed.PanTilt.x = velocity
        #self.requesta.Speed.PanTilt.y = velocity
        self.ptz.Stop({'ProfileToken': self.media_profile.token})  # Stop any ongoing movement
        ret = self.ptz.AbsoluteMove(self.requesta)
        time.sleep(0.1)

    # Relative move functions --NO ERRORS BUT CAMERA DOES NOT MOVE
    def move_relative(self, pan, tilt, velocity):
        self.requestr.Translation.PanTilt.x = pan
        self.requestr.Translation.PanTilt.y = tilt
        self.requestr.Speed.PanTilt = [velocity,velocity]
        # self.requestr.Speed.PanTilt.x = velocity
        # self.requestr.Speed.PanTilt.y = velocity
        self.requestr.Speed.Zoom = 0
        ret = self.ptz.RelativeMove(self.requestr)

    def zoom_relative(self, zoom, velocity):
        self.requestr.Translation.PanTilt.x = 0
        self.requestr.Translation.PanTilt.y = 0
        self.requestr.Translation.Zoom.x = zoom
        self.requestr.Speed.PanTilt.x = 0
        self.requestr.Speed.PanTilt.y = 0
        self.requestr.Speed.Zoom.x = velocity
        ret = self.ptz.RelativeMove(self.requestr)

        # Sets preset set, query and and go to

    def set_preset(self, name):
        self.requestp.PresetName = name
        self.requestp.PresetToken = '1'
        self.preset = self.ptz.SetPreset(self.requestp)  # returns the PresetToken

    def get_preset(self):
        self.ptzPresetsList = self.ptz.GetPresets(self.requestc)

    def goto_preset(self):
        self.requestg.PresetToken = '1'
        self.ptz.GotoPreset(self.requestg)


if __name__ == '__main__':
    IP = "192.168.168.106"  # Camera IP address
    PORT = 80  # Port
    USER = "havoc-ptz1"  # Username
    PASS = "1nspectR"  # Password
    ptz = ptzControl(IP, PORT, USER, PASS)
    
    pygame.init()
    pygame.display.set_mode((100, 100))  # Initialize a small window for keyboard events

    com_pan = 0.0
    com_tilt = 0.0
    com_zoom = 0.0

    ptz.move_to_absolute(pan=-1.0, tilt=-0.75, zoom=0.5)
    time.sleep(2)  # Wait for the move to complete
    ptz.move_to_absolute(pan=1.0, tilt=0.75, zoom=1.0)  # Reset to initial position
    time.sleep(2)  # Wait for the move to complete
        # # Example usage
    
    pan_speed = 0.0
    tilt_speed = 0.0
    increment = 0.001
    while True:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    print("Camera Up")
                    tilt_speed += -increment
                    com_tilt += -increment
                elif event.key == pygame.K_DOWN:
                    print("Camera Down")
                    tilt_speed += increment
                    com_tilt += increment
                elif event.key == pygame.K_LEFT:
                    print("Camera Left")
                    pan_speed += -increment
                    com_pan += -increment
                elif event.key == pygame.K_RIGHT:
                    print("Camera Right")
                    pan_speed += increment
                    com_pan += increment
                elif event.key == pygame.K_q:
                    print("Exiting...")
                    break
        ptz.move_to_absolute(com_pan, com_tilt, com_zoom)
        #ptz.move_continuous(pan=pan_speed, tilt=tilt_speed)
        print('state: ',ptz.get_state())

    # start_time = time()
    # for i in  range(0,100):
    #     cur_time = time()
    #     #if i % 2 == 0:
    #     ptz.move_continuous(pan=-0.2, tilt=-0.1)
    #     #else:
    #     #    ptz.move_continuous(pan=-0.1, tilt=-0.1)
    #     print('time:',cur_time,'   status:',ptz.get_state())
    # print(f"Elapsed time: {time() - start_time:.2f} seconds")
    # print('cycle time:', (time() - start_time) / 100)
    #ptz.move_continuous(pan=0.5, tilt=-0.4)  # Move pan and tilt
    #ptz.zoom_relative(0.5, 0.4)
    #sleep(2)  # Wait for the zoom to complete
    ptz.stop()  # Stop any ongoing movement
    print("PTZ control test completed.")
    pygame.quit()