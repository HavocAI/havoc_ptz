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

        #Track a target initializations
        self.camera_position = [0.0,0.0,0.0] #(lat, lon, alt)
        self.target_position = [0.0,0.0,0.0] #(lat, lon, alt)

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
        time.sleep(0.5)  # Wait for the move to complete

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

    def geodetic_to_ecef(lat, lon, alt):
        # WGS84 ellipsoid constants:
        a = 6378137.0          # semi-major axis
        e2 = 6.69437999014e-3  # first eccentricity squared

        lat = np.radians(lat)
        lon = np.radians(lon)

        N = a / np.sqrt(1 - e2 * np.sin(lat)**2)

        x = (N + alt) * np.cos(lat) * np.cos(lon)
        y = (N + alt) * np.cos(lat) * np.sin(lon)
        z = (N * (1 - e2) + alt) * np.sin(lat)

        return np.array([x, y, z])


    def enu_vector_to_relative_bearing(boat_pos, target_pos, boat_heading_deg):
        """
        Returns the angle (degrees) from the boat heading to the target direction in the ENU plane.
        Positive = target is to the right (starboard), negative = left (port).
        0 = target is straight ahead of the boat heading.
        """
        # Compute ENU vector from boat to target
        p_boat = geodetic_to_ecef(*boat_pos)
        p_target = geodetic_to_ecef(*target_pos)
        vec = p_target - p_boat
        lat0, lon0, _ = boat_pos
        lat0 = np.radians(lat0)
        lon0 = np.radians(lon0)
        east = np.array([-np.sin(lon0), np.cos(lon0), 0])
        north = np.array([-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)])
        e = np.dot(vec, east)
        n = np.dot(vec, north)
        # Angle of target vector in ENU (0 = North, increases clockwise)
        target_angle = np.degrees(np.arctan2(e, n)) % 360
        # Relative angle to boat heading
        rel_angle = (target_angle - boat_heading_deg + 180) % 360 - 180
        return rel_angle


    def compute_tilt_angle(boat_pos, target_pos, boat_pitch_deg):
        """
        Computes the tilt angle required to point at the target from the boat, compensating for the boat's pitch.
        - boat_pos: (lat, lon, alt) of the boat
        - target_pos: (lat, lon, alt) of the target
        - boat_pitch_deg: pitch of the boat in degrees (positive = bow up, negative = bow down)

        Returns:
        - tilt angle in degrees (positive = up, negative = down)
        """
        # Extract altitude differences
        alt_diff = target_pos[2] - boat_pos[2]

        # Compute ENU vector from boat to target
        p_boat = geodetic_to_ecef(*boat_pos)
        p_target = geodetic_to_ecef(*target_pos)
        vec = p_target - p_boat

        # Horizontal distance in EN plane
        lat0, lon0, _ = boat_pos
        lat0 = np.radians(lat0)
        lon0 = np.radians(lon0)
        east = np.array([-np.sin(lon0), np.cos(lon0), 0])
        north = np.array([-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)])
        e = np.dot(vec, east)
        n = np.dot(vec, north)
        horiz_dist = np.linalg.norm([e, n])

        
        # Elevation angle (from local horizontal up to target)
        elevation = np.degrees(np.arctan2(alt_diff, horiz_dist))

        # Compensate for boat pitch
        tilt_angle = elevation - boat_pitch_deg
        print(horiz_dist, alt_diff,elevation, boat_pitch_deg, tilt_angle)
        return tilt_angle
    

    def pan_tilt_zoom_angle_to_ptz_frame(pan_deg, tilt_deg, zoom_percent):
        #Pan: 0 = 0 degrees, 1 = 180 degrees, -1 = -180 degrees 
        #Tilt: 1 = -30, 0 = 30, -1 = 90 degrees
        #Zoom: 0-1

        #Rescale the pan_deg angle to match the pelco pan range
        #Assuming 0 is north for now and 180 is south and max is 360 degrees in pelco world
        ptz_pan = 0.0
        ptz_tilt = 0.0
        
        if pan_deg > 180.0:
            ptz_pan = ((pan_deg-360.0)/180.0)
        elif pan_deg <= 180.0:
            ptz_pan = (pan_deg / 180.0)

        #Rescale the tilt_deg angle to match the pelco tilt range
        if tilt_deg < -30.0:
            tilt = 1.0
        elif tilt_deg > 90.0:
            tilt_deg = -1.0
        else:
            if tilt_deg <= 30.0:
                angle = tilt_deg - 30.00
                tilt = -1.0 * (angle / 60.0) 
            elif tilt_deg > 30.0:
                angle = tilt_deg - 30.0
                tilt = -1.0 * (angle / 60.0)
        ptz_tilt = tilt

        ptz_zoom = (zoom_percent/100.0)

        #clip numbers to valid ranges
        ptz_pan = max(-1.0, min(1.0, ptz_pan))
        ptz_tilt = max(-1.0, min(1.0, ptz_tilt))
        ptz_zoom = max(0.0, min(1.0, ptz_zoom)) 

        return ptz_pan, ptz_tilt, ptz_zoom

    def track_target(self, target_position, boat_position, boat_heading_deg, boat_pitch_deg, zoom_percent=0.0):
        """
        Track a target by calculating the required pan and tilt angles based on the target's position
        relative to the camera's position and the boat's heading and pitch. 
        - can update camera commands at ~2hz max.
        - target_position: (lat, lon, alt) of the target
        - boat_position: (lat, lon, alt) of the boat
        - boat_heading_deg: heading of the boat in degrees (0 = North, increases clockwise)
        - boat_pitch_deg: pitch of the boat in degrees (positive = bow up, negative = bow down)
        - zoom_percent: zoom level as a percentage (0-100)

        """
        # Calculate relative bearing to target
        pan_angle = self.enu_vector_to_relative_bearing(boat_position, target_position, boat_heading_deg)
        
        # Calculate tilt angle to target
        tilt_angle = self.compute_tilt_angle(boat_position, target_position, boat_pitch_deg)

        ptz_pan, ptz_tilt, ptz_zoom = self.pan_tilt_zoom_angle_to_ptz_frame(pan_angle, tilt_angle, zoom_percent)  # Assuming zoom at 100%
        
        self.move_to_absolute(ptz_pan, ptz_tilt, ptz_zoom)

        return {ptz_pan, ptz_tilt, ptz_zoom, pan_angle, tilt_angle}