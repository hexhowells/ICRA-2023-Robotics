from controller import Robot
import sys
sys.path.append('..')

from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
from utils.camera import Camera

import cv2
import numpy as np
import random
import time

from floor import Floor
from imutils import filter_lines

#from inference import ObjModel


class HexBot (Robot):
    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = Camera(self)
        self.cameraBottom = Camera(self, camera_name='CameraBottom')
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        self.counter = 0
        self.opponent_x = [0]*10

        self.floor_model = Floor(threshold=10, img_step=5, img_size=(160, 120))

        self.sonarL = self.getDevice('Sonar/Left')
        self.sonarL.enable(1)
        self.sonarR = self.getDevice('Sonar/Right')
        self.sonarR.enable(1)

        self.head = self.getDevice('HeadPitch')
        self.head.setPosition(0.25)
        
        self.last_time = 0

        #self.obj_model = ObjModel("ObjDetectModel_best.pth")

        self.LHipPitch = self.getDevice('LHipPitch')
        self.RHipPitch = self.getDevice('RHipPitch')

        self.LKneePitch = self.getDevice('LKneePitch')
        self.RKneePitch = self.getDevice('RKneePitch')

        self.LAnklePitch = self.getDevice('LAnklePitch')
        self.RAnklePitch = self.getDevice('RAnklePitch')

        self.RShoulderRoll = self.getDevice('RShoulderRoll')
        self.LShoulderRoll = self.getDevice('LShoulderRoll')

        self.RShoulderPitch = self.getDevice('RShoulderPitch')
        self.LShoulderPitch = self.getDevice('LShoulderPitch')

        self.RElbowRoll = self.getDevice('RElbowRoll')
        self.LElbowRoll = self.getDevice('LElbowRoll')
        
        self.RElbowYaw = self.getDevice('RElbowYaw')
        self.LElbowYaw = self.getDevice('LElbowYaw')
        
        self.light_it_up_contest()


    def light_it_up_contest(self):
        self.getDevice('ChestBoard/Led').set(-1)
        self.getDevice('Face/Led/Left').set(-1)
        self.getDevice('Face/Led/Right').set(-1)
        self.getDevice('Ears/Led/Right').set(-1)
        self.getDevice('Ears/Led/Right').set(-1)
        self.getDevice('LFoot/Led').set(-1)
        self.getDevice('RFoot/Led').set(-1)


    def position_arms(self):
        self.RShoulderPitch.setPosition(0.6)
        self.LShoulderPitch.setPosition(0.6)
        
        self.RElbowYaw.setPosition(1.5)
        self.LElbowYaw.setPosition(-1.5)
        
        self.RElbowRoll.setPosition(0.8)
        self.LElbowRoll.setPosition(-0.8)
        
        
    def attack(self):
        t = self.getTime()

        if (t - self.last_time) > 0.8:
            self.RShoulderPitch.setPosition(0)
            self.LShoulderPitch.setPosition(0)
            
            self.RElbowRoll.setPosition(-0.5)
            self.LElbowRoll.setPosition(0.5)
        else:
            self.position_arms()
            
        if (t - self.last_time) > 1:
            self.last_time = t
        
        
        
    def dive(self):
        self.LHipPitch.setPosition(-1)
        self.RHipPitch.setPosition(-1)

        self.LKneePitch.setPosition(0.8)
        self.RKneePitch.setPosition(0.8)

        self.LAnklePitch.setPosition(-0.5)
        self.RAnklePitch.setPosition(-0.5)

        self.RShoulderRoll.setPosition(-0.6)
        self.LShoulderRoll.setPosition(0.6)

        self.step(200)


        
    def run(self):
        #self.dive()
        self.position_arms()
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            t = self.getTime()
            self.gait_manager.update_theta()

            if 0.3 < t < 1.5:
                #pass
                self.start_sequence()
            elif t > 1.5:
                self.fall_detector.check()
                self.position_arms()
                self.attack()

                edge = self.detect_line()
                if edge:
                    self.gait_manager.command_to_motors(desired_radius=0.1, heading_angle=0)
                else:
                    self.walk()


    def detect_line(self):
        # Load the image
        img = self.cameraBottom.get_image()
        top_img = self.camera.get_image()

        #if random.randint(0, 10) == 1:
            #cv2.imwrite(f"topImages3/img_{random.randint(0, 10_000)}.png", top_img)

        filtered_img = filter_lines(img)

        coords = self.floor_model.segment_floor(filtered_img)

        heights = [filtered_img.shape[0]-a[1] for a, b in coords]
        heights = heights[3:-3]
        heights = [x if x > 0 else 100 for x in heights]
        
        if min(heights) < 60:
            return True
        else:
            return False



    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        self.gait_manager.command_to_motors(heading_angle=0)


    def detect_sonar(self):
        """Return a tuple of the recorded distances from each sonar sensor. (LeftSonar, RightSonar)"""
        return (self.sonarL.getValue(), self.sonarR.getValue())


    def update_opponent_x(self, img):
         x_pos = self._get_normalized_opponent_x(img)  # -0.1 and 0.1 is basically facing the opponent
         self.opponent_x.append(x_pos)
         self.opponent_x.pop(0)

         return sum(self.opponent_x) / 10


    def calculate_variance(self, arr):
        n = len(arr)
        mean = sum(arr) / n
        variance = sum((x - mean) ** 2 for x in arr) / n

        return variance


    def hallucinating(self):
        # do we think we see the opponent when we actually dont?
        # issue with the given _get_normalised_opponent_x function
        var = self.calculate_variance(self.opponent_x)
        return var > 0.25


    def walk(self): 
        img = self.camera.get_image()

        x_pos = self.update_opponent_x(img)
        
        #opp_detected = self.obj_model.detect_opponent(img)
        #print(f'{opp_detected=}')

        self.gait_manager.command_to_motors(desired_radius=0, heading_angle=0)
        return 0

        if (-0.4 < x_pos < 0.4) or self.hallucinating(): # forward
            self.gait_manager.command_to_motors(desired_radius=0, heading_angle=0)
        elif x_pos > 0: # right
            self.gait_manager.command_to_motors(desired_radius=0.1, heading_angle=1)
        else: # left
            self.gait_manager.command_to_motors(desired_radius=-0.1, heading_angle=1)


    def _get_normalized_opponent_x(self, img):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        #img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)

        if horizontal_coordinate is None:
            return 0

        return horizontal_coordinate * 2 / img.shape[1] - 1


# create the Robot instance and run main loop
wrestler = HexBot()
wrestler.run()
