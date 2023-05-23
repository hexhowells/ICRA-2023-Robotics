from controller import Robot
import sys
sys.path.append('..')

from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
from utils.camera import Camera
from utils.running_average import RunningAverage

import cv2
import random
import time

from floor import Floor
from imutils import filter_lines


class HexBot (Robot):
    def __init__(self):
        Robot.__init__(self)
        # get time step
        self.time_step = int(self.getBasicTimeStep())

        # initialise objects
        self.camera = Camera(self)
        self.cameraBottom = Camera(self, camera_name='CameraBottom')
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        
        # used for detecting the edge of the ring
        self.floor_model = Floor(threshold=10, img_step=5, img_size=(160, 120))

        # enable distance sensors
        self.sonarL = self.getDevice('Sonar/Left')
        self.sonarL.enable(1)
        self.sonarR = self.getDevice('Sonar/Right')
        self.sonarR.enable(1)

        # set head position
        self.head = self.getDevice('HeadPitch')
        self.head.setPosition(0.25)
        
        # set variables
        self.last_time = 0
        self.start_time = 2
        self.edge_dist = 100
        self.direction = random.choice([1, -1])
        self.attack_freq = 2
        self.attack_len = 0.2
        self.opponent_x_history = RunningAverage(dimensions=1, history_steps=5)

        # get all joint devices required
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

        self.RWristYaw = self.getDevice('RWristYaw')
        self.LWristYaw = self.getDevice('LWristYaw')
        
        self.light_it_up_contest()


    def light_it_up_contest(self):
        """Activate LEDs on the robot"""
        self.getDevice('ChestBoard/Led').set(0x8007D4)
        self.getDevice('Face/Led/Left').set(0x8007D4)
        self.getDevice('Face/Led/Right').set(0x8007D4)
        self.getDevice('Ears/Led/Left').set(0x8007D4)
        self.getDevice('Ears/Led/Right').set(0x8007D4)
        self.getDevice('LFoot/Led').set(0x8007D4)
        self.getDevice('RFoot/Led').set(0x8007D4)


    def position_arms(self):
        """Position the arms to their default value"""
        self.RShoulderPitch.setPosition(0.9)
        self.LShoulderPitch.setPosition(0.9)

        self.RShoulderRoll.setPosition(0)
        self.LShoulderRoll.setPosition(0)
        
        self.RElbowYaw.setPosition(1.5)
        self.LElbowYaw.setPosition(-1.5)
        
        self.RElbowRoll.setPosition(0)
        self.LElbowRoll.setPosition(0)

        self.RWristYaw.setPosition(-1.5)
        self.LWristYaw.setPosition(1.5)

        
    def attack(self):
        """Swing arms to attempt to attack the opponent"""
        t = self.getTime()

        # perform attack
        if (t - self.last_time) > self.attack_freq:
            self.RShoulderPitch.setPosition(0.1)
            self.LShoulderPitch.setPosition(0.1)

            self.RShoulderRoll.setPosition(0.2)
            self.LShoulderRoll.setPosition(-0.2)

            self.RElbowRoll.setPosition(0.4)
            self.LElbowRoll.setPosition(-0.4)
            
            self.step(10)
        else:
            self.position_arms()
            
        # reset timer
        if (t - self.last_time) > self.attack_freq + self.attack_len:
            self.last_time = t
        
        
        
    def flip(self):
        """Dive forward and flip over"""
        self.RShoulderPitch.setPosition(0.6)
        self.LShoulderPitch.setPosition(0.6)

        self.step(200)

        self.LAnklePitch.setPosition(-0.5)
        self.RAnklePitch.setPosition(-0.5)

        self.step(300)

        self.LHipPitch.setPosition(-1)
        self.RHipPitch.setPosition(-1)

        self.LKneePitch.setPosition(0)
        self.RKneePitch.setPosition(0)

        self.step(500)

        self.RShoulderPitch.setPosition(-2)
        self.LShoulderPitch.setPosition(-2)

        self.step(500)


        
    def run(self):
        """Start the controller"""
        self.position_arms()

        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            t = self.getTime()
            self.gait_manager.update_theta()

            # move closer to the edge after 1 minute
            if t > 60:
                self.edge_dist = 50

            # move even closer to the edge after 2 minutes
            if t > 120:
                self.edge_dist = 40

            # start moving towards the edge of the ring for the final flip
            if 175 > t > 173:
                self.gait_manager.command_to_motors(desired_radius=self.direction*0.1, heading_angle=0)
                continue
            if t > 175:  # Last ditch effort to gain some movement points
                self.flip()

            # turn at the start of the round
            if t < self.start_time:
                self.start_sequence()
            elif t > self.start_time:  # this code runs most of the time
                fallen = self.fall_detector.check()

                if fallen:  # reset positions after a fall
                    self.position_arms()
                    self.step(300)

                self.attack()

                edge = self.detect_line()
                if edge:
                    self.gait_manager.command_to_motors(desired_radius=self.direction*0.1, heading_angle=self.direction*0.7)
                else:
                    #self.gait_manager.command_to_motors(desired_radius=0, heading_angle=0)
                    self.walk()


    def detect_line(self):
        """Detect the edge of the ring via its distinctive red border"""
        img = self.cameraBottom.get_image()
        top_img = self.camera.get_image()

        filtered_img = filter_lines(img)

        coords = self.floor_model.segment_floor(filtered_img)

        heights = [filtered_img.shape[0]-a[1] for a, b in coords]
        heights = heights[3:-3]
        heights = [x if x > 0 else 100 for x in heights]
        
        if min(heights) < self.edge_dist:
            return True
        else:
            return False


    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        #self.gait_manager.command_to_motors(desired_radius=-1, heading_angle=-1.4)
        self.gait_manager.command_to_motors(desired_radius=0, heading_angle=self.direction*0.7)
        #self.gait_manager.command_to_motors(heading_angle=0)


    def detect_sonar(self):
        """Return a tuple of the recorded distances from each sonar sensor. (LeftSonar, RightSonar)"""
        return (self.sonarL.getValue(), self.sonarR.getValue())


    def update_opponent_x(self, img):
        """Get the moving average of the opponents x location"""
        x_pos = self._get_normalized_opponent_x(img)  # -0.1 and 0.1 is basically facing the opponent
        x_pos = self.opponent_x_history.get_new_average(x_pos)

        return x_pos


    def calculate_variance(self, arr):
        """Calculate the variance of an array"""
        n = len(arr)
        mean = sum(arr) / n
        variance = sum((x - mean) ** 2 for x in arr) / n

        return variance


    def hallucinating(self):
        # do we think we see the opponent when we actually dont?
        # issue with the given _get_normalised_opponent_x function
        var = self.calculate_variance(self.opponent_x_history.history)
        print(var)
        return var > 0.30


    def walk(self): 
        """Main rountine for tracking the opponent and updating the bots direction accordingly"""
        img = self.camera.get_image()

        x_pos = self.update_opponent_x(img)

        if (-0.18 < x_pos < 0.18) or self.hallucinating(): # forward
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
