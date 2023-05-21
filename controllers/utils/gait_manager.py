# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from .ellipsoid_gait_generator import EllipsoidGaitGenerator
from .kinematics import Kinematics
#import csv
import math
import numpy as np


class DataTracker:
    def __init__(self, n):
        self.n = n
        self.values = []

    def update(self, value):
        self.values.append(value)
        if len(self.values) > self.n:
            self.values = self.values[-self.n:]

    def average(self):
        return sum(self.values) / len(self.values) if len(self.values) > 0 else 0

    def variance(self):
        if len(self.values) < 2:
            return 0

        average = self.average()
        squared_diffs = [(x - average) ** 2 for x in self.values]
        variance = sum(squared_diffs) / (len(self.values) - 1)
        return variance * 1000

    def correlation(self):
        if len(self.values) < 2:
            return 0

        average = self.average()
        total_diff_prod = 0
        for i in range(len(self.values) - 1):
            diff1 = (self.values[i] - average) * 10000
            diff2 = (self.values[i + 1] - average) * 10000
            total_diff_prod += diff1 * diff2

        #correlation = total_diff_prod / ((len(self.values) - 1) * math.sqrt(self.variance()))
        correlation = np.corrcoef(self.values, range(len(self.values)))[0, 1]
        if correlation > 0:
            return 1
        elif correlation < 0:
            return -1
        else:
            return 0


class GaitManager():
    """Connects the Kinematics class and the EllipsoidGaitGenerator class together to have a simple gait interface."""

    def __init__(self, robot, time_step):
        self.time_step = time_step
        self.gait_generator = EllipsoidGaitGenerator(robot, self.time_step)
        self.kinematics = Kinematics()
        joints = ['HipYawPitch', 'HipRoll', 'HipPitch', 'KneePitch', 'AnklePitch', 'AnkleRoll']
        self.L_leg_motors = []
        for joint in joints:
            motor = robot.getDevice(f'L{joint}')
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(time_step)
            self.L_leg_motors.append(motor)

        self.R_leg_motors = []
        for joint in joints:
            motor = robot.getDevice(f'R{joint}')
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(time_step)
            self.R_leg_motors.append(motor)

        #self.file = open('right_z.csv', mode='a', newline='')
        #self.writer = csv.writer(self.file)
        #self.file1 = open('left_z.csv', mode='a', newline='')
        #self.writer1 = csv.writer(self.file1)

        self.rz_tracker = DataTracker(10)
        self.lz_tracker = DataTracker(10)
        self.corr_tracker = DataTracker(3)

    def update_theta(self):
        self.gait_generator.update_theta()

    def command_to_motors(self, desired_radius=None, heading_angle=0):
        """
        Compute the desired positions of the robot's legs for a desired radius (R > 0 is a right turn)
        and a desired heading angle (in radians. 0 is straight on, > 0 is turning left).
        Send the commands to the motors.
        """

        # Move right leg
        if not desired_radius: desired_radius = 1e3

        self.corr_tracker.update(int(self.rz_tracker.correlation() == self.lz_tracker.correlation()))

        x, y, z, yaw = self.gait_generator.compute_leg_position(
            is_left=False, desired_radius=desired_radius, heading_angle=heading_angle)

        z = min(z, -0.28)
        z = max(z, -0.33)

        #if sum(self.corr_tracker.values) == 3:
            #x, y, z, yaw = (0.014311165485430028, -0.0600001101197227, -0.29294549058807747, -1.5389342845162082e-05)

        self.rz_tracker.update(z)

        right_target_commands = self.kinematics.inverse_leg(x * 1e3, y * 1e3, z * 1e3, 0, 0, yaw, is_left=False)

        for command, motor in zip(right_target_commands, self.R_leg_motors):
            motor.setPosition(command)

        #self.writer.writerow([z])

        # Move left leg
        x, y, z, yaw = self.gait_generator.compute_leg_position(
            is_left=True, desired_radius=desired_radius, heading_angle=heading_angle)
        #print(f'[{x, y, z, yaw}]')
        z = min(z, -0.28)
        z = max(z, -0.33)

        if sum(self.corr_tracker.values) == 3:
            #x, y, z, yaw = (-0.013116045384055327, 0.0599999075163715, -0.3198323535116744, 1.4102364776525524e-05)
            #print("Oscillations detected!")
            if self.rz_tracker.correlation() == 1:
                z = z - 0.02
            else:
                z = z + 0.02
            x = x * -1
            

        self.lz_tracker.update(z)

        left_target_commands = self.kinematics.inverse_leg(x * 1e3, y * 1e3, z * 1e3, 0, 0, yaw, is_left=True)

        for command, motor in zip(left_target_commands, self.L_leg_motors):
            motor.setPosition(command)

        #self.writer1.writerow([z])

        #print(f'Values: R:{self.rz_tracker.values[-1]:.5f} L:{self.lz_tracker.values[-1]:.5f}')
        #print(f'Average: R:{self.rz_tracker.average():.5f} L:{self.lz_tracker.average():.5f}')
        #print(f'Variance: R:{self.rz_tracker.variance():.5f} L:{self.lz_tracker.variance():.5f}')
        #print(f'Correlation: R:{self.rz_tracker.correlation():.5f} L:{self.lz_tracker.correlation():.5f}\n\n')
