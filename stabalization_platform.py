import numpy as np
import math
import sympy as sp
from utils import calculate_internal_angles, rotate_vector

class StabalizationPlatform:

    def __init__(self, baseplate_radius: float, topplate_radius: float, 
                 actuator_min: float, actuator_max, actuator_neutral=0.0):
        """Constructor for StabalizationPlatform

        Args:
            baseplate_radius (float): Distance from the centre of the baseplate to actuator mounts
            topplate_radius (float): Distance from the centre of the topplate to actuator mounts
            actuator_min (float): Minimum length of actuator
            actuator_max (float): Maximum length of actuator
            actuator_neutral (float, optional): The middle length of the actuator. Defaults to average of minimum and maximum constraints.
            num_actuators (int, optional): _description_. Defaults to 3.
        """
        # Class properties
        self.baseplate_radius = baseplate_radius
        self.topplate_radius = topplate_radius
        self.actuator_min = actuator_min
        self.actuator_max = actuator_max
        self.actuator_neutral =actuator_neutral

        # Neutral position of actuator default
        if self.actuator_neutral == 0:
            self.actuator_neutral = (self.actuator_min + self.actuator_max) / 2.0

        # Top plate position and orientation vectors
        dr = abs(self.baseplate_radius - self.topplate_radius)
        h0 = math.sqrt(self.actuator_neutral**2 - dr**2)
        self.topplate_position_vector = sp.Matrix([0, 0, h0])
        self.toppplate_rpy = [0,0,0]

        # Actuator position vectors
        self.actuator_base_position_vectors = []
        self.actuator_top_position_vectors = []
        self.actuator_lengths = [actuator_neutral, actuator_neutral, actuator_neutral]
        self.actuator_vectors = []

        # Initialize base position vectors for each of 3 actuators
        r_base = self.baseplate_radius
        A1_base = sp.Matrix([r_base * sp.cos(0), r_base * sp.sin(0), 0])
        A2_base = sp.Matrix([r_base * sp.cos(2*sp.pi/3), r_base * sp.sin(2*sp.pi/3), 0])
        A3_base = sp.Matrix([r_base * sp.cos(4*sp.pi/3), r_base * sp.sin(4*sp.pi/3), 0])
        self.actuator_base_position_vectors = [A1_base, A2_base, A3_base]

        # Initialize all others
        L_init = self.actuator_neutral
        self.update_solve_pose(L_init, L_init, L_init)

    def update_solve_pose_on_change(self, l1, l2, l3):

        new_actuator_lengths = [l1, l2, l3]
        old_actuator_lengths = self.actuator_lengths

        # find the actuator that changes most, this becomes axis of rotation
        diffs = [abs(a - b) for a, b in zip(new_actuator_lengths, old_actuator_lengths)]
        max_diff = max(diffs)
        change_id = diffs.index(max_diff)

        # find the axis of rotation as cross product of actuator vector and base position vector
        v1 = self.actuator_base_position_vectors[change_id]
        v2 = self.actuator_vectors[change_id]
        axis_of_rotation = -(v1.cross(v2))

        # old value of theta
        h = self.topplate_position_vector[2]
        r_base = self.baseplate_radius
        r_top = self.topplate_radius
        L_old = self.actuator_lengths[change_id]
        angles = calculate_internal_angles(r_top, h, r_base, L_old)
        theta_old = 2*math.pi - angles[0]-angles[-1]

        # new value of theta
        L_new = new_actuator_lengths[change_id]
        angles = calculate_internal_angles(r_top, h, r_base, L_new)
        theta_new = 2*math.pi - angles[0]-angles[-1]

        # rotation parameters
        d_theta = theta_new-theta_old

        print(change_id)

        # Given that the height and centre of the top plate doenst change, 
        # The actuator top position vectors are found by rotating the vector
        # from the centre of the top plate to the actuator top mount
        for i in range(len(self.actuator_top_position_vectors)):
            top_to_mount_vector = self.actuator_top_position_vectors[i] - self.topplate_position_vector
            rotated_vector = rotate_vector(top_to_mount_vector, d_theta, axis_of_rotation)
            new_actuator_top_position_vector = self.topplate_position_vector + rotated_vector
            self.actuator_top_position_vectors[i] = new_actuator_top_position_vector

        # Actuator vector
        A1_top, A2_top, A3_top = self.actuator_top_position_vectors
        A1_base, A2_base, A3_base = self.actuator_base_position_vectors
        self.actuator_vectors = [A1_top-A1_base, A2_top-A2_base, A3_top-A3_base]
        self.actuator_lengths = [float(self.actuator_vectors[0].norm()), float(self.actuator_vectors[1].norm()), float(self.actuator_vectors[2].norm())]

        x, y, z = (A1_top + A2_top + A3_top) / 3
        self.topplate_position_vector = sp.Matrix([x, y, z])


    def update_solve_pose(self, L1, L2, L3):
        """Solves for the orientation of the output given the lengths of the input vectors

        Args:
            L1 (float): Length of actuator 1 [m]
            L2 (float): Length of actuator 2 [m]
            L3 (float): Length of actuator 3 [m]
        """
        self.actuator_lengths = [L1, L2, L3]
        r_base = self.baseplate_radius
        r_top = self.topplate_radius

        # Position vector of actuator mount on top
        A1_top = sp.Matrix([r_top * sp.cos(0), r_top * sp.sin(0), L1])
        A2_top = sp.Matrix([r_top * sp.cos(2*sp.pi/3), r_top * sp.sin(2*sp.pi/3), L2])
        A3_top = sp.Matrix([r_top * sp.cos(4*sp.pi/3), r_top * sp.sin(4*sp.pi/3), L3])
        self.actuator_top_position_vectors = [A1_top, A2_top, A3_top]

        # Actuator vector
        A1_base = self.actuator_base_position_vectors[0]
        A2_base = self.actuator_base_position_vectors[1]
        A3_base = self.actuator_base_position_vectors[2]
        self.actuator_vectors = [A1_top-A1_base, A2_top-A2_base, A3_top-A3_base]

        # Top plate position vector
        x, y, z = (A1_top + A2_top + A3_top) / 3
        self.topplate_position_vector = sp.Matrix([x, y, z])