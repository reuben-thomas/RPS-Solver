import math
import sympy as sp

def calculate_internal_angles(AB, BC, CD, DA):
    """Calculates the internal angles of a quadrilateral given the length of the 4 sides
       There are points A, B, C, D with respective internal angles, 

    Args:
        AB (_type_): _description_
        BC (_type_): _description_
        CD (_type_): _description_
        DA (_type_): _description_

    Returns:
        _type_: _description_
    """
    angle_A = math.acos((BC**2 + DA**2 - AB**2) / (2 * BC * DA))
    angle_B = math.acos((CD**2 + AB**2 - BC**2) / (2 * CD * AB))
    angle_C = math.acos((DA**2 + BC**2 - CD**2) / (2 * DA * BC))
    angle_D = math.acos((AB**2 + CD**2 - DA**2) / (2 * AB * CD))
    
    return angle_A, angle_B, angle_C, angle_D

def rotate_vector(vector, theta, axis):
    # Normalize the axis vector
    axis = axis.normalized()

    # Compute the rotation matrix
    cos_theta = sp.cos(theta)
    sin_theta = sp.sin(theta)
    rotation_matrix = sp.Matrix([
        [cos_theta + axis[0]**2*(1 - cos_theta), axis[0]*axis[1]*(1 - cos_theta) - axis[2]*sin_theta, axis[0]*axis[2]*(1 - cos_theta) + axis[1]*sin_theta],
        [axis[1]*axis[0]*(1 - cos_theta) + axis[2]*sin_theta, cos_theta + axis[1]**2*(1 - cos_theta), axis[1]*axis[2]*(1 - cos_theta) - axis[0]*sin_theta],
        [axis[2]*axis[0]*(1 - cos_theta) - axis[1]*sin_theta, axis[2]*axis[1]*(1 - cos_theta) + axis[0]*sin_theta, cos_theta + axis[2]**2*(1 - cos_theta)]
    ])

    # Apply the rotation to the vector
    rotated_vector = rotation_matrix * vector

    return rotated_vector
