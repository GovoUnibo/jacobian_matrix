from sympy import Matrix, cos, sin
from scipy.spatial.transform import Rotation
import numpy as np

def Rx(gamma):
    return (Matrix([[1, 0 ,          0 ,            0],
                    [0, cos(gamma), -sin(gamma),    0],
                    [0, sin(gamma), cos(gamma),     0],
                    [0, 0,          0,              1] 
                ])
            )

def Ry(beta):
    return (Matrix([[cos(beta),  0 , sin(beta) ,   0],
                    [0,           1,  0,             0],
                    [-sin(beta), 0,  cos(beta),    0],
                    [0,           0,  0,             1] 
                ])
            )

def Rz(alpha):
    return (Matrix([[cos(alpha), -sin(alpha), 0,   0],
                    [sin(alpha), cos(alpha),  0,   0],
                    [0,          0,           1,   0],
                    [0,          0,           0,   1] 
                ])
            )

def RotationMatrix_ZYZ_Convention(phi = None, theta = None, psi = None):
    '''
        Input: function takes in input radiants!!!!
    '''
    try:
        return Rz(psi) * Ry(theta) * Rz(phi)
    except: 
        print("This function has three arguments Required")
    return


def RotationMatrix_ZYX_Convention(phi = None, theta = None, psi = None):
    '''
        Input: function takes in input radiants!!!!
    '''
    try:
        return Rz(psi) * Ry(theta) * Rx(phi)
    except: 
        print("This function has three arguments Required")
    return

def Homogeneus_Matrix_Euler(x, y, z, psi, theta, phi):
    
    return Matrix([
                    [cos(phi)*cos(theta), 
                    cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), 
                    cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), 
                    x],


                    [sin(phi)*cos(theta), 
                    sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),
                    sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), 
                    y],

                    [-sin(theta),
                    cos(theta)*sin(psi),
                    cos(theta)*cos(psi),
                    z],

                    [0,
                    0,
                    0,
                    1] 
                ])

def Homogeneus_Matrix_Quaternion(x, y, z, qw, qx, qy, qz):
        
        return Matrix([
                        [1 - 2*qy**2 - 2*qz**2, 
                        2*qx*qy - 2*qz*qw, 
                        2*qx*qz + 2*qy*qw, 
                        x],
    
    
                        [2*qx*qy + 2*qz*qw, 
                        1 - 2*qx**2 - 2*qz**2,
                        2*qy*qz - 2*qx*qw, 
                        y],
    
                        [2*qx*qz - 2*qy*qw,
                        2*qy*qz + 2*qx*qw,
                        1 - 2*qx**2 - 2*qy**2,
                        z],
    
                        [0,
                        0,
                        0,
                        1] 
                    ])
        
        
def homogeneous_to_pose_euler(matrix):
    # Estrai la parte di traslazione dalla matrice omogenea
    translation = matrix[:3, 3]

    # Estrai la parte di rotazione dalla matrice omogenea
    rotation_matrix = matrix[:3, :3]

    # Converte la matrice di rotazione in quaternion utilizzando scipy
    quaternion = Rotation.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)

    return translation, quaternion


def homogeneous_to_pose_quat(matrix):
    # Estrai la parte di traslazione dalla matrice omogenea
    translation = matrix[:3, 3]
    translation = [translation[0], translation[1], translation[2]]

    # Estrai la parte di rotazione dalla matrice omogenea
    rotation_matrix = matrix[:3, :3]

    # Converte la matrice di rotazione in quaternion utilizzando scipy
    quaternion = Rotation.from_matrix(rotation_matrix).as_quat() # qx qy qz qw
    quaternion = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]] #qw qx qy qz
    return translation, quaternion
    
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x**2 + y**2)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y**2 + z**2)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def euler_to_quaternion(roll_x, pitch_y, yaw_z):
    qx = np.sin(roll_x/2) * np.cos(pitch_y/2) * np.cos(yaw_z/2) - np.cos(roll_x/2) * np.sin(pitch_y/2) * np.sin(yaw_z/2)
    qy = np.cos(roll_x/2) * np.sin(pitch_y/2) * np.cos(yaw_z/2) + np.sin(roll_x/2) * np.cos(pitch_y/2) * np.sin(yaw_z/2)
    qz = np.cos(roll_x/2) * np.cos(pitch_y/2) * np.sin(yaw_z/2) - np.sin(roll_x/2) * np.sin(pitch_y/2) * np.cos(yaw_z/2)
    qw = np.cos(roll_x/2) * np.cos(pitch_y/2) * np.cos(yaw_z/2) + np.sin(roll_x/2) * np.sin(pitch_y/2) * np.sin(yaw_z/2)
    return qx, qy, qz, qw

