from sympy import Matrix, cos, sin
from scipy.spatial.transform import Rotation

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

def Homogeneus_Matrix(x, y, z, psi, theta, phi):
    
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


def homogeneous_to_translation_quaternion(matrix):
    # Estrai la parte di traslazione dalla matrice omogenea
    translation = matrix[:3, 3]

    # Estrai la parte di rotazione dalla matrice omogenea
    rotation_matrix = matrix[:3, :3]

    # Converte la matrice di rotazione in quaternion utilizzando scipy
    quaternion = Rotation.from_matrix(rotation_matrix).as_quat()

    return translation, quaternion
