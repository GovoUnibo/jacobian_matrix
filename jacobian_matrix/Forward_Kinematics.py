from rigidBodyTF import *
from sympy import *
from sympy.physics.vector import init_vprinting
from sympy.physics.mechanics import dynamicsymbols
from sympy import lambdify
import numpy as np


class FowardKinematics():
    
    def __init__(self,  num_of_joints):
        
        self.num_joints = num_of_joints

        i=0

        self._x      = [symbols('x%i' % ii)     for ii in range(self.num_joints)]
        self._y      = [symbols('y%i' % ii)     for ii in range(self.num_joints)]
        self._z      = [symbols('z%i' % ii)     for ii in range(self.num_joints)]
        self._roll   = [symbols('roll%i' % ii)  for ii in range(self.num_joints)]
        self._pitch  = [symbols('pitch%i' % ii) for ii in range(self.num_joints)]
        self._yaw    = [symbols('yaw%i' % ii)   for ii in range(self.num_joints)]

        self._alpha = [symbols('alpha%i' % ii)  for ii in range(self.num_joints)]
        self._beta  = [symbols('beta%i' % ii)   for ii in range(self.num_joints)]
        self._gamma = [symbols('gamma%i' % ii)  for ii in range(self.num_joints)]

        self._theta = [dynamicsymbols('theta%i' % ii) for ii in range(self.num_joints)]

        '''SETUP MATRICI PARAMETRICHE'''
        
        #calcolo Giunto Per Giunto Matrici Omogenee Forma Parametrica --> una volta derivate servono per calcolare la J_v forma parametrica
        self.parametric_HomTfMatrix  = []

        #for i in range(self.num_of_frames):
        #    self.parametric_HomTfMatrix.append(Homogeneus_Matrix(self._x[i], self._y[i], self._z[i], self._roll[i], self._pitch[i], self._yaw[i] ))
            #self.parametric_HomTf[0]= HomTF_01
        
        #calcolo la serie di matrici di rotaione che legano fame by frame --> SERVIRANNO PER LA J_w
        self.parametric_RotationMatrix = []
        for i in range(self.num_joints):
            self.parametric_RotationMatrix.append(RotationMatrix_ZYX_Convention(self._gamma[i], self._beta[i], self._alpha[i]))


    def setPos_World_Base(self, x, y, z, phi, theta, psi):
        self.T_0b = Homogeneus_Matrix(x, y, z, phi, theta, psi)

    def setPos_Base_J1(self, x, y, z, phi, theta, psi):
        self.T_b1 = Homogeneus_Matrix(x, y, z, phi, theta, psi)

    def setPos_J1_J2(self, x, y, z, phi, theta, psi):
        self.T_12 = Homogeneus_Matrix(x, y, z, phi, theta, psi)

    def setPos_J2_J3(self, x, y, z, phi, theta, psi):
        self.T_23 = Homogeneus_Matrix(x, y, z, phi, theta, psi)
    
    def setPos_J3_J4(self, x, y, z, phi, theta, psi):
        self.T_34 = Homogeneus_Matrix(x, y, z, phi, theta, psi)

    def setPos_J4_J5(self, x, y, z, phi, theta, psi):
        self.T_45 = Homogeneus_Matrix(x, y, z, phi, theta, psi)

    def setPos_J5_J6(self, x, y, z, phi, theta, psi):
        self.T_56 = Homogeneus_Matrix(x, y, z, phi, theta, psi)

    def setJ1AxisRotation(self, axis_rot = []):
        J1_index = 0
        self.axis_rotation_J1 = [axis_rot[k]*self._theta[J1_index] for k in range(3)]
        self.RotMatrix_funcOfTheta1 = self.parametric_RotationMatrix[J1_index].subs({self._gamma[J1_index]  : self.axis_rotation_J1[0],
                                                                                    self._beta[J1_index]    : self.axis_rotation_J1[1],
                                                                                    self._alpha[J1_index]   : self.axis_rotation_J1[2]
                                                                                    })

    def setJ2AxisRotation(self, axis_rot = []):
        J2_index = 1
        self.axis_rotation_J2 = [axis_rot[k]*self._theta[J2_index] for k in range(3)]
        self.RotMatrix_funcOfTheta2 = self.parametric_RotationMatrix[J2_index].subs({self._gamma[J2_index] : self.axis_rotation_J2[0],
                                                                                    self._beta[J2_index]   : self.axis_rotation_J2[1],
                                                                                    self._alpha[J2_index]  : self.axis_rotation_J2[2]
                                                                                    })
    
    def setJ3AxisRotation(self, axis_rot = []):
        J3_index = 2
        self.axis_rotation_J3 = [axis_rot[k]*self._theta[J3_index] for k in range(3)]
        self.RotMatrix_funcOfTheta3 = self.parametric_RotationMatrix[J3_index].subs({self._gamma[J3_index] : self.axis_rotation_J3[0],
                                                                                    self._beta[J3_index]   : self.axis_rotation_J3[1],
                                                                                    self._alpha[J3_index]  : self.axis_rotation_J3[2]
                                                                            })

    def setJ4AxisRotation(self, axis_rot = []):
        J4_index = 3
        self.axis_rotation_J4 = [axis_rot[k]*self._theta[J4_index] for k in range(3)]
        self.RotMatrix_funcOfTheta4 = self.parametric_RotationMatrix[J4_index].subs({self._gamma[J4_index] : self.axis_rotation_J4[0],
                                                                                    self._beta[J4_index]   : self.axis_rotation_J4[1],
                                                                                    self._alpha[J4_index]  : self.axis_rotation_J4[2]
                                                                                    })

    def setJ5AxisRotation(self, axis_rot = []):
        J5_index = 4
        self.axis_rotation_J5 = [axis_rot[k]*self._theta[J5_index] for k in range(3)]
        self.RotMatrix_funcOfTheta5 = self.parametric_RotationMatrix[J5_index].subs({self._gamma[J5_index] : self.axis_rotation_J5[0],
                                                                                    self._beta[J5_index]   : self.axis_rotation_J5[1],
                                                                                    self._alpha[J5_index]  : self.axis_rotation_J5[2]
                                                                                    })

    def setJ6AxisRotation(self, axis_rot = []):
        J6_index = 5
        self.axis_rotation_J6 = [axis_rot[k]*self._theta[J6_index] for k in range(3)]
        self.RotMatrix_funcOfTheta6 = self.parametric_RotationMatrix[J6_index].subs({self._gamma[J6_index] : self.axis_rotation_J6[0],
                                                                                    self._beta[J6_index]   : self.axis_rotation_J6[1],
                                                                                    self._alpha[J6_index]  : self.axis_rotation_J6[2]
                                                                                    })


    def getDirectKin_ThetaParam(self):
        ''' calcola la cinematica diretta tenendo il grado motore come parametro'''
        
        T_b1_theta = self.T_b1*self.RotMatrix_funcOfTheta1
        T_12_theta = self.T_12*self.RotMatrix_funcOfTheta2
        T_23_theta = self.T_23*self.RotMatrix_funcOfTheta3
        T_34_theta = self.T_34*self.RotMatrix_funcOfTheta4
        T_45_theta = self.T_45*self.RotMatrix_funcOfTheta5
        T_56_theta = self.T_56*self.RotMatrix_funcOfTheta6

        return self.T_0b * T_b1_theta * T_12_theta * T_23_theta * T_34_theta * T_45_theta * T_56_theta
    
    def getDirectKin_numeric(self, j1, j2, j3, j4, j5, j6):

        T_06_param = self.getDirectKin_ThetaParam()

        return T_06_param.subs({self._theta[0] : j1 ,
                                self._theta[1] : j2 ,
                                self._theta[2] : j3 ,
                                self._theta[3] : j4 ,
                                self._theta[4] : j5 ,
                                self._theta[5] : j6
                                })
        






if __name__ == '__main__':

    axis_rotation_j1 = [0, 1, 0] #ruota positivo attorno all'asse y quando il verso di rotazione è positivo
    axis_rotation_j2 = [0, 0, -1]#ruota negativo attorno all'asse z quando il verso di rotazione è positivo
    axis_rotation_j3 = [0, 0, -1]
    axis_rotation_j4 = [0, 0, -1]
    axis_rotation_j5 = [0, 1, 0]
    axis_rotation_j6 = [0, 0, 1]

    FK = FowardKinematics(6)

    FK.setPos_World_Base(0, 0, 0,       1.5708, 0, 0)
    FK.setPos_Base_J1(0, 0.1452, 0,     0, 0, 0)
    FK.setPos_J1_J2(0, 0, 0.146,        0, 0, 0)
    FK.setPos_J2_J3(0, 0.429, -0.1297,  0, 0, 0)
    FK.setPos_J3_J4(0, 0.4115, 0.106,   0, 0, 0)
    FK.setPos_J4_J5(0, 0.106, 0,        0, 0, 0)
    FK.setPos_J5_J6(0, 0, 0.11315,      0, 0, 0)
    

    FK.setJ1AxisRotation(axis_rotation_j1)
    FK.setJ2AxisRotation(axis_rotation_j2)
    FK.setJ3AxisRotation(axis_rotation_j3)
    FK.setJ4AxisRotation(axis_rotation_j4)
    FK.setJ5AxisRotation(axis_rotation_j5)
    FK.setJ6AxisRotation(axis_rotation_j6)

    
    T_06 = FK.getDirectKin_numeric(0, -2.21, -1.5, -1.19, 1.1, 1.08)

    print (T_06)
