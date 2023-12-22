from rigidBodyTF import *
from sympy import *
from sympy.physics.vector import init_vprinting
from sympy.physics.mechanics import dynamicsymbols
from sympy.matrices import eye

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
        
        self.list_of_tf = []
        self.list_of_axis_rot = []
        self.list_of_parametric_RotationMatrix = []
        
        
        #calcolo la serie di matrici di rotaione che legano fame by frame --> SERVIRANNO PER LA J_w
        self.parametric_RotationMatrix = []
        for i in range(self.num_joints):
            self.parametric_RotationMatrix.append(RotationMatrix_ZYX_Convention(self._gamma[i], self._beta[i], self._alpha[i]))

    def set_World_Bl_pose(self, x, y, z, psi, theta, phi):
        self.T_0b = Homogeneus_Matrix(x, y, z, psi, theta, phi)
    
    def add_robot_link(self, pose =[], axis_rot=[], joint_number=0):
        J_index = joint_number
        self.list_of_tf.append(Homogeneus_Matrix(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
        self.list_of_axis_rot.append([axis_rot[k]*self._theta[J_index] for k in range(3)])
        self.list_of_parametric_RotationMatrix.append(self.parametric_RotationMatrix[J_index].subs({self._gamma[J_index]  : self.list_of_axis_rot[J_index][0],
                                                                                                    self._beta[J_index]   : self.list_of_axis_rot[J_index][1],
                                                                                                    self._alpha[J_index]  : self.list_of_axis_rot[J_index][2]
                                                                                    })
                                                     )
    def getDirectKin_ThetaParam(self, last_joint_index = 0):
        ''' calcola la cinematica diretta tenendo il grado motore come parametro'''
        if last_joint_index > self.num_joints:
            last_joint_index = self.num_joints
        T_bl_to_ee = eye(4)
        for i in range(last_joint_index):
            T_bl_to_ee =  T_bl_to_ee*self.list_of_tf[i]*self.list_of_parametric_RotationMatrix[i]

        return self.T_0b * T_bl_to_ee
    
    def getDirectKin_numeric(self, joint_array=[]):
        last_joint_index = len(joint_array)
        T_06_param = self.getDirectKin_ThetaParam(last_joint_index)

        for i in range(last_joint_index):
            T_06_param = T_06_param.subs({self._theta[i] : joint_array[i]})

        return T_06_param 

        # return T_06_param.subs({self._theta[0] : joint_array[0] ,
        #                         self._theta[1] : joint_array[1] ,
        #                         self._theta[2] : joint_array[2] ,
        #                         self._theta[3] : joint_array[3] ,
        #                         self._theta[4] : joint_array[4] ,
        #                         self._theta[5] : joint_array[5]
        #                         })


if __name__ == '__main__':

    axis_rotation_j1 = [0, 0, 1] #ruota positivo attorno all'asse y quando il verso di rotazione è positivo
    axis_rotation_j2 = [0, 0, 1]#ruota negativo attorno all'asse z quando il verso di rotazione è positivo
    axis_rotation_j3 = [0, 0, 1]
    axis_rotation_j4 = [0, 0, 1]
    axis_rotation_j5 = [0, 0, 1]
    axis_rotation_j6 = [0, 0, 1]

    w_b = [0, 0, 0, 0, 0, 0]
    bl_to_j1 = [0.000, 0.000, 0.163, 0.000, 0.000, 3.142]
    j1_to_j2 = [0, 0, 0, 1.571, -0.000, 0.000]
    j2_to_j3 = [-0.425, 0.000, 0.000, 0, 0, 0]
    j3_to_j4 = [-0.392, 0.000, 0.133, 0, 0, 0]
    j4_to_j5 = [0.000, -0.100, -0.000, 1.571, -0.000, 0.000]
    j5_to_j6 = [0.000, 0.100, -0.000, -1.571, 0.000, -0.000]

    FK = FowardKinematics(6)
    FK.set_World_Bl_pose(0, 0, 0, 0, 0, 0)  
    FK.add_robot_link(bl_to_j1, axis_rotation_j1, 0)
    FK.add_robot_link(j1_to_j2, axis_rotation_j2, 1)
    FK.add_robot_link(j2_to_j3, axis_rotation_j3, 2)
    FK.add_robot_link(j3_to_j4, axis_rotation_j4, 3)
    FK.add_robot_link(j4_to_j5, axis_rotation_j5, 4)
    FK.add_robot_link(j5_to_j6, axis_rotation_j6, 5)

    joint_array = [0.994, -2.199, 0.8726, 0, 1.117, -2.618]    
    T_06 = FK.getDirectKin_numeric(joint_array)

    print("Direct:")
    print (T_06)
    # print("Moveit")
    # print(Homogeneus_Matrix(-0.170, 0.064, 0.950, -0.453, -0.260, 0.069))