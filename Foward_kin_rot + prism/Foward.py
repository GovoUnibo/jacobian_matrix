from Foward_kin.rigidBodyTf import *
from sympy import *
from sympy.physics.vector import init_vprinting
from sympy.physics.mechanics import dynamicsymbols
from sympy.matrices import eye

class FowardKinematics():
    
    def __init__(self,  num_of_joints):
        
        self.num_joints = num_of_joints

        i=0

        self._theta = [dynamicsymbols('theta%i' % ii) for ii in range(self.num_joints)]
        '''SETUP MATRICI'''
        
        self._joints__ = [None] * self.num_joints
        self._transforms_ = [None] * self.num_joints
        self.T_0b = eye(4)
        

    def set_World_Bl_pose(self, pos=[], rot=[]):
        '''setta la posizione del sistema di riferimento del mondo rispetto al sistema di riferimento del robot'''
        if len(pos) != 3 and (len(rot) != 3 or len(rot) != 4):
            raise ValueError("The number of parameters must be 3 for pos and e or 4 rot")
        if len(rot) == 3:
            self.T_0b = Homogeneus_Matrix_Euler(pos[0], pos[1], pos[2], rot[0], rot[1], rot[2])
        elif len(rot) == 4:
            self.T_0b = Homogeneus_Matrix_Quaternion(pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3])
    
    def _add_transform(self, pose =[], rot=[], joint_number=0):
        if len(rot)==3:
            self._transforms_[joint_number]= Homogeneus_Matrix_Euler(pose[0], pose[1], pose[2], rot[0], rot[1], rot[2])
        elif len(rot)==4:
            self._transforms_[joint_number]= Homogeneus_Matrix_Quaternion(pose[0], pose[1], pose[2], rot[0], rot[1], rot[2], rot[3])
        elif (len(rot)!=3 or len(rot)!=4) and len(pose)!=3:
            raise ValueError("The number of rot parameters must be 3 for pose or 4 for quaternion, 3 for axis_tr")

    def add_prismatic_joint(self, pos =[], rot=[], axis_tr=[], joint_number=0):
        if len(axis_tr) != 3 and joint_number > self.num_joints:
            raise ValueError("The number of parameters must be 3 for axis_tr and joint_number must be less than num_joints")
        self._add_transform(pos, rot, joint_number)
        axis_translation = [axis_tr[k]*self._theta[joint_number] for k in range(3)]
        self._joints__[joint_number] = Homogeneus_Matrix_Euler(axis_translation[0], axis_translation[1], axis_translation[2], 0, 0, 0)
        
    def add_rot_joint(self, pos =[], rot=[], axis_rot=[], joint_number=0):
        if len(axis_rot) != 3 and joint_number > self.num_joints:
            raise ValueError("The number of parameters must be 3 for axis_tr and joint_number must be less than num_joints")
        self._add_transform(pos, rot, joint_number)
        #se sta ruotando attorno z allora la rotazione è [0 0 1] --> [0 0 1]*theta --> [0 0 theta]
        axis_rotation = [axis_rot[k]*self._theta[joint_number] for k in range(3)] 
        self._joints__[joint_number] = Homogeneus_Matrix_Euler(0, 0, 0, axis_rotation[0], axis_rotation[1], axis_rotation[2])
        
        
    def getDirectKin_ThetaParam(self, last_joint_index = 0):
        ''' calcola la cinematica diretta tenendo il grado motore come parametro'''
        if last_joint_index > self.num_joints:
            last_joint_index = self.num_joints
        T_bl_to_ee = eye(4)
        for i in range(last_joint_index):
            T_bl_to_ee =  T_bl_to_ee*self._transforms_[i]*self._joints__[i]

        return self.T_0b * T_bl_to_ee
    
    def getDirectKin_numeric(self, joint_array=[], as_matrix=False, as_pos_euler=False, as_pos_quaternion=False):
        if not isinstance(joint_array, list) or len(joint_array) > self.num_joints:
            raise ValueError("The joint_array must be a list and the length must be less than num_joints")
        # print(joint_array)
        last_joint_index = len(joint_array)
        T_F_param = self.getDirectKin_ThetaParam(last_joint_index)

        for i in range(last_joint_index):
            T_F_param = T_F_param.subs({self._theta[i] : joint_array[i]})
        if as_matrix:
            return T_F_param
        elif as_pos_euler:
            return homogeneous_to_pose_euler(T_F_param)
        elif as_pos_quaternion:
            return homogeneous_to_pose_quat(T_F_param)
    
    def get_World_to_BL(self, as_matrix=False, as_pos_euler=False, as_pos_quaternion=False):
        if as_matrix:
            return self.T_0b
        elif as_pos_euler:
            return homogeneous_to_pose_euler(self.T_0b)
        elif as_pos_quaternion:
            return homogeneous_to_pose_quat(self.T_0b)
        
if __name__ == '__main__':

    axis_rotation_j1 = [0, 0, 1] #ruota positivo attorno all'asse y quando il verso di rotazione è positivo
    axis_rotation_j2 = [0, 0, 1] #ruota negativo attorno all'asse z quando il verso di rotazione è positivo
    axis_rotation_j3 = [0, 0, 1]
    axis_rotation_j4 = [0, 0, 1]
    axis_rotation_j5 = [0, 0, 1]
    axis_rotation_j6 = [0, 0, 1]

    w_b         = [0,   0, 0, 0, 0, 0]
    bl_to_j1    = [ 0.000, 0.000, 3.142]
    j1_to_j2    = [0, 0, 0, 1.571, -0.000, 0.000]
    j2_to_j3    = [-0.425, 0.000, 0.000, 0, 0, 0]
    j3_to_j4    = [-0.392, 0.000, 0.133, 0, 0, 0]
    j4_to_j5    = [0.000, -0.100, -0.000, 1.571, -0.000, 0.000]
    j5_to_j6    = [0.000, 0.100, -0.000, -1.571, 0.000, -0.000]

    FK = FowardKinematics(6)
    FK.set_World_Bl_pose([0, 0, 0], [0, 0, 0, 1])
    FK.add_prismatic_joint([0.000, 0.000, 0.163,], [0, 0, 0], [0, 0, 1], 0)
    FK.add_rot_joint([0.000, 0.000, 0.000,], [0, 0, 0], [0, 0, 1], 1)
    FK.add_rot_joint([-0.425, 0.000, 0.000], [0, 0, 0], [0, 0, 1], 2)


    # joint_array = [0.994, -2.199, 0.8726, 0, 1.117, -2.618]
    for i in range(3):
        joint_array = [0+i, 0, 0] 
        T_06 = FK.getDirectKin_numeric(joint_array)
        print("Direct:")
        print (T_06)
  