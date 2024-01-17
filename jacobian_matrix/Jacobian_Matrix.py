from requests import get
from sympy import Matrix, simplify
from Forward_Kinematics import FowardKinematics



class Jacobian(FowardKinematics):
    def __init__(self, num_of_joints):
        super().__init__(num_of_joints = num_of_joints)
        self.num_of_joints = num_of_joints
    def get_angular_velocity(self, joint_index):
        axis_of_joint = Matrix([self.list_of_axis_rot[joint_index][0], self.list_of_axis_rot[joint_index][1], self.list_of_axis_rot[joint_index][2]])
        axis_of_joint = axis_of_joint.subs({self._theta[joint_index]: 1})
        FK_to_Joint = self.getDirectKin_ThetaParam(joint_index+1) #brutto il +1 ma va bene così perchè c'è un for in rage nella funzione
        return FK_to_Joint[0:3, 0:3] * axis_of_joint #estraggo la colonna della matrice di rotazione del giunto n relativa all'asse di rotazione
        
    def getJacobianParametric(self):
        T_06_param = self.getDirectKin_ThetaParam(self.num_of_joints)

        p_x_y_z = T_06_param.col(-1) #prende ultima colonna

        p_x = p_x_y_z.row(0)
        p_y = p_x_y_z.row(1)
        p_z = p_x_y_z.row(2)

        J_v_raw1 = []
        J_v_raw2 = []
        J_v_raw3 = [] 

        for i in range(self.num_joints):
            J_v_raw1.append(p_x.diff(self._theta[i]))
            J_v_raw2.append(p_y.diff(self._theta[i]))
            J_v_raw3.append(p_z.diff(self._theta[i]))

        #prendo la colonna dell'asse di rotazione z_ii
        Jw_col1 = self.get_angular_velocity(0)
        Jw_col2 = self.get_angular_velocity(1)
        Jw_col3 = self.get_angular_velocity(2)
        Jw_col4 = self.get_angular_velocity(3)
        Jw_col5 = self.get_angular_velocity(4)
        Jw_col6 = self.get_angular_velocity(5)


        Jacobian =  Matrix([
                        [J_v_raw1[0],   J_v_raw1[1],    J_v_raw1[2],    J_v_raw1[3],    J_v_raw1[4],    J_v_raw1[5]], 
                        [J_v_raw2[0],   J_v_raw2[1],    J_v_raw2[2],    J_v_raw2[3],    J_v_raw2[4],    J_v_raw2[5]],
                        [J_v_raw3[0],   J_v_raw3[1],    J_v_raw3[2],    J_v_raw3[3],    J_v_raw3[4],    J_v_raw3[5]],
                        [Jw_col1[0],    Jw_col2[0],     Jw_col3[0],     Jw_col4[0],     Jw_col5[0],     Jw_col6[0]],
                        [Jw_col1[1],    Jw_col2[1],     Jw_col3[1],     Jw_col4[1],     Jw_col5[1],     Jw_col6[1]],
                        [Jw_col1[2],    Jw_col2[2],     Jw_col3[2],     Jw_col4[2],     Jw_col5[2],     Jw_col6[2]]
                    
                        ])
        
        # return Jacobian
        Jacobian = simplify(Jacobian)
        
        return Jacobian.evalf(4)

    def getJacobianNumeric(self, joint_values = []):
        
        

        Jacobian = self.getJacobianParametric()

        for i in range(self.num_joints):
            for j in range(self.num_joints):
                for k in range(self.num_joints):
                    Jacobian[i,j] = Jacobian[i,j].subs({self._theta[k]: joint_values[k]})
        return Jacobian

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

    Jc = Jacobian(6)
    Jc.set_World_Bl_pose(0, 0, 0, 0, 0, 0)  
    Jc.add_robot_link(bl_to_j1, axis_rotation_j1, 0)
    Jc.add_robot_link(j1_to_j2, axis_rotation_j2, 1)
    Jc.add_robot_link(j2_to_j3, axis_rotation_j3, 2)
    Jc.add_robot_link(j3_to_j4, axis_rotation_j4, 3)
    Jc.add_robot_link(j4_to_j5, axis_rotation_j5, 4)
    Jc.add_robot_link(j5_to_j6, axis_rotation_j6, 5)



    # print("Direct: \n",Jc.getDirectKin_numeric(-1.564, -0.762, -1.764, 3.91, 1.567, 0.0))
    # print("Jacobian \n",Jc.getJacobianNumeric(0.994, -2.199, 0.8726, 0, 1.117, -2.618))
    # print("Direct: \n",Jc.getDirectKin_numeric(0,0,0,0,0,0))
    # print("Jacobian \n",Jc.getJacobianNumeric([-2.548871977575965, -2.0248609214314786, -1.574481982947777, -0.29908405377582326, 5.536207367575946, 0.6055001576352761]))
    print(Jc.getJacobianParametric())

