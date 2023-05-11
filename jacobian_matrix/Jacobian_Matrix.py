from numpy import matrix
from sympy import Matrix
from sympy import evalf
from Forward_Kinematics import FowardKinematics
import sympy



class Jacobian(FowardKinematics):
    def __init__(self, num_of_joints):
        super().__init__(num_of_joints = num_of_joints)
    
    def __column1(self):
        #prendo la colonna della matrice corrispondente all'asse di rotazione di rotazione z_ii
        z_11 = Matrix([self.axis_rotation_J1[0], self.axis_rotation_J1[1], self.axis_rotation_J1[2]])
        z_11 = z_11.subs({self._theta[0]: 1})
        return self.T_0b[0:3, 0:3] * self.T_b1[0:3, 0:3] * z_11
         
    
    def __column2(self):
        z_22 = Matrix([self.axis_rotation_J2[0], self.axis_rotation_J2[1], self.axis_rotation_J2[2]])
        z_22 = z_22.subs({self._theta[1]: 1})
        return self.T_0b[0:3, 0:3] * self.T_b1[0:3, 0:3] * self.T_12[0:3, 0:3] * z_22

    def __column3(self):
        z_33 = Matrix([self.axis_rotation_J3[0], self.axis_rotation_J3[1], self.axis_rotation_J3[2]])
        z_33 = z_33.subs({self._theta[2]: 1})
        return self.T_0b[0:3, 0:3] * self.T_b1[0:3, 0:3] * self.T_12[0:3, 0:3] * self.T_23[0:3, 0:3] * z_33

    def __column4(self):
        z_44 = Matrix([self.axis_rotation_J4[0], self.axis_rotation_J4[1], self.axis_rotation_J4[2]])
        z_44 = z_44.subs({self._theta[3]: 1})
        return self.T_0b[0:3, 0:3] * self.T_b1[0:3, 0:3] * self.T_12[0:3, 0:3] * self.T_23[0:3, 0:3] * self.T_34[0:3, 0:3] * z_44

    def __column5(self):
        z_55 = Matrix([self.axis_rotation_J5[0], self.axis_rotation_J5[1], self.axis_rotation_J5[2]])
        z_55 = z_55.subs({self._theta[4]: 1})
        return self.T_0b[0:3, 0:3] * self.T_b1[0:3, 0:3] * self.T_12[0:3, 0:3] * self.T_23[0:3, 0:3] * self.T_34[0:3, 0:3] * self.T_45[0:3, 0:3] * z_55

    def __column6(self):
        z_66 = Matrix([self.axis_rotation_J6[0], self.axis_rotation_J6[1], self.axis_rotation_J6[2]])
        z_66 = z_66.subs({self._theta[5]: 1})
        return self.T_0b[0:3, 0:3] * self.T_b1[0:3, 0:3] * self.T_12[0:3, 0:3] * self.T_23[0:3, 0:3] * self.T_34[0:3, 0:3] * self.T_45[0:3, 0:3] * self.T_56[0:3, 0:3] * z_66

    def getJacobianParametric(self):

        T_06_param = self.getDirectKin_ThetaParam()
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
        
        Jw_col1 = self.__column1()
        Jw_col2 = self.__column2()
        Jw_col3 = self.__column3()
        Jw_col4 = self.__column4()
        Jw_col5 = self.__column5()
        Jw_col6 = self.__column6()


        Jacobian =  Matrix([
                        [J_v_raw1[0],   J_v_raw1[1],    J_v_raw1[2],    J_v_raw1[3],    J_v_raw1[4],    J_v_raw1[5]], 
                        [J_v_raw2[0],   J_v_raw2[1],    J_v_raw2[2],    J_v_raw2[3],    J_v_raw2[4],    J_v_raw2[5]],
                        [J_v_raw3[0],   J_v_raw3[1],    J_v_raw3[2],    J_v_raw3[3],    J_v_raw3[4],    J_v_raw3[5]],
                        [Jw_col1[0],    Jw_col2[0],     Jw_col3[0],     Jw_col4[0],     Jw_col5[0],     Jw_col6[0]],
                        [Jw_col1[1],    Jw_col2[1],     Jw_col3[1],     Jw_col4[1],     Jw_col5[1],     Jw_col6[1]],
                        [Jw_col1[2],    Jw_col2[2],     Jw_col3[2],     Jw_col4[2],     Jw_col5[2],     Jw_col6[2]]
                    
                        ])
        
        # return Jacobian
        Jacobian = sympy.simplify(Jacobian)
        
        return Jacobian.evalf(3)

    def getJacobianNumeric(self, j1, j2, j3, j4, j5, j6):
        joint_values = []
        joint_values.append(j1)
        joint_values.append(j2)
        joint_values.append(j3)
        joint_values.append(j4)
        joint_values.append(j5)
        joint_values.append(j6)

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

    Jc = Jacobian(6)



    
    # Jc.setPos_World_Base(-1, 0.4, 0, 0, 0, -1.57)
    Jc.setPos_World_Base(0, 0, 0, 0, 0, 0)
    Jc.setPos_Base_J1(0, 0, 0.1625,     0, 0, 3.14)
    Jc.setPos_J1_J2(0, 0, 0,        1.570796327, 0, 0)
    Jc.setPos_J2_J3(-0.425, 0, 0,  0, 0, 0)
    Jc.setPos_J3_J4(-0.3922, 0, 0.1333,   0, 0, 0)
    Jc.setPos_J4_J5(0, -0.0997, -2.044881182297852e-11,       1.570796327, 0, 0)
    Jc.setPos_J5_J6(0, 0.09959999999999999, -2.042830148012698e-11,      1.570796326589793, 3.141592653589793, 3.141592653589793)
    
    
    
    Jc.setJ1AxisRotation(axis_rotation_j1)
    Jc.setJ2AxisRotation(axis_rotation_j2)
    Jc.setJ3AxisRotation(axis_rotation_j3)
    Jc.setJ4AxisRotation(axis_rotation_j4)
    Jc.setJ5AxisRotation(axis_rotation_j5)
    Jc.setJ6AxisRotation(axis_rotation_j6)


    # print("Direct: \n",Jc.getDirectKin_numeric(-1.564, -0.762, -1.764, 3.91, 1.567, 0.0))
    # print("Jacobian \n",Jc.getJacobianNumeric(-1.564, -0.762, -1.764, 3.91, 1.567, 0.0))
    # print("Direct: \n",Jc.getDirectKin_numeric(0,0,0,0,0,0))
    # print("Jacobian \n",Jc.getJacobianNumeric(0,0,0,0,0,0))
    print(Jc.getJacobianParametric())

