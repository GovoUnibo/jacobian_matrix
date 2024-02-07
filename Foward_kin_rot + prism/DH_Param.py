import numpy as np

def degree_to_radian(degree):
    return degree * np.pi / 180

class FowardKinematics(): # e la traslazione lungo y?
    arm_dh_parameters = [
        {"alpha": 0,          "a": 0.15505, "d": -0.151,  "theta":0.0, "theta_lower_limit": 0,     "theta_upper_limit": 2.75},
        {"alpha": np.pi / 2,  "a": 0.125,   "d": -0.0165, "theta":0.0, "theta_lower_limit": -1.57, "theta_upper_limit": 1.09},
        {"alpha": -np.pi / 2, "a": 0,       "d": -0.0895, "theta":0.0, "theta_lower_limit": -3.53, "theta_upper_limit": 1.57},
        {"alpha": np.pi / 2,  "a": 0.02,    "d": -0.027,  "theta":0.0, "theta_lower_limit": -0.39, "theta_upper_limit": 2.36},
        {"alpha": -np.pi / 2, "a": 0.02,    "d": 0.162,   "theta":0.0, "theta_lower_limit": -2.09, "theta_upper_limit": 2.09},
        {"alpha": np.pi / 2,  "a": 0,       "d": 0,       "theta":0.0, "theta_lower_limit": -1.41, "theta_upper_limit": 1.41},
        {"alpha": -np.pi / 2, "a": 0,       "d": 0,       "theta":0.0, "theta_lower_limit": -2.09, "theta_upper_limit": 2.09}
    ]
    
    head = [
        {"alpha": None, "a": None, "d": None, "theta":0.0, "theta_lower_limit": 0, "theta_upper_limit": 0},
        {"alpha": None, "a": None, "d": None, "theta":0.0, "theta_lower_limit": 0, "theta_upper_limit": 0}
    ]

    #from base to torso
    torso_dh_parameter = {"alpha": 0, "a": 0, "d": 0, "theta":0.0, "theta_lower_limit": 0, "theta_upper_limit": 0}#0.79
    
    def __init__(self):
        self.num_torso_joint = len(self.torso_dh_parameter)
        self.num_arm_joint = len(self.arm_dh_parameters)
        self.num_head_joint = len(self.head)

    @staticmethod
    def dh_params_to_matrix(alpha, a, d, theta_lower_limit, theta_upper_limit, theta=0):
        if theta < theta_lower_limit or theta > theta_upper_limit:
            raise ValueError("The theta value must be between {} and {}".format(theta_lower_limit, theta_upper_limit))
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0,              np.sin(alpha),                  np.cos(alpha),                 d                ],
            [0,              0,                              0,                             1                ]
        ])

    def _update_joint(self, torso_joint_pos=0, arm_joint_pos=[], head_joint_pos=[]):
        self.torso_dh_parameter['theta'] = torso_joint_pos
        for i in range(len(arm_joint_pos)):
            self.arm_dh_parameters[i]['theta'] = arm_joint_pos[i]
        for i in range(len(head_joint_pos)):
            self.head[i]['theta'] = head_joint_pos[i]
        
    def get_foward_arm(self, torso_joint_pos=0, arm_joint_pos=[0, 0, 0, 0, 0, 0, 0]):
        if len(arm_joint_pos) > self.num_arm_joint:
            raise ValueError("The number of arm joint position must be at least {}".format(self.num_arm_joint))
        torso_pos = self.torso_dh_parameter['d'] + torso_joint_pos 

        torso_matrix = self.dh_params_to_matrix(**self.torso_dh_parameter)
        arm_matrix = np.eye(4)
        for i in range(len(arm_joint_pos)):
            arm_matrix = arm_matrix @ self.dh_params_to_matrix(**self.arm_dh_parameters[i])
        return torso_matrix @ arm_matrix
    

if __name__ == "__main__":
    Fk = FowardKinematics()
    # dh param misses tf
    torso_joint_pos = 0.15
    arm1_pos = degree_to_radian(11)
    arm2_pos = degree_to_radian(-7)
    arm3_pos = degree_to_radian(111)
    arm4_pos = degree_to_radian(-90)
    arm5_pos = degree_to_radian(78)
    arm6_pos = degree_to_radian(0)
    arm_joint_pos = [arm1_pos, arm2_pos]

    print(Fk.get_foward_arm(torso_joint_pos, arm_joint_pos))