from Foward_kin.Foward import FowardKinematics


class TiagoForward():

    def __init__(self):

        # Create ForwardKinematics objects for each tree.
        self.base_to_arm = FowardKinematics(10)  
        self.base_to_head = FowardKinematics(3)



        # Adding joints in base to arm tree
        self.base_to_arm.add_prismatic_joint([-0.062, 0.000, 0.79], [1.000, 0.000, 0.000, 0.000], [0, 0, 1], 0)    # torso joint
        self.base_to_arm.add_rot_joint([0.15505, 0.014, -0.151], [0.77741, 0.000, 0.000, -0.6633], [0, 0, 1], 1) # arm1 joint
        self.base_to_arm.add_rot_joint([0.125, 0.019, -0.031], [0.707, 0.707, -0.001, 0.001], [0, 0, 1], 2) # arm2 joint
        self.base_to_arm.add_rot_joint([0.089, 0.000, -0.002], [0.500, -0.500, -0.500, 0.500], [0, 0, 1], 3)    #arm3 joint
        self.base_to_arm.add_rot_joint([-0.020, -0.027, -0.222], [-0.500, 0.500, 0.500, 0.500], [0, 0, 1], 4)   #arm4 joint
        self.base_to_arm.add_rot_joint([-0.162, 0.020, 0.027], [0.707, -0.002, -0.707, 0.002], [0, 0, 1], 5)    #arm5 joint
        self.base_to_arm.add_rot_joint([0.000, 0.000, 0.150], [-0.499, 0.499, 0.501, 0.501], [0, 0, 1], 6)  # arm6 joint
        self.base_to_arm.add_rot_joint([0.077, -0.000, 0.000], [0.707, -0.000, -0.707, 0.000], [0, 0, 1], 7)    # wrist joint
        self.base_to_arm.add_prismatic_joint([0.000, 0.000, 0.000], [1.000, 0.000, 0.000, 0.000], [-1, 0, 0], 8)    # gripper1 joint
        self.base_to_arm.add_prismatic_joint([0.000, 0.000, 0.000], [0.000, 0.000, 0.000, 1.000], [1, 0, 0], 9)    #gripper2 joint

        # Adding joints in base to head tree
        self.base_to_head.add_prismatic_joint([-0.062, 0.000, 0.79], [1.000, 0.000, 0.000, 0.000], [0, 0, 1], 0)    # torso joint
        self.base_to_head.add_rot_joint([0.182, 0.000, 0.000], [0.000, 0.000, 0.000, 1.000], [0, 0, 1], 1)  # head1 joint
        self.base_to_head.add_rot_joint([0.005, 0.000, 0.098], [0.707, -0.000, 0.000, 0.707], [0, 0, 1], 2) # head2 joint

    def setWorldBlPose_base_to_arm(self, position, orientation):
        self.base_to_arm.set_World_Bl_pose(position, orientation)
    
    def setWorldBlPose_base_to_head(self, position, orientation):
        self.base_to_head.set_World_Bl_pose(position, orientation)

    def get_world_to_base_arm_tree(self):
        return self.base_to_arm.get_World_to_BL(as_pos_quaternion=True)

    # Getting foreward kinematics for base to arm tree.
    def getForwardKin_base_to_arm(self, joint_array=[], as_matrix=False, as_pos_euler=False, as_pos_quaternion=False):
        return self.base_to_arm.getDirectKin_numeric(joint_array, as_matrix, as_pos_euler, as_pos_quaternion)
        
    # Getting forward kinematics for base to head tree.
    def getForwardKin_base_to_head(self, joint_array=[], as_matrix=False, as_pos_euler=False, as_pos_quaternion=False):
        return self.base_to_head.getDirectKin_numeric(joint_array, as_matrix, as_pos_euler, as_pos_quaternion)

    

        