import pybullet as p

class KinematicsController:
    def __init__(self, robot_id, end_effector_index, arm_joint_indices, finger_joint_indices):
        self.robot_id = robot_id
        self.end_effector_index = end_effector_index
        self.arm_joint_indices = arm_joint_indices
        self.finger_joint_indices = finger_joint_indices

    def move_to_target(self, target_coords, grip_state):
        """
        Calculates IK for the target coordinates and actuates the motors.
        """
        # Calculate Inverse Kinematics
        # We optionally pass a target orientation (quaternion) pointing down towards the table
        targetOrientation = p.getQuaternionFromEuler([3.14, 0, 0]) # Gripper pointing down
        
        jointPoses = p.calculateInverseKinematics(
            self.robot_id, 
            self.end_effector_index, 
            target_coords, 
            targetOrientation,
            maxNumIterations=100,
            residualThreshold=0.01
        )
        
        # Apply position control to drive motors to target angles
        for i in range(len(self.arm_joint_indices)):
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=self.arm_joint_indices[i],
                controlMode=p.POSITION_CONTROL,
                targetPosition=jointPoses[i],
                force=100.0,          # Max motor force
                maxVelocity=1.5       # Speed limit
            )
            
        # Apply position control to fingers
        finger_target_pos = 0.04 if grip_state == 1 else 0.0 # 0.04 is open, 0.0 is closed
        for finger in self.finger_joint_indices:
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=finger,
                controlMode=p.POSITION_CONTROL,
                targetPosition=finger_target_pos,
                force=50.0 # Finger force
            )
