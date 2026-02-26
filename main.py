import pybullet as p
import pybullet_data
import time
import os
import threading
from dotenv import load_dotenv

# Import local modules
from agent import SpatialReasoningAgent
from kinematics import KinematicsController

# Load environment variables
load_dotenv()

# Global variable to store the latest target position
target_position = None
position_lock = threading.Lock()

def command_thread(agent):
    """Runs continuously, prompting the user for commands without blocking the simulation."""
    global target_position
    
    # Wait a bit for PyBullet to initialize before prompting
    time.sleep(2)
    
    while True:
        try:
            command = input("\n[Agent] Enter a spatial command (or 'quit' to exit): ")
            if command.lower() in ['quit', 'exit', 'q']:
                os._exit(0)
                
            if not command.strip():
                continue
                
            print("[Agent] Processing command...")
            parsed = agent.get_target_coordinates(command)
            
            if parsed:
                coords = parsed["coords"]
                grip_state = parsed["grip"]
                print(f"[Agent] Target: X={coords[0]:.3f}, Y={coords[1]:.3f}, Z={coords[2]:.3f} | Grip: {'Open' if grip_state == 1 else 'Closed'}")
                with position_lock:
                    target_position = parsed
            else:
                print("[Agent] Failed to calculate target parameters.")
                
        except Exception as e:
            print(f"[Agent] Error reading command: {e}")
            break

def main():
    global target_position
    
    # Initialize the LLM Agent
    agent = SpatialReasoningAgent()
    
    # --- 1. PyBullet Environment Setup ---
    # Connect to PyBullet GUI
    physicsClient = p.connect(p.GUI)
    
    # Configure camera to look at the workspace
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.5, 0, 0])

    # Set the search path to find PyBullet's built-in data files
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set standard Earth gravity
    p.setGravity(0, 0, -9.81)
    
    # Load environment
    planeId = p.loadURDF("plane.urdf")
    tableId = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65]) # Place table so surface is roughly Z=0
    
    # Load the Franka Panda robotic arm
    robotStartPos = [0, 0, 0]
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("franka_panda/panda.urdf", robotStartPos, robotStartOrientation, useFixedBase=True)
    
    # Identify the end effector link (link 11 is typical for the Panda's gripper center)
    numJoints = p.getNumJoints(robotId)
    endEffectorIndex = 11 
    
    # Define joint mapping (Panda has 7 DOF joints, plus 2 finger joints)
    revoluteJointIndices = []
    fingerJointIndices = []
    
    for j in range(numJoints):
        jointInfo = p.getJointInfo(robotId, j)
        qIndex = jointInfo[3]
        jointName = jointInfo[1].decode("utf-8")
        
        if qIndex > -1: # Movable joint
            revoluteJointIndices.append(j)
            
        # The Panda fingers are prismatic/revolute joints named panda_finger_joint1 and panda_finger_joint2
        if "finger_joint" in jointName:
            fingerJointIndices.append(j)
    
    # We only care about the 7 arm joints for IK
    armJointIndices = revoluteJointIndices[:7]

    # Initialize Kinematics Controller
    kinematics = KinematicsController(robotId, endEffectorIndex, armJointIndices, fingerJointIndices)

    # --- 2. Initial Setup ---
    # Move arm to a starting posture
    initial_angles = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785] # Standard ready position
    for i, angle in enumerate(initial_angles):
        p.resetJointState(robotId, armJointIndices[i], angle)
        
    # Open gripper initially (0.04m is roughly open for panda fingers)
    for finger in fingerJointIndices:
        p.resetJointState(robotId, finger, 0.04)
        
    print("Simulation environment initialized.")
    
    # Start the user input thread
    input_thread = threading.Thread(target=command_thread, args=(agent,), daemon=True)
    input_thread.start()

    # --- 3. Execution Loop ---
    # Draw a line representing target coordinates in debug visualization
    debug_marker_id = -1
    
    try:
        while True:
            # Check if there's a new target position to go to
            current_target = None
            current_grip = 1 # Default to open
            
            with position_lock:
                if target_position is not None:
                    current_target = list(target_position["coords"])
                    current_grip = target_position["grip"]

            if current_target:
                # Actuate robot using the Kinematics Controller
                kinematics.move_to_target(current_target, current_grip)
                    
                # Update debug visualizer marker
                if debug_marker_id != -1:
                    p.removeUserDebugItem(debug_marker_id)
                # Draw a small red line to indicate target
                debug_marker_id = p.addUserDebugLine(
                    current_target, 
                    [current_target[0], current_target[1], current_target[2] + 0.1], 
                    lineColorRGB=[1, 0, 0], 
                    lineWidth=3
                )

            # Step physics 
            p.stepSimulation()
            time.sleep(1./240.) # 240 Hz
            
    except KeyboardInterrupt:
        print("\nShutting down simulation based on keyboard interrupt.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()

