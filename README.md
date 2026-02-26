# Agentic Control System for 6-DOF Robotic Manipulators

This project bridges natural language processing and mechatronics by allowing an AI agent to control a simulated 6-Degree-of-Freedom (6-DOF) robotic arm.

Instead of hard-coding physical waypoints, this system uses an LLM (Llama 3.3) as a spatial reasoning engine to translate human commands into precise 3D Cartesian coordinates, which are then solved via Inverse Kinematics (IK) to actuate the robot's motors.

## ⚙️ Architecture

1. **The Brain (LLM):** Llama 3.3 (via Groq API) processes text commands and outputs strict JSON spatial targets `(x, y, z)`.
2. **The Physics Bridge (IK):** PyBullet's physics engine takes the target coordinates and calculates the exact joint angles `(θ)` required for the end-effector to reach the destination.
3. **The Simulation:** A Franka Emika Panda arm is actuated in real-time within a PyBullet GUI using position control loops.

## 🚀 Tech Stack

- **Simulation & Kinematics:** PyBullet, NumPy
- **AI/LLM:** Llama 3.3 (Groq API)
- **Language:** Python 3.11+

## 🛠️ Quick Start

1. Clone the repository:

   ```bash
   git clone https://github.com/yourusername/agentic-manipulator-control.git
   cd agentic-manipulator-control
   ```

2. Install dependencies:

   ```bash
   pip install -r requirements.txt
   ```

3. Add your API key:
   Create a `.env` file in the root directory and add:

   ```env
   GROQ_API_KEY=your_api_key_here
   ```

4. Run the simulation:
   ```bash
   python main.py
   ```

## 🎥 Demo

_(Add a GIF or link to a video demonstrating the terminal prompt and the robotic arm moving here)_

_Note: Make sure to replace `yourusername` in the clone link with your actual GitHub username!_
