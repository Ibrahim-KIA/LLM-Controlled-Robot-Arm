import json
import os
from groq import Groq

# Define workspace boundaries based on the simulated table
WORKSPACE_BOUNDS = {
    "X": [0.3, 0.7],    # Reach out in front
    "Y": [-0.5, 0.5],   # Left to right
    "Z": [0.1, 0.6]     # Above the table
}

class SpatialReasoningAgent:
    def __init__(self):
        if not os.getenv("GROQ_API_KEY"):
            print("WARNING: GROQ_API_KEY not found in environment variables. Please add it to your .env file.")
        try:
            self.client = Groq()
        except Exception as e:
            print(f"Error initializing Groq client: {e}")
            self.client = None

    def get_target_coordinates(self, user_prompt):
        """Sends the user prompt to Llama 3.3 and parses the returned JSON target coordinates."""
        if not self.client:
            print("Groq client not initialized. Cannot process command.")
            return None

        system_prompt = f"""
        You are a spatial reasoning AI engine controlling a 6-DOF robotic manipulator.
        Your task is to convert plain English commands into exact 3D Cartesian coordinates (X, Y, Z).
        
        The robot operates in a physical workspace with the following boundaries (in meters):
        - X (Forward/Backward): {WORKSPACE_BOUNDS['X'][0]} to {WORKSPACE_BOUNDS['X'][1]}
        - Y (Left/Right): {WORKSPACE_BOUNDS['Y'][0]} to {WORKSPACE_BOUNDS['Y'][1]}
        - Z (Up/Down): {WORKSPACE_BOUNDS['Z'][0]} to {WORKSPACE_BOUNDS['Z'][1]}
        
        A typical resting position is roughly X=0.5, Y=0.0, Z=0.3.
        
        CRITICAL RULES:
        1. Output strictly valid JSON and nothing else. Do not use markdown blocks (e.g., no ```json ... ```).
        2. Ensure the coordinates strictly fall within the provided workspace boundaries. If a command implies moving outside these bounds, constrain the target to the nearest boundary edge.
        3. The JSON must follow exactly this format: {{"x": float, "y": float, "z": float, "grip": int}}
        4. The "grip" parameter represents the gripper state: 0 for closed (grabbing), 1 for open (releasing).
        5. DO NOT output multi-line mathematical expressions or formulas inside the JSON. Do the math internally and only output the final computed float values.
        6. If the user gives a multi-step command (e.g. "go down, close grip, then go up"), output ONLY the FINAL intended coordinate and grip state at the end of their logic.
        """

        try:
            response = self.client.chat.completions.create(
                model="llama-3.3-70b-versatile",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.0, # Deterministic outputs for safety
                max_tokens=50
            )
            
            raw_output = response.choices[0].message.content.strip()
            
            # Clean up markdown if the LLM hallucinated it despite instructions
            if raw_output.startswith("```json"):
                raw_output = raw_output[7:]
            if raw_output.startswith("```"):
                raw_output = raw_output[3:]
            if raw_output.endswith("```"):
                raw_output = raw_output[:-3]
                
            target_data = json.loads(raw_output.strip())
            
            # Validate format
            if all(key in target_data for key in ["x", "y", "z", "grip"]):
                return {
                    "coords": [target_data["x"], target_data["y"], target_data["z"]],
                    "grip": target_data["grip"]
                }
            else:
                print(f"Error: LLM output missing required keys. Output: {target_data}")
                return None
                
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON from LLM: {e}")
            print(f"Raw LLM output was: {raw_output}")
            return None
        except Exception as e:
            print(f"Error processing command with Groq: {e}")
            return None
