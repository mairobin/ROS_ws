import os
import sys

"""
I separated the use_api.py script from the ROS scripts
to make it more realisitic. 
Therefore I add the location of the API to the path programmaticaly
"""
script_dir = os.path.join(os.path.dirname(__file__), 'scripts')
sys.path.insert(0, script_dir)

# Import the RobotStateAPI class from the custom API
from my_api import RobotStateAPI

def main():
    # Instantiate the RobotStateAPI
    api = RobotStateAPI()
    # Get the robot state
    robot_state = api.get_robot_state()
    
    # Print the joint states
    print("Joint Names:", robot_state.name)
    print("Joint Positions:", robot_state.position)
    print("Joint Velocities:", robot_state.velocity)
    print("Joint Efforts:", robot_state.effort)

if __name__ == "__main__":
    main()
