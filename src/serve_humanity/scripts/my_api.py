import rospy
from sensor_msgs.msg import JointState

class RobotStateAPI:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('robot_state_api_node', anonymous=True)
        # Initialize the joint_states variable
        self.joint_state = None
        # Subscribe to the /joint_states topic
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        # Callback to update the joint state
        self.joint_state = msg

    def get_robot_state(self):
        # Get the latest joint state
        while self.joint_state is None:
            rospy.sleep(0.1)
        return self.joint_state
    
