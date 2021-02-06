import rospy
from moveit_msgs.msg import DisplayTrajectory
from mrs_msgs.msg import UavState
from mrs_msgs.srv import Vec4
import numpy as np

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", str(data.trajectory[0].multi_dof_joint_trajectory.points[1].transforms[0].translation))
    #rospy.loginfo("I got it!")
def listener():
    rospy.init_node('trajectory_listener', anonymous=False)

    rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, callback)

    rospy.spin()

class mrs_utils:
    def __init__(self):
        self.position_cmd_call = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)
        self.tolerable_error = 0.2

    def take_observation(self):
        data_pose = None
        
        while data_pose is None:
            try:
                data_uavstate = rospy.wait_for_message('/uav1/odometry/uav_state', UavState, timeout=1)
                data_pose = data_uavstate.pose 
            except:
                #a = 1
                rospy.loginfo("Current drone pose not ready yet, retrying for getting robot pose")
        return data_pose

    def distance(self, data_pose, reference_position):
        current_pose = [data_pose.position.x, data_pose.position.y, data_pose.position.z]
        
        err = np.subtract(current_pose, reference_position)
        #w = np.array([1, 1, 4])
        #err = np.multiply(w,err)
        dist = np.linalg.norm(err)
        return dist

    def cmd_achieve(self, command_call): 
        cmd_achieved = False
        cmd_position_3D = command_call[:-1]
        while cmd_achieved == False:
            data_pose = self.take_observation()
            dist = self.distance(data_pose, cmd_position_3D)
            if dist < self.tolerable_error:
                cmd_achieved = True
        return cmd_achieved

if __name__ == '__main__':
    listener()
    mrs = mrs_utils()