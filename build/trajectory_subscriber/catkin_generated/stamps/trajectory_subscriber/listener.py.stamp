import rospy
from moveit_msgs.msg import DisplayTrajectory

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.trajectory.multi_dof_joint_trajectory.points.transforms.translation[0])

def listener():
    rospy.init_node('trajectory_listener', anonymous=False)

    rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()