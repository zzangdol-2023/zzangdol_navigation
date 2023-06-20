import sys
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from std_msgs.msg import Header
import actionlib
import actionlib_msgs.msg
current_pose = None
rospy.init_node("goal_navigation")

# 목표 위치 선언
goals = []
goals.append(MoveBaseActionGoal(MoveBaseGoal(PoseStamped(Pose(Point(0.0, 0.0, 0.0))))))
goals.append(MoveBaseActionGoal(MoveBaseGoal(PoseStamped(Pose(Point(0.0, 0.0, 0.0))))))
goals.append(MoveBaseActionGoal(MoveBaseGoal(PoseStamped(Pose(Point(0.0, 0.0, 0.0))))))
goals.append(MoveBaseActionGoal(MoveBaseGoal(PoseStamped(Pose(Point(0.0, 0.0, 0.0))))))

# 현재 위치 콜백 함수
def pose_callback(msg):
    global current_pose
    current_pose = msg.pose

    #actionlib을 통해 순서대로 목표 위치로 이동
    client = actionlib.SimpleActionClient('move_base', MoveBaseActionGoal)

    #actionlib 가능 함수 종류는 다음과 같다.
    # client.wait_for_server()
    # client.get_state()
    # client.get_result()
    # client.get_goal_status_text()
    # client.cancel_goal()
    # client.wait_for_result()
    # client.send_goal(goal)
    # client.wait_for_result(rospy.Duration.from_sec(5.0))
    # client.wait_for_result()
    # client.send_goal(goal, feedback_cb=feedback_callback)
    # client.wait_for_result()

    
    

    # if (goals[0].goal.target_pose.pose.position.x - 2.0 < current_pose.position.x < goals[0].goal.target_pose.pose.position.x + 2.0 and
    #         goals[0].goal.target_pose.pose.position.y - 2.0 < current_pose.position.y < goals[0].goal.target_pose.pose.position.y + 2.0):
    
    #     goal = PoseStamped()
    #     goal = goals[0].goal.target_pose
    #     goal_pub.publish(goal)
    #     goals.pop(0)
    #     rospy.loginfo("Next goal set: {}".format(goal.goal.target_pose.pose.position))        

    # elif not goals:
    #     rospy.loginfo("All goals achieved")
    #     sys.exit(0)

    # else:
    #     #goal.goal.target_pose = goals[0].target_pose
    #     goal.pose.position.x = goals[0].goal.target_pose.pose.position.x
    #     goal_pub.publish(goal)
    #     distance = ((goals[0].goal.target_pose.pose.position.x - current_pose.position.x) ** 2 +
    #                 (goals[0].goal.target_pose.pose.position.y - current_pose.position.y) ** 2) ** 0.5
    #     rospy.loginfo("Remaining distance to the goal: {}".format(distance))


if __name__ == "__main__":
    pose_sub = rospy.Subscriber("current_pose", PoseStamped, pose_callback)
    goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)

    rospy.spin()
