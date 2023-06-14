#!/usr/bin/env python

import sys
import math
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal

# 현재위치 선언
global current_pose
current_pose = None

# goals & pre defined goals
goals = []
pre_defined_goals = [
    PoseStamped( 
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        )
    ),
    PoseStamped(
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        )
    ),
    PoseStamped(
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        )
    ),
    PoseStamped(
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        )
    ),            
]


# 현재위치
def pose_callback(msg):
    current_pose = msg.pose
    goal = MoveBaseActionGoal()
    goal.goal.target_pose.header.frame_id = "map"
    if  (   goals.pose.position.x - 2.0 < current_pose.position.x < goals.pose.position.x + 2.0 and #position위치맞나?, 거리는 임의지정!!
            goals.pose.position.y - 2.0 < current_pose.position.y < goals.pose.position.y + 2.0 ):        
        goal.goal.target_pose.pose = goals.pop(0)
        goal_pub.publish(goal)
        rospy.loginfo("next goal set: {}".format(goal.goal.target_pose.pose.position))

    #array가 비었을때
    elif (goals == []):
        rospy.loginfo("all goal acheived")
        sys.exit(0) 

    else:
        goal.goal.target_pose.pose = goals[0] #
        goal_pub.publish(goal) 
        distance = math.sqrt( math.pow(goals.pose.position.x - current_pose.position.x , 2) + 
                      math.pow(goals.pose.position.y - current_pose.position.y , 2) )
        rospy.loginfo("remaining distance to the goal : {}".format(distance))

def goal_reached_callback(msg):
    if msg.status.status == 3: #도착하면 loginfo뜨고 set_next
        rospy.loginfo("one of the goal acheived")    


#sys keyword 입력
input_keyword = sys.argv[1].lower()                 # ./실행파일, 입력 키워드
splited_input_keyword = input_keyword.split(" ")    # 입력 키워드 정리 'start', val1, 'len', val2 순서로 입력

#sys keyword 검사
if ((splited_input_keyword[0] == 'start') and ((splited_input_keyword[2] == 'len') )):
    if(splited_input_keyword[1] == 1 or 2 or 3 or 4): 
        start_pose = splited_input_keyword[1]
    else: rospy.loginfo("input error")

    if (splited_input_keyword[3] == 1 or 2 or 3 or 4): 
        number_of_pose = splited_input_keyword[3]
    else: rospy.loginfo("input error")
else: rospy.loginfo("input keyword error")

#goals에 predefined 순서대로 넣기
for i in range(number_of_pose):
    goals.append(pre_defined_goals[(start_pose + i - 1) % 4])

if __name__ == "__main__":
    rospy.init_node("goal_navigation")
    pose_sub = rospy.Subscriber("/current_pose", PoseStamped, pose_callback)                                 # current_pose_topic은 임의로 넣었음 토픽이름 바꿔줘야함.
    goal_reached_sub = rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_reached_callback)    # 도착 점검           
    goal_pub = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=1)
    rospy.spin()


"""
pub토픽 바꿔줘야함.
0.  키워드를 통해 시작위치와 개수를 입력받는다, goals array를 통해 goal을 미리 선언해놓는다
1.  (시작위치+1) pose를 pub한다.
2.  current_pose와 (시작위치+1)_pose를 비교한다. 
3.  만약 범위 내에 있다면 goals에 current_pose+1을 pop한다.
"""


