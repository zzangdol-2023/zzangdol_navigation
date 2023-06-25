import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# 목적지 포인트를 사각형 경로에 추가하는 함수
def add_waypoint(path, x, y):
    point = PoseStamped()
    point.header.frame_id = "map"
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.orientation.w = 1.0
    path.append(point)

# 현재 위치와 목적지 위치를 비교하여 도착 여부를 확인하는 함수
def check_arrival(current_pose, goal_pose):
    # distance_threshold = 3  # 도착 거리 임계값 설정
    distance_threshold = 2  # 도착 거리 임계값 설정

    # 현재 위치와 목적지 위치 사이의 거리 계산
    distance = ((goal_pose.pose.position.x - current_pose.pose.position.x) ** 2 +
                (goal_pose.pose.position.y - current_pose.pose.position.y) ** 2) ** 0.5

    # 도착 여부 반환
    return distance < distance_threshold

def pose_callback(msg):
    global current_pose
    current_pose = msg
    # rospy.loginfo("current_pose: {}".format(current_pose))

# 메인 함수
def main():
    rospy.init_node("goal_publisher")

    # 사각형 경로의 목적지 포인트 설정
    path = []
#졸프 Move base goal point


#출발지점

#출발지점-코너1 직선구간
    add_waypoint(path, 4.09 , -0.36)
    add_waypoint(path, 10.4 , -0.5)

#코너 1 in
    add_waypoint(path, 18.9 , -0.735 )


#코너 1 out
    add_waypoint(path, 21.7 , 5 )


#코너 1-2 직선구간
    add_waypoint(path, 22.4 , 13.6)
    add_waypoint(path, 22.5 , 21.2)
    add_waypoint(path, 22.7 , 27.8)


#코너 2 in
    add_waypoint(path, 22.8 , 37.4)


#코너 2 out 
    add_waypoint(path, 18.8 , 39.4)

#코너 2-3 직선구간
    add_waypoint(path, 11.5 , 40.0)
    add_waypoint(path, 4.72 , 40.2)
    add_waypoint(path, -3.95 , 40.6)
    add_waypoint(path, -11.4 , 41.1)
    add_waypoint(path, -16.7 , 41.4)


#코너 3 in
    add_waypoint(path, -20.9 , 40.8)
    add_waypoint(path, -21.7 , 41.6)


#코너 3 out
    add_waypoint(path, -24.6 , 38.2)


#코너 3-4 직선구간
    add_waypoint(path, -24.8 , 32)
    add_waypoint(path, -25.1 , 26.1)
    add_waypoint(path, -25.3 , 18.8 )
    add_waypoint(path, -25.2 , 12.6)


#코너 4 in
    add_waypoint(path, -25.4 , 6.73)


#코너 4 out
    add_waypoint(path, -21.3 , 1.12)

#코너 4-도착지점 직선구간
    add_waypoint(path, -14.1 , 0.8)
    add_waypoint(path, -8.12 , 0.62)


#도착지점
    add_waypoint(path, -2.12, 0.509)

######

    # add_waypoint(path, 1.0, -1.0)  # 테스트 포인트
    # add_waypoint(path, 1.0, 0.0)  # 테스트 포인트

    # 액션 클라이언트 생성
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    # 현재 위치 구독자 생성
   

    # 사각형 경로 순회
    for goal_pose in path:
        # 목표 위치 설정
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose

        # 이동 명령 전송
        client.send_goal(goal)

        # 이동 명령 수행 및 도착 여부 확인
        while not rospy.is_shutdown():
            # 현재 위치 수신 대기
            rospy.wait_for_message("current_pose", PoseStamped)
            
            flag=check_arrival(current_pose, goal_pose)
            # rospy.loginfo("current_pose: {}".format(current_pose.pose.positions))
            # rospy.loginfo("goal_pose: {}".format(goal_pose.pose.positions))
            # rospy.loginfo(len(path))
            # 현재 위치와 목적지 위치 비교
            if flag:
                rospy.loginfo("Reached goal: {}".format(goal_pose.pose.position))
                break
            
        # 이동 명령 완료 대기
        # client.wait_for_result()
        # rate.sleep()

    rospy.loginfo("All goals achieved")

if __name__ == "__main__":
    current_pose_sub = rospy.Subscriber("/current_pose", PoseStamped, pose_callback)
    main()
