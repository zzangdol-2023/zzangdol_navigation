#! /usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion


def pose_callback(msg):
    global current_pose
    current_pose = msg
    # rospy.loginfo("current_pose: {}".format(current_pose))

# 목적지 포인트를 사각형 경로에 추가하는 함수
def add_pathPoint(path, x, y):
    point = PoseStamped()
    point.header.frame_id = "map"
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.orientation.w = 1.0
    path.append(point)

# 현재 위치와 목적지 위치를 비교하여 도착 여부를 확인하는 함수
def check_arrival(current_pose, goal_pose):
    distance_threshold = 1  # 도착 거리 임계값 설정

    # 현재 위치와 목적지 위치 사이의 거리 계산
    distance = ((goal_pose.pose.position.x - current_pose.pose.position.x) ** 2 +
                (goal_pose.pose.position.y - current_pose.pose.position.y) ** 2) ** 0.5
    
    rospy.loginfo(f"distance to goal_pose: {distance}")
    # 도착 여부 반환
    return distance < distance_threshold

# 메인 함수
def main():
    rospy.init_node("goal_publisher")
    rospy.loginfo(f"goal_publisher_started -v1")
    path = [] # 사각형 경로의 목적지 포인트 설정

##### Move base goal point

#출발지점 - 코너 1 직선구간
    add_pathPoint(path, 15.8 , -1.46) # in
    add_pathPoint(path, 24.9 , -1.33) # mid-1
    add_pathPoint(path, 34.7 , -2.99) # mid
    add_pathPoint(path, 42.6 , -3.82) # mid-2

#코너 1
    add_pathPoint(path, 51.8 , -4.48) # 직선구간 out - 코너1 in
    add_pathPoint(path, 55.5 , -4.67) # 코너1 mid
    add_pathPoint(path, 58.5 , -1.15 ) # 코너1 out - 직선구간 in

#코너 1 - 코너2 직선구간
    add_pathPoint(path, 59.5 , 11.3) # mid -1
    add_pathPoint(path, 60 , 17.6) # mid
    add_pathPoint(path, 60.4 , 23.9) # mid
    
#코너 2 
    add_pathPoint(path, 60.8 , 30.0) # 직선구간 out - 코너2 in
    add_pathPoint(path, 58.8 , 35.2) # 코너2 mid
    add_pathPoint(path, 54.9 , 36.3) # 코너2 out - 직선구간 in

#코너 2 - 코너3 직선구간
    add_pathPoint(path, 48.2 , 36.9) # mid-1
    add_pathPoint(path, 37.7 , 37.4) # mid
    add_pathPoint(path, 25.6 , 38.1) # mid-2

#코너 3 
    add_pathPoint(path, 18.3 , 38.8) # 직선구간 out - 코너3 in
    add_pathPoint(path, 15.2 , 37.2) # 코너3 mid
    add_pathPoint(path, 13.4 , 31.4) # 코너3 out - 직선구간 in

#코너 3 - 4 직선구간
    add_pathPoint(path, 12.9 , 26.3) # mid-1 
    add_pathPoint(path, 12.2 , 19.6) # mid
    add_pathPoint(path, 11.6 , 10.3) # mid-2
    
#코너 4 in
    add_pathPoint(path, 11.4 , 0.412) # 직선구간 out - 코너4 in

#도착지점
    add_pathPoint(path, 15.8, -1.47) # 코너 1 in
######

    # 액션 클라이언트 생성
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    
    # 사각형 경로 순회
    for goal_pose in path:
        # 목표 위치 설정
        goal_location = MoveBaseGoal()
        goal_location.target_pose = goal_pose

        # 이동 명령 전송
        client.send_goal(goal_location)

        # 이동 명령 수행 및 도착 여부 확인
        while not rospy.is_shutdown():
            # 현재 위치 수신 대기
            rospy.wait_for_message("current_pose", PoseStamped)
            flag = check_arrival(current_pose, goal_pose)

            # 현재 위치와 목적지 위치 비교
            if flag:
                rospy.loginfo(f"Reached goal: {goal_pose.pose.position}")
                break
    rospy.loginfo("All goals achieved")

if __name__ == "__main__":
    # 현재 위치 구독자 생성
    current_pose_sub = rospy.Subscriber("/current_pose", PoseStamped, pose_callback)
    main()



# rospy.loginfo("current_pose: {}".format(current_pose.pose.positions))
# rospy.loginfo("goal_pose: {}".format(goal_pose.pose.positions))
# rospy.loginfo(len(path))