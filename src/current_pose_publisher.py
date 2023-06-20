import rospy
import tf2_ros
import geometry_msgs.msg

rospy.init_node('current_pose_publisher')

tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)
pose_publisher = rospy.Publisher('current_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

while not rospy.is_shutdown():
    try:
        # 현재 시간 기준으로 'map' 좌표계에서 'base_link' 좌표계로 변환된 pose 가져오기
        transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time())

        # pose 정보를 geometry_msgs.msg.PoseStamped 형태로 변환하여 발행
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose.position = transform.transform.translation
        pose_stamped.pose.orientation = transform.transform.rotation

        pose_publisher.publish(pose_stamped)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue
