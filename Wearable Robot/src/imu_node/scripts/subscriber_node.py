import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32

def callback(data):
    rospy.loginfo(f"Received data: {data.data}")

    # 데이터의 합 계산
    total_sum = sum(data.data)

    output = 1 if total_sum > 0 else 0

    # 결과 퍼블리시 (퍼블리셔 기능을 추가)
    pub.publish(output)

    # 결과 출력
    rospy.loginfo(f"Output: {output}")

# 퍼블리셔 설정
pub = rospy.Publisher('/imu_output', Int32, queue_size=10)

# 노드 초기화
def listener():
    rospy.init_node('imu_subscriber_node', anonymous=True)
    rospy.Subscriber("/mpu6050_data", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
