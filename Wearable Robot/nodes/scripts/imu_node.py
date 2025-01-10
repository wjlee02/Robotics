import rospy
from std_msgs.msg import Float32MultiArray
import serial

def talker():
    pub = rospy.Publisher('mpu6050_data', Float32MultiArray, queue_size=10)
    rospy.init_node('mpu6050_node', anonymous=True)
    rate = rospy.Rate(1)  # 10 Hz

    # Adjust the serial port and baudrate as per your setup
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            rospy.loginfo(f"Raw data received: {line}")  # 디버깅용 로그

            try:
                # Parse CSV data
                data = [float(x) for x in line.split(",")]
                if len(data) == 6:  # Ensure 6 values are received
                    msg = Float32MultiArray(data=data)
                    pub.publish(msg)
                else:
                    rospy.logwarn(f"Unexpected data format: {line}")
            except ValueError:
                rospy.logwarn(f"Failed to parse line: {line}")

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
