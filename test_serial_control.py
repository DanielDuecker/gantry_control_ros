import serial_control as sc
import time
import rospy
from geometry_msgs.msg import PointStamped
# y_axis_control_serial = sc.MotorCommunication('/dev/ttyS1', 'spindle_drive', 19200, 'spindle', 1600, 945800)
# y_axis_control_serial.open_port()
# y_axis_control_serial.set_drive_max_speed(2000)
# y_axis_control_serial.set_home_pos_known(True)
# print(y_axis_control_serial.get_posmmrad())
# y_axis_control_serial.go_to_pos_mmrad(-100)
# time.sleep(2)
# y_axis_control_serial.go_to_pos_mmrad(100)
# print(y_axis_control_serial.get_posmmrad())
# print("done")
tmp=True
pos_desired=None

def get_desired_pos(data):
    global pos_desired

    # data = PointStamped()
    pos_desired = data.point.y
    #print(pos_desired)
    return

def motor_control_publisher():
    global pos_desired
    y_axis_control_serial = sc.MotorCommunication('/dev/ttyS1', 'spindle_drive', 19200, 'spindle', 1600, 945800)
    y_axis_control_serial.open_port()
    y_axis_control_serial.set_home_pos_known(True)
    y_axis_control_serial.set_drive_max_speed(1000)
    pos_x_mm = y_axis_control_serial.get_posmmrad()

    pub = rospy.Publisher('test_pos', PointStamped, queue_size=10)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        print(pos_desired)
        if pos_desired is not None :
            y_axis_control_serial.go_to_pos_mmrad(pos_desired)
            y_axis_control_serial.go_to_pos_mmrad(-600)
            time.sleep(5)
            y_axis_control_serial.set_drive_speed(0)
            time.sleep(5)
            y_axis_control_serial.go_to_pos_mmrad(-600)
        else:
            print("is not set yet")
        pos_x_mm = y_axis_control_serial.get_posmmrad()
        send_point = PointStamped()
        send_point.header.stamp = rospy.Time.now()
        send_point.point.y = pos_x_mm
        pub.publish(send_point)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("/gantry_position_des", PointStamped, get_desired_pos)
    try:
        motor_control_publisher()
    except rospy.ROSInterruptException:
        pass