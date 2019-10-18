import serial_control as sc
import time
import rospy
import numpy as np
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsResponse  # has to be changed to actual srv
from geometry_msgs.msg import PointStamped  # has to be changed to gantry msg

# GLOBAL CONST:
MAX_POSITION_X = [0, 3100]
MAX_POSITION_Y = [0, 1600]
MAX_POSITION_Z = [0, 500]

MAX_VELOCITY_X = 3000
MAX_VELOCITY_y = 3000
MAX_VELOCITY_z = 3000

WHEN_REACHED_DISTANCE = 1  # distance in 3d in mm

# GLOBAL VARIABLES
x_axis_control = None
y_axis_control = None
pos_x_mm = None
pos_y_mm = None
pos_reached = None
pos_desired_x_mm = None
pos_desired_y_mm = None
# Program State:
stop_all = False
initialize_home = False
move_to_relative_pos = False
move_to_absolute_position = False
move_with_velocity = False
real_time_pos = False


def init_axis(axis):
    axis.open_port()
    axis.set_home_pos_known(True)
    axis.set_drive_max_speed(1000)
    return axis.get_posmmrad()


def stop_everything():
    x_axis_control.set_drive_speed(0)
    y_axis_control.set_drive_speed(0)


def service_stop_all(data):
    global stop_all
    stop_all = data.data
    stop_everything()
    return AddTwoIntsResponse("Gantry Stopped")


def service_initialize_home():
    pass


def service_move_to_relative_pos():
    pass


def service_move_to_absolute_position():
    pass


def service_move_with_velocity():
    pass


def motor_control_publisher():
    global x_axis_control, y_axis_control, pos_x_mm, pos_y_mm

    pub = rospy.Publisher('/gantry/current_position', PointStamped, queue_size=10)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if not (stop_all or initialize_home or move_to_relative_pos or move_with_velocity):
            # code for control:
            if pos_desired_x_mm is not None:
                x_axis_control.go_to_pos_mmrad(pos_desired_x_mm)
            else:
                print("No x control started since pos_desired is not set yet")
            if pos_desired_y_mm is not None:
                y_axis_control.go_to_pos_mmrad(pos_desired_y_mm)
            else:
                print("No y control started since pos_desired is not set yet")
        # publish position
        pos_x_mm = x_axis_control.get_posmmrad()
        pos_y_mm = y_axis_control.get_posmmrad()
        if np.sqrt((pos_x_mm - pos_desired_x_mm) ** 2 + (pos_x_mm - pos_desired_y_mm) ** 2) < WHEN_REACHED_DISTANCE:
            reached = True
        else:
            reached = False
        send_point = PointStamped()
        send_point.header.stamp = rospy.Time.now()
        send_point.point.x = pos_x_mm
        send_point.point.y = pos_y_mm
        pub.publish(send_point)
        rate.sleep()


def get_desired_pos():
    pass


if __name__ == '__main__':
    global x_axis_control, y_axis_control, pos_x_mm, pos_y_mm
    test = 5
    rospy.init_node('Gantry', anonymous=True)
    # Start all services
    rospy.Service('/gantry/stop_all', AddTwoInts, service_stop_all)
    rospy.Service('/gantry/init_home', AddTwoInts, service_initialize_home)
    rospy.Service('/gantry/rel_pos', AddTwoInts, service_move_to_relative_pos)
    rospy.Service('/gantry/abs_pos', AddTwoInts, service_move_to_absolute_position)
    rospy.Service('/gantry/velocity_direct', AddTwoInts, service_move_with_velocity)
    # Start all subscribers
    # Start Gantry
    x_axis_control = sc.MotorCommunication('/dev/ttyS0', 'belt_drive', 115200, 'belt', 3100, 2000e3)
    y_axis_control = sc.MotorCommunication('/dev/ttyS1', 'spindle_drive', 19200, 'spindle', 1600, 945800)
    pos_x_mm = init_axis(x_axis_control)
    pos_y_mm = init_axis(y_axis_control)
    # Start all publishers
    try:
        motor_control_publisher()
    except rospy.ROSInterruptException:
        pass
    stop_everything()