import serial_control as sc
import time
import rospy
import numpy as np
from gantry_control_ros.srv import init_home, move_to_abs_pos, move_to_relative_pos, move_with_vel, stop_gantry, \
    init_homeResponse, move_to_abs_posResponse, move_to_relative_posResponse, move_with_velResponse, \
    stop_gantryResponse, real_time_mode, real_time_modeResponse, max_velocity, max_velocityResponse
from gantry_control_ros.msg import gantry

# GLOBAL CONST:
MAX_POSITION_X = [0, 3100]
MAX_POSITION_Y = [0, 1600]
MAX_POSITION_Z = [0, 500]

MAX_VELOCITY_X = 1000
MAX_VELOCITY_y = 1000
MAX_VELOCITY_z = 1000

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
move_to_rel_pos = False
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
    print("Gantry Stopped = ", data.data)
    stop_all = data.data

    if stop_all:
        stop_everything()
        return stop_gantryResponse("Gantry Stopped")
    else:
        return stop_gantryResponse("Gantry Moves")


def service_initialize_home(data):
    global initialize_home
    initialize_home = True
    print("INIT Home")
    x_axis_control.initialize_home_pos()
    y_axis_control.initialize_home_pos()
    return init_homeResponse("moving...")


def service_move_to_relative_pos(data):
    global move_to_rel_pos
    move_to_rel_pos = True
    x_axis_control.go_to_delta_pos_mmrad(data.x)
    y_axis_control.go_to_delta_pos_mmrad(data.y)
    return move_to_relative_posResponse("Moving to relative Position")


def service_start_realtime_mode(data):
    global stop_all, move_with_velocity, move_to_rel_pos, initialize_home, move_to_absolute_position
    stop_all = move_with_velocity = move_to_rel_pos = initialize_home = move_to_absolute_position = False
    return real_time_modeResponse("Starting Real Time Mode")

def service_set_max_velocity(data):
    if not (data.x>3000 or data.x<0):
        x_axis_control.set_drive_max_speed(data.x)
    if not (data.y>9000 or data.y<0):
        y_axis_control.set_drive_max_speed(data.y)
    # if not (data.z>101 or data.z<0):
    #     z_axis_control.set_drive_max_speed(data.z)
    return max_velocityResponse("Max Velocity Set X="+str(data.x)+" Y="+str(data.y))


def service_move_to_absolute_position(data):
    global move_to_absolute_position, stop_all
    if stop_all:
        print("STOP BUTTON PRESSED IGNORING COMMAND")
        return move_to_abs_posResponse("STOP BUTTON PRESSED IGNORING COMMAND")
    else:
        move_to_absolute_position = True

        x_axis_control.go_to_pos_mmrad(data.x)
        y_axis_control.go_to_pos_mmrad(data.y)
        # print("move_abs")
        return move_to_abs_posResponse(
            "Moving to position X=" + str(data.x) + " Y=" + str(data.y) + " Z=" + str(data.z))

        return move_to_abs_posResponse("Moving to position X=" + str(data.x) + " Y=" + str(data.y) + " Z=" + str(data.z))


def service_move_with_velocity(data):
    global move_with_velocity, stop_all
    if stop_all:
        print("STOP BUTTON PRESSED IGNORING COMMAND")
        return move_with_velResponse("STOP BUTTON PRESSED IGNORING COMMAND")
    else:
        print("move_vel")
        # TODO MAX VELOCITY MISSING
        move_with_velocity = True
        x_axis_control.set_drive_speed(data.x_d)
        y_axis_control.set_drive_speed(data.y_d)

        return move_with_velResponse("Moving with velocity of X=" + str(data.x_d) + " Y=" + str(data.y_d))


def motor_control_publisher():
    global x_axis_control, y_axis_control, pos_x_mm, pos_y_mm, pos_desired_x_mm, pos_desired_y_mm, stop_all, move_with_velocity, move_to_rel_pos, initialize_home

    pub = rospy.Publisher('/gantry/current_position', gantry, queue_size=10)
    reached = False
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():

        if not (stop_all or initialize_home or move_to_rel_pos or move_with_velocity or move_to_absolute_position):
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
        if not pos_desired_x_mm is None:
            # print("here")
            if np.sqrt((pos_x_mm - pos_desired_x_mm) ** 2 + (pos_y_mm - pos_desired_y_mm) ** 2) < WHEN_REACHED_DISTANCE:
                reached = True
            else:
                reached = False
        send_point = gantry()
        send_point.header.stamp = rospy.Time.now()
        send_point.pos_gantry.x = pos_x_mm
        send_point.pos_gantry.y = pos_y_mm
        send_point.reached = reached
        pub.publish(send_point)
        rate.sleep()


def get_desired_pos(data):
    global pos_desired_x_mm, pos_desired_y_mm

    # data = gantry()
    pos_desired_x_mm = data.pos_gantry.x
    pos_desired_y_mm = data.pos_gantry.y
    # print(pos_desired_y_mm)
    # print(pos_desired)
    return


if __name__ == '__main__':
    # global x_axis_control, y_axis_control, pos_x_mm, pos_y_mm

    rospy.init_node('Gantry', anonymous=True)
    # Start all services
    rospy.Service('/gantry/stop_all', stop_gantry, service_stop_all)
    rospy.Service('/gantry/init_home', init_home, service_initialize_home)
    rospy.Service('/gantry/rel_pos', move_to_relative_pos, service_move_to_relative_pos)
    rospy.Service('/gantry/realtime_mode', real_time_mode, service_start_realtime_mode)
    rospy.Service('/gantry/abs_pos', move_to_abs_pos, service_move_to_absolute_position)
    rospy.Service('/gantry/velocity_direct', move_with_vel, service_move_with_velocity)
    rospy.Service('/gantry/max_velocity', max_velocity, service_set_max_velocity)
    # TODO create service for max velocity
    # Start all subscribers
    rospy.Subscriber("/gantry/position_des", gantry, get_desired_pos)
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
