import serial_control as sc
import time
import rospy


# GLOBAL CONST:
MAX_POSITION_X =[0,3100]
MAX_POSITION_Y =[0,1600]
MAX_POSITION_Z =[0,500]

MAX_VELOCITY_X = 3000
MAX_VELOCITY_y = 3000
MAX_VELOCITY_z = 3000

# GLOBAL VARIABLES

x_axis_control = None
y_axis_control = None
pos_x_mm = None
pos_y_mm = None
pos_reached = None
# Program State:
stop_all = False
initialize_home = False
move_to_relative_pos = False
move_to_absolute_position = False
move_with_velocity = False








def init_axis(axis):
    axis.open_port()
    axis.set_home_pos_known(True)
    axis.set_drive_max_speed(1000)
    return axis.get_posmmrad()

def service_stop_all():
    pass
def stop_everything():
    x_axis_control.set_drive_speed(0)
    y_axis_control.set_drive_speed(0)
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

    if not( stop_all or initialize_home or move_to_relative_pos or move_to_absolute_position or move_with_velocity):
        #code for control:
        pass
    # publish position




    pass
def get_desired_pos():
    pass


if __name__ == '__main__':
    global x_axis_control,y_axis_control,pos_x_mm,pos_y_mm
    rospy.init_node('Gantry', anonymous=True)
    # Start all services
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