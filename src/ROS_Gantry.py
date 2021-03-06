import serial_control as sc
import time
import rospy
import numpy as np
import warnings
from gantry_control_ros.srv import init_home, move_to_abs_pos, move_to_relative_pos, move_with_vel, stop_gantry, \
    init_homeResponse, move_to_abs_posResponse, move_to_relative_posResponse, move_with_velResponse, \
    stop_gantryResponse, real_time_mode, real_time_modeResponse, max_velocity, max_velocityResponse

from gantry_control_ros.msg import gantry

WHEN_REACHED_DISTANCE = 2  # distance in 3d in mm

# GLOBAL VARIABLES
oGantry = None
pos_reached = None

# Program State:
stop_all = False
initialize_home = False
flag_move_to_rel_pos = False
rel_pos_des_glob_mm = np.array([0, 0, 0])

flag_move_to_abs_pos = False
abs_pos_des_glob_mm = np.array([0, 0, 0])

flag_real_time_mode = False

# move_with_velocity = False
real_time_pos = False


def init_system():
    global oGantry

    print("[ROS_Gantry] booting...")
    gantry_status = oGantry.open_port()

    time.sleep(0.5)
    time_boot = time.time()
    init_boot_time = 2
    time_out_sec = 0.2
    time_out_timer = time.time()
    gantry_msg_counter = 0
    while time.time() - time_boot < init_boot_time:
        if oGantry.update_gantry_data():
            time_out_timer = time.time()  # reset time out
            gantry_msg_counter += 1
        if (time.time() - time_out_timer) > time_out_sec:
            warnings.warn("[ROS_GANTRY] Serial link to gantry time out! Shutting Down")
            stop_everything()
            return False
        time.sleep(0.01)  # avoid looping too fast

    print("[ROS_Gantry] Booting completed: receiving gantry data @" +
          str(round(gantry_msg_counter / (time.time() - time_boot), 1)) + "Hz")

    print("[ROS_Gantry] Running stable since: " + time.ctime())
    return True


def stop_everything():
    oGantry.target_velocity(np.array([0, 0, 0]))


def service_stop_all(data):
    global stop_all
    print("[ROS_Gantry] Gantry Stopped = ", data.data)
    stop_all = data.data

    if stop_all:
        stop_everything()
        return stop_gantryResponse("[ROS_Gantry] Gantry Stopped")
    else:
        return stop_gantryResponse("[ROS_Gantry] Gantry Moves")


def service_initialize_home(data):
    global initialize_home
    initialize_home = True
    print("[ROS_Gantry] Init Home Position")
    oGantry.start_home_seq()
    return init_homeResponse("[ROS_Gantry] process homing sequence...")


def service_start_realtime_mode(data):
    global stop_all, move_with_velocity, move_to_rel_pos, initialize_home, move_to_absolute_position, flag_real_time_mode
    oGantry.target_velocity(np.array([0, 0, 0]))
    stop_all = move_with_velocity = move_to_rel_pos = initialize_home = move_to_absolute_position = False
    flag_real_time_mode = True
    return real_time_modeResponse("[ROS_Gantry] Starting Real Time Mode")


def service_set_max_velocity(max_vel_ms):
    oGantry.set_max_speed_ms(np.array([max_vel_ms.x, max_vel_ms.y, max_vel_ms.z]))
    return max_velocityResponse("[ROS_Gantry] Max Velocity Set X=" + str(max_vel_ms.x) +
                                "m/s Y=" + str(max_vel_ms.y) +
                                "m/s Z=" + str(max_vel_ms.z))


def service_move_to_absolute_position_m(abs_pos_des_m):
    global stop_all, flag_move_to_abs_pos, abs_pos_des_glob_mm
    if stop_all:
        print("[ROS_GANTRY] STOP BUTTON PRESSED IGNORING COMMAND")
        return move_to_abs_posResponse("[ROS_GANTRY] STOP BUTTON PRESSED IGNORING COMMAND")
    else:
        flag_move_to_abs_pos = True
        oGantry.goto_position(np.array([abs_pos_des_m.x, abs_pos_des_m.y, abs_pos_des_m.z]))
        abs_pos_des_glob_mm = np.array([abs_pos_des_m.x * 1000, abs_pos_des_m.y * 1000, abs_pos_des_m.z * 1000])
        return move_to_abs_posResponse(
            "[ROS_Gantry] Moving to position X=" + str(abs_pos_des_m.x) +
            " Y=" + str(abs_pos_des_m.y) +
            " Z=" + str(abs_pos_des_m.z))


def service_move_to_relative_pos_m(rel_pos_m):
    global stop_all, flag_move_to_rel_pos, rel_pos_des_glob_mm

    if stop_all:
        print("[ROS_Gantry] STOP BUTTON PRESSED IGNORING COMMAND")
        return move_to_relative_posResponse("[ROS_GANTRY] STOP BUTTON PRESSED IGNORING COMMAND")
    else:
        oGantry.gorel_position(np.array([rel_pos_m.x, rel_pos_m.y, rel_pos_m.z]))  # in meter
        flag_move_to_rel_pos = True
        rel_pos_des_glob_mm = np.array([rel_pos_m.x * 1000, rel_pos_m.y * 1000, rel_pos_m.z * 1000])
        return move_to_relative_posResponse("[ROS_GANTRY] Moving to relative Position")


def service_move_with_velocity(des_vel_ms):
    global flag_move_with_velocity, stop_all
    if stop_all:
        print("[ROS_GANTRY] STOP BUTTON PRESSED IGNORING COMMAND")
        return move_with_velResponse("STOP BUTTON PRESSED IGNORING COMMAND")
    else:
        # TODO MAX VELOCITY MISSING
        flag_move_with_velocity = True
        print("[ROS_Gantry] Moving with velocity of X=" + str(des_vel_ms.vx) +
              " Y=" + str(des_vel_ms.vy) + " Z=" + str(des_vel_ms.vz))
        oGantry.target_velocity(np.array([des_vel_ms.vx, des_vel_ms.vy, des_vel_ms.vz]))

        return move_with_velResponse("[ROS_Gantry] Moving with velocity of X=" + str(round(des_vel_ms.vx, 3)) +
                                     "m/s Y=" + str(round(des_vel_ms.vy, 3)) +
                                     "m/s Z=" + str(round(des_vel_ms.vz, 3)) + "m/s")


def motor_control_publisher():
    global oGantry, stop_all, flag_move_to_abs_pos, flag_move_to_rel_pos, flag_move_with_velocity, initialize_home
    global abs_pos_des_glob_mm, rel_pos_des_glob_mm
    abs_pos_des_glob_mm_old = abs_pos_des_glob_mm
    pub = rospy.Publisher('/gantry/current_position', gantry, queue_size=10)
    reached = False
    rospy_rate = 30
    rate = rospy.Rate(rospy_rate)
    # print("[ROS_Gantry] ROS publisher running at " + str(rospy_rate) + "Hz")
    #TODO muss nachgucken was da gesendet werden soll oGantry.set_max_speed_ms(np.array([10, 10, 10]))#max velocity in the beginning(1/2)
    time_out_sec = 0.5
    time_out_timer = time.time()
    while not rospy.is_shutdown():

        if oGantry.update_gantry_data():
            time_out_timer = time.time()  # reset time out
        if (time.time() - time_out_timer) > time_out_sec:
            warnings.warn("[ROS_Gantry] Serial link to gantry time out! Shutting Down")
            stop_everything()
            break

        gantry_pos_m = oGantry.get_position_m()
        # print(gantry_pos_m)
        gantry_vel_ms = oGantry.get_velocity_ms()
        gantry_pos_mm = gantry_pos_m * 1000

        if flag_move_to_abs_pos:
            if np.linalg.norm(gantry_pos_mm - abs_pos_des_glob_mm) < WHEN_REACHED_DISTANCE:
                reached = True
                flag_move_to_abs_pos = False
                print("[ROS_Gantry] reached target position")
            else:
                reached = False

        if flag_real_time_mode and not stop_all:
            if not abs_pos_des_glob_mm_old == abs_pos_des_glob_mm:
                oGantry.goto_position(abs_pos_des_glob_mm / 1000)
            if np.linalg.norm(gantry_vel_ms) < 0.001:
                oGantry.goto_position(abs_pos_des_glob_mm / 1000)
            # print("dist " + str(np.linalg.norm(gantry_pos_mm - abs_pos_des_glob_mm)))
            if np.linalg.norm(gantry_pos_mm - abs_pos_des_glob_mm) < WHEN_REACHED_DISTANCE:
                reached = True
                # oGantry.goto_position(abs_pos_des_glob_mm / 1000)
                # print("reached real-time target position")
            else:

                reached = False

        send_point = gantry()
        send_point.header.stamp = rospy.Time.now()
        send_point.pos_gantry.x = gantry_pos_m[0]
        send_point.pos_gantry.y = gantry_pos_m[1]
        send_point.pos_gantry.z = gantry_pos_m[2]
        send_point.vel_gantry.linear.x = gantry_vel_ms[0]
        send_point.vel_gantry.linear.y = gantry_vel_ms[1]
        send_point.vel_gantry.linear.z = gantry_vel_ms[2]

        send_point.reached = reached
        pub.publish(send_point)
        rate.sleep()

    print("[ROS_Gantry] rospy shutdow...")
    return True


def get_desired_pos(abs_pos_des_m):
    global abs_pos_des_glob_mm
    abs_pos_des_glob_mm = np.array([abs_pos_des_m.pos_gantry.x * 1000,
                                    abs_pos_des_m.pos_gantry.y * 1000,
                                    abs_pos_des_m.pos_gantry.z * 1000])
    return True


if __name__ == '__main__':
    print("[ROS_Gantry] start")

    rospy.init_node('Gantry', anonymous=True)
    # Start all services
    rospy.Service('/gantry/stop_all', stop_gantry, service_stop_all)
    rospy.Service('/gantry/init_home', init_home, service_initialize_home)
    rospy.Service('/gantry/rel_pos', move_to_relative_pos, service_move_to_relative_pos_m)
    rospy.Service('/gantry/realtime_mode', real_time_mode, service_start_realtime_mode)
    rospy.Service('/gantry/abs_pos', move_to_abs_pos, service_move_to_absolute_position_m)
    rospy.Service('/gantry/velocity_direct', move_with_vel, service_move_with_velocity)
    rospy.Service('/gantry/max_velocity', max_velocity, service_set_max_velocity)
    # TODO create service for max velocity
    # Start all subscribers
    rospy.Subscriber("/gantry/position_des", gantry, get_desired_pos)
    # Start Gantry
    oGantry = sc.GantryCommunication('/dev/ttyS0', 'teensy_40', 57600)
    if init_system():  # opens serial port to teensy
        # Start all publishers
        try:
            motor_control_publisher()
        except rospy.ROSInterruptException:
            pass
        stop_everything()
