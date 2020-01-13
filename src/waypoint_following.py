import numpy as np
import Tkinter
import rospy
from gantry_control_ros.msg import gantry
import tkFileDialog
import time

reached = None
gantry_pos = None


def save_as_dialog(windowtitle='Save as...', myFormats=[('Text file', '*.txt')]):
    root = Tkinter.Tk()
    root.withdraw()  # get rid of the tk-app window in the background
    filename = tkFileDialog.asksaveasfilename(parent=root, filetypes=myFormats, title=windowtitle)
    if len(filename) > 0:
        print ('Now saving under ' + filename)
    return filename


def select_file(myFormats=[('Text file', '*.txt')]):
    root = Tkinter.Tk()
    root.withdraw()  # get rid of the tk-app window in the background
    filename = tkFileDialog.askopenfilename(parent=root, filetypes=myFormats, title='Choose a file')
    if filename is not None:
        print ('You chose ' + filename)

    #    data = file.read()
    #    file.close()
    #    print "I got %d bytes from this file." % len(data)
    return filename


def move_to_position_ros(pub, current_target):
    msg_gantry = gantry()
    msg_gantry.header.stamp = rospy.Time.now()
    msg_gantry.pos_gantry.x = current_target[0]
    msg_gantry.pos_gantry.y = current_target[1]
    msg_gantry.pos_gantry.z = current_target[2]
    pub.publish(msg_gantry)


def follow_wp_and_take_measurements():  # (self, start_wp=[1000, 1000], sample_size=32):
    wplist_filename = select_file()
    print(wplist_filename)
    wp_append_list=[]

    with open(wplist_filename, 'r') as wpfile:
        load_description = True
        load_grid_settings = False
        load_wplist = False

        for i, line in enumerate(wpfile):

            if line == '### begin grid settings\n':
                print('griddata found')
                load_description = False
                load_grid_settings = True
                load_wplist = False
                continue
            elif line == '### begin wp_list\n':
                load_description = False
                load_grid_settings = False
                load_wplist = True
                print('### found')
                continue
            if load_description:
                print('file description')
                print(line)

            if load_grid_settings and not load_wplist:
                grid_settings = map(float, line.split(' '))
                x0 = [grid_settings[0], grid_settings[1], grid_settings[2]]
                xn = [grid_settings[3], grid_settings[4], grid_settings[5]]
                grid_dxdyda = [grid_settings[6], grid_settings[7], grid_settings[8]]
                timemeas = grid_settings[9]

                data_shape = []
                for i in range(3):  # range(num_dof)
                    try:
                        shapei = int((xn[i] - x0[i]) / grid_dxdyda[i] + i)
                    except ZeroDivisionError:
                        shapei = 1
                    data_shape.append(shapei)
            if load_wplist and not load_grid_settings:
                # print('read wplist')
                wp_append_list.append(map(float, line[:-2].split(' ')))
        wpfile.close()


    wp_list = np.asarray(wp_append_list)  # array: number x y z time
    num_wp = len(wp_list)
    print('Number of way points: ' + str(num_wp))
    start_time = time.time()
    start_position = np.array([wp_list[1], wp_list[2], wp_list[3]])

    rate = rospy.Rate(30)
    pub = rospy.Publisher('/gantry/position_des', gantry, queue_size=10)
    move_to_position_ros(pub, start_position)

    # start
    global reached, gantry_pos
    rospy.Subscriber("/gantry/current_position", gantry, callback)
    reached = False  # not robust solution
    for wp in wp_list:
        current_target_m = np.array([round(wp[1]/1000,3), round(wp[2]/1000,3), round(wp[3]/1000,3)])
        print("next wp = " + str(current_target_m))

        while not reached or np.linalg.norm(current_target_m - gantry_pos) > 0.002:

            move_to_position_ros(pub, current_target_m)
            rate.sleep()

        time.sleep(wp[4])

    return True


def callback(data):
    global reached, gantry_pos
    reached = data.reached
    gantry_pos = np.array([data.pos_gantry.x,
                           data.pos_gantry.y,
                           data.pos_gantry.z])

if __name__ == '__main__':
    rospy.init_node('Waypoint_Driver', anonymous=True)
    follow_wp_and_take_measurements()
