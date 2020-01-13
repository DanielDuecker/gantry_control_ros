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
    # self.start_RfEar()
    # self.__oRf.set_samplesize(sample_size)
    # sample_size = self.__oRf.get_samplesize()

    wplist_filename = select_file()
    print(wplist_filename)
    wp_append_list = []

    with open(wplist_filename, 'r') as wpfile:
        load_description = True
        load_grid_settings = False
        load_wplist = False
        wp_append_list = []
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

                # old: data_shape = [xn[0] / grid_dxdyda[0] + 1, xn[1] / grid_dxdyda[1] + 1, xn[2] / grid_dxdyda[2] + 1]

            if load_wplist and not load_grid_settings:
                # print('read wplist')
                wp_append_list.append(map(float, line[:-2].split(' ')))

        wp_data_mat = np.asarray(wp_append_list)
        # print(str(wp_data_mat))

        wpfile.close()
    # print(str(np.asarray(wp_append_list)))

    wp_list = np.asarray(wp_append_list)  # array: number x y z time
    num_wp = len(wp_list)
    print('Number of way points: ' + str(num_wp))
    start_time = time.time()
    start_position = np.array([wp_list[1], wp_list[2], wp_list[3]])

    rate = rospy.Rate(30)
    pub = rospy.Publisher('/gantry/position_des', gantry, queue_size=10)
    move_to_position_ros(pub, start_position)
    # wait 5 seconds
    # time.sleep(5)
    # start_moving_gantry_to_target(current_target)
    # print(current_target)

    # start
    global reached, gantry_pos
    rospy.Subscriber("/gantry/current_position", gantry, callback)
    reached = False  # not robust solution
    for wp in wp_list:
        current_target_m = np.array([wp[1]/1000, wp[2]/1000, wp[3]/1000])
        print("next wp = "+ str(current_target_m))

        while not reached or np.linalg.norm(current_target_m - gantry_pos) > 0.002:
            #pass
        # while reached:
        #     current_target = np.array([wp[1], wp[2], wp[3]])
        #     print('Moving to position = ' + str(current_target))
        #     move_to_position_ros(pub, current_target)
        #     time.sleep(0.2)

            move_to_position_ros(pub, current_target_m)
            rate.sleep()
        # while not reached:
        #     time.sleep(0.2)

        time.sleep(wp[4])

        # wait n seconds

        # take measurement

        # time_elapsed = time.time() - start_time
        # data_row = np.append([meas_counter, time_elapsed, pos_x_mm, pos_y_mm], pxx_den_max)
        # data_list.append(data_row)
        # wait for n seconds
        # start from above

        # meas_freq = meas_counter / time_elapsed
        # print('Logging with avg. ' + str(meas_freq) + ' Hz')

    # with open(measdata_filename, 'w') as measfile:
    #     measfile.write('Measurement file for trajectory following\n')
    #     measfile.write('Measurement was taken on ' + t.ctime() + '\n')
    #     measfile.write('### begin grid settings\n')
    #     measfile.write('sample size = ' + str(sample_size) + ' [*1024]\n')
    #     measfile.write('avg. meas frequency = ' + str(meas_freq) + ' Hz\n')
    #     measfile.write('start_point =' + str(start_wp) + '\n')
    #     measfile.write('wp_list =' + str(wp_list) + '\n')
    #     measfile.write('data format = [meas_counter, time_elapsed, pos_x_mm, pos_y_mm], pxx_den_max\n')
    #     measfile.write('### begin data log\n')
    #     data_mat = np.asarray(data_list)
    #     for row in data_mat:
    #         row_string = ''
    #         for i in range(len(row)):
    #             row_string += str(row[i]) + ','
    #         row_string += '\n'
    #         measfile.write(row_string)
    #
    #     measfile.close()

    return True


def callback(data):
    global reached, gantry_pos
    reached = data.reached
    gantry_pos = np.array([data.pos_gantry.x,
                           data.pos_gantry.y,
                           data.pos_gantry.z])

if __name__ == '__main__':
    rospy.init_node('Waypoint Driver', anonymous=True)
    follow_wp_and_take_measurements()
