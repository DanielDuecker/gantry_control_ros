"""
gantry_gui requires a running "ROS_Gantry.py" on the gantry-desktop computer

ROS_Gantry.py provides the ros-gantry-control services.

all ros communication is in m, m/s, however gantry uses mm, mm/s since this avoids typing "."

January 2020
"""


# coding=utf-8
import Tkinter as Tk
import ttk
import numpy as np
import time as t

from gantry_control_ros.srv import init_home, move_to_abs_pos, move_to_relative_pos, move_with_vel, stop_gantry, real_time_mode, max_velocity
from gantry_control_ros.msg import gantry
import rospy
rospy.init_node('gantry_gui', anonymous=True)
# from std_msgs.msg import Bool
# from gantry_msgs.msg import Gantry

# import hippocampus_toolbox as hc_tools

LARGE_FONT = ('Tahoma', 12)
SUPERLARGE_FONT = ('Tahoma', 30)
SUPERLARGE_FONT_FOR_START = ('Tahoma', 20)
import Tkinter
import tkFileDialog
import tkSimpleDialog


# ======== "Save as" dialog:
def save_as_dialog(windowtitle='Save as...', myFormats=[('Text file', '*.txt')]):

    root = Tkinter.Tk()
    root.withdraw()  # get rid of the tk-app window in the background
    filename = tkFileDialog.asksaveasfilename(parent=root, filetypes=myFormats, title=windowtitle)
    if len(filename) > 0:
        print ('Now saving under ' + filename)
    return filename


# ======== Select a directory:
def select_directory():
    root = Tkinter.Tk()
    root.withdraw()  # get rid of the tk-app window in the background
    dirname = tkFileDialog.askdirectory(parent=root,initialdir="/",title='Please select a directory')
    if len(dirname) > 0:
        print ('You chose ' + dirname)
    return dirname


# ======== Select a file for opening:
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


# ======== Select a directory:
def write_descrition():
    root = Tkinter.Tk()
    root.withdraw()  # get rid of the tk-app window in the background
    textinput = tkSimpleDialog.askstring("Description prompt", "enter your description")
    if textinput is not None:
        print ('You chose: ' + textinput)
    return textinput




class GantryControlROS(object):
    def __init__(self):

        self.__position_m = np.array([0.0, 0.0, 0.0])  # m
        self.__velocity_ms = np.array([0.0, 0.0, 0.0])

        self.__max_speed_mms = np.array([200, 250, 70])  # mm/s

        """ 
        Service Proxys
        """
        self.stop_all_proxy = rospy.ServiceProxy('/gantry/stop_all', stop_gantry)
        self.init_home_proxy = rospy.ServiceProxy('/gantry/init_home', init_home)
        self.rel_pos_proxy = rospy.ServiceProxy('/gantry/rel_pos', move_to_relative_pos)
        self.realtime_mode_proxy = rospy.ServiceProxy('/gantry/realtime_mode', real_time_mode)
        self.abs_pos_proxy = rospy.ServiceProxy('/gantry/abs_pos', move_to_abs_pos)
        self.vel_direct_proxy = rospy.ServiceProxy('/gantry/velocity_direct', move_with_vel)
        self.max_velocity_proxy = rospy.ServiceProxy('/gantry/max_velocity', max_velocity)

        self.sub_position = rospy.Subscriber("/gantry/current_position", gantry, self.get_position_gantry_sub)

    def get_position_gantry_sub(self, data):
        self.__position_m = np.array([data.pos_gantry.x,
                                    data.pos_gantry.y,
                                    data.pos_gantry.z])

    def set_max_velocity_mms(self):
        # all ROS information is in m/s
        print(self.max_velocity_proxy(float(self.__max_speed_mms[0])/1000,
                                      float(self.__max_speed_mms[1])/1000,
                                      float(self.__max_speed_mms[2])/1000))

    def get_position_gantry_xyz(self):
        return self.__position_m

    def stop_all_service(self):
        print(self.stop_all_proxy(True))
        return True

    def start_all_service(self):
        print(self.stop_all_proxy(False))
        return True

    def real_time_mode_service(self):
        print(self.realtime_mode_proxy(True))
        return True

    def init_home_service(self):
        print(self.init_home_proxy(True))
        return True

    def move_to_abs_pos(self, x_m, y_m, z_m):
        print(self.abs_pos_proxy(float(x_m), float(y_m), float(z_m)))
        print("GUI: move_to_abs_pos: " + str(x_m) + "," + str(y_m) + "," + str(z_m))
        return True

    def move_to_relative_pos(self, dx_m, dy_m, dz_m):
        print(self.rel_pos_proxy(float(dx_m), float(dy_m), float(dz_m)))
        return True

    def move_with_vel_mms_x(self, vx_mms):
        self.__velocity_ms[0] = float(vx_mms)/1000  # update vx
        print("velocity_ms_0= " + str(float(vx_mms)/1000))
        self.start_moving_with_vel()
        return True

    def move_with_vel_mms_y(self, vy_mms):
        self.__velocity_ms[1] = float(vy_mms)/1000  # update vy
        self.start_moving_with_vel()
        return True

    def move_with_vel_mms_z(self, vz_mms):
        self.__velocity_ms[2] = float(vz_mms)/1000  # update vz
        self.start_moving_with_vel()
        return True

    def start_moving_with_vel(self):
        # velocity in mm/s
        #print(self.__velocity_ms)
        print(self.vel_direct_proxy(self.__velocity_ms[0], self.__velocity_ms[1], self.__velocity_ms[2]))
        return True

    def print_sth(self, data):
        print(data)
        return

    def max_velocity_mms_x(self, vx_max_mms):
        self.__max_speed_mms[0] = vx_max_mms
        self.set_max_velocity_mms()
        return

    def max_velocity_mms_y(self, vy_max_mms):
        self.__max_speed_mms[1] = vy_max_mms
        self.set_max_velocity_mms()
        return

    def max_velocity_mms_z(self, vz_max_mms):
        self.__max_speed[2] = vz_max_mms
        self.set_max_velocity_mms()
        return

    def start_waypoint_following(self):
        measdata_filename = select_file()



class GantryGui(Tk.Tk):
    def __init__(self, *args, **kwargs):
        Tk.Tk.__init__(self, *args, **kwargs)

        Tk.Tk.wm_title(self, 'Gantry Control ROS')
        container = Tk.Frame(self)
        container.pack()

        container.pack(side='top', fill='both', expand=True)

        container.rowconfigure(0, weight=1)
        container.columnconfigure(0, weight=1)

        gantrycontroller = GantryControlROS()

        self.frames = {}
        for F in (StartPage, PageOne):
            frame = F(container, self, gantrycontroller)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky='nsew')

        self.show_frame(StartPage)

        self.frames[StartPage].get_position()  # update position label

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()  # raise frame to the front


class StartPage(Tk.Frame):
    def __init__(self, parent, controller, gantry_interface):
        Tk.Frame.__init__(self, parent)
        self.gantry_interface = gantry_interface # GantryControllerObj()

        # Notebook
        # notebook_label = ttk.Label(self, text="Control")
        # notebook_label.grid(row=3, column=2, pady=3)

        notebook_frame = ttk.Notebook(self)
        notebook_frame.grid(row=4, column=1, padx=30, pady=4)

        velcontrl_frame = ttk.Frame(notebook_frame)
        absposcontrl_frame = ttk.Frame(notebook_frame)
        relposcontrl_frame = ttk.Frame(notebook_frame)
        man_contrl_frame = ttk.Frame(notebook_frame)

        notebook_frame.add(velcontrl_frame, text="Velocity Control")
        notebook_frame.add(absposcontrl_frame, text="Abs Position Control")
        notebook_frame.add(relposcontrl_frame, text="Rel Position Control")

        self.__label_pos_xyz = ttk.Label(self, text='X = ? mm\nY = ? mm\nZ = ? mm')
        self.__label_pos_xyz.grid(row=0, column=0)

        """
        Belt-Drive (x axis)
        """
        firstrow_belt = 2

        label_spindle_name = ttk.Label(velcontrl_frame, text='X-drive', font=LARGE_FONT)
        label_spindle_name.grid(row=firstrow_belt + 0, column=1)

        button3 = Tk.Button(velcontrl_frame, text='<-- V [-]', activebackground='green',
                            command=lambda: self.gantry_interface.move_with_vel_mms_x(-1 * int(entry_v_x.get())))
        button3.grid(row=firstrow_belt + 1, column=0)

        button4 = Tk.Button(velcontrl_frame, text='STOP', fg='red', activeforeground='black', activebackground='red',
                            command=lambda: self.gantry_interface.move_with_vel_mms_x(0))
        button4.grid(row=firstrow_belt + 1, column=1)

        button5 = Tk.Button(velcontrl_frame, text='[+] V -->', activebackground='green',
                            command=lambda: self.gantry_interface.move_with_vel_mms_x(1 * int(entry_v_x.get())))
        button5.grid(row=firstrow_belt + 1, column=2)

        label_v_belt = ttk.Label(velcontrl_frame, text='Velocity [mm/s]:')
        entry_v_x = ttk.Entry(velcontrl_frame)
        entry_v_x.insert(0, '0')

        label_v_belt.grid(row=firstrow_belt + 2, column=0)
        entry_v_x.grid(row=firstrow_belt + 2, column=1)

        """
        Spindle-Drive (y axis)
        """
        firstrow_spindle = 5

        label_spindle_name = ttk.Label(velcontrl_frame, text='Y-drive', font=LARGE_FONT)
        label_spindle_name.grid(row=firstrow_spindle + 0, column=1)

        button2 = Tk.Button(velcontrl_frame, text='<-- V [-]', activebackground='green',
                            command=lambda: self.gantry_interface.move_with_vel_mms_y(-1 * int(entry_v_y.get())))
        button2.grid(row=firstrow_spindle + 1, column=0)

        button3 = Tk.Button(velcontrl_frame, text='STOP', fg='red', activeforeground='black', activebackground='red',
                            command=lambda: self.gantry_interface.move_with_vel_mms_y(0))
        button3.grid(row=firstrow_spindle + 1, column=1)

        button4 = Tk.Button(velcontrl_frame, text='[+] V -->', activebackground='green',
                            command=lambda: self.gantry_interface.move_with_vel_mms_y(1 * int(entry_v_y.get())))
        button4.grid(row=firstrow_spindle + 1, column=2)

        label_v_spindle = ttk.Label(velcontrl_frame, text='Velocity [mm/s]:')
        entry_v_y = ttk.Entry(velcontrl_frame)
        entry_v_y.insert(0, '0')

        label_v_spindle.grid(row=firstrow_spindle + 2, column=0)
        entry_v_y.grid(row=firstrow_spindle + 2, column=1)

        """
        Threaded-rod-Drive (z axis)
        """
        firstrow_rod = 8

        label_rod_name = ttk.Label(velcontrl_frame, text='Z-drive', font=LARGE_FONT)
        label_rod_name.grid(row=firstrow_rod + 0, column=1)

        button2 = Tk.Button(velcontrl_frame, text='<-- V [-]', activebackground='green',
                            command=lambda: self.gantry_interface.move_with_vel_mms_z(-1 * int(entry_v_z.get())))
        button2.grid(row=firstrow_rod + 1, column=0)

        button3 = Tk.Button(velcontrl_frame, text='STOP', fg='red', activeforeground='black', activebackground='red',
                            command=lambda: self.gantry_interface.move_with_vel_mms_z(0))
        button3.grid(row=firstrow_rod + 1, column=1)

        button4 = Tk.Button(velcontrl_frame, text='[+] V -->', activebackground='green',
                            command=lambda: self.gantry_interface.move_with_vel_mms_z(1 * int(entry_v_z.get())))
        button4.grid(row=firstrow_rod + 1, column=2)

        label_v_rod = ttk.Label(velcontrl_frame, text='Velocity [mm/s]:')
        entry_v_z = ttk.Entry(velcontrl_frame)
        entry_v_z.insert(0, '0')

        label_v_rod.grid(row=firstrow_rod + 2, column=0)
        entry_v_z.grid(row=firstrow_rod + 2, column=1)

        """
        Absolute Position control
        """
        entry_abs_pos_belt = ttk.Entry(absposcontrl_frame)
        entry_abs_pos_belt.insert(0, '')
        entry_abs_pos_belt.grid(row=2, column=1)

        entry_abs_pos_spindle = ttk.Entry(absposcontrl_frame)
        entry_abs_pos_spindle.insert(0, '')
        entry_abs_pos_spindle.grid(row=3, column=1)

        entry_abs_pos_rod = ttk.Entry(absposcontrl_frame)
        entry_abs_pos_rod.insert(0, '')
        entry_abs_pos_rod.grid(row=4, column=1)

        button_goto_abs_pos = ttk.Button(absposcontrl_frame, text='go to X/Y/Z - pos [mm]',
                                         command=lambda: self.gantry_interface.move_to_abs_pos(
                                             float(entry_abs_pos_belt.get())/1000,
                                             float(entry_abs_pos_spindle.get())/1000,
                                             float(entry_abs_pos_rod.get())/1000))
        button_goto_abs_pos.grid(row=5, column=1, sticky='W', pady=4)

        """
        Relative Position control
        """
        entry_rel_pos_belt = ttk.Entry(relposcontrl_frame)
        entry_rel_pos_belt.insert(0, '0')
        entry_rel_pos_belt.grid(row=2, column=1)

        entry_rel_pos_spindle = ttk.Entry(relposcontrl_frame)
        entry_rel_pos_spindle.insert(0, '0')
        entry_rel_pos_spindle.grid(row=3, column=1)

        entry_rel_pos_rod = ttk.Entry(relposcontrl_frame)
        entry_rel_pos_rod.insert(0, '0')
        entry_rel_pos_rod.grid(row=4, column=1)

        button_goto_rel_pos = ttk.Button(relposcontrl_frame, text='move by dx dy dz [mm]',
                                         command=lambda: self.gantry_interface.move_to_relative_pos(
                                             float(entry_rel_pos_belt.get())/1000,
                                             float(entry_rel_pos_spindle.get())/1000,
                                             float(entry_rel_pos_rod.get())/1000))
        button_goto_rel_pos.grid(row=5, column=1, sticky='W', pady=4)

        """
        Quit-Button
        """
        button_quit = ttk.Button(self, text='Quit', command=self.quit)
        button_quit.grid(row=5, column=2, sticky='W', pady=4)

        """
        Drive Setting - Button
        """
        button1 = ttk.Button(self, text='Drive Settings', command=lambda: controller.show_frame(PageOne))
        button1.grid(row=5, column=1)

        """
        Initialize Home Position
        """
        button_home_seq = ttk.Button(self, text='Initialize Home Position',
                                     command=lambda: self.gantry_interface.init_home_service())
        button_home_seq.grid(row=0, column=1, sticky='W', padx=10, pady=4)

        """
        Real Time Mode Button
        """
        button_real_time_mode = ttk.Button(self, text='Start Real Time Mode',
                                           command=lambda: self.gantry_interface.real_time_mode_service())
        button_real_time_mode.grid(row=0, column=2, sticky='W', padx=10, pady=4)

        """
        Start Waypointfile - Button
        """
        button_real_time_mode = ttk.Button(self, text='Start Waypoint Following',
                                           command=lambda: self.gantry_interface.start_waypoint_following())
        button_real_time_mode.grid(row=1, column=2, sticky='W', padx=10, pady=4)

        """
        Emergency-Stop-Button
        """
        button_stop = Tk.Button(self, text='STOP', width=6, command=lambda: (
            self.gantry_interface.stop_all_service()), background='#ff7070',
                                activebackground='red', highlightbackground="black", font=SUPERLARGE_FONT)
        button_stop.grid(row=4, column=0, sticky='W', ipady=30)

        """
        Start - Button
        """
        button_stop = Tk.Button(self, text='START', width=6, command=lambda: (
            self.gantry_interface.start_all_service()), activebackground='green', activeforeground='black',
                                highlightbackground="black", fg='green', font=SUPERLARGE_FONT)
        button_stop.grid(row=5, column=0, sticky='W')

    def get_position(self):
        pos_x_mm, pos_y_mm, pos_z_mm = self.gantry_interface.get_position_gantry_xyz()  # TODO ROS EINFUEGEN
        self.__label_pos_xyz.configure(
            text='X = ' + str(int(pos_x_mm)) + ' mm \n'+
                 'Y = ' + str(int(pos_y_mm)) + ' mm \n'+
                 'Z = ' + str(int(pos_z_mm)) +  ' mm')
        self.__label_pos_xyz.after(200, self.get_position)  # update position label every 200ms
        return True


class PageOne(Tk.Frame):
    def __init__(self, parent, controller, gantry_interface):
        Tk.Frame.__init__(self, parent)
        self.gantry_interface = gantry_interface
        label = ttk.Label(self, text='Drive Settings', font=LARGE_FONT)
        label.grid(row=1, column=1, pady=10, padx=10)

        button1 = ttk.Button(self, text='-> back to Controller Menu', command=lambda: controller.show_frame(StartPage))
        button1.grid(row=1, column=3)

        """
        Settings
        """
        # max: max_speed_mms = np.array([200, 250, 70])  # mm/s

        entry_max_speed_belt = ttk.Entry(self)
        entry_max_speed_belt.insert(0, '200')
        entry_max_speed_belt.grid(row=3, column=1, padx=10)
        button_max_speed_belt = ttk.Button(self, text='set max Speed X axis (<=200mm/s!)',
                                           command=lambda: self.gantry_interface.max_velocity_mms_x(entry_max_speed_belt.get()))
        button_max_speed_belt.grid(row=3, column=2, sticky='W', pady=4)

        entry_max_speed_spindle = ttk.Entry(self)
        entry_max_speed_spindle.insert(0, '250')
        entry_max_speed_spindle.grid(row=4, column=1, padx=10)
        button_max_speed_spindle = ttk.Button(self, text='set max Speed Y axis (<=250 mm/s!)',
                                              command=lambda: self.gantry_interface.max_velocity_mms_y(entry_max_speed_spindle.get()))
        button_max_speed_spindle.grid(row=4, column=2, sticky='W', pady=4)

        entry_max_speed_rod = ttk.Entry(self)
        entry_max_speed_rod.insert(0, '70')
        entry_max_speed_rod.grid(row=5, column=1, padx=10)
        button_max_speed_rod = ttk.Button(self, text='set max Speed Z axis (<=70mm/s!)',
                                          command=lambda: self.gantry_interface.max_velocity_mms_z(entry_max_speed_rod.get()))
        button_max_speed_rod.grid(row=5, column=2, sticky='W', pady=4)


app = GantryGui()

app.mainloop()
