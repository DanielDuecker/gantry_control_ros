# coding=utf-8
import Tkinter as Tk
import ttk
# import gantry_control
import numpy as np
import time as t
import threading as Thread
from gantry_control_ros.srv import init_home, move_to_abs_pos, move_to_relative_pos, move_with_vel, stop_gantry, real_time_mode, max_velocity
from gantry_control_ros.msg import gantry
import rospy
rospy.init_node('gantry_gui', anonymous=True)
# from std_msgs.msg import Bool
# from gantry_msgs.msg import Gantry

LARGE_FONT = ('Tahoma', 12)
SUPERLARGE_FONT = ('Tahoma', 30)
SUPERLARGE_FONT_FOR_START = ('Tahoma', 20)

"""
only gantry GUI
no longer measurements and WP following
"""

"""
Für Tim: 
Am besten schreibst du dir einen ähnlichen Part mit den gleichen Variablennamen um die GUI in dein ROS Skript  einzubinden
"""


class GantryControllerObj(object):
    def __init__(self):
        self.current_v_x = 0
        self.current_v_y = 0
        self.current_v_z = 0
        self.position_gantry_x = 0
        self.position_gantry_y = 0
        self.position_gantry_z = 0
        self.max_v_x = 1000
        self.max_v_y = 3000
        self.max_v_z = 101
        self.stop_all__proxy = rospy.ServiceProxy('/gantry/stop_all', stop_gantry)
        self.init_home_proxy = rospy.ServiceProxy('/gantry/init_home', init_home)
        self.rel_pos_proxy = rospy.ServiceProxy('/gantry/rel_pos', move_to_relative_pos)
        self.realtime_mode_proxy = rospy.ServiceProxy('/gantry/realtime_mode', real_time_mode)
        self.abs_pos_proxy = rospy.ServiceProxy('/gantry/abs_pos', move_to_abs_pos)
        self.vel_direct_proxy = rospy.ServiceProxy('/gantry/velocity_direct', move_with_vel)
        self.max_velocity_proxy = rospy.ServiceProxy('/gantry/max_velocity', max_velocity)
        self.sub = rospy.Subscriber("/gantry/current_position", gantry, self.get_position_gantry_sub)

    def get_position_gantry_sub(self, data):
        self.position_gantry_x = data.pos_gantry.x
        self.position_gantry_y = data.pos_gantry.y
        self.position_gantry_z = data.pos_gantry.z

    def set_max_velocity(self):
        print(self.max_velocity_proxy(float(self.max_v_x),float(self.max_v_y),float(self.max_v_z)))

    def get_position_gantry_xyz(self):
        return self.position_gantry_x, self.position_gantry_y, self.position_gantry_z

    def stop_all_service(self):
        print(self.stop_all__proxy(True))
        return

    def start_all_service(self):
        print(self.stop_all__proxy(False))
        return

    def real_time_mode_service(self):
        print(self.realtime_mode_proxy(True))
        return

    def init_home_service(self):
        print(self.init_home_proxy(True))
        return

    def move_to_abs_pos(self, x, y, z):
        print(self.abs_pos_proxy(float(x), float(y), float(z)))
        # print(x,y,z)
        return

    def move_to_relative_pos(self, x, y, z):
        print(self.rel_pos_proxy(float(x), float(y), float(z)))
        return

    def move_with_vel_x(self, x):
        self.current_v_x = x
        self.move_with_vel()
        return

    def move_with_vel_y(self, y):
        self.current_v_y = y
        self.move_with_vel()
        return

    def move_with_vel_z(self, z):
        self.current_v_z = z
        self.move_with_vel()
        return

    def move_with_vel(self):
        print(self.vel_direct_proxy(self.current_v_x, self.current_v_y, self.current_v_z))
        return

    def print_sth(self, data):
        print(data)
        return

    def max_velocity_x(self, x_d):
        self.max_v_x = x_d
        self.set_max_velocity()
        return

    def max_velocity_y(self, y_d):
        self.max_v_y = y_d

        self.set_max_velocity()
        return

    def max_velocity_z(self, z_d):
        self.max_v_z = z_d
        self.set_max_velocity()
        return
        # def max_velocity(self):#TODO MAX VELOCITY MISSING
        #     self.init_home_proxy(True)
        #     return


class GantryGui(Tk.Tk):
    def __init__(self, *args, **kwargs):
        Tk.Tk.__init__(self, *args, **kwargs)

        Tk.Tk.wm_title(self, 'Gantry Control ROS')
        container = Tk.Frame(self)
        container.pack()

        container.pack(side='top', fill='both', expand=True)

        container.rowconfigure(0, weight=1)
        container.columnconfigure(0, weight=1)

        gantrycontroller = GantryControllerObj()

        self.frames = {}
        for F in (StartPage, PageOne):
            frame = F(container, self, gantrycontroller)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky='nsew')

        self.show_frame(StartPage)

        self.frames[StartPage].get_position()

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()  # raise frame to the front


class StartPage(Tk.Frame):
    def __init__(self, parent, controller, gantrycontroller):
        Tk.Frame.__init__(self, parent)
        self.communication_ROS = GantryControllerObj()

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

        # def get_position(obj):
        #     pos_x_mm, pos_y_mm, pos_z_rad = 0,1,1
        #     obj.__label_pos_xyz.configure(
        #         text='X = ' + str(int(pos_x_mm)) + ' mm \nY = ' + str(int(pos_y_mm)) + ' mm \nA = ' + str(
        #             round(float(pos_z_rad), 4)) + ' mm' + str(round(t.time(), 2)))
        #     return True



        """
        Belt-Drive (x axix)
        """
        firstrow_belt = 2

        label_spindle_name = ttk.Label(velcontrl_frame, text='X-drive', font=LARGE_FONT)
        label_spindle_name.grid(row=firstrow_belt + 0, column=1)

        button3 = Tk.Button(velcontrl_frame, text='<-- V [-]', activebackground='green',
                            command=lambda: self.communication_ROS.move_with_vel_x(-1 * int(entry_v_x.get())))
        button3.grid(row=firstrow_belt + 1, column=0)

        button4 = Tk.Button(velcontrl_frame, text='STOP', fg='red', activeforeground='black', activebackground='red',
                            command=lambda: self.communication_ROS.move_with_vel_x(0))
        button4.grid(row=firstrow_belt + 1, column=1)

        button5 = Tk.Button(velcontrl_frame, text='[+] V -->', activebackground='green',
                            command=lambda: self.communication_ROS.move_with_vel_x(1 * int(entry_v_x.get())))
        button5.grid(row=firstrow_belt + 1, column=2)

        label_v_belt = ttk.Label(velcontrl_frame, text='Velocity:')
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
                            command=lambda: self.communication_ROS.move_with_vel_y(-1 * int(entry_v_y.get())))
        button2.grid(row=firstrow_spindle + 1, column=0)

        button3 = Tk.Button(velcontrl_frame, text='STOP', fg='red', activeforeground='black', activebackground='red',
                            command=lambda: self.communication_ROS.move_with_vel_y(0))
        button3.grid(row=firstrow_spindle + 1, column=1)

        button4 = Tk.Button(velcontrl_frame, text='[+] V -->', activebackground='green',
                            command=lambda: self.communication_ROS.move_with_vel_y(1 * int(entry_v_y.get())))
        button4.grid(row=firstrow_spindle + 1, column=2)

        label_v_spindle = ttk.Label(velcontrl_frame, text='Velocity:')
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
                            command=lambda: self.communication_ROS.move_with_vel_z(-1 * int(entry_v_z.get())))
        button2.grid(row=firstrow_rod + 1, column=0)

        button3 = Tk.Button(velcontrl_frame, text='STOP', fg='red', activeforeground='black', activebackground='red',
                            command=lambda: self.communication_ROS.move_with_vel_z(0))
        button3.grid(row=firstrow_rod + 1, column=1)

        button4 = Tk.Button(velcontrl_frame, text='[+] V -->', activebackground='green',
                            command=lambda: self.communication_ROS.move_with_vel_z(1 * int(entry_v_z.get())))
        button4.grid(row=firstrow_rod + 1, column=2)

        label_v_rod = ttk.Label(velcontrl_frame, text='Velocity:')
        entry_v_z = ttk.Entry(velcontrl_frame)
        entry_v_z.insert(0, '0')

        label_v_rod.grid(row=firstrow_rod + 2, column=0)
        entry_v_z.grid(row=firstrow_rod + 2, column=1)

        """
        Abs Postion control
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
                                         command=lambda: self.communication_ROS.move_to_abs_pos(
                                             entry_abs_pos_belt.get(), entry_abs_pos_spindle.get(),
                                             entry_abs_pos_rod.get()))
        button_goto_abs_pos.grid(row=5, column=1, sticky='W', pady=4)

        """
        Rel Postion control
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
                                         command=lambda: self.communication_ROS.move_to_relative_pos(entry_rel_pos_belt.get(),entry_rel_pos_spindle.get(),entry_rel_pos_rod.get()))
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
                                     command=lambda: self.communication_ROS.init_home_service())
        button_home_seq.grid(row=0, column=1, sticky='W', padx=10, pady=4)

        """
        Real Time Mode Button
        """
        button_real_time_mode = ttk.Button(self, text='Start Real Time Mode',
                                           command=lambda: self.communication_ROS.real_time_mode_service())
        button_real_time_mode.grid(row=0, column=2, sticky='W', padx=10, pady=4)

        """
        Emergency-Stop-Button
        """
        button_stop = Tk.Button(self, text='STOP', width=6, command=lambda: (
            self.communication_ROS.stop_all_service()), background='#ff7070',
                                activebackground='red', highlightbackground="black", font=SUPERLARGE_FONT)
        button_stop.grid(row=4, column=0, sticky='W', ipady=30)

        """
        Start - Button
        """
        button_stop = Tk.Button(self, text='START', width=6, command=lambda: (
            self.communication_ROS.start_all_service()), activebackground='green', activeforeground='black',
                                highlightbackground="black", fg='green', font=SUPERLARGE_FONT)
        button_stop.grid(row=5, column=0, sticky='W')

    def get_position(self):
        pos_x_mm, pos_y_mm, pos_z_rad = self.communication_ROS.get_position_gantry_xyz()  # TODO ROS EINFÜGEN
        self.__label_pos_xyz.configure(
            text='X = ' + str(int(pos_x_mm)) + ' mm \nY = ' + str(int(pos_y_mm)) + ' mm \nA = ' + str(
                round(float(pos_z_rad), 4)) + ' mm')
        self.__label_pos_xyz.after(200, self.get_position)  # update position every 1000ms

        return True


class PageOne(Tk.Frame):
    def __init__(self, parent, controller, gantrycontroller):
        Tk.Frame.__init__(self, parent)
        self.communication_ROS = GantryControllerObj()
        label = ttk.Label(self, text='Drive Settings', font=LARGE_FONT)
        label.grid(row=1, column=1, pady=10, padx=10)

        button1 = ttk.Button(self, text='-> back to Controller Menu', command=lambda: controller.show_frame(StartPage))
        button1.grid(row=1, column=3)

        """
        Settings
        """
        entry_max_speed_belt = ttk.Entry(self)
        entry_max_speed_belt.insert(0, '1000')
        entry_max_speed_belt.grid(row=3, column=1, padx=10)
        button_max_speed_belt = ttk.Button(self, text='set max Speed X axis (<=3000!)',
                                           command=lambda: self.communication_ROS.max_velocity_x(entry_max_speed_belt.get()))
        button_max_speed_belt.grid(row=3, column=2, sticky='W', pady=4)

        entry_max_speed_spindle = ttk.Entry(self)
        entry_max_speed_spindle.insert(0, '3000')
        entry_max_speed_spindle.grid(row=4, column=1, padx=10)
        button_max_speed_spindle = ttk.Button(self, text='set max Speed Y axis (<=9000!)',
                                              command=lambda: self.communication_ROS.max_velocity_y(entry_max_speed_spindle.get()))
        button_max_speed_spindle.grid(row=4, column=2, sticky='W', pady=4)

        entry_max_speed_rod = ttk.Entry(self)
        entry_max_speed_rod.insert(0, '101')
        entry_max_speed_rod.grid(row=5, column=1, padx=10)
        button_max_speed_rod = ttk.Button(self, text='set max Speed Z axis (<=101!)',
                                          command=lambda: self.communication_ROS.max_velocity_z(entry_max_speed_rod.get()))
        button_max_speed_rod.grid(row=5, column=2, sticky='W', pady=4)


app = GantryGui()

app.mainloop()
