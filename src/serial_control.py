import serial
import numpy as np


class GantryCommunication(object):
    def __init__(self, portname, name, baudrate):  #
        self.__oserial = []
        self.__portname = portname
        self.__name = name
        self.__isdummy = False
        # "isdummy = True" is for emulation of non connected drives -> gui can run with less drives attached
        self.__baudrate = baudrate
        self.__isopen = False
        self.__timewritewait = 0.02  # [s]
        self.__timereadwait = 0.02  # [s]

        self.__homeknown = False


        self.__manualinit = False
        self.__ismoving = False

        self.__gantry_pos_m = np.array([0, 0, 0])
        self.__gantry_vel_ms = np.array([0, 0, 0])

    """
        Gantry parameter on teensy
        
        incPERmm_x = 2000000/3100
        incPERmm_y = 945800/1600
        incPERmm_z = 1920612/940
        mmPERsPERrpm_x = 1000/28.2/500   # 1000mm/28.2s @  500rpm
        mmPERsPERrpm_y = 1000/35.4/1000  # 1000mm/35.4s @ 1000rpm
        mmPERsPERrpm_z = 940/38.2/3000   #  940mm/38.2s @ 3000rpm
        
        incPERmm = np.array([incPERmm_x, incPERmm_y, incPERmm_z])
        mmPERsPERrpm = np.array([mmPERsPERrpm_x, mmPERsPERrpm_y, mmPERsPERrpm_z])
        self.__gantry_param = [incPERmm, mmPERsPERrpm]
    """
    """
    def get_gantry_param(self):
        
        #incPERmm = np.array([incPERmm_x, incPERmm_y, incPERmm_z])
        #mmPERsPERrpm = np.array([mmPERsPERrpm_x, mmPERsPERrpm_y, mmPERsPERrpm_z])

        :return: [incPERmm, mmPERsPERrpm]
        
        return self.__gantry_param
    """

    def open_port(self):
        """
        :return:
        """
        # configure the serial connections (the parameters differs on the device you are connecting to)
        try:
            self.__oserial = serial.Serial(
                port=self.__portname,
                baudrate=self.__baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
        except serial.SerialException as err:
            self.__isdummy = True
            print('~~~~~~~~ Serial port ' + str(self.__name) + ' is not properly connected: it has been set as a dummy DOF')
            print('~~~~~~~~ (Error message: ' + str(err) + ')')
            print('~~~~~~~~ Please Check the Ports.')
        else:
            # open serial port
            self.__isopen = self.__oserial.isOpen()
            print('Serial port ' + self.__portname + ' is open!')
        return True

    def close_port(self):
        if not self.__isdummy:
            self.__oserial.close()
            self.__isopen = False
            print('Serial port ' + self.__portname + ' closed!')
        return True

    def write_on_port(self, motorcommand, arg1, arg2, arg3):

        str_arg1 = "%06d" % (arg1,)
        str_arg2 = "%06d" % (arg2,)
        str_arg3 = "%06d" % (arg3,)
        if len(str_arg1) > 6:
            print("motor command argument 1 is too long")
            print(str_arg1)
            return False
        if len(str_arg2) > 6:
            print("motor command argument 2 is too long")
            print(str_arg2)
            return False
        if len(str_arg3) > 6:
            print("motor command argument 3 is too long")
            print(str_arg3)
            return False

        command = {
            'enable': "a",
            'disable': "b",
            'setmaxspeed': "c",
            'gohomeseq': "d",
            'goto': "e",
            'gorel': "f",
            'tagetvelocity': "g"
        }
        print("write on port: " + motorcommand)
        str_command = command.get(motorcommand) + str_arg1 + str_arg2 + str_arg3

        if len(str_command) == 19:
            # print("command: " + str_command + " has length:" + str(len(str_command)))
            str_com_full = str_command+'\r\n'
            self.__oserial.write(str_com_full)
            print(str_com_full)
            return True
        else:
            print("motor command is too long")
            print(str_command)
            print("length = " + str(len(str_command)))
            return False

    def listen_to_port(self):
        out = ""
        while self.__oserial.inWaiting() > 0:
            new_data = self.__oserial.read(1)
            out += new_data  # pure number string

        out_split = out.rstrip().split('\r\n')
        last_string = out_split[-1].rstrip().split(',')
        if len(last_string) == 6:
            if last_string[-1] == '':
                return False
            else:
                x_pos_mm = int(last_string[0])
                x_vel_mms = int(last_string[1])
                y_pos_mm = int(last_string[2])
                y_vel_mms = int(last_string[3])
                z_pos_mm = int(last_string[4])
                z_vel_mms = int(last_string[5])

                self.__gantry_pos_m = np.array([float(x_pos_mm)/1000, float(y_pos_mm)/1000, float(z_pos_mm)/1000])
                self.__gantry_vel_ms = np.array([float(x_vel_mms)/1000, float(y_vel_mms)/1000, float(z_vel_mms)/1000])
                return True
        return False

    def update_gantry_data(self):
        if self.listen_to_port():
            if max(abs(self.__gantry_vel_ms)) < 5:
                self.__ismoving = False
            else:
                self.__ismoving = True
            return True
        else:
            return False

    def enable_drives(self):
        self.write_on_port('enable', 0, 0, 0)

    def disable_drives(self):
        self.write_on_port('disable', 0, 0, 0)

    def goto_position(self, target_pos_m):
        self.write_on_port('goto', int(target_pos_m[0]*1000),
                                   int(target_pos_m[1]*1000),
                                   int(target_pos_m[2]*1000))

    def gorel_position(self, move_rel_pos_m):
        self.write_on_port('gorel', int(move_rel_pos_m[0] * 1000),
                                    int(move_rel_pos_m[1] * 1000),
                                    int(move_rel_pos_m[2] * 1000))

    def target_velocity(self, target_vel_ms):
        self.write_on_port('tagetvelocity', int(target_vel_ms[0]*1000),
                                            int(target_vel_ms[1]*1000),
                                            int(target_vel_ms[2]*1000))

    def start_home_seq(self):
        self.write_on_port('gohomeseq', 0, 0, 0)

    def set_max_speed_ms(self, max_speed_ms):
        self.write_on_port('setmaxspeed', int(max_speed_ms[0]*1000),
                                          int(max_speed_ms[1]*1000),
                                          int(max_speed_ms[2]*1000))

    def get_position_m(self):
        return self.__gantry_pos_m

    def get_velocity_ms(self):
        return self.__gantry_vel_ms


"""
    def enter_manual_init_data(self):
        print('Do you want to enter extreme position manually? (yes/no)')
        input = raw_input("")
        if input == 'yes':
            print('Enter extreme position in [inc]-units:')
            input = raw_input("")
            self.__posincmax = int(input)
            self.__homeknown = True
            self.__extremeknown = True
            self.set_manual_init(True)
        return True

    def start_manual_mode(self, safetycheck=True):
        print ('Enter your commands below.')
        print ('Type "AUTO_MODE" for switching to AUTO_MODE')
        print ('Type "status" for a status report')
        print ('Type "exit" to leave manual mode')
        print ('Type "exitall" to close the application')
        running = True

        while running:
            # get keyboard input
            input = raw_input(">> ")
            # Python 3 users
            # input = input(">> ")

            if input == 'AUTO_MODE':
                if safetycheck:
                    print('\n  ###  SAFETY_CHECK for ' + self.__name + ' ###  ')
                    print('Is everything ready to start? (yes/no)')
                    safety_input = raw_input(">> ")
                    if safety_input == 'yes':
                        return True
                    else:
                        print ('Safetycheck: FAILED!')
                        print ('Switching to "MANUAL_MODE"\n')
                        print ('Enter your commands below.')
                        print ('Type "AUTO_MODE" for switching to AUTO_MODE')
                        print ('Type "status" for a status report')
                        print ('Type "exit" to leave manual mode')
                        print ('Type "exitall" to close the application')

            elif input == 'exit':
                break

            elif input == 'exitall':
                self.__oserial.close()
                exit()

            elif input == 'status':
                self.get_status()

            else:
                # send the character to the device
                # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
                self.__oserial.write(input + '\r\n')
                out = ''
                # let's wait 0.1 second before reading output (let's give device time to answer)
                time.sleep(0.1)
                while self.__oserial.inWaiting() > 0:
                    out += self.__oserial.read(1)

                if out != '':
                    print "<<" + out

"""

