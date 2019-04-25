import simulation.vrepSim.vrep as vrep
import sys
import time

NUM_OF_MOTORS = 18
MOTORS_PREFIX = "ART_"
BIOLOID_IN_VREP = 'BIOLOID'
FLOOR_IN_VREP = '5mx5mWoodenFloor'

MODE = vrep.simx_opmode_oneshot_wait;
MODE_STREAMING = vrep.simx_opmode_streaming
MODE_BUFFER = vrep.simx_opmode_buffer


class Bioloid:

    def __init__(self):

        # here we establish a connection to Vrep
        self.clientID = self.connect()

        # self.MODE = vrep.simx_opmode_oneshot_wait;
        # self.MODE_STREAMING = vrep.simx_opmode_streaming
        # self.MODE_BUFFER=vrep.simx_opmode_buffer

        # this list will contain the Motors IDs with size = 18 i.e ID[3]= id for motor 4

        # get the Robot Objects Handles (Handlers)
        self.ID, self.robot, self.floor = self.get_handles()

        self.init_mode()
        pass

    #     # todo : this must be implemented as we want
    #     start_time=time.time()
    #     ret = vrep.simxCallScriptFunction(self.clientID,'G    yroSensor',vrep.sim_scripttype_childscript,'getGyroData',None,None,None,None,self.MODE)
    #
    #     errcode=ret[0]
    #     imu_data=ret[2]
    #
    #     #After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
    #     if ret == vrep.simx_return_ok:
    #
    #
    #     pass

    def get_handles(self):

        ID = [0] * NUM_OF_MOTORS
        # get Motors Handles
        for i in range(NUM_OF_MOTORS):
            motor_name = MOTORS_PREFIX + str(i + 1)
            errcode, ID[i] = vrep.simxGetObjectHandle(self.clientID, motor_name, MODE)
            # errcode= 0 means The function executed fine
            if errcode == 0:
                print("Motor (" + str(i + 1) + ") connected !")
            else:
                print("Warning :Motor (" + str(i + 1) + ") is not connected !")

        # Robot Handle
        robot = vrep.simxGetObjectHandle(self.clientID, BIOLOID_IN_VREP, MODE)

        if errcode == 0:
            print("Robot Connected !")
        else:
            print("Warning : Robot disconnected !")

        # floor handle

        floor = vrep.simxGetObjectHandle(self.clientID, FLOOR_IN_VREP, MODE)

        if errcode == 0:
            print("floor Connected !")
        else:
            print("Warning : floor disconnected !")

        # # IMU Handle TODO
        # self.IMU = self.get_IMU_handle()

        return ID, robot, floor

        # def get_IMU_handle(self):

    def read_imu_data(self):
        start_time = time.time()
        ret = vrep.simxCallScriptFunction(self.clientID, 'GyroSensor', vrep.sim_scripttype_childscript, 'getGyroData',
                                          None, None, None, None, MODE)

        while time.time() - start_time < 0.028:
            ret = vrep.simxCallScriptFunction(self.clientID, 'GyroSensor', vrep.sim_scripttype_childscript,
                                              'getGyroData', None, None, None, None, MODE)
            errcode = ret[0]
            imu_data = ret[2]

            # After initialization of streaming, it will take a few ms before the first value arrives, so check the
            # return code
            if errcode == vrep.simx_return_ok:
                print("Gyro data: [r p y] = " + str(imu_data))
                return imu_data

        pass

    def read_cm(self):
        errCode, pos = vrep.simxGetObjectPosition(self.clientID, self.robot, self.floor, MODE_STREAMING)
        print("CM Position : [ x , y , z ] = " + str(pos))
        return pos

    def set_degree(self, connection_time, pos):
        c = 0
        while (self.clientID != -1) & (c != connection_time):
            for i in range(0, NUM_OF_MOTORS):
                vrep.simxSetJointTargetPosition(self.clientID, self.ID[i], self.dynamexil2rad(pos[i]),
                                                MODE_STREAMING)

            c += 1
        # TODO play with this value
        time.sleep(0.3)
        pass

    # this function convert the angle in degree to Dynamexil Rang
    def rad2dynamixel(self, degree):
        dy = (degree + 2.62) / 0.00511711875
        return dy

    # this funcion convert from Motor Rang i.e [0 , 1023] to degree
    def dynamexil2rad(self, dy):
        degree = -2.62 + (dy * 0.00511711875)
        return degree

    # this method return the current value of robot motors / specific motor with an id
    # id is zero based indexing
    def read_motor(self, id=None):

        if id:
            return vrep.simxGetJointPosition(self.clientID, self.ID[id], MODE_BUFFER)

        motor_pos = [0.0] * NUM_OF_MOTORS

        for i in range(0, NUM_OF_MOTORS):
            errCode, motor_pos[i] = vrep.simxGetJointPosition(self.clientID, self.ID[i], MODE_BUFFER)
            print("Motor " + str(i + 1) + " = " + str(self.rad2dynamixel(motor_pos[i])))

        return motor_pos

    def init_mode(self):
        for i in range(0, NUM_OF_MOTORS):
            vrep.simxGetJointPosition(self.clientID, self.ID[i], MODE_STREAMING)
        pass

    def clear(self):
        # TODO
        pass

    @staticmethod
    def connect():
        vrep.simxFinish(-1)  # just in case, close all opened connections
        clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
        if clientID != -1:
            print("Connection succeeded!")
        else:
            print("Connection Failed!")
            sys.exit("Cannot connect!")

        return clientID

    @staticmethod
    def disconnect(self):
        # TODO
        pass


if __name__ == "__main__":
    Bioloid()
