import smbus
import struct
import time
import rospy
import pigpio
from classes.BNO055 import *
from classes.imu_publisher import *

class ImuData(object):
    def __init__(self, address=BNO055_ADDRESS_A):
        self.address = address
        self.is_init_imu = False
        self.is_init_device = False
        self.is_init_calibration = False

        self.offset_yaw = 0.0
        self.offset_roll = 0.0
        self.offset_pitch = 0.0

        #self.bno = BNO055(serial_port=serial_port, rst=self.reset_imu)
        self.bno = BNO055(address=self.address)

        # IMU info
        self.imu_linear_acceleration_x = 0
        self.imu_linear_acceleration_y = 0
        self.imu_linear_acceleration_z = 0

        self.imu_angular_velocity_x = 0
        self.imu_angular_velocity_y = 0
        self.imu_angular_velocity_z = 0

        self.imu_euler_yaw = 0
        self.imu_euler_roll = 0
        self.imu_euler_pitch = 0

        self.imu_temperature = 0

    def init_imu(self):
        while not self.is_init_imu:
            try:
                self.bno.begin()
                self.is_init_imu = True
                rospy.loginfo('Connected to BNO055 at port: ' + str(self.address) )
            except:
                rospy.loginfo('Failed to initialize BNO055 at port: ' + str(self.address) )
                time.sleep(0.1)

        while not self.is_init_device :
            status, self_test, error = self.bno.get_system_status(False)
            if error == 0:
                self.is_init_device = True
            else:
                rospy.loginfo('Failed to initialize get_system_status error at port: ' + str(self.address))
                time.sleep(0.1)

        if not self.is_init_calibration:
            # Load precomputed calibration values
            self.load_calibration()

        while not self.is_init_calibration:
            cal_sys, cal_gyro, cal_accel, cal_mag = self.bno.get_calibration_status()

            if cal_gyro > 0 and cal_accel > 0:
                self.is_init_calibration = True
            else:
                self.load_calibration()
                #time.sleep(0.1)
                #rospy.loginfo("Waiting for IMU calibration: [S %f, G %f, A %f, M %f]" % (cal_sys, cal_gyro, cal_accel, cal_mag))
                time.sleep(0.1)

    def load_calibration(self):
        # computed using tutorial:
        # https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/webgl-example
        data = [253, 255, 5, 0, 166, 0, 205, 246, 93, 252, 95, 1, 254, 255, 255, 255, 1, 0, 232, 3, 163, 1]
        self.bno.set_calibration(data)
        return 'OK'
