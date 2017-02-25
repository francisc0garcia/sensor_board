import rospy
import math

from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

class ImuPublisher(object):
    degrees2rad = math.pi/180.0

    def __init__(self, topic_name = '/sensor/imu', frame_name='sensor_tf'):
        self.seq = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.yaw_deg = 0

        self.imu_msg = Imu()

        self.imu_msg.orientation_covariance[0] = -1
        self.imu_msg.angular_velocity_covariance[0] = -1
        self.imu_msg.linear_acceleration_covariance[0] = -1

        self.topic_name = topic_name
        self.frame_name = frame_name
        self.pub_imu = rospy.Publisher(self.topic_name, Imu, queue_size=1)

    def publish_info(self, sensor_imu_data):
        self.imu_msg.linear_acceleration.x = sensor_imu_data.imu_linear_acceleration_x
        self.imu_msg.linear_acceleration.y = sensor_imu_data.imu_linear_acceleration_y
        self.imu_msg.linear_acceleration.z = sensor_imu_data.imu_linear_acceleration_z

        self.imu_msg.angular_velocity.x = sensor_imu_data.imu_angular_velocity_x
        self.imu_msg.angular_velocity.y = sensor_imu_data.imu_angular_velocity_y
        self.imu_msg.angular_velocity.z = sensor_imu_data.imu_angular_velocity_z

        self.yaw_deg = sensor_imu_data.imu_euler_yaw
        self.roll = sensor_imu_data.imu_euler_roll
        self.pitch = sensor_imu_data.imu_euler_pitch

        self.yaw = self.yaw_deg * self.degrees2rad
        self.pitch = self.pitch * self.degrees2rad
        self.roll = self.roll * self.degrees2rad

        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        self.imu_msg.orientation.x = q[0]
        self.imu_msg.orientation.y = q[1]
        self.imu_msg.orientation.z = q[2]
        self.imu_msg.orientation.w = q[3]
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = self.frame_name
        self.imu_msg.header.seq = self.seq

        self.pub_imu.publish(self.imu_msg)

    def publish_imu(self, imu, seq):

        self.seq = seq
        [yaw, roll, pitch, lx, ly, lz, gx, gy, gz] = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        try:
            # qx, qy, qz, qw = self.bno.read_quaterion()
            # mx, my, mz = self.bno.read_magnetometer()
            #ax, ay, az = self.bno.read_accelerometer()
            #gvx, gvy, gvz = self.bno.read_gravity()
            yaw, roll, pitch = imu.bno.read_euler()
            #gx, gy, gz = imu.bno.read_gyroscope()
            #lx, ly, lz = imu.bno.read_linear_acceleration()
            # rospy.loginfo("read_gravity gvx: %d gvy: %d gvz: %d",  gvx, gvy, gvz)
        except:
            rospy.loginfo("Not possible to publish imu data port: " + str(imu.address))
            #
            imu.is_init_device = True
            imu.is_init_imu = False
            imu.is_init_calibration = False
            imu.init_imu()

            #time.sleep(1)

        if abs(yaw) > 0 or abs(roll) > 0 or abs(pitch) > 0:
            # IMU info
            imu.imu_linear_acceleration_x = lx
            imu.imu_linear_acceleration_y = ly
            imu.imu_linear_acceleration_z = lz
            imu.imu_angular_velocity_x = gx
            imu.imu_angular_velocity_y = gy
            imu.imu_angular_velocity_z = gz
            imu.imu_euler_yaw = yaw + imu.offset_yaw
            imu.imu_euler_roll = roll + imu.offset_roll
            imu.imu_euler_pitch = pitch + imu.offset_pitch
            # rospy.loginfo("roll %f pitch  %f  yaw  %f ", roll, pitch, yaw)

            # publish information over ROS
            self.publish_info(imu)