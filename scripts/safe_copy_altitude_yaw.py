#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from tello_driver.msg import TelloStatus
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

from tello_controller.msg import system_vars_msg

from dynamic_reconfigure.server import Server
from tello_controller.cfg import dynamic_gainsConfig

from PD import PD

TO_DEG = 57.2957795131
TO_RAD = 0.0174532925

Hz = 10 # loop rate in Hz
dt = 1/float(Hz) # loop rate period

class TelloControl:
    def __init__(self):

        rospy.init_node('tello_controller', anonymous=False)
        rospy.loginfo("Node Initialized")

        srv = Server(dynamic_gainsConfig, self.dynamic_params_callback)

        self.sub_status = rospy.Subscriber("/tello/status", TelloStatus, self.status_callback)
        self.sub_key = rospy.Subscriber("/key_pressed", String, self.key_callback)
        self.sub_imu = rospy.Subscriber("/tello/imu", Imu, self.imu_callback)

        self.pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/tello/land', Empty, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.pub_var = rospy.Publisher('/system_vars', system_vars_msg, queue_size=10)

        self.vel_msg = Twist()
        self.empty_msg = Empty()
        self.system_vars_msg = system_vars_msg()
        
        self.rate = rospy.Rate(Hz)

        # Z variables
        self.U_z = 0
        self.z = 0
        self.z_d = 1.0
        self.kp_z = 0.5
        self.kd_z = 0.05
        self.pd_z = PD(dt)

        # YAW variables
        self.U_yaw = 0
        self.yaw = 0
        self.yaw_d = 0
        self.kp_yaw = 0.5
        self.kd_yaw = 0.05
        self.pd_yaw = PD(dt)


        self.pitch = 0
        self.roll = 0
        self.t = 0
        self.is_flying = False
        self.is_on_ground = False
        self.down_visual_state = False
        self.is_em_open = False # is_em_open True in flight, False when landed
        self.is_drone_hover = False
        self.camera_state = 0

        self.command = ""

        self.k = 0 # general discrete time index
        self.kaux=0 # auxiliar discrete time index, for console logs

        self.loop()

    def status_callback(self, msg):
        self.z = msg.height_m
        self.t = msg.flight_time_sec
        self.is_flying = msg.is_flying
        self.is_on_ground = msg.is_on_ground
        self.down_visual_state = msg.down_visual_state
        self.is_em_open = msg.is_em_open
        self.is_drone_hover = msg.is_drone_hover
        self.camera_state = msg.camera_state

    def key_callback(self, msg):
        self.command = msg.data

    def imu_callback(self, msg):
        orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

    def dynamic_params_callback(self, config, level):
        # Z parameters
        self.z_d = config.z_d
        self.kp_z = config.kp_z
        self.kd_z = config.kd_z

        # YAW parameters
        self.yaw_d = config.yaw_d * TO_RAD
        self.kp_yaw = config.kp_yaw
        self.kd_yaw = config.kd_yaw

        return config

    def loginfo(self):
        if(self.kaux==10):
            self.kaux = 0
            print(f"is flying? = {self.is_flying}")
            print(f"is on ground? = {self.is_on_ground}")
            print(f"is em open? = {self.is_em_open}")
            print(f"down visual state = {self.down_visual_state}")
            print(f"camera state = {self.camera_state}")
            print(f"---------------------------------")
            print(f"z_d = {self.z_d:.2f} \t z = {self.z:.2f} \t kp_z = {self.kp_z:.2f} \t kd_z = {self.kd_z:.2f} \t U_z = {self.U_z:.2f}")
            print(f"---------------------------------")
            print(f"yaw_d = {self.yaw_d*TO_DEG:.2f} \t yaw = {self.yaw*TO_DEG:.2f} \t kp_yaw = {self.kp_yaw:.2f} \t kd_yaw = {self.kd_yaw:.2f} \t U_yaw = {self.U_yaw:.2f}")
            print(f"---------------------------------")
            # print(f"yaw = {self.yaw:.2f} \t pitch = {self.pitch:.2f} \t roll = {self.roll:.2f}")
            # print(f"---------------------------------")
            print(f"t = {(self.k*dt):.2f}")
            print(f"---------------------------------\n\n")
        self.kaux = self.kaux + 1

        self.system_vars_msg.z_d = self.z_d
        self.system_vars_msg.z = self.z
        self.system_vars_msg.yaw_d = self.yaw_d*TO_DEG
        self.system_vars_msg.yaw = self.yaw*TO_DEG
        self.pub_var.publish(self.system_vars_msg)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                if self.command == "Take off":
                    rospy.logwarn("Taking off ...\n")
                    self.pub_takeoff.publish(self.empty_msg)
                    self.command = ""

                elif self.command == "Land":
                    rospy.logwarn("Landing ...\n")
                    self.pub_land.publish(self.empty_msg)
                    self.command = ""
                
                elif self.command == "w":
                    self.z_d = 2.0
                    rospy.logwarn("Reference altitude set to 2m\n")
                    self.command = ""
                    
                elif self.command == "s":
                    rospy.logwarn("Reference altitude set to 1m\n")
                    self.z_d = 1.0
                    self.command = ""
                
                if self.is_em_open:
                    self.pd_z.set_gains(self.kp_z, self.kd_z)
                    self.U_z = self.pd_z.control(self.z_d, self.z)
                    self.pd_yaw.set_gains(self.kp_yaw, self.kd_yaw)
                    self.U_yaw = self.pd_yaw.control(self.yaw_d, self.yaw)
                else:
                    self.U_z = 0.0
                    self.U_yaw = 0.0
                
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0 
                self.vel_msg.linear.z = self.U_z
                self.vel_msg.angular.z = self.U_yaw
                self.pub_cmd_vel.publish(self.vel_msg)
                
                self.loginfo()
                self.k = self.k + 1

            except Exception as e:
                rospy.logerr(f"Error dentro del bucle de control: {e}")

            self.rate.sleep()


def main():
    try:
        nodo = TelloControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
