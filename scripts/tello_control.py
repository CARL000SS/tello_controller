#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from tello_driver.msg import TelloStatus
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
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

        self.command = "Stop"
        self.mission_flag = False
        self.orb_flag = False
        self.offset_x = 0
        self.offset_y = 0

        srv = Server(dynamic_gainsConfig, self.dynamic_params_callback)

        self.sub_status = rospy.Subscriber("/tello/status", TelloStatus, self.status_callback)
        self.sub_key = rospy.Subscriber("/key_pressed", String, self.key_callback)
        self.sub_imu = rospy.Subscriber("/tello/imu", Imu, self.imu_callback)
        self.sub_orb = rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.orb_callback)

        self.pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/tello/land', Empty, queue_size=10)
        self.pub_emergency = rospy.Publisher('/tello/emergency', Empty, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.pub_var = rospy.Publisher('/system_vars', system_vars_msg, queue_size=10)

        self.vel_msg = Twist()
        self.empty_msg = Empty()
        self.system_vars_msg = system_vars_msg()
        
        self.rate = rospy.Rate(Hz)

        # X variables
        self.U_x = 0.0
        self.x = 0.0
        self.x_d = 0.0
        self.kp_x = 0.5
        self.kd_x = 0.05
        self.pd_x = PD(dt)

        # Y variables
        self.U_y = 0.0
        self.y = 0.0
        self.y_d = 0.0
        self.kp_y = 0.5
        self.kd_y = 0.05
        self.pd_y = PD(dt)
        
        # Z variables
        self.U_z = 0.0
        self.z = 0.0
        self.z_d = 1.0
        self.kp_z = 0.5
        self.kd_z = 0.05
        self.pd_z = PD(dt)

        # YAW variables
        self.U_yaw = 0.0
        self.yaw = 0.0
        self.yaw_d = 0.0
        self.kp_yaw = 0.5
        self.kd_yaw = 0.05
        self.pd_yaw = PD(dt)


        self.pitch = 0.0
        self.roll = 0.0
        self.battery = 0
        self.is_flying = False
        self.is_on_ground = False
        self.down_visual_state = False
        self.is_em_open = False # is_em_open True in flight, False when landed
        self.is_drone_hover = False
        self.camera_state = 0

        self.k = 0 # general discrete time index
        self.kaux=0 # auxiliar discrete time index, for console logs

        self.loop()

    def status_callback(self, msg):
        self.z = msg.height_m
        self.battery = msg.battery_percentage
        self.is_flying = msg.is_flying
        self.is_on_ground = msg.is_on_ground
        self.down_visual_state = msg.down_visual_state
        self.is_em_open = msg.is_em_open
        self.is_drone_hover = msg.is_drone_hover
        self.camera_state = msg.camera_state

    def key_callback(self, msg):
        self.command = msg.data

        if self.command == 'orb' and not(self.orb_flag): # when command, the drone will set its current position as the zero x & y coordenate
            self.orb_flag = True
            self.offset_x = self.x
            self.offset_y = self.y
            self.command = "PD"

    def imu_callback(self, msg):
        orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

    def orb_callback(self, msg):
        self.x = msg.pose.position.x - self.offset_x
        self.y = msg.pose.position.y - self.offset_y

    def dynamic_params_callback(self, config, level):
        if not(self.mission_flag): # Drone is not in mission path
            self.x_d = config.x_d
            self.y_d = config.y_d
            self.z_d = config.z_d
            self.yaw_d = config.yaw_d * TO_RAD
        
        self.kp_x = config.kp_x
        self.kd_x = config.kd_x
        self.kp_y = config.kp_y
        self.kd_y = config.kd_y
        self.kp_z = config.kp_z
        self.kd_z = config.kd_z
        self.kp_yaw = config.kp_yaw
        self.kd_yaw = config.kd_yaw

        return config

    def loginfo(self):
        if(self.kaux==10):
            self.kaux = 0
            print("""
            KEYBOARD CONTROL DRON TELLO WITH ROS
            
                TAKEOFF         - t
                LAND            - space bar
                ENERGENCY       - p
                HOVER           - h
                ORB Slam        - o
                MISSION 1       - 1
                MISSION 2       - 2
                PD              - any other key (set references with sliders)
            """)
            # print(f"is flying? = {self.is_flying}")
            # print(f"is on ground? = {self.is_on_ground}")
            # print(f"is em open? = {self.is_em_open}")
            # print(f"down visual state = {self.down_visual_state}")
            # print(f"camera state = {self.camera_state}")
            print(f"-------------------------------------------------------------------------------")
            print(f"x_d = {self.x_d:.2f} \t x = {self.x*10:.3f} \t kp_x = {self.kp_x:.2f} \t kd_x = {self.kd_x:.2f} \t U_x = {self.U_x:.2f}")
            print(f"-------------------------------------------------------------------------------")
            print(f"y_d = {self.y_d:.2f} \t y = {self.y*10:.3f} \t kp_y = {self.kp_y:.2f} \t kd_y = {self.kd_y:.2f} \t U_y = {self.U_y:.2f}")
            print(f"-------------------------------------------------------------------------------")
            print(f"z_d = {self.z_d:.2f} \t z = {self.z:.2f} \t kp_z = {self.kp_z:.2f} \t kd_z = {self.kd_z:.2f} \t U_z = {self.U_z:.2f}")
            print(f"-------------------------------------------------------------------------------")
            print(f"yaw_d = {int(self.yaw_d*TO_DEG)} \t yaw = {int(self.yaw*TO_DEG)} \t kp_yaw = {self.kp_yaw:.2f} \t kd_yaw = {self.kd_yaw:.2f} \t U_yaw = {self.U_yaw:.2f}")
            print(f"-------------------------------------------------------------------------------")
            print(f"\nt = {int(self.k*dt)}\t\tBattery: {(self.battery)}\t\tFlying: {(self.is_em_open)}\t\tORB: {(self.orb_flag)}\n")
            print(f"-------------------------------------------------------------------------------")
            rospy.logwarn(f"EXECUTING: {self.command}")
            print(f"-------------------------------------------------------------------------------\n\n")
        self.kaux = self.kaux + 1

        self.system_vars_msg.x_d = self.x_d
        self.system_vars_msg.x = self.x*10
        self.system_vars_msg.y_d = self.y_d
        self.system_vars_msg.y = self.y*10
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
                    self.command = "PD"

                elif self.command == "Land":
                    rospy.logwarn("Landing ...\n")
                    self.pub_land.publish(self.empty_msg)
                    self.command = "Stop"

                elif self.command == "Emergency":
                    rospy.logwarn("Emergency ...\n")
                    self.pub_emergency.publish(self.empty_msg)
                    self.command = "Stop"

                elif not(self.is_em_open) or self.command == "Stop": # if landed ...
                    self.U_x = 0.0
                    self.U_y = 0.0
                    self.U_z = 0.0
                    self.U_yaw = 0.0
                    self.command = "Stop"

                else: # if flying ...

                    if self.command == 'Mission 1': # circle
                        self.mission_flag = True
                        self.x_d = 0.7 * np.cos(0.4*self.k*dt)
                        self.y_d = 0.7 * np.sin(0.4*self.k*dt)
                        # self.z_d = 1 + 1 * np.sin(0.2*self.k*dt)
                        self.z_d = 2.25

                    elif self.command == 'Mission 2': # bouncing
                        self.mission_flag = True
                        self.z_d = 1 + 1.5 * np.sin(0.3*self.k*dt)

                    elif self.command == "PD":
                        self.mission_flag = False


                    if self.orb_flag == True: #  If X and Y coordenates are already set with ORB Slam
                        self.pd_x.set_gains(self.kp_x, self.kd_x)
                        self.pd_y.set_gains(self.kp_y, self.kd_y)
                        self.U_x = self.pd_x.control(self.x_d, self.x*10) # Scale factor
                        self.U_y = self.pd_y.control(self.y_d, self.y*10)
                    else:
                        self.U_x = 0.0
                        self.U_y = 0.0
                        

                    self.pd_z.set_gains(self.kp_z, self.kd_z)
                    self.pd_yaw.set_gains(self.kp_yaw, self.kd_yaw)                    
                    self.U_z = self.pd_z.control(self.z_d, self.z)
                    self.U_yaw = self.pd_yaw.control(self.yaw_d, self.yaw)

                
                self.vel_msg.linear.x = -self.U_y
                self.vel_msg.linear.y = self.U_x
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
