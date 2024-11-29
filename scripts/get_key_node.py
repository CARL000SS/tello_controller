#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from getkey import getkey, keys


def key_publisher():
    rospy.init_node('key_publisher', anonymous=True)    
    pub = rospy.Publisher('/key_pressed', String, queue_size=10)
    
    rospy.loginfo("Press any key to publish it.")

    while not rospy.is_shutdown():
        try:
            key = getkey()
            if key == keys.T:
                command = 'Take off'
                pub.publish(command)

            elif key == keys.SPACE:
                command = 'Land'
                pub.publish(command)

            elif key == 'h':
                command = 'Stop'
                pub.publish(command)
                
            elif key == '1':
                command = 'Mission 1'
                pub.publish(command)

            elif key == '2':
                command = 'Mission 2'
                pub.publish(command)

            elif key == 'p':
                command = 'Emergency'
                pub.publish(command)

            elif key == 'o':
                command = 'orb'
                pub.publish(command)
            
            else:
                command = ''
                pub.publish('PD')

            # print(f"Publishing key: {key} ({command})")
            


        except Exception as e:
            rospy.logerr(f"Error con getKey: {e}")

if __name__ == '__main__':
    try:
        key_publisher()
    except rospy.ROSInterruptException:
        pass