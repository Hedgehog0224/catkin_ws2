#!/usr/bin/python3

from robot_pkg.msg import servodata
import rospy

rospy.init_node('manual')

predData = [0,0,0,0,0,0]
pub = rospy.Publisher('cameraData', servodata, queue_size=10)
msg = servodata()

def pubData():
    r = float(input("r:"))
    t = float(input("thet:"))
    f = float(input("fi:"))
    msg.servo0 = int(r*100)
    msg.servo1 = int(t*100)
    msg.servo2 = int(f*100)
    msg.servo3 = int(0*100)
    msg.servo4 = int(0*100)
    msg.servo5 = int(0*100)
    try:
        pub.publish(msg)
    except:
        print("ERROR")
    predData = [msg.servo0, msg.servo1, msg.servo2, msg.servo3, msg.servo4, msg.servo5]
    rospy.sleep(0.4)


while not rospy.is_shutdown():
    pubData()
