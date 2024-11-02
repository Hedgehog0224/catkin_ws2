#!/usr/bin/python3
from robot_pkg.msg import Servodata
import rospy

class manual():
    def __init__(self):
        rospy.init_node('manual')
        self.predData = [0,0,0,0,0,0]
        self.pub = rospy.Publisher('enterData', Servodata, queue_size=10)
        self.msg = Servodata()


    def pubData(self):
        self.r = float(input("r:"))
        self.t = float(input("thet:"))
        self.f = float(input("fi:"))
        self.msg.servo0 = int(r*100)
        self.msg.servo1 = int(t*100)
        self.msg.servo2 = int(f*100)
        self.msg.servo3 = int(0*100)
        self.msg.servo4 = int(0*100)
        self.msg.servo5 = int(0*100)
        try:
            self.pub.publish(msg)
        except:
            print("ERROR")
        self.predData = [self.msg.servo0, self.msg.servo1, self.msg.servo2, self.msg.servo3, self.msg.servo4, self.msg.servo5]
        rospy.sleep(0.4)

def main():
    Ob = manual()
    while not rospy.is_shutdown():
        Ob.pubData()

if __name__=="__main__":
    main()